function Foundation() {}

Foundation.prototype.Schema =
	"<empty/>";

Foundation.prototype.Init = function()
{
	// Foundations are initially 'uncommitted' and do not block unit movement at all
	// (to prevent players exploiting free foundations to confuse enemy units).
	// The first builder to reach the uncommitted foundation will tell friendly units
	// and animals to move out of the way, then will commit the foundation and enable
	// its obstruction once there's nothing in the way.
	this.committed = false;

	this.builders = new Map(); // Map of builder entities to their work per second
	this.totalBuilderRate = 0; // Total amount of work the builders do each second
	this.buildMultiplier = 1; // Multiplier for the amount of work builders do
	this.buildTimePenalty = 0.7; // Penalty for having multiple builders

	this.previewEntity = INVALID_ENTITY;
};

Foundation.prototype.Serialize = function()
{
	let ret = Object.assign({}, this);
	ret.previewEntity = INVALID_ENTITY;
	return ret;
};

Foundation.prototype.Deserialize = function(data)
{
	this.Init();
	Object.assign(this, data);
};

Foundation.prototype.OnDeserialized = function()
{
	this.CreateConstructionPreview();
};

Foundation.prototype.InitialiseConstruction = function(owner, template)
{
	this.finalTemplateName = template;

	// We need to know the owner in OnDestroy, but at that point the entity has already been
	// decoupled from its owner, so we need to remember it in here (and assume it won't change)
	this.owner = owner;

	// Remember the cost here, so if it changes after construction begins (from auras or technologies)
	// we will use the correct values to refund partial construction costs
	let cmpCost = Engine.QueryInterface(this.entity, IID_Cost);
	if (!cmpCost)
		error("A foundation must have a cost component to know the build time");

	this.costs = cmpCost.GetResourceCosts(owner);

	this.maxProgress = 0;

	this.initialised = true;
};

/**
 * Moving the revelation logic from Build to here makes the building sink if
 * it is attacked.
 */
Foundation.prototype.OnHealthChanged = function(msg)
{
	// Gradually reveal the final building preview
	let cmpPosition = Engine.QueryInterface(this.previewEntity, IID_Position);
	if (cmpPosition)
		cmpPosition.SetConstructionProgress(this.GetBuildProgress());

	Engine.PostMessage(this.entity, MT_FoundationProgressChanged, { "to": this.GetBuildPercentage() });
};

/**
 * Returns the current build progress in a [0,1] range.
 */
Foundation.prototype.GetBuildProgress = function()
{
	var cmpHealth = Engine.QueryInterface(this.entity, IID_Health);
	if (!cmpHealth)
		return 0;

	var hitpoints = cmpHealth.GetHitpoints();
	var maxHitpoints = cmpHealth.GetMaxHitpoints();

	return hitpoints / maxHitpoints;
};

Foundation.prototype.GetBuildPercentage = function()
{
	return Math.floor(this.GetBuildProgress() * 100);
};

/**
 * Returns the current builders.
 *
 * @return {number[]} - An array containing the entity IDs of assigned builders.
 */
Foundation.prototype.GetBuilders = function()
{
	return Array.from(this.builders.keys());
};

Foundation.prototype.GetNumBuilders = function()
{
	return this.builders.size;
};

Foundation.prototype.IsFinished = function()
{
	return (this.GetBuildProgress() == 1.0);
};

Foundation.prototype.OnOwnershipChanged = function(msg)
{
	if (msg.to != INVALID_PLAYER && this.previewEntity != INVALID_ENTITY)
	{
		let cmpPreviewOwnership = Engine.QueryInterface(this.previewEntity, IID_Ownership);
		if (cmpPreviewOwnership)
			cmpPreviewOwnership.SetOwner(msg.to);
		return;
	}

	if (msg.to != INVALID_PLAYER)
		return;

	// Refund a portion of the construction cost, proportional to the amount of build progress remaining

	if (!this.initialised) // this happens if the foundation was destroyed because the player had insufficient resources
		return;

	if (this.previewEntity != INVALID_ENTITY)
	{
		Engine.DestroyEntity(this.previewEntity);
		this.previewEntity = INVALID_ENTITY;
	}

	if (this.IsFinished())
		return;

	let cmpPlayer = QueryPlayerIDInterface(this.owner);

	for (var r in this.costs)
	{
		var scaled = Math.ceil(this.costs[r] * (1.0 - this.maxProgress));
		if (scaled)
		{
			cmpPlayer.AddResource(r, scaled);
			var cmpStatisticsTracker = QueryPlayerIDInterface(this.owner, IID_StatisticsTracker);
			if (cmpStatisticsTracker)
				cmpStatisticsTracker.IncreaseResourceUsedCounter(r, -scaled);
		}
	}
};

/**
 * Adds an array of builders.
 *
 * @param {number[]} builders - An array containing the entity IDs of builders to assign.
 */
Foundation.prototype.AddBuilders = function(builders)
{
	let changed = false;
	for (let builder of builders)
		changed = this.AddBuilderHelper(builder) || changed;

	if (changed)
		this.HandleBuildersChanged();
};

/**
 * Adds a single builder to this entity.
 *
 * @param {number} builderEnt - The entity to add.
 * @return {boolean} - Whether the addition was successful.
 */
Foundation.prototype.AddBuilderHelper = function(builderEnt)
{
	if (this.builders.has(builderEnt))
		return false;

	let cmpBuilder = Engine.QueryInterface(builderEnt, IID_Builder) ||
		Engine.QueryInterface(this.entity, IID_AutoBuildable);
	if (!cmpBuilder)
		return false;

	let buildRate = cmpBuilder.GetRate();
	this.builders.set(builderEnt, buildRate);
	this.totalBuilderRate += buildRate;

	return true;
};

/**
 * Adds a builder to the counter.
 *
 * @param {number} builderEnt - The entity to add.
 */
Foundation.prototype.AddBuilder = function(builderEnt)
{
	if (this.AddBuilderHelper(builderEnt))
		this.HandleBuildersChanged();
};

Foundation.prototype.RemoveBuilder = function(builderEnt)
{
	if (!this.builders.has(builderEnt))
		return;

	this.totalBuilderRate -= this.builders.get(builderEnt);
	this.builders.delete(builderEnt);
	this.HandleBuildersChanged();
};

/**
 * This has to be called whenever the number of builders change.
 */
Foundation.prototype.HandleBuildersChanged = function()
{
	this.SetBuildMultiplier();

	let cmpVisual = Engine.QueryInterface(this.entity, IID_Visual);
	if (cmpVisual)
		cmpVisual.SetVariable("numbuilders", this.builders.size);

	Engine.PostMessage(this.entity, MT_FoundationBuildersChanged, { "to": this.GetBuilders() });
};

/**
 * The build multiplier is a penalty that is applied to each builder.
 * For example, ten women build at a combined rate of 10^0.7 = 5.01 instead of 10.
 */
Foundation.prototype.CalculateBuildMultiplier = function(num)
{
	// Avoid division by zero, in particular 0/0 = NaN which isn't reliably serialized
	return num < 2 ? 1 : Math.pow(num, this.buildTimePenalty) / num;
};

Foundation.prototype.SetBuildMultiplier = function()
{
	this.buildMultiplier = this.CalculateBuildMultiplier(this.GetNumBuilders());
};

Foundation.prototype.GetBuildTime = function()
{
	let timeLeft = (1 - this.GetBuildProgress()) * Engine.QueryInterface(this.entity, IID_Cost).GetBuildTime();
	let rate = this.totalBuilderRate * this.buildMultiplier;
	// The rate if we add another woman to the foundation.
	let rateNew = (this.totalBuilderRate + 1) * this.CalculateBuildMultiplier(this.GetNumBuilders() + 1);
	return {
		// Avoid division by zero, in particular 0/0 = NaN which isn't reliably serialized
		"timeRemaining": rate ? timeLeft / rate : 0,
		"timeRemainingNew": timeLeft / rateNew
	};
};

/**
 * Perform some number of seconds of construction work.
 * Returns true if the construction is completed.
 */
Foundation.prototype.Build = function(builderEnt, work)
{
	// Do nothing if we've already finished building
	// (The entity will be destroyed soon after completion so
	// this won't happen much)
	if (this.GetBuildProgress() == 1.0)
		return;

	var cmpObstruction = Engine.QueryInterface(this.entity, IID_Obstruction);
	// If there are any units in the way, ask them to move away and return early from this method.
	if (cmpObstruction && cmpObstruction.GetBlockMovementFlag())
	{
		// Remove animal corpses
		for (let ent of cmpObstruction.GetEntitiesDeletedUponConstruction())
			Engine.DestroyEntity(ent);

		let collisions = cmpObstruction.GetEntitiesBlockingConstruction();
		if (collisions.length)
		{
			for (var ent of collisions)
			{
				var cmpUnitAI = Engine.QueryInterface(ent, IID_UnitAI);
				if (cmpUnitAI)
					cmpUnitAI.LeaveFoundation(this.entity);

				// TODO: What if an obstruction has no UnitAI?
			}

			// TODO: maybe we should tell the builder to use a special
			// animation to indicate they're waiting for people to get
			// out the way

			return;
		}
	}

	// Handle the initial 'committing' of the foundation
	if (!this.committed)
	{
		// The obstruction always blocks new foundations/construction,
		// but we've temporarily allowed units to walk all over it
		// (via CCmpTemplateManager). Now we need to remove that temporary
		// blocker-disabling, so that we'll perform standard unit blocking instead.
		if (cmpObstruction)
			cmpObstruction.SetDisableBlockMovementPathfinding(false, false, -1);

		// Call the related trigger event
		var cmpTrigger = Engine.QueryInterface(SYSTEM_ENTITY, IID_Trigger);
		cmpTrigger.CallEvent("ConstructionStarted", {
			"foundation": this.entity,
			"template": this.finalTemplateName
		});

		// Switch foundation to scaffold variant
		var cmpFoundationVisual = Engine.QueryInterface(this.entity, IID_Visual);
		if (cmpFoundationVisual)
			cmpFoundationVisual.SelectAnimation("scaffold", false, 1.0);

		this.committed = true;
		this.CreateConstructionPreview();
	}

	// Add an appropriate proportion of hitpoints
	var cmpHealth = Engine.QueryInterface(this.entity, IID_Health);
	if (!cmpHealth)
	{
		error("Foundation " + this.entity + " does not have a health component.");
		return;
	}
	var deltaHP = work * this.GetBuildRate() * this.buildMultiplier;
	if (deltaHP > 0)
		cmpHealth.Increase(deltaHP);

	// Update the total builder rate
	this.totalBuilderRate += work - this.builders.get(builderEnt);
	this.builders.set(builderEnt, work);

	var progress = this.GetBuildProgress();

	// Remember our max progress for partial refund in case of destruction
	this.maxProgress = Math.max(this.maxProgress, progress);

	if (progress >= 1.0)
	{
		// Finished construction

		// Create the real entity
		var building = Engine.AddEntity(this.finalTemplateName);

		// Copy various parameters from the foundation

		var cmpVisual = Engine.QueryInterface(this.entity, IID_Visual);
		var cmpBuildingVisual = Engine.QueryInterface(building, IID_Visual);
		if (cmpVisual && cmpBuildingVisual)
			cmpBuildingVisual.SetActorSeed(cmpVisual.GetActorSeed());

		var cmpPosition = Engine.QueryInterface(this.entity, IID_Position);
		if (!cmpPosition || !cmpPosition.IsInWorld())
		{
			error("Foundation " + this.entity + " does not have a position in-world.");
			Engine.DestroyEntity(building);
			return;
		}
		var cmpBuildingPosition = Engine.QueryInterface(building, IID_Position);
		if (!cmpBuildingPosition)
		{
			error("New building " + building + " has no position component.");
			Engine.DestroyEntity(building);
			return;
		}
		var pos = cmpPosition.GetPosition2D();
		cmpBuildingPosition.JumpTo(pos.x, pos.y);
		var rot = cmpPosition.GetRotation();
		cmpBuildingPosition.SetYRotation(rot.y);
		cmpBuildingPosition.SetXZRotation(rot.x, rot.z);
		// TODO: should add a ICmpPosition::CopyFrom() instead of all this

		var cmpRallyPoint = Engine.QueryInterface(this.entity, IID_RallyPoint);
		var cmpBuildingRallyPoint = Engine.QueryInterface(building, IID_RallyPoint);
		if(cmpRallyPoint && cmpBuildingRallyPoint)
		{
			var rallyCoords = cmpRallyPoint.GetPositions();
			var rallyData = cmpRallyPoint.GetData();
			for (var i = 0; i < rallyCoords.length; ++i)
			{
				cmpBuildingRallyPoint.AddPosition(rallyCoords[i].x, rallyCoords[i].z);
				cmpBuildingRallyPoint.AddData(rallyData[i]);
			}
		}

		// ----------------------------------------------------------------------

		var owner;
		var cmpTerritoryDecay = Engine.QueryInterface(building, IID_TerritoryDecay);
		if (cmpTerritoryDecay && cmpTerritoryDecay.HasTerritoryOwnership())
		{
			let cmpTerritoryManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_TerritoryManager);
			owner = cmpTerritoryManager.GetOwner(pos.x, pos.y);
		}
		else
		{
			let cmpOwnership = Engine.QueryInterface(this.entity, IID_Ownership);
			if (!cmpOwnership)
			{
				error("Foundation " + this.entity + " has no ownership.");
				Engine.DestroyEntity(building);
				return;
			}
			owner = cmpOwnership.GetOwner();
		}
		var cmpBuildingOwnership = Engine.QueryInterface(building, IID_Ownership);
		if (!cmpBuildingOwnership)
		{
			error("New Building " + building + " has no ownership.");
			Engine.DestroyEntity(building);
			return;
		}
		cmpBuildingOwnership.SetOwner(owner);

		/*
		Copy over the obstruction control group IDs from the foundation
		entities. This is needed to ensure that when a foundation is completed
		and replaced by a new entity, it remains in the same control group(s)
		as any other foundation entities that may surround it. This is the
		mechanism that is used to e.g. enable wall pieces to be built closely
		together, ignoring their mutual obstruction shapes (since they would
		otherwise be prevented from being built so closely together). If the
		control groups are not copied over, the new entity will default to a
		new control group containing only itself, and will hence block
		construction of any surrounding foundations that it was previously in
		the same control group with.

		Note that this will result in the completed building entities having
		control group IDs that equal entity IDs of old (and soon to be deleted)
		foundation entities. This should not have any consequences, however,
		since the control group IDs are only meant to be unique identifiers,
		which is still true when reusing the old ones.
		*/

		var cmpBuildingObstruction = Engine.QueryInterface(building, IID_Obstruction);
		if (cmpObstruction && cmpBuildingObstruction)
		{
			cmpBuildingObstruction.SetControlGroup(cmpObstruction.GetControlGroup());
			cmpBuildingObstruction.SetControlGroup2(cmpObstruction.GetControlGroup2());
		}

		var cmpPlayerStatisticsTracker = QueryOwnerInterface(this.entity, IID_StatisticsTracker);
		if (cmpPlayerStatisticsTracker)
			cmpPlayerStatisticsTracker.IncreaseConstructedBuildingsCounter(building);

		var cmpBuildingHealth = Engine.QueryInterface(building, IID_Health);
		if (cmpBuildingHealth)
			cmpBuildingHealth.SetHitpoints(progress * cmpBuildingHealth.GetMaxHitpoints());

		PlaySound("constructed", building);

		Engine.PostMessage(this.entity, MT_ConstructionFinished,
			{ "entity": this.entity, "newentity": building });
		Engine.PostMessage(this.entity, MT_EntityRenamed, { "entity": this.entity, "newentity": building });

		// Inform the builders that repairing has finished.
		// This not done by listening to a global message due to performance.
		for (let builder of this.GetBuilders())
		{
			let cmpUnitAIBuilder = Engine.QueryInterface(builder, IID_UnitAI);
			if (cmpUnitAIBuilder)
				cmpUnitAIBuilder.ConstructionFinished({ "entity": this.entity, "newentity": building });
		}

		Engine.DestroyEntity(this.entity);
	}
};

Foundation.prototype.GetBuildRate = function()
{
	let cmpHealth = Engine.QueryInterface(this.entity, IID_Health);
	let cmpCost = Engine.QueryInterface(this.entity, IID_Cost);
	// Return infinity for instant structure conversion
	return cmpHealth.GetMaxHitpoints() / cmpCost.GetBuildTime();
};

/**
 * Create preview entity and copy various parameters from the foundation.
 */
Foundation.prototype.CreateConstructionPreview = function()
{
	if (this.previewEntity)
	{
		Engine.DestroyEntity(this.previewEntity);
		this.previewEntity = INVALID_ENTITY;
	}

	if (!this.committed)
		return;

	let cmpFoundationVisual = Engine.QueryInterface(this.entity, IID_Visual);
	if (!cmpFoundationVisual || !cmpFoundationVisual.HasConstructionPreview())
		return;

	this.previewEntity = Engine.AddLocalEntity("construction|"+this.finalTemplateName);
	let cmpFoundationOwnership = Engine.QueryInterface(this.entity, IID_Ownership);
	let cmpPreviewOwnership = Engine.QueryInterface(this.previewEntity, IID_Ownership);
	if (cmpFoundationOwnership && cmpPreviewOwnership)
		cmpPreviewOwnership.SetOwner(cmpFoundationOwnership.GetOwner());

	// TODO: the 'preview' would be invisible if it doesn't have the below component,
	// Maybe it makes more sense to simply delete it then?

	// Initially hide the preview underground
	let cmpPreviewPosition = Engine.QueryInterface(this.previewEntity, IID_Position);
	let cmpFoundationPosition = Engine.QueryInterface(this.entity, IID_Position);
	if (cmpPreviewPosition && cmpFoundationPosition)
	{
		let rot = cmpFoundationPosition.GetRotation();
		cmpPreviewPosition.SetYRotation(rot.y);
		cmpPreviewPosition.SetXZRotation(rot.x, rot.z);

		let pos = cmpFoundationPosition.GetPosition2D();
		cmpPreviewPosition.JumpTo(pos.x, pos.y);

		cmpPreviewPosition.SetConstructionProgress(this.GetBuildProgress());
	}

	let cmpPreviewVisual = Engine.QueryInterface(this.previewEntity, IID_Visual);
	if (cmpPreviewVisual && cmpFoundationVisual)
	{
		cmpPreviewVisual.SetActorSeed(cmpFoundationVisual.GetActorSeed());
		cmpPreviewVisual.SelectAnimation("scaffold", false, 1.0);
	}
};

function FoundationMirage() {}
FoundationMirage.prototype.Init = function(cmpFoundation)
{
	this.numBuilders = cmpFoundation.GetNumBuilders();
	this.buildTime = cmpFoundation.GetBuildTime();
};

FoundationMirage.prototype.GetNumBuilders = function() { return this.numBuilders; };
FoundationMirage.prototype.GetBuildTime = function() { return this.buildTime; };

Engine.RegisterGlobal("FoundationMirage", FoundationMirage);

Foundation.prototype.Mirage = function()
{
	let mirage = new FoundationMirage();
	mirage.Init(this);
	return mirage;
};

Engine.RegisterComponentType(IID_Foundation, "Foundation", Foundation);
