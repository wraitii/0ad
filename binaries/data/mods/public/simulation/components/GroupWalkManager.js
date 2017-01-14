function GroupWalkManager() {}

GroupWalkManager.prototype.Schema =
	"<a:component type='system'/><empty/>";

// This is a simple system component that keeps track of existing "groups"
// when a "walk together" order is issued as well as necessary information.

GroupWalkManager.prototype.Init = function()
{
	this.groups = new Map();
	this.nextGroupID = 0;
};

GroupWalkManager.prototype.Serialize = function()
{
};

GroupWalkManager.prototype.CreateGroup = function(formableEntsID, x, z, range, formationTemplate)
{
	let speed = 50.0; // TODO: put formation max speed?
	for (let ent of formableEntsID)
	{
		let cmpUnitMotion = Engine.QueryInterface(ent, IID_UnitMotion);
		let sp = cmpUnitMotion.GetBaseSpeed();
		if (sp < speed)
			speed = sp;
	}
	let group = {
		"entities" : formableEntsID,
		"x" : x,
		"z" : z,
		"range": range,
		"formationTemplate":formationTemplate,
		"maxSpeed": speed,
		"state": "waiting",
		"readyForNextStep" : [],
		"step":0,
		"waypoints": [],
		"offsets": {}
	};
	this.groups.set(this.nextGroupID++, group);

	return this.nextGroupID - 1;
};

GroupWalkManager.prototype.GetMaxSpeed = function(ID)
{
	// undefined if undefined
	return !!this.groups.get(ID) ? this.groups.get(ID).maxSpeed : 0;
}

GroupWalkManager.prototype.GetGroup = function(ID)
{
	// undefined if undefined
	return this.groups.get(ID);
}

GroupWalkManager.prototype.SetReady = function(ID, ent)
{
	if (!this.groups.get(ID))
	{
		error("Entity " + ent + " ready for unkown group " + ID);
		return;
	}
	let group = this.groups.get(ID);
	if (group.entities.indexOf(ent) === -1)
	{
		error("Entity " + ent + " ready for group " + ID + " it is not a part of.");
		return;
	}
	if (group.readyForNextStep.indexOf(ent) === -1)
		group.readyForNextStep.push(ent);

	if (group.readyForNextStep.length !== group.entities.length)
		return;

	group.readyForNextStep = [];
	if (group.state == "waiting")
	{
		group.rallyPoint = { "x":0, "z": 0 };
		// TODO: it might be better to get an averaged position, but we need to make sure to arrive somewhere units can reach.
		let cmpPosition = Engine.QueryInterface(group.entities[0], IID_Position);
		group.rallyPoint.x = cmpPosition.GetPosition2D().x;
		group.rallyPoint.z = cmpPosition.GetPosition2D().y;
		let cmpPathfinder = Engine.QueryInterface(SYSTEM_ENTITY, IID_Pathfinder);
		let path = cmpPathfinder.ComputePath(group.rallyPoint.x,group.rallyPoint.z, group.x, group.z, "large");
		group.waypoints = path;
		group.step = group.waypoints.length;
		if (group.waypoints.length > 2)
		{
			group.rallyPoint = { "x":group.waypoints[group.step-1].x, "z":group.waypoints[group.step-1].y };
			group.step--;
		}
		
		// compute offsets.
		let p1 = new Vector2D(cmpPosition.GetPosition2D().x, cmpPosition.GetPosition2D().y);
		let p2 = new Vector2D(group.rallyPoint.x, group.rallyPoint.z);
		p1.sub(p2).mult(-1);

		group.offsets = this.ComputeOffsetsForWaypoint(cmpPosition.GetRotation().y, p1, group.entities);

		group.state = "walking";
	}
	else if (group.state == "walking")
	{
		if (group.step === 0)
		{
			group.state = "arrived";
			return;
		}
		let p1 = new Vector2D(group.rallyPoint.x, group.rallyPoint.z);

		group.rallyPoint = { "x":group.waypoints[group.step-1].x, "z":group.waypoints[group.step-1].y };
		group.step--;

		// compute offsets.
		let p2 = new Vector2D(group.rallyPoint.x, group.rallyPoint.z);
		p1.sub(p2).mult(-1);

		let angle = Math.atan2(p1.x, p1.y);
		group.offsets = this.ComputeOffsetsForWaypoint(angle, p1, group.entities);

		group.state = "walking";
	}
}

GroupWalkManager.prototype.ComputeOffsetsForWaypoint = function(angle, center, entities)
{
	// for now we'll do a simple rectangular formations
	// TODO: support more stuff.
	let ret = {};

	let xW = Math.min(6, Math.max(2, entities.length/4));
	let y = -1;
	let largeEntities = [];
	for (let i = 0; i < entities.length; i++)
	{
		let ent = entities[i];
		let cmpUnitMotion = Engine.QueryInterface(ent, IID_UnitMotion);
		if (cmpUnitMotion.GetUnitClearance() > 1)
		{
			largeEntities.push(ent);
			continue;
		}
		let x = i % xW;
		if (x == 0)
			y++;
		let offsetX = 3 * (x-(xW+1)/2.0);
		let offsetY = -4 * y;
		let vector = new Vector2D(offsetX, offsetY);
		ret[ent] = vector.rotate(angle);
	}
	let baseY = y * -4;
	y = 0;
	xW = Math.min(3, Math.max(1, largeEntities.length/2));
	for (let i = 0; i < largeEntities.length; i++)
	{
		let ent = largeEntities[i];
		let x = i % xW;
		if (x == 0)
			y++;
		let offsetX = 9 * (x-xW/2.0);
		let offsetY = baseY -10 * y;
		let vector = new Vector2D(offsetX, offsetY);
		ret[ent] = vector.rotate(angle);
	}
	return ret;
}

GroupWalkManager.prototype.ResignFromGroup = function(ID, ent)
{
	let group = this.groups.get(ID);
	if (!group)
		return;
	group.entities.splice(group.entities.indexOf(ent), 1);
	if (group.readyForNextStep.indexOf(ent) !== -1)
		group.readyForNextStep.splice(group.readyForNextStep.indexOf(ent), 1);

	if (!group.entities.length)
		this.groups.delete(ID);
}

Engine.RegisterSystemComponentType(IID_GroupWalkManager, "GroupWalkManager", GroupWalkManager);
