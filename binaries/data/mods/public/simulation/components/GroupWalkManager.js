function GroupWalkManager() {}

GroupWalkManager.prototype.Schema =
	"<a:component type='system'/><empty/>";

// This is a simple system component that keeps track of existing "groups"
// when a "walk together" order is issued as well as necessary information.

GroupWalkManager.prototype.Init = function()
{
	this.groups = new Map();
	this.nextGroupID = 0;
	warn("here");
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
		// this seems to return oddly few waypoints ?
		let path = cmpPathfinder.ComputePath(group.rallyPoint.x,group.rallyPoint.z, group.x, group.z, "default");
		group.waypoints = path;
		group.step = group.waypoints.length;
		if (group.waypoints.length > 2)
		{
			group.rallyPoint = { "x":group.waypoints[group.step-1].x, "z":group.waypoints[group.step-1].y };
			group.step--;
		}
		// TODO: compute proper offsets from Formation.JS
		group.state = "walking";
	}
	else if (group.state == "walking")
	{
		if (group.step === 0)
		{
			group.state = "arrived";
			return;
		}
		group.rallyPoint = { "x":group.waypoints[group.step-1].x, "z":group.waypoints[group.step-1].y };
		group.step--;
		// TODO: compute proper offsets from Formation.JS
		group.state = "walking";
	}
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
