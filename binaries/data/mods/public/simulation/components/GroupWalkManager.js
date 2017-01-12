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
	let group = {
		"entities" : formableEntsID,
		"x" : x,
		"z" : z,
		"range": range,
		"formationTemplate":formationTemplate,
		"state": "waiting",
		"readyForNextStep" : [],
		"step":0,
	};
	this.groups.set(this.nextGroupID++, group);

	return this.nextGroupID - 1;
};

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
		let cmpPosition = Engine.QueryInterface(group.entities[0], IID_Position);
		group.rallyPoint.x = cmpPosition.GetPosition2D().x;
		group.rallyPoint.z = cmpPosition.GetPosition2D().y;
		// TODO: compute proper offsets from Formation.JS
		// TODO: compute path and generate waypoints
		group.state = "walking";
		group.step = 1;
	}
	else if (group.state == "walking")
	{
		if (group.step === 0)
		{
			group.state = "arrived";
			return;
		}
		group.step--;
		group.rallyPoint = { "x":group.x, "z":group.z };
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
