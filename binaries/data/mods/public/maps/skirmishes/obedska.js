var cmpTrigger = Engine.QueryInterface(SYSTEM_ENTITY, IID_Trigger);

Trigger.prototype.StartCutscene = function(data)
{
	var cmpCinemaManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_CinemaManager);
	if (!cmpCinemaManager)
		return;
	cmpCinemaManager.AddCinemaPathToQueue("test");
	cmpCinemaManager.Play();
};

const TRADER = 24272;

Trigger.prototype.TraderMove = function()
{
	ProcessCommand(1, {
		"type": "walk",
		"entities": [TRADER],
		"x": 550,
		"z": 860,
		"queued": false,
		"force": true
	});
	ProcessCommand(1, {
		"type": "walk",
		"entities": [TRADER],
		"x": 575,
		"z": 828,
		"queued": true,
		"force": true
	});
};

Trigger.prototype.WomanGo = function()
{
	ProcessCommand(1, {
		"type": "walk",
		"entities": [24276],
		"x": 582,
		"z": 839,
		"queued": false,
		"force": true
	});
};

Trigger.prototype.TraderMeet = function()
{
	ProcessCommand(1, {
		"type": "walk",
		"entities": [24269, 24275],
		"x": 575,
		"z": 828,
		"queued": false,
		"force": true
	});
};

Trigger.prototype.Cavalry = function()
{
	ProcessCommand(2, {
		"type": "walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285],
		"x": 478,
		"z": 876,
		"queued": false,
		"force": true,
		"formation": "special/formations/box"
	});
	ProcessCommand(2, {
		"type": "walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285],
		"x": 550,
		"z": 860,
		"queued": true,
		"force": true,
		"formation": "special/formations/box"
	});
	ProcessCommand(2, {
		"type": "attack-walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285],
		"x": 575,
		"z": 828,
		"queued": true,
		"force": true
	});
};

Trigger.prototype.WomenFlee = function()
{
	ProcessCommand(1, {
		"type": "walk",
		"entities": [24272, 24274, 24275, 24276, 24277],
		"x": 700,
		"z": 900,
		"queued": false,
		"force": true
	});
};

Trigger.prototype.CavChase = function()
{
	ProcessCommand(2, {
		"type": "attack",
		"entities": [24278],
		"target": 24272,
		"queued": false,
		"force": true
	});
	ProcessCommand(2, {
		"type": "attack",
		"entities": [24279],
		"target": 24275,
		"queued": false,
		"force": true
	});
	ProcessCommand(2, {
		"type": "attack",
		"entities": [24285],
		"target": 24277,
		"queued": false,
		"force": true
	});
};

Trigger.prototype.CavLeave = function()
{
	ProcessCommand(2, {
		"type": "walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285, 24330],
		"x": 550,
		"z": 860,
		"queued": false,
		"force": true,
		"formation": "special/formations/box"
	});
	ProcessCommand(2, {
		"type": "walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285, 24330],
		"x": 478,
		"z": 876,
		"queued": true,
		"force": true,
		"formation": "special/formations/box"
	});
	ProcessCommand(2, {
		"type": "walk",
		"entities": [24278, 24279, 24280, 24281, 24282, 24283, 24285, 24330],
		"x": 401,
		"z": 836,
		"queued": true,
		"force": true,
		"formation": "special/formations/box"
	});
};

cmpTrigger.DoAfterDelay(4000, "TraderMove", {});
cmpTrigger.DoAfterDelay(1000, "Cavalry", {});
cmpTrigger.DoAfterDelay(7000, "WomanGo", {});
cmpTrigger.DoAfterDelay(11000, "TraderMeet", {});
cmpTrigger.DoAfterDelay(16000, "WomenFlee", {});
cmpTrigger.DoAfterDelay(13000, "CavChase", {});
cmpTrigger.DoAfterDelay(21000, "CavLeave", {});

Trigger.prototype.StartCutscene = function(data)
{
	var cmpCinemaManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_CinemaManager);
	if (!cmpCinemaManager)
		return;

	cmpCinemaManager.AddPath({
		"name": "path",
		"orientation": "target",
		"targetNodes": [
			{
				"deltaTime": 0,
				"position": new Vector3D(458, 0, 876)
			},
			{
				"deltaTime": 5,
				"position": new Vector3D(575, 0, 828)
			},
			{
				"deltaTime": 10,
				"position": new Vector3D(560, 10, 820)
			},
			{
				"deltaTime": 5,
				"position": new Vector3D(575, 10, 838)
			},
			{
				"deltaTime": 7,
				"position": new Vector3D(476, -10, 876)
			},
			{
				"deltaTime": 15,
				"position": new Vector3D(378, -30, 850)
			}
		],
		"positionNodes": [
			{
				"deltaTime": 0,
				"position": new Vector3D(438, 50, 876)
			},
			{
				"deltaTime": 5,
				"position": new Vector3D(500, 90, 878)
			},
			{
				"deltaTime": 5,
				"position": new Vector3D(650, 90, 880)
			},
			{
				"deltaTime": 7,
				"position": new Vector3D(670, 120, 780)
			},
			{
				"deltaTime": 15,
				"position": new Vector3D(478, 150, 876)
			},
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

cmpTrigger.DoAfterDelay(0, "StartCutscene", {});


var cmpModifiersManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_ModifiersManager);

cmpModifiersManager.AddModifiers("higher_attack", {
	"Attack/Melee/Damage/Hack": [{ "affects": ["Unit"], "multiply": 5 }],
}, 4); // player 2 is ent 4

cmpModifiersManager.AddModifiers("higher_attack", {
	"Vision/Range": [{ "affects": ["Unit"], "replace": 40 }],
}, 3);


//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
