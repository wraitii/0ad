var cmpTrigger = Engine.QueryInterface(SYSTEM_ENTITY, IID_Trigger);

Trigger.prototype.StartCutscene = function(data)
{
	var cmpCinemaManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_CinemaManager);
	if (!cmpCinemaManager)
		return;

	var cmpRangeManager = Engine.QueryInterface(SYSTEM_ENTITY, IID_RangeManager);
	cmpRangeManager.SetLosRevealAll(-1, true);

	cmpCinemaManager.AddPath({
		"name": "path",
		"orientation": "target",
		"targetNodes": [
			{ "deltaTime": 0, "position": new Vector3D(756.3661499023438,331.74896240234375,506.24627685546875	).add(new Vector3D(-0.777819037437439,-0.5735764503479004,-0.2569195628166199).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(602.4013061523438,331.7489013671875,47.745487213134766	).add(new Vector3D(-0.47752082347869873,-0.5735764503479004,0.6655704379081726).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(113.9375000000000,331.7488708496094,86.86974334716797	).add(new Vector3D(0.5695949792861938,-0.5735764503479004,0.5887033343315125).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(53.42950439453125,331.7489013671875,679.3149414062500	).add(new Vector3D(0.6311855912208557,-0.5735764503479004,-0.5221253633499146).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(756.3661499023438,331.74896240234375,506.24627685546875) },
			{ "deltaTime": 5, "position": new Vector3D(602.4013061523438,331.7489013671875,47.745487213134766) },
			{ "deltaTime": 5, "position": new Vector3D(113.9375,331.7488708496094,86.86974334716797) },
			{ "deltaTime": 5, "position": new Vector3D(53.42950439453125,331.7489013671875,679.31494140625) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

//{z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}\), \({z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}

cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
