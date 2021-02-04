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
			{ "deltaTime": 0, "position": new Vector3D(-153, 316, 517).add(new Vector3D(0.8284144997596741, -0.5555157661437988,  0.07163543999195099 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(117, 316, 686).add(new Vector3D(0.7729105353355408,   -0.5555157661437988,  -0.3066129684448242 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(642, 316, 755).add(new Vector3D(0.41272735595703125,  -0.5555157661437988,  -0.7218437194824219 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1203, 351, 716).add(new Vector3D(-0.8874087929725647, -0.3936375379562378,  -0.23990653455257416 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1400, 280, 900).add(new Vector3D(-0.8874087929725647, -0.3936375379562378,  -0.23990653455257416 ).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(-153, 316, 517) },
			{ "deltaTime": 5, "position": new Vector3D(117, 316, 686) },
			{ "deltaTime": 5, "position": new Vector3D(642, 316, 755) },
			{ "deltaTime": 5, "position": new Vector3D(1203, 351, 716) },
			{ "deltaTime": 5, "position": new Vector3D(1400, 280, 900) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};


cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
