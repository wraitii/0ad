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
			{ "deltaTime": 0, "position": new Vector3D(153, 182, 122).add(new Vector3D(0.698122501373291,  -0.12115322053432465,   0.7056534886360168 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(318, 182, 289).add(new Vector3D(0.698122501373291,    -0.12115322053432465,    0.7056534886360168 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(436, 204, 408).add(new Vector3D(0.6726852655410767,   -0.2918452322483063,    0.6799418330192566 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(679, 291, 481).add(new Vector3D(-0.12154343724250793,  -0.5665917992591858,    0.8149852752685547 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1063, 418, 909).add(new Vector3D(-0.6747995018959045,  -0.5665917992591858,    -0.4728841483592987 ).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(681, 545, 1236).add(new Vector3D(-0.1910856068134308,  -0.5665917992591858,    -0.8015360236167908 ).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(153, 182, 122) },
			{ "deltaTime": 5, "position": new Vector3D(318, 182, 289) },
			{ "deltaTime": 5, "position": new Vector3D(436, 204, 408) },
			{ "deltaTime": 5, "position": new Vector3D(679, 291, 481) },
			{ "deltaTime": 5, "position": new Vector3D(1063, 418, 909) },
			{ "deltaTime": 5, "position": new Vector3D(681, 545, 1236) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
