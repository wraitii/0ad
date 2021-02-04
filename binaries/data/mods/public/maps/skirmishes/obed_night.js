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
			{ "deltaTime": 0, "position": new Vector3D(1270.2269287109375,24.247547149658203,525.1804809570312	).add(new Vector3D(-0.8571109175682068,-0.07297495752573013,-0.5099368095397949).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1232.0391845703125,24.247547149658203,575.5914306640625	).add(new Vector3D(-0.9473342299461365,-0.07297495752573013,-0.3118214011192322).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1173.763916015625,28.914596557617188,545.3802490234375	).add(new Vector3D(-0.8740957975387573,-0.07297495752573013,-0.48024073243141174).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1084.1422119140625,70.60296630859375,489.4082946777344	).add(new Vector3D(-0.9397404789924622,-0.29344743490219116,-0.17543227970600128).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(937.0448608398438,179.8878173828125,362.1608886718750	).add(new Vector3D(-0.8087526559829712,-0.5356340408325195,0.24293076992034912).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(527.9802856445312,214.1100311279297,144.09922790527344	).add(new Vector3D(-0.08875357359647751,-0.5356340408325195,0.8397731781005859).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(40.28840255737305,231.2898712158203,498.9992370605469	).add(new Vector3D(0.5438663959503174,-0.5356340408325195,0.645991861820221).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(-250.565185546875,231.2898406982422,1063.351318359375	).add(new Vector3D(0.837176501750946,-0.5356340408325195,-0.11059724539518356).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(1270.2269287109375,24.247547149658203,525.1804809570312) },
			{ "deltaTime": 5, "position": new Vector3D(1232.0391845703125,24.247547149658203,575.5914306640625) },
			{ "deltaTime": 5, "position": new Vector3D(1173.763916015625,28.914596557617188,545.3802490234375) },
			{ "deltaTime": 5, "position": new Vector3D(1084.1422119140625,70.60296630859375,489.4082946777344) },
			{ "deltaTime": 5, "position": new Vector3D(937.0448608398438,179.8878173828125,362.160888671875) },
			{ "deltaTime": 5, "position": new Vector3D(527.9802856445312,214.1100311279297,144.09922790527344) },
			{ "deltaTime": 5, "position": new Vector3D(40.28840255737305,231.2898712158203,498.9992370605469) },
			{ "deltaTime": 5, "position": new Vector3D(-250.565185546875,231.2898406982422,1063.351318359375) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

//{z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}\), \({z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}


cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
