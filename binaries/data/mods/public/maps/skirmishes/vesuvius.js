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
			{ "deltaTime": 0, "position": new Vector3D(574, 452, 654 ).add(new Vector3D(-0.21416030824184418, -0.9764558672904968, 0.025870362296700478).mult(100)) },
			{ "deltaTime": 3, "position": new Vector3D(616, 553, 671 ).add(new Vector3D(-0.39778634905815125, -0.9132617712020874, -0.08785727620124817).mult(100)) },
			{ "deltaTime": 4, "position": new Vector3D(650, 699, 731 ).add(new Vector3D(-0.3353399336338043, -0.9134439826011658, -0.23058006167411804).mult(100)) },
			{ "deltaTime": 5, "position": new Vector3D(667, 935, 908 ).add(new Vector3D(-0.17444799840450287, -0.9161279201507568, -0.36093971133232117).mult(100)) },
			{ "deltaTime": 20, "position": new Vector3D(863, 504, 1314).add(new Vector3D(-0.3979124426841736, -0.40478235483169556, -0.823296308517456).mult(100)) },
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(574, 452, 654 ) },
			{ "deltaTime": 3, "position": new Vector3D(616, 553, 671 ) },
			{ "deltaTime": 4, "position": new Vector3D(650, 699, 731 ) },
			{ "deltaTime": 5, "position": new Vector3D(667, 935, 908 ) },
			{ "deltaTime": 20, "position": new Vector3D(863, 504, 1314) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};


cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
