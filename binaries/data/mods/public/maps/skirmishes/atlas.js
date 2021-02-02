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
			{ "deltaTime": 0, "position": new Vector3D(1430, 63, 129).add(new Vector3D(-0.6647380590438843,   -0.24460452795028687,  0.7058979272842407 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(1178, 63, 397).add(new Vector3D(-0.877405047416687,   -0.24460452795028687,  0.4127092957496643 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(958, 70, 438).add(new Vector3D(-0.7094832062721252,   -0.24460452795028687,  0.6609100699424744 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(839, 117, 550).add(new Vector3D(-0.6751381158828735,   -0.3855551481246948,  0.628916323184967 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(703, 184, 974).add(new Vector3D(-0.6353691816329956, -0.6066830158233643, -0.4777465760707855 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(350, 281, 1189).add(new Vector3D( 0.2786460518836975, -0.6066830158233643, -0.7445079684257507 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(42, 358, 987).add(new Vector3D( 0.748331606388092, -0.6066830158233643, -0.26820799708366394 ).mult(100))},
			{ "deltaTime": 3, "position": new Vector3D(-223, 553, 675).add(new Vector3D( 0.7913284301757812,   -0.6066830158233643,  0.0757298693060875 ).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(1430, 63, 129) },
			{ "deltaTime": 3, "position": new Vector3D(1178, 63, 397) },
			{ "deltaTime": 3, "position": new Vector3D(958, 70, 438) },
			{ "deltaTime": 3, "position": new Vector3D(839, 117, 550) },
			{ "deltaTime": 3, "position": new Vector3D(703, 184, 974) },
			{ "deltaTime": 3, "position": new Vector3D(350, 281, 1189) },
			{ "deltaTime": 3, "position": new Vector3D(42, 358, 987) },
			{ "deltaTime": 3, "position": new Vector3D(-223, 553, 675) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
