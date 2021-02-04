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
			{ "deltaTime": 0, "position": new Vector3D(1305.6390380859375,471.8636169433594,10.302860260009766	).add(new Vector3D(-0.6585933566093445,-0.5735764503479004,0.48709845542907715).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1405.0277099609375,471.8636169433594,467.2446594238281	).add(new Vector3D(-0.6585933566093445,-0.5735764503479004,0.48709845542907715).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(1388.708740234375,471.8636169433594,907.7100219726562	).add(new Vector3D(-0.6585933566093445,-0.5735764503479004,0.48709845542907715).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(978.6598510742188,471.8636169433594,1590.3873291015625	).add(new Vector3D(-0.33691880106925964,-0.5735764503479004,-0.7466564178466797).mult(100))},
			{ "deltaTime": 5, "position": new Vector3D(429.2502136230469,581.9830932617188,1829.9393310546875	).add(new Vector3D(0.19349412620067596,-0.5735764503479004,-0.7959712147712708).mult(100))},
		],
		"positionNodes": [
			{ "deltaTime": 0, "position": new Vector3D(1305.6390380859375,471.8636169433594,10.302860260009766) },
			{ "deltaTime": 5, "position": new Vector3D(1405.0277099609375,471.8636169433594,467.2446594238281) },
			{ "deltaTime": 5, "position": new Vector3D(1388.708740234375,471.8636169433594,907.7100219726562) },
			{ "deltaTime": 5, "position": new Vector3D(978.6598510742188,471.8636169433594,1590.3873291015625) },
			{ "deltaTime": 5, "position": new Vector3D(429.2502136230469,581.9830932617188,1829.9393310546875) },
		]
	});
	cmpCinemaManager.AddCinemaPathToQueue("path");
	cmpCinemaManager.Play();
};

//{z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}\), \({z:([-0-9.]+), y:([-0-9.]+), x:([-0-9.]+)}

cmpTrigger.DoAfterDelay(1000, "StartCutscene", {});

//cmpTrigger.DoAfterDelay(6000, "StartCutscene", {});
