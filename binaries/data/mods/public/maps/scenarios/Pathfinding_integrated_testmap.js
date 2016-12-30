warn("Setting up test");

var tests = {};
var start = 0;

var failedTests = 0;

Trigger.prototype.setupTests = function()
{
	let cmpTimer = Engine.QueryInterface(SYSTEM_ENTITY, IID_Timer);
	start = cmpTimer.GetTime();

	let cmpTechMgr = QueryPlayerIDInterface(1, IID_TechnologyManager);

	//XXXtreme hack: create a fake technology to drastically limit the range of everybody in place.
	cmpTechMgr.modifications = {};
	cmpTechMgr.modifications['Attack/Ranged/MaxRange'] = [ {"affects":["Unit"], "replace":2.5} ];

	cmpTechMgr = QueryPlayerIDInterface(2, IID_TechnologyManager);
	cmpTechMgr.modifications = {};
	cmpTechMgr.modifications['Attack/Ranged/MaxRange'] = [ {"affects":["Unit"], "replace":0} ];

	let cmpFoundation = Engine.QueryInterface(391, IID_Foundation);
	cmpFoundation.InitialiseConstruction(1, "structures/maur_house");

	// Ready to order people around.

	tests = {
		"21" : {"target":353, "continuous":true}, // inbetween house: allies present
		"200" : {"target":352, "continuous":true, "expectfail":true}, // Inbetweenhouse- should fail
		"24" : {"target":361}, // basic labyrinth
		"23" : {"target":356},	// corner house
		"22" : {"target":354},	// between house + around
		"49" : {"target":355},	// straight
		"210" : {"target":357},	// formation - sparse
		"211" : {"target":358},	// formation - wall
		"186" : {"target":359, "expectfail" : true},	// inside forest - dense
		"372" : {"target":359, "continuous":true, "expectfail" : true},	// inside forest - dense
		"187" : {"target":360},	// inside forest - sparse
		"50" : {"target":352},	// super long trip
		"46" : {"target":363},	// trip inside hills
		"53" : {"target":362},	// labyrinth: with hole for small
		"54" : {"target":365},	// labyrinth: with hole for small - this is the elephant
		"85" : {"target":362},	// labyrinth: with hole for small - this is the ram
		"390" : {"target":391, "type" : "build"},	// build a house
		"393" : {"target":392, "type" : "hunt"},	// hunt a chicken
	};

	// order units to move
	for (let test in tests)
	{
		let cmpTesterAI = Engine.QueryInterface(+test, IID_UnitAI);
		let cmpPos = Engine.QueryInterface(tests[test].target, IID_Position);

		if (tests[test].type && tests[test].type === "build")
		{
			cmpTesterAI.Repair(tests[test].target, false, false);
			continue;
		}
		else if (tests[test].type && tests[test].type === "hunt")
		{
			cmpTesterAI.Gather(tests[test].target, false);
			continue;
		}

		if (!tests[test].continuous)
			cmpTesterAI.Walk(cmpPos.GetPosition2D().x, cmpPos.GetPosition2D().y, false);
		else
		{
			let TgPos = new Vector2D(cmpPos.GetPosition2D().x,cmpPos.GetPosition2D().y);
			let MyCmpPos = Engine.QueryInterface(+test, IID_Position);
			let MyPos = new Vector2D(MyCmpPos.GetPosition2D().x,MyCmpPos.GetPosition2D().y);

			// must be below C++ constant for "short pathfinder only"
			let vector = new Vector2D(TgPos.x,TgPos.y);
			vector = vector.sub(MyPos).normalize().mult(3.4); // 2 happened to put a waypoint inside a unit, which is unreachable.
			
			let position = new Vector2D(MyPos.x,MyPos.y);
			while (position.distanceToSquared(TgPos) > 12)
			{
				position.add(vector);
				cmpTesterAI.Walk(position.x, position.y, true);
			}
		}
	}

	// set up traders, they're not tested but their behavior should be looked at.
	let traders = Engine.GetEntitiesWithInterface(IID_Trader);
	for (let tID of traders)
	{
		let cmpTraderAI = Engine.QueryInterface(+tID, IID_UnitAI);
		cmpTraderAI.SetupTradeRoute(401, 402, undefined, false);
	}

	let cmpTrigger = Engine.QueryInterface(SYSTEM_ENTITY, IID_Trigger);

	cmpTrigger.RegisterTrigger("OnInterval", "CheckUnits", {
		"enabled": true,
		"delay": 2 * 1000,
		"interval": 1 * 1000,
	});
}

let cmpTrigger = Engine.QueryInterface(SYSTEM_ENTITY, IID_Trigger);
cmpTrigger.DoAfterDelay(4000, "setupTests", {});

function Success(test)
{
	warn("SUCCESS : test " + test + " succeeded");
	delete(tests[test]);
}

function Fail(test)
{
	error("ERROR : test " + test + " failed");
	delete(tests[test]);
	failedTests++;
}

function testBuild(test)
{
	let cmpFoundation = Engine.QueryInterface(tests[test].target, IID_Foundation);
	if (cmpFoundation.GetBuildProgress() > 0.1)
		Success(test);
}

function testHunt(test)
{
	/*
	let cmpFoundation = Engine.QueryInterface(tests[tester].target, IID_Foundation);
	if (cmpFoundation.GetBuildProgress() > 0)
		Success(tester);
	*/
}

function testWalk(test)
{
	let cmpTimer = Engine.QueryInterface(SYSTEM_ENTITY, IID_Timer);
	let time = cmpTimer.GetTime();

	let cmpTesterAI = Engine.QueryInterface(+test, IID_UnitAI);
	let cmpTesterUM = Engine.QueryInterface(+test, IID_UnitMotion);
	if (cmpTesterUM.IsTryingToMove())
			return;

	let cmpPos = Engine.QueryInterface(tests[test].target, IID_Position);
	let TgPos = new Vector2D(cmpPos.GetPosition2D().x,cmpPos.GetPosition2D().y);
	let MyCmpPos = Engine.QueryInterface(+test, IID_Position);
	let MyPos = new Vector2D(MyCmpPos.GetPosition2D().x,MyCmpPos.GetPosition2D().y);
	
	cmpTesterAI.Stop();

	if (MyPos.distanceTo(TgPos) > 8 || (tests[test].underTime && time > tests[test].underTime))
		if (!tests[test].expectfail)
		{
			Fail(test);
			return;
		}
	else if (tests[test].expectfail && MyPos.distanceTo(TgPos) <= 8 && (!tests[test].underTime || time <= tests[test].underTime))
	{
		Fail(test);
		return;
	}
	Success(test);
}


Trigger.prototype.CheckUnits = function(data)
{
	// Check all tests
	let cmpTimer = Engine.QueryInterface(SYSTEM_ENTITY, IID_Timer);
	let time = cmpTimer.GetTime();

	let testsleft = 0;
	for (let test in tests)
	{
		testsleft++;
		let cmpTesterAI = Engine.QueryInterface(+test, IID_UnitAI);

		if (!tests[test].type)
			testWalk(test);
		else if (tests[test].type === "build")
			testBuild(test);
		else if (tests[test].type === "hunt")
			testHunt(test);
	}
	if (time > 120000)
		for (let test in tests)
		{
			let cmpTesterAI = Engine.QueryInterface(+test, IID_UnitAI);
			cmpTesterAI.Stop();
			Fail(test);
		}
	if (testsleft === 0)
	{
		if (failedTests > 0)
			QueryPlayerIDInterface(1, IID_Player).SetState("defeated");
		else
			QueryPlayerIDInterface(1, IID_Player).SetState("won");
	}
}
