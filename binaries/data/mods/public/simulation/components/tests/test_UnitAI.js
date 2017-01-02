Engine.LoadHelperScript("FSM.js");
Engine.LoadComponentScript("interfaces/UnitAI.js");
Engine.LoadComponentScript("UnitAI.js");

Engine.LoadComponentScript("interfaces/Timer.js");
Engine.LoadComponentScript("interfaces/Heal.js");
Engine.LoadComponentScript("interfaces/Sound.js");
Engine.LoadComponentScript("interfaces/GarrisonHolder.js");
Engine.LoadComponentScript("interfaces/DamageReceiver.js");
Engine.LoadComponentScript("interfaces/Pack.js");

Engine.LoadHelperScript("Sound.js");

const PLAYER_ENTITY = 2;
const UNIT_ID = 3;
const TARGET_ENTITY = 4;

var lastAnimationSet = "";

function SetupMocks()
{
	AddMock(SYSTEM_ENTITY, IID_Timer, {
		SetInterval: function() { },
		SetTimeout: function() { },
	});

	AddMock(SYSTEM_ENTITY, IID_TemplateManager, {
		GetCurrentTemplateName: function(ent) { return "units/gaul_infantry_spearman_b"; },
	});

	AddMock(UNIT_ID, IID_Sound, {
		PlaySoundGroup: function() {},
	});

	AddMock(UNIT_ID, IID_Position, {
		"IsInWorld" : function() { return true; },
		"GetPosition" : function() { return new Vector2D(0,0); }
	});

	AddMock(UNIT_ID, IID_UnitMotion, {
		GetTopSpeedRatio : function() { return 0; },
		SetSpeed: function() {},
	});

	AddMock(UNIT_ID, IID_DamageReceiver, {
		SetInvulnerability : function() {},
	});

	AddMock(UNIT_ID, IID_Pack, {
		Pack : function() {},
		Unpack : function() { },
	});

	AddMock(UNIT_ID, IID_Visual, {
		SelectAnimation : function(name) { lastAnimationSet = name; },
		SetVariant : function(key, name) { },
	});
/*
	AddMock(SYSTEM_ENTITY, IID_RangeManager, {
		CreateActiveQuery: function(ent, minRange, maxRange, players, iid, flags) {
			return 1;
		},
		EnableActiveQuery: function(id) { },
		ResetActiveQuery: function(id) { if (mode == 0) return []; else return [enemy]; },
		DisableActiveQuery: function(id) { },
		GetEntityFlagMask: function(identifier) { },
	});

	AddMock(SYSTEM_ENTITY, IID_TemplateManager, {
		GetCurrentTemplateName: function(ent) { return "units/gaul_infantry_spearman_b"; },
	});

	AddMock(SYSTEM_ENTITY, IID_PlayerManager, {
		GetPlayerByID: function(id) { return PLAYER_ENTITY; },
		GetNumPlayers: function() { return 1; },
	});*/
}

// The intention of this test is to validate that unitAI states that select an animation correctly reset it when leaving
// This tests on "unevaled" FSM state instead of trying to get every state because it's basically a nightmare to get 100% coverage in UnitAI
// And this seems to be good enough to actually detect the bugs.
function testAnimationsAreReset()
{
	ResetState();
	SetupMocks();

	let cmpUnitAI = ConstructComponent(UNIT_ID, "UnitAI", { "FormationController": "false", "DefaultStance": "aggressive" });

	cmpUnitAI.OnCreate();
	TS_ASSERT_EQUALS(cmpUnitAI.UnitFsm.GetCurrentState(cmpUnitAI), "INDIVIDUAL.IDLE");

	cmpUnitAI.order = {"data" : { "targetClasses" : [], "target" : TARGET_ENTITY }};

	let TestForReset = function(cmpUnitAI, totest)
	{
		let shouldReset = false;
		for (let fc in totest)
		{
			if (fc === "leave")
				continue;

			let stringified = uneval(totest[fc]);
			let pos = stringified.search("SelectAnimation");
			if (pos !== -1)
			{
				let animation = stringified.substr(pos, stringified.indexOf(")", pos) - pos) + ")";
				if (animation.search("idle") === -1 && animation.search(", true") === -1)
					shouldReset = true;
			}
		}
		if (shouldReset)
		{
			if (!totest.leave)
			{
				TS_FAIL("No leave");
				return false;
			}

			let doesReset = false;
			let stringified = uneval(totest.leave);
			let pos = stringified.search("SelectAnimation");
			if (pos !== -1)
			{
				let animation = stringified.substr(pos, stringified.indexOf(")", pos) - pos) + ")";
				if (animation.search("idle") !== -1)
					doesReset = true;
			}
			if (!doesReset)
			{
				TS_FAIL("No reset in the leave");
				return false;
			}
		}
		return true;
	}

	for (let i in cmpUnitAI.UnitFsmSpec.INDIVIDUAL)
	{
		// skip the default "Enter" states and such.
		if (typeof cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i] === "function")
			continue;

		// skip IDLE because the following dumb test doesn't detect it properly.
		if (i === "IDLE")
			continue;

		// check if this state has 2 levels or 3 levels
		// eg INDIVIDUAL.FLEEING or INDIVIDUAL.COMBAT.SOMETHING
		let hasChildren = false;
		for (let child in cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i])
			if (typeof cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i][child] !== "function")
			{
				hasChildren = true;
				break;
			}
		if (hasChildren)
		{
			for (let child in cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i])
			{
				if (!TestForReset(cmpUnitAI, cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i][child]))
					warn("Failed in " + i + " substate " + child);
			}
		}
		else
			if (!TestForReset(cmpUnitAI, cmpUnitAI.UnitFsmSpec.INDIVIDUAL[i]))
				warn("Failed in " + i);
	}

//	TS_ASSERT_EQUALS(ApplyValueModificationsToEntity("Component/Value", 5, targetEnt), 15);
}

testAnimationsAreReset();