var timer;
var lastStanza;
var g_PlayerAssignments;

function escapeText(text)
{
	return text.replace(/\\/g, "\\\\").replace(/\[/g, "\\[");
}

/**
 * Merge players by team to remove duplicate Team entries, thus reducing the packet size of the lobby report.
 */
function playerDataToStringifiedTeamList(playerData)
{
	let teamList = {};

	for (let pData of playerData)
	{
		let team = pData.Team === undefined ? -1 : pData.Team;
		if (!teamList[team])
			teamList[team] = [];
		teamList[team].push(pData);
		delete teamList[team].Team;
	}

	return escapeText(JSON.stringify(teamList));
}

/**
 * Send a list of playernames and distinct between players and observers.
 * Don't send teams, AIs or anything else until the game was started.
 */
function formatClientsForStanza()
{
	let connectedPlayers = 0;
	let playerData = [];

	for (let guid in g_PlayerAssignments)
	{
		let pData = { "Name": g_PlayerAssignments[guid].name };

		if (g_PlayerAssignments[guid].player != -1)
			++connectedPlayers;
		else
			pData.Team = "observer";

		playerData.push(pData);
	}

	return {
		"list": playerDataToStringifiedTeamList(playerData),
		"connectedPlayers": connectedPlayers
	};
}

function sendStanza()
{
	warn("Sending Stanza");
	warn(uneval(lastStanza));
	Engine.SendRegisterGame(lastStanza);
	timer = undefined;
}

function onSettingsUpdate(settings)
{
	try
	{
		let clients = formatClientsForStanza();

		let stanza = {
			"name": Engine.LobbyGetNick() + " dedicated server",
			"hostUsername": Engine.LobbyGetNick(),
			"hostJID": "", // Overwritten by C++, placeholder.
			"mapName": settings.initAttribs.map,
			// TODO: if the map name was always up-to-date we wouldn't need the mapcache here.
			"niceMapName": settings.initAttribs.settings.Name,
			"mapSize": settings.initAttribs.settings.Size,
			"mapType": settings.initAttribs.mapType,
			"victoryConditions": Array.from(settings.initAttribs.settings.VictoryConditions).join(","),
			"nbp": clients.connectedPlayers,
			"maxnbp": settings.initAttribs.settings.PlayerData.length,
			"players": clients.list,
			// TODO: fetch host mods.
			"mods": "[]",
			"hasPassword": "",
		};

		// Only send the stanza if one of these properties changed
		if (lastStanza && Object.keys(stanza).every(prop => lastStanza[prop] == stanza[prop]))
			return;

		lastStanza = stanza;
		if (!timer)
			timer = setTimeout(() => sendStanza(), 1000);
	}
	catch(err)
	{
		error(uneval(err));
	}
}

Engine.LoadScript("dedicatedserver/timer.js");

function Tick()
{
	updateTimers();
	while (true)
	{
		let message = Engine.LobbyGuiPollNewMessages()
		if (!message)
			break;

		warn("Lobby:" + uneval(message));
	}

	while (true)
	{
		let message = Engine.PollNetworkClient()
		if (!message)
			break;

		warn("Message: " + message.type);
		if (message.type === "start")
			Engine.MarkLoadFinished();
		else if (message.type === "players")
			g_PlayerAssignments = message.newAssignments;
		else if (message.type === "gamesetup" && (message.data.type === "update" || message.data.type === "initial-update"))
			onSettingsUpdate(message.data)
		else
			warn(uneval(message));
	}

	// False means DON'T QUIT
	return false;
}
