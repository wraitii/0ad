/* Copyright (C) 2022 Wildfire Games.
 * This file is part of 0 A.D.
 *
 * 0 A.D. is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * 0 A.D. is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 0 A.D.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "precompiled.h"

#include "DedicatedServer.h"

#include "lib/input.h"
#include "lib/timer.h"
#include "lobby/IXmppClient.h"
#include "lobby/scripting/JSInterface_Lobby.h"
#include "network/NetClient.h"
#include "network/NetServer.h"
#include "network/StunClient.h"
#include "network/scripting/JSInterface_Network.h"
#include "ps/ConfigDB.h"
#include "ps/Game.h"
#include "ps/GUID.h"
#include "ps/GameSetup/CmdLineArgs.h"
#include "scriptinterface/FunctionWrapper.h"
#include "scriptinterface/ScriptContext.h"
#include "scriptinterface/ScriptInterface.h"
#include "simulation2/Simulation2.h"

#include "lib/external_libraries/libsdl.h"

#include <chrono>
#include <thread>

namespace
{
void LoadFinished()
{
	if (!g_NetClient)
		return;
	g_NetClient->LoadFinished();
}
void LoadScript(const ScriptInterface& scriptInterface, CStr file)
{
	if (!scriptInterface.LoadGlobalScriptFile(file))
		ScriptException::Raise(ScriptRequest(scriptInterface), "Could not load file %s", file.c_str());
}
}

void RunDedicatedServer(const CmdLineArgs& args)
{
	ScriptInterface scriptInterface("Engine", "DedicatedServer", g_ScriptContext);

	{
		ScriptRequest rq(scriptInterface);
		JSI_Lobby::RegisterScriptFunctions(rq);
		JSI_Network::RegisterScriptFunctions(rq);
		ScriptFunction::Register<&LoadFinished>(rq, "MarkLoadFinished");
		ScriptFunction::Register<&LoadScript>(rq, "LoadScript");
	}

	if (!scriptInterface.LoadGlobalScriptFile("dedicatedserver/DedicatedServer.js"))
	{
		LOGERROR("Could not load dedicated server script");
		return;
	}

	std::string lobbyPassword;
	std::string lobbyLogin;
	std::string lobbyRoom;
	CFG_GET_VAL("lobby.password", lobbyPassword);
	CFG_GET_VAL("lobby.login", lobbyLogin);
	CFG_GET_VAL("lobby.room", lobbyRoom);

	LOGWARNING("Attempting to connect to lobby as %s", lobbyLogin);
	g_XmppClient = IXmppClient::create(&scriptInterface,
						lobbyLogin,
						lobbyPassword,
						lobbyRoom,
						lobbyLogin);
	g_XmppClient->connect();

	g_NetServer = new CNetServer(true);

	u16 port = 20595;
	if (!g_NetServer->SetupConnection(port))
	{
		SAFE_DELETE(g_NetServer);
		SAFE_DELETE(g_XmppClient);
		return;
	}

	g_NetServer->SetConnectionDataViaSTUN();

	CStr ip = g_NetServer->GetPublicIp();

	// Generate a secret to identify the host client.
	std::string secret = ps_generate_guid();
	g_NetServer->SetControllerSecret(secret);

	double time = timer_Time();

	constexpr double LOBBY_CONNECTION_TIMEOUT = 30.0;
	while (g_XmppClient && !g_XmppClient->isConnected())
	{
		g_XmppClient->recv();

		if (timer_Time() - time > LOBBY_CONNECTION_TIMEOUT)
		{
			LOGERROR("Could not connect to 0 A.D. Lobby. Please make sure the configuration is correct");
			SAFE_DELETE(g_NetServer);
			SAFE_DELETE(g_XmppClient);
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}


	LOGWARNING("Hosting at %s:%i, JID %s, secret %s", ip, port, g_XmppClient->GetJID(), secret);
	LOGWARNING("Engine.PushGuiPage('page_gamesetup_mp.xml', { 'multiplayerGameType': 'dedicated', 'name': '%s', 'pass': '', 'hostJID': '%s', 'secret': '%s' });", lobbyLogin, g_XmppClient->GetJID(), secret);

	g_NetClient = new CNetClient(nullptr, scriptInterface);
	g_NetClient->SetUserName(L"Fake observer");
	g_NetClient->SetHostJID("Fake observer");
	g_NetClient->SetGamePassword("");
	g_NetClient->SetControllerSecret("");
	g_NetClient->SetupServerData("127.0.0.1", port, false);
	if (!g_NetClient->SetupConnection(nullptr))
		LOGWARNING("failed to connect client");

	while (g_NetServer)
	{
		if (g_XmppClient)
			g_XmppClient->recv();

		g_NetClient->Poll();

		{
			ScriptRequest rq(scriptInterface);
			JS::RootedValue glob(rq.cx, rq.globalValue());
			bool shouldQuit = false;
			ScriptFunction::Call(rq, glob, "Tick", shouldQuit);
			if (shouldQuit)
				break;
		}

		SDL_Event_ ev;
		while (in_poll_event(&ev))
			in_dispatch_event(&ev);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	};

	// Always attempt to unregister the game.
	g_XmppClient->SendIqUnregisterGame();

	g_NetClient->DestroyConnection();
	SAFE_DELETE(g_NetClient);
	SAFE_DELETE(g_NetServer);

	if (g_XmppClient)
		g_XmppClient->recv();

	SAFE_DELETE(g_XmppClient);

	return;
}
