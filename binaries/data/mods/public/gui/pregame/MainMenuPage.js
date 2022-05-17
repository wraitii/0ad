/**
 * This is the handler that coordinates all other handlers on this GUI page.
 */
class MainMenuPage
{
	constructor(data, hotloadData, mainMenuItems, backgroundLayerData, projectInformation, communityButtons)
	{
		this.backgroundHandler = new BackgroundHandler(pickRandom(backgroundLayerData));
		this.menuHandler = new MainMenuItemHandler(mainMenuItems);
		this.splashScreenHandler = new SplashScreenHandler(data, hotloadData && hotloadData.splashScreenHandler);

		new MusicHandler();
		new ProjectInformationHandler(projectInformation);
		new CommunityButtonHandler(communityButtons);

		// Need to shim console for this to work
		globalThis.console = {
			log: error,
			warn: error
		};

		Engine.LoadESModule("wasm/asmExample.mjs");
	}

	getHotloadData()
	{
		return {
			"splashScreenHandler": this.splashScreenHandler.getHotloadData()
		};
	}
}

class MusicHandler
{
	constructor()
	{
		initMusic();
		global.music.setState(global.music.states.MENU);
	}
}

class ProjectInformationHandler
{
	constructor(projectInformation)
	{
		for (let objectName in projectInformation)
			for (let propertyName in projectInformation[objectName])
				Engine.GetGUIObjectByName(objectName)[propertyName] = projectInformation[objectName][propertyName];
	}
}

class CommunityButtonHandler
{
	constructor(communityButtons)
	{
		let buttons = Engine.GetGUIObjectByName("communityButtons").children;

		communityButtons.forEach((buttonInfo, i) => {
			let button = buttons[i];
			button.hidden = false;
			for (let propertyName in buttonInfo)
				button[propertyName] = buttonInfo[propertyName];
		});

		if (buttons.length < communityButtons.length)
			error("GUI page has space for " + buttons.length + " community buttons, but " + menuItems.length + " items are provided!");
	}
}

async function toto()
{
	let to;
	try {
		const data = Uint8Array.from([
0x00, 0x61, 0x73, 0x6D, 0x01, 0x00, 0x00, 0x00, 0x01, 0x07, 0x01, 0x60, 0x02, 0x7F, 0x7F, 0x01, 0x7F, 0x03, 0x02, 0x01, 0x00, 0x05, 0x03, 0x01, 0x00, 0x00, 0x07, 0x10, 0x02, 0x03, 0x61, 0x64, 0x64, 0x00, 0x00, 0x06, 0x6D, 0x65, 0x6D, 0x6F, 0x72, 0x79, 0x02, 0x00, 0x0A, 0x09, 0x01, 0x07, 0x00, 0x20, 0x00, 0x20, 0x01, 0x6A, 0x0B, 0x00, 0x24, 0x10, 0x73, 0x6F, 0x75, 0x72, 0x63, 0x65, 0x4D, 0x61, 0x70, 0x70, 0x69, 0x6E, 0x67, 0x55, 0x52, 0x4C, 0x12, 0x2E, 0x2F, 0x72, 0x65, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x2E, 0x77, 0x61, 0x73, 0x6D, 0x2E, 0x6D, 0x61, 0x70])
		warn(uneval(data));
		to = await WebAssembly.instantiate(data);
	} catch (ex) {
		warn(uneval(ex));
		warn(ex.toString());
	}
	warn("Done");
	try {
		warn(uneval(to));
		warn(uneval(to.instance.exports.add(5, 9)));
	} catch (ex)
	{
		warn(uneval(ex));
		warn(ex.toString());

	}
}
