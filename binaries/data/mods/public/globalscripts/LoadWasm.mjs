import asc from "assemblyscript/asc";

export async function LoadWasm(path)
{
	let moduleData;
	try {
		let res = await asc.main([
			path,
			"--outFile", "__.wasm",
			"--optimize"
		], {
			readFile: (x) => {
				if (x === "asconfig.json")
					return null;
				return Engine.ReadFile(x)
			},
			writeFile: (filename, data) => moduleData = data,
			listFiles: (x) => { warn("now" + x); error(x); return [] },
		});
	} catch (exc)
	{
		error(uneval(exc));
	}
	try {
		return (await WebAssembly.instantiate(moduleData)).instance.exports;
	} catch (err) {
		warn(uneval(err));
	}
}
