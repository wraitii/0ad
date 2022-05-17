import { LoadWasm } from 'globalscripts/LoadWasm.mjs'

let addModule = await LoadWasm("wasm/add.ts");
warn("WASM Module says 4 + 5 is " + addModule.add(4, 5));
