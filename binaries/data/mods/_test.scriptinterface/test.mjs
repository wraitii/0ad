import circle from "./modules/test_circle.mjs";

const area = new circle(10).area;

if (area != (Math.PI * 100)) {
    throw new Error("Module Evalutation Error");
}
