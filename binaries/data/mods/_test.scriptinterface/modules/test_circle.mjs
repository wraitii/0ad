import {circle_area} from "./geometry/test_area.mjs";

class Circle {
    constructor(radius) {
        this.radius = radius;
    }

    get area() {
        return circle_area(this.radius);
    }
}

export default Circle;
