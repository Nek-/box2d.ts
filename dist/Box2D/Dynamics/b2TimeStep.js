"use strict";
/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
Object.defineProperty(exports, "__esModule", { value: true });
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
/// Profiling data. Times are in milliseconds.
var b2Profile = /** @class */ (function () {
    function b2Profile() {
        this.step = 0;
        this.collide = 0;
        this.solve = 0;
        this.solveInit = 0;
        this.solveVelocity = 0;
        this.solvePosition = 0;
        this.broadphase = 0;
        this.solveTOI = 0;
    }
    b2Profile.prototype.Reset = function () {
        this.step = 0;
        this.collide = 0;
        this.solve = 0;
        this.solveInit = 0;
        this.solveVelocity = 0;
        this.solvePosition = 0;
        this.broadphase = 0;
        this.solveTOI = 0;
        return this;
    };
    return b2Profile;
}());
exports.b2Profile = b2Profile;
/// This is an internal structure.
var b2TimeStep = /** @class */ (function () {
    function b2TimeStep() {
        this.dt = 0; // time step
        this.inv_dt = 0; // inverse time step (0 if dt == 0).
        this.dtRatio = 0; // dt * inv_dt0
        this.velocityIterations = 0;
        this.positionIterations = 0;
        // #if B2_ENABLE_PARTICLE
        this.particleIterations = 0;
        // #endif
        this.warmStarting = false;
    }
    b2TimeStep.prototype.Copy = function (step) {
        this.dt = step.dt;
        this.inv_dt = step.inv_dt;
        this.dtRatio = step.dtRatio;
        this.positionIterations = step.positionIterations;
        this.velocityIterations = step.velocityIterations;
        // #if B2_ENABLE_PARTICLE
        this.particleIterations = step.particleIterations;
        // #endif
        this.warmStarting = step.warmStarting;
        return this;
    };
    return b2TimeStep;
}());
exports.b2TimeStep = b2TimeStep;
var b2Position = /** @class */ (function () {
    function b2Position() {
        this.c = new b2Math_1.b2Vec2();
        this.a = 0;
    }
    b2Position.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2Position(); });
    };
    return b2Position;
}());
exports.b2Position = b2Position;
var b2Velocity = /** @class */ (function () {
    function b2Velocity() {
        this.v = new b2Math_1.b2Vec2();
        this.w = 0;
    }
    b2Velocity.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2Velocity(); });
    };
    return b2Velocity;
}());
exports.b2Velocity = b2Velocity;
var b2SolverData = /** @class */ (function () {
    function b2SolverData() {
        this.step = new b2TimeStep();
    }
    return b2SolverData;
}());
exports.b2SolverData = b2SolverData;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUaW1lU3RlcC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL2IyVGltZVN0ZXAudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLG1EQUFtRDtBQUNuRCwyQ0FBMEM7QUFFMUMsOENBQThDO0FBQzlDO0lBQUE7UUFDUyxTQUFJLEdBQVcsQ0FBQyxDQUFDO1FBQ2pCLFlBQU8sR0FBVyxDQUFDLENBQUM7UUFDcEIsVUFBSyxHQUFXLENBQUMsQ0FBQztRQUNsQixjQUFTLEdBQVcsQ0FBQyxDQUFDO1FBQ3RCLGtCQUFhLEdBQVcsQ0FBQyxDQUFDO1FBQzFCLGtCQUFhLEdBQVcsQ0FBQyxDQUFDO1FBQzFCLGVBQVUsR0FBVyxDQUFDLENBQUM7UUFDdkIsYUFBUSxHQUFXLENBQUMsQ0FBQztJQWE5QixDQUFDO0lBWFEseUJBQUssR0FBWjtRQUNFLElBQUksQ0FBQyxJQUFJLEdBQUcsQ0FBQyxDQUFDO1FBQ2QsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLENBQUM7UUFDakIsSUFBSSxDQUFDLEtBQUssR0FBRyxDQUFDLENBQUM7UUFDZixJQUFJLENBQUMsU0FBUyxHQUFHLENBQUMsQ0FBQztRQUNuQixJQUFJLENBQUMsYUFBYSxHQUFHLENBQUMsQ0FBQztRQUN2QixJQUFJLENBQUMsYUFBYSxHQUFHLENBQUMsQ0FBQztRQUN2QixJQUFJLENBQUMsVUFBVSxHQUFHLENBQUMsQ0FBQztRQUNwQixJQUFJLENBQUMsUUFBUSxHQUFHLENBQUMsQ0FBQztRQUNsQixPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFDSCxnQkFBQztBQUFELENBQUMsQUFyQkQsSUFxQkM7QUFyQlksOEJBQVM7QUF1QnRCLGtDQUFrQztBQUNsQztJQUFBO1FBQ1MsT0FBRSxHQUFXLENBQUMsQ0FBQyxDQUFDLFlBQVk7UUFDNUIsV0FBTSxHQUFXLENBQUMsQ0FBQyxDQUFDLG9DQUFvQztRQUN4RCxZQUFPLEdBQVcsQ0FBQyxDQUFDLENBQUMsZUFBZTtRQUNwQyx1QkFBa0IsR0FBVyxDQUFDLENBQUM7UUFDL0IsdUJBQWtCLEdBQVcsQ0FBQyxDQUFDO1FBQ3RDLHlCQUF5QjtRQUNsQix1QkFBa0IsR0FBVyxDQUFDLENBQUM7UUFDdEMsU0FBUztRQUNGLGlCQUFZLEdBQVksS0FBSyxDQUFDO0lBY3ZDLENBQUM7SUFaUSx5QkFBSSxHQUFYLFVBQVksSUFBZ0I7UUFDMUIsSUFBSSxDQUFDLEVBQUUsR0FBRyxJQUFJLENBQUMsRUFBRSxDQUFDO1FBQ2xCLElBQUksQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDLE1BQU0sQ0FBQztRQUMxQixJQUFJLENBQUMsT0FBTyxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUM7UUFDNUIsSUFBSSxDQUFDLGtCQUFrQixHQUFHLElBQUksQ0FBQyxrQkFBa0IsQ0FBQztRQUNsRCxJQUFJLENBQUMsa0JBQWtCLEdBQUcsSUFBSSxDQUFDLGtCQUFrQixDQUFDO1FBQ2xELHlCQUF5QjtRQUN6QixJQUFJLENBQUMsa0JBQWtCLEdBQUcsSUFBSSxDQUFDLGtCQUFrQixDQUFDO1FBQ2xELFNBQVM7UUFDVCxJQUFJLENBQUMsWUFBWSxHQUFHLElBQUksQ0FBQyxZQUFZLENBQUM7UUFDdEMsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0lBQ0gsaUJBQUM7QUFBRCxDQUFDLEFBdkJELElBdUJDO0FBdkJZLGdDQUFVO0FBeUJ2QjtJQUFBO1FBQ2tCLE1BQUMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2xDLE1BQUMsR0FBVyxDQUFDLENBQUM7SUFLdkIsQ0FBQztJQUhlLG9CQUFTLEdBQXZCLFVBQXdCLE1BQWM7UUFDcEMsT0FBTyx3QkFBVyxDQUFDLE1BQU0sRUFBRSxVQUFDLENBQVMsSUFBaUIsT0FBQSxJQUFJLFVBQVUsRUFBRSxFQUFoQixDQUFnQixDQUFDLENBQUM7SUFDMUUsQ0FBQztJQUNILGlCQUFDO0FBQUQsQ0FBQyxBQVBELElBT0M7QUFQWSxnQ0FBVTtBQVN2QjtJQUFBO1FBQ2tCLE1BQUMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2xDLE1BQUMsR0FBVyxDQUFDLENBQUM7SUFLdkIsQ0FBQztJQUhlLG9CQUFTLEdBQXZCLFVBQXdCLE1BQWM7UUFDcEMsT0FBTyx3QkFBVyxDQUFDLE1BQU0sRUFBRSxVQUFDLENBQVMsSUFBaUIsT0FBQSxJQUFJLFVBQVUsRUFBRSxFQUFoQixDQUFnQixDQUFDLENBQUM7SUFDMUUsQ0FBQztJQUNILGlCQUFDO0FBQUQsQ0FBQyxBQVBELElBT0M7QUFQWSxnQ0FBVTtBQVN2QjtJQUFBO1FBQ2tCLFNBQUksR0FBZSxJQUFJLFVBQVUsRUFBRSxDQUFDO0lBR3RELENBQUM7SUFBRCxtQkFBQztBQUFELENBQUMsQUFKRCxJQUlDO0FBSlksb0NBQVkifQ==