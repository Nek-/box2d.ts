"use strict";
/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
// #if B2_ENABLE_CONTROLLER
var b2Controller_1 = require("./b2Controller");
var b2Math_1 = require("../Common/b2Math");
var b2Settings_1 = require("../Common/b2Settings");
/**
 * Applies top down linear damping to the controlled bodies
 * The damping is calculated by multiplying velocity by a matrix
 * in local co-ordinates.
 */
var b2TensorDampingController = /** @class */ (function (_super) {
    __extends(b2TensorDampingController, _super);
    function b2TensorDampingController() {
        var _this = _super !== null && _super.apply(this, arguments) || this;
        /// Tensor to use in damping model
        _this.T = new b2Math_1.b2Mat22();
        /*Some examples (matrixes in format (row1; row2))
        (-a 0; 0 -a)    Standard isotropic damping with strength a
        ( 0 a; -a 0)    Electron in fixed field - a force at right angles to velocity with proportional magnitude
        (-a 0; 0 -b)    Differing x and y damping. Useful e.g. for top-down wheels.
        */
        //By the way, tensor in this case just means matrix, don't let the terminology get you down.
        /// Set this to a positive number to clamp the maximum amount of damping done.
        _this.maxTimestep = 0;
        return _this;
    }
    // Typically one wants maxTimestep to be 1/(max eigenvalue of T), so that damping will never cause something to reverse direction
    /**
     * @see b2Controller::Step
     */
    b2TensorDampingController.prototype.Step = function (step) {
        var timestep = step.dt;
        if (timestep <= b2Settings_1.b2_epsilon) {
            return;
        }
        if (timestep > this.maxTimestep && this.maxTimestep > 0) {
            timestep = this.maxTimestep;
        }
        for (var i = this.m_bodyList; i; i = i.nextBody) {
            var body = i.body;
            if (!body.IsAwake()) {
                continue;
            }
            var damping = body.GetWorldVector(b2Math_1.b2Mat22.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity(), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t1), b2TensorDampingController.Step_s_damping);
            //    body->SetLinearVelocity(body->GetLinearVelocity() + timestep * damping);
            body.SetLinearVelocity(b2Math_1.b2Vec2.AddVV(body.GetLinearVelocity(), b2Math_1.b2Vec2.MulSV(timestep, damping, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t1));
        }
    };
    b2TensorDampingController.prototype.Draw = function (draw) { };
    /**
     * Sets damping independantly along the x and y axes
     */
    b2TensorDampingController.prototype.SetAxisAligned = function (xDamping, yDamping) {
        this.T.ex.x = (-xDamping);
        this.T.ex.y = 0;
        this.T.ey.x = 0;
        this.T.ey.y = (-yDamping);
        if (xDamping > 0 || yDamping > 0) {
            this.maxTimestep = 1 / b2Math_1.b2Max(xDamping, yDamping);
        }
        else {
            this.maxTimestep = 0;
        }
    };
    b2TensorDampingController.Step_s_damping = new b2Math_1.b2Vec2();
    return b2TensorDampingController;
}(b2Controller_1.b2Controller));
exports.b2TensorDampingController = b2TensorDampingController;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUZW5zb3JEYW1waW5nQ29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbnRyb2xsZXJzL2IyVGVuc29yRGFtcGluZ0NvbnRyb2xsZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOzs7Ozs7Ozs7Ozs7QUFFSCwyQkFBMkI7QUFFM0IsK0NBQThDO0FBQzlDLDJDQUEwRDtBQUUxRCxtREFBa0Q7QUFHbEQ7Ozs7R0FJRztBQUNIO0lBQStDLDZDQUFZO0lBQTNEO1FBQUEscUVBNERDO1FBM0RHLGtDQUFrQztRQUNsQixPQUFDLEdBQUcsSUFBSSxnQkFBTyxFQUFFLENBQUM7UUFDbEM7Ozs7VUFJRTtRQUNGLDRGQUE0RjtRQUU1Riw4RUFBOEU7UUFDdkUsaUJBQVcsR0FBRyxDQUFDLENBQUM7O0lBaUQzQixDQUFDO0lBaERHLGlJQUFpSTtJQUVqSTs7T0FFRztJQUNJLHdDQUFJLEdBQVgsVUFBWSxJQUFnQjtRQUN4QixJQUFJLFFBQVEsR0FBRyxJQUFJLENBQUMsRUFBRSxDQUFDO1FBQ3ZCLElBQUksUUFBUSxJQUFJLHVCQUFVLEVBQUU7WUFDeEIsT0FBTztTQUNWO1FBQ0QsSUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLFdBQVcsSUFBSSxJQUFJLENBQUMsV0FBVyxHQUFHLENBQUMsRUFBRTtZQUNyRCxRQUFRLEdBQUcsSUFBSSxDQUFDLFdBQVcsQ0FBQztTQUMvQjtRQUNELEtBQUssSUFBSSxDQUFDLEdBQUcsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7WUFDN0MsSUFBTSxJQUFJLEdBQUcsQ0FBQyxDQUFDLElBQUksQ0FBQztZQUNwQixJQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sRUFBRSxFQUFFO2dCQUNqQixTQUFTO2FBQ1o7WUFDRCxJQUFNLE9BQU8sR0FBRyxJQUFJLENBQUMsY0FBYyxDQUNuQyxnQkFBTyxDQUFDLEtBQUssQ0FDVCxJQUFJLENBQUMsQ0FBQyxFQUNOLElBQUksQ0FBQyxjQUFjLENBQ25CLElBQUksQ0FBQyxpQkFBaUIsRUFBRSxFQUN4QixlQUFNLENBQUMsSUFBSSxDQUFDLEVBQ1osZUFBTSxDQUFDLElBQUksQ0FBQyxFQUNoQix5QkFBeUIsQ0FBQyxjQUFjLENBQUMsQ0FBQztZQUMxQyw4RUFBOEU7WUFDOUUsSUFBSSxDQUFDLGlCQUFpQixDQUFDLGVBQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLGlCQUFpQixFQUFFLEVBQUUsZUFBTSxDQUFDLEtBQUssQ0FBQyxRQUFRLEVBQUUsT0FBTyxFQUFFLGVBQU0sQ0FBQyxJQUFJLENBQUMsRUFBRSxlQUFNLENBQUMsSUFBSSxDQUFDLENBQUMsQ0FBQztTQUM3SDtJQUNMLENBQUM7SUFHTSx3Q0FBSSxHQUFYLFVBQVksSUFBWSxJQUFHLENBQUM7SUFFNUI7O09BRUc7SUFDSSxrREFBYyxHQUFyQixVQUFzQixRQUFnQixFQUFFLFFBQWdCO1FBQ3RELElBQUksQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxDQUFDLENBQUM7UUFDMUIsSUFBSSxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUNoQixJQUFJLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQ2hCLElBQUksQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxDQUFDLENBQUM7UUFDMUIsSUFBSSxRQUFRLEdBQUcsQ0FBQyxJQUFJLFFBQVEsR0FBRyxDQUFDLEVBQUU7WUFDaEMsSUFBSSxDQUFDLFdBQVcsR0FBRyxDQUFDLEdBQUcsY0FBSyxDQUFDLFFBQVEsRUFBRSxRQUFRLENBQUMsQ0FBQztTQUNsRDthQUFNO1lBQ0wsSUFBSSxDQUFDLFdBQVcsR0FBRyxDQUFDLENBQUM7U0FDdEI7SUFDSCxDQUFDO0lBakJjLHdDQUFjLEdBQUcsSUFBSSxlQUFNLEVBQUUsQ0FBQztJQWtCakQsZ0NBQUM7Q0FBQSxBQTVERCxDQUErQywyQkFBWSxHQTREMUQ7QUE1RFksOERBQXlCO0FBOER0QyxTQUFTIn0=