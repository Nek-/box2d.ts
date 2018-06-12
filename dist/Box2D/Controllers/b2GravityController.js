"use strict";
/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
/**
 * Applies simplified gravity between every pair of bodies
 */
var b2GravityController = /** @class */ (function (_super) {
    __extends(b2GravityController, _super);
    function b2GravityController() {
        var _this = _super !== null && _super.apply(this, arguments) || this;
        /**
         * Specifies the strength of the gravitiation force
         */
        _this.G = 1;
        /**
         * If true, gravity is proportional to r^-2, otherwise r^-1
         */
        _this.invSqr = true;
        return _this;
    }
    /**
     * @see b2Controller::Step
     */
    b2GravityController.prototype.Step = function (step) {
        if (this.invSqr) {
            for (var i = this.m_bodyList; i; i = i.nextBody) {
                var body1 = i.body;
                var p1 = body1.GetWorldCenter();
                var mass1 = body1.GetMass();
                for (var j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    var body2 = j.body;
                    var p2 = body2.GetWorldCenter();
                    var mass2 = body2.GetMass();
                    var dx = p2.x - p1.x;
                    var dy = p2.y - p1.y;
                    var r2 = dx * dx + dy * dy;
                    if (r2 < b2Settings_1.b2_epsilon) {
                        continue;
                    }
                    var f = b2GravityController.Step_s_f.Set(dx, dy);
                    f.SelfMul(this.G / r2 / b2Math_1.b2Sqrt(r2) * mass1 * mass2);
                    if (body1.IsAwake()) {
                        body1.ApplyForce(f, p1);
                    }
                    if (body2.IsAwake()) {
                        body2.ApplyForce(f.SelfMul(-1), p2);
                    }
                }
            }
        }
        else {
            for (var i = this.m_bodyList; i; i = i.nextBody) {
                var body1 = i.body;
                var p1 = body1.GetWorldCenter();
                var mass1 = body1.GetMass();
                for (var j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    var body2 = j.body;
                    var p2 = body2.GetWorldCenter();
                    var mass2 = body2.GetMass();
                    var dx = p2.x - p1.x;
                    var dy = p2.y - p1.y;
                    var r2 = dx * dx + dy * dy;
                    if (r2 < b2Settings_1.b2_epsilon) {
                        continue;
                    }
                    var f = b2GravityController.Step_s_f.Set(dx, dy);
                    f.SelfMul(this.G / r2 * mass1 * mass2);
                    if (body1.IsAwake()) {
                        body1.ApplyForce(f, p1);
                    }
                    if (body2.IsAwake()) {
                        body2.ApplyForce(f.SelfMul(-1), p2);
                    }
                }
            }
        }
    };
    b2GravityController.prototype.Draw = function (draw) { };
    b2GravityController.Step_s_f = new b2Math_1.b2Vec2();
    return b2GravityController;
}(b2Controller_1.b2Controller));
exports.b2GravityController = b2GravityController;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHcmF2aXR5Q29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbnRyb2xsZXJzL2IyR3Jhdml0eUNvbnRyb2xsZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOzs7Ozs7Ozs7Ozs7QUFFSCwyQkFBMkI7QUFFM0IsK0NBQThDO0FBRTlDLG1EQUFrRDtBQUNsRCwyQ0FBa0Q7QUFHbEQ7O0dBRUc7QUFDSDtJQUF5Qyx1Q0FBWTtJQUFyRDtRQUFBLHFFQXFFQztRQXBFQzs7V0FFRztRQUNJLE9BQUMsR0FBRyxDQUFDLENBQUM7UUFDYjs7V0FFRztRQUNJLFlBQU0sR0FBRyxJQUFJLENBQUM7O0lBNkR2QixDQUFDO0lBM0RDOztPQUVHO0lBQ0ksa0NBQUksR0FBWCxVQUFZLElBQWdCO1FBQzFCLElBQUksSUFBSSxDQUFDLE1BQU0sRUFBRTtZQUNmLEtBQUssSUFBSSxDQUFDLEdBQUcsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7Z0JBQy9DLElBQU0sS0FBSyxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7Z0JBQ3JCLElBQU0sRUFBRSxHQUFHLEtBQUssQ0FBQyxjQUFjLEVBQUUsQ0FBQztnQkFDbEMsSUFBTSxLQUFLLEdBQUcsS0FBSyxDQUFDLE9BQU8sRUFBRSxDQUFDO2dCQUM5QixLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxJQUFJLENBQUMsS0FBSyxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7b0JBQzFELElBQU0sS0FBSyxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7b0JBQ3JCLElBQU0sRUFBRSxHQUFHLEtBQUssQ0FBQyxjQUFjLEVBQUUsQ0FBQztvQkFDbEMsSUFBTSxLQUFLLEdBQUcsS0FBSyxDQUFDLE9BQU8sRUFBRSxDQUFDO29CQUM5QixJQUFNLEVBQUUsR0FBRyxFQUFFLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQyxDQUFDLENBQUM7b0JBQ3ZCLElBQU0sRUFBRSxHQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDLENBQUMsQ0FBQztvQkFDdkIsSUFBTSxFQUFFLEdBQUcsRUFBRSxHQUFHLEVBQUUsR0FBRyxFQUFFLEdBQUcsRUFBRSxDQUFDO29CQUM3QixJQUFJLEVBQUUsR0FBRyx1QkFBVSxFQUFFO3dCQUNuQixTQUFTO3FCQUNWO29CQUNELElBQU0sQ0FBQyxHQUFHLG1CQUFtQixDQUFDLFFBQVEsQ0FBQyxHQUFHLENBQUMsRUFBRSxFQUFFLEVBQUUsQ0FBQyxDQUFDO29CQUNuRCxDQUFDLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxDQUFDLEdBQUcsRUFBRSxHQUFHLGVBQU0sQ0FBQyxFQUFFLENBQUMsR0FBRyxLQUFLLEdBQUcsS0FBSyxDQUFDLENBQUM7b0JBQ3BELElBQUksS0FBSyxDQUFDLE9BQU8sRUFBRSxFQUFFO3dCQUNuQixLQUFLLENBQUMsVUFBVSxDQUFDLENBQUMsRUFBRSxFQUFFLENBQUMsQ0FBQztxQkFDekI7b0JBQ0QsSUFBSSxLQUFLLENBQUMsT0FBTyxFQUFFLEVBQUU7d0JBQ25CLEtBQUssQ0FBQyxVQUFVLENBQUMsQ0FBQyxDQUFDLE9BQU8sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDO3FCQUNyQztpQkFDRjthQUNGO1NBQ0Y7YUFBTTtZQUNMLEtBQUssSUFBSSxDQUFDLEdBQUcsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7Z0JBQy9DLElBQU0sS0FBSyxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7Z0JBQ3JCLElBQU0sRUFBRSxHQUFHLEtBQUssQ0FBQyxjQUFjLEVBQUUsQ0FBQztnQkFDbEMsSUFBTSxLQUFLLEdBQUcsS0FBSyxDQUFDLE9BQU8sRUFBRSxDQUFDO2dCQUM5QixLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxJQUFJLENBQUMsS0FBSyxDQUFDLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQyxRQUFRLEVBQUU7b0JBQzFELElBQU0sS0FBSyxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7b0JBQ3JCLElBQU0sRUFBRSxHQUFHLEtBQUssQ0FBQyxjQUFjLEVBQUUsQ0FBQztvQkFDbEMsSUFBTSxLQUFLLEdBQUcsS0FBSyxDQUFDLE9BQU8sRUFBRSxDQUFDO29CQUM5QixJQUFNLEVBQUUsR0FBRyxFQUFFLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQyxDQUFDLENBQUM7b0JBQ3ZCLElBQU0sRUFBRSxHQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDLENBQUMsQ0FBQztvQkFDdkIsSUFBTSxFQUFFLEdBQUcsRUFBRSxHQUFHLEVBQUUsR0FBRyxFQUFFLEdBQUcsRUFBRSxDQUFDO29CQUM3QixJQUFJLEVBQUUsR0FBRyx1QkFBVSxFQUFFO3dCQUNuQixTQUFTO3FCQUNWO29CQUNELElBQU0sQ0FBQyxHQUFHLG1CQUFtQixDQUFDLFFBQVEsQ0FBQyxHQUFHLENBQUMsRUFBRSxFQUFFLEVBQUUsQ0FBQyxDQUFDO29CQUNuRCxDQUFDLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxDQUFDLEdBQUcsRUFBRSxHQUFHLEtBQUssR0FBRyxLQUFLLENBQUMsQ0FBQztvQkFDdkMsSUFBSSxLQUFLLENBQUMsT0FBTyxFQUFFLEVBQUU7d0JBQ25CLEtBQUssQ0FBQyxVQUFVLENBQUMsQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDO3FCQUN6QjtvQkFDRCxJQUFJLEtBQUssQ0FBQyxPQUFPLEVBQUUsRUFBRTt3QkFDbkIsS0FBSyxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUMsT0FBTyxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsRUFBRSxDQUFDLENBQUM7cUJBQ3JDO2lCQUNGO2FBQ0Y7U0FDRjtJQUNILENBQUM7SUFHTSxrQ0FBSSxHQUFYLFVBQVksSUFBWSxJQUFJLENBQUM7SUFGZCw0QkFBUSxHQUFHLElBQUksZUFBTSxFQUFFLENBQUM7SUFHekMsMEJBQUM7Q0FBQSxBQXJFRCxDQUF5QywyQkFBWSxHQXFFcEQ7QUFyRVksa0RBQW1CO0FBdUVoQyxTQUFTIn0=