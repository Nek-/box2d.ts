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
var b2Math_1 = require("../Common/b2Math");
/**
 * Applies a force every frame
 */
var b2ConstantForceController = /** @class */ (function (_super) {
    __extends(b2ConstantForceController, _super);
    function b2ConstantForceController() {
        var _this = _super !== null && _super.apply(this, arguments) || this;
        /**
         * The force to apply
         */
        _this.F = new b2Math_1.b2Vec2(0, 0);
        return _this;
    }
    b2ConstantForceController.prototype.Step = function (step) {
        for (var i = this.m_bodyList; i; i = i.nextBody) {
            var body = i.body;
            if (!body.IsAwake()) {
                continue;
            }
            body.ApplyForce(this.F, body.GetWorldCenter());
        }
    };
    b2ConstantForceController.prototype.Draw = function (draw) { };
    return b2ConstantForceController;
}(b2Controller_1.b2Controller));
exports.b2ConstantForceController = b2ConstantForceController;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb25zdGFudEZvcmNlQ29udHJvbGxlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbnRyb2xsZXJzL2IyQ29uc3RhbnRGb3JjZUNvbnRyb2xsZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOzs7Ozs7Ozs7Ozs7QUFFSCwyQkFBMkI7QUFFM0IsK0NBQThDO0FBQzlDLDJDQUEwQztBQUkxQzs7R0FFRztBQUNIO0lBQStDLDZDQUFZO0lBQTNEO1FBQUEscUVBaUJDO1FBaEJDOztXQUVHO1FBQ2EsT0FBQyxHQUFHLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQzs7SUFhdkMsQ0FBQztJQVhRLHdDQUFJLEdBQVgsVUFBWSxJQUFnQjtRQUMxQixLQUFLLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxFQUFFLENBQUMsR0FBRyxDQUFDLENBQUMsUUFBUSxFQUFFO1lBQy9DLElBQU0sSUFBSSxHQUFHLENBQUMsQ0FBQyxJQUFJLENBQUM7WUFDcEIsSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLEVBQUUsRUFBRTtnQkFDbkIsU0FBUzthQUNWO1lBQ0QsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsQ0FBQyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsQ0FBQyxDQUFDO1NBQ2hEO0lBQ0gsQ0FBQztJQUVNLHdDQUFJLEdBQVgsVUFBWSxJQUFZLElBQUcsQ0FBQztJQUM5QixnQ0FBQztBQUFELENBQUMsQUFqQkQsQ0FBK0MsMkJBQVksR0FpQjFEO0FBakJZLDhEQUF5QjtBQW1CdEMsU0FBUyJ9