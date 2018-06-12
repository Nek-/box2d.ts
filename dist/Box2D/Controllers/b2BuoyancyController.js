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
var b2Settings_1 = require("../Common/b2Settings");
var b2Draw_1 = require("../Common/b2Draw");
/**
 * Calculates buoyancy forces for fluids in the form of a half
 * plane.
 */
var b2BuoyancyController = /** @class */ (function (_super) {
    __extends(b2BuoyancyController, _super);
    function b2BuoyancyController() {
        var _this = _super !== null && _super.apply(this, arguments) || this;
        /**
         * The outer surface normal
         */
        _this.normal = new b2Math_1.b2Vec2(0, 1);
        /**
         * The height of the fluid surface along the normal
         */
        _this.offset = 0;
        /**
         * The fluid density
         */
        _this.density = 0;
        /**
         * Fluid velocity, for drag calculations
         */
        _this.velocity = new b2Math_1.b2Vec2(0, 0);
        /**
         * Linear drag co-efficient
         */
        _this.linearDrag = 0;
        /**
         * Angular drag co-efficient
         */
        _this.angularDrag = 0;
        /**
         * If false, bodies are assumed to be uniformly dense, otherwise
         * use the shapes densities
         */
        _this.useDensity = false; //False by default to prevent a gotcha
        /**
         * If true, gravity is taken from the world instead of the
         */
        _this.useWorldGravity = true;
        /**
         * Gravity vector, if the world's gravity is not used
         */
        _this.gravity = new b2Math_1.b2Vec2(0, 0);
        return _this;
    }
    b2BuoyancyController.prototype.Step = function (step) {
        if (!this.m_bodyList) {
            return;
        }
        if (this.useWorldGravity) {
            this.gravity.Copy(this.m_bodyList.body.GetWorld().GetGravity());
        }
        for (var i = this.m_bodyList; i; i = i.nextBody) {
            var body = i.body;
            if (!body.IsAwake()) {
                //Buoyancy force is just a function of position,
                //so unlike most forces, it is safe to ignore sleeping bodes
                continue;
            }
            var areac = new b2Math_1.b2Vec2();
            var massc = new b2Math_1.b2Vec2();
            var area = 0;
            var mass = 0;
            for (var fixture = body.GetFixtureList(); fixture; fixture = fixture.m_next) {
                var sc = new b2Math_1.b2Vec2();
                var sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
                area += sarea;
                areac.x += sarea * sc.x;
                areac.y += sarea * sc.y;
                var shapeDensity = 0;
                if (this.useDensity) {
                    //TODO: Expose density publicly
                    shapeDensity = fixture.GetDensity();
                }
                else {
                    shapeDensity = 1;
                }
                mass += sarea * shapeDensity;
                massc.x += sarea * sc.x * shapeDensity;
                massc.y += sarea * sc.y * shapeDensity;
            }
            areac.x /= area;
            areac.y /= area;
            //    b2Vec2 localCentroid = b2MulT(body->GetXForm(),areac);
            massc.x /= mass;
            massc.y /= mass;
            if (area < b2Settings_1.b2_epsilon) {
                continue;
            }
            //Buoyancy
            var buoyancyForce = this.gravity.Clone().SelfNeg();
            buoyancyForce.SelfMul(this.density * area);
            body.ApplyForce(buoyancyForce, massc);
            //Linear drag
            var dragForce = body.GetLinearVelocityFromWorldPoint(areac, new b2Math_1.b2Vec2());
            dragForce.SelfSub(this.velocity);
            dragForce.SelfMul((-this.linearDrag * area));
            body.ApplyForce(dragForce, areac);
            //Angular drag
            //TODO: Something that makes more physical sense?
            body.ApplyTorque((-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag));
        }
    };
    b2BuoyancyController.prototype.Draw = function (debugDraw) {
        var r = 100;
        var p1 = new b2Math_1.b2Vec2();
        var p2 = new b2Math_1.b2Vec2();
        p1.x = this.normal.x * this.offset + this.normal.y * r;
        p1.y = this.normal.y * this.offset - this.normal.x * r;
        p2.x = this.normal.x * this.offset - this.normal.y * r;
        p2.y = this.normal.y * this.offset + this.normal.x * r;
        var color = new b2Draw_1.b2Color(0, 0, 0.8);
        debugDraw.DrawSegment(p1, p2, color);
    };
    return b2BuoyancyController;
}(b2Controller_1.b2Controller));
exports.b2BuoyancyController = b2BuoyancyController;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCdW95YW5jeUNvbnRyb2xsZXIuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9Cb3gyRC9Cb3gyRC9Db250cm9sbGVycy9iMkJ1b3lhbmN5Q29udHJvbGxlci50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7Ozs7Ozs7Ozs7OztBQUVILDJCQUEyQjtBQUUzQiwrQ0FBZ0U7QUFDaEUsMkNBQTBDO0FBRTFDLG1EQUFrRDtBQUNsRCwyQ0FBbUQ7QUFFbkQ7OztHQUdHO0FBQ0g7SUFBMEMsd0NBQVk7SUFBdEQ7UUFBQSxxRUE4R0M7UUE3R0M7O1dBRUc7UUFDYSxZQUFNLEdBQUcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQzFDOztXQUVHO1FBQ0ksWUFBTSxHQUFHLENBQUMsQ0FBQztRQUNsQjs7V0FFRztRQUNJLGFBQU8sR0FBRyxDQUFDLENBQUM7UUFDbkI7O1dBRUc7UUFDYSxjQUFRLEdBQUcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQzVDOztXQUVHO1FBQ0ksZ0JBQVUsR0FBRyxDQUFDLENBQUM7UUFDdEI7O1dBRUc7UUFDSSxpQkFBVyxHQUFHLENBQUMsQ0FBQztRQUN2Qjs7O1dBR0c7UUFDSSxnQkFBVSxHQUFHLEtBQUssQ0FBQyxDQUFDLHNDQUFzQztRQUNqRTs7V0FFRztRQUNJLHFCQUFlLEdBQUcsSUFBSSxDQUFDO1FBQzlCOztXQUVHO1FBQ2EsYUFBTyxHQUFHLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQzs7SUF5RTdDLENBQUM7SUF2RVEsbUNBQUksR0FBWCxVQUFZLElBQWdCO1FBQzFCLElBQUksQ0FBQyxJQUFJLENBQUMsVUFBVSxFQUFFO1lBQ3BCLE9BQU87U0FDUjtRQUNELElBQUksSUFBSSxDQUFDLGVBQWUsRUFBRTtZQUN4QixJQUFJLENBQUMsT0FBTyxDQUFDLElBQUks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