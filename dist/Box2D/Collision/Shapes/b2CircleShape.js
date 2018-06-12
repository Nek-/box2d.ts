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
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Shape_1 = require("./b2Shape");
/// A circle shape.
var b2CircleShape = /** @class */ (function (_super) {
    __extends(b2CircleShape, _super);
    function b2CircleShape(radius) {
        if (radius === void 0) { radius = 0; }
        var _this = _super.call(this, b2Shape_1.b2ShapeType.e_circleShape, radius) || this;
        _this.m_p = new b2Math_1.b2Vec2();
        return _this;
    }
    b2CircleShape.prototype.Set = function (position, radius) {
        if (radius === void 0) { radius = this.m_radius; }
        this.m_p.Copy(position);
        this.m_radius = radius;
        return this;
    };
    /// Implement b2Shape.
    b2CircleShape.prototype.Clone = function () {
        return new b2CircleShape().Copy(this);
    };
    b2CircleShape.prototype.Copy = function (other) {
        _super.prototype.Copy.call(this, other);
        // DEBUG: b2Assert(other instanceof b2CircleShape);
        this.m_p.Copy(other.m_p);
        return this;
    };
    /// @see b2Shape::GetChildCount
    b2CircleShape.prototype.GetChildCount = function () {
        return 1;
    };
    b2CircleShape.prototype.TestPoint = function (transform, p) {
        var center = b2Math_1.b2Transform.MulXV(transform, this.m_p, b2CircleShape.TestPoint_s_center);
        var d = b2Math_1.b2Vec2.SubVV(p, center, b2CircleShape.TestPoint_s_d);
        return b2Math_1.b2Vec2.DotVV(d, d) <= b2Math_1.b2Sq(this.m_radius);
    };
    b2CircleShape.prototype.ComputeDistance = function (xf, p, normal, childIndex) {
        var center = b2Math_1.b2Transform.MulXV(xf, this.m_p, b2CircleShape.ComputeDistance_s_center);
        b2Math_1.b2Vec2.SubVV(p, center, normal);
        return normal.Normalize() - this.m_radius;
    };
    b2CircleShape.prototype.RayCast = function (output, input, transform, childIndex) {
        var position = b2Math_1.b2Transform.MulXV(transform, this.m_p, b2CircleShape.RayCast_s_position);
        var s = b2Math_1.b2Vec2.SubVV(input.p1, position, b2CircleShape.RayCast_s_s);
        var b = b2Math_1.b2Vec2.DotVV(s, s) - b2Math_1.b2Sq(this.m_radius);
        // Solve quadratic equation.
        var r = b2Math_1.b2Vec2.SubVV(input.p2, input.p1, b2CircleShape.RayCast_s_r);
        var c = b2Math_1.b2Vec2.DotVV(s, r);
        var rr = b2Math_1.b2Vec2.DotVV(r, r);
        var sigma = c * c - rr * b;
        // Check for negative discriminant and short segment.
        if (sigma < 0 || rr < b2Settings_1.b2_epsilon) {
            return false;
        }
        // Find the point of intersection of the line with the circle.
        var a = (-(c + b2Math_1.b2Sqrt(sigma)));
        // Is the intersection point on the segment?
        if (0 <= a && a <= input.maxFraction * rr) {
            a /= rr;
            output.fraction = a;
            b2Math_1.b2Vec2.AddVMulSV(s, a, r, output.normal).SelfNormalize();
            return true;
        }
        return false;
    };
    b2CircleShape.prototype.ComputeAABB = function (aabb, transform, childIndex) {
        var p = b2Math_1.b2Transform.MulXV(transform, this.m_p, b2CircleShape.ComputeAABB_s_p);
        aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius);
        aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius);
    };
    /// @see b2Shape::ComputeMass
    b2CircleShape.prototype.ComputeMass = function (massData, density) {
        var radius_sq = b2Math_1.b2Sq(this.m_radius);
        massData.mass = density * b2Settings_1.b2_pi * radius_sq;
        massData.center.Copy(this.m_p);
        // inertia about the local origin
        massData.I = massData.mass * (0.5 * radius_sq + b2Math_1.b2Vec2.DotVV(this.m_p, this.m_p));
    };
    b2CircleShape.prototype.SetupDistanceProxy = function (proxy, index) {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_p);
        proxy.m_count = 1;
        proxy.m_radius = this.m_radius;
    };
    b2CircleShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
        var p = b2Math_1.b2Transform.MulXV(xf, this.m_p, new b2Math_1.b2Vec2());
        var l = (-(b2Math_1.b2Vec2.DotVV(normal, p) - offset));
        if (l < (-this.m_radius) + b2Settings_1.b2_epsilon) {
            // Completely dry
            return 0;
        }
        if (l > this.m_radius) {
            // Completely wet
            c.Copy(p);
            return b2Settings_1.b2_pi * this.m_radius * this.m_radius;
        }
        // Magic
        var r2 = this.m_radius * this.m_radius;
        var l2 = l * l;
        var area = r2 * (b2Math_1.b2Asin(l / this.m_radius) + b2Settings_1.b2_pi / 2) + l * b2Math_1.b2Sqrt(r2 - l2);
        var com = (-2 / 3 * b2Math_1.b2Pow(r2 - l2, 1.5) / area);
        c.x = p.x + normal.x * com;
        c.y = p.y + normal.y * com;
        return area;
    };
    b2CircleShape.prototype.Dump = function (log) {
        log("    const shape: b2CircleShape = new b2CircleShape();\n");
        log("    shape.m_radius = %.15f;\n", this.m_radius);
        log("    shape.m_p.Set(%.15f, %.15f);\n", this.m_p.x, this.m_p.y);
    };
    /// Implement b2Shape.
    b2CircleShape.TestPoint_s_center = new b2Math_1.b2Vec2();
    b2CircleShape.TestPoint_s_d = new b2Math_1.b2Vec2();
    // #if B2_ENABLE_PARTICLE
    /// @see b2Shape::ComputeDistance
    b2CircleShape.ComputeDistance_s_center = new b2Math_1.b2Vec2();
    // #endif
    /// Implement b2Shape.
    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.1.2
    // x = s + a * r
    // norm(x) = radius
    b2CircleShape.RayCast_s_position = new b2Math_1.b2Vec2();
    b2CircleShape.RayCast_s_s = new b2Math_1.b2Vec2();
    b2CircleShape.RayCast_s_r = new b2Math_1.b2Vec2();
    /// @see b2Shape::ComputeAABB
    b2CircleShape.ComputeAABB_s_p = new b2Math_1.b2Vec2();
    return b2CircleShape;
}(b2Shape_1.b2Shape));
exports.b2CircleShape = b2CircleShape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDaXJjbGVTaGFwZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9TaGFwZXMvYjJDaXJjbGVTaGFwZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLDZEQUE2RDtBQUM3RCxzREFBNEQ7QUFDNUQsOENBQTJGO0FBSTNGLHFDQUFpRDtBQUVqRCxtQkFBbUI7QUFDbkI7SUFBbUMsaUNBQU87SUFHeEMsdUJBQVksTUFBa0I7UUFBbEIsdUJBQUEsRUFBQSxVQUFrQjtRQUE5QixZQUNFLGtCQUFNLHFCQUFXLENBQUMsYUFBYSxFQUFFLE1BQU0sQ0FBQyxTQUN6QztRQUplLFNBQUcsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDOztJQUkzQyxDQUFDO0lBRU0sMkJBQUcsR0FBVixVQUFXLFFBQVksRUFBRSxNQUE4QjtRQUE5Qix1QkFBQSxFQUFBLFNBQWlCLElBQUksQ0FBQyxRQUFRO1FBQ3JELElBQUksQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDO1FBQ3hCLElBQUksQ0FBQyxRQUFRLEdBQUcsTUFBTSxDQUFDO1FBQ3ZCLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQUVELHNCQUFzQjtJQUNmLDZCQUFLLEdBQVo7UUFDRSxPQUFPLElBQUksYUFBYSxFQUFFLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO0lBQ3hDLENBQUM7SUFFTSw0QkFBSSxHQUFYLFVBQVksS0FBb0I7UUFDOUIsaUJBQU0sSUFBSSxZQUFDLEtBQUssQ0FBQyxDQUFDO1FBRWxCLG1EQUFtRDtRQUVuRCxJQUFJLENBQUMsR0FBRyxDQUFDLElBQUksQ0FBQyxLQUFLLENBQUMsR0FBRyxDQUFDLENBQUM7UUFDekIsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0lBRUQsK0JBQStCO0lBQ3hCLHFDQUFhLEdBQXBCO1FB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