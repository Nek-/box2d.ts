"use strict";
/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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
/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
var b2EdgeShape = /** @class */ (function (_super) {
    __extends(b2EdgeShape, _super);
    function b2EdgeShape() {
        var _this = _super.call(this, b2Shape_1.b2ShapeType.e_edgeShape, b2Settings_1.b2_polygonRadius) || this;
        _this.m_vertex1 = new b2Math_1.b2Vec2();
        _this.m_vertex2 = new b2Math_1.b2Vec2();
        _this.m_vertex0 = new b2Math_1.b2Vec2();
        _this.m_vertex3 = new b2Math_1.b2Vec2();
        _this.m_hasVertex0 = false;
        _this.m_hasVertex3 = false;
        return _this;
    }
    /// Set this as an isolated edge.
    b2EdgeShape.prototype.Set = function (v1, v2) {
        this.m_vertex1.Copy(v1);
        this.m_vertex2.Copy(v2);
        this.m_hasVertex0 = false;
        this.m_hasVertex3 = false;
        return this;
    };
    /// Implement b2Shape.
    b2EdgeShape.prototype.Clone = function () {
        return new b2EdgeShape().Copy(this);
    };
    b2EdgeShape.prototype.Copy = function (other) {
        _super.prototype.Copy.call(this, other);
        // DEBUG: b2Assert(other instanceof b2EdgeShape);
        this.m_vertex1.Copy(other.m_vertex1);
        this.m_vertex2.Copy(other.m_vertex2);
        this.m_vertex0.Copy(other.m_vertex0);
        this.m_vertex3.Copy(other.m_vertex3);
        this.m_hasVertex0 = other.m_hasVertex0;
        this.m_hasVertex3 = other.m_hasVertex3;
        return this;
    };
    /// @see b2Shape::GetChildCount
    b2EdgeShape.prototype.GetChildCount = function () {
        return 1;
    };
    /// @see b2Shape::TestPoint
    b2EdgeShape.prototype.TestPoint = function (xf, p) {
        return false;
    };
    b2EdgeShape.prototype.ComputeDistance = function (xf, p, normal, childIndex) {
        var v1 = b2Math_1.b2Transform.MulXV(xf, this.m_vertex1, b2EdgeShape.ComputeDistance_s_v1);
        var v2 = b2Math_1.b2Transform.MulXV(xf, this.m_vertex2, b2EdgeShape.ComputeDistance_s_v2);
        var d = b2Math_1.b2Vec2.SubVV(p, v1, b2EdgeShape.ComputeDistance_s_d);
        var s = b2Math_1.b2Vec2.SubVV(v2, v1, b2EdgeShape.ComputeDistance_s_s);
        var ds = b2Math_1.b2Vec2.DotVV(d, s);
        if (ds > 0) {
            var s2 = b2Math_1.b2Vec2.DotVV(s, s);
            if (ds > s2) {
                b2Math_1.b2Vec2.SubVV(p, v2, d);
            }
            else {
                d.SelfMulSub(ds / s2, s);
            }
        }
        normal.Copy(d);
        return normal.Normalize();
    };
    b2EdgeShape.prototype.RayCast = function (output, input, xf, childIndex) {
        // Put the ray into the edge's frame of reference.
        var p1 = b2Math_1.b2Transform.MulTXV(xf, input.p1, b2EdgeShape.RayCast_s_p1);
        var p2 = b2Math_1.b2Transform.MulTXV(xf, input.p2, b2EdgeShape.RayCast_s_p2);
        var d = b2Math_1.b2Vec2.SubVV(p2, p1, b2EdgeShape.RayCast_s_d);
        var v1 = this.m_vertex1;
        var v2 = this.m_vertex2;
        var e = b2Math_1.b2Vec2.SubVV(v2, v1, b2EdgeShape.RayCast_s_e);
        var normal = output.normal.Set(e.y, -e.x).SelfNormalize();
        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        var numerator = b2Math_1.b2Vec2.DotVV(normal, b2Math_1.b2Vec2.SubVV(v1, p1, b2Math_1.b2Vec2.s_t0));
        var denominator = b2Math_1.b2Vec2.DotVV(normal, d);
        if (denominator === 0) {
            return false;
        }
        var t = numerator / denominator;
        if (t < 0 || input.maxFraction < t) {
            return false;
        }
        var q = b2Math_1.b2Vec2.AddVMulSV(p1, t, d, b2EdgeShape.RayCast_s_q);
        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        var r = b2Math_1.b2Vec2.SubVV(v2, v1, b2EdgeShape.RayCast_s_r);
        var rr = b2Math_1.b2Vec2.DotVV(r, r);
        if (rr === 0) {
            return false;
        }
        var s = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(q, v1, b2Math_1.b2Vec2.s_t0), r) / rr;
        if (s < 0 || 1 < s) {
            return false;
        }
        output.fraction = t;
        b2Math_1.b2Rot.MulRV(xf.q, output.normal, output.normal);
        if (numerator > 0) {
            output.normal.SelfNeg();
        }
        return true;
    };
    b2EdgeShape.prototype.ComputeAABB = function (aabb, xf, childIndex) {
        var v1 = b2Math_1.b2Transform.MulXV(xf, this.m_vertex1, b2EdgeShape.ComputeAABB_s_v1);
        var v2 = b2Math_1.b2Transform.MulXV(xf, this.m_vertex2, b2EdgeShape.ComputeAABB_s_v2);
        b2Math_1.b2Vec2.MinV(v1, v2, aabb.lowerBound);
        b2Math_1.b2Vec2.MaxV(v1, v2, aabb.upperBound);
        var r = this.m_radius;
        aabb.lowerBound.SelfSubXY(r, r);
        aabb.upperBound.SelfAddXY(r, r);
    };
    /// @see b2Shape::ComputeMass
    b2EdgeShape.prototype.ComputeMass = function (massData, density) {
        massData.mass = 0;
        b2Math_1.b2Vec2.MidVV(this.m_vertex1, this.m_vertex2, massData.center);
        massData.I = 0;
    };
    b2EdgeShape.prototype.SetupDistanceProxy = function (proxy, index) {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_vertex1);
        proxy.m_vertices[1].Copy(this.m_vertex2);
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    };
    b2EdgeShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
        c.SetZero();
        return 0;
    };
    b2EdgeShape.prototype.Dump = function (log) {
        log("    const shape: b2EdgeShape = new b2EdgeShape();\n");
        log("    shape.m_radius = %.15f;\n", this.m_radius);
        log("    shape.m_vertex0.Set(%.15f, %.15f);\n", this.m_vertex0.x, this.m_vertex0.y);
        log("    shape.m_vertex1.Set(%.15f, %.15f);\n", this.m_vertex1.x, this.m_vertex1.y);
        log("    shape.m_vertex2.Set(%.15f, %.15f);\n", this.m_vertex2.x, this.m_vertex2.y);
        log("    shape.m_vertex3.Set(%.15f, %.15f);\n", this.m_vertex3.x, this.m_vertex3.y);
        log("    shape.m_hasVertex0 = %s;\n", this.m_hasVertex0);
        log("    shape.m_hasVertex3 = %s;\n", this.m_hasVertex3);
    };
    // #if B2_ENABLE_PARTICLE
    /// @see b2Shape::ComputeDistance
    b2EdgeShape.ComputeDistance_s_v1 = new b2Math_1.b2Vec2();
    b2EdgeShape.ComputeDistance_s_v2 = new b2Math_1.b2Vec2();
    b2EdgeShape.ComputeDistance_s_d = new b2Math_1.b2Vec2();
    b2EdgeShape.ComputeDistance_s_s = new b2Math_1.b2Vec2();
    // #endif
    /// Implement b2Shape.
    // p = p1 + t * d
    // v = v1 + s * e
    // p1 + t * d = v1 + s * e
    // s * e - t * d = p1 - v1
    b2EdgeShape.RayCast_s_p1 = new b2Math_1.b2Vec2();
    b2EdgeShape.RayCast_s_p2 = new b2Math_1.b2Vec2();
    b2EdgeShape.RayCast_s_d = new b2Math_1.b2Vec2();
    b2EdgeShape.RayCast_s_e = new b2Math_1.b2Vec2();
    b2EdgeShape.RayCast_s_q = new b2Math_1.b2Vec2();
    b2EdgeShape.RayCast_s_r = new b2Math_1.b2Vec2();
    /// @see b2Shape::ComputeAABB
    b2EdgeShape.ComputeAABB_s_v1 = new b2Math_1.b2Vec2();
    b2EdgeShape.ComputeAABB_s_v2 = new b2Math_1.b2Vec2();
    return b2EdgeShape;
}(b2Shape_1.b2Shape));
exports.b2EdgeShape = b2EdgeShape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJFZGdlU2hhcGUuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9Db2xsaXNpb24vU2hhcGVzL2IyRWRnZVNoYXBlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBRUYsNkRBQTZEO0FBQzdELHNEQUEyRDtBQUMzRCw4Q0FBcUU7QUFJckUscUNBQWlEO0FBRWpELDBFQUEwRTtBQUMxRSx3RUFBd0U7QUFDeEUsNEJBQTRCO0FBQzVCO0lBQWlDLCtCQUFPO0lBUXRDO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLFdBQVcsRUFBRSw2QkFBZ0IsQ0FBQyxTQUNqRDtRQVRlLGVBQVMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2pDLGVBQVMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2pDLGVBQVMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2pDLGVBQVMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQzFDLGtCQUFZLEdBQVksS0FBSyxDQUFDO1FBQzlCLGtCQUFZLEdBQVksS0FBSyxDQUFDOztJQUlyQyxDQUFDO0lBRUQsaUNBQWlDO0lBQzFCLHlCQUFHLEdBQVYsVUFBVyxFQUFNLEVBQUUsRUFBTTtRQUN2QixJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUN4QixJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUN4QixJQUFJLENBQUMsWUFBWSxHQUFHLEtBQUssQ0FBQztRQUMxQixJQUFJLENBQUMsWUFBWSxHQUFHLEtBQUssQ0FBQztRQUMxQixPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFFRCxzQkFBc0I7SUFDZiwyQkFBSyxHQUFaO1FBQ0UsT0FBTyxJQUFJLFdBQVcsRUFBRSxDQUFDLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztJ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