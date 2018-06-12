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
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
var b2DistanceProxy = /** @class */ (function () {
    function b2DistanceProxy() {
        this.m_buffer = b2Math_1.b2Vec2.MakeArray(2);
        this.m_vertices = this.m_buffer;
        this.m_count = 0;
        this.m_radius = 0;
    }
    b2DistanceProxy.prototype.Reset = function () {
        this.m_vertices = this.m_buffer;
        this.m_count = 0;
        this.m_radius = 0;
        return this;
    };
    b2DistanceProxy.prototype.SetShape = function (shape, index) {
        shape.SetupDistanceProxy(this, index);
    };
    b2DistanceProxy.prototype.GetSupport = function (d) {
        var bestIndex = 0;
        var bestValue = b2Math_1.b2Vec2.DotVV(this.m_vertices[0], d);
        for (var i = 1; i < this.m_count; ++i) {
            var value = b2Math_1.b2Vec2.DotVV(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return bestIndex;
    };
    b2DistanceProxy.prototype.GetSupportVertex = function (d) {
        var bestIndex = 0;
        var bestValue = b2Math_1.b2Vec2.DotVV(this.m_vertices[0], d);
        for (var i = 1; i < this.m_count; ++i) {
            var value = b2Math_1.b2Vec2.DotVV(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return this.m_vertices[bestIndex];
    };
    b2DistanceProxy.prototype.GetVertexCount = function () {
        return this.m_count;
    };
    b2DistanceProxy.prototype.GetVertex = function (index) {
        // DEBUG: b2Assert(0 <= index && index < this.m_count);
        return this.m_vertices[index];
    };
    return b2DistanceProxy;
}());
exports.b2DistanceProxy = b2DistanceProxy;
var b2SimplexCache = /** @class */ (function () {
    function b2SimplexCache() {
        this.metric = 0;
        this.count = 0;
        this.indexA = [0, 0, 0];
        this.indexB = [0, 0, 0];
    }
    b2SimplexCache.prototype.Reset = function () {
        this.metric = 0;
        this.count = 0;
        return this;
    };
    return b2SimplexCache;
}());
exports.b2SimplexCache = b2SimplexCache;
var b2DistanceInput = /** @class */ (function () {
    function b2DistanceInput() {
        this.proxyA = new b2DistanceProxy(); // TODO: readonly
        this.proxyB = new b2DistanceProxy(); // TODO: readonly
        this.transformA = new b2Math_1.b2Transform();
        this.transformB = new b2Math_1.b2Transform();
        this.useRadii = false;
    }
    b2DistanceInput.prototype.Reset = function () {
        this.proxyA.Reset();
        this.proxyB.Reset();
        this.transformA.SetIdentity();
        this.transformB.SetIdentity();
        this.useRadii = false;
        return this;
    };
    return b2DistanceInput;
}());
exports.b2DistanceInput = b2DistanceInput;
var b2DistanceOutput = /** @class */ (function () {
    function b2DistanceOutput() {
        this.pointA = new b2Math_1.b2Vec2();
        this.pointB = new b2Math_1.b2Vec2();
        this.distance = 0;
        this.iterations = 0; ///< number of GJK iterations used
    }
    b2DistanceOutput.prototype.Reset = function () {
        this.pointA.SetZero();
        this.pointB.SetZero();
        this.distance = 0;
        this.iterations = 0;
        return this;
    };
    return b2DistanceOutput;
}());
exports.b2DistanceOutput = b2DistanceOutput;
exports.b2_gjkCalls = 0;
exports.b2_gjkIters = 0;
exports.b2_gjkMaxIters = 0;
function b2_gjk_reset() {
    exports.b2_gjkCalls = 0;
    exports.b2_gjkIters = 0;
    exports.b2_gjkMaxIters = 0;
}
exports.b2_gjk_reset = b2_gjk_reset;
var b2SimplexVertex = /** @class */ (function () {
    function b2SimplexVertex() {
        this.wA = new b2Math_1.b2Vec2(); // support point in proxyA
        this.wB = new b2Math_1.b2Vec2(); // support point in proxyB
        this.w = new b2Math_1.b2Vec2(); // wB - wA
        this.a = 0; // barycentric coordinate for closest point
        this.indexA = 0; // wA index
        this.indexB = 0; // wB index
    }
    b2SimplexVertex.prototype.Copy = function (other) {
        this.wA.Copy(other.wA); // support point in proxyA
        this.wB.Copy(other.wB); // support point in proxyB
        this.w.Copy(other.w); // wB - wA
        this.a = other.a; // barycentric coordinate for closest point
        this.indexA = other.indexA; // wA index
        this.indexB = other.indexB; // wB index
        return this;
    };
    return b2SimplexVertex;
}());
exports.b2SimplexVertex = b2SimplexVertex;
var b2Simplex = /** @class */ (function () {
    function b2Simplex() {
        this.m_v1 = new b2SimplexVertex();
        this.m_v2 = new b2SimplexVertex();
        this.m_v3 = new b2SimplexVertex();
        this.m_vertices = [ /*3*/];
        this.m_count = 0;
        this.m_vertices[0] = this.m_v1;
        this.m_vertices[1] = this.m_v2;
        this.m_vertices[2] = this.m_v3;
    }
    b2Simplex.prototype.ReadCache = function (cache, proxyA, transformA, proxyB, transformB) {
        // DEBUG: b2Assert(0 <= cache.count && cache.count <= 3);
        // Copy data from cache.
        this.m_count = cache.count;
        var vertices = this.m_vertices;
        for (var i = 0; i < this.m_count; ++i) {
            var v = vertices[i];
            v.indexA = cache.indexA[i];
            v.indexB = cache.indexB[i];
            var wALocal = proxyA.GetVertex(v.indexA);
            var wBLocal = proxyB.GetVertex(v.indexB);
            b2Math_1.b2Transform.MulXV(transformA, wALocal, v.wA);
            b2Math_1.b2Transform.MulXV(transformB, wBLocal, v.wB);
            b2Math_1.b2Vec2.SubVV(v.wB, v.wA, v.w);
            v.a = 0;
        }
        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (this.m_count > 1) {
            var metric1 = cache.metric;
            var metric2 = this.GetMetric();
            if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < b2Settings_1.b2_epsilon) {
                // Reset the simplex.
                this.m_count = 0;
            }
        }
        // If the cache is empty or invalid ...
        if (this.m_count === 0) {
            var v = vertices[0];
            v.indexA = 0;
            v.indexB = 0;
            var wALocal = proxyA.GetVertex(0);
            var wBLocal = proxyB.GetVertex(0);
            b2Math_1.b2Transform.MulXV(transformA, wALocal, v.wA);
            b2Math_1.b2Transform.MulXV(transformB, wBLocal, v.wB);
            b2Math_1.b2Vec2.SubVV(v.wB, v.wA, v.w);
            v.a = 1;
            this.m_count = 1;
        }
    };
    b2Simplex.prototype.WriteCache = function (cache) {
        cache.metric = this.GetMetric();
        cache.count = this.m_count;
        var vertices = this.m_vertices;
        for (var i = 0; i < this.m_count; ++i) {
            cache.indexA[i] = vertices[i].indexA;
            cache.indexB[i] = vertices[i].indexB;
        }
    };
    b2Simplex.prototype.GetSearchDirection = function (out) {
        switch (this.m_count) {
            case 1:
                return b2Math_1.b2Vec2.NegV(this.m_v1.w, out);
            case 2: {
                var e12 = b2Math_1.b2Vec2.SubVV(this.m_v2.w, this.m_v1.w, out);
                var sgn = b2Math_1.b2Vec2.CrossVV(e12, b2Math_1.b2Vec2.NegV(this.m_v1.w, b2Math_1.b2Vec2.s_t0));
                if (sgn > 0) {
                    // Origin is left of e12.
                    return b2Math_1.b2Vec2.CrossOneV(e12, out);
                }
                else {
                    // Origin is right of e12.
                    return b2Math_1.b2Vec2.CrossVOne(e12, out);
                }
            }
            default:
                // DEBUG: b2Assert(false);
                return out.SetZero();
        }
    };
    b2Simplex.prototype.GetClosestPoint = function (out) {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                return out.SetZero();
            case 1:
                return out.Copy(this.m_v1.w);
            case 2:
                return out.Set(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
            case 3:
                return out.SetZero();
            default:
                // DEBUG: b2Assert(false);
                return out.SetZero();
        }
    };
    b2Simplex.prototype.GetWitnessPoints = function (pA, pB) {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                break;
            case 1:
                pA.Copy(this.m_v1.wA);
                pB.Copy(this.m_v1.wB);
                break;
            case 2:
                pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
                pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
                pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
                pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
                break;
            case 3:
                pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
                pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
                break;
            default:
                // DEBUG: b2Assert(false);
                break;
        }
    };
    b2Simplex.prototype.GetMetric = function () {
        switch (this.m_count) {
            case 0:
                // DEBUG: b2Assert(false);
                return 0;
            case 1:
                return 0;
            case 2:
                return b2Math_1.b2Vec2.DistanceVV(this.m_v1.w, this.m_v2.w);
            case 3:
                return b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.SubVV(this.m_v2.w, this.m_v1.w, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(this.m_v3.w, this.m_v1.w, b2Math_1.b2Vec2.s_t1));
            default:
                // DEBUG: b2Assert(false);
                return 0;
        }
    };
    b2Simplex.prototype.Solve2 = function () {
        var w1 = this.m_v1.w;
        var w2 = this.m_v2.w;
        var e12 = b2Math_1.b2Vec2.SubVV(w2, w1, b2Simplex.s_e12);
        // w1 region
        var d12_2 = (-b2Math_1.b2Vec2.DotVV(w1, e12));
        if (d12_2 <= 0) {
            // a2 <= 0, so we clamp it to 0
            this.m_v1.a = 1;
            this.m_count = 1;
            return;
        }
        // w2 region
        var d12_1 = b2Math_1.b2Vec2.DotVV(w2, e12);
        if (d12_1 <= 0) {
            // a1 <= 0, so we clamp it to 0
            this.m_v2.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
            return;
        }
        // Must be in e12 region.
        var inv_d12 = 1 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
    };
    b2Simplex.prototype.Solve3 = function () {
        var w1 = this.m_v1.w;
        var w2 = this.m_v2.w;
        var w3 = this.m_v3.w;
        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        var e12 = b2Math_1.b2Vec2.SubVV(w2, w1, b2Simplex.s_e12);
        var w1e12 = b2Math_1.b2Vec2.DotVV(w1, e12);
        var w2e12 = b2Math_1.b2Vec2.DotVV(w2, e12);
        var d12_1 = w2e12;
        var d12_2 = (-w1e12);
        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        var e13 = b2Math_1.b2Vec2.SubVV(w3, w1, b2Simplex.s_e13);
        var w1e13 = b2Math_1.b2Vec2.DotVV(w1, e13);
        var w3e13 = b2Math_1.b2Vec2.DotVV(w3, e13);
        var d13_1 = w3e13;
        var d13_2 = (-w1e13);
        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        var e23 = b2Math_1.b2Vec2.SubVV(w3, w2, b2Simplex.s_e23);
        var w2e23 = b2Math_1.b2Vec2.DotVV(w2, e23);
        var w3e23 = b2Math_1.b2Vec2.DotVV(w3, e23);
        var d23_1 = w3e23;
        var d23_2 = (-w2e23);
        // Triangle123
        var n123 = b2Math_1.b2Vec2.CrossVV(e12, e13);
        var d123_1 = n123 * b2Math_1.b2Vec2.CrossVV(w2, w3);
        var d123_2 = n123 * b2Math_1.b2Vec2.CrossVV(w3, w1);
        var d123_3 = n123 * b2Math_1.b2Vec2.CrossVV(w1, w2);
        // w1 region
        if (d12_2 <= 0 && d13_2 <= 0) {
            this.m_v1.a = 1;
            this.m_count = 1;
            return;
        }
        // e12
        if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
            var inv_d12 = 1 / (d12_1 + d12_2);
            this.m_v1.a = d12_1 * inv_d12;
            this.m_v2.a = d12_2 * inv_d12;
            this.m_count = 2;
            return;
        }
        // e13
        if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
            var inv_d13 = 1 / (d13_1 + d13_2);
            this.m_v1.a = d13_1 * inv_d13;
            this.m_v3.a = d13_2 * inv_d13;
            this.m_count = 2;
            this.m_v2.Copy(this.m_v3);
            return;
        }
        // w2 region
        if (d12_1 <= 0 && d23_2 <= 0) {
            this.m_v2.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v2);
            return;
        }
        // w3 region
        if (d13_1 <= 0 && d23_1 <= 0) {
            this.m_v3.a = 1;
            this.m_count = 1;
            this.m_v1.Copy(this.m_v3);
            return;
        }
        // e23
        if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
            var inv_d23 = 1 / (d23_1 + d23_2);
            this.m_v2.a = d23_1 * inv_d23;
            this.m_v3.a = d23_2 * inv_d23;
            this.m_count = 2;
            this.m_v1.Copy(this.m_v3);
            return;
        }
        // Must be in triangle123
        var inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
        this.m_v1.a = d123_1 * inv_d123;
        this.m_v2.a = d123_2 * inv_d123;
        this.m_v3.a = d123_3 * inv_d123;
        this.m_count = 3;
    };
    b2Simplex.s_e12 = new b2Math_1.b2Vec2();
    b2Simplex.s_e13 = new b2Math_1.b2Vec2();
    b2Simplex.s_e23 = new b2Math_1.b2Vec2();
    return b2Simplex;
}());
exports.b2Simplex = b2Simplex;
var b2Distance_s_simplex = new b2Simplex();
var b2Distance_s_saveA = [0, 0, 0];
var b2Distance_s_saveB = [0, 0, 0];
var b2Distance_s_p = new b2Math_1.b2Vec2();
var b2Distance_s_d = new b2Math_1.b2Vec2();
var b2Distance_s_normal = new b2Math_1.b2Vec2();
var b2Distance_s_supportA = new b2Math_1.b2Vec2();
var b2Distance_s_supportB = new b2Math_1.b2Vec2();
function b2Distance(output, cache, input) {
    ++exports.b2_gjkCalls;
    var proxyA = input.proxyA;
    var proxyB = input.proxyB;
    var transformA = input.transformA;
    var transformB = input.transformB;
    // Initialize the simplex.
    var simplex = b2Distance_s_simplex;
    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
    // Get simplex vertices as an array.
    var vertices = simplex.m_vertices;
    var k_maxIters = 20;
    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    var saveA = b2Distance_s_saveA;
    var saveB = b2Distance_s_saveB;
    var saveCount = 0;
    var distanceSqr1 = b2Settings_1.b2_maxFloat;
    var distanceSqr2 = distanceSqr1;
    // Main iteration loop.
    var iter = 0;
    while (iter < k_maxIters) {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (var i = 0; i < saveCount; ++i) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }
        switch (simplex.m_count) {
            case 1:
                break;
            case 2:
                simplex.Solve2();
                break;
            case 3:
                simplex.Solve3();
                break;
            default:
                // DEBUG: b2Assert(false);
                break;
        }
        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count === 3) {
            break;
        }
        // Compute closest point.
        var p = simplex.GetClosestPoint(b2Distance_s_p);
        distanceSqr2 = p.LengthSquared();
        // Ensure progress
        /*
        TODO: to fix compile warning
        if (distanceSqr2 > distanceSqr1) {
          //break;
        }
        */
        distanceSqr1 = distanceSqr2;
        // Get search direction.
        var d = simplex.GetSearchDirection(b2Distance_s_d);
        // Ensure the search direction is numerically fit.
        if (d.LengthSquared() < b2Settings_1.b2_epsilon_sq) {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.
            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }
        // Compute a tentative new simplex vertex using support points.
        var vertex = vertices[simplex.m_count];
        vertex.indexA = proxyA.GetSupport(b2Math_1.b2Rot.MulTRV(transformA.q, b2Math_1.b2Vec2.NegV(d, b2Math_1.b2Vec2.s_t0), b2Distance_s_supportA));
        b2Math_1.b2Transform.MulXV(transformA, proxyA.GetVertex(vertex.indexA), vertex.wA);
        vertex.indexB = proxyB.GetSupport(b2Math_1.b2Rot.MulTRV(transformB.q, d, b2Distance_s_supportB));
        b2Math_1.b2Transform.MulXV(transformB, proxyB.GetVertex(vertex.indexB), vertex.wB);
        b2Math_1.b2Vec2.SubVV(vertex.wB, vertex.wA, vertex.w);
        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++exports.b2_gjkIters;
        // Check for duplicate support points. This is the main termination criteria.
        var duplicate = false;
        for (var i = 0; i < saveCount; ++i) {
            if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
                duplicate = true;
                break;
            }
        }
        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate) {
            break;
        }
        // New vertex is ok and needed.
        ++simplex.m_count;
    }
    exports.b2_gjkMaxIters = b2Math_1.b2Max(exports.b2_gjkMaxIters, iter);
    // Prepare output.
    simplex.GetWitnessPoints(output.pointA, output.pointB);
    output.distance = b2Math_1.b2Vec2.DistanceVV(output.pointA, output.pointB);
    output.iterations = iter;
    // Cache the simplex.
    simplex.WriteCache(cache);
    // Apply radii if requested.
    if (input.useRadii) {
        var rA = proxyA.m_radius;
        var rB = proxyB.m_radius;
        if (output.distance > (rA + rB) && output.distance > b2Settings_1.b2_epsilon) {
            // Shapes are still no overlapped.
            // Move the witness points to the outer surface.
            output.distance -= rA + rB;
            var normal = b2Math_1.b2Vec2.SubVV(output.pointB, output.pointA, b2Distance_s_normal);
            normal.Normalize();
            output.pointA.SelfMulAdd(rA, normal);
            output.pointB.SelfMulSub(rB, normal);
        }
        else {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            var p = b2Math_1.b2Vec2.MidVV(output.pointA, output.pointB, b2Distance_s_p);
            output.pointA.Copy(p);
            output.pointB.Copy(p);
            output.distance = 0;
        }
    }
}
exports.b2Distance = b2Distance;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEaXN0YW5jZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9iMkRpc3RhbmNlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiwwREFBMEQ7QUFDMUQsbURBQThFO0FBQzlFLDJDQUFxRTtBQUdyRSxrREFBa0Q7QUFDbEQsOEJBQThCO0FBQzlCO0lBQUE7UUFDUyxhQUFRLEdBQWEsZUFBTSxDQUFDLFNBQVMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN6QyxlQUFVLEdBQWEsSUFBSSxDQUFDLFFBQVEsQ0FBQztRQUNyQyxZQUFPLEdBQVcsQ0FBQyxDQUFDO1FBQ3BCLGFBQVEsR0FBVyxDQUFDLENBQUM7SUFpRDlCLENBQUM7SUEvQ1EsK0JBQUssR0FBWjtRQUNFLElBQUksQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDLFFBQVEsQ0FBQztRQUNoQyxJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsQ0FBQztRQUNqQixJQUFJLENBQUMsUUFBUSxHQUFHLENBQUMsQ0FBQztRQUNsQixPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFFTSxrQ0FBUSxHQUFmLFVBQWdCLEtBQWMsRUFBRSxLQUFhO1FBQzNDLEtBQUssQ0FBQyxrQkFBa0IsQ0FBQyxJQUFJLEVBQUUsS0FBSyxDQUFDLENBQUM7SUFDeEMsQ0FBQztJQUVNLG9DQUFVLEdBQWpCLFVBQWtCLENBQVM7UUFDekIsSUFBSSxTQUFTLEdBQVcsQ0FBQyxDQUFDO1FBQzFCLElBQUksU0FBUyxHQUFXLGVBQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUM1RCxLQUFLLElBQUksQ0FBQyxHQUFXLENBQUMsRUFBRSxDQUFDLEdBQUcsSUFBSSxDQUFDLE9BQU8sRUFBRSxFQUFFLENBQUMsRUFBRTtZQUM3QyxJQUFNLEtBQUss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LEVBQUUsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQztnQkFDdEIsTUFBTTtZQUVSLEtBQUssQ0FBQztnQkFDSixFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7Z0JBQ25FLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztnQkFDbkUsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO2dCQUNuRSxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7Z0JBQ25FLE1BQU07WUFFUixLQUFLLENBQUM7Z0JBQ0osRUFBRSxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxJQUFJLENB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