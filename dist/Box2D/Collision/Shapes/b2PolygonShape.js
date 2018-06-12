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
// DEBUG: import { b2Assert, b2_epsilon_sq } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Shape_1 = require("./b2Shape");
var b2Shape_2 = require("./b2Shape");
/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
var b2PolygonShape = /** @class */ (function (_super) {
    __extends(b2PolygonShape, _super);
    function b2PolygonShape() {
        var _this = _super.call(this, b2Shape_2.b2ShapeType.e_polygonShape, b2Settings_1.b2_polygonRadius) || this;
        _this.m_centroid = new b2Math_1.b2Vec2(0, 0);
        _this.m_vertices = []; // b2Vec2.MakeArray(b2_maxPolygonVertices);
        _this.m_normals = []; // b2Vec2.MakeArray(b2_maxPolygonVertices);
        _this.m_count = 0;
        return _this;
    }
    /// Implement b2Shape.
    b2PolygonShape.prototype.Clone = function () {
        return new b2PolygonShape().Copy(this);
    };
    b2PolygonShape.prototype.Copy = function (other) {
        _super.prototype.Copy.call(this, other);
        // DEBUG: b2Assert(other instanceof b2PolygonShape);
        this.m_centroid.Copy(other.m_centroid);
        this.m_count = other.m_count;
        this.m_vertices = b2Math_1.b2Vec2.MakeArray(this.m_count);
        this.m_normals = b2Math_1.b2Vec2.MakeArray(this.m_count);
        for (var i = 0; i < this.m_count; ++i) {
            this.m_vertices[i].Copy(other.m_vertices[i]);
            this.m_normals[i].Copy(other.m_normals[i]);
        }
        return this;
    };
    /// @see b2Shape::GetChildCount
    b2PolygonShape.prototype.GetChildCount = function () {
        return 1;
    };
    b2PolygonShape.prototype.Set = function (vertices, count, start) {
        if (count === void 0) { count = vertices.length; }
        if (start === void 0) { start = 0; }
        // DEBUG: b2Assert(3 <= count && count <= b2_maxPolygonVertices);
        if (count < 3) {
            return this.SetAsBox(1, 1);
        }
        var n = b2Math_1.b2Min(count, b2Settings_1.b2_maxPolygonVertices);
        // Perform welding and copy vertices into local buffer.
        var ps = b2PolygonShape.Set_s_ps;
        var tempCount = 0;
        for (var i = 0; i < n; ++i) {
            var /*b2Vec2*/ v = vertices[start + i];
            var /*bool*/ unique = true;
            for (var /*int32*/ j = 0; j < tempCount; ++j) {
                if (b2Math_1.b2Vec2.DistanceSquaredVV(v, ps[j]) < ((0.5 * b2Settings_1.b2_linearSlop) * (0.5 * b2Settings_1.b2_linearSlop))) {
                    unique = false;
                    break;
                }
            }
            if (unique) {
                ps[tempCount++].Copy(v); // ps[tempCount++] = v;
            }
        }
        n = tempCount;
        if (n < 3) {
            // Polygon is degenerate.
            // DEBUG: b2Assert(false);
            return this.SetAsBox(1.0, 1.0);
        }
        // Create the convex hull using the Gift wrapping algorithm
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
        // Find the right most point on the hull
        var i0 = 0;
        var x0 = ps[0].x;
        for (var i = 1; i < n; ++i) {
            var x = ps[i].x;
            if (x > x0 || (x === x0 && ps[i].y < ps[i0].y)) {
                i0 = i;
                x0 = x;
            }
        }
        var hull = b2PolygonShape.Set_s_hull;
        var m = 0;
        var ih = i0;
        for (;;) {
            hull[m] = ih;
            var ie = 0;
            for (var j = 1; j < n; ++j) {
                if (ie === ih) {
                    ie = j;
                    continue;
                }
                var r = b2Math_1.b2Vec2.SubVV(ps[ie], ps[hull[m]], b2PolygonShape.Set_s_r);
                var v = b2Math_1.b2Vec2.SubVV(ps[j], ps[hull[m]], b2PolygonShape.Set_s_v);
                var c = b2Math_1.b2Vec2.CrossVV(r, v);
                if (c < 0) {
                    ie = j;
                }
                // Collinearity check
                if (c === 0 && v.LengthSquared() > r.LengthSquared()) {
                    ie = j;
                }
            }
            ++m;
            ih = ie;
            if (ie === i0) {
                break;
            }
        }
        this.m_count = m;
        this.m_vertices = b2Math_1.b2Vec2.MakeArray(this.m_count);
        this.m_normals = b2Math_1.b2Vec2.MakeArray(this.m_count);
        // Copy vertices.
        for (var i = 0; i < m; ++i) {
            this.m_vertices[i].Copy(ps[hull[i]]);
        }
        // Compute normals. Ensure the edges have non-zero length.
        for (var i = 0; i < m; ++i) {
            var vertexi1 = this.m_vertices[i];
            var vertexi2 = this.m_vertices[(i + 1) % m];
            var edge = b2Math_1.b2Vec2.SubVV(vertexi2, vertexi1, b2Math_1.b2Vec2.s_t0); // edge uses s_t0
            // DEBUG: b2Assert(edge.LengthSquared() > b2_epsilon_sq);
            b2Math_1.b2Vec2.CrossVOne(edge, this.m_normals[i]).SelfNormalize();
        }
        // Compute the polygon centroid.
        b2PolygonShape.ComputeCentroid(this.m_vertices, m, this.m_centroid);
        return this;
    };
    b2PolygonShape.prototype.SetAsArray = function (vertices, count) {
        if (count === void 0) { count = vertices.length; }
        return this.Set(vertices, count);
    };
    /// Build vertices to represent an axis-aligned box or an oriented box.
    /// @param hx the half-width.
    /// @param hy the half-height.
    /// @param center the center of the box in local coordinates.
    /// @param angle the rotation of the box in local coordinates.
    b2PolygonShape.prototype.SetAsBox = function (hx, hy, center, angle) {
        if (angle === void 0) { angle = 0; }
        this.m_count = 4;
        this.m_vertices = b2Math_1.b2Vec2.MakeArray(this.m_count);
        this.m_normals = b2Math_1.b2Vec2.MakeArray(this.m_count);
        this.m_vertices[0].Set((-hx), (-hy));
        this.m_vertices[1].Set(hx, (-hy));
        this.m_vertices[2].Set(hx, hy);
        this.m_vertices[3].Set((-hx), hy);
        this.m_normals[0].Set(0, (-1));
        this.m_normals[1].Set(1, 0);
        this.m_normals[2].Set(0, 1);
        this.m_normals[3].Set((-1), 0);
        this.m_centroid.SetZero();
        if (center) {
            this.m_centroid.Copy(center);
            var xf = new b2Math_1.b2Transform();
            xf.SetPosition(center);
            xf.SetRotationAngle(angle);
            // Transform vertices and normals.
            for (var i = 0; i < this.m_count; ++i) {
                b2Math_1.b2Transform.MulXV(xf, this.m_vertices[i], this.m_vertices[i]);
                b2Math_1.b2Rot.MulRV(xf.q, this.m_normals[i], this.m_normals[i]);
            }
        }
        return this;
    };
    b2PolygonShape.prototype.TestPoint = function (xf, p) {
        var pLocal = b2Math_1.b2Transform.MulTXV(xf, p, b2PolygonShape.TestPoint_s_pLocal);
        for (var i = 0; i < this.m_count; ++i) {
            var dot = b2Math_1.b2Vec2.DotVV(this.m_normals[i], b2Math_1.b2Vec2.SubVV(pLocal, this.m_vertices[i], b2Math_1.b2Vec2.s_t0));
            if (dot > 0) {
                return false;
            }
        }
        return true;
    };
    b2PolygonShape.prototype.ComputeDistance = function (xf, p, normal, childIndex) {
        var pLocal = b2Math_1.b2Transform.MulTXV(xf, p, b2PolygonShape.ComputeDistance_s_pLocal);
        var maxDistance = -b2Settings_1.b2_maxFloat;
        var normalForMaxDistance = b2PolygonShape.ComputeDistance_s_normalForMaxDistance.Copy(pLocal);
        for (var i = 0; i < this.m_count; ++i) {
            var dot = b2Math_1.b2Vec2.DotVV(this.m_normals[i], b2Math_1.b2Vec2.SubVV(pLocal, this.m_vertices[i], b2Math_1.b2Vec2.s_t0));
            if (dot > maxDistance) {
                maxDistance = dot;
                normalForMaxDistance.Copy(this.m_normals[i]);
            }
        }
        if (maxDistance > 0) {
            var minDistance = b2PolygonShape.ComputeDistance_s_minDistance.Copy(normalForMaxDistance);
            var minDistance2 = maxDistance * maxDistance;
            for (var i = 0; i < this.m_count; ++i) {
                var distance = b2Math_1.b2Vec2.SubVV(pLocal, this.m_vertices[i], b2PolygonShape.ComputeDistance_s_distance);
                var distance2 = distance.LengthSquared();
                if (minDistance2 > distance2) {
                    minDistance.Copy(distance);
                    minDistance2 = distance2;
                }
            }
            b2Math_1.b2Rot.MulRV(xf.q, minDistance, normal);
            normal.Normalize();
            return Math.sqrt(minDistance2);
        }
        else {
            b2Math_1.b2Rot.MulRV(xf.q, normalForMaxDistance, normal);
            return maxDistance;
        }
    };
    b2PolygonShape.prototype.RayCast = function (output, input, xf, childIndex) {
        // Put the ray into the polygon's frame of reference.
        var p1 = b2Math_1.b2Transform.MulTXV(xf, input.p1, b2PolygonShape.RayCast_s_p1);
        var p2 = b2Math_1.b2Transform.MulTXV(xf, input.p2, b2PolygonShape.RayCast_s_p2);
        var d = b2Math_1.b2Vec2.SubVV(p2, p1, b2PolygonShape.RayCast_s_d);
        var lower = 0, upper = input.maxFraction;
        var index = -1;
        for (var i = 0; i < this.m_count; ++i) {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            var numerator = b2Math_1.b2Vec2.DotVV(this.m_normals[i], b2Math_1.b2Vec2.SubVV(this.m_vertices[i], p1, b2Math_1.b2Vec2.s_t0));
            var denominator = b2Math_1.b2Vec2.DotVV(this.m_normals[i], d);
            if (denominator === 0) {
                if (numerator < 0) {
                    return false;
                }
            }
            else {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
                if (denominator < 0 && numerator < lower * denominator) {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = i;
                }
                else if (denominator > 0 && numerator < upper * denominator) {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }
            // The use of epsilon here causes the assert on lower to trip
            // in some cases. Apparently the use of epsilon was to make edge
            // shapes work, but now those are handled separately.
            // if (upper < lower - b2_epsilon)
            if (upper < lower) {
                return false;
            }
        }
        // DEBUG: b2Assert(0 <= lower && lower <= input.maxFraction);
        if (index >= 0) {
            output.fraction = lower;
            b2Math_1.b2Rot.MulRV(xf.q, this.m_normals[index], output.normal);
            return true;
        }
        return false;
    };
    b2PolygonShape.prototype.ComputeAABB = function (aabb, xf, childIndex) {
        var lower = b2Math_1.b2Transform.MulXV(xf, this.m_vertices[0], aabb.lowerBound);
        var upper = aabb.upperBound.Copy(lower);
        for (var i = 0; i < this.m_count; ++i) {
            var v = b2Math_1.b2Transform.MulXV(xf, this.m_vertices[i], b2PolygonShape.ComputeAABB_s_v);
            b2Math_1.b2Vec2.MinV(v, lower, lower);
            b2Math_1.b2Vec2.MaxV(v, upper, upper);
        }
        var r = this.m_radius;
        lower.SelfSubXY(r, r);
        upper.SelfAddXY(r, r);
    };
    b2PolygonShape.prototype.ComputeMass = function (massData, density) {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.
        // DEBUG: b2Assert(this.m_count >= 3);
        var center = b2PolygonShape.ComputeMass_s_center.SetZero();
        var area = 0;
        var I = 0;
        // s is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        var s = b2PolygonShape.ComputeMass_s_s.SetZero();
        // This code would put the reference point inside the polygon.
        for (var i = 0; i < this.m_count; ++i) {
            s.SelfAdd(this.m_vertices[i]);
        }
        s.SelfMul(1 / this.m_count);
        var k_inv3 = 1 / 3;
        for (var i = 0; i < this.m_count; ++i) {
            // Triangle vertices.
            var e1 = b2Math_1.b2Vec2.SubVV(this.m_vertices[i], s, b2PolygonShape.ComputeMass_s_e1);
            var e2 = b2Math_1.b2Vec2.SubVV(this.m_vertices[(i + 1) % this.m_count], s, b2PolygonShape.ComputeMass_s_e2);
            var D = b2Math_1.b2Vec2.CrossVV(e1, e2);
            var triangleArea = 0.5 * D;
            area += triangleArea;
            // Area weighted centroid
            center.SelfAdd(b2Math_1.b2Vec2.MulSV(triangleArea * k_inv3, b2Math_1.b2Vec2.AddVV(e1, e2, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t1));
            var ex1 = e1.x;
            var ey1 = e1.y;
            var ex2 = e2.x;
            var ey2 = e2.y;
            var intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            var inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;
            I += (0.25 * k_inv3 * D) * (intx2 + inty2);
        }
        // Total mass
        massData.mass = density * area;
        // Center of mass
        // DEBUG: b2Assert(area > b2_epsilon);
        center.SelfMul(1 / area);
        b2Math_1.b2Vec2.AddVV(center, s, massData.center);
        // Inertia tensor relative to the local origin (point s).
        massData.I = density * I;
        // Shift to center of mass then to original body origin.
        massData.I += massData.mass * (b2Math_1.b2Vec2.DotVV(massData.center, massData.center) - b2Math_1.b2Vec2.DotVV(center, center));
    };
    b2PolygonShape.prototype.Validate = function () {
        for (var i = 0; i < this.m_count; ++i) {
            var i1 = i;
            var i2 = (i + 1) % this.m_count;
            var p = this.m_vertices[i1];
            var e = b2Math_1.b2Vec2.SubVV(this.m_vertices[i2], p, b2PolygonShape.Validate_s_e);
            for (var j = 0; j < this.m_count; ++j) {
                if (j === i1 || j === i2) {
                    continue;
                }
                var v = b2Math_1.b2Vec2.SubVV(this.m_vertices[j], p, b2PolygonShape.Validate_s_v);
                var c = b2Math_1.b2Vec2.CrossVV(e, v);
                if (c < 0) {
                    return false;
                }
            }
        }
        return true;
    };
    b2PolygonShape.prototype.SetupDistanceProxy = function (proxy, index) {
        proxy.m_vertices = this.m_vertices;
        proxy.m_count = this.m_count;
        proxy.m_radius = this.m_radius;
    };
    b2PolygonShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
        // Transform plane into shape co-ordinates
        var normalL = b2Math_1.b2Rot.MulTRV(xf.q, normal, b2PolygonShape.ComputeSubmergedArea_s_normalL);
        var offsetL = offset - b2Math_1.b2Vec2.DotVV(normal, xf.p);
        var depths = b2PolygonShape.ComputeSubmergedArea_s_depths;
        var diveCount = 0;
        var intoIndex = -1;
        var outoIndex = -1;
        var lastSubmerged = false;
        for (var i_1 = 0; i_1 < this.m_count; ++i_1) {
            depths[i_1] = b2Math_1.b2Vec2.DotVV(normalL, this.m_vertices[i_1]) - offsetL;
            var isSubmerged = depths[i_1] < (-b2Settings_1.b2_epsilon);
            if (i_1 > 0) {
                if (isSubmerged) {
                    if (!lastSubmerged) {
                        intoIndex = i_1 - 1;
                        diveCount++;
                    }
                }
                else {
                    if (lastSubmerged) {
                        outoIndex = i_1 - 1;
                        diveCount++;
                    }
                }
            }
            lastSubmerged = isSubmerged;
        }
        switch (diveCount) {
            case 0:
                if (lastSubmerged) {
                    // Completely submerged
                    var md = b2PolygonShape.ComputeSubmergedArea_s_md;
                    this.ComputeMass(md, 1);
                    b2Math_1.b2Transform.MulXV(xf, md.center, c);
                    return md.mass;
                }
                else {
                    // Completely dry
                    return 0;
                }
            case 1:
                if (intoIndex === (-1)) {
                    intoIndex = this.m_count - 1;
                }
                else {
                    outoIndex = this.m_count - 1;
                }
                break;
        }
        var intoIndex2 = ((intoIndex + 1) % this.m_count);
        var outoIndex2 = ((outoIndex + 1) % this.m_count);
        var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
        var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
        var intoVec = b2PolygonShape.ComputeSubmergedArea_s_intoVec.Set(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda, this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
        var outoVec = b2PolygonShape.ComputeSubmergedArea_s_outoVec.Set(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda, this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
        // Initialize accumulator
        var area = 0;
        var center = b2PolygonShape.ComputeSubmergedArea_s_center.SetZero();
        var p2 = this.m_vertices[intoIndex2];
        var p3;
        // An awkward loop from intoIndex2+1 to outIndex2
        var i = intoIndex2;
        while (i !== outoIndex2) {
            i = (i + 1) % this.m_count;
            if (i === outoIndex2) {
                p3 = outoVec;
            }
            else {
                p3 = this.m_vertices[i];
            }
            var triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
            area += triangleArea;
            // Area weighted centroid
            center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
            center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
            p2 = p3;
        }
        // Normalize and transform centroid
        center.SelfMul(1 / area);
        b2Math_1.b2Transform.MulXV(xf, center, c);
        return area;
    };
    b2PolygonShape.prototype.Dump = function (log) {
        log("    const shape: b2PolygonShape = new b2PolygonShape();\n");
        log("    const vs: b2Vec2[] = b2Vec2.MakeArray(%d);\n", b2Settings_1.b2_maxPolygonVertices);
        for (var i = 0; i < this.m_count; ++i) {
            log("    vs[%d].Set(%.15f, %.15f);\n", i, this.m_vertices[i].x, this.m_vertices[i].y);
        }
        log("    shape.Set(vs, %d);\n", this.m_count);
    };
    b2PolygonShape.ComputeCentroid = function (vs, count, out) {
        // DEBUG: b2Assert(count >= 3);
        var c = out;
        c.SetZero();
        var area = 0;
        // s is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        var pRef = b2PolygonShape.ComputeCentroid_s_pRef.SetZero();
        /*
    #if 0
        // This code would put the reference point inside the polygon.
        for (let i: number = 0; i < count; ++i) {
          pRef.SelfAdd(vs[i]);
        }
        pRef.SelfMul(1 / count);
    #endif
        */
        var inv3 = 1 / 3;
        for (var i = 0; i < count; ++i) {
            // Triangle vertices.
            var p1 = pRef;
            var p2 = vs[i];
            var p3 = vs[(i + 1) % count];
            var e1 = b2Math_1.b2Vec2.SubVV(p2, p1, b2PolygonShape.ComputeCentroid_s_e1);
            var e2 = b2Math_1.b2Vec2.SubVV(p3, p1, b2PolygonShape.ComputeCentroid_s_e2);
            var D = b2Math_1.b2Vec2.CrossVV(e1, e2);
            var triangleArea = 0.5 * D;
            area += triangleArea;
            // Area weighted centroid
            c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
            c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
        }
        // Centroid
        // DEBUG: b2Assert(area > b2_epsilon);
        c.SelfMul(1 / area);
        return c;
    };
    /// Create a convex hull from the given array of points.
    /// The count must be in the range [3, b2_maxPolygonVertices].
    /// @warning the points may be re-ordered, even if they form a convex polygon
    /// @warning collinear points are handled but not removed. Collinear points
    /// may lead to poor stacking behavior.
    b2PolygonShape.Set_s_ps = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxPolygonVertices);
    b2PolygonShape.Set_s_hull = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxPolygonVertices);
    b2PolygonShape.Set_s_r = new b2Math_1.b2Vec2();
    b2PolygonShape.Set_s_v = new b2Math_1.b2Vec2();
    /// @see b2Shape::TestPoint
    b2PolygonShape.TestPoint_s_pLocal = new b2Math_1.b2Vec2();
    // #if B2_ENABLE_PARTICLE
    /// @see b2Shape::ComputeDistance
    b2PolygonShape.ComputeDistance_s_pLocal = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeDistance_s_normalForMaxDistance = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeDistance_s_minDistance = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeDistance_s_distance = new b2Math_1.b2Vec2();
    // #endif
    /// Implement b2Shape.
    b2PolygonShape.RayCast_s_p1 = new b2Math_1.b2Vec2();
    b2PolygonShape.RayCast_s_p2 = new b2Math_1.b2Vec2();
    b2PolygonShape.RayCast_s_d = new b2Math_1.b2Vec2();
    /// @see b2Shape::ComputeAABB
    b2PolygonShape.ComputeAABB_s_v = new b2Math_1.b2Vec2();
    /// @see b2Shape::ComputeMass
    b2PolygonShape.ComputeMass_s_center = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeMass_s_s = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeMass_s_e1 = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeMass_s_e2 = new b2Math_1.b2Vec2();
    b2PolygonShape.Validate_s_e = new b2Math_1.b2Vec2();
    b2PolygonShape.Validate_s_v = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeSubmergedArea_s_normalL = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeSubmergedArea_s_depths = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxPolygonVertices);
    b2PolygonShape.ComputeSubmergedArea_s_md = new b2Shape_1.b2MassData();
    b2PolygonShape.ComputeSubmergedArea_s_intoVec = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeSubmergedArea_s_outoVec = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeSubmergedArea_s_center = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeCentroid_s_pRef = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeCentroid_s_e1 = new b2Math_1.b2Vec2();
    b2PolygonShape.ComputeCentroid_s_e2 = new b2Math_1.b2Vec2();
    return b2PolygonShape;
}(b2Shape_2.b2Shape));
exports.b2PolygonShape = b2PolygonShape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQb2x5Z29uU2hhcGUuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9Db2xsaXNpb24vU2hhcGVzL2IyUG9seWdvblNoYXBlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBRUYsNEVBQTRFO0FBQzVFLHNEQUE2STtBQUM3SSw4Q0FBNEU7QUFHNUUscUNBQXVDO0FBQ3ZDLHFDQUFpRDtBQUVqRCwwRUFBMEU7QUFDMUUsMEJBQTBCO0FBQzFCLDhFQUE4RTtBQUM5RSx5RUFBeUU7QUFDekU7SUFBb0Msa0NBQU87SUFNekM7UUFBQSxZQUNFLGtCQUFNLHFCQUFXLENBQUMsY0FBYyxFQUFFLDZCQUFnQixDQUFDLFNBQ3BEO1FBUGUsZ0JBQVUsR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDL0MsZ0JBQVUsR0FBYSxFQUFFLENBQUMsQ0FBQywyQ0FBMkM7UUFDdEUsZUFBUyxHQUFhLEVBQUUsQ0FBQyxDQUFDLDJDQUEyQztRQUNyRSxhQUFPLEdBQVcsQ0FBQyxDQUFDOztJQUkzQixDQUFDO0lBRUQsc0JBQXNCO0lBQ2YsOEJBQUssR0FBWjtRQUNFLE9BQU8sSUFBSSxjQUFjLEVBQUUsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7SUFDekMsQ0FBQztJQUVNLDZCQUFJLEdBQVgsVUFBWSxLQUFxQjtRQUMvQixpQkFBTSxJQUFJLFlBQUMsS0FBSyxDQUFDLENBQUM7UUFFbEIsb0RBQW9EO1FBRXBELElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLEtBQUssQ0FBQyxVQUFVLENBQUMsQ0FBQztRQUN2QyxJQUFJLENBQUMsT0FBTyxHQUFHLEtBQUssQ0FBQyxPQUFPLENBQUM7UUFDN0IsSUFBSSxDQUFDLFVBQVUsR0FBRyxlQUFNLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMs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