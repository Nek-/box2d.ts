"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Collision_1 = require("./b2Collision");
var b2Collision_2 = require("./b2Collision");
var b2CollideEdgeAndCircle_s_Q = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_e = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_d = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_e1 = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_e2 = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_P = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_n = new b2Math_1.b2Vec2();
var b2CollideEdgeAndCircle_s_id = new b2Collision_1.b2ContactID();
function b2CollideEdgeAndCircle(manifold, edgeA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle in frame of edge
    var Q = b2Math_1.b2Transform.MulTXV(xfA, b2Math_1.b2Transform.MulXV(xfB, circleB.m_p, b2Math_1.b2Vec2.s_t0), b2CollideEdgeAndCircle_s_Q);
    var A = edgeA.m_vertex1;
    var B = edgeA.m_vertex2;
    var e = b2Math_1.b2Vec2.SubVV(B, A, b2CollideEdgeAndCircle_s_e);
    // Barycentric coordinates
    var u = b2Math_1.b2Vec2.DotVV(e, b2Math_1.b2Vec2.SubVV(B, Q, b2Math_1.b2Vec2.s_t0));
    var v = b2Math_1.b2Vec2.DotVV(e, b2Math_1.b2Vec2.SubVV(Q, A, b2Math_1.b2Vec2.s_t0));
    var radius = edgeA.m_radius + circleB.m_radius;
    // const cf: b2ContactFeature = new b2ContactFeature();
    var id = b2CollideEdgeAndCircle_s_id;
    id.cf.indexB = 0;
    id.cf.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
    // Region A
    if (v <= 0) {
        var P_1 = A;
        var d_1 = b2Math_1.b2Vec2.SubVV(Q, P_1, b2CollideEdgeAndCircle_s_d);
        var dd_1 = b2Math_1.b2Vec2.DotVV(d_1, d_1);
        if (dd_1 > radius * radius) {
            return;
        }
        // Is there an edge connected to A?
        if (edgeA.m_hasVertex0) {
            var A1 = edgeA.m_vertex0;
            var B1 = A;
            var e1 = b2Math_1.b2Vec2.SubVV(B1, A1, b2CollideEdgeAndCircle_s_e1);
            var u1 = b2Math_1.b2Vec2.DotVV(e1, b2Math_1.b2Vec2.SubVV(B1, Q, b2Math_1.b2Vec2.s_t0));
            // Is the circle in Region AB of the previous edge?
            if (u1 > 0) {
                return;
            }
        }
        id.cf.indexA = 0;
        id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = b2Collision_2.b2ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P_1);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region B
    if (u <= 0) {
        var P_2 = B;
        var d_2 = b2Math_1.b2Vec2.SubVV(Q, P_2, b2CollideEdgeAndCircle_s_d);
        var dd_2 = b2Math_1.b2Vec2.DotVV(d_2, d_2);
        if (dd_2 > radius * radius) {
            return;
        }
        // Is there an edge connected to B?
        if (edgeA.m_hasVertex3) {
            var B2 = edgeA.m_vertex3;
            var A2 = B;
            var e2 = b2Math_1.b2Vec2.SubVV(B2, A2, b2CollideEdgeAndCircle_s_e2);
            var v2 = b2Math_1.b2Vec2.DotVV(e2, b2Math_1.b2Vec2.SubVV(Q, A2, b2Math_1.b2Vec2.s_t0));
            // Is the circle in Region AB of the next edge?
            if (v2 > 0) {
                return;
            }
        }
        id.cf.indexA = 1;
        id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = b2Collision_2.b2ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P_2);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region AB
    var den = b2Math_1.b2Vec2.DotVV(e, e);
    // DEBUG: b2Assert(den > 0);
    var P = b2CollideEdgeAndCircle_s_P;
    P.x = (1 / den) * (u * A.x + v * B.x);
    P.y = (1 / den) * (u * A.y + v * B.y);
    var d = b2Math_1.b2Vec2.SubVV(Q, P, b2CollideEdgeAndCircle_s_d);
    var dd = b2Math_1.b2Vec2.DotVV(d, d);
    if (dd > radius * radius) {
        return;
    }
    var n = b2CollideEdgeAndCircle_s_n.Set(-e.y, e.x);
    if (b2Math_1.b2Vec2.DotVV(n, b2Math_1.b2Vec2.SubVV(Q, A, b2Math_1.b2Vec2.s_t0)) < 0) {
        n.Set(-n.x, -n.y);
    }
    n.Normalize();
    id.cf.indexA = 0;
    id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_face;
    manifold.pointCount = 1;
    manifold.type = b2Collision_2.b2ManifoldType.e_faceA;
    manifold.localNormal.Copy(n);
    manifold.localPoint.Copy(A);
    manifold.points[0].id.Copy(id);
    // manifold.points[0].id.key = 0;
    // manifold.points[0].id.cf = cf;
    manifold.points[0].localPoint.Copy(circleB.m_p);
}
exports.b2CollideEdgeAndCircle = b2CollideEdgeAndCircle;
var b2EPAxis = /** @class */ (function () {
    function b2EPAxis() {
        this.type = 0 /* e_unknown */;
        this.index = 0;
        this.separation = 0;
    }
    return b2EPAxis;
}());
var b2TempPolygon = /** @class */ (function () {
    function b2TempPolygon() {
        this.vertices = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxPolygonVertices);
        this.normals = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxPolygonVertices);
        this.count = 0;
    }
    return b2TempPolygon;
}());
var b2ReferenceFace = /** @class */ (function () {
    function b2ReferenceFace() {
        this.i1 = 0;
        this.i2 = 0;
        this.v1 = new b2Math_1.b2Vec2();
        this.v2 = new b2Math_1.b2Vec2();
        this.normal = new b2Math_1.b2Vec2();
        this.sideNormal1 = new b2Math_1.b2Vec2();
        this.sideOffset1 = 0;
        this.sideNormal2 = new b2Math_1.b2Vec2();
        this.sideOffset2 = 0;
    }
    return b2ReferenceFace;
}());
var b2EPCollider = /** @class */ (function () {
    function b2EPCollider() {
        this.m_polygonB = new b2TempPolygon();
        this.m_xf = new b2Math_1.b2Transform();
        this.m_centroidB = new b2Math_1.b2Vec2();
        this.m_v0 = new b2Math_1.b2Vec2();
        this.m_v1 = new b2Math_1.b2Vec2();
        this.m_v2 = new b2Math_1.b2Vec2();
        this.m_v3 = new b2Math_1.b2Vec2();
        this.m_normal0 = new b2Math_1.b2Vec2();
        this.m_normal1 = new b2Math_1.b2Vec2();
        this.m_normal2 = new b2Math_1.b2Vec2();
        this.m_normal = new b2Math_1.b2Vec2();
        this.m_type1 = 0 /* e_isolated */;
        this.m_type2 = 0 /* e_isolated */;
        this.m_lowerLimit = new b2Math_1.b2Vec2();
        this.m_upperLimit = new b2Math_1.b2Vec2();
        this.m_radius = 0;
        this.m_front = false;
    }
    b2EPCollider.prototype.Collide = function (manifold, edgeA, xfA, polygonB, xfB) {
        b2Math_1.b2Transform.MulTXX(xfA, xfB, this.m_xf);
        b2Math_1.b2Transform.MulXV(this.m_xf, polygonB.m_centroid, this.m_centroidB);
        this.m_v0.Copy(edgeA.m_vertex0);
        this.m_v1.Copy(edgeA.m_vertex1);
        this.m_v2.Copy(edgeA.m_vertex2);
        this.m_v3.Copy(edgeA.m_vertex3);
        var hasVertex0 = edgeA.m_hasVertex0;
        var hasVertex3 = edgeA.m_hasVertex3;
        var edge1 = b2Math_1.b2Vec2.SubVV(this.m_v2, this.m_v1, b2EPCollider.s_edge1);
        edge1.Normalize();
        this.m_normal1.Set(edge1.y, -edge1.x);
        var offset1 = b2Math_1.b2Vec2.DotVV(this.m_normal1, b2Math_1.b2Vec2.SubVV(this.m_centroidB, this.m_v1, b2Math_1.b2Vec2.s_t0));
        var offset0 = 0;
        var offset2 = 0;
        var convex1 = false;
        var convex2 = false;
        // Is there a preceding edge?
        if (hasVertex0) {
            var edge0 = b2Math_1.b2Vec2.SubVV(this.m_v1, this.m_v0, b2EPCollider.s_edge0);
            edge0.Normalize();
            this.m_normal0.Set(edge0.y, -edge0.x);
            convex1 = b2Math_1.b2Vec2.CrossVV(edge0, edge1) >= 0;
            offset0 = b2Math_1.b2Vec2.DotVV(this.m_normal0, b2Math_1.b2Vec2.SubVV(this.m_centroidB, this.m_v0, b2Math_1.b2Vec2.s_t0));
        }
        // Is there a following edge?
        if (hasVertex3) {
            var edge2 = b2Math_1.b2Vec2.SubVV(this.m_v3, this.m_v2, b2EPCollider.s_edge2);
            edge2.Normalize();
            this.m_normal2.Set(edge2.y, -edge2.x);
            convex2 = b2Math_1.b2Vec2.CrossVV(edge1, edge2) > 0;
            offset2 = b2Math_1.b2Vec2.DotVV(this.m_normal2, b2Math_1.b2Vec2.SubVV(this.m_centroidB, this.m_v2, b2Math_1.b2Vec2.s_t0));
        }
        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3) {
            if (convex1 && convex2) {
                this.m_front = offset0 >= 0 || offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex1) {
                this.m_front = offset0 >= 0 || (offset1 >= 0 && offset2 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex2) {
                this.m_front = offset2 >= 0 || (offset0 >= 0 && offset1 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex0) {
            if (convex1) {
                this.m_front = offset0 >= 0 || offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex3) {
            if (convex2) {
                this.m_front = offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
            else {
                this.m_front = offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
        }
        else {
            this.m_front = offset1 >= 0;
            if (this.m_front) {
                this.m_normal.Copy(this.m_normal1);
                this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
            }
            else {
                this.m_normal.Copy(this.m_normal1).SelfNeg();
                this.m_lowerLimit.Copy(this.m_normal1);
                this.m_upperLimit.Copy(this.m_normal1);
            }
        }
        // Get polygonB in frameA
        this.m_polygonB.count = polygonB.m_count;
        for (var i = 0; i < polygonB.m_count; ++i) {
            b2Math_1.b2Transform.MulXV(this.m_xf, polygonB.m_vertices[i], this.m_polygonB.vertices[i]);
            b2Math_1.b2Rot.MulRV(this.m_xf.q, polygonB.m_normals[i], this.m_polygonB.normals[i]);
        }
        this.m_radius = 2 * b2Settings_1.b2_polygonRadius;
        manifold.pointCount = 0;
        var edgeAxis = this.ComputeEdgeSeparation(b2EPCollider.s_edgeAxis);
        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type === 0 /* e_unknown */) {
            return;
        }
        if (edgeAxis.separation > this.m_radius) {
            return;
        }
        var polygonAxis = this.ComputePolygonSeparation(b2EPCollider.s_polygonAxis);
        if (polygonAxis.type !== 0 /* e_unknown */ && polygonAxis.separation > this.m_radius) {
            return;
        }
        // Use hysteresis for jitter reduction.
        var k_relativeTol = 0.98;
        var k_absoluteTol = 0.001;
        var primaryAxis;
        if (polygonAxis.type === 0 /* e_unknown */) {
            primaryAxis = edgeAxis;
        }
        else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol) {
            primaryAxis = polygonAxis;
        }
        else {
            primaryAxis = edgeAxis;
        }
        var ie = b2EPCollider.s_ie;
        var rf = b2EPCollider.s_rf;
        if (primaryAxis.type === 1 /* e_edgeA */) {
            manifold.type = b2Collision_2.b2ManifoldType.e_faceA;
            // Search for the polygon normal that is most anti-parallel to the edge normal.
            var bestIndex = 0;
            var bestValue = b2Math_1.b2Vec2.DotVV(this.m_normal, this.m_polygonB.normals[0]);
            for (var i = 1; i < this.m_polygonB.count; ++i) {
                var value = b2Math_1.b2Vec2.DotVV(this.m_normal, this.m_polygonB.normals[i]);
                if (value < bestValue) {
                    bestValue = value;
                    bestIndex = i;
                }
            }
            var i1 = bestIndex;
            var i2 = (i1 + 1) % this.m_polygonB.count;
            var ie0 = ie[0];
            ie0.v.Copy(this.m_polygonB.vertices[i1]);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = i1;
            ie0.id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_face;
            ie0.id.cf.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
            var ie1 = ie[1];
            ie1.v.Copy(this.m_polygonB.vertices[i2]);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = i2;
            ie1.id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_face;
            ie1.id.cf.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
            if (this.m_front) {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1.Copy(this.m_v1);
                rf.v2.Copy(this.m_v2);
                rf.normal.Copy(this.m_normal1);
            }
            else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1.Copy(this.m_v2);
                rf.v2.Copy(this.m_v1);
                rf.normal.Copy(this.m_normal1).SelfNeg();
            }
        }
        else {
            manifold.type = b2Collision_2.b2ManifoldType.e_faceB;
            var ie0 = ie[0];
            ie0.v.Copy(this.m_v1);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = primaryAxis.index;
            ie0.id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_vertex;
            ie0.id.cf.typeB = b2Collision_1.b2ContactFeatureType.e_face;
            var ie1 = ie[1];
            ie1.v.Copy(this.m_v2);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = primaryAxis.index;
            ie1.id.cf.typeA = b2Collision_1.b2ContactFeatureType.e_vertex;
            ie1.id.cf.typeB = b2Collision_1.b2ContactFeatureType.e_face;
            rf.i1 = primaryAxis.index;
            rf.i2 = (rf.i1 + 1) % this.m_polygonB.count;
            rf.v1.Copy(this.m_polygonB.vertices[rf.i1]);
            rf.v2.Copy(this.m_polygonB.vertices[rf.i2]);
            rf.normal.Copy(this.m_polygonB.normals[rf.i1]);
        }
        rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2.Copy(rf.sideNormal1).SelfNeg();
        rf.sideOffset1 = b2Math_1.b2Vec2.DotVV(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = b2Math_1.b2Vec2.DotVV(rf.sideNormal2, rf.v2);
        // Clip incident edge against extruded edge1 side edges.
        var clipPoints1 = b2EPCollider.s_clipPoints1;
        var clipPoints2 = b2EPCollider.s_clipPoints2;
        var np = 0;
        // Clip to box side 1
        np = b2Collision_2.b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
        if (np < b2Settings_1.b2_maxManifoldPoints) {
            return;
        }
        // Clip to negative box side 1
        np = b2Collision_2.b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
        if (np < b2Settings_1.b2_maxManifoldPoints) {
            return;
        }
        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type === 1 /* e_edgeA */) {
            manifold.localNormal.Copy(rf.normal);
            manifold.localPoint.Copy(rf.v1);
        }
        else {
            manifold.localNormal.Copy(polygonB.m_normals[rf.i1]);
            manifold.localPoint.Copy(polygonB.m_vertices[rf.i1]);
        }
        var pointCount = 0;
        for (var i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            var separation = void 0;
            separation = b2Math_1.b2Vec2.DotVV(rf.normal, b2Math_1.b2Vec2.SubVV(clipPoints2[i].v, rf.v1, b2Math_1.b2Vec2.s_t0));
            if (separation <= this.m_radius) {
                var cp = manifold.points[pointCount];
                if (primaryAxis.type === 1 /* e_edgeA */) {
                    b2Math_1.b2Transform.MulTXV(this.m_xf, clipPoints2[i].v, cp.localPoint);
                    cp.id = clipPoints2[i].id;
                }
                else {
                    cp.localPoint.Copy(clipPoints2[i].v);
                    cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
                    cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
                    cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
                    cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
                }
                ++pointCount;
            }
        }
        manifold.pointCount = pointCount;
    };
    b2EPCollider.prototype.ComputeEdgeSeparation = function (out) {
        var axis = out;
        axis.type = 1 /* e_edgeA */;
        axis.index = this.m_front ? 0 : 1;
        axis.separation = b2Settings_1.b2_maxFloat;
        for (var i = 0; i < this.m_polygonB.count; ++i) {
            var s = b2Math_1.b2Vec2.DotVV(this.m_normal, b2Math_1.b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, b2Math_1.b2Vec2.s_t0));
            if (s < axis.separation) {
                axis.separation = s;
            }
        }
        return axis;
    };
    b2EPCollider.prototype.ComputePolygonSeparation = function (out) {
        var axis = out;
        axis.type = 0 /* e_unknown */;
        axis.index = -1;
        axis.separation = -b2Settings_1.b2_maxFloat;
        var perp = b2EPCollider.s_perp.Set(-this.m_normal.y, this.m_normal.x);
        for (var i = 0; i < this.m_polygonB.count; ++i) {
            var n = b2Math_1.b2Vec2.NegV(this.m_polygonB.normals[i], b2EPCollider.s_n);
            var s1 = b2Math_1.b2Vec2.DotVV(n, b2Math_1.b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, b2Math_1.b2Vec2.s_t0));
            var s2 = b2Math_1.b2Vec2.DotVV(n, b2Math_1.b2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v2, b2Math_1.b2Vec2.s_t0));
            var s = b2Math_1.b2Min(s1, s2);
            if (s > this.m_radius) {
                // No collision
                axis.type = 2 /* e_edgeB */;
                axis.index = i;
                axis.separation = s;
                return axis;
            }
            // Adjacency
            if (b2Math_1.b2Vec2.DotVV(n, perp) >= 0) {
                if (b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(n, this.m_upperLimit, b2Math_1.b2Vec2.s_t0), this.m_normal) < -b2Settings_1.b2_angularSlop) {
                    continue;
                }
            }
            else {
                if (b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(n, this.m_lowerLimit, b2Math_1.b2Vec2.s_t0), this.m_normal) < -b2Settings_1.b2_angularSlop) {
                    continue;
                }
            }
            if (s > axis.separation) {
                axis.type = 2 /* e_edgeB */;
                axis.index = i;
                axis.separation = s;
            }
        }
        return axis;
    };
    b2EPCollider.s_edge1 = new b2Math_1.b2Vec2();
    b2EPCollider.s_edge0 = new b2Math_1.b2Vec2();
    b2EPCollider.s_edge2 = new b2Math_1.b2Vec2();
    b2EPCollider.s_ie = b2Collision_2.b2ClipVertex.MakeArray(2);
    b2EPCollider.s_rf = new b2ReferenceFace();
    b2EPCollider.s_clipPoints1 = b2Collision_2.b2ClipVertex.MakeArray(2);
    b2EPCollider.s_clipPoints2 = b2Collision_2.b2ClipVertex.MakeArray(2);
    b2EPCollider.s_edgeAxis = new b2EPAxis();
    b2EPCollider.s_polygonAxis = new b2EPAxis();
    b2EPCollider.s_n = new b2Math_1.b2Vec2();
    b2EPCollider.s_perp = new b2Math_1.b2Vec2();
    return b2EPCollider;
}());
var b2CollideEdgeAndPolygon_s_collider = new b2EPCollider();
function b2CollideEdgeAndPolygon(manifold, edgeA, xfA, polygonB, xfB) {
    var collider = b2CollideEdgeAndPolygon_s_collider;
    collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
exports.b2CollideEdgeAndPolygon = b2CollideEdgeAndPolygon;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlRWRnZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9iMkNvbGxpZGVFZGdlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7O0FBQUEsMERBQTBEO0FBQzFELG1EQUFrSTtBQUNsSSwyQ0FBcUU7QUFDckUsNkNBQWtFO0FBQ2xFLDZDQUErRztBQUsvRyxJQUFNLDBCQUEwQixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDeEQsSUFBTSwwQkFBMEIsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO0FBQ3hELElBQU0sMEJBQTBCLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztBQUN4RCxJQUFNLDJCQUEyQixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDekQsSUFBTSwyQkFBMkIsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO0FBQ3pELElBQU0sMEJBQTBCLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztBQUN4RCxJQUFNLDBCQUEwQixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDeEQsSUFBTSwyQkFBMkIsR0FBZ0IsSUFBSSx5QkFBVyxFQUFFLENBQUM7QUFDbkUsZ0NBQXVDLFFBQW9CLEVBQUUsS0FBa0IsRUFBRSxHQUFnQixFQUFFLE9BQXNCLEVBQUUsR0FBZ0I7SUFDekksUUFBUSxDQUFDLFVBQVUsR0FBRyxDQUFDLENBQUM7SUFFeEIsa0NBQWtDO0lBQ2xDLElBQU0sQ0FBQyxHQUFXLG9CQUFXLENBQUMsTUFBTSxDQUFDLEdBQUcsRUFBRSxvQkFBVyxDQUFDLEtBQUssQ0FBQyxHQUFHLEVBQUUsT0FBTyxDQUFDLEdBQUcsRUFBRSxlQUFNLENBQUMsSUFBSSxDQUFDLEVBQUUsMEJBQTBCLENBQUMsQ0FBQztJQUV4SCxJQUFNLENBQUMsR0FBVyxLQUFLLENBQUMsU0FBUyxDQUFDO0lBQ2xDLElBQU0sQ0FBQyxHQUFXLEtBQUssQ0FBQyxTQUFTLENBQUM7SUFDbEMsSUFB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