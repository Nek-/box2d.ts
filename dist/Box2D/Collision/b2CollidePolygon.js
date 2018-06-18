"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Collision_1 = require("./b2Collision");
var b2Collision_2 = require("./b2Collision");
var b2EdgeSeparation_s_normal1World = new b2Math_1.b2Vec2();
var b2EdgeSeparation_s_normal1 = new b2Math_1.b2Vec2();
var b2EdgeSeparation_s_v1 = new b2Math_1.b2Vec2();
var b2EdgeSeparation_s_v2 = new b2Math_1.b2Vec2();
function b2EdgeSeparation(poly1, xf1, edge1, poly2, xf2) {
    // DEBUG: const count1: number = poly1.m_count;
    var vertices1 = poly1.m_vertices;
    var normals1 = poly1.m_normals;
    var count2 = poly2.m_count;
    var vertices2 = poly2.m_vertices;
    // DEBUG: b2Assert(0 <= edge1 && edge1 < count1);
    // Convert normal from poly1's frame into poly2's frame.
    var normal1World = b2Math_1.b2Rot.MulRV(xf1.q, normals1[edge1], b2EdgeSeparation_s_normal1World);
    var normal1 = b2Math_1.b2Rot.MulTRV(xf2.q, normal1World, b2EdgeSeparation_s_normal1);
    // Find support vertex on poly2 for -normal.
    var index = 0;
    var minDot = b2Settings_1.b2_maxFloat;
    for (var i = 0; i < count2; ++i) {
        var dot = b2Math_1.b2Vec2.DotVV(vertices2[i], normal1);
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }
    var v1 = b2Math_1.b2Transform.MulXV(xf1, vertices1[edge1], b2EdgeSeparation_s_v1);
    var v2 = b2Math_1.b2Transform.MulXV(xf2, vertices2[index], b2EdgeSeparation_s_v2);
    var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(v2, v1, b2Math_1.b2Vec2.s_t0), normal1World);
    return separation;
}
var b2FindMaxSeparation_s_d = new b2Math_1.b2Vec2();
var b2FindMaxSeparation_s_dLocal1 = new b2Math_1.b2Vec2();
function b2FindMaxSeparation(edgeIndex, poly1, xf1, poly2, xf2) {
    var count1 = poly1.m_count;
    var normals1 = poly1.m_normals;
    // Vector pointing from the centroid of poly1 to the centroid of poly2.
    var d = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Transform.MulXV(xf2, poly2.m_centroid, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Transform.MulXV(xf1, poly1.m_centroid, b2Math_1.b2Vec2.s_t1), b2FindMaxSeparation_s_d);
    var dLocal1 = b2Math_1.b2Rot.MulTRV(xf1.q, d, b2FindMaxSeparation_s_dLocal1);
    // Find edge normal on poly1 that has the largest projection onto d.
    var edge = 0;
    var maxDot = (-b2Settings_1.b2_maxFloat);
    for (var i = 0; i < count1; ++i) {
        var dot = b2Math_1.b2Vec2.DotVV(normals1[i], dLocal1);
        if (dot > maxDot) {
            maxDot = dot;
            edge = i;
        }
    }
    // Get the separation for the edge normal.
    var s = b2EdgeSeparation(poly1, xf1, edge, poly2, xf2);
    // Check the separation for the previous edge normal.
    var prevEdge = (edge + count1 - 1) % count1;
    var sPrev = b2EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
    // Check the separation for the next edge normal.
    var nextEdge = (edge + 1) % count1;
    var sNext = b2EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
    // Find the best edge and the search direction.
    var bestEdge = 0;
    var bestSeparation = 0;
    var increment = 0;
    if (sPrev > s && sPrev > sNext) {
        increment = -1;
        bestEdge = prevEdge;
        bestSeparation = sPrev;
    }
    else if (sNext > s) {
        increment = 1;
        bestEdge = nextEdge;
        bestSeparation = sNext;
    }
    else {
        edgeIndex[0] = edge;
        return s;
    }
    // Perform a local search for the best edge normal.
    while (true) {
        if (increment === -1) {
            edge = (bestEdge + count1 - 1) % count1;
        }
        else {
            edge = (bestEdge + 1) % count1;
        }
        s = b2EdgeSeparation(poly1, xf1, edge, poly2, xf2);
        if (s > bestSeparation) {
            bestEdge = edge;
            bestSeparation = s;
        }
        else {
            break;
        }
    }
    edgeIndex[0] = bestEdge;
    return bestSeparation;
}
var b2FindIncidentEdge_s_normal1 = new b2Math_1.b2Vec2();
function b2FindIncidentEdge(c, poly1, xf1, edge1, poly2, xf2) {
    // DEBUG: const count1: number = poly1.m_count;
    var normals1 = poly1.m_normals;
    var count2 = poly2.m_count;
    var vertices2 = poly2.m_vertices;
    var normals2 = poly2.m_normals;
    // DEBUG: b2Assert(0 <= edge1 && edge1 < count1);
    // Get the normal of the reference edge in poly2's frame.
    var normal1 = b2Math_1.b2Rot.MulTRV(xf2.q, b2Math_1.b2Rot.MulRV(xf1.q, normals1[edge1], b2Math_1.b2Vec2.s_t0), b2FindIncidentEdge_s_normal1);
    // Find the incident edge on poly2.
    var index = 0;
    var minDot = b2Settings_1.b2_maxFloat;
    for (var i = 0; i < count2; ++i) {
        var dot = b2Math_1.b2Vec2.DotVV(normal1, normals2[i]);
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }
    // Build the clip vertices for the incident edge.
    var i1 = index;
    var i2 = (i1 + 1) % count2;
    var c0 = c[0];
    b2Math_1.b2Transform.MulXV(xf2, vertices2[i1], c0.v);
    var cf0 = c0.id.cf;
    cf0.indexA = edge1;
    cf0.indexB = i1;
    cf0.typeA = b2Collision_1.b2ContactFeatureType.e_face;
    cf0.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
    var c1 = c[1];
    b2Math_1.b2Transform.MulXV(xf2, vertices2[i2], c1.v);
    var cf1 = c1.id.cf;
    cf1.indexA = edge1;
    cf1.indexB = i2;
    cf1.typeA = b2Collision_1.b2ContactFeatureType.e_face;
    cf1.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
}
var b2CollidePolygons_s_incidentEdge = b2Collision_2.b2ClipVertex.MakeArray(2);
var b2CollidePolygons_s_clipPoints1 = b2Collision_2.b2ClipVertex.MakeArray(2);
var b2CollidePolygons_s_clipPoints2 = b2Collision_2.b2ClipVertex.MakeArray(2);
var b2CollidePolygons_s_edgeA = [0];
var b2CollidePolygons_s_edgeB = [0];
var b2CollidePolygons_s_localTangent = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_localNormal = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_planePoint = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_normal = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_tangent = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_ntangent = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_v11 = new b2Math_1.b2Vec2();
var b2CollidePolygons_s_v12 = new b2Math_1.b2Vec2();
function b2CollidePolygons(manifold, polyA, xfA, polyB, xfB) {
    manifold.pointCount = 0;
    var totalRadius = polyA.m_radius + polyB.m_radius;
    var edgeA = b2CollidePolygons_s_edgeA;
    edgeA[0] = 0;
    var separationA = b2FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
    if (separationA > totalRadius) {
        return;
    }
    var edgeB = b2CollidePolygons_s_edgeB;
    edgeB[0] = 0;
    var separationB = b2FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
    if (separationB > totalRadius) {
        return;
    }
    var poly1; // reference polygon
    var poly2; // incident polygon
    var xf1, xf2;
    var edge1 = 0; // reference edge
    var flip = 0;
    var k_relativeTol = 0.98;
    var k_absoluteTol = 0.001;
    if (separationB > k_relativeTol * separationA + k_absoluteTol) {
        poly1 = polyB;
        poly2 = polyA;
        xf1 = xfB;
        xf2 = xfA;
        edge1 = edgeB[0];
        manifold.type = b2Collision_2.b2ManifoldType.e_faceB;
        flip = 1;
    }
    else {
        poly1 = polyA;
        poly2 = polyB;
        xf1 = xfA;
        xf2 = xfB;
        edge1 = edgeA[0];
        manifold.type = b2Collision_2.b2ManifoldType.e_faceA;
        flip = 0;
    }
    var incidentEdge = b2CollidePolygons_s_incidentEdge;
    b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
    var count1 = poly1.m_count;
    var vertices1 = poly1.m_vertices;
    var iv1 = edge1;
    var iv2 = (edge1 + 1) % count1;
    var local_v11 = vertices1[iv1];
    var local_v12 = vertices1[iv2];
    var localTangent = b2Math_1.b2Vec2.SubVV(local_v12, local_v11, b2CollidePolygons_s_localTangent);
    localTangent.Normalize();
    var localNormal = b2Math_1.b2Vec2.CrossVOne(localTangent, b2CollidePolygons_s_localNormal);
    var planePoint = b2Math_1.b2Vec2.MidVV(local_v11, local_v12, b2CollidePolygons_s_planePoint);
    var tangent = b2Math_1.b2Rot.MulRV(xf1.q, localTangent, b2CollidePolygons_s_tangent);
    var normal = b2Math_1.b2Vec2.CrossVOne(tangent, b2CollidePolygons_s_normal);
    var v11 = b2Math_1.b2Transform.MulXV(xf1, local_v11, b2CollidePolygons_s_v11);
    var v12 = b2Math_1.b2Transform.MulXV(xf1, local_v12, b2CollidePolygons_s_v12);
    // Face offset.
    var frontOffset = b2Math_1.b2Vec2.DotVV(normal, v11);
    // Side offsets, extended by polytope skin thickness.
    var sideOffset1 = -b2Math_1.b2Vec2.DotVV(tangent, v11) + totalRadius;
    var sideOffset2 = b2Math_1.b2Vec2.DotVV(tangent, v12) + totalRadius;
    // Clip incident edge against extruded edge1 side edges.
    var clipPoints1 = b2CollidePolygons_s_clipPoints1;
    var clipPoints2 = b2CollidePolygons_s_clipPoints2;
    var np;
    // Clip to box side 1
    var ntangent = b2Math_1.b2Vec2.NegV(tangent, b2CollidePolygons_s_ntangent);
    np = b2Collision_2.b2ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1);
    if (np < 2) {
        return;
    }
    // Clip to negative box side 1
    np = b2Collision_2.b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);
    if (np < 2) {
        return;
    }
    // Now clipPoints2 contains the clipped points.
    manifold.localNormal.Copy(localNormal);
    manifold.localPoint.Copy(planePoint);
    var pointCount = 0;
    for (var i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
        var cv = clipPoints2[i];
        var separation = b2Math_1.b2Vec2.DotVV(normal, cv.v) - frontOffset;
        if (separation <= totalRadius) {
            var cp = manifold.points[pointCount];
            b2Math_1.b2Transform.MulTXV(xf2, cv.v, cp.localPoint);
            cp.id.Copy(cv.id);
            if (flip) {
                // Swap features
                var cf = cp.id.cf;
                cp.id.cf.indexA = cf.indexB;
                cp.id.cf.indexB = cf.indexA;
                cp.id.cf.typeA = cf.typeB;
                cp.id.cf.typeB = cf.typeA;
            }
            ++pointCount;
        }
    }
    manifold.pointCount = pointCount;
}
exports.b2CollidePolygons = b2CollidePolygons;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlUG9seWdvbi5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9iMkNvbGxpZGVQb2x5Z29uLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7O0FBQUEsMERBQTBEO0FBQzFELG1EQUF5RTtBQUN6RSwyQ0FBOEQ7QUFDOUQsNkNBQXVFO0FBQ3ZFLDZDQUErRztBQUcvRyxJQUFNLCtCQUErQixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDN0QsSUFBTSwwQkFBMEIsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO0FBQ3hELElBQU0scUJBQXFCLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztBQUNuRCxJQUFNLHFCQUFxQixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDbkQsMEJBQTBCLEtBQXFCLEVBQUUsR0FBZ0IsRUFBRSxLQUFhLEVBQUUsS0FBcUIsRUFBRSxHQUFnQjtJQUN2SCwrQ0FBK0M7SUFDL0MsSUFBTSxTQUFTLEdBQWEsS0FBSyxDQUFDLFVBQVUsQ0FBQztJQUM3QyxJQUFNLFFBQVEsR0FBYSxLQUFLLENBQUMsU0FBUyxDQUFDO0lBRTNDLElBQU0sTUFBTSxHQUFXLEtBQUssQ0FBQyxPQUFPLENBQUM7SUFDckMsSUFBTSxTQUFTLEdBQWEsS0FBSyxDQUFDLFVBQVUsQ0FBQztJQUU3QyxpREFBaUQ7SUFFakQsd0RBQXdEO0lBQ3hELElBQU0sWUFBWSxHQUFXLGNBQUssQ0FBQyxLQUFLLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxRQUFRLENBQUMsS0FBSyxDQUFDLEVBQUUsK0JBQStCLENBQUMsQ0FBQztJQUNsRyxJQUFNLE9BQU8sR0FBVyxjQUFLLENBQUMsTUFBTSxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsWUFBWSxFQUFFLDBCQUEwQixDQUFDLENBQUM7SUFFdEYsNENBQTRDO0lBQzVDLElBQUksS0FBSyxHQUFXLENBQUMsQ0FBQztJQUN0QixJQUFJLE1BQU0s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