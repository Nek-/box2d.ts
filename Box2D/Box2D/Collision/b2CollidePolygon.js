System.register(["../Common/b2Settings", "../Common/b2Math", "./b2Collision"], function (exports_1, context_1) {
    "use strict";
    var b2Settings_1, b2Math_1, b2Collision_1, b2Collision_2, b2EdgeSeparation_s_normal1World, b2EdgeSeparation_s_normal1, b2EdgeSeparation_s_v1, b2EdgeSeparation_s_v2, b2FindMaxSeparation_s_d, b2FindMaxSeparation_s_dLocal1, b2FindIncidentEdge_s_normal1, b2CollidePolygons_s_incidentEdge, b2CollidePolygons_s_clipPoints1, b2CollidePolygons_s_clipPoints2, b2CollidePolygons_s_edgeA, b2CollidePolygons_s_edgeB, b2CollidePolygons_s_localTangent, b2CollidePolygons_s_localNormal, b2CollidePolygons_s_planePoint, b2CollidePolygons_s_normal, b2CollidePolygons_s_tangent, b2CollidePolygons_s_ntangent, b2CollidePolygons_s_v11, b2CollidePolygons_s_v12;
    var __moduleName = context_1 && context_1.id;
    function b2EdgeSeparation(poly1, xf1, edge1, poly2, xf2) {
        ///const count1: number = poly1.m_count;
        const vertices1 = poly1.m_vertices;
        const normals1 = poly1.m_normals;
        const count2 = poly2.m_count;
        const vertices2 = poly2.m_vertices;
        ///b2Assert(0 <= edge1 && edge1 < count1);
        // Convert normal from poly1's frame into poly2's frame.
        const normal1World = b2Math_1.b2Rot.MulRV(xf1.q, normals1[edge1], b2EdgeSeparation_s_normal1World);
        const normal1 = b2Math_1.b2Rot.MulTRV(xf2.q, normal1World, b2EdgeSeparation_s_normal1);
        // Find support vertex on poly2 for -normal.
        let index = 0;
        let minDot = b2Settings_1.b2_maxFloat;
        for (let i = 0; i < count2; ++i) {
            const dot = b2Math_1.b2Vec2.DotVV(vertices2[i], normal1);
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }
        const v1 = b2Math_1.b2Transform.MulXV(xf1, vertices1[edge1], b2EdgeSeparation_s_v1);
        const v2 = b2Math_1.b2Transform.MulXV(xf2, vertices2[index], b2EdgeSeparation_s_v2);
        const separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(v2, v1, b2Math_1.b2Vec2.s_t0), normal1World);
        return separation;
    }
    function b2FindMaxSeparation(edgeIndex, poly1, xf1, poly2, xf2) {
        const count1 = poly1.m_count;
        const normals1 = poly1.m_normals;
        // Vector pointing from the centroid of poly1 to the centroid of poly2.
        const d = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Transform.MulXV(xf2, poly2.m_centroid, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Transform.MulXV(xf1, poly1.m_centroid, b2Math_1.b2Vec2.s_t1), b2FindMaxSeparation_s_d);
        const dLocal1 = b2Math_1.b2Rot.MulTRV(xf1.q, d, b2FindMaxSeparation_s_dLocal1);
        // Find edge normal on poly1 that has the largest projection onto d.
        let edge = 0;
        let maxDot = (-b2Settings_1.b2_maxFloat);
        for (let i = 0; i < count1; ++i) {
            const dot = b2Math_1.b2Vec2.DotVV(normals1[i], dLocal1);
            if (dot > maxDot) {
                maxDot = dot;
                edge = i;
            }
        }
        // Get the separation for the edge normal.
        let s = b2EdgeSeparation(poly1, xf1, edge, poly2, xf2);
        // Check the separation for the previous edge normal.
        const prevEdge = (edge + count1 - 1) % count1;
        const sPrev = b2EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
        // Check the separation for the next edge normal.
        const nextEdge = (edge + 1) % count1;
        const sNext = b2EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
        // Find the best edge and the search direction.
        let bestEdge = 0;
        let bestSeparation = 0;
        let increment = 0;
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
            if (increment === -1)
                edge = (bestEdge + count1 - 1) % count1;
            else
                edge = (bestEdge + 1) % count1;
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
    function b2FindIncidentEdge(c, poly1, xf1, edge1, poly2, xf2) {
        ///const count1: number = poly1.m_count;
        const normals1 = poly1.m_normals;
        const count2 = poly2.m_count;
        const vertices2 = poly2.m_vertices;
        const normals2 = poly2.m_normals;
        ///b2Assert(0 <= edge1 && edge1 < count1);
        // Get the normal of the reference edge in poly2's frame.
        const normal1 = b2Math_1.b2Rot.MulTRV(xf2.q, b2Math_1.b2Rot.MulRV(xf1.q, normals1[edge1], b2Math_1.b2Vec2.s_t0), b2FindIncidentEdge_s_normal1);
        // Find the incident edge on poly2.
        let index = 0;
        let minDot = b2Settings_1.b2_maxFloat;
        for (let i = 0; i < count2; ++i) {
            const dot = b2Math_1.b2Vec2.DotVV(normal1, normals2[i]);
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }
        // Build the clip vertices for the incident edge.
        const i1 = index;
        const i2 = (i1 + 1) % count2;
        const c0 = c[0];
        b2Math_1.b2Transform.MulXV(xf2, vertices2[i1], c0.v);
        const cf0 = c0.id.cf;
        cf0.indexA = edge1;
        cf0.indexB = i1;
        cf0.typeA = b2Collision_1.b2ContactFeatureType.e_face;
        cf0.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
        const c1 = c[1];
        b2Math_1.b2Transform.MulXV(xf2, vertices2[i2], c1.v);
        const cf1 = c1.id.cf;
        cf1.indexA = edge1;
        cf1.indexB = i2;
        cf1.typeA = b2Collision_1.b2ContactFeatureType.e_face;
        cf1.typeB = b2Collision_1.b2ContactFeatureType.e_vertex;
    }
    function b2CollidePolygons(manifold, polyA, xfA, polyB, xfB) {
        manifold.pointCount = 0;
        const totalRadius = polyA.m_radius + polyB.m_radius;
        const edgeA = b2CollidePolygons_s_edgeA;
        edgeA[0] = 0;
        const separationA = b2FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
        if (separationA > totalRadius)
            return;
        const edgeB = b2CollidePolygons_s_edgeB;
        edgeB[0] = 0;
        const separationB = b2FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
        if (separationB > totalRadius)
            return;
        let poly1; // reference polygon
        let poly2; // incident polygon
        let xf1, xf2;
        let edge1 = 0; // reference edge
        let flip = 0;
        const k_relativeTol = 0.98;
        const k_absoluteTol = 0.001;
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
        const incidentEdge = b2CollidePolygons_s_incidentEdge;
        b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
        const count1 = poly1.m_count;
        const vertices1 = poly1.m_vertices;
        const iv1 = edge1;
        const iv2 = (edge1 + 1) % count1;
        const local_v11 = vertices1[iv1];
        const local_v12 = vertices1[iv2];
        const localTangent = b2Math_1.b2Vec2.SubVV(local_v12, local_v11, b2CollidePolygons_s_localTangent);
        localTangent.Normalize();
        const localNormal = b2Math_1.b2Vec2.CrossVOne(localTangent, b2CollidePolygons_s_localNormal);
        const planePoint = b2Math_1.b2Vec2.MidVV(local_v11, local_v12, b2CollidePolygons_s_planePoint);
        const tangent = b2Math_1.b2Rot.MulRV(xf1.q, localTangent, b2CollidePolygons_s_tangent);
        const normal = b2Math_1.b2Vec2.CrossVOne(tangent, b2CollidePolygons_s_normal);
        const v11 = b2Math_1.b2Transform.MulXV(xf1, local_v11, b2CollidePolygons_s_v11);
        const v12 = b2Math_1.b2Transform.MulXV(xf1, local_v12, b2CollidePolygons_s_v12);
        // Face offset.
        const frontOffset = b2Math_1.b2Vec2.DotVV(normal, v11);
        // Side offsets, extended by polytope skin thickness.
        const sideOffset1 = -b2Math_1.b2Vec2.DotVV(tangent, v11) + totalRadius;
        const sideOffset2 = b2Math_1.b2Vec2.DotVV(tangent, v12) + totalRadius;
        // Clip incident edge against extruded edge1 side edges.
        const clipPoints1 = b2CollidePolygons_s_clipPoints1;
        const clipPoints2 = b2CollidePolygons_s_clipPoints2;
        let np;
        // Clip to box side 1
        const ntangent = b2Math_1.b2Vec2.NegV(tangent, b2CollidePolygons_s_ntangent);
        np = b2Collision_2.b2ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1);
        if (np < 2)
            return;
        // Clip to negative box side 1
        np = b2Collision_2.b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);
        if (np < 2) {
            return;
        }
        // Now clipPoints2 contains the clipped points.
        manifold.localNormal.Copy(localNormal);
        manifold.localPoint.Copy(planePoint);
        let pointCount = 0;
        for (let i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            const cv = clipPoints2[i];
            const separation = b2Math_1.b2Vec2.DotVV(normal, cv.v) - frontOffset;
            if (separation <= totalRadius) {
                const cp = manifold.points[pointCount];
                b2Math_1.b2Transform.MulTXV(xf2, cv.v, cp.localPoint);
                cp.id.Copy(cv.id);
                if (flip) {
                    // Swap features
                    const cf = cp.id.cf;
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
    exports_1("b2CollidePolygons", b2CollidePolygons);
    return {
        setters: [
            function (b2Settings_1_1) {
                b2Settings_1 = b2Settings_1_1;
            },
            function (b2Math_1_1) {
                b2Math_1 = b2Math_1_1;
            },
            function (b2Collision_1_1) {
                b2Collision_1 = b2Collision_1_1;
                b2Collision_2 = b2Collision_1_1;
            }
        ],
        execute: function () {
            b2EdgeSeparation_s_normal1World = new b2Math_1.b2Vec2();
            b2EdgeSeparation_s_normal1 = new b2Math_1.b2Vec2();
            b2EdgeSeparation_s_v1 = new b2Math_1.b2Vec2();
            b2EdgeSeparation_s_v2 = new b2Math_1.b2Vec2();
            b2FindMaxSeparation_s_d = new b2Math_1.b2Vec2();
            b2FindMaxSeparation_s_dLocal1 = new b2Math_1.b2Vec2();
            b2FindIncidentEdge_s_normal1 = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_incidentEdge = b2Collision_2.b2ClipVertex.MakeArray(2);
            b2CollidePolygons_s_clipPoints1 = b2Collision_2.b2ClipVertex.MakeArray(2);
            b2CollidePolygons_s_clipPoints2 = b2Collision_2.b2ClipVertex.MakeArray(2);
            b2CollidePolygons_s_edgeA = [0];
            b2CollidePolygons_s_edgeB = [0];
            b2CollidePolygons_s_localTangent = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_localNormal = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_planePoint = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_normal = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_tangent = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_ntangent = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_v11 = new b2Math_1.b2Vec2();
            b2CollidePolygons_s_v12 = new b2Math_1.b2Vec2();
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlUG9seWdvbi5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbImIyQ29sbGlkZVBvbHlnb24udHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7OztJQVVBLDBCQUEwQixLQUFxQixFQUFFLEdBQWdCLEVBQUUsS0FBYSxFQUFFLEtBQXFCLEVBQUUsR0FBZ0I7UUFDdkgsd0NBQXdDO1FBQ3hDLE1BQU0sU0FBUyxHQUFhLEtBQUssQ0FBQyxVQUFVLENBQUM7UUFDN0MsTUFBTSxRQUFRLEdBQWEsS0FBSyxDQUFDLFNBQVMsQ0FBQztRQUUzQyxNQUFNLE1BQU0sR0FBVyxLQUFLLENBQUMsT0FBTyxDQUFDO1FBQ3JDLE1BQU0sU0FBUyxHQUFhLEtBQUssQ0FBQyxVQUFVLENBQUM7UUFFN0MsMENBQTBDO1FBRTFDLHdEQUF3RDtRQUN4RCxNQUFNLFlBQVksR0FBVyxjQUFLLENBQUMsS0FBSyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsUUFBUSxDQUFDLEtBQUssQ0FBQyxFQUFFLCtCQUErQixDQUFDLENBQUM7UUFDbEcsTUFBTSxPQUFPLEdBQVcsY0FBSyxDQUFDLE1BQU0sQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLFlBQVksRUFBRSwwQkFBMEIsQ0FBQyxDQUFDO1FBRXRGLDRDQUE0QztRQUM1QyxJQUFJLEtBQUssR0FBVyxDQUFDLENBQUM7UUFDdEIsSUFBSSxNQUFNLEdBQVcsd0JBQVcsQ0FBQztRQUVqQyxLQUFLLElBQUksQ0FBQyxHQUFXLENBQUMsRUFBRSxDQUFDLEdBQUcsTUFBTSxFQUFFLEVBQUUsQ0FBQyxFQUFFO1lBQ3ZDLE1BQU0sR0FBRyxHQUFXLGVBQU0sQ0FBQyxLQUFLLENBQUMsU0FBUyxDQUFDLENBQUMsQ0FBQyxFQUFFLE9BQU8sQ0FBQyxDQUFDO1lBQ3hELElBQUksR0FBRyxHQUFHLE1BQU0sRUFBRTtnQkFDaEIsTUFBTSxHQUFHLEdBQUcsQ0FBQztnQkFDYixLQUFLLEdBQUcsQ0FBQyxDQUFDO2FBQ1g7U0FDRjtRQUVE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