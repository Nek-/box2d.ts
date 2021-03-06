System.register(["../Common/b2Settings", "../Common/b2Math", "./b2Collision"], function (exports_1, context_1) {
    "use strict";
    var b2Settings_1, b2Math_1, b2Collision_1, b2CollideCircles_s_pA, b2CollideCircles_s_pB, b2CollidePolygonAndCircle_s_c, b2CollidePolygonAndCircle_s_cLocal, b2CollidePolygonAndCircle_s_faceCenter;
    var __moduleName = context_1 && context_1.id;
    function b2CollideCircles(manifold, circleA, xfA, circleB, xfB) {
        manifold.pointCount = 0;
        const pA = b2Math_1.b2Transform.MulXV(xfA, circleA.m_p, b2CollideCircles_s_pA);
        const pB = b2Math_1.b2Transform.MulXV(xfB, circleB.m_p, b2CollideCircles_s_pB);
        const distSqr = b2Math_1.b2Vec2.DistanceSquaredVV(pA, pB);
        const radius = circleA.m_radius + circleB.m_radius;
        if (distSqr > radius * radius) {
            return;
        }
        manifold.type = b2Collision_1.b2ManifoldType.e_circles;
        manifold.localPoint.Copy(circleA.m_p);
        manifold.localNormal.SetZero();
        manifold.pointCount = 1;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    exports_1("b2CollideCircles", b2CollideCircles);
    function b2CollidePolygonAndCircle(manifold, polygonA, xfA, circleB, xfB) {
        manifold.pointCount = 0;
        // Compute circle position in the frame of the polygon.
        const c = b2Math_1.b2Transform.MulXV(xfB, circleB.m_p, b2CollidePolygonAndCircle_s_c);
        const cLocal = b2Math_1.b2Transform.MulTXV(xfA, c, b2CollidePolygonAndCircle_s_cLocal);
        // Find the min separating edge.
        let normalIndex = 0;
        let separation = (-b2Settings_1.b2_maxFloat);
        const radius = polygonA.m_radius + circleB.m_radius;
        const vertexCount = polygonA.m_count;
        const vertices = polygonA.m_vertices;
        const normals = polygonA.m_normals;
        for (let i = 0; i < vertexCount; ++i) {
            const s = b2Math_1.b2Vec2.DotVV(normals[i], b2Math_1.b2Vec2.SubVV(cLocal, vertices[i], b2Math_1.b2Vec2.s_t0));
            if (s > radius) {
                // Early out.
                return;
            }
            if (s > separation) {
                separation = s;
                normalIndex = i;
            }
        }
        // Vertices that subtend the incident face.
        const vertIndex1 = normalIndex;
        const vertIndex2 = (vertIndex1 + 1) % vertexCount;
        const v1 = vertices[vertIndex1];
        const v2 = vertices[vertIndex2];
        // If the center is inside the polygon ...
        if (separation < b2Settings_1.b2_epsilon) {
            manifold.pointCount = 1;
            manifold.type = b2Collision_1.b2ManifoldType.e_faceA;
            manifold.localNormal.Copy(normals[normalIndex]);
            b2Math_1.b2Vec2.MidVV(v1, v2, manifold.localPoint);
            manifold.points[0].localPoint.Copy(circleB.m_p);
            manifold.points[0].id.key = 0;
            return;
        }
        // Compute barycentric coordinates
        const u1 = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cLocal, v1, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(v2, v1, b2Math_1.b2Vec2.s_t1));
        const u2 = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cLocal, v2, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(v1, v2, b2Math_1.b2Vec2.s_t1));
        if (u1 <= 0) {
            if (b2Math_1.b2Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
                return;
            }
            manifold.pointCount = 1;
            manifold.type = b2Collision_1.b2ManifoldType.e_faceA;
            b2Math_1.b2Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize();
            manifold.localPoint.Copy(v1);
            manifold.points[0].localPoint.Copy(circleB.m_p);
            manifold.points[0].id.key = 0;
        }
        else if (u2 <= 0) {
            if (b2Math_1.b2Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
                return;
            }
            manifold.pointCount = 1;
            manifold.type = b2Collision_1.b2ManifoldType.e_faceA;
            b2Math_1.b2Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize();
            manifold.localPoint.Copy(v2);
            manifold.points[0].localPoint.Copy(circleB.m_p);
            manifold.points[0].id.key = 0;
        }
        else {
            const faceCenter = b2Math_1.b2Vec2.MidVV(v1, v2, b2CollidePolygonAndCircle_s_faceCenter);
            separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cLocal, faceCenter, b2Math_1.b2Vec2.s_t1), normals[vertIndex1]);
            if (separation > radius) {
                return;
            }
            manifold.pointCount = 1;
            manifold.type = b2Collision_1.b2ManifoldType.e_faceA;
            manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize();
            manifold.localPoint.Copy(faceCenter);
            manifold.points[0].localPoint.Copy(circleB.m_p);
            manifold.points[0].id.key = 0;
        }
    }
    exports_1("b2CollidePolygonAndCircle", b2CollidePolygonAndCircle);
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
            }
        ],
        execute: function () {
            b2CollideCircles_s_pA = new b2Math_1.b2Vec2();
            b2CollideCircles_s_pB = new b2Math_1.b2Vec2();
            b2CollidePolygonAndCircle_s_c = new b2Math_1.b2Vec2();
            b2CollidePolygonAndCircle_s_cLocal = new b2Math_1.b2Vec2();
            b2CollidePolygonAndCircle_s_faceCenter = new b2Math_1.b2Vec2();
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaWRlQ2lyY2xlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiYjJDb2xsaWRlQ2lyY2xlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7Ozs7SUFRQSwwQkFBaUMsUUFBb0IsRUFBRSxPQUFzQixFQUFFLEdBQWdCLEVBQUUsT0FBc0IsRUFBRSxHQUFnQjtRQUN2SSxRQUFRLENBQUMsVUFBVSxHQUFHLENBQUMsQ0FBQztRQUV4QixNQUFNLEVBQUUsR0FBVyxvQkFBVyxDQUFDLEtBQUssQ0FBQyxHQUFHLEVBQUUsT0FBTyxDQUFDLEdBQUcsRUFBRSxxQkFBcUIsQ0FBQyxDQUFDO1FBQzlFLE1BQU0sRUFBRSxHQUFXLG9CQUFXLENBQUMsS0FBSyxDQUFDLEdBQUcsRUFBRSxPQUFPLENBQUMsR0FBRyxFQUFFLHFCQUFxQixDQUFDLENBQUM7UUFFOUUsTUFBTSxPQUFPLEdBQVcsZUFBTSxDQUFDLGlCQUFpQixDQUFDLEVBQUUsRUFBRSxFQUFFLENBQUMsQ0FBQztRQUN6RCxNQUFNLE1BQU0sR0FBVyxPQUFPLENBQUMsUUFBUSxHQUFHLE9BQU8sQ0FBQyxRQUFRLENBQUM7UUFDM0QsSUFBSSxPQUFPLEdBQUcsTUFBTSxHQUFHLE1BQU0sRUFBRTtZQUM3QixPQUFPO1NBQ1I7UUFFRCxRQUFRLENBQUMsSUFBSSxHQUFHLDRCQUFjLENBQUMsU0FBUyxDQUFDO1FBQ3pDLFFBQVEsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUN0QyxRQUFRLENBQUMsV0FBVyxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQy9CLFFBQVEsQ0FBQyxVQUFVLEdBQUcsQ0FBQyxDQUFDO1FBRXhCLFFBQVEsQ0FBQyxNQUFNLENBQUMsQ0FBQyxDQUFDLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsR0FBRyxDQUFDLENBQUM7UUFDaEQsUUFBUSxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMs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