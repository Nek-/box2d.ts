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
System.register(["../Common/b2Settings", "../Common/b2Math", "./b2Distance"], function (exports_1, context_1) {
    "use strict";
    var b2Settings_1, b2Math_1, b2Distance_1, b2ContactFeatureType, b2ContactFeature, b2ContactID, b2ManifoldPoint, b2ManifoldType, b2Manifold, b2WorldManifold, b2PointState, b2ClipVertex, b2RayCastInput, b2RayCastOutput, b2AABB, b2TestOverlapShape_s_input, b2TestOverlapShape_s_simplexCache, b2TestOverlapShape_s_output;
    var __moduleName = context_1 && context_1.id;
    /// Compute the point states given two manifolds. The states pertain to the transition from manifold1
    /// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
    function b2GetPointStates(state1, state2, manifold1, manifold2) {
        // Detect persists and removes.
        let i;
        for (i = 0; i < manifold1.pointCount; ++i) {
            const id = manifold1.points[i].id;
            const key = id.key;
            state1[i] = b2PointState.b2_removeState;
            for (let j = 0, jct = manifold2.pointCount; j < jct; ++j) {
                if (manifold2.points[j].id.key === key) {
                    state1[i] = b2PointState.b2_persistState;
                    break;
                }
            }
        }
        for (; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            state1[i] = b2PointState.b2_nullState;
        }
        // Detect persists and adds.
        for (i = 0; i < manifold2.pointCount; ++i) {
            const id = manifold2.points[i].id;
            const key = id.key;
            state2[i] = b2PointState.b2_addState;
            for (let j = 0, jct = manifold1.pointCount; j < jct; ++j) {
                if (manifold1.points[j].id.key === key) {
                    state2[i] = b2PointState.b2_persistState;
                    break;
                }
            }
        }
        for (; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            state2[i] = b2PointState.b2_nullState;
        }
    }
    exports_1("b2GetPointStates", b2GetPointStates);
    function b2TestOverlapAABB(a, b) {
        const d1_x = b.lowerBound.x - a.upperBound.x;
        const d1_y = b.lowerBound.y - a.upperBound.y;
        const d2_x = a.lowerBound.x - b.upperBound.x;
        const d2_y = a.lowerBound.y - b.upperBound.y;
        if (d1_x > 0 || d1_y > 0)
            return false;
        if (d2_x > 0 || d2_y > 0)
            return false;
        return true;
    }
    exports_1("b2TestOverlapAABB", b2TestOverlapAABB);
    /// Clipping for contact manifolds.
    function b2ClipSegmentToLine(vOut, vIn, normal, offset, vertexIndexA) {
        // Start with no output points
        let numOut = 0;
        const vIn0 = vIn[0];
        const vIn1 = vIn[1];
        // Calculate the distance of end points to the line
        const distance0 = b2Math_1.b2Vec2.DotVV(normal, vIn0.v) - offset;
        const distance1 = b2Math_1.b2Vec2.DotVV(normal, vIn1.v) - offset;
        // If the points are behind the plane
        if (distance0 <= 0)
            vOut[numOut++].Copy(vIn0);
        if (distance1 <= 0)
            vOut[numOut++].Copy(vIn1);
        // If the points are on different sides of the plane
        if (distance0 * distance1 < 0) {
            // Find intersection point of edge and plane
            const interp = distance0 / (distance0 - distance1);
            const v = vOut[numOut].v;
            v.x = vIn0.v.x + interp * (vIn1.v.x - vIn0.v.x);
            v.y = vIn0.v.y + interp * (vIn1.v.y - vIn0.v.y);
            // VertexA is hitting edgeB.
            const id = vOut[numOut].id;
            id.cf.indexA = vertexIndexA;
            id.cf.indexB = vIn0.id.cf.indexB;
            id.cf.typeA = b2ContactFeatureType.e_vertex;
            id.cf.typeB = b2ContactFeatureType.e_face;
            ++numOut;
        }
        return numOut;
    }
    exports_1("b2ClipSegmentToLine", b2ClipSegmentToLine);
    function b2TestOverlapShape(shapeA, indexA, shapeB, indexB, xfA, xfB) {
        const input = b2TestOverlapShape_s_input.Reset();
        input.proxyA.SetShape(shapeA, indexA);
        input.proxyB.SetShape(shapeB, indexB);
        input.transformA.Copy(xfA);
        input.transformB.Copy(xfB);
        input.useRadii = true;
        const simplexCache = b2TestOverlapShape_s_simplexCache.Reset();
        simplexCache.count = 0;
        const output = b2TestOverlapShape_s_output.Reset();
        b2Distance_1.b2Distance(output, simplexCache, input);
        return output.distance < 10 * b2Settings_1.b2_epsilon;
    }
    exports_1("b2TestOverlapShape", b2TestOverlapShape);
    return {
        setters: [
            function (b2Settings_1_1) {
                b2Settings_1 = b2Settings_1_1;
            },
            function (b2Math_1_1) {
                b2Math_1 = b2Math_1_1;
            },
            function (b2Distance_1_1) {
                b2Distance_1 = b2Distance_1_1;
            }
        ],
        execute: function () {
            /// @file
            /// Structures and functions used for computing contact points, distance
            /// queries, and TOI queries.
            (function (b2ContactFeatureType) {
                b2ContactFeatureType[b2ContactFeatureType["e_vertex"] = 0] = "e_vertex";
                b2ContactFeatureType[b2ContactFeatureType["e_face"] = 1] = "e_face";
            })(b2ContactFeatureType || (b2ContactFeatureType = {}));
            exports_1("b2ContactFeatureType", b2ContactFeatureType);
            /// The features that intersect to form the contact point
            /// This must be 4 bytes or less.
            b2ContactFeature = class b2ContactFeature {
                constructor() {
                    this._key = 0;
                    this._key_invalid = false;
                    this._indexA = 0;
                    this._indexB = 0;
                    this._typeA = 0;
                    this._typeB = 0;
                }
                get key() {
                    if (this._key_invalid) {
                        this._key_invalid = false;
                        this._key = this._indexA | (this._indexB << 8) | (this._typeA << 16) | (this._typeB << 24);
                    }
                    return this._key;
                }
                set key(value) {
                    this._key = value;
                    this._key_invalid = false;
                    this._indexA = this._key & 0xff;
                    this._indexB = (this._key >> 8) & 0xff;
                    this._typeA = (this._key >> 16) & 0xff;
                    this._typeB = (this._key >> 24) & 0xff;
                }
                get indexA() {
                    return this._indexA;
                }
                set indexA(value) {
                    this._indexA = value;
                    this._key_invalid = true;
                }
                get indexB() {
                    return this._indexB;
                }
                set indexB(value) {
                    this._indexB = value;
                    this._key_invalid = true;
                }
                get typeA() {
                    return this._typeA;
                }
                set typeA(value) {
                    this._typeA = value;
                    this._key_invalid = true;
                }
                get typeB() {
                    return this._typeB;
                }
                set typeB(value) {
                    this._typeB = value;
                    this._key_invalid = true;
                }
            };
            exports_1("b2ContactFeature", b2ContactFeature);
            /// Contact ids to facilitate warm starting.
            b2ContactID = class b2ContactID {
                constructor() {
                    this.cf = new b2ContactFeature();
                }
                Copy(o) {
                    this.key = o.key;
                    return this;
                }
                Clone() {
                    return new b2ContactID().Copy(this);
                }
                get key() {
                    return this.cf.key;
                }
                set key(value) {
                    this.cf.key = value;
                }
            };
            exports_1("b2ContactID", b2ContactID);
            /// A manifold point is a contact point belonging to a contact
            /// manifold. It holds details related to the geometry and dynamics
            /// of the contact points.
            /// The local point usage depends on the manifold type:
            /// -e_circles: the local center of circleB
            /// -e_faceA: the local center of cirlceB or the clip point of polygonB
            /// -e_faceB: the clip point of polygonA
            /// This structure is stored across time steps, so we keep it small.
            /// Note: the impulses are used for internal caching and may not
            /// provide reliable contact forces, especially for high speed collisions.
            b2ManifoldPoint = class b2ManifoldPoint {
                constructor() {
                    this.localPoint = new b2Math_1.b2Vec2(); ///< usage depends on manifold type
                    this.normalImpulse = 0; ///< the non-penetration impulse
                    this.tangentImpulse = 0; ///< the friction impulse
                    this.id = new b2ContactID(); ///< uniquely identifies a contact point between two shapes
                }
                static MakeArray(length) {
                    return b2Settings_1.b2MakeArray(length, (i) => new b2ManifoldPoint());
                }
                Reset() {
                    this.localPoint.SetZero();
                    this.normalImpulse = 0;
                    this.tangentImpulse = 0;
                    this.id.key = 0;
                }
                Copy(o) {
                    this.localPoint.Copy(o.localPoint);
                    this.normalImpulse = o.normalImpulse;
                    this.tangentImpulse = o.tangentImpulse;
                    this.id.Copy(o.id);
                    return this;
                }
            };
            exports_1("b2ManifoldPoint", b2ManifoldPoint);
            (function (b2ManifoldType) {
                b2ManifoldType[b2ManifoldType["e_unknown"] = -1] = "e_unknown";
                b2ManifoldType[b2ManifoldType["e_circles"] = 0] = "e_circles";
                b2ManifoldType[b2ManifoldType["e_faceA"] = 1] = "e_faceA";
                b2ManifoldType[b2ManifoldType["e_faceB"] = 2] = "e_faceB";
            })(b2ManifoldType || (b2ManifoldType = {}));
            exports_1("b2ManifoldType", b2ManifoldType);
            /// A manifold for two touching convex shapes.
            /// Box2D supports multiple types of contact:
            /// - clip point versus plane with radius
            /// - point versus point with radius (circles)
            /// The local point usage depends on the manifold type:
            /// -e_circles: the local center of circleA
            /// -e_faceA: the center of faceA
            /// -e_faceB: the center of faceB
            /// Similarly the local normal usage:
            /// -e_circles: not used
            /// -e_faceA: the normal on polygonA
            /// -e_faceB: the normal on polygonB
            /// We store contacts in this way so that position correction can
            /// account for movement, which is critical for continuous physics.
            /// All contact scenarios must be expressed in one of these types.
            /// This structure is stored across time steps, so we keep it small.
            b2Manifold = class b2Manifold {
                constructor() {
                    this.points = b2ManifoldPoint.MakeArray(b2Settings_1.b2_maxManifoldPoints);
                    this.localNormal = new b2Math_1.b2Vec2();
                    this.localPoint = new b2Math_1.b2Vec2();
                    this.type = b2ManifoldType.e_unknown;
                    this.pointCount = 0;
                }
                Reset() {
                    for (let i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
                        ///b2Assert(this.points[i] instanceof b2ManifoldPoint);
                        this.points[i].Reset();
                    }
                    this.localNormal.SetZero();
                    this.localPoint.SetZero();
                    this.type = b2ManifoldType.e_unknown;
                    this.pointCount = 0;
                }
                Copy(o) {
                    this.pointCount = o.pointCount;
                    for (let i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
                        ///b2Assert(this.points[i] instanceof b2ManifoldPoint);
                        this.points[i].Copy(o.points[i]);
                    }
                    this.localNormal.Copy(o.localNormal);
                    this.localPoint.Copy(o.localPoint);
                    this.type = o.type;
                    return this;
                }
                Clone() {
                    return new b2Manifold().Copy(this);
                }
            };
            exports_1("b2Manifold", b2Manifold);
            b2WorldManifold = class b2WorldManifold {
                constructor() {
                    this.normal = new b2Math_1.b2Vec2();
                    this.points = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxManifoldPoints);
                    this.separations = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxManifoldPoints);
                }
                Initialize(manifold, xfA, radiusA, xfB, radiusB) {
                    if (manifold.pointCount === 0) {
                        return;
                    }
                    switch (manifold.type) {
                        case b2ManifoldType.e_circles:
                            {
                                this.normal.Set(1, 0);
                                const pointA = b2Math_1.b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_pointA);
                                const pointB = b2Math_1.b2Transform.MulXV(xfB, manifold.points[0].localPoint, b2WorldManifold.Initialize_s_pointB);
                                if (b2Math_1.b2Vec2.DistanceSquaredVV(pointA, pointB) > b2Settings_1.b2_epsilon_sq) {
                                    b2Math_1.b2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
                                }
                                const cA = b2Math_1.b2Vec2.AddVMulSV(pointA, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                                const cB = b2Math_1.b2Vec2.SubVMulSV(pointB, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                                b2Math_1.b2Vec2.MidVV(cA, cB, this.points[0]);
                                this.separations[0] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
                            }
                            break;
                        case b2ManifoldType.e_faceA:
                            {
                                b2Math_1.b2Rot.MulRV(xfA.q, manifold.localNormal, this.normal);
                                const planePoint = b2Math_1.b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
                                for (let i = 0; i < manifold.pointCount; ++i) {
                                    const clipPoint = b2Math_1.b2Transform.MulXV(xfB, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                                    const s = radiusA - b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal);
                                    const cA = b2Math_1.b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cA);
                                    const cB = b2Math_1.b2Vec2.SubVMulSV(clipPoint, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                                    b2Math_1.b2Vec2.MidVV(cA, cB, this.points[i]);
                                    this.separations[i] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
                                }
                            }
                            break;
                        case b2ManifoldType.e_faceB:
                            {
                                b2Math_1.b2Rot.MulRV(xfB.q, manifold.localNormal, this.normal);
                                const planePoint = b2Math_1.b2Transform.MulXV(xfB, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
                                for (let i = 0; i < manifold.pointCount; ++i) {
                                    const clipPoint = b2Math_1.b2Transform.MulXV(xfA, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                                    const s = radiusB - b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal);
                                    const cB = b2Math_1.b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cB);
                                    const cA = b2Math_1.b2Vec2.SubVMulSV(clipPoint, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                                    b2Math_1.b2Vec2.MidVV(cA, cB, this.points[i]);
                                    this.separations[i] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cA, cB, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cA - cB, normal);
                                }
                                // Ensure normal points from A to B.
                                this.normal.SelfNeg();
                            }
                            break;
                    }
                }
            };
            b2WorldManifold.Initialize_s_pointA = new b2Math_1.b2Vec2();
            b2WorldManifold.Initialize_s_pointB = new b2Math_1.b2Vec2();
            b2WorldManifold.Initialize_s_cA = new b2Math_1.b2Vec2();
            b2WorldManifold.Initialize_s_cB = new b2Math_1.b2Vec2();
            b2WorldManifold.Initialize_s_planePoint = new b2Math_1.b2Vec2();
            b2WorldManifold.Initialize_s_clipPoint = new b2Math_1.b2Vec2();
            exports_1("b2WorldManifold", b2WorldManifold);
            /// This is used for determining the state of contact points.
            (function (b2PointState) {
                b2PointState[b2PointState["b2_nullState"] = 0] = "b2_nullState";
                b2PointState[b2PointState["b2_addState"] = 1] = "b2_addState";
                b2PointState[b2PointState["b2_persistState"] = 2] = "b2_persistState";
                b2PointState[b2PointState["b2_removeState"] = 3] = "b2_removeState"; ///< point was removed in the update
            })(b2PointState || (b2PointState = {}));
            exports_1("b2PointState", b2PointState);
            /// Used for computing contact manifolds.
            b2ClipVertex = class b2ClipVertex {
                constructor() {
                    this.v = new b2Math_1.b2Vec2();
                    this.id = new b2ContactID();
                }
                static MakeArray(length) {
                    return b2Settings_1.b2MakeArray(length, (i) => new b2ClipVertex());
                }
                Copy(other) {
                    this.v.Copy(other.v);
                    this.id.Copy(other.id);
                    return this;
                }
            };
            exports_1("b2ClipVertex", b2ClipVertex);
            /// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
            b2RayCastInput = class b2RayCastInput {
                constructor() {
                    this.p1 = new b2Math_1.b2Vec2();
                    this.p2 = new b2Math_1.b2Vec2();
                    this.maxFraction = 1;
                }
                Copy(o) {
                    this.p1.Copy(o.p1);
                    this.p2.Copy(o.p2);
                    this.maxFraction = o.maxFraction;
                    return this;
                }
            };
            exports_1("b2RayCastInput", b2RayCastInput);
            /// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
            /// come from b2RayCastInput.
            b2RayCastOutput = class b2RayCastOutput {
                constructor() {
                    this.normal = new b2Math_1.b2Vec2();
                    this.fraction = 0;
                }
                Copy(o) {
                    this.normal.Copy(o.normal);
                    this.fraction = o.fraction;
                    return this;
                }
            };
            exports_1("b2RayCastOutput", b2RayCastOutput);
            /// An axis aligned bounding box.
            b2AABB = class b2AABB {
                constructor() {
                    this.lowerBound = new b2Math_1.b2Vec2(); ///< the lower vertex
                    this.upperBound = new b2Math_1.b2Vec2(); ///< the upper vertex
                    this.m_cache_center = new b2Math_1.b2Vec2(); // access using GetCenter()
                    this.m_cache_extent = new b2Math_1.b2Vec2(); // access using GetExtents()
                }
                Copy(o) {
                    this.lowerBound.Copy(o.lowerBound);
                    this.upperBound.Copy(o.upperBound);
                    return this;
                }
                /// Verify that the bounds are sorted.
                IsValid() {
                    const d_x = this.upperBound.x - this.lowerBound.x;
                    const d_y = this.upperBound.y - this.lowerBound.y;
                    let valid = d_x >= 0 && d_y >= 0;
                    valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
                    return valid;
                }
                /// Get the center of the AABB.
                GetCenter() {
                    return b2Math_1.b2Vec2.MidVV(this.lowerBound, this.upperBound, this.m_cache_center);
                }
                /// Get the extents of the AABB (half-widths).
                GetExtents() {
                    return b2Math_1.b2Vec2.ExtVV(this.lowerBound, this.upperBound, this.m_cache_extent);
                }
                /// Get the perimeter length
                GetPerimeter() {
                    const wx = this.upperBound.x - this.lowerBound.x;
                    const wy = this.upperBound.y - this.lowerBound.y;
                    return 2 * (wx + wy);
                }
                /// Combine an AABB into this one.
                Combine1(aabb) {
                    this.lowerBound.x = b2Math_1.b2Min(this.lowerBound.x, aabb.lowerBound.x);
                    this.lowerBound.y = b2Math_1.b2Min(this.lowerBound.y, aabb.lowerBound.y);
                    this.upperBound.x = b2Math_1.b2Max(this.upperBound.x, aabb.upperBound.x);
                    this.upperBound.y = b2Math_1.b2Max(this.upperBound.y, aabb.upperBound.y);
                    return this;
                }
                /// Combine two AABBs into this one.
                Combine2(aabb1, aabb2) {
                    this.lowerBound.x = b2Math_1.b2Min(aabb1.lowerBound.x, aabb2.lowerBound.x);
                    this.lowerBound.y = b2Math_1.b2Min(aabb1.lowerBound.y, aabb2.lowerBound.y);
                    this.upperBound.x = b2Math_1.b2Max(aabb1.upperBound.x, aabb2.upperBound.x);
                    this.upperBound.y = b2Math_1.b2Max(aabb1.upperBound.y, aabb2.upperBound.y);
                    return this;
                }
                static Combine(aabb1, aabb2, out) {
                    out.Combine2(aabb1, aabb2);
                    return out;
                }
                /// Does this aabb contain the provided AABB.
                Contains(aabb) {
                    let result = true;
                    result = result && this.lowerBound.x <= aabb.lowerBound.x;
                    result = result && this.lowerBound.y <= aabb.lowerBound.y;
                    result = result && aabb.upperBound.x <= this.upperBound.x;
                    result = result && aabb.upperBound.y <= this.upperBound.y;
                    return result;
                }
                // From Real-time Collision Detection, p179.
                RayCast(output, input) {
                    let tmin = (-b2Settings_1.b2_maxFloat);
                    let tmax = b2Settings_1.b2_maxFloat;
                    const p_x = input.p1.x;
                    const p_y = input.p1.y;
                    const d_x = input.p2.x - input.p1.x;
                    const d_y = input.p2.y - input.p1.y;
                    const absD_x = b2Math_1.b2Abs(d_x);
                    const absD_y = b2Math_1.b2Abs(d_y);
                    const normal = output.normal;
                    if (absD_x < b2Settings_1.b2_epsilon) {
                        // Parallel.
                        if (p_x < this.lowerBound.x || this.upperBound.x < p_x) {
                            return false;
                        }
                    }
                    else {
                        const inv_d = 1 / d_x;
                        let t1 = (this.lowerBound.x - p_x) * inv_d;
                        let t2 = (this.upperBound.x - p_x) * inv_d;
                        // Sign of the normal vector.
                        let s = (-1);
                        if (t1 > t2) {
                            const t3 = t1;
                            t1 = t2;
                            t2 = t3;
                            s = 1;
                        }
                        // Push the min up
                        if (t1 > tmin) {
                            normal.x = s;
                            normal.y = 0;
                            tmin = t1;
                        }
                        // Pull the max down
                        tmax = b2Math_1.b2Min(tmax, t2);
                        if (tmin > tmax) {
                            return false;
                        }
                    }
                    if (absD_y < b2Settings_1.b2_epsilon) {
                        // Parallel.
                        if (p_y < this.lowerBound.y || this.upperBound.y < p_y) {
                            return false;
                        }
                    }
                    else {
                        const inv_d = 1 / d_y;
                        let t1 = (this.lowerBound.y - p_y) * inv_d;
                        let t2 = (this.upperBound.y - p_y) * inv_d;
                        // Sign of the normal vector.
                        let s = (-1);
                        if (t1 > t2) {
                            const t3 = t1;
                            t1 = t2;
                            t2 = t3;
                            s = 1;
                        }
                        // Push the min up
                        if (t1 > tmin) {
                            normal.x = 0;
                            normal.y = s;
                            tmin = t1;
                        }
                        // Pull the max down
                        tmax = b2Math_1.b2Min(tmax, t2);
                        if (tmin > tmax) {
                            return false;
                        }
                    }
                    // Does the ray start inside the box?
                    // Does the ray intersect beyond the max fraction?
                    if (tmin < 0 || input.maxFraction < tmin) {
                        return false;
                    }
                    // Intersection.
                    output.fraction = tmin;
                    return true;
                }
                TestOverlap(other) {
                    const d1_x = other.lowerBound.x - this.upperBound.x;
                    const d1_y = other.lowerBound.y - this.upperBound.y;
                    const d2_x = this.lowerBound.x - other.upperBound.x;
                    const d2_y = this.lowerBound.y - other.upperBound.y;
                    if (d1_x > 0 || d1_y > 0)
                        return false;
                    if (d2_x > 0 || d2_y > 0)
                        return false;
                    return true;
                }
            };
            exports_1("b2AABB", b2AABB);
            /// Determine if two generic shapes overlap.
            b2TestOverlapShape_s_input = new b2Distance_1.b2DistanceInput();
            b2TestOverlapShape_s_simplexCache = new b2Distance_1.b2SimplexCache();
            b2TestOverlapShape_s_output = new b2Distance_1.b2DistanceOutput();
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaXNpb24uanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyJiMkNvbGxpc2lvbi50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7SUFtUkYscUdBQXFHO0lBQ3JHLDhGQUE4RjtJQUM5RiwwQkFBaUMsTUFBc0IsRUFBRSxNQUFzQixFQUFFLFNBQXFCLEVBQUUsU0FBcUI7UUFDM0gsK0JBQStCO1FBQy9CLElBQUksQ0FBUyxDQUFDO1FBQ2QsS0FBSyxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxTQUFTLENBQUMsVUFBVSxFQUFFLEVBQUUsQ0FBQyxFQUFFO1lBQ3pDLE1BQU0sRUFBRSxHQUFnQixTQUFTLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQztZQUMvQyxNQUFNLEdBQUcsR0FBVyxFQUFFLENBQUMsR0FBRyxDQUFDO1lBRTNCLE1BQU0sQ0FBQyxDQUFDLENBQUMsR0FBRyxZQUFZLENBQUMsY0FBYyxDQUFDO1lBRXhDLEtBQUssSUFBSSxDQUFDLEdBQVcsQ0FBQyxFQUFFLEdBQUcsR0FBRyxTQUFTLENBQUMsVUFBVSxFQUFFLENBQUMsR0FBRyxHQUFHLEVBQUUsRUFBRSxDQUFDLEVBQUU7Z0JBQ2hFLElBQUksU0FBUyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsR0FBRyxLQUFLLEdBQUcsRUFBRTtvQkFDdEMsTUFBTSxDQUFDLENBQUMsQ0FBQyxHQUFHLFlBQVksQ0FBQyxlQUFlLENBQUM7b0JBQ3pDLE1BQU07aUJBQ1A7YUFDRjtTQUNGO1FBQ0QsT0FBTyxDQUFDLEdBQUcsaUNBQW9CLEVBQUUsRUFBRSxDQUFDLEVBQUU7WUFDcEMsTUFBTSxDQUFDLENBQUMsQ0FBQyxHQUFHLFlBQVksQ0FBQyxZQUFZLENBQUM7U0FDdkM7UUFFRCw0QkFBNEI7UUFDNUIsS0FBSyxDQUFDLEdBQUcsQ0FBQyxFQUFFLENBQUMsR0FBRyxTQUFT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