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
var b2Distance_1 = require("./b2Distance");
/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.
var b2ContactFeatureType;
(function (b2ContactFeatureType) {
    b2ContactFeatureType[b2ContactFeatureType["e_vertex"] = 0] = "e_vertex";
    b2ContactFeatureType[b2ContactFeatureType["e_face"] = 1] = "e_face";
})(b2ContactFeatureType = exports.b2ContactFeatureType || (exports.b2ContactFeatureType = {}));
/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
var b2ContactFeature = /** @class */ (function () {
    function b2ContactFeature() {
        this._key = 0;
        this._key_invalid = false;
        this._indexA = 0;
        this._indexB = 0;
        this._typeA = 0;
        this._typeB = 0;
    }
    Object.defineProperty(b2ContactFeature.prototype, "key", {
        get: function () {
            if (this._key_invalid) {
                this._key_invalid = false;
                this._key = this._indexA | (this._indexB << 8) | (this._typeA << 16) | (this._typeB << 24);
            }
            return this._key;
        },
        set: function (value) {
            this._key = value;
            this._key_invalid = false;
            this._indexA = this._key & 0xff;
            this._indexB = (this._key >> 8) & 0xff;
            this._typeA = (this._key >> 16) & 0xff;
            this._typeB = (this._key >> 24) & 0xff;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(b2ContactFeature.prototype, "indexA", {
        get: function () {
            return this._indexA;
        },
        set: function (value) {
            this._indexA = value;
            this._key_invalid = true;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(b2ContactFeature.prototype, "indexB", {
        get: function () {
            return this._indexB;
        },
        set: function (value) {
            this._indexB = value;
            this._key_invalid = true;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(b2ContactFeature.prototype, "typeA", {
        get: function () {
            return this._typeA;
        },
        set: function (value) {
            this._typeA = value;
            this._key_invalid = true;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(b2ContactFeature.prototype, "typeB", {
        get: function () {
            return this._typeB;
        },
        set: function (value) {
            this._typeB = value;
            this._key_invalid = true;
        },
        enumerable: true,
        configurable: true
    });
    return b2ContactFeature;
}());
exports.b2ContactFeature = b2ContactFeature;
/// Contact ids to facilitate warm starting.
var b2ContactID = /** @class */ (function () {
    function b2ContactID() {
        this.cf = new b2ContactFeature();
    }
    b2ContactID.prototype.Copy = function (o) {
        this.key = o.key;
        return this;
    };
    b2ContactID.prototype.Clone = function () {
        return new b2ContactID().Copy(this);
    };
    Object.defineProperty(b2ContactID.prototype, "key", {
        get: function () {
            return this.cf.key;
        },
        set: function (value) {
            this.cf.key = value;
        },
        enumerable: true,
        configurable: true
    });
    return b2ContactID;
}());
exports.b2ContactID = b2ContactID;
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
var b2ManifoldPoint = /** @class */ (function () {
    function b2ManifoldPoint() {
        this.localPoint = new b2Math_1.b2Vec2(); ///< usage depends on manifold type
        this.normalImpulse = 0; ///< the non-penetration impulse
        this.tangentImpulse = 0; ///< the friction impulse
        this.id = new b2ContactID(); // TODO: readonly  ///< uniquely identifies a contact point between two shapes
    }
    b2ManifoldPoint.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2ManifoldPoint(); });
    };
    b2ManifoldPoint.prototype.Reset = function () {
        this.localPoint.SetZero();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.id.key = 0;
    };
    b2ManifoldPoint.prototype.Copy = function (o) {
        this.localPoint.Copy(o.localPoint);
        this.normalImpulse = o.normalImpulse;
        this.tangentImpulse = o.tangentImpulse;
        this.id.Copy(o.id);
        return this;
    };
    return b2ManifoldPoint;
}());
exports.b2ManifoldPoint = b2ManifoldPoint;
var b2ManifoldType;
(function (b2ManifoldType) {
    b2ManifoldType[b2ManifoldType["e_unknown"] = -1] = "e_unknown";
    b2ManifoldType[b2ManifoldType["e_circles"] = 0] = "e_circles";
    b2ManifoldType[b2ManifoldType["e_faceA"] = 1] = "e_faceA";
    b2ManifoldType[b2ManifoldType["e_faceB"] = 2] = "e_faceB";
})(b2ManifoldType = exports.b2ManifoldType || (exports.b2ManifoldType = {}));
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
var b2Manifold = /** @class */ (function () {
    function b2Manifold() {
        this.points = b2ManifoldPoint.MakeArray(b2Settings_1.b2_maxManifoldPoints);
        this.localNormal = new b2Math_1.b2Vec2();
        this.localPoint = new b2Math_1.b2Vec2();
        this.type = b2ManifoldType.e_unknown;
        this.pointCount = 0;
    }
    b2Manifold.prototype.Reset = function () {
        for (var i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            // DEBUG: b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = b2ManifoldType.e_unknown;
        this.pointCount = 0;
    };
    b2Manifold.prototype.Copy = function (o) {
        this.pointCount = o.pointCount;
        for (var i = 0; i < b2Settings_1.b2_maxManifoldPoints; ++i) {
            // DEBUG: b2Assert(this.points[i] instanceof b2ManifoldPoint);
            this.points[i].Copy(o.points[i]);
        }
        this.localNormal.Copy(o.localNormal);
        this.localPoint.Copy(o.localPoint);
        this.type = o.type;
        return this;
    };
    b2Manifold.prototype.Clone = function () {
        return new b2Manifold().Copy(this);
    };
    return b2Manifold;
}());
exports.b2Manifold = b2Manifold;
var b2WorldManifold = /** @class */ (function () {
    function b2WorldManifold() {
        this.normal = new b2Math_1.b2Vec2();
        this.points = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxManifoldPoints);
        this.separations = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxManifoldPoints);
    }
    b2WorldManifold.prototype.Initialize = function (manifold, xfA, radiusA, xfB, radiusB) {
        if (manifold.pointCount === 0) {
            return;
        }
        switch (manifold.type) {
            case b2ManifoldType.e_circles:
                {
                    this.normal.Set(1, 0);
                    var pointA = b2Math_1.b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_pointA);
                    var pointB = b2Math_1.b2Transform.MulXV(xfB, manifold.points[0].localPoint, b2WorldManifold.Initialize_s_pointB);
                    if (b2Math_1.b2Vec2.DistanceSquaredVV(pointA, pointB) > b2Settings_1.b2_epsilon_sq) {
                        b2Math_1.b2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
                    }
                    var cA = b2Math_1.b2Vec2.AddVMulSV(pointA, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                    var cB = b2Math_1.b2Vec2.SubVMulSV(pointB, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                    b2Math_1.b2Vec2.MidVV(cA, cB, this.points[0]);
                    this.separations[0] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
                }
                break;
            case b2ManifoldType.e_faceA:
                {
                    b2Math_1.b2Rot.MulRV(xfA.q, manifold.localNormal, this.normal);
                    var planePoint = b2Math_1.b2Transform.MulXV(xfA, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
                    for (var i = 0; i < manifold.pointCount; ++i) {
                        var clipPoint = b2Math_1.b2Transform.MulXV(xfB, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                        var s = radiusA - b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal);
                        var cA = b2Math_1.b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cA);
                        var cB = b2Math_1.b2Vec2.SubVMulSV(clipPoint, radiusB, this.normal, b2WorldManifold.Initialize_s_cB);
                        b2Math_1.b2Vec2.MidVV(cA, cB, this.points[i]);
                        this.separations[i] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cB - cA, normal);
                    }
                }
                break;
            case b2ManifoldType.e_faceB:
                {
                    b2Math_1.b2Rot.MulRV(xfB.q, manifold.localNormal, this.normal);
                    var planePoint = b2Math_1.b2Transform.MulXV(xfB, manifold.localPoint, b2WorldManifold.Initialize_s_planePoint);
                    for (var i = 0; i < manifold.pointCount; ++i) {
                        var clipPoint = b2Math_1.b2Transform.MulXV(xfA, manifold.points[i].localPoint, b2WorldManifold.Initialize_s_clipPoint);
                        var s = radiusB - b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal);
                        var cB = b2Math_1.b2Vec2.AddVMulSV(clipPoint, s, this.normal, b2WorldManifold.Initialize_s_cB);
                        var cA = b2Math_1.b2Vec2.SubVMulSV(clipPoint, radiusA, this.normal, b2WorldManifold.Initialize_s_cA);
                        b2Math_1.b2Vec2.MidVV(cA, cB, this.points[i]);
                        this.separations[i] = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(cA, cB, b2Math_1.b2Vec2.s_t0), this.normal); // b2Dot(cA - cB, normal);
                    }
                    // Ensure normal points from A to B.
                    this.normal.SelfNeg();
                }
                break;
        }
    };
    b2WorldManifold.Initialize_s_pointA = new b2Math_1.b2Vec2();
    b2WorldManifold.Initialize_s_pointB = new b2Math_1.b2Vec2();
    b2WorldManifold.Initialize_s_cA = new b2Math_1.b2Vec2();
    b2WorldManifold.Initialize_s_cB = new b2Math_1.b2Vec2();
    b2WorldManifold.Initialize_s_planePoint = new b2Math_1.b2Vec2();
    b2WorldManifold.Initialize_s_clipPoint = new b2Math_1.b2Vec2();
    return b2WorldManifold;
}());
exports.b2WorldManifold = b2WorldManifold;
/// This is used for determining the state of contact points.
var b2PointState;
(function (b2PointState) {
    b2PointState[b2PointState["b2_nullState"] = 0] = "b2_nullState";
    b2PointState[b2PointState["b2_addState"] = 1] = "b2_addState";
    b2PointState[b2PointState["b2_persistState"] = 2] = "b2_persistState";
    b2PointState[b2PointState["b2_removeState"] = 3] = "b2_removeState";
})(b2PointState = exports.b2PointState || (exports.b2PointState = {}));
/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
function b2GetPointStates(state1, state2, manifold1, manifold2) {
    // Detect persists and removes.
    var i;
    for (i = 0; i < manifold1.pointCount; ++i) {
        var id = manifold1.points[i].id;
        var key = id.key;
        state1[i] = b2PointState.b2_removeState;
        for (var j = 0, jct = manifold2.pointCount; j < jct; ++j) {
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
        var id = manifold2.points[i].id;
        var key = id.key;
        state2[i] = b2PointState.b2_addState;
        for (var j = 0, jct = manifold1.pointCount; j < jct; ++j) {
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
exports.b2GetPointStates = b2GetPointStates;
/// Used for computing contact manifolds.
var b2ClipVertex = /** @class */ (function () {
    function b2ClipVertex() {
        this.v = new b2Math_1.b2Vec2();
        this.id = new b2ContactID();
    }
    b2ClipVertex.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2ClipVertex(); });
    };
    b2ClipVertex.prototype.Copy = function (other) {
        this.v.Copy(other.v);
        this.id.Copy(other.id);
        return this;
    };
    return b2ClipVertex;
}());
exports.b2ClipVertex = b2ClipVertex;
/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
var b2RayCastInput = /** @class */ (function () {
    function b2RayCastInput() {
        this.p1 = new b2Math_1.b2Vec2();
        this.p2 = new b2Math_1.b2Vec2();
        this.maxFraction = 1;
    }
    b2RayCastInput.prototype.Copy = function (o) {
        this.p1.Copy(o.p1);
        this.p2.Copy(o.p2);
        this.maxFraction = o.maxFraction;
        return this;
    };
    return b2RayCastInput;
}());
exports.b2RayCastInput = b2RayCastInput;
/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
var b2RayCastOutput = /** @class */ (function () {
    function b2RayCastOutput() {
        this.normal = new b2Math_1.b2Vec2();
        this.fraction = 0;
    }
    b2RayCastOutput.prototype.Copy = function (o) {
        this.normal.Copy(o.normal);
        this.fraction = o.fraction;
        return this;
    };
    return b2RayCastOutput;
}());
exports.b2RayCastOutput = b2RayCastOutput;
/// An axis aligned bounding box.
var b2AABB = /** @class */ (function () {
    function b2AABB() {
        this.lowerBound = new b2Math_1.b2Vec2(); ///< the lower vertex
        this.upperBound = new b2Math_1.b2Vec2(); ///< the upper vertex
        this.m_cache_center = new b2Math_1.b2Vec2(); // access using GetCenter()
        this.m_cache_extent = new b2Math_1.b2Vec2(); // access using GetExtents()
    }
    b2AABB.prototype.Copy = function (o) {
        this.lowerBound.Copy(o.lowerBound);
        this.upperBound.Copy(o.upperBound);
        return this;
    };
    /// Verify that the bounds are sorted.
    b2AABB.prototype.IsValid = function () {
        var d_x = this.upperBound.x - this.lowerBound.x;
        var d_y = this.upperBound.y - this.lowerBound.y;
        var valid = d_x >= 0 && d_y >= 0;
        valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
        return valid;
    };
    /// Get the center of the AABB.
    b2AABB.prototype.GetCenter = function () {
        return b2Math_1.b2Vec2.MidVV(this.lowerBound, this.upperBound, this.m_cache_center);
    };
    /// Get the extents of the AABB (half-widths).
    b2AABB.prototype.GetExtents = function () {
        return b2Math_1.b2Vec2.ExtVV(this.lowerBound, this.upperBound, this.m_cache_extent);
    };
    /// Get the perimeter length
    b2AABB.prototype.GetPerimeter = function () {
        var wx = this.upperBound.x - this.lowerBound.x;
        var wy = this.upperBound.y - this.lowerBound.y;
        return 2 * (wx + wy);
    };
    /// Combine an AABB into this one.
    b2AABB.prototype.Combine1 = function (aabb) {
        this.lowerBound.x = b2Math_1.b2Min(this.lowerBound.x, aabb.lowerBound.x);
        this.lowerBound.y = b2Math_1.b2Min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.x = b2Math_1.b2Max(this.upperBound.x, aabb.upperBound.x);
        this.upperBound.y = b2Math_1.b2Max(this.upperBound.y, aabb.upperBound.y);
        return this;
    };
    /// Combine two AABBs into this one.
    b2AABB.prototype.Combine2 = function (aabb1, aabb2) {
        this.lowerBound.x = b2Math_1.b2Min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = b2Math_1.b2Min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = b2Math_1.b2Max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = b2Math_1.b2Max(aabb1.upperBound.y, aabb2.upperBound.y);
        return this;
    };
    b2AABB.Combine = function (aabb1, aabb2, out) {
        out.Combine2(aabb1, aabb2);
        return out;
    };
    /// Does this aabb contain the provided AABB.
    b2AABB.prototype.Contains = function (aabb) {
        var result = true;
        result = result && this.lowerBound.x <= aabb.lowerBound.x;
        result = result && this.lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= this.upperBound.x;
        result = result && aabb.upperBound.y <= this.upperBound.y;
        return result;
    };
    // From Real-time Collision Detection, p179.
    b2AABB.prototype.RayCast = function (output, input) {
        var tmin = (-b2Settings_1.b2_maxFloat);
        var tmax = b2Settings_1.b2_maxFloat;
        var p_x = input.p1.x;
        var p_y = input.p1.y;
        var d_x = input.p2.x - input.p1.x;
        var d_y = input.p2.y - input.p1.y;
        var absD_x = b2Math_1.b2Abs(d_x);
        var absD_y = b2Math_1.b2Abs(d_y);
        var normal = output.normal;
        if (absD_x < b2Settings_1.b2_epsilon) {
            // Parallel.
            if (p_x < this.lowerBound.x || this.upperBound.x < p_x) {
                return false;
            }
        }
        else {
            var inv_d = 1 / d_x;
            var t1 = (this.lowerBound.x - p_x) * inv_d;
            var t2 = (this.upperBound.x - p_x) * inv_d;
            // Sign of the normal vector.
            var s = (-1);
            if (t1 > t2) {
                var t3 = t1;
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
            var inv_d = 1 / d_y;
            var t1 = (this.lowerBound.y - p_y) * inv_d;
            var t2 = (this.upperBound.y - p_y) * inv_d;
            // Sign of the normal vector.
            var s = (-1);
            if (t1 > t2) {
                var t3 = t1;
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
    };
    b2AABB.prototype.TestContain = function (point) {
        if (point.x < this.lowerBound.x || this.upperBound.x < point.x) {
            return false;
        }
        if (point.y < this.lowerBound.y || this.upperBound.y < point.y) {
            return false;
        }
        return true;
    };
    b2AABB.prototype.TestOverlap = function (other) {
        var d1_x = other.lowerBound.x - this.upperBound.x;
        var d1_y = other.lowerBound.y - this.upperBound.y;
        var d2_x = this.lowerBound.x - other.upperBound.x;
        var d2_y = this.lowerBound.y - other.upperBound.y;
        if (d1_x > 0 || d1_y > 0) {
            return false;
        }
        if (d2_x > 0 || d2_y > 0) {
            return false;
        }
        return true;
    };
    return b2AABB;
}());
exports.b2AABB = b2AABB;
function b2TestOverlapAABB(a, b) {
    var d1_x = b.lowerBound.x - a.upperBound.x;
    var d1_y = b.lowerBound.y - a.upperBound.y;
    var d2_x = a.lowerBound.x - b.upperBound.x;
    var d2_y = a.lowerBound.y - b.upperBound.y;
    if (d1_x > 0 || d1_y > 0) {
        return false;
    }
    if (d2_x > 0 || d2_y > 0) {
        return false;
    }
    return true;
}
exports.b2TestOverlapAABB = b2TestOverlapAABB;
/// Clipping for contact manifolds.
function b2ClipSegmentToLine(vOut, vIn, normal, offset, vertexIndexA) {
    // Start with no output points
    var numOut = 0;
    var vIn0 = vIn[0];
    var vIn1 = vIn[1];
    // Calculate the distance of end points to the line
    var distance0 = b2Math_1.b2Vec2.DotVV(normal, vIn0.v) - offset;
    var distance1 = b2Math_1.b2Vec2.DotVV(normal, vIn1.v) - offset;
    // If the points are behind the plane
    if (distance0 <= 0) {
        vOut[numOut++].Copy(vIn0);
    }
    if (distance1 <= 0) {
        vOut[numOut++].Copy(vIn1);
    }
    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0) {
        // Find intersection point of edge and plane
        var interp = distance0 / (distance0 - distance1);
        var v = vOut[numOut].v;
        v.x = vIn0.v.x + interp * (vIn1.v.x - vIn0.v.x);
        v.y = vIn0.v.y + interp * (vIn1.v.y - vIn0.v.y);
        // VertexA is hitting edgeB.
        var id = vOut[numOut].id;
        id.cf.indexA = vertexIndexA;
        id.cf.indexB = vIn0.id.cf.indexB;
        id.cf.typeA = b2ContactFeatureType.e_vertex;
        id.cf.typeB = b2ContactFeatureType.e_face;
        ++numOut;
    }
    return numOut;
}
exports.b2ClipSegmentToLine = b2ClipSegmentToLine;
/// Determine if two generic shapes overlap.
var b2TestOverlapShape_s_input = new b2Distance_1.b2DistanceInput();
var b2TestOverlapShape_s_simplexCache = new b2Distance_1.b2SimplexCache();
var b2TestOverlapShape_s_output = new b2Distance_1.b2DistanceOutput();
function b2TestOverlapShape(shapeA, indexA, shapeB, indexB, xfA, xfB) {
    var input = b2TestOverlapShape_s_input.Reset();
    input.proxyA.SetShape(shapeA, indexA);
    input.proxyB.SetShape(shapeB, indexB);
    input.transformA.Copy(xfA);
    input.transformB.Copy(xfB);
    input.useRadii = true;
    var simplexCache = b2TestOverlapShape_s_simplexCache.Reset();
    simplexCache.count = 0;
    var output = b2TestOverlapShape_s_output.Reset();
    b2Distance_1.b2Distance(output, simplexCache, input);
    return output.distance < 10 * b2Settings_1.b2_epsilon;
}
exports.b2TestOverlapShape = b2TestOverlapShape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb2xsaXNpb24uanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9Cb3gyRC9Cb3gyRC9Db2xsaXNpb24vYjJDb2xsaXNpb24udHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLDBEQUEwRDtBQUMxRCxtREFBb0k7QUFDcEksMkNBQW1GO0FBRW5GLDJDQUE2RjtBQUU3RixTQUFTO0FBQ1Qsd0VBQXdFO0FBQ3hFLDZCQUE2QjtBQUU3QixJQUFZLG9CQUdYO0FBSEQsV0FBWSxvQkFBb0I7SUFDOUIsdUVBQVksQ0FBQTtJQUNaLG1FQUFVLENBQUE7QUFDWixDQUFDLEVBSFcsb0JBQW9CLEdBQXBCLDRCQUFvQixLQUFwQiw0QkFBb0IsUUFHL0I7QUFFRCx5REFBeUQ7QUFDekQsaUNBQWlDO0FBQ2pDO0lBUUU7UUFQTyxTQUFJLEdBQVcsQ0FBQyxDQUFDO1FBQ2pCLGlCQUFZLEdBQUcsS0FBSyxDQUFDO1FBQ3JCLFlBQU8sR0FBVyxDQUFDLENBQUM7UUFDcEIsWUFBTyxHQUFXLENBQUMsQ0FBQztRQUNwQixXQUFNLEdBQVcsQ0FBQyxDQUFDO1FBQ25CLFdBQU0sR0FBVyxDQUFDLENBQUM7SUFHMUIsQ0FBQztJQUVELHNCQUFXLGlDQUFHO2FBQWQ7WUFDRSxJQUFJLElBQUksQ0FBQyxZQUFZLEVBQUU7Z0JBQ3JCLElBQUksQ0FBQyxZQUFZLEdBQUcsS0FBSyxDQUFDO2dCQUMxQixJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxPQUFPLEdBQUcsQ0FBQyxJQUFJLENBQUMsT0FBTyxJQUFJLENBQUMsQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLE1BQU0sSUFBSSxFQUFFLENBQUMsR0FBRyxDQUFDLElBQUksQ0FBQyxNQUFNLElBQUksRUFBRSxDQUFDLENBQUM7YUFDNUY7WUFDRCxPQUFPLElBQUksQ0FBQyxJQUFJLENBQUM7UUFDbkIsQ0FBQzthQUVELFVBQWUsS0FBYTtZQUMxQixJ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