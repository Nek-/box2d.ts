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
var b2Timer_1 = require("../Common/b2Timer");
var b2Distance_1 = require("./b2Distance");
exports.b2_toiTime = 0;
exports.b2_toiMaxTime = 0;
exports.b2_toiCalls = 0;
exports.b2_toiIters = 0;
exports.b2_toiMaxIters = 0;
exports.b2_toiRootIters = 0;
exports.b2_toiMaxRootIters = 0;
function b2_toi_reset() {
    exports.b2_toiTime = 0;
    exports.b2_toiMaxTime = 0;
    exports.b2_toiCalls = 0;
    exports.b2_toiIters = 0;
    exports.b2_toiMaxIters = 0;
    exports.b2_toiRootIters = 0;
    exports.b2_toiMaxRootIters = 0;
}
exports.b2_toi_reset = b2_toi_reset;
var b2TimeOfImpact_s_xfA = new b2Math_1.b2Transform();
var b2TimeOfImpact_s_xfB = new b2Math_1.b2Transform();
var b2TimeOfImpact_s_pointA = new b2Math_1.b2Vec2();
var b2TimeOfImpact_s_pointB = new b2Math_1.b2Vec2();
var b2TimeOfImpact_s_normal = new b2Math_1.b2Vec2();
var b2TimeOfImpact_s_axisA = new b2Math_1.b2Vec2();
var b2TimeOfImpact_s_axisB = new b2Math_1.b2Vec2();
/// Input parameters for b2TimeOfImpact
var b2TOIInput = /** @class */ (function () {
    function b2TOIInput() {
        this.proxyA = new b2Distance_1.b2DistanceProxy();
        this.proxyB = new b2Distance_1.b2DistanceProxy();
        this.sweepA = new b2Math_1.b2Sweep();
        this.sweepB = new b2Math_1.b2Sweep();
        this.tMax = 0; // defines sweep interval [0, tMax]
    }
    return b2TOIInput;
}());
exports.b2TOIInput = b2TOIInput;
var b2TOIOutputState;
(function (b2TOIOutputState) {
    b2TOIOutputState[b2TOIOutputState["e_unknown"] = 0] = "e_unknown";
    b2TOIOutputState[b2TOIOutputState["e_failed"] = 1] = "e_failed";
    b2TOIOutputState[b2TOIOutputState["e_overlapped"] = 2] = "e_overlapped";
    b2TOIOutputState[b2TOIOutputState["e_touching"] = 3] = "e_touching";
    b2TOIOutputState[b2TOIOutputState["e_separated"] = 4] = "e_separated";
})(b2TOIOutputState = exports.b2TOIOutputState || (exports.b2TOIOutputState = {}));
var b2TOIOutput = /** @class */ (function () {
    function b2TOIOutput() {
        this.state = b2TOIOutputState.e_unknown;
        this.t = 0;
    }
    return b2TOIOutput;
}());
exports.b2TOIOutput = b2TOIOutput;
var b2SeparationFunctionType;
(function (b2SeparationFunctionType) {
    b2SeparationFunctionType[b2SeparationFunctionType["e_unknown"] = -1] = "e_unknown";
    b2SeparationFunctionType[b2SeparationFunctionType["e_points"] = 0] = "e_points";
    b2SeparationFunctionType[b2SeparationFunctionType["e_faceA"] = 1] = "e_faceA";
    b2SeparationFunctionType[b2SeparationFunctionType["e_faceB"] = 2] = "e_faceB";
})(b2SeparationFunctionType = exports.b2SeparationFunctionType || (exports.b2SeparationFunctionType = {}));
var b2SeparationFunction = /** @class */ (function () {
    function b2SeparationFunction() {
        this.m_sweepA = new b2Math_1.b2Sweep();
        this.m_sweepB = new b2Math_1.b2Sweep();
        this.m_type = b2SeparationFunctionType.e_unknown;
        this.m_localPoint = new b2Math_1.b2Vec2();
        this.m_axis = new b2Math_1.b2Vec2();
    }
    b2SeparationFunction.prototype.Initialize = function (cache, proxyA, sweepA, proxyB, sweepB, t1) {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        var count = cache.count;
        // DEBUG: b2Assert(0 < count && count < 3);
        this.m_sweepA.Copy(sweepA);
        this.m_sweepB.Copy(sweepB);
        var xfA = b2TimeOfImpact_s_xfA;
        var xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t1);
        this.m_sweepB.GetTransform(xfB, t1);
        if (count === 1) {
            this.m_type = b2SeparationFunctionType.e_points;
            var localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            var localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            b2Math_1.b2Vec2.SubVV(pointB, pointA, this.m_axis);
            var s = this.m_axis.Normalize();
            // #if B2_ENABLE_PARTICLE
            this.m_localPoint.SetZero();
            // #endif
            return s;
        }
        else if (cache.indexA[0] === cache.indexA[1]) {
            // Two points on B and one on A.
            this.m_type = b2SeparationFunctionType.e_faceB;
            var localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            var localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
            b2Math_1.b2Vec2.CrossVOne(b2Math_1.b2Vec2.SubVV(localPointB2, localPointB1, b2Math_1.b2Vec2.s_t0), this.m_axis).SelfNormalize();
            var normal = b2Math_1.b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
            b2Math_1.b2Vec2.MidVV(localPointB1, localPointB2, this.m_localPoint);
            var pointB = b2Math_1.b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
            var localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
            var s = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointA, pointB, b2Math_1.b2Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
        else {
            // Two points on A and one or two points on B.
            this.m_type = b2SeparationFunctionType.e_faceA;
            var localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
            var localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
            b2Math_1.b2Vec2.CrossVOne(b2Math_1.b2Vec2.SubVV(localPointA2, localPointA1, b2Math_1.b2Vec2.s_t0), this.m_axis).SelfNormalize();
            var normal = b2Math_1.b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
            b2Math_1.b2Vec2.MidVV(localPointA1, localPointA2, this.m_localPoint);
            var pointA = b2Math_1.b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
            var localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
            var s = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
    };
    b2SeparationFunction.prototype.FindMinSeparation = function (indexA, indexB, t) {
        var xfA = b2TimeOfImpact_s_xfA;
        var xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
            case b2SeparationFunctionType.e_points: {
                var axisA = b2Math_1.b2Rot.MulTRV(xfA.q, this.m_axis, b2TimeOfImpact_s_axisA);
                var axisB = b2Math_1.b2Rot.MulTRV(xfB.q, b2Math_1.b2Vec2.NegV(this.m_axis, b2Math_1.b2Vec2.s_t0), b2TimeOfImpact_s_axisB);
                indexA[0] = this.m_proxyA.GetSupport(axisA);
                indexB[0] = this.m_proxyB.GetSupport(axisB);
                var localPointA = this.m_proxyA.GetVertex(indexA[0]);
                var localPointB = this.m_proxyB.GetVertex(indexB[0]);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), this.m_axis);
                return separation;
            }
            case b2SeparationFunctionType.e_faceA: {
                var normal = b2Math_1.b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
                var axisB = b2Math_1.b2Rot.MulTRV(xfB.q, b2Math_1.b2Vec2.NegV(normal, b2Math_1.b2Vec2.s_t0), b2TimeOfImpact_s_axisB);
                indexA[0] = -1;
                indexB[0] = this.m_proxyB.GetSupport(axisB);
                var localPointB = this.m_proxyB.GetVertex(indexB[0]);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), normal);
                return separation;
            }
            case b2SeparationFunctionType.e_faceB: {
                var normal = b2Math_1.b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
                var axisA = b2Math_1.b2Rot.MulTRV(xfA.q, b2Math_1.b2Vec2.NegV(normal, b2Math_1.b2Vec2.s_t0), b2TimeOfImpact_s_axisA);
                indexB[0] = -1;
                indexA[0] = this.m_proxyA.GetSupport(axisA);
                var localPointA = this.m_proxyA.GetVertex(indexA[0]);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointA, pointB, b2Math_1.b2Vec2.s_t0), normal);
                return separation;
            }
            default:
                // DEBUG: b2Assert(false);
                indexA[0] = -1;
                indexB[0] = -1;
                return 0;
        }
    };
    b2SeparationFunction.prototype.Evaluate = function (indexA, indexB, t) {
        var xfA = b2TimeOfImpact_s_xfA;
        var xfB = b2TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
            case b2SeparationFunctionType.e_points: {
                var localPointA = this.m_proxyA.GetVertex(indexA);
                var localPointB = this.m_proxyB.GetVertex(indexB);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), this.m_axis);
                return separation;
            }
            case b2SeparationFunctionType.e_faceA: {
                var normal = b2Math_1.b2Rot.MulRV(xfA.q, this.m_axis, b2TimeOfImpact_s_normal);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, this.m_localPoint, b2TimeOfImpact_s_pointA);
                var localPointB = this.m_proxyB.GetVertex(indexB);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, localPointB, b2TimeOfImpact_s_pointB);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), normal);
                return separation;
            }
            case b2SeparationFunctionType.e_faceB: {
                var normal = b2Math_1.b2Rot.MulRV(xfB.q, this.m_axis, b2TimeOfImpact_s_normal);
                var pointB = b2Math_1.b2Transform.MulXV(xfB, this.m_localPoint, b2TimeOfImpact_s_pointB);
                var localPointA = this.m_proxyA.GetVertex(indexA);
                var pointA = b2Math_1.b2Transform.MulXV(xfA, localPointA, b2TimeOfImpact_s_pointA);
                var separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointA, pointB, b2Math_1.b2Vec2.s_t0), normal);
                return separation;
            }
            default:
                // DEBUG: b2Assert(false);
                return 0;
        }
    };
    return b2SeparationFunction;
}());
exports.b2SeparationFunction = b2SeparationFunction;
var b2TimeOfImpact_s_timer = new b2Timer_1.b2Timer();
var b2TimeOfImpact_s_cache = new b2Distance_1.b2SimplexCache();
var b2TimeOfImpact_s_distanceInput = new b2Distance_1.b2DistanceInput();
var b2TimeOfImpact_s_distanceOutput = new b2Distance_1.b2DistanceOutput();
var b2TimeOfImpact_s_fcn = new b2SeparationFunction();
var b2TimeOfImpact_s_indexA = [0];
var b2TimeOfImpact_s_indexB = [0];
var b2TimeOfImpact_s_sweepA = new b2Math_1.b2Sweep();
var b2TimeOfImpact_s_sweepB = new b2Math_1.b2Sweep();
function b2TimeOfImpact(output, input) {
    var timer = b2TimeOfImpact_s_timer.Reset();
    ++exports.b2_toiCalls;
    output.state = b2TOIOutputState.e_unknown;
    output.t = input.tMax;
    var proxyA = input.proxyA;
    var proxyB = input.proxyB;
    var sweepA = b2TimeOfImpact_s_sweepA.Copy(input.sweepA);
    var sweepB = b2TimeOfImpact_s_sweepB.Copy(input.sweepB);
    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();
    var tMax = input.tMax;
    var totalRadius = proxyA.m_radius + proxyB.m_radius;
    var target = b2Math_1.b2Max(b2Settings_1.b2_linearSlop, totalRadius - 3 * b2Settings_1.b2_linearSlop);
    var tolerance = 0.25 * b2Settings_1.b2_linearSlop;
    // DEBUG: b2Assert(target > tolerance);
    var t1 = 0;
    var k_maxIterations = 20; // TODO_ERIN b2Settings
    var iter = 0;
    // Prepare input for distance query.
    var cache = b2TimeOfImpact_s_cache;
    cache.count = 0;
    var distanceInput = b2TimeOfImpact_s_distanceInput;
    distanceInput.proxyA = input.proxyA;
    distanceInput.proxyB = input.proxyB;
    distanceInput.useRadii = false;
    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
        var xfA = b2TimeOfImpact_s_xfA;
        var xfB = b2TimeOfImpact_s_xfB;
        sweepA.GetTransform(xfA, t1);
        sweepB.GetTransform(xfB, t1);
        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA.Copy(xfA);
        distanceInput.transformB.Copy(xfB);
        var distanceOutput = b2TimeOfImpact_s_distanceOutput;
        b2Distance_1.b2Distance(distanceOutput, cache, distanceInput);
        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0) {
            // Failure!
            output.state = b2TOIOutputState.e_overlapped;
            output.t = 0;
            break;
        }
        if (distanceOutput.distance < target + tolerance) {
            // Victory!
            output.state = b2TOIOutputState.e_touching;
            output.t = t1;
            break;
        }
        // Initialize the separating axis.
        var fcn = b2TimeOfImpact_s_fcn;
        fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);
        /*
        #if 0
            // Dump the curve seen by the root finder {
              const int32 N = 100;
              float32 dx = 1.0f / N;
              float32 xs[N+1];
              float32 fs[N+1];
        
              float32 x = 0.0f;
        
              for (int32 i = 0; i <= N; ++i) {
                sweepA.GetTransform(&xfA, x);
                sweepB.GetTransform(&xfB, x);
                float32 f = fcn.Evaluate(xfA, xfB) - target;
        
                printf("%g %g\n", x, f);
        
                xs[i] = x;
                fs[i] = f;
        
                x += dx;
              }
            }
        #endif
        */
        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        var done = false;
        var t2 = tMax;
        var pushBackIter = 0;
        for (;;) {
            // Find the deepest point at t2. Store the witness point indices.
            var indexA = b2TimeOfImpact_s_indexA;
            var indexB = b2TimeOfImpact_s_indexB;
            var s2 = fcn.FindMinSeparation(indexA, indexB, t2);
            // Is the final configuration separated?
            if (s2 > (target + tolerance)) {
                // Victory!
                output.state = b2TOIOutputState.e_separated;
                output.t = tMax;
                done = true;
                break;
            }
            // Has the separation reached tolerance?
            if (s2 > (target - tolerance)) {
                // Advance the sweeps
                t1 = t2;
                break;
            }
            // Compute the initial separation of the witness points.
            var s1 = fcn.Evaluate(indexA[0], indexB[0], t1);
            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < (target - tolerance)) {
                output.state = b2TOIOutputState.e_failed;
                output.t = t1;
                done = true;
                break;
            }
            // Check for touching
            if (s1 <= (target + tolerance)) {
                // Victory! t1 should hold the TOI (could be 0.0).
                output.state = b2TOIOutputState.e_touching;
                output.t = t1;
                done = true;
                break;
            }
            // Compute 1D root of: f(x) - target = 0
            var rootIterCount = 0;
            var a1 = t1;
            var a2 = t2;
            for (;;) {
                // Use a mix of the secant rule and bisection.
                var t = 0;
                if (rootIterCount & 1) {
                    // Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                }
                else {
                    // Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2);
                }
                ++rootIterCount;
                ++exports.b2_toiRootIters;
                var s = fcn.Evaluate(indexA[0], indexB[0], t);
                if (b2Math_1.b2Abs(s - target) < tolerance) {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }
                // Ensure we continue to bracket the root.
                if (s > target) {
                    a1 = t;
                    s1 = s;
                }
                else {
                    a2 = t;
                    s2 = s;
                }
                if (rootIterCount === 50) {
                    break;
                }
            }
            exports.b2_toiMaxRootIters = b2Math_1.b2Max(exports.b2_toiMaxRootIters, rootIterCount);
            ++pushBackIter;
            if (pushBackIter === b2Settings_1.b2_maxPolygonVertices) {
                break;
            }
        }
        ++iter;
        ++exports.b2_toiIters;
        if (done) {
            break;
        }
        if (iter === k_maxIterations) {
            // Root finder got stuck. Semi-victory.
            output.state = b2TOIOutputState.e_failed;
            output.t = t1;
            break;
        }
    }
    exports.b2_toiMaxIters = b2Math_1.b2Max(exports.b2_toiMaxIters, iter);
    var time = timer.GetMilliseconds();
    exports.b2_toiMaxTime = b2Math_1.b2Max(exports.b2_toiMaxTime, time);
    exports.b2_toiTime += time;
}
exports.b2TimeOfImpact = b2TimeOfImpact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJUaW1lT2ZJbXBhY3QuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9Cb3gyRC9Cb3gyRC9Db2xsaXNpb24vYjJUaW1lT2ZJbXBhY3QudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLDBEQUEwRDtBQUMxRCxtREFBNEU7QUFDNUUsMkNBQXFGO0FBQ3JGLDZDQUE0QztBQUM1QywyQ0FBOEc7QUFFbkcsUUFBQSxVQUFVLEdBQVcsQ0FBQyxDQUFDO0FBQ3ZCLFFBQUEsYUFBYSxHQUFXLENBQUMsQ0FBQztBQUMxQixRQUFBLFdBQVcsR0FBVyxDQUFDLENBQUM7QUFDeEIsUUFBQSxXQUFXLEdBQVcsQ0FBQyxDQUFDO0FBQ3hCLFFBQUEsY0FBYyxHQUFXLENBQUMsQ0FBQztBQUMzQixRQUFBLGVBQWUsR0FBVyxDQUFDLENBQUM7QUFDNUIsUUFBQSxrQkFBa0IsR0FBVyxDQUFDLENBQUM7QUFDMUM7SUFDRSxrQkFBVSxHQUFHLENBQUMsQ0FBQztJQUNmLHFCQUFhLEdBQUcsQ0FBQyxDQUFDO0lBQ2xCLG1CQUFXLEdBQUcsQ0FBQyxDQUFDO0lBQ2hCLG1CQUFXLEdBQUcsQ0FBQyxDQUFDO0lBQ2hCLHNCQUFjLEdBQUcsQ0FBQyxDQUFDO0lBQ25CLHVCQUFlLEdBQUcsQ0FBQyxDQUFDO0lBQ3BCLDBCQUFrQixHQUFHLENBQUMsQ0FBQztBQUN6QixDQUFDO0FBUkQsb0NBUUM7QUFFRCxJQUFNLG9CQUFvQixHQUFnQixJQUFJLG9CQUFXLEVBQUUsQ0FBQztBQUM1RCxJQUFNLG9CQUFvQixHQUFnQixJQUFJLG9CQUFXLEVBQUUsQ0FBQztBQUM1RCxJQUFNLHVCQUF1QixHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7QUFDckQsSUFBTSx1QkFBdUIsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO0FBQ3JELElBQU0sdUJBQXVCLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztBQUNyRCxJQUFNLHNC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