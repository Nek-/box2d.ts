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
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Collision_1 = require("../../Collision/b2Collision");
var b2Collision_2 = require("../../Collision/b2Collision");
var b2TimeStep_1 = require("../b2TimeStep");
var b2VelocityConstraintPoint = /** @class */ (function () {
    function b2VelocityConstraintPoint() {
        this.rA = new b2Math_1.b2Vec2();
        this.rB = new b2Math_1.b2Vec2();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.normalMass = 0;
        this.tangentMass = 0;
        this.velocityBias = 0;
    }
    b2VelocityConstraintPoint.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2VelocityConstraintPoint(); });
    };
    return b2VelocityConstraintPoint;
}());
exports.b2VelocityConstraintPoint = b2VelocityConstraintPoint;
var b2ContactVelocityConstraint = /** @class */ (function () {
    function b2ContactVelocityConstraint() {
        this.points = b2VelocityConstraintPoint.MakeArray(b2Settings_1.b2_maxManifoldPoints);
        this.normal = new b2Math_1.b2Vec2();
        this.tangent = new b2Math_1.b2Vec2();
        this.normalMass = new b2Math_1.b2Mat22();
        this.K = new b2Math_1.b2Mat22();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = 0;
        this.invMassB = 0;
        this.invIA = 0;
        this.invIB = 0;
        this.friction = 0;
        this.restitution = 0;
        this.tangentSpeed = 0;
        this.pointCount = 0;
        this.contactIndex = 0;
    }
    b2ContactVelocityConstraint.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2ContactVelocityConstraint(); });
    };
    return b2ContactVelocityConstraint;
}());
exports.b2ContactVelocityConstraint = b2ContactVelocityConstraint;
var b2ContactPositionConstraint = /** @class */ (function () {
    function b2ContactPositionConstraint() {
        this.localPoints = b2Math_1.b2Vec2.MakeArray(b2Settings_1.b2_maxManifoldPoints);
        this.localNormal = new b2Math_1.b2Vec2();
        this.localPoint = new b2Math_1.b2Vec2();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = 0;
        this.invMassB = 0;
        this.localCenterA = new b2Math_1.b2Vec2();
        this.localCenterB = new b2Math_1.b2Vec2();
        this.invIA = 0;
        this.invIB = 0;
        this.type = b2Collision_2.b2ManifoldType.e_unknown;
        this.radiusA = 0;
        this.radiusB = 0;
        this.pointCount = 0;
    }
    b2ContactPositionConstraint.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2ContactPositionConstraint(); });
    };
    return b2ContactPositionConstraint;
}());
exports.b2ContactPositionConstraint = b2ContactPositionConstraint;
var b2ContactSolverDef = /** @class */ (function () {
    function b2ContactSolverDef() {
        this.step = new b2TimeStep_1.b2TimeStep();
        this.count = 0;
        this.allocator = null;
    }
    return b2ContactSolverDef;
}());
exports.b2ContactSolverDef = b2ContactSolverDef;
var b2PositionSolverManifold = /** @class */ (function () {
    function b2PositionSolverManifold() {
        this.normal = new b2Math_1.b2Vec2();
        this.point = new b2Math_1.b2Vec2();
        this.separation = 0;
    }
    b2PositionSolverManifold.prototype.Initialize = function (pc, xfA, xfB, index) {
        var pointA = b2PositionSolverManifold.Initialize_s_pointA;
        var pointB = b2PositionSolverManifold.Initialize_s_pointB;
        var planePoint = b2PositionSolverManifold.Initialize_s_planePoint;
        var clipPoint = b2PositionSolverManifold.Initialize_s_clipPoint;
        // DEBUG: b2Assert(pc.pointCount > 0);
        switch (pc.type) {
            case b2Collision_2.b2ManifoldType.e_circles: {
                // b2Vec2 pointA = b2Mul(xfA, pc->localPoint);
                b2Math_1.b2Transform.MulXV(xfA, pc.localPoint, pointA);
                // b2Vec2 pointB = b2Mul(xfB, pc->localPoints[0]);
                b2Math_1.b2Transform.MulXV(xfB, pc.localPoints[0], pointB);
                // normal = pointB - pointA;
                // normal.Normalize();
                b2Math_1.b2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
                // point = 0.5f * (pointA + pointB);
                b2Math_1.b2Vec2.MidVV(pointA, pointB, this.point);
                // separation = b2Dot(pointB - pointA, normal) - pc->radius;
                this.separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pointB, pointA, b2Math_1.b2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                break;
            }
            case b2Collision_2.b2ManifoldType.e_faceA: {
                // normal = b2Mul(xfA.q, pc->localNormal);
                b2Math_1.b2Rot.MulRV(xfA.q, pc.localNormal, this.normal);
                // b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);
                b2Math_1.b2Transform.MulXV(xfA, pc.localPoint, planePoint);
                // b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
                b2Math_1.b2Transform.MulXV(xfB, pc.localPoints[index], clipPoint);
                // separation = b2Dot(clipPoint - planePoint, normal) - pc->radius;
                this.separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                // point = clipPoint;
                this.point.Copy(clipPoint);
                break;
            }
            case b2Collision_2.b2ManifoldType.e_faceB: {
                // normal = b2Mul(xfB.q, pc->localNormal);
                b2Math_1.b2Rot.MulRV(xfB.q, pc.localNormal, this.normal);
                // b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);
                b2Math_1.b2Transform.MulXV(xfB, pc.localPoint, planePoint);
                // b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
                b2Math_1.b2Transform.MulXV(xfA, pc.localPoints[index], clipPoint);
                // separation = b2Dot(clipPoint - planePoint, normal) - pc->radius;
                this.separation = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(clipPoint, planePoint, b2Math_1.b2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
                // point = clipPoint;
                this.point.Copy(clipPoint);
                // Ensure normal points from A to B
                // normal = -normal;
                this.normal.SelfNeg();
                break;
            }
        }
    };
    b2PositionSolverManifold.Initialize_s_pointA = new b2Math_1.b2Vec2();
    b2PositionSolverManifold.Initialize_s_pointB = new b2Math_1.b2Vec2();
    b2PositionSolverManifold.Initialize_s_planePoint = new b2Math_1.b2Vec2();
    b2PositionSolverManifold.Initialize_s_clipPoint = new b2Math_1.b2Vec2();
    return b2PositionSolverManifold;
}());
exports.b2PositionSolverManifold = b2PositionSolverManifold;
var b2ContactSolver = /** @class */ (function () {
    function b2ContactSolver() {
        this.m_step = new b2TimeStep_1.b2TimeStep();
        this.m_allocator = null;
        this.m_positionConstraints = b2ContactPositionConstraint.MakeArray(1024); // TODO: b2Settings
        this.m_velocityConstraints = b2ContactVelocityConstraint.MakeArray(1024); // TODO: b2Settings
        this.m_count = 0;
    }
    b2ContactSolver.prototype.Initialize = function (def) {
        this.m_step.Copy(def.step);
        this.m_allocator = def.allocator;
        this.m_count = def.count;
        // TODO:
        if (this.m_positionConstraints.length < this.m_count) {
            var new_length = b2Math_1.b2Max(this.m_positionConstraints.length * 2, this.m_count);
            while (this.m_positionConstraints.length < new_length) {
                this.m_positionConstraints[this.m_positionConstraints.length] = new b2ContactPositionConstraint();
            }
        }
        // TODO:
        if (this.m_velocityConstraints.length < this.m_count) {
            var new_length = b2Math_1.b2Max(this.m_velocityConstraints.length * 2, this.m_count);
            while (this.m_velocityConstraints.length < new_length) {
                this.m_velocityConstraints[this.m_velocityConstraints.length] = new b2ContactVelocityConstraint();
            }
        }
        this.m_positions = def.positions;
        this.m_velocities = def.velocities;
        this.m_contacts = def.contacts;
        // Initialize position independent portions of the constraints.
        for (var i = 0; i < this.m_count; ++i) {
            var contact = this.m_contacts[i];
            var fixtureA = contact.m_fixtureA;
            var fixtureB = contact.m_fixtureB;
            var shapeA = fixtureA.GetShape();
            var shapeB = fixtureB.GetShape();
            var radiusA = shapeA.m_radius;
            var radiusB = shapeB.m_radius;
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();
            var manifold = contact.GetManifold();
            var pointCount = manifold.pointCount;
            // DEBUG: b2Assert(pointCount > 0);
            var vc = this.m_velocityConstraints[i];
            vc.friction = contact.m_friction;
            vc.restitution = contact.m_restitution;
            vc.tangentSpeed = contact.m_tangentSpeed;
            vc.indexA = bodyA.m_islandIndex;
            vc.indexB = bodyB.m_islandIndex;
            vc.invMassA = bodyA.m_invMass;
            vc.invMassB = bodyB.m_invMass;
            vc.invIA = bodyA.m_invI;
            vc.invIB = bodyB.m_invI;
            vc.contactIndex = i;
            vc.pointCount = pointCount;
            vc.K.SetZero();
            vc.normalMass.SetZero();
            var pc = this.m_positionConstraints[i];
            pc.indexA = bodyA.m_islandIndex;
            pc.indexB = bodyB.m_islandIndex;
            pc.invMassA = bodyA.m_invMass;
            pc.invMassB = bodyB.m_invMass;
            pc.localCenterA.Copy(bodyA.m_sweep.localCenter);
            pc.localCenterB.Copy(bodyB.m_sweep.localCenter);
            pc.invIA = bodyA.m_invI;
            pc.invIB = bodyB.m_invI;
            pc.localNormal.Copy(manifold.localNormal);
            pc.localPoint.Copy(manifold.localPoint);
            pc.pointCount = pointCount;
            pc.radiusA = radiusA;
            pc.radiusB = radiusB;
            pc.type = manifold.type;
            for (var j = 0; j < pointCount; ++j) {
                var cp = manifold.points[j];
                var vcp = vc.points[j];
                if (this.m_step.warmStarting) {
                    vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
                    vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
                }
                else {
                    vcp.normalImpulse = 0;
                    vcp.tangentImpulse = 0;
                }
                vcp.rA.SetZero();
                vcp.rB.SetZero();
                vcp.normalMass = 0;
                vcp.tangentMass = 0;
                vcp.velocityBias = 0;
                pc.localPoints[j].Copy(cp.localPoint);
            }
        }
        return this;
    };
    b2ContactSolver.prototype.InitializeVelocityConstraints = function () {
        var xfA = b2ContactSolver.InitializeVelocityConstraints_s_xfA;
        var xfB = b2ContactSolver.InitializeVelocityConstraints_s_xfB;
        var worldManifold = b2ContactSolver.InitializeVelocityConstraints_s_worldManifold;
        var k_maxConditionNumber = 1000;
        for (var i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var pc = this.m_positionConstraints[i];
            var radiusA = pc.radiusA;
            var radiusB = pc.radiusB;
            var manifold = this.m_contacts[vc.contactIndex].GetManifold();
            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var mB = vc.invMassB;
            var iA = vc.invIA;
            var iB = vc.invIB;
            var localCenterA = pc.localCenterA;
            var localCenterB = pc.localCenterB;
            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;
            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;
            // DEBUG: b2Assert(manifold.pointCount > 0);
            xfA.q.SetAngle(aA);
            xfB.q.SetAngle(aB);
            b2Math_1.b2Vec2.SubVV(cA, b2Math_1.b2Rot.MulRV(xfA.q, localCenterA, b2Math_1.b2Vec2.s_t0), xfA.p);
            b2Math_1.b2Vec2.SubVV(cB, b2Math_1.b2Rot.MulRV(xfB.q, localCenterB, b2Math_1.b2Vec2.s_t0), xfB.p);
            worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);
            vc.normal.Copy(worldManifold.normal);
            b2Math_1.b2Vec2.CrossVOne(vc.normal, vc.tangent); // compute from normal
            var pointCount = vc.pointCount;
            for (var j = 0; j < pointCount; ++j) {
                var vcp = vc.points[j];
                // vcp->rA = worldManifold.points[j] - cA;
                b2Math_1.b2Vec2.SubVV(worldManifold.points[j], cA, vcp.rA);
                // vcp->rB = worldManifold.points[j] - cB;
                b2Math_1.b2Vec2.SubVV(worldManifold.points[j], cB, vcp.rB);
                var rnA = b2Math_1.b2Vec2.CrossVV(vcp.rA, vc.normal);
                var rnB = b2Math_1.b2Vec2.CrossVV(vcp.rB, vc.normal);
                var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0;
                // b2Vec2 tangent = b2Cross(vc->normal, 1.0f);
                var tangent = vc.tangent; // precomputed from normal
                var rtA = b2Math_1.b2Vec2.CrossVV(vcp.rA, tangent);
                var rtB = b2Math_1.b2Vec2.CrossVV(vcp.rB, tangent);
                var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;
                // Setup a velocity bias for restitution.
                vcp.velocityBias = 0;
                // float32 vRel = b2Dot(vc->normal, vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA));
                var vRel = b2Math_1.b2Vec2.DotVV(vc.normal, b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0));
                if (vRel < (-b2Settings_1.b2_velocityThreshold)) {
                    vcp.velocityBias += (-vc.restitution * vRel);
                }
            }
            // If we have two points, then prepare the block solver.
            if (vc.pointCount === 2) {
                var vcp1 = vc.points[0];
                var vcp2 = vc.points[1];
                var rn1A = b2Math_1.b2Vec2.CrossVV(vcp1.rA, vc.normal);
                var rn1B = b2Math_1.b2Vec2.CrossVV(vcp1.rB, vc.normal);
                var rn2A = b2Math_1.b2Vec2.CrossVV(vcp2.rA, vc.normal);
                var rn2B = b2Math_1.b2Vec2.CrossVV(vcp2.rB, vc.normal);
                var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
                // Ensure a reasonable condition number.
                // float32 k_maxConditionNumber = 1000.0f;
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    // K is safe to invert.
                    vc.K.ex.Set(k11, k12);
                    vc.K.ey.Set(k12, k22);
                    vc.K.GetInverse(vc.normalMass);
                }
                else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.pointCount = 1;
                }
            }
        }
    };
    b2ContactSolver.prototype.WarmStart = function () {
        var P = b2ContactSolver.WarmStart_s_P;
        // Warm start.
        for (var i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var iA = vc.invIA;
            var mB = vc.invMassB;
            var iB = vc.invIB;
            var pointCount = vc.pointCount;
            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;
            var normal = vc.normal;
            // b2Vec2 tangent = b2Cross(normal, 1.0f);
            var tangent = vc.tangent; // precomputed from normal
            for (var j = 0; j < pointCount; ++j) {
                var vcp = vc.points[j];
                // b2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
                b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.MulSV(vcp.normalImpulse, normal, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.MulSV(vcp.tangentImpulse, tangent, b2Math_1.b2Vec2.s_t1), P);
                // wA -= iA * b2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.b2Vec2.CrossVV(vcp.rA, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wB += iB * b2Cross(vcp->rB, P);
                wB += iB * b2Math_1.b2Vec2.CrossVV(vcp.rB, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
            }
            // this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
            // this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    };
    b2ContactSolver.prototype.SolveVelocityConstraints = function () {
        var dv = b2ContactSolver.SolveVelocityConstraints_s_dv;
        var dv1 = b2ContactSolver.SolveVelocityConstraints_s_dv1;
        var dv2 = b2ContactSolver.SolveVelocityConstraints_s_dv2;
        var P = b2ContactSolver.SolveVelocityConstraints_s_P;
        var a = b2ContactSolver.SolveVelocityConstraints_s_a;
        var b = b2ContactSolver.SolveVelocityConstraints_s_b;
        var x = b2ContactSolver.SolveVelocityConstraints_s_x;
        var d = b2ContactSolver.SolveVelocityConstraints_s_d;
        var P1 = b2ContactSolver.SolveVelocityConstraints_s_P1;
        var P2 = b2ContactSolver.SolveVelocityConstraints_s_P2;
        var P1P2 = b2ContactSolver.SolveVelocityConstraints_s_P1P2;
        for (var i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var iA = vc.invIA;
            var mB = vc.invMassB;
            var iB = vc.invIB;
            var pointCount = vc.pointCount;
            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;
            // b2Vec2 normal = vc->normal;
            var normal = vc.normal;
            // b2Vec2 tangent = b2Cross(normal, 1.0f);
            var tangent = vc.tangent; // precomputed from normal
            var friction = vc.friction;
            // DEBUG: b2Assert(pointCount === 1 || pointCount === 2);
            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for (var j = 0; j < pointCount; ++j) {
                var vcp = vc.points[j];
                // Relative velocity at contact
                // b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
                b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.b2Vec2.s_t1), dv);
                // Compute tangent force
                // float32 vt = b2Dot(dv, tangent) - vc->tangentSpeed;
                var vt = b2Math_1.b2Vec2.DotVV(dv, tangent) - vc.tangentSpeed;
                var lambda = vcp.tangentMass * (-vt);
                // b2Clamp the accumulated force
                var maxFriction = friction * vcp.normalImpulse;
                var newImpulse = b2Math_1.b2Clamp(vcp.tangentImpulse + lambda, (-maxFriction), maxFriction);
                lambda = newImpulse - vcp.tangentImpulse;
                vcp.tangentImpulse = newImpulse;
                // Apply contact impulse
                // b2Vec2 P = lambda * tangent;
                b2Math_1.b2Vec2.MulSV(lambda, tangent, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wA -= iA * b2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.b2Vec2.CrossVV(vcp.rA, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
                // wB += iB * b2Cross(vcp->rB, P);
                wB += iB * b2Math_1.b2Vec2.CrossVV(vcp.rB, P);
            }
            // Solve normal constraints
            if (vc.pointCount === 1) {
                var vcp = vc.points[0];
                // Relative velocity at contact
                // b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
                b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.b2Vec2.s_t1), dv);
                // Compute normal impulse
                // float32 vn = b2Dot(dv, normal);
                var vn = b2Math_1.b2Vec2.DotVV(dv, normal);
                var lambda = (-vcp.normalMass * (vn - vcp.velocityBias));
                // b2Clamp the accumulated impulse
                // float32 newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0f);
                var newImpulse = b2Math_1.b2Max(vcp.normalImpulse + lambda, 0);
                lambda = newImpulse - vcp.normalImpulse;
                vcp.normalImpulse = newImpulse;
                // Apply contact impulse
                // b2Vec2 P = lambda * normal;
                b2Math_1.b2Vec2.MulSV(lambda, normal, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wA -= iA * b2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.b2Vec2.CrossVV(vcp.rA, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
                // wB += iB * b2Cross(vcp->rB, P);
                wB += iB * b2Math_1.b2Vec2.CrossVV(vcp.rB, P);
            }
            else {
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch
                //
                // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a;
                var cp1 = vc.points[0];
                var cp2 = vc.points[1];
                // b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
                a.Set(cp1.normalImpulse, cp2.normalImpulse);
                // DEBUG: b2Assert(a.x >= 0 && a.y >= 0);
                // Relative velocity at contact
                // b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
                b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, cp1.rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, cp1.rA, b2Math_1.b2Vec2.s_t1), dv1);
                // b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
                b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, cp2.rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, cp2.rA, b2Math_1.b2Vec2.s_t1), dv2);
                // Compute normal velocity
                // float32 vn1 = b2Dot(dv1, normal);
                var vn1 = b2Math_1.b2Vec2.DotVV(dv1, normal);
                // float32 vn2 = b2Dot(dv2, normal);
                var vn2 = b2Math_1.b2Vec2.DotVV(dv2, normal);
                // b2Vec2 b;
                b.x = vn1 - cp1.velocityBias;
                b.y = vn2 - cp2.velocityBias;
                // Compute b'
                // b -= b2Mul(vc->K, a);
                b.SelfSub(b2Math_1.b2Mat22.MulMV(vc.K, a, b2Math_1.b2Vec2.s_t0));
                /*
                #if B2_DEBUG_SOLVER === 1
                const k_errorTol: number = 0.001;
                #endif
                */
                for (;;) {
                    //
                    // Case 1: vn = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = - inv(A) * b'
                    //
                    // b2Vec2 x = - b2Mul(vc->normalMass, b);
                    b2Math_1.b2Mat22.MulMV(vc.normalMass, b, x).SelfNeg();
                    if (x.x >= 0 && x.y >= 0) {
                        // Get the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Math_1.b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Math_1.b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Math_1.b2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.b2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.b2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                        #if B2_DEBUG_SOLVER === 1
                        // Postconditions
                        dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
                        dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
            
                        // Compute normal velocity
                        vn1 = b2Dot(dv1, normal);
                        vn2 = b2Dot(dv2, normal);
            
                        b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
                        b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
                        #endif
                        */
                        break;
                    }
                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.x = (-cp1.normalMass * b.x);
                    x.y = 0;
                    vn1 = 0;
                    vn2 = vc.K.ex.y * x.x + b.y;
                    if (x.x >= 0 && vn2 >= 0) {
                        // Get the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Math_1.b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Math_1.b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Math_1.b2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.b2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.b2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                        #if B2_DEBUG_SOLVER === 1
                        // Postconditions
                        dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
            
                        // Compute normal velocity
                        vn1 = b2Dot(dv1, normal);
            
                        b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
                        #endif
                        */
                        break;
                    }
                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.x = 0;
                    x.y = (-cp2.normalMass * b.y);
                    vn1 = vc.K.ey.x * x.y + b.x;
                    vn2 = 0;
                    if (x.y >= 0 && vn1 >= 0) {
                        // Resubstitute for the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Math_1.b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Math_1.b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Math_1.b2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.b2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.b2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        /*
                        #if B2_DEBUG_SOLVER === 1
                        // Postconditions
                        dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
            
                        // Compute normal velocity
                        vn2 = b2Dot(dv2, normal);
            
                        b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
                        #endif
                        */
                        break;
                    }
                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2;
                    x.x = 0;
                    x.y = 0;
                    vn1 = b.x;
                    vn2 = b.y;
                    if (vn1 >= 0 && vn2 >= 0) {
                        // Resubstitute for the incremental impulse
                        // b2Vec2 d = x - a;
                        b2Math_1.b2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // b2Vec2 P1 = d.x * normal;
                        b2Math_1.b2Vec2.MulSV(d.x, normal, P1);
                        // b2Vec2 P2 = d.y * normal;
                        b2Math_1.b2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.b2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.b2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.b2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.b2Vec2.CrossVV(cp2.rB, P2));
                        // Accumulate
                        cp1.normalImpulse = x.x;
                        cp2.normalImpulse = x.y;
                        break;
                    }
                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }
            // this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
            // this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    };
    b2ContactSolver.prototype.StoreImpulses = function () {
        for (var i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var manifold = this.m_contacts[vc.contactIndex].GetManifold();
            for (var j = 0; j < vc.pointCount; ++j) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
            }
        }
    };
    b2ContactSolver.prototype.SolvePositionConstraints = function () {
        var xfA = b2ContactSolver.SolvePositionConstraints_s_xfA;
        var xfB = b2ContactSolver.SolvePositionConstraints_s_xfB;
        var psm = b2ContactSolver.SolvePositionConstraints_s_psm;
        var rA = b2ContactSolver.SolvePositionConstraints_s_rA;
        var rB = b2ContactSolver.SolvePositionConstraints_s_rB;
        var P = b2ContactSolver.SolvePositionConstraints_s_P;
        var minSeparation = 0;
        for (var i = 0; i < this.m_count; ++i) {
            var pc = this.m_positionConstraints[i];
            var indexA = pc.indexA;
            var indexB = pc.indexB;
            var localCenterA = pc.localCenterA;
            var mA = pc.invMassA;
            var iA = pc.invIA;
            var localCenterB = pc.localCenterB;
            var mB = pc.invMassB;
            var iB = pc.invIB;
            var pointCount = pc.pointCount;
            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;
            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (var j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Math_1.b2Vec2.SubVV(cA, b2Math_1.b2Rot.MulRV(xfA.q, localCenterA, b2Math_1.b2Vec2.s_t0), xfA.p);
                b2Math_1.b2Vec2.SubVV(cB, b2Math_1.b2Rot.MulRV(xfB.q, localCenterB, b2Math_1.b2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                var normal = psm.normal;
                var point = psm.point;
                var separation = psm.separation;
                // b2Vec2 rA = point - cA;
                b2Math_1.b2Vec2.SubVV(point, cA, rA);
                // b2Vec2 rB = point - cB;
                b2Math_1.b2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Math_1.b2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                var C = b2Math_1.b2Clamp(b2Settings_1.b2_baumgarte * (separation + b2Settings_1.b2_linearSlop), (-b2Settings_1.b2_maxLinearCorrection), 0);
                // Compute the effective mass.
                // float32 rnA = b2Cross(rA, normal);
                var rnA = b2Math_1.b2Vec2.CrossVV(rA, normal);
                // float32 rnB = b2Cross(rB, normal);
                var rnB = b2Math_1.b2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                var impulse = K > 0 ? -C / K : 0;
                // b2Vec2 P = impulse * normal;
                b2Math_1.b2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * b2Cross(rA, P);
                aA -= iA * b2Math_1.b2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * b2Cross(rB, P);
                aB += iB * b2Math_1.b2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation > (-3 * b2Settings_1.b2_linearSlop);
    };
    b2ContactSolver.prototype.SolveTOIPositionConstraints = function (toiIndexA, toiIndexB) {
        var xfA = b2ContactSolver.SolveTOIPositionConstraints_s_xfA;
        var xfB = b2ContactSolver.SolveTOIPositionConstraints_s_xfB;
        var psm = b2ContactSolver.SolveTOIPositionConstraints_s_psm;
        var rA = b2ContactSolver.SolveTOIPositionConstraints_s_rA;
        var rB = b2ContactSolver.SolveTOIPositionConstraints_s_rB;
        var P = b2ContactSolver.SolveTOIPositionConstraints_s_P;
        var minSeparation = 0;
        for (var i = 0; i < this.m_count; ++i) {
            var pc = this.m_positionConstraints[i];
            var indexA = pc.indexA;
            var indexB = pc.indexB;
            var localCenterA = pc.localCenterA;
            var localCenterB = pc.localCenterB;
            var pointCount = pc.pointCount;
            var mA = 0;
            var iA = 0;
            if (indexA === toiIndexA || indexA === toiIndexB) {
                mA = pc.invMassA;
                iA = pc.invIA;
            }
            var mB = 0;
            var iB = 0;
            if (indexB === toiIndexA || indexB === toiIndexB) {
                mB = pc.invMassB;
                iB = pc.invIB;
            }
            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;
            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (var j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Math_1.b2Vec2.SubVV(cA, b2Math_1.b2Rot.MulRV(xfA.q, localCenterA, b2Math_1.b2Vec2.s_t0), xfA.p);
                b2Math_1.b2Vec2.SubVV(cB, b2Math_1.b2Rot.MulRV(xfB.q, localCenterB, b2Math_1.b2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                var normal = psm.normal;
                var point = psm.point;
                var separation = psm.separation;
                // b2Vec2 rA = point - cA;
                b2Math_1.b2Vec2.SubVV(point, cA, rA);
                // b2Vec2 rB = point - cB;
                b2Math_1.b2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Math_1.b2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                var C = b2Math_1.b2Clamp(b2Settings_1.b2_toiBaumgarte * (separation + b2Settings_1.b2_linearSlop), (-b2Settings_1.b2_maxLinearCorrection), 0);
                // Compute the effective mass.
                // float32 rnA = b2Cross(rA, normal);
                var rnA = b2Math_1.b2Vec2.CrossVV(rA, normal);
                // float32 rnB = b2Cross(rB, normal);
                var rnB = b2Math_1.b2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                var impulse = K > 0 ? -C / K : 0;
                // b2Vec2 P = impulse * normal;
                b2Math_1.b2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * b2Cross(rA, P);
                aA -= iA * b2Math_1.b2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * b2Cross(rB, P);
                aB += iB * b2Math_1.b2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation >= -1.5 * b2Settings_1.b2_linearSlop;
    };
    b2ContactSolver.InitializeVelocityConstraints_s_xfA = new b2Math_1.b2Transform();
    b2ContactSolver.InitializeVelocityConstraints_s_xfB = new b2Math_1.b2Transform();
    b2ContactSolver.InitializeVelocityConstraints_s_worldManifold = new b2Collision_1.b2WorldManifold();
    b2ContactSolver.WarmStart_s_P = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_dv = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_dv1 = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_dv2 = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_a = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_b = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_x = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_d = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_P1 = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_P2 = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveVelocityConstraints_s_P1P2 = new b2Math_1.b2Vec2();
    b2ContactSolver.SolvePositionConstraints_s_xfA = new b2Math_1.b2Transform();
    b2ContactSolver.SolvePositionConstraints_s_xfB = new b2Math_1.b2Transform();
    b2ContactSolver.SolvePositionConstraints_s_psm = new b2PositionSolverManifold();
    b2ContactSolver.SolvePositionConstraints_s_rA = new b2Math_1.b2Vec2();
    b2ContactSolver.SolvePositionConstraints_s_rB = new b2Math_1.b2Vec2();
    b2ContactSolver.SolvePositionConstraints_s_P = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveTOIPositionConstraints_s_xfA = new b2Math_1.b2Transform();
    b2ContactSolver.SolveTOIPositionConstraints_s_xfB = new b2Math_1.b2Transform();
    b2ContactSolver.SolveTOIPositionConstraints_s_psm = new b2PositionSolverManifold();
    b2ContactSolver.SolveTOIPositionConstraints_s_rA = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveTOIPositionConstraints_s_rB = new b2Math_1.b2Vec2();
    b2ContactSolver.SolveTOIPositionConstraints_s_P = new b2Math_1.b2Vec2();
    return b2ContactSolver;
}());
exports.b2ContactSolver = b2ContactSolver;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250YWN0U29sdmVyLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvQ29udGFjdHMvYjJDb250YWN0U29sdmVyLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiw2REFBNkQ7QUFDN0Qsc0RBQXdLO0FBQ3hLLDhDQUFpRztBQUdqRywyREFBOEQ7QUFDOUQsMkRBQTZEO0FBSzdELDRDQUFtRTtBQUVuRTtJQUFBO1FBQ2tCLE9BQUUsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQzFCLE9BQUUsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ25DLGtCQUFhLEdBQVcsQ0FBQyxDQUFDO1FBQzFCLG1CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBQzNCLGVBQVUsR0FBVyxDQUFDLENBQUM7UUFDdkIsZ0JBQVcsR0FBVyxDQUFDLENBQUM7UUFDeEIsaUJBQVksR0FBVyxDQUFDLENBQUM7SUFLbEMsQ0FBQztJQUhlLG1DQUFTLEdBQXZCLFVBQXdCLE1BQWM7UUFDcEMsT0FBTyx3QkFBVyxDQUFDLE1BQU0sRUFBRSxVQUFDLENBQVMsSUFBSyxPQUFBLElBQUkseUJBQXlCLEVBQUUsRUFBL0IsQ0FBK0IsQ0FBQyxDQUFDO0lBQzdFLENBQUM7SUFDSCxnQ0FBQztBQUFELENBQUMsQUFaRCxJQVlDO0FBWlksOERBQXlCO0FBY3RDO0lBQUE7UUFDUyxXQUFNLEdBQWdDLHlCQUF5QixDQUFDLFNBQVMsQ0FBQyxpQ0FBb0IsQ0FBQyxDQUFDO1FBQ3ZGLFdBQU0sR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQzlCLFlBQU8sR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQy9CLGVBQVUsR0FBWSxJQUFJLGdCQUFPLEVBQUUsQ0FBQztRQUNwQyxNQUFDLEdBQVksSUFBSSxnQkFBTyxFQUFFLENBQUM7UUFDcEMsV0FBTSxHQUFXLENBQUMsQ0FBQztR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