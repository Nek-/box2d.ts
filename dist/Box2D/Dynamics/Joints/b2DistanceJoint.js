"use strict";
/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
var b2DistanceJointDef = /** @class */ (function (_super) {
    __extends(b2DistanceJointDef, _super);
    function b2DistanceJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_distanceJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2();
        _this.localAnchorB = new b2Math_1.b2Vec2();
        _this.length = 1;
        _this.frequencyHz = 0;
        _this.dampingRatio = 0;
        return _this;
    }
    b2DistanceJointDef.prototype.Initialize = function (b1, b2, anchor1, anchor2) {
        this.bodyA = b1;
        this.bodyB = b2;
        this.bodyA.GetLocalPoint(anchor1, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor2, this.localAnchorB);
        this.length = b2Math_1.b2Vec2.DistanceVV(anchor1, anchor2);
        this.frequencyHz = 0;
        this.dampingRatio = 0;
    };
    return b2DistanceJointDef;
}(b2Joint_1.b2JointDef));
exports.b2DistanceJointDef = b2DistanceJointDef;
var b2DistanceJoint = /** @class */ (function (_super) {
    __extends(b2DistanceJoint, _super);
    function b2DistanceJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_frequencyHz = 0;
        _this.m_dampingRatio = 0;
        _this.m_bias = 0;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_gamma = 0;
        _this.m_impulse = 0;
        _this.m_length = 0;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_u = new b2Math_1.b2Vec2();
        _this.m_rA = new b2Math_1.b2Vec2();
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_localCenterA = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_invMassA = 0;
        _this.m_invMassB = 0;
        _this.m_invIA = 0;
        _this.m_invIB = 0;
        _this.m_mass = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_frequencyHz = b2Settings_1.b2Maybe(def.frequencyHz, 0);
        _this.m_dampingRatio = b2Settings_1.b2Maybe(def.dampingRatio, 0);
        _this.m_localAnchorA.Copy(def.localAnchorA);
        _this.m_localAnchorB.Copy(def.localAnchorB);
        _this.m_length = def.length;
        return _this;
    }
    b2DistanceJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2DistanceJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2DistanceJoint.prototype.GetReactionForce = function (inv_dt, out) {
        out.x = inv_dt * this.m_impulse * this.m_u.x;
        out.y = inv_dt * this.m_impulse * this.m_u.y;
        return out;
    };
    b2DistanceJoint.prototype.GetReactionTorque = function (inv_dt) {
        return 0;
    };
    b2DistanceJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2DistanceJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2DistanceJoint.prototype.SetLength = function (length) {
        this.m_length = length;
    };
    b2DistanceJoint.prototype.Length = function () {
        return this.m_length;
    };
    b2DistanceJoint.prototype.SetFrequency = function (hz) {
        this.m_frequencyHz = hz;
    };
    b2DistanceJoint.prototype.GetFrequency = function () {
        return this.m_frequencyHz;
    };
    b2DistanceJoint.prototype.SetDampingRatio = function (ratio) {
        this.m_dampingRatio = ratio;
    };
    b2DistanceJoint.prototype.GetDampingRatio = function () {
        return this.m_dampingRatio;
    };
    b2DistanceJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2DistanceJointDef = new b2DistanceJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.length = %.15f;\n", this.m_length);
        log("  jd.frequencyHz = %.15f;\n", this.m_frequencyHz);
        log("  jd.dampingRatio = %.15f;\n", this.m_dampingRatio);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2DistanceJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // const qA: b2Rot = new b2Rot(aA), qB: b2Rot = new b2Rot(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // m_u = cB + m_rB - cA - m_rA;
        this.m_u.x = cB.x + this.m_rB.x - cA.x - this.m_rA.x;
        this.m_u.y = cB.y + this.m_rB.y - cA.y - this.m_rA.y;
        // Handle singularity.
        var length = this.m_u.Length();
        if (length > b2Settings_1.b2_linearSlop) {
            this.m_u.SelfMul(1 / length);
        }
        else {
            this.m_u.SetZero();
        }
        // float32 crAu = b2Cross(m_rA, m_u);
        var crAu = b2Math_1.b2Vec2.CrossVV(this.m_rA, this.m_u);
        // float32 crBu = b2Cross(m_rB, m_u);
        var crBu = b2Math_1.b2Vec2.CrossVV(this.m_rB, this.m_u);
        // float32 invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;
        var invMass = this.m_invMassA + this.m_invIA * crAu * crAu + this.m_invMassB + this.m_invIB * crBu * crBu;
        // Compute the effective mass matrix.
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        if (this.m_frequencyHz > 0) {
            var C = length - this.m_length;
            // Frequency
            var omega = 2 * b2Settings_1.b2_pi * this.m_frequencyHz;
            // Damping coefficient
            var d = 2 * this.m_mass * this.m_dampingRatio * omega;
            // Spring stiffness
            var k = this.m_mass * omega * omega;
            // magic formulas
            var h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
            this.m_bias = C * h * k * this.m_gamma;
            invMass += this.m_gamma;
            this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        }
        else {
            this.m_gamma = 0;
            this.m_bias = 0;
        }
        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse * m_u;
            var P = b2Math_1.b2Vec2.MulSV(this.m_impulse, this.m_u, b2DistanceJoint.InitVelocityConstraints_s_P);
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            // wA -= m_invIA * b2Cross(m_rA, P);
            wA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, P);
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            // wB += m_invIB * b2Cross(m_rB, P);
            wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, P);
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2DistanceJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // b2Vec2 vpA = vA + b2Cross(wA, m_rA);
        var vpA = b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2DistanceJoint.SolveVelocityConstraints_s_vpA);
        // b2Vec2 vpB = vB + b2Cross(wB, m_rB);
        var vpB = b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2DistanceJoint.SolveVelocityConstraints_s_vpB);
        // float32 Cdot = b2Dot(m_u, vpB - vpA);
        var Cdot = b2Math_1.b2Vec2.DotVV(this.m_u, b2Math_1.b2Vec2.SubVV(vpB, vpA, b2Math_1.b2Vec2.s_t0));
        var impulse = (-this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse));
        this.m_impulse += impulse;
        // b2Vec2 P = impulse * m_u;
        var P = b2Math_1.b2Vec2.MulSV(impulse, this.m_u, b2DistanceJoint.SolveVelocityConstraints_s_P);
        // vA -= m_invMassA * P;
        vA.SelfMulSub(this.m_invMassA, P);
        // wA -= m_invIA * b2Cross(m_rA, P);
        wA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, P);
        // vB += m_invMassB * P;
        vB.SelfMulAdd(this.m_invMassB, P);
        // wB += m_invIB * b2Cross(m_rB, P);
        wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, P);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2DistanceJoint.prototype.SolvePositionConstraints = function (data) {
        if (this.m_frequencyHz > 0) {
            // There is no position correction for soft distance constraints.
            return true;
        }
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        // const qA: b2Rot = new b2Rot(aA), qB: b2Rot = new b2Rot(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA); // use m_rA
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB); // use m_rB
        // b2Vec2 u = cB + rB - cA - rA;
        var u = this.m_u; // use m_u
        u.x = cB.x + rB.x - cA.x - rA.x;
        u.y = cB.y + rB.y - cA.y - rA.y;
        // float32 length = u.Normalize();
        var length = this.m_u.Normalize();
        // float32 C = length - m_length;
        var C = length - this.m_length;
        C = b2Math_1.b2Clamp(C, (-b2Settings_1.b2_maxLinearCorrection), b2Settings_1.b2_maxLinearCorrection);
        var impulse = (-this.m_mass * C);
        // b2Vec2 P = impulse * u;
        var P = b2Math_1.b2Vec2.MulSV(impulse, u, b2DistanceJoint.SolvePositionConstraints_s_P);
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        // aA -= m_invIA * b2Cross(rA, P);
        aA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(rA, P);
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        // aB += m_invIB * b2Cross(rB, P);
        aB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(rB, P);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return b2Math_1.b2Abs(C) < b2Settings_1.b2_linearSlop;
    };
    b2DistanceJoint.InitVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2DistanceJoint.SolveVelocityConstraints_s_vpA = new b2Math_1.b2Vec2();
    b2DistanceJoint.SolveVelocityConstraints_s_vpB = new b2Math_1.b2Vec2();
    b2DistanceJoint.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2DistanceJoint.SolvePositionConstraints_s_P = new b2Math_1.b2Vec2();
    return b2DistanceJoint;
}(b2Joint_1.b2Joint));
exports.b2DistanceJoint = b2DistanceJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEaXN0YW5jZUpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyRGlzdGFuY2VKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLHNEQUFnRztBQUNoRyw4Q0FBd0U7QUFDeEUscUNBQTBFO0FBWTFFLHdEQUF3RDtBQUN4RCw4REFBOEQ7QUFDOUQsMkRBQTJEO0FBQzNELGdFQUFnRTtBQUNoRSx3REFBd0Q7QUFDeEQsK0NBQStDO0FBQy9DO0lBQXdDLHNDQUFVO0lBT2hEO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLGVBQWUsQ0FBQyxTQUNuQztRQVJlLGtCQUFZLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUNwQyxrQkFBWSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDN0MsWUFBTSxHQUFXLENBQUMsQ0FBQztRQUNuQixpQkFBVyxHQUFXLENBQUMsQ0FBQztRQUN4QixrQkFBWSxHQUFXLENBQUMsQ0FBQzs7SUFJaEMsQ0FBQztJQUVNLHVDQUFVLEdBQWpCLFVBQWtCLEVBQVUsRUFBRSxFQUFVLEVBQUUsT0FBVyxFQUFFLE9BQVc7UUFDaEUsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssQ0FBQyxhQUFhLENBQUMsT0FBTyxFQUFFLElBQUksQ0FBQyxZQUFZLENBQUMsQ0FBQztRQUNyRCxJQUFJLENBQUMsS0FBSyxDQUFDLGFBQWEsQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLFlBQVksQ0FBQyxDQUFDO1FBQ3JELElBQUksQ0FBQyxNQUFNLEdBQUcsZUFBTSxDQUFDLFVBQVUsQ0FBQyxPQUFPLEVBQUUsT0FBTyxDQUFDLENBQUM7UUFDbEQsSUFBSSxDQUFDLFdB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