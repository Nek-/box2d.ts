"use strict";
/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
var b2WheelJointDef = /** @class */ (function (_super) {
    __extends(b2WheelJointDef, _super);
    function b2WheelJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_wheelJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2(0, 0);
        _this.localAnchorB = new b2Math_1.b2Vec2(0, 0);
        _this.localAxisA = new b2Math_1.b2Vec2(1, 0);
        _this.enableMotor = false;
        _this.maxMotorTorque = 0;
        _this.motorSpeed = 0;
        _this.frequencyHz = 2;
        _this.dampingRatio = 0.7;
        return _this;
    }
    b2WheelJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
    };
    return b2WheelJointDef;
}(b2Joint_1.b2JointDef));
exports.b2WheelJointDef = b2WheelJointDef;
var b2WheelJoint = /** @class */ (function (_super) {
    __extends(b2WheelJoint, _super);
    function b2WheelJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_frequencyHz = 0;
        _this.m_dampingRatio = 0;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_localXAxisA = new b2Math_1.b2Vec2();
        _this.m_localYAxisA = new b2Math_1.b2Vec2();
        _this.m_impulse = 0;
        _this.m_motorImpulse = 0;
        _this.m_springImpulse = 0;
        _this.m_maxMotorTorque = 0;
        _this.m_motorSpeed = 0;
        _this.m_enableMotor = false;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_localCenterA = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_invMassA = 0;
        _this.m_invMassB = 0;
        _this.m_invIA = 0;
        _this.m_invIB = 0;
        _this.m_ax = new b2Math_1.b2Vec2();
        _this.m_ay = new b2Math_1.b2Vec2();
        _this.m_sAx = 0;
        _this.m_sBx = 0;
        _this.m_sAy = 0;
        _this.m_sBy = 0;
        _this.m_mass = 0;
        _this.m_motorMass = 0;
        _this.m_springMass = 0;
        _this.m_bias = 0;
        _this.m_gamma = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_rA = new b2Math_1.b2Vec2();
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_frequencyHz = b2Settings_1.b2Maybe(def.frequencyHz, 2);
        _this.m_dampingRatio = b2Settings_1.b2Maybe(def.dampingRatio, 0.7);
        _this.m_localAnchorA.Copy(b2Settings_1.b2Maybe(def.localAnchorA, b2Math_1.b2Vec2.ZERO));
        _this.m_localAnchorB.Copy(b2Settings_1.b2Maybe(def.localAnchorB, b2Math_1.b2Vec2.ZERO));
        _this.m_localXAxisA.Copy(b2Settings_1.b2Maybe(def.localAxisA, b2Math_1.b2Vec2.UNITX));
        b2Math_1.b2Vec2.CrossOneV(_this.m_localXAxisA, _this.m_localYAxisA);
        _this.m_maxMotorTorque = b2Settings_1.b2Maybe(def.maxMotorTorque, 0);
        _this.m_motorSpeed = b2Settings_1.b2Maybe(def.motorSpeed, 0);
        _this.m_enableMotor = b2Settings_1.b2Maybe(def.enableMotor, false);
        _this.m_ax.SetZero();
        _this.m_ay.SetZero();
        return _this;
    }
    b2WheelJoint.prototype.GetMotorSpeed = function () {
        return this.m_motorSpeed;
    };
    b2WheelJoint.prototype.GetMaxMotorTorque = function () {
        return this.m_maxMotorTorque;
    };
    b2WheelJoint.prototype.SetSpringFrequencyHz = function (hz) {
        this.m_frequencyHz = hz;
    };
    b2WheelJoint.prototype.GetSpringFrequencyHz = function () {
        return this.m_frequencyHz;
    };
    b2WheelJoint.prototype.SetSpringDampingRatio = function (ratio) {
        this.m_dampingRatio = ratio;
    };
    b2WheelJoint.prototype.GetSpringDampingRatio = function () {
        return this.m_dampingRatio;
    };
    b2WheelJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // Compute the effective masses.
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = cB + rB - cA - rA;
        var d = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVV(cB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVV(cA, rA, b2Math_1.b2Vec2.s_t1), b2WheelJoint.InitVelocityConstraints_s_d);
        // Point to line constraint
        {
            // m_ay = b2Mul(qA, m_localYAxisA);
            b2Math_1.b2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
            // m_sAy = b2Cross(d + rA, m_ay);
            this.m_sAy = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), this.m_ay);
            // m_sBy = b2Cross(rB, m_ay);
            this.m_sBy = b2Math_1.b2Vec2.CrossVV(rB, this.m_ay);
            this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;
            if (this.m_mass > 0) {
                this.m_mass = 1 / this.m_mass;
            }
        }
        // Spring constraint
        this.m_springMass = 0;
        this.m_bias = 0;
        this.m_gamma = 0;
        if (this.m_frequencyHz > 0) {
            // m_ax = b2Mul(qA, m_localXAxisA);
            b2Math_1.b2Rot.MulRV(qA, this.m_localXAxisA, this.m_ax);
            // m_sAx = b2Cross(d + rA, m_ax);
            this.m_sAx = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), this.m_ax);
            // m_sBx = b2Cross(rB, m_ax);
            this.m_sBx = b2Math_1.b2Vec2.CrossVV(rB, this.m_ax);
            var invMass = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;
            if (invMass > 0) {
                this.m_springMass = 1 / invMass;
                var C = b2Math_1.b2Vec2.DotVV(d, this.m_ax);
                // Frequency
                var omega = 2 * b2Settings_1.b2_pi * this.m_frequencyHz;
                // Damping coefficient
                var dc = 2 * this.m_springMass * this.m_dampingRatio * omega;
                // Spring stiffness
                var k = this.m_springMass * omega * omega;
                // magic formulas
                var h = data.step.dt;
                this.m_gamma = h * (dc + h * k);
                if (this.m_gamma > 0) {
                    this.m_gamma = 1 / this.m_gamma;
                }
                this.m_bias = C * h * k * this.m_gamma;
                this.m_springMass = invMass + this.m_gamma;
                if (this.m_springMass > 0) {
                    this.m_springMass = 1 / this.m_springMass;
                }
            }
        }
        else {
            this.m_springImpulse = 0;
        }
        // Rotational motor
        if (this.m_enableMotor) {
            this.m_motorMass = iA + iB;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        else {
            this.m_motorMass = 0;
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse *= data.step.dtRatio;
            this.m_springImpulse *= data.step.dtRatio;
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
            var P = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.MulSV(this.m_impulse, this.m_ay, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.MulSV(this.m_springImpulse, this.m_ax, b2Math_1.b2Vec2.s_t1), b2WheelJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
            var LA = this.m_impulse * this.m_sAy + this.m_springImpulse * this.m_sAx + this.m_motorImpulse;
            // float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
            var LB = this.m_impulse * this.m_sBy + this.m_springImpulse * this.m_sBx + this.m_motorImpulse;
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * LA;
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            wB += this.m_invIB * LB;
        }
        else {
            this.m_impulse = 0;
            this.m_springImpulse = 0;
            this.m_motorImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2WheelJoint.prototype.SolveVelocityConstraints = function (data) {
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // Solve spring constraint
        {
            var Cdot = b2Math_1.b2Vec2.DotVV(this.m_ax, b2Math_1.b2Vec2.SubVV(vB, vA, b2Math_1.b2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
            var impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
            this.m_springImpulse += impulse;
            // b2Vec2 P = impulse * m_ax;
            var P = b2Math_1.b2Vec2.MulSV(impulse, this.m_ax, b2WheelJoint.SolveVelocityConstraints_s_P);
            var LA = impulse * this.m_sAx;
            var LB = impulse * this.m_sBx;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // Solve rotational motor constraint
        {
            var Cdot = wB - wA - this.m_motorSpeed;
            var impulse = -this.m_motorMass * Cdot;
            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Math_1.b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve point to line constraint
        {
            var Cdot = b2Math_1.b2Vec2.DotVV(this.m_ay, b2Math_1.b2Vec2.SubVV(vB, vA, b2Math_1.b2Vec2.s_t0)) + this.m_sBy * wB - this.m_sAy * wA;
            var impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;
            // b2Vec2 P = impulse * m_ay;
            var P = b2Math_1.b2Vec2.MulSV(impulse, this.m_ay, b2WheelJoint.SolveVelocityConstraints_s_P);
            var LA = impulse * this.m_sAy;
            var LB = impulse * this.m_sBy;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2WheelJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = (cB - cA) + rB - rA;
        var d = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(rB, rA, b2Math_1.b2Vec2.s_t1), b2WheelJoint.SolvePositionConstraints_s_d);
        // b2Vec2 ay = b2Mul(qA, m_localYAxisA);
        var ay = b2Math_1.b2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
        // float32 sAy = b2Cross(d + rA, ay);
        var sAy = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), ay);
        // float32 sBy = b2Cross(rB, ay);
        var sBy = b2Math_1.b2Vec2.CrossVV(rB, ay);
        // float32 C = b2Dot(d, ay);
        var C = b2Math_1.b2Vec2.DotVV(d, this.m_ay);
        var k = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;
        var impulse;
        if (k !== 0) {
            impulse = -C / k;
        }
        else {
            impulse = 0;
        }
        // b2Vec2 P = impulse * ay;
        var P = b2Math_1.b2Vec2.MulSV(impulse, ay, b2WheelJoint.SolvePositionConstraints_s_P);
        var LA = impulse * sAy;
        var LB = impulse * sBy;
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * LA;
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return b2Math_1.b2Abs(C) <= b2Settings_1.b2_linearSlop;
    };
    b2WheelJoint.prototype.GetDefinition = function (def) {
        // DEBUG: b2Assert(false); // TODO
        return def;
    };
    b2WheelJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2WheelJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2WheelJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
        out.x = inv_dt * (this.m_impulse * this.m_ay.x + this.m_springImpulse * this.m_ax.x);
        out.y = inv_dt * (this.m_impulse * this.m_ay.y + this.m_springImpulse * this.m_ax.y);
        return out;
    };
    b2WheelJoint.prototype.GetReactionTorque = function (inv_dt) {
        return inv_dt * this.m_motorImpulse;
    };
    b2WheelJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2WheelJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2WheelJoint.prototype.GetLocalAxisA = function () { return this.m_localXAxisA; };
    b2WheelJoint.prototype.GetJointTranslation = function () {
        return this.GetPrismaticJointTranslation();
    };
    b2WheelJoint.prototype.GetJointSpeed = function () {
        return this.GetRevoluteJointSpeed();
    };
    b2WheelJoint.prototype.GetPrismaticJointTranslation = function () {
        var bA = this.m_bodyA;
        var bB = this.m_bodyB;
        var pA = bA.GetWorldPoint(this.m_localAnchorA, new b2Math_1.b2Vec2());
        var pB = bB.GetWorldPoint(this.m_localAnchorB, new b2Math_1.b2Vec2());
        var d = b2Math_1.b2Vec2.SubVV(pB, pA, new b2Math_1.b2Vec2());
        var axis = bA.GetWorldVector(this.m_localXAxisA, new b2Math_1.b2Vec2());
        var translation = b2Math_1.b2Vec2.DotVV(d, axis);
        return translation;
    };
    b2WheelJoint.prototype.GetPrismaticJointSpeed = function () {
        var bA = this.m_bodyA;
        var bB = this.m_bodyB;
        // b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
        // b2Vec2 pA = bA->m_sweep.c + rA;
        var pA = b2Math_1.b2Vec2.AddVV(bA.m_sweep.c, rA, b2Math_1.b2Vec2.s_t0); // pA uses s_t0
        // b2Vec2 pB = bB->m_sweep.c + rB;
        var pB = b2Math_1.b2Vec2.AddVV(bB.m_sweep.c, rB, b2Math_1.b2Vec2.s_t1); // pB uses s_t1
        // b2Vec2 d = pB - pA;
        var d = b2Math_1.b2Vec2.SubVV(pB, pA, b2Math_1.b2Vec2.s_t2); // d uses s_t2
        // b2Vec2 axis = b2Mul(bA.m_xf.q, m_localXAxisA);
        var axis = bA.GetWorldVector(this.m_localXAxisA, new b2Math_1.b2Vec2());
        var vA = bA.m_linearVelocity;
        var vB = bB.m_linearVelocity;
        var wA = bA.m_angularVelocity;
        var wB = bB.m_angularVelocity;
        // float32 speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
        var speed = b2Math_1.b2Vec2.DotVV(d, b2Math_1.b2Vec2.CrossSV(wA, axis, b2Math_1.b2Vec2.s_t0)) +
            b2Math_1.b2Vec2.DotVV(axis, b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, rA, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0));
        return speed;
    };
    b2WheelJoint.prototype.GetRevoluteJointAngle = function () {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_sweep.a - bA->this.m_sweep.a;
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
    };
    b2WheelJoint.prototype.GetRevoluteJointSpeed = function () {
        var wA = this.m_bodyA.m_angularVelocity;
        var wB = this.m_bodyB.m_angularVelocity;
        return wB - wA;
    };
    b2WheelJoint.prototype.IsMotorEnabled = function () {
        return this.m_enableMotor;
    };
    b2WheelJoint.prototype.EnableMotor = function (flag) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_enableMotor = flag;
    };
    b2WheelJoint.prototype.SetMotorSpeed = function (speed) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    };
    b2WheelJoint.prototype.SetMaxMotorTorque = function (force) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_maxMotorTorque = force;
    };
    b2WheelJoint.prototype.GetMotorTorque = function (inv_dt) {
        return inv_dt * this.m_motorImpulse;
    };
    b2WheelJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2WheelJointDef = new b2WheelJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.localAxisA.Set(%.15f, %.15f);\n", this.m_localXAxisA.x, this.m_localXAxisA.y);
        log("  jd.enableMotor = %s;\n", (this.m_enableMotor) ? ("true") : ("false"));
        log("  jd.motorSpeed = %.15f;\n", this.m_motorSpeed);
        log("  jd.maxMotorTorque = %.15f;\n", this.m_maxMotorTorque);
        log("  jd.frequencyHz = %.15f;\n", this.m_frequencyHz);
        log("  jd.dampingRatio = %.15f;\n", this.m_dampingRatio);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2WheelJoint.InitVelocityConstraints_s_d = new b2Math_1.b2Vec2();
    b2WheelJoint.InitVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2WheelJoint.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2WheelJoint.SolvePositionConstraints_s_d = new b2Math_1.b2Vec2();
    b2WheelJoint.SolvePositionConstraints_s_P = new b2Math_1.b2Vec2();
    return b2WheelJoint;
}(b2Joint_1.b2Joint));
exports.b2WheelJoint = b2WheelJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJXaGVlbEpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyV2hlZWxKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLDZEQUE2RDtBQUM3RCxzREFBd0U7QUFDeEUsOENBQXdFO0FBQ3hFLHFDQUEwRTtBQXNCMUUsNERBQTREO0FBQzVELHVFQUF1RTtBQUN2RSxvRUFBb0U7QUFDcEUsc0VBQXNFO0FBQ3RFLHFFQUFxRTtBQUNyRSxrRUFBa0U7QUFDbEU7SUFBcUMsbUNBQVU7SUFpQjdDO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLFlBQVksQ0FBQyxTQUNoQztRQWxCZSxrQkFBWSxHQUFXLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUV4QyxrQkFBWSxHQUFXLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUV4QyxnQkFBVSxHQUFXLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUUvQyxpQkFBVyxHQUFHLEtBQUssQ0FBQztRQUVwQixvQkFBYyxHQUFXLENBQUMsQ0FBQztRQUUzQixnQkFBVSxHQUFXLENBQUMsQ0FBQztRQUV2QixpQkFBVyxHQUFXLENBQUMsQ0FBQztRQUV4QixrQkFBWSxHQUFXLEdBQUcsQ0FBQzs7SUFJbEMsQ0FBQztJQUVNLG9DQUFVLEdBQWpCLFVBQWtCLEVBQVUsRUFBRSxFQUFVLEVBQUUsTUFBYyxFQUFFLElBQVk7UUFDcEUsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssQ0FBQyxhQUFhLENBQUMsTUFBTSxFQUFFLElBQUksQ0FBQyxZQUFZLENBQUMsQ0FB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