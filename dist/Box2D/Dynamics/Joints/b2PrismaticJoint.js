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
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
var b2PrismaticJointDef = /** @class */ (function (_super) {
    __extends(b2PrismaticJointDef, _super);
    function b2PrismaticJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_prismaticJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2();
        _this.localAnchorB = new b2Math_1.b2Vec2();
        _this.localAxisA = new b2Math_1.b2Vec2(1, 0);
        _this.referenceAngle = 0;
        _this.enableLimit = false;
        _this.lowerTranslation = 0;
        _this.upperTranslation = 0;
        _this.enableMotor = false;
        _this.maxMotorForce = 0;
        _this.motorSpeed = 0;
        return _this;
    }
    b2PrismaticJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    };
    return b2PrismaticJointDef;
}(b2Joint_1.b2JointDef));
exports.b2PrismaticJointDef = b2PrismaticJointDef;
var b2PrismaticJoint = /** @class */ (function (_super) {
    __extends(b2PrismaticJoint, _super);
    function b2PrismaticJoint(def) {
        var _this = _super.call(this, def) || this;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_localXAxisA = new b2Math_1.b2Vec2();
        _this.m_localYAxisA = new b2Math_1.b2Vec2();
        _this.m_referenceAngle = 0;
        _this.m_impulse = new b2Math_1.b2Vec3(0, 0, 0);
        _this.m_motorImpulse = 0;
        _this.m_lowerTranslation = 0;
        _this.m_upperTranslation = 0;
        _this.m_maxMotorForce = 0;
        _this.m_motorSpeed = 0;
        _this.m_enableLimit = false;
        _this.m_enableMotor = false;
        _this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_localCenterA = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_invMassA = 0;
        _this.m_invMassB = 0;
        _this.m_invIA = 0;
        _this.m_invIB = 0;
        _this.m_axis = new b2Math_1.b2Vec2(0, 0);
        _this.m_perp = new b2Math_1.b2Vec2(0, 0);
        _this.m_s1 = 0;
        _this.m_s2 = 0;
        _this.m_a1 = 0;
        _this.m_a2 = 0;
        _this.m_K = new b2Math_1.b2Mat33();
        _this.m_K3 = new b2Math_1.b2Mat33();
        _this.m_K2 = new b2Math_1.b2Mat22();
        _this.m_motorMass = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_rA = new b2Math_1.b2Vec2();
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_localAnchorA.Copy(b2Settings_1.b2Maybe(def.localAnchorA, b2Math_1.b2Vec2.ZERO));
        _this.m_localAnchorB.Copy(b2Settings_1.b2Maybe(def.localAnchorB, b2Math_1.b2Vec2.ZERO));
        _this.m_localXAxisA.Copy(b2Settings_1.b2Maybe(def.localAxisA, new b2Math_1.b2Vec2(1, 0))).SelfNormalize();
        b2Math_1.b2Vec2.CrossOneV(_this.m_localXAxisA, _this.m_localYAxisA);
        _this.m_referenceAngle = b2Settings_1.b2Maybe(def.referenceAngle, 0);
        _this.m_lowerTranslation = b2Settings_1.b2Maybe(def.lowerTranslation, 0);
        _this.m_upperTranslation = b2Settings_1.b2Maybe(def.upperTranslation, 0);
        _this.m_maxMotorForce = b2Settings_1.b2Maybe(def.maxMotorForce, 0);
        _this.m_motorSpeed = b2Settings_1.b2Maybe(def.motorSpeed, 0);
        _this.m_enableLimit = b2Settings_1.b2Maybe(def.enableLimit, false);
        _this.m_enableMotor = b2Settings_1.b2Maybe(def.enableMotor, false);
        return _this;
    }
    b2PrismaticJoint.prototype.InitVelocityConstraints = function (data) {
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
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // Compute the effective masses.
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = (cB - cA) + rB - rA;
        var d = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.SubVV(cB, cA, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(rB, rA, b2Math_1.b2Vec2.s_t1), b2PrismaticJoint.InitVelocityConstraints_s_d);
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        // Compute motor Jacobian and effective mass.
        {
            // m_axis = b2Mul(qA, m_localXAxisA);
            b2Math_1.b2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
            // m_a1 = b2Cross(d + rA, m_axis);
            this.m_a1 = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), this.m_axis);
            // m_a2 = b2Cross(rB, m_axis);
            this.m_a2 = b2Math_1.b2Vec2.CrossVV(rB, this.m_axis);
            this.m_motorMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        // Prismatic constraint.
        {
            // m_perp = b2Mul(qA, m_localYAxisA);
            b2Math_1.b2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
            // m_s1 = b2Cross(d + rA, m_perp);
            this.m_s1 = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), this.m_perp);
            // m_s2 = b2Cross(rB, m_perp);
            this.m_s2 = b2Math_1.b2Vec2.CrossVV(rB, this.m_perp);
            // float32 k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
            this.m_K.ex.x = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
            // float32 k12 = iA * m_s1 + iB * m_s2;
            this.m_K.ex.y = iA * this.m_s1 + iB * this.m_s2;
            // float32 k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
            this.m_K.ex.z = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
            this.m_K.ey.x = this.m_K.ex.y;
            // float32 k22 = iA + iB;
            this.m_K.ey.y = iA + iB;
            if (this.m_K.ey.y === 0) {
                // For bodies with fixed rotation.
                this.m_K.ey.y = 1;
            }
            // float32 k23 = iA * m_a1 + iB * m_a2;
            this.m_K.ey.z = iA * this.m_a1 + iB * this.m_a2;
            this.m_K.ez.x = this.m_K.ex.z;
            this.m_K.ez.y = this.m_K.ey.z;
            // float32 k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
            this.m_K.ez.z = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            // m_K.ex.Set(k11, k12, k13);
            // m_K.ey.Set(k12, k22, k23);
            // m_K.ez.Set(k13, k23, k33);
        }
        // Compute motor and limit terms.
        if (this.m_enableLimit) {
            // float32 jointTranslation = b2Dot(m_axis, d);
            var jointTranslation = b2Math_1.b2Vec2.DotVV(this.m_axis, d);
            if (b2Math_1.b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2Settings_1.b2_linearSlop) {
                this.m_limitState = b2Joint_1.b2LimitState.e_equalLimits;
            }
            else if (jointTranslation <= this.m_lowerTranslation) {
                if (this.m_limitState !== b2Joint_1.b2LimitState.e_atLowerLimit) {
                    this.m_limitState = b2Joint_1.b2LimitState.e_atLowerLimit;
                    this.m_impulse.z = 0;
                }
            }
            else if (jointTranslation >= this.m_upperTranslation) {
                if (this.m_limitState !== b2Joint_1.b2LimitState.e_atUpperLimit) {
                    this.m_limitState = b2Joint_1.b2LimitState.e_atUpperLimit;
                    this.m_impulse.z = 0;
                }
            }
            else {
                this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
            this.m_impulse.z = 0;
        }
        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            // m_impulse *= data.step.dtRatio;
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
            var P = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.MulSV(this.m_impulse.x, this.m_perp, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.MulSV((this.m_motorImpulse + this.m_impulse.z), this.m_axis, b2Math_1.b2Vec2.s_t1), b2PrismaticJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
            var LA = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
            // float32 LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
            var LB = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2PrismaticJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        // Solve linear motor constraint.
        if (this.m_enableMotor && this.m_limitState !== b2Joint_1.b2LimitState.e_equalLimits) {
            // float32 Cdot = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            var Cdot = b2Math_1.b2Vec2.DotVV(this.m_axis, b2Math_1.b2Vec2.SubVV(vB, vA, b2Math_1.b2Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
            var impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = b2Math_1.b2Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            // b2Vec2 P = impulse * m_axis;
            var P = b2Math_1.b2Vec2.MulSV(impulse, this.m_axis, b2PrismaticJoint.SolveVelocityConstraints_s_P);
            var LA = impulse * this.m_a1;
            var LB = impulse * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // b2Vec2 Cdot1;
        // Cdot1.x = b2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
        var Cdot1_x = b2Math_1.b2Vec2.DotVV(this.m_perp, b2Math_1.b2Vec2.SubVV(vB, vA, b2Math_1.b2Vec2.s_t0)) + this.m_s2 * wB - this.m_s1 * wA;
        // Cdot1.y = wB - wA;
        var Cdot1_y = wB - wA;
        if (this.m_enableLimit && this.m_limitState !== b2Joint_1.b2LimitState.e_inactiveLimit) {
            // Solve prismatic and limit constraint in block form.
            // float32 Cdot2;
            // Cdot2 = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            var Cdot2 = b2Math_1.b2Vec2.DotVV(this.m_axis, b2Math_1.b2Vec2.SubVV(vB, vA, b2Math_1.b2Vec2.s_t0)) + this.m_a2 * wB - this.m_a1 * wA;
            // b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // b2Vec3 f1 = m_impulse;
            var f1 = b2PrismaticJoint.SolveVelocityConstraints_s_f1.Copy(this.m_impulse);
            // b2Vec3 df =  m_K.Solve33(-Cdot);
            var df3 = this.m_K.Solve33((-Cdot1_x), (-Cdot1_y), (-Cdot2), b2PrismaticJoint.SolveVelocityConstraints_s_df3);
            // m_impulse += df;
            this.m_impulse.SelfAdd(df3);
            if (this.m_limitState === b2Joint_1.b2LimitState.e_atLowerLimit) {
                this.m_impulse.z = b2Math_1.b2Max(this.m_impulse.z, 0);
            }
            else if (this.m_limitState === b2Joint_1.b2LimitState.e_atUpperLimit) {
                this.m_impulse.z = b2Math_1.b2Min(this.m_impulse.z, 0);
            }
            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            // b2Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * b2Vec2(m_K.ez.x, m_K.ez.y);
            var b_x = (-Cdot1_x) - (this.m_impulse.z - f1.z) * this.m_K.ez.x;
            var b_y = (-Cdot1_y) - (this.m_impulse.z - f1.z) * this.m_K.ez.y;
            // b2Vec2 f2r = m_K.Solve22(b) + b2Vec2(f1.x, f1.y);
            var f2r = this.m_K.Solve22(b_x, b_y, b2PrismaticJoint.SolveVelocityConstraints_s_f2r);
            f2r.x += f1.x;
            f2r.y += f1.y;
            // m_impulse.x = f2r.x;
            this.m_impulse.x = f2r.x;
            // m_impulse.y = f2r.y;
            this.m_impulse.y = f2r.y;
            // df = m_impulse - f1;
            df3.x = this.m_impulse.x - f1.x;
            df3.y = this.m_impulse.y - f1.y;
            df3.z = this.m_impulse.z - f1.z;
            // b2Vec2 P = df.x * m_perp + df.z * m_axis;
            var P = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.MulSV(df3.x, this.m_perp, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.MulSV(df3.z, this.m_axis, b2Math_1.b2Vec2.s_t1), b2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y + df.z * m_a1;
            var LA = df3.x * this.m_s1 + df3.y + df3.z * this.m_a1;
            // float32 LB = df.x * m_s2 + df.y + df.z * m_a2;
            var LB = df3.x * this.m_s2 + df3.y + df3.z * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            // b2Vec2 df = m_K.Solve22(-Cdot1);
            var df2 = this.m_K.Solve22((-Cdot1_x), (-Cdot1_y), b2PrismaticJoint.SolveVelocityConstraints_s_df2);
            this.m_impulse.x += df2.x;
            this.m_impulse.y += df2.y;
            // b2Vec2 P = df.x * m_perp;
            var P = b2Math_1.b2Vec2.MulSV(df2.x, this.m_perp, b2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y;
            var LA = df2.x * this.m_s1 + df2.y;
            // float32 LB = df.x * m_s2 + df.y;
            var LB = df2.x * this.m_s2 + df2.y;
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
    b2PrismaticJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 d = cB + rB - cA - rA;
        var d = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVV(cB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVV(cA, rA, b2Math_1.b2Vec2.s_t1), b2PrismaticJoint.SolvePositionConstraints_s_d);
        // b2Vec2 axis = b2Mul(qA, m_localXAxisA);
        var axis = b2Math_1.b2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
        // float32 a1 = b2Cross(d + rA, axis);
        var a1 = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), axis);
        // float32 a2 = b2Cross(rB, axis);
        var a2 = b2Math_1.b2Vec2.CrossVV(rB, axis);
        // b2Vec2 perp = b2Mul(qA, m_localYAxisA);
        var perp = b2Math_1.b2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
        // float32 s1 = b2Cross(d + rA, perp);
        var s1 = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.AddVV(d, rA, b2Math_1.b2Vec2.s_t0), perp);
        // float32 s2 = b2Cross(rB, perp);
        var s2 = b2Math_1.b2Vec2.CrossVV(rB, perp);
        // b2Vec3 impulse;
        var impulse = b2PrismaticJoint.SolvePositionConstraints_s_impulse;
        // b2Vec2 C1;
        // C1.x = b2Dot(perp, d);
        var C1_x = b2Math_1.b2Vec2.DotVV(perp, d);
        // C1.y = aB - aA - m_referenceAngle;
        var C1_y = aB - aA - this.m_referenceAngle;
        var linearError = b2Math_1.b2Abs(C1_x);
        var angularError = b2Math_1.b2Abs(C1_y);
        var active = false;
        var C2 = 0;
        if (this.m_enableLimit) {
            // float32 translation = b2Dot(axis, d);
            var translation = b2Math_1.b2Vec2.DotVV(axis, d);
            if (b2Math_1.b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2 * b2Settings_1.b2_linearSlop) {
                // Prevent large angular corrections
                C2 = b2Math_1.b2Clamp(translation, (-b2Settings_1.b2_maxLinearCorrection), b2Settings_1.b2_maxLinearCorrection);
                linearError = b2Math_1.b2Max(linearError, b2Math_1.b2Abs(translation));
                active = true;
            }
            else if (translation <= this.m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math_1.b2Clamp(translation - this.m_lowerTranslation + b2Settings_1.b2_linearSlop, (-b2Settings_1.b2_maxLinearCorrection), 0);
                linearError = b2Math_1.b2Max(linearError, this.m_lowerTranslation - translation);
                active = true;
            }
            else if (translation >= this.m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math_1.b2Clamp(translation - this.m_upperTranslation - b2Settings_1.b2_linearSlop, 0, b2Settings_1.b2_maxLinearCorrection);
                linearError = b2Math_1.b2Max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }
        if (active) {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            var k12 = iA * s1 + iB * s2;
            // float32 k13 = iA * s1 * a1 + iB * s2 * a2;
            var k13 = iA * s1 * a1 + iB * s2 * a2;
            // float32 k22 = iA + iB;
            var k22 = iA + iB;
            if (k22 === 0) {
                // For fixed rotation
                k22 = 1;
            }
            // float32 k23 = iA * a1 + iB * a2;
            var k23 = iA * a1 + iB * a2;
            // float32 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            var k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            // b2Mat33 K;
            var K = this.m_K3;
            // K.ex.Set(k11, k12, k13);
            K.ex.SetXYZ(k11, k12, k13);
            // K.ey.Set(k12, k22, k23);
            K.ey.SetXYZ(k12, k22, k23);
            // K.ez.Set(k13, k23, k33);
            K.ez.SetXYZ(k13, k23, k33);
            // b2Vec3 C;
            // C.x = C1.x;
            // C.y = C1.y;
            // C.z = C2;
            // impulse = K.Solve33(-C);
            impulse = K.Solve33((-C1_x), (-C1_y), (-C2), impulse);
        }
        else {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            var k12 = iA * s1 + iB * s2;
            // float32 k22 = iA + iB;
            var k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }
            // b2Mat22 K;
            var K2 = this.m_K2;
            // K.ex.Set(k11, k12);
            K2.ex.Set(k11, k12);
            // K.ey.Set(k12, k22);
            K2.ey.Set(k12, k22);
            // b2Vec2 impulse1 = K.Solve(-C1);
            var impulse1 = K2.Solve((-C1_x), (-C1_y), b2PrismaticJoint.SolvePositionConstraints_s_impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }
        // b2Vec2 P = impulse.x * perp + impulse.z * axis;
        var P = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.MulSV(impulse.x, perp, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.MulSV(impulse.z, axis, b2Math_1.b2Vec2.s_t1), b2PrismaticJoint.SolvePositionConstraints_s_P);
        // float32 LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        var LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        // float32 LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        var LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        // cA -= mA * P;
        cA.SelfMulSub(mA, P);
        aA -= iA * LA;
        // cB += mB * P;
        cB.SelfMulAdd(mB, P);
        aB += iB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError <= b2Settings_1.b2_linearSlop && angularError <= b2Settings_1.b2_angularSlop;
    };
    b2PrismaticJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2PrismaticJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2PrismaticJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
        out.x = inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x);
        out.y = inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y);
        return out;
    };
    b2PrismaticJoint.prototype.GetReactionTorque = function (inv_dt) {
        return inv_dt * this.m_impulse.y;
    };
    b2PrismaticJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2PrismaticJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2PrismaticJoint.prototype.GetLocalAxisA = function () { return this.m_localXAxisA; };
    b2PrismaticJoint.prototype.GetReferenceAngle = function () { return this.m_referenceAngle; };
    b2PrismaticJoint.prototype.GetJointTranslation = function () {
        // b2Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
        var pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, b2PrismaticJoint.GetJointTranslation_s_pA);
        // b2Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
        var pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, b2PrismaticJoint.GetJointTranslation_s_pB);
        // b2Vec2 d = pB - pA;
        var d = b2Math_1.b2Vec2.SubVV(pB, pA, b2PrismaticJoint.GetJointTranslation_s_d);
        // b2Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);
        var axis = this.m_bodyA.GetWorldVector(this.m_localXAxisA, b2PrismaticJoint.GetJointTranslation_s_axis);
        // float32 translation = b2Dot(d, axis);
        var translation = b2Math_1.b2Vec2.DotVV(d, axis);
        return translation;
    };
    b2PrismaticJoint.prototype.GetJointSpeed = function () {
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
        var axis = bA.GetWorldVector(this.m_localXAxisA, this.m_axis);
        var vA = bA.m_linearVelocity;
        var vB = bB.m_linearVelocity;
        var wA = bA.m_angularVelocity;
        var wB = bB.m_angularVelocity;
        // float32 speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
        var speed = b2Math_1.b2Vec2.DotVV(d, b2Math_1.b2Vec2.CrossSV(wA, axis, b2Math_1.b2Vec2.s_t0)) +
            b2Math_1.b2Vec2.DotVV(axis, b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, rA, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0));
        return speed;
    };
    b2PrismaticJoint.prototype.IsLimitEnabled = function () {
        return this.m_enableLimit;
    };
    b2PrismaticJoint.prototype.EnableLimit = function (flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    };
    b2PrismaticJoint.prototype.GetLowerLimit = function () {
        return this.m_lowerTranslation;
    };
    b2PrismaticJoint.prototype.GetUpperLimit = function () {
        return this.m_upperTranslation;
    };
    b2PrismaticJoint.prototype.SetLimits = function (lower, upper) {
        if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_impulse.z = 0;
        }
    };
    b2PrismaticJoint.prototype.IsMotorEnabled = function () {
        return this.m_enableMotor;
    };
    b2PrismaticJoint.prototype.EnableMotor = function (flag) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_enableMotor = flag;
    };
    b2PrismaticJoint.prototype.SetMotorSpeed = function (speed) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    };
    b2PrismaticJoint.prototype.GetMotorSpeed = function () {
        return this.m_motorSpeed;
    };
    b2PrismaticJoint.prototype.SetMaxMotorForce = function (force) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_maxMotorForce = force;
    };
    b2PrismaticJoint.prototype.GetMaxMotorForce = function () { return this.m_maxMotorForce; };
    b2PrismaticJoint.prototype.GetMotorForce = function (inv_dt) {
        return inv_dt * this.m_motorImpulse;
    };
    b2PrismaticJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2PrismaticJointDef = new b2PrismaticJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.localAxisA.Set(%.15f, %.15f);\n", this.m_localXAxisA.x, this.m_localXAxisA.y);
        log("  jd.referenceAngle = %.15f;\n", this.m_referenceAngle);
        log("  jd.enableLimit = %s;\n", (this.m_enableLimit) ? ("true") : ("false"));
        log("  jd.lowerTranslation = %.15f;\n", this.m_lowerTranslation);
        log("  jd.upperTranslation = %.15f;\n", this.m_upperTranslation);
        log("  jd.enableMotor = %s;\n", (this.m_enableMotor) ? ("true") : ("false"));
        log("  jd.motorSpeed = %.15f;\n", this.m_motorSpeed);
        log("  jd.maxMotorForce = %.15f;\n", this.m_maxMotorForce);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2PrismaticJoint.InitVelocityConstraints_s_d = new b2Math_1.b2Vec2();
    b2PrismaticJoint.InitVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolveVelocityConstraints_s_f2r = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolveVelocityConstraints_s_f1 = new b2Math_1.b2Vec3();
    b2PrismaticJoint.SolveVelocityConstraints_s_df3 = new b2Math_1.b2Vec3();
    b2PrismaticJoint.SolveVelocityConstraints_s_df2 = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolvePositionConstraints_s_d = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolvePositionConstraints_s_impulse = new b2Math_1.b2Vec3();
    b2PrismaticJoint.SolvePositionConstraints_s_impulse1 = new b2Math_1.b2Vec2();
    b2PrismaticJoint.SolvePositionConstraints_s_P = new b2Math_1.b2Vec2();
    b2PrismaticJoint.GetJointTranslation_s_pA = new b2Math_1.b2Vec2();
    b2PrismaticJoint.GetJointTranslation_s_pB = new b2Math_1.b2Vec2();
    b2PrismaticJoint.GetJointTranslation_s_d = new b2Math_1.b2Vec2();
    b2PrismaticJoint.GetJointTranslation_s_axis = new b2Math_1.b2Vec2();
    return b2PrismaticJoint;
}(b2Joint_1.b2Joint));
exports.b2PrismaticJoint = b2PrismaticJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQcmlzbWF0aWNKb2ludC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0pvaW50cy9iMlByaXNtYXRpY0pvaW50LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBRUYsc0RBQXlHO0FBQ3pHLDhDQUFnSDtBQUVoSCxxQ0FBd0Y7QUF5QnhGLGdFQUFnRTtBQUNoRSx1RUFBdUU7QUFDdkUsb0VBQW9FO0FBQ3BFLHNFQUFzRTtBQUN0RSxxRUFBcUU7QUFDckUsa0VBQWtFO0FBQ2xFO0lBQXlDLHVDQUFVO0lBcUJqRDtRQUFBLFlBQ0Usa0JBQU0scUJBQVcsQ0FBQyxnQkFBZ0IsQ0FBQyxTQUNwQztRQXRCZSxrQkFBWSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFFcEMsa0JBQVksR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBRXBDLGdCQUFVLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRS9DLG9CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBRTNCLGlCQUFXLEdBQUcsS0FBSyxDQUFDO1FBRXBCLHNCQUFnQixHQUFXLENBQUMsQ0FBQztRQUU3QixzQkFBZ0IsR0FBVyxDQUFDLENBQUM7UUFFN0IsaUJBQVcsR0FBRyxLQUFLLENBQUM7UUFFcEIsbUJBQWEsR0FBVyxDQUFDLENBQUM7UUFFMUIsZ0JBQVUsR0FBVyxDQUFDLENBQUM7O0lBSTlCLENBQUM7SUFFTSx3Q0FBVSxHQUFqQixVQUFrQixFQUFVLEVBQUUsRUFBVSxFQUFFLE1BQWMsRUFBRSxJQUFZO1FBQ3BFLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxDQUFDO1FBQ2hCLElBQUksQ0FBQyxLQUFLLEdBQUcsRUFBRSxDQUFDO1FBQ2hCLElBQUksQ0FBQyxLQUFLLENBQUMsYUFBYSxDQUFDLE1BQU0sRUFBRSxJQUFJLENBQUMs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