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
/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
var b2RevoluteJointDef = /** @class */ (function (_super) {
    __extends(b2RevoluteJointDef, _super);
    function b2RevoluteJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_revoluteJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2(0, 0);
        _this.localAnchorB = new b2Math_1.b2Vec2(0, 0);
        _this.referenceAngle = 0;
        _this.enableLimit = false;
        _this.lowerAngle = 0;
        _this.upperAngle = 0;
        _this.enableMotor = false;
        _this.motorSpeed = 0;
        _this.maxMotorTorque = 0;
        return _this;
    }
    b2RevoluteJointDef.prototype.Initialize = function (bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    };
    return b2RevoluteJointDef;
}(b2Joint_1.b2JointDef));
exports.b2RevoluteJointDef = b2RevoluteJointDef;
var b2RevoluteJoint = /** @class */ (function (_super) {
    __extends(b2RevoluteJoint, _super);
    function b2RevoluteJoint(def) {
        var _this = _super.call(this, def) || this;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_impulse = new b2Math_1.b2Vec3();
        _this.m_motorImpulse = 0;
        _this.m_enableMotor = false;
        _this.m_maxMotorTorque = 0;
        _this.m_motorSpeed = 0;
        _this.m_enableLimit = false;
        _this.m_referenceAngle = 0;
        _this.m_lowerAngle = 0;
        _this.m_upperAngle = 0;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_rA = new b2Math_1.b2Vec2();
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_localCenterA = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_invMassA = 0;
        _this.m_invMassB = 0;
        _this.m_invIA = 0;
        _this.m_invIB = 0;
        _this.m_mass = new b2Math_1.b2Mat33(); // effective mass for point-to-point constraint.
        _this.m_motorMass = 0; // effective mass for motor/limit angular constraint.
        _this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_K = new b2Math_1.b2Mat22();
        _this.m_localAnchorA.Copy(b2Settings_1.b2Maybe(def.localAnchorA, b2Math_1.b2Vec2.ZERO));
        _this.m_localAnchorB.Copy(b2Settings_1.b2Maybe(def.localAnchorB, b2Math_1.b2Vec2.ZERO));
        _this.m_referenceAngle = b2Settings_1.b2Maybe(def.referenceAngle, 0);
        _this.m_impulse.SetZero();
        _this.m_motorImpulse = 0;
        _this.m_lowerAngle = b2Settings_1.b2Maybe(def.lowerAngle, 0);
        _this.m_upperAngle = b2Settings_1.b2Maybe(def.upperAngle, 0);
        _this.m_maxMotorTorque = b2Settings_1.b2Maybe(def.maxMotorTorque, 0);
        _this.m_motorSpeed = b2Settings_1.b2Maybe(def.motorSpeed, 0);
        _this.m_enableLimit = b2Settings_1.b2Maybe(def.enableLimit, false);
        _this.m_enableMotor = b2Settings_1.b2Maybe(def.enableMotor, false);
        _this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
        return _this;
    }
    b2RevoluteJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // b2Rot qA(aA), qB(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var fixedRotation = (iA + iB === 0);
        this.m_mass.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        this.m_mass.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        this.m_mass.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        this.m_mass.ex.y = this.m_mass.ey.x;
        this.m_mass.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        this.m_mass.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        this.m_mass.ex.z = this.m_mass.ez.x;
        this.m_mass.ey.z = this.m_mass.ez.y;
        this.m_mass.ez.z = iA + iB;
        this.m_motorMass = iA + iB;
        if (this.m_motorMass > 0) {
            this.m_motorMass = 1 / this.m_motorMass;
        }
        if (!this.m_enableMotor || fixedRotation) {
            this.m_motorImpulse = 0;
        }
        if (this.m_enableLimit && !fixedRotation) {
            var jointAngle = aB - aA - this.m_referenceAngle;
            if (b2Math_1.b2Abs(this.m_upperAngle - this.m_lowerAngle) < 2 * b2Settings_1.b2_angularSlop) {
                this.m_limitState = b2Joint_1.b2LimitState.e_equalLimits;
            }
            else if (jointAngle <= this.m_lowerAngle) {
                if (this.m_limitState !== b2Joint_1.b2LimitState.e_atLowerLimit) {
                    this.m_impulse.z = 0;
                }
                this.m_limitState = b2Joint_1.b2LimitState.e_atLowerLimit;
            }
            else if (jointAngle >= this.m_upperAngle) {
                if (this.m_limitState !== b2Joint_1.b2LimitState.e_atUpperLimit) {
                    this.m_impulse.z = 0;
                }
                this.m_limitState = b2Joint_1.b2LimitState.e_atUpperLimit;
            }
            else {
                this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = b2Joint_1.b2LimitState.e_inactiveLimit;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // b2Vec2 P(m_impulse.x, m_impulse.y);
            var P = b2RevoluteJoint.InitVelocityConstraints_s_P.Set(this.m_impulse.x, this.m_impulse.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.b2Vec2.CrossVV(this.m_rA, P) + this.m_motorImpulse + this.m_impulse.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.b2Vec2.CrossVV(this.m_rB, P) + this.m_motorImpulse + this.m_impulse.z);
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
    b2RevoluteJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var fixedRotation = (iA + iB === 0);
        // Solve motor constraint.
        if (this.m_enableMotor && this.m_limitState !== b2Joint_1.b2LimitState.e_equalLimits && !fixedRotation) {
            var Cdot = wB - wA - this.m_motorSpeed;
            var impulse = -this.m_motorMass * Cdot;
            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Math_1.b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve limit constraint.
        if (this.m_enableLimit && this.m_limitState !== b2Joint_1.b2LimitState.e_inactiveLimit && !fixedRotation) {
            // b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            var Cdot1 = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Math_1.b2Vec2.s_t1), b2RevoluteJoint.SolveVelocityConstraints_s_Cdot1);
            var Cdot2 = wB - wA;
            // b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // b2Vec3 impulse = -this.m_mass.Solve33(Cdot);
            var impulse_v3 = this.m_mass.Solve33(Cdot1.x, Cdot1.y, Cdot2, b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v3).SelfNeg();
            if (this.m_limitState === b2Joint_1.b2LimitState.e_equalLimits) {
                this.m_impulse.SelfAdd(impulse_v3);
            }
            else if (this.m_limitState === b2Joint_1.b2LimitState.e_atLowerLimit) {
                var newImpulse = this.m_impulse.z + impulse_v3.z;
                if (newImpulse < 0) {
                    // b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
                    var rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x;
                    var rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y;
                    var reduced_v2 = this.m_mass.Solve22(rhs_x, rhs_y, b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2);
                    impulse_v3.x = reduced_v2.x;
                    impulse_v3.y = reduced_v2.y;
                    impulse_v3.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced_v2.x;
                    this.m_impulse.y += reduced_v2.y;
                    this.m_impulse.z = 0;
                }
                else {
                    this.m_impulse.SelfAdd(impulse_v3);
                }
            }
            else if (this.m_limitState === b2Joint_1.b2LimitState.e_atUpperLimit) {
                var newImpulse = this.m_impulse.z + impulse_v3.z;
                if (newImpulse > 0) {
                    // b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
                    var rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x;
                    var rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y;
                    var reduced_v2 = this.m_mass.Solve22(rhs_x, rhs_y, b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2);
                    impulse_v3.x = reduced_v2.x;
                    impulse_v3.y = reduced_v2.y;
                    impulse_v3.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced_v2.x;
                    this.m_impulse.y += reduced_v2.y;
                    this.m_impulse.z = 0;
                }
                else {
                    this.m_impulse.SelfAdd(impulse_v3);
                }
            }
            // b2Vec2 P(impulse.x, impulse.y);
            var P = b2RevoluteJoint.SolveVelocityConstraints_s_P.Set(impulse_v3.x, impulse_v3.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.b2Vec2.CrossVV(this.m_rA, P) + impulse_v3.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.b2Vec2.CrossVV(this.m_rB, P) + impulse_v3.z);
        }
        else {
            // Solve point-to-point constraint
            // b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            var Cdot_v2 = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Math_1.b2Vec2.s_t1), b2RevoluteJoint.SolveVelocityConstraints_s_Cdot_v2);
            // b2Vec2 impulse = m_mass.Solve22(-Cdot);
            var impulse_v2 = this.m_mass.Solve22(-Cdot_v2.x, -Cdot_v2.y, b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v2);
            this.m_impulse.x += impulse_v2.x;
            this.m_impulse.y += impulse_v2.y;
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulse_v2);
            wA -= iA * b2Math_1.b2Vec2.CrossVV(this.m_rA, impulse_v2);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulse_v2);
            wB += iB * b2Math_1.b2Vec2.CrossVV(this.m_rB, impulse_v2);
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2RevoluteJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        // b2Rot qA(aA), qB(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        var angularError = 0;
        var positionError = 0;
        var fixedRotation = (this.m_invIA + this.m_invIB === 0);
        // Solve angular limit constraint.
        if (this.m_enableLimit && this.m_limitState !== b2Joint_1.b2LimitState.e_inactiveLimit && !fixedRotation) {
            var angle = aB - aA - this.m_referenceAngle;
            var limitImpulse = 0;
            if (this.m_limitState === b2Joint_1.b2LimitState.e_equalLimits) {
                // Prevent large angular corrections
                var C = b2Math_1.b2Clamp(angle - this.m_lowerAngle, -b2Settings_1.b2_maxAngularCorrection, b2Settings_1.b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
                angularError = b2Math_1.b2Abs(C);
            }
            else if (this.m_limitState === b2Joint_1.b2LimitState.e_atLowerLimit) {
                var C = angle - this.m_lowerAngle;
                angularError = -C;
                // Prevent large angular corrections and allow some slop.
                C = b2Math_1.b2Clamp(C + b2Settings_1.b2_angularSlop, -b2Settings_1.b2_maxAngularCorrection, 0);
                limitImpulse = -this.m_motorMass * C;
            }
            else if (this.m_limitState === b2Joint_1.b2LimitState.e_atUpperLimit) {
                var C = angle - this.m_upperAngle;
                angularError = C;
                // Prevent large angular corrections and allow some slop.
                C = b2Math_1.b2Clamp(C - b2Settings_1.b2_angularSlop, 0, b2Settings_1.b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
            }
            aA -= this.m_invIA * limitImpulse;
            aB += this.m_invIB * limitImpulse;
        }
        // Solve point-to-point constraint.
        {
            qA.SetAngle(aA);
            qB.SetAngle(aB);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
            var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
            var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
            // b2Vec2 C = cB + rB - cA - rA;
            var C_v2 = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVV(cB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVV(cA, rA, b2Math_1.b2Vec2.s_t1), b2RevoluteJoint.SolvePositionConstraints_s_C_v2);
            // positionError = C.Length();
            positionError = C_v2.Length();
            var mA = this.m_invMassA, mB = this.m_invMassB;
            var iA = this.m_invIA, iB = this.m_invIB;
            var K = this.m_K;
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
            // b2Vec2 impulse = -K.Solve(C);
            var impulse = K.Solve(C_v2.x, C_v2.y, b2RevoluteJoint.SolvePositionConstraints_s_impulse).SelfNeg();
            // cA -= mA * impulse;
            cA.SelfMulSub(mA, impulse);
            aA -= iA * b2Math_1.b2Vec2.CrossVV(rA, impulse);
            // cB += mB * impulse;
            cB.SelfMulAdd(mB, impulse);
            aB += iB * b2Math_1.b2Vec2.CrossVV(rB, impulse);
        }
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return positionError <= b2Settings_1.b2_linearSlop && angularError <= b2Settings_1.b2_angularSlop;
    };
    b2RevoluteJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2RevoluteJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2RevoluteJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // b2Vec2 P(this.m_impulse.x, this.m_impulse.y);
        // return inv_dt * P;
        out.x = inv_dt * this.m_impulse.x;
        out.y = inv_dt * this.m_impulse.y;
        return out;
    };
    b2RevoluteJoint.prototype.GetReactionTorque = function (inv_dt) {
        return inv_dt * this.m_impulse.z;
    };
    b2RevoluteJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2RevoluteJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2RevoluteJoint.prototype.GetReferenceAngle = function () { return this.m_referenceAngle; };
    b2RevoluteJoint.prototype.GetJointAngle = function () {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_sweep.a - bA->this.m_sweep.a - this.m_referenceAngle;
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
    };
    b2RevoluteJoint.prototype.GetJointSpeed = function () {
        // b2Body* bA = this.m_bodyA;
        // b2Body* bB = this.m_bodyB;
        // return bB->this.m_angularVelocity - bA->this.m_angularVelocity;
        return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
    };
    b2RevoluteJoint.prototype.IsMotorEnabled = function () {
        return this.m_enableMotor;
    };
    b2RevoluteJoint.prototype.EnableMotor = function (flag) {
        if (this.m_enableMotor !== flag) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
    };
    b2RevoluteJoint.prototype.GetMotorTorque = function (inv_dt) {
        return inv_dt * this.m_motorImpulse;
    };
    b2RevoluteJoint.prototype.GetMotorSpeed = function () {
        return this.m_motorSpeed;
    };
    b2RevoluteJoint.prototype.SetMaxMotorTorque = function (torque) {
        this.m_maxMotorTorque = torque;
    };
    b2RevoluteJoint.prototype.GetMaxMotorTorque = function () { return this.m_maxMotorTorque; };
    b2RevoluteJoint.prototype.IsLimitEnabled = function () {
        return this.m_enableLimit;
    };
    b2RevoluteJoint.prototype.EnableLimit = function (flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    };
    b2RevoluteJoint.prototype.GetLowerLimit = function () {
        return this.m_lowerAngle;
    };
    b2RevoluteJoint.prototype.GetUpperLimit = function () {
        return this.m_upperAngle;
    };
    b2RevoluteJoint.prototype.SetLimits = function (lower, upper) {
        if (lower !== this.m_lowerAngle || upper !== this.m_upperAngle) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_impulse.z = 0;
            this.m_lowerAngle = lower;
            this.m_upperAngle = upper;
        }
    };
    b2RevoluteJoint.prototype.SetMotorSpeed = function (speed) {
        if (this.m_motorSpeed !== speed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
    };
    b2RevoluteJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2RevoluteJointDef = new b2RevoluteJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.referenceAngle = %.15f;\n", this.m_referenceAngle);
        log("  jd.enableLimit = %s;\n", (this.m_enableLimit) ? ("true") : ("false"));
        log("  jd.lowerAngle = %.15f;\n", this.m_lowerAngle);
        log("  jd.upperAngle = %.15f;\n", this.m_upperAngle);
        log("  jd.enableMotor = %s;\n", (this.m_enableMotor) ? ("true") : ("false"));
        log("  jd.motorSpeed = %.15f;\n", this.m_motorSpeed);
        log("  jd.maxMotorTorque = %.15f;\n", this.m_maxMotorTorque);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2RevoluteJoint.InitVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolveVelocityConstraints_s_Cdot1 = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v3 = new b2Math_1.b2Vec3();
    b2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2 = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolveVelocityConstraints_s_impulse_v2 = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolvePositionConstraints_s_C_v2 = new b2Math_1.b2Vec2();
    b2RevoluteJoint.SolvePositionConstraints_s_impulse = new b2Math_1.b2Vec2();
    return b2RevoluteJoint;
}(b2Joint_1.b2Joint));
exports.b2RevoluteJoint = b2RevoluteJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJSZXZvbHV0ZUpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyUmV2b2x1dGVKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLHNEQUEwRztBQUMxRyw4Q0FBa0c7QUFFbEcscUNBQXdGO0FBdUJ4Rix3REFBd0Q7QUFDeEQsNERBQTREO0FBQzVELDhEQUE4RDtBQUM5RCx5REFBeUQ7QUFDekQsNkRBQTZEO0FBQzdELHlDQUF5QztBQUN6QywrREFBK0Q7QUFDL0QsMkNBQTJDO0FBQzNDLDJEQUEyRDtBQUMzRCxtRUFBbUU7QUFDbkUsaUNBQWlDO0FBQ2pDO0lBQXdDLHNDQUFVO0lBbUJoRDtRQUFBLFlBQ0Usa0JBQU0scUJBQVcsQ0FBQyxlQUFlLENBQUMsU0FDbkM7UUFwQmUsa0JBQVksR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFeEMsa0JBQVksR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFakQsb0JBQWMsR0FBVyxDQUFDLENBQUM7UUFFM0IsaUJBQVcsR0FBRyxLQUFLLENBQUM7UUFFcEIsZ0JBQVUsR0FBVyxDQUFDLENBQUM7UUFFdkIsZ0JBQVUsR0FBVyxDQUFDLENBQUM7UUFFdkIsaUJBQVcsR0FBRyxLQUFLLENBQUM7UUFFcEIsZ0JBQVUsR0FBVyxDQUFDLENBQUM7UUFFdkIsb0JBQWMsR0FBVyxDQUFDLENBQUM7O0lBSWxDLENBQUM7SUFFTSx1Q0FBVSxHQUFqQixVQUFrQixFQUFVLEVBQUUsRUFBVSxFQUFFLE1BQWM7UUFDdEQsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssQ0FB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LEdBQUcsSUFBSSxDQUFDLE1BQU0sQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3BDLElBQUksQ0FBQyxNQUFNLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxFQUFFLEdBQUcsRUFBRSxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLEVBQUUsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDN0YsSUFBSSxDQUFDLE1BQU0sQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQyxHQUFHLEVBQUUsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDdkQsSUFBSSxDQUFDLE1BQU0sQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxNQUFNLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNwQyxJQUFJLENBQUMsTUFBTSxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsSUFBSSxDQUFDLE1BQU0sQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ3BDLElBQUksQ0FBQyxNQUFNLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxFQUFFLEdBQUcsRUFBRSxDQUFDO1FBRTNCLElBQUksQ0FBQyxXQUFXLEdBQUcsRUFBRSxHQUFHLEVBQUUsQ0FBQztRQUMzQixJQUFJLElBQUksQ0FBQyxXQUFXLEdBQUcsQ0FBQyxFQUFFO1lBQ3hCLElBQUksQ0FBQyxXQUFXLEdBQUcsQ0FBQyxHQUFHLElBQUksQ0FBQyxXQUFXLENBQUM7U0FDekM7UUFFRCxJQUFJLENBQUMsSUFBSSxDQUFDLGFBQWEsSUFBSSxhQUFhLEVBQUU7WUFDeEMsSUFBSSxDQUFDLGNBQWMsR0FBRyxDQUFDLENBQUM7U0FDekI7UUFFRCxJQUFJLElBQUksQ0FBQyxhQUFhLElBQUksQ0FBQyxhQUFhLEVBQUU7WUFD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