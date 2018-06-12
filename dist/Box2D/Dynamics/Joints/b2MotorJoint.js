"use strict";
/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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
// DEBUG: import { b2IsValid } from "../../Common/b2Math";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
var b2MotorJointDef = /** @class */ (function (_super) {
    __extends(b2MotorJointDef, _super);
    function b2MotorJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_motorJoint) || this;
        _this.linearOffset = new b2Math_1.b2Vec2(0, 0);
        _this.angularOffset = 0;
        _this.maxForce = 1;
        _this.maxTorque = 1;
        _this.correctionFactor = 0.3;
        return _this;
    }
    b2MotorJointDef.prototype.Initialize = function (bA, bB) {
        this.bodyA = bA;
        this.bodyB = bB;
        // b2Vec2 xB = bodyB->GetPosition();
        // linearOffset = bodyA->GetLocalPoint(xB);
        this.bodyA.GetLocalPoint(this.bodyB.GetPosition(), this.linearOffset);
        var angleA = this.bodyA.GetAngle();
        var angleB = this.bodyB.GetAngle();
        this.angularOffset = angleB - angleA;
    };
    return b2MotorJointDef;
}(b2Joint_1.b2JointDef));
exports.b2MotorJointDef = b2MotorJointDef;
var b2MotorJoint = /** @class */ (function (_super) {
    __extends(b2MotorJoint, _super);
    function b2MotorJoint(def) {
        var _this = _super.call(this, def) || this;
        // Solver shared
        _this.m_linearOffset = new b2Math_1.b2Vec2();
        _this.m_angularOffset = 0;
        _this.m_linearImpulse = new b2Math_1.b2Vec2();
        _this.m_angularImpulse = 0;
        _this.m_maxForce = 0;
        _this.m_maxTorque = 0;
        _this.m_correctionFactor = 0.3;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_rA = new b2Math_1.b2Vec2();
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_localCenterA = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_linearError = new b2Math_1.b2Vec2();
        _this.m_angularError = 0;
        _this.m_invMassA = 0;
        _this.m_invMassB = 0;
        _this.m_invIA = 0;
        _this.m_invIB = 0;
        _this.m_linearMass = new b2Math_1.b2Mat22();
        _this.m_angularMass = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_K = new b2Math_1.b2Mat22();
        _this.m_linearOffset.Copy(b2Settings_1.b2Maybe(def.linearOffset, b2Math_1.b2Vec2.ZERO));
        _this.m_linearImpulse.SetZero();
        _this.m_maxForce = b2Settings_1.b2Maybe(def.maxForce, 0);
        _this.m_maxTorque = b2Settings_1.b2Maybe(def.maxTorque, 0);
        _this.m_correctionFactor = b2Settings_1.b2Maybe(def.correctionFactor, 0.3);
        return _this;
    }
    b2MotorJoint.prototype.GetAnchorA = function (out) {
        var pos = this.m_bodyA.GetPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    };
    b2MotorJoint.prototype.GetAnchorB = function (out) {
        var pos = this.m_bodyB.GetPosition();
        out.x = pos.x;
        out.y = pos.y;
        return out;
    };
    b2MotorJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // return inv_dt * m_linearImpulse;
        return b2Math_1.b2Vec2.MulSV(inv_dt, this.m_linearImpulse, out);
    };
    b2MotorJoint.prototype.GetReactionTorque = function (inv_dt) {
        return inv_dt * this.m_angularImpulse;
    };
    b2MotorJoint.prototype.SetLinearOffset = function (linearOffset) {
        if (!b2Math_1.b2Vec2.IsEqualToV(linearOffset, this.m_linearOffset)) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_linearOffset.Copy(linearOffset);
        }
    };
    b2MotorJoint.prototype.GetLinearOffset = function () {
        return this.m_linearOffset;
    };
    b2MotorJoint.prototype.SetAngularOffset = function (angularOffset) {
        if (angularOffset !== this.m_angularOffset) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_angularOffset = angularOffset;
        }
    };
    b2MotorJoint.prototype.GetAngularOffset = function () {
        return this.m_angularOffset;
    };
    b2MotorJoint.prototype.SetMaxForce = function (force) {
        // DEBUG: b2Assert(b2IsValid(force) && force >= 0);
        this.m_maxForce = force;
    };
    b2MotorJoint.prototype.GetMaxForce = function () {
        return this.m_maxForce;
    };
    b2MotorJoint.prototype.SetMaxTorque = function (torque) {
        // DEBUG: b2Assert(b2IsValid(torque) && torque >= 0);
        this.m_maxTorque = torque;
    };
    b2MotorJoint.prototype.GetMaxTorque = function () {
        return this.m_maxTorque;
    };
    b2MotorJoint.prototype.InitVelocityConstraints = function (data) {
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
        // Compute the effective mass matrix.
        // this.m_rA = b2Mul(qA, m_linearOffset - this.m_localCenterA);
        var rA = b2Math_1.b2Rot.MulRV(qA, b2Math_1.b2Vec2.SubVV(this.m_linearOffset, this.m_localCenterA, b2Math_1.b2Vec2.s_t0), this.m_rA);
        // this.m_rB = b2Mul(qB, -this.m_localCenterB);
        var rB = b2Math_1.b2Rot.MulRV(qB, b2Math_1.b2Vec2.NegV(this.m_localCenterB, b2Math_1.b2Vec2.s_t0), this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        // Upper 2 by 2 of K for point to point
        var K = this.m_K;
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
        // this.m_linearMass = K.GetInverse();
        K.GetInverse(this.m_linearMass);
        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }
        // this.m_linearError = cB + rB - cA - rA;
        b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVV(cB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVV(cA, rA, b2Math_1.b2Vec2.s_t1), this.m_linearError);
        this.m_angularError = aB - aA - this.m_angularOffset;
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            // this.m_linearImpulse *= data.step.dtRatio;
            this.m_linearImpulse.SelfMul(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;
            // b2Vec2 P(this.m_linearImpulse.x, this.m_linearImpulse.y);
            var P = this.m_linearImpulse;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.b2Vec2.CrossVV(rA, P) + this.m_angularImpulse);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.b2Vec2.CrossVV(rB, P) + this.m_angularImpulse);
        }
        else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA; // vA is a reference
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB; // vB is a reference
        data.velocities[this.m_indexB].w = wB;
    };
    b2MotorJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var h = data.step.dt;
        var inv_h = data.step.inv_dt;
        // Solve angular friction
        {
            var Cdot = wB - wA + inv_h * this.m_correctionFactor * this.m_angularError;
            var impulse = -this.m_angularMass * Cdot;
            var oldImpulse = this.m_angularImpulse;
            var maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Math_1.b2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve linear friction
        {
            var rA = this.m_rA;
            var rB = this.m_rB;
            // b2Vec2 Cdot = vB + b2Vec2.CrossSV(wB, rB) - vA - b2Vec2.CrossSV(wA, rA) + inv_h * this.m_correctionFactor * this.m_linearError;
            var Cdot_v2 = b2Math_1.b2Vec2.AddVV(b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVV(vB, b2Math_1.b2Vec2.CrossSV(wB, rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVV(vA, b2Math_1.b2Vec2.CrossSV(wA, rA, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t2), b2Math_1.b2Vec2.MulSV(inv_h * this.m_correctionFactor, this.m_linearError, b2Math_1.b2Vec2.s_t3), b2MotorJoint.SolveVelocityConstraints_s_Cdot_v2);
            // b2Vec2 impulse = -b2Mul(this.m_linearMass, Cdot);
            var impulse_v2 = b2Math_1.b2Mat22.MulMV(this.m_linearMass, Cdot_v2, b2MotorJoint.SolveVelocityConstraints_s_impulse_v2).SelfNeg();
            // b2Vec2 oldImpulse = this.m_linearImpulse;
            var oldImpulse_v2 = b2MotorJoint.SolveVelocityConstraints_s_oldImpulse_v2.Copy(this.m_linearImpulse);
            // this.m_linearImpulse += impulse;
            this.m_linearImpulse.SelfAdd(impulse_v2);
            var maxImpulse = h * this.m_maxForce;
            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                // this.m_linearImpulse *= maxImpulse;
                this.m_linearImpulse.SelfMul(maxImpulse);
            }
            // impulse = this.m_linearImpulse - oldImpulse;
            b2Math_1.b2Vec2.SubVV(this.m_linearImpulse, oldImpulse_v2, impulse_v2);
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulse_v2);
            // wA -= iA * b2Vec2.CrossVV(rA, impulse);
            wA -= iA * b2Math_1.b2Vec2.CrossVV(rA, impulse_v2);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulse_v2);
            // wB += iB * b2Vec2.CrossVV(rB, impulse);
            wB += iB * b2Math_1.b2Vec2.CrossVV(rB, impulse_v2);
        }
        // data.velocities[this.m_indexA].v = vA; // vA is a reference
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB; // vB is a reference
        data.velocities[this.m_indexB].w = wB;
    };
    b2MotorJoint.prototype.SolvePositionConstraints = function (data) {
        return true;
    };
    b2MotorJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2MotorJointDef = new b2MotorJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.linearOffset.Set(%.15f, %.15f);\n", this.m_linearOffset.x, this.m_linearOffset.y);
        log("  jd.angularOffset = %.15f;\n", this.m_angularOffset);
        log("  jd.maxForce = %.15f;\n", this.m_maxForce);
        log("  jd.maxTorque = %.15f;\n", this.m_maxTorque);
        log("  jd.correctionFactor = %.15f;\n", this.m_correctionFactor);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2MotorJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Math_1.b2Vec2();
    b2MotorJoint.SolveVelocityConstraints_s_impulse_v2 = new b2Math_1.b2Vec2();
    b2MotorJoint.SolveVelocityConstraints_s_oldImpulse_v2 = new b2Math_1.b2Vec2();
    return b2MotorJoint;
}(b2Joint_1.b2Joint));
exports.b2MotorJoint = b2MotorJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJNb3RvckpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyTW90b3JKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLDZEQUE2RDtBQUM3RCwwREFBMEQ7QUFDMUQsc0RBQWtEO0FBQ2xELDhDQUEwRTtBQUUxRSxxQ0FBMEU7QUE4QjFFO0lBQXFDLG1DQUFVO0lBVzdDO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLFlBQVksQ0FBQyxTQUNoQztRQVplLGtCQUFZLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRWpELG1CQUFhLEdBQVcsQ0FBQyxDQUFDO1FBRTFCLGNBQVEsR0FBVyxDQUFDLENBQUM7UUFFckIsZUFBUyxHQUFXLENBQUMsQ0FBQztRQUV0QixzQkFBZ0IsR0FBVyxHQUFHLENBQUM7O0lBSXRDLENBQUM7SUFFTSxvQ0FBVSxHQUFqQixVQUFrQixFQUFVLEVBQUUsRUFBVTtRQUN0QyxJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixvQ0FBb0M7UUFDcEMsMkNBQTJDO1FBQzNDLElBQUksQ0FBQyxLQUFLLENBQUMsYUFBYSxDQUFDLElBQUksQ0FBQyxLQUFLLENBQUMsV0FBVyxFQUFFLEVBQUUsSUFBSSxDQUFDLFlBQVksQ0FBQyxDQUFDO1FBRXRFLElBQU0sTUFBTSxHQUFXLElBQUksQ0FBQyxLQUFLLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDN0MsSUFBTSxNQUFNLEdBQVcsSUFBSSxDQUFDLEtBQUssQ0FBQyxRQUFRLEVBQUUsQ0FBQztRQUM3QyxJQUFJLENBQUMsYUFBYSxHQUFHLE1BQU0sR0FBRyxNQUFNLENBQUM7SUFDdkMsQ0FBQztJQUNILHNC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