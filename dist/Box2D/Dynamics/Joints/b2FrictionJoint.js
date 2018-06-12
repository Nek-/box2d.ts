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
/// Friction joint definition.
var b2FrictionJointDef = /** @class */ (function (_super) {
    __extends(b2FrictionJointDef, _super);
    function b2FrictionJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_frictionJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2();
        _this.localAnchorB = new b2Math_1.b2Vec2();
        _this.maxForce = 0;
        _this.maxTorque = 0;
        return _this;
    }
    b2FrictionJointDef.prototype.Initialize = function (bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
    };
    return b2FrictionJointDef;
}(b2Joint_1.b2JointDef));
exports.b2FrictionJointDef = b2FrictionJointDef;
var b2FrictionJoint = /** @class */ (function (_super) {
    __extends(b2FrictionJoint, _super);
    function b2FrictionJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        // Solver shared
        _this.m_linearImpulse = new b2Math_1.b2Vec2();
        _this.m_angularImpulse = 0;
        _this.m_maxForce = 0;
        _this.m_maxTorque = 0;
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
        _this.m_linearMass = new b2Math_1.b2Mat22();
        _this.m_angularMass = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_K = new b2Math_1.b2Mat22();
        _this.m_localAnchorA.Copy(def.localAnchorA);
        _this.m_localAnchorB.Copy(def.localAnchorB);
        _this.m_linearImpulse.SetZero();
        _this.m_maxForce = b2Settings_1.b2Maybe(def.maxForce, 0);
        _this.m_maxTorque = b2Settings_1.b2Maybe(def.maxTorque, 0);
        _this.m_linearMass.SetZero();
        return _this;
    }
    b2FrictionJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        // const cA: b2Vec2 = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        // const cB: b2Vec2 = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // const qA: b2Rot = new b2Rot(aA), qB: b2Rot = new b2Rot(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // Compute the effective mass matrix.
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var K = this.m_K; // new b2Mat22();
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
        K.GetInverse(this.m_linearMass);
        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            // m_linearImpulse *= data.step.dtRatio;
            this.m_linearImpulse.SelfMul(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;
            // const P: b2Vec2(m_linearImpulse.x, m_linearImpulse.y);
            var P = this.m_linearImpulse;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            // wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
            wA -= iA * (b2Math_1.b2Vec2.CrossVV(this.m_rA, P) + this.m_angularImpulse);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            // wB += iB * (b2Cross(m_rB, P) + m_angularImpulse);
            wB += iB * (b2Math_1.b2Vec2.CrossVV(this.m_rB, P) + this.m_angularImpulse);
        }
        else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2FrictionJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;
        var h = data.step.dt;
        // Solve angular friction
        {
            var Cdot = wB - wA;
            var impulse = (-this.m_angularMass * Cdot);
            var oldImpulse = this.m_angularImpulse;
            var maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Math_1.b2Clamp(this.m_angularImpulse + impulse, (-maxImpulse), maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve linear friction
        {
            // b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
            var Cdot_v2 = b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Math_1.b2Vec2.s_t1), b2FrictionJoint.SolveVelocityConstraints_s_Cdot_v2);
            // b2Vec2 impulse = -b2Mul(m_linearMass, Cdot);
            var impulseV = b2Math_1.b2Mat22.MulMV(this.m_linearMass, Cdot_v2, b2FrictionJoint.SolveVelocityConstraints_s_impulseV).SelfNeg();
            // b2Vec2 oldImpulse = m_linearImpulse;
            var oldImpulseV = b2FrictionJoint.SolveVelocityConstraints_s_oldImpulseV.Copy(this.m_linearImpulse);
            // m_linearImpulse += impulse;
            this.m_linearImpulse.SelfAdd(impulseV);
            var maxImpulse = h * this.m_maxForce;
            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                this.m_linearImpulse.SelfMul(maxImpulse);
            }
            // impulse = m_linearImpulse - oldImpulse;
            b2Math_1.b2Vec2.SubVV(this.m_linearImpulse, oldImpulseV, impulseV);
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulseV);
            // wA -= iA * b2Cross(m_rA, impulse);
            wA -= iA * b2Math_1.b2Vec2.CrossVV(this.m_rA, impulseV);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulseV);
            // wB += iB * b2Cross(m_rB, impulse);
            wB += iB * b2Math_1.b2Vec2.CrossVV(this.m_rB, impulseV);
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2FrictionJoint.prototype.SolvePositionConstraints = function (data) {
        return true;
    };
    b2FrictionJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2FrictionJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2FrictionJoint.prototype.GetReactionForce = function (inv_dt, out) {
        out.x = inv_dt * this.m_linearImpulse.x;
        out.y = inv_dt * this.m_linearImpulse.y;
        return out;
    };
    b2FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
        return inv_dt * this.m_angularImpulse;
    };
    b2FrictionJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2FrictionJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2FrictionJoint.prototype.SetMaxForce = function (force) {
        this.m_maxForce = force;
    };
    b2FrictionJoint.prototype.GetMaxForce = function () {
        return this.m_maxForce;
    };
    b2FrictionJoint.prototype.SetMaxTorque = function (torque) {
        this.m_maxTorque = torque;
    };
    b2FrictionJoint.prototype.GetMaxTorque = function () {
        return this.m_maxTorque;
    };
    b2FrictionJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2FrictionJointDef = new b2FrictionJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.maxForce = %.15f;\n", this.m_maxForce);
        log("  jd.maxTorque = %.15f;\n", this.m_maxTorque);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2FrictionJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Math_1.b2Vec2();
    b2FrictionJoint.SolveVelocityConstraints_s_impulseV = new b2Math_1.b2Vec2();
    b2FrictionJoint.SolveVelocityConstraints_s_oldImpulseV = new b2Math_1.b2Vec2();
    return b2FrictionJoint;
}(b2Joint_1.b2Joint));
exports.b2FrictionJoint = b2FrictionJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJGcmljdGlvbkpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyRnJpY3Rpb25Kb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLHNEQUFrRDtBQUNsRCw4Q0FBMEU7QUFDMUUscUNBQTBFO0FBYzFFLDhCQUE4QjtBQUM5QjtJQUF3QyxzQ0FBVTtJQVNoRDtRQUFBLFlBQ0Usa0JBQU0scUJBQVcsQ0FBQyxlQUFlLENBQUMsU0FDbkM7UUFWZSxrQkFBWSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFFcEMsa0JBQVksR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBRTdDLGNBQVEsR0FBVyxDQUFDLENBQUM7UUFFckIsZUFBUyxHQUFXLENBQUMsQ0FBQzs7SUFJN0IsQ0FBQztJQUVNLHVDQUFVLEdBQWpCLFVBQWtCLEVBQVUsRUFBRSxFQUFVLEVBQUUsTUFBYztRQUN0RCxJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxHQUFHLEVBQUUsQ0FBQztRQUNoQixJQUFJLENBQUMsS0FBSyxDQUFDLGFBQWEsQ0FBQyxNQUFNLEVBQUUsSUFBSSxDQUFDLFlBQVksQ0FBQyxDQUFDO1FBQ3BELElBQUksQ0FBQyxLQUFLLENBQUMsYUFBYSxDQUFDLE1BQU0sRUFBRSxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUM7SUFDdEQsQ0FBQztJQUNILHlCQUFDO0FBQUQsQ0FBQyxBQW5CRCxDQUF3QyxvQkFBVSxHQW1CakQ7QUFuQlksZ0RBQWtCO0FBcUIvQjtJQUFxQyxtQ0FBTztJQThCMUMseUJBQVksR0FBd0I7UUFBcEMsWUFDRSxrQkFBTSxHQUFHLENBQUMsU0FVWDtRQXhDZSxvQkFBYyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDdEMsb0JBQWMsR0FB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