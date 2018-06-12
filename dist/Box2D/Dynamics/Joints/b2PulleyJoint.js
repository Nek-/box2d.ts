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
// DEBUG: import { b2Assert, b2_epsilon } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
exports.b2_minPulleyLength = 2;
/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
var b2PulleyJointDef = /** @class */ (function (_super) {
    __extends(b2PulleyJointDef, _super);
    function b2PulleyJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_pulleyJoint) || this;
        _this.groundAnchorA = new b2Math_1.b2Vec2(-1, 1);
        _this.groundAnchorB = new b2Math_1.b2Vec2(1, 1);
        _this.localAnchorA = new b2Math_1.b2Vec2(-1, 0);
        _this.localAnchorB = new b2Math_1.b2Vec2(1, 0);
        _this.lengthA = 0;
        _this.lengthB = 0;
        _this.ratio = 1;
        _this.collideConnected = true;
        return _this;
    }
    b2PulleyJointDef.prototype.Initialize = function (bA, bB, groundA, groundB, anchorA, anchorB, r) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.groundAnchorA.Copy(groundA);
        this.groundAnchorB.Copy(groundB);
        this.bodyA.GetLocalPoint(anchorA, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchorB, this.localAnchorB);
        this.lengthA = b2Math_1.b2Vec2.DistanceVV(anchorA, groundA);
        this.lengthB = b2Math_1.b2Vec2.DistanceVV(anchorB, groundB);
        this.ratio = r;
        // DEBUG: b2Assert(this.ratio > b2_epsilon);
    };
    return b2PulleyJointDef;
}(b2Joint_1.b2JointDef));
exports.b2PulleyJointDef = b2PulleyJointDef;
var b2PulleyJoint = /** @class */ (function (_super) {
    __extends(b2PulleyJoint, _super);
    function b2PulleyJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_groundAnchorA = new b2Math_1.b2Vec2();
        _this.m_groundAnchorB = new b2Math_1.b2Vec2();
        _this.m_lengthA = 0;
        _this.m_lengthB = 0;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_constant = 0;
        _this.m_ratio = 0;
        _this.m_impulse = 0;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_uA = new b2Math_1.b2Vec2();
        _this.m_uB = new b2Math_1.b2Vec2();
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
        _this.m_groundAnchorA.Copy(b2Settings_1.b2Maybe(def.groundAnchorA, new b2Math_1.b2Vec2(-1, 1)));
        _this.m_groundAnchorB.Copy(b2Settings_1.b2Maybe(def.groundAnchorB, new b2Math_1.b2Vec2(1, 0)));
        _this.m_localAnchorA.Copy(b2Settings_1.b2Maybe(def.localAnchorA, new b2Math_1.b2Vec2(-1, 0)));
        _this.m_localAnchorB.Copy(b2Settings_1.b2Maybe(def.localAnchorB, new b2Math_1.b2Vec2(1, 0)));
        _this.m_lengthA = b2Settings_1.b2Maybe(def.lengthA, 0);
        _this.m_lengthB = b2Settings_1.b2Maybe(def.lengthB, 0);
        // DEBUG: b2Assert(b2Maybe(def.ratio, 1) !== 0);
        _this.m_ratio = b2Settings_1.b2Maybe(def.ratio, 1);
        _this.m_constant = b2Settings_1.b2Maybe(def.lengthA, 0) + _this.m_ratio * b2Settings_1.b2Maybe(def.lengthB, 0);
        _this.m_impulse = 0;
        return _this;
    }
    b2PulleyJoint.prototype.InitVelocityConstraints = function (data) {
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
        // b2Rot qA(aA), qB(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Get the pulley axes.
        // m_uA = cA + m_rA - m_groundAnchorA;
        this.m_uA.Copy(cA).SelfAdd(this.m_rA).SelfSub(this.m_groundAnchorA);
        // m_uB = cB + m_rB - m_groundAnchorB;
        this.m_uB.Copy(cB).SelfAdd(this.m_rB).SelfSub(this.m_groundAnchorB);
        var lengthA = this.m_uA.Length();
        var lengthB = this.m_uB.Length();
        if (lengthA > 10 * b2Settings_1.b2_linearSlop) {
            this.m_uA.SelfMul(1 / lengthA);
        }
        else {
            this.m_uA.SetZero();
        }
        if (lengthB > 10 * b2Settings_1.b2_linearSlop) {
            this.m_uB.SelfMul(1 / lengthB);
        }
        else {
            this.m_uB.SetZero();
        }
        // Compute effective mass.
        var ruA = b2Math_1.b2Vec2.CrossVV(this.m_rA, this.m_uA);
        var ruB = b2Math_1.b2Vec2.CrossVV(this.m_rB, this.m_uB);
        var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        var mB = this.m_invMassB + this.m_invIB * ruB * ruB;
        this.m_mass = mA + this.m_ratio * this.m_ratio * mB;
        if (this.m_mass > 0) {
            this.m_mass = 1 / this.m_mass;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            this.m_impulse *= data.step.dtRatio;
            // Warm starting.
            // b2Vec2 PA = -(m_impulse) * m_uA;
            var PA = b2Math_1.b2Vec2.MulSV(-(this.m_impulse), this.m_uA, b2PulleyJoint.InitVelocityConstraints_s_PA);
            // b2Vec2 PB = (-m_ratio * m_impulse) * m_uB;
            var PB = b2Math_1.b2Vec2.MulSV((-this.m_ratio * this.m_impulse), this.m_uB, b2PulleyJoint.InitVelocityConstraints_s_PB);
            // vA += m_invMassA * PA;
            vA.SelfMulAdd(this.m_invMassA, PA);
            wA += this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, PA);
            // vB += m_invMassB * PB;
            vB.SelfMulAdd(this.m_invMassB, PB);
            wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, PB);
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2PulleyJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // b2Vec2 vpA = vA + b2Cross(wA, m_rA);
        var vpA = b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2PulleyJoint.SolveVelocityConstraints_s_vpA);
        // b2Vec2 vpB = vB + b2Cross(wB, m_rB);
        var vpB = b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2PulleyJoint.SolveVelocityConstraints_s_vpB);
        var Cdot = -b2Math_1.b2Vec2.DotVV(this.m_uA, vpA) - this.m_ratio * b2Math_1.b2Vec2.DotVV(this.m_uB, vpB);
        var impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;
        // b2Vec2 PA = -impulse * m_uA;
        var PA = b2Math_1.b2Vec2.MulSV(-impulse, this.m_uA, b2PulleyJoint.SolveVelocityConstraints_s_PA);
        // b2Vec2 PB = -m_ratio * impulse * m_uB;
        var PB = b2Math_1.b2Vec2.MulSV(-this.m_ratio * impulse, this.m_uB, b2PulleyJoint.SolveVelocityConstraints_s_PB);
        // vA += m_invMassA * PA;
        vA.SelfMulAdd(this.m_invMassA, PA);
        wA += this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, PA);
        // vB += m_invMassB * PB;
        vB.SelfMulAdd(this.m_invMassB, PB);
        wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, PB);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2PulleyJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        // b2Rot qA(aA), qB(aB);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Get the pulley axes.
        // b2Vec2 uA = cA + rA - m_groundAnchorA;
        var uA = this.m_uA.Copy(cA).SelfAdd(rA).SelfSub(this.m_groundAnchorA);
        // b2Vec2 uB = cB + rB - m_groundAnchorB;
        var uB = this.m_uB.Copy(cB).SelfAdd(rB).SelfSub(this.m_groundAnchorB);
        var lengthA = uA.Length();
        var lengthB = uB.Length();
        if (lengthA > 10 * b2Settings_1.b2_linearSlop) {
            uA.SelfMul(1 / lengthA);
        }
        else {
            uA.SetZero();
        }
        if (lengthB > 10 * b2Settings_1.b2_linearSlop) {
            uB.SelfMul(1 / lengthB);
        }
        else {
            uB.SetZero();
        }
        // Compute effective mass.
        var ruA = b2Math_1.b2Vec2.CrossVV(rA, uA);
        var ruB = b2Math_1.b2Vec2.CrossVV(rB, uB);
        var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        var mB = this.m_invMassB + this.m_invIB * ruB * ruB;
        var mass = mA + this.m_ratio * this.m_ratio * mB;
        if (mass > 0) {
            mass = 1 / mass;
        }
        var C = this.m_constant - lengthA - this.m_ratio * lengthB;
        var linearError = b2Math_1.b2Abs(C);
        var impulse = -mass * C;
        // b2Vec2 PA = -impulse * uA;
        var PA = b2Math_1.b2Vec2.MulSV(-impulse, uA, b2PulleyJoint.SolvePositionConstraints_s_PA);
        // b2Vec2 PB = -m_ratio * impulse * uB;
        var PB = b2Math_1.b2Vec2.MulSV(-this.m_ratio * impulse, uB, b2PulleyJoint.SolvePositionConstraints_s_PB);
        // cA += m_invMassA * PA;
        cA.SelfMulAdd(this.m_invMassA, PA);
        aA += this.m_invIA * b2Math_1.b2Vec2.CrossVV(rA, PA);
        // cB += m_invMassB * PB;
        cB.SelfMulAdd(this.m_invMassB, PB);
        aB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(rB, PB);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError < b2Settings_1.b2_linearSlop;
    };
    b2PulleyJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2PulleyJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2PulleyJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // b2Vec2 P = m_impulse * m_uB;
        // return inv_dt * P;
        out.x = inv_dt * this.m_impulse * this.m_uB.x;
        out.y = inv_dt * this.m_impulse * this.m_uB.y;
        return out;
    };
    b2PulleyJoint.prototype.GetReactionTorque = function (inv_dt) {
        return 0;
    };
    b2PulleyJoint.prototype.GetGroundAnchorA = function () {
        return this.m_groundAnchorA;
    };
    b2PulleyJoint.prototype.GetGroundAnchorB = function () {
        return this.m_groundAnchorB;
    };
    b2PulleyJoint.prototype.GetLengthA = function () {
        return this.m_lengthA;
    };
    b2PulleyJoint.prototype.GetLengthB = function () {
        return this.m_lengthB;
    };
    b2PulleyJoint.prototype.GetRatio = function () {
        return this.m_ratio;
    };
    b2PulleyJoint.prototype.GetCurrentLengthA = function () {
        // b2Vec2 p = m_bodyA->GetWorldPoint(m_localAnchorA);
        // b2Vec2 s = m_groundAnchorA;
        // b2Vec2 d = p - s;
        // return d.Length();
        var p = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, b2PulleyJoint.GetCurrentLengthA_s_p);
        var s = this.m_groundAnchorA;
        return b2Math_1.b2Vec2.DistanceVV(p, s);
    };
    b2PulleyJoint.prototype.GetCurrentLengthB = function () {
        // b2Vec2 p = m_bodyB->GetWorldPoint(m_localAnchorB);
        // b2Vec2 s = m_groundAnchorB;
        // b2Vec2 d = p - s;
        // return d.Length();
        var p = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, b2PulleyJoint.GetCurrentLengthB_s_p);
        var s = this.m_groundAnchorB;
        return b2Math_1.b2Vec2.DistanceVV(p, s);
    };
    b2PulleyJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2PulleyJointDef = new b2PulleyJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.groundAnchorA.Set(%.15f, %.15f);\n", this.m_groundAnchorA.x, this.m_groundAnchorA.y);
        log("  jd.groundAnchorB.Set(%.15f, %.15f);\n", this.m_groundAnchorB.x, this.m_groundAnchorB.y);
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.lengthA = %.15f;\n", this.m_lengthA);
        log("  jd.lengthB = %.15f;\n", this.m_lengthB);
        log("  jd.ratio = %.15f;\n", this.m_ratio);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2PulleyJoint.prototype.ShiftOrigin = function (newOrigin) {
        this.m_groundAnchorA.SelfSub(newOrigin);
        this.m_groundAnchorB.SelfSub(newOrigin);
    };
    b2PulleyJoint.InitVelocityConstraints_s_PA = new b2Math_1.b2Vec2();
    b2PulleyJoint.InitVelocityConstraints_s_PB = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolveVelocityConstraints_s_vpA = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolveVelocityConstraints_s_vpB = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolveVelocityConstraints_s_PA = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolveVelocityConstraints_s_PB = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolvePositionConstraints_s_PA = new b2Math_1.b2Vec2();
    b2PulleyJoint.SolvePositionConstraints_s_PB = new b2Math_1.b2Vec2();
    b2PulleyJoint.GetCurrentLengthA_s_p = new b2Math_1.b2Vec2();
    b2PulleyJoint.GetCurrentLengthB_s_p = new b2Math_1.b2Vec2();
    return b2PulleyJoint;
}(b2Joint_1.b2Joint));
exports.b2PulleyJoint = b2PulleyJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQdWxsZXlKb2ludC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0pvaW50cy9iMlB1bGxleUpvaW50LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBRUYseUVBQXlFO0FBQ3pFLHNEQUFpRTtBQUNqRSw4Q0FBK0Q7QUFFL0QscUNBQTBFO0FBRzdELFFBQUEsa0JBQWtCLEdBQVcsQ0FBQyxDQUFDO0FBa0I1Qyw4REFBOEQ7QUFDOUQsdURBQXVEO0FBQ3ZEO0lBQXNDLG9DQUFVO0lBZTlDO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLGFBQWEsQ0FBQyxTQUVqQztRQWpCZSxtQkFBYSxHQUFXLElBQUksZUFBTSxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRTFDLG1CQUFhLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRXpDLGtCQUFZLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFekMsa0JBQVksR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFakQsYUFBTyxHQUFXLENBQUMsQ0FBQztRQUVwQixhQUFPLEdBQVcsQ0FBQyxDQUFDO1FBRXBCLFdBQUssR0FBVyxDQUFDLENBQUM7UUFJdkIsS0FBSSxDQUFDLGdCQUFnQixHQUFHLElBQUksQ0FBQzs7SUFDL0IsQ0FBQztJQUVNLHFDQUFVLEdBQWpCLFVBQWtCLEVBQVUsRUFBRSxFQUFVLEVBQUUsT0FBZSxFQUFFLE9BQWUsRUFBRSxPQUFlLEVBQUUsT0FBZSxFQUFFLENBQVM7UUFDckgsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFBSSxDQUFDLEtBQUssR0FBRyxFQUFFLENBQUM7UUFDaEIsSUFB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