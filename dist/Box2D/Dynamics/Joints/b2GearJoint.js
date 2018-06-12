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
// DEBUG: import { b2IsValid } from "../../Common/b2Math";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
var b2GearJointDef = /** @class */ (function (_super) {
    __extends(b2GearJointDef, _super);
    function b2GearJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_gearJoint) || this;
        _this.ratio = 1;
        return _this;
    }
    return b2GearJointDef;
}(b2Joint_1.b2JointDef));
exports.b2GearJointDef = b2GearJointDef;
var b2GearJoint = /** @class */ (function (_super) {
    __extends(b2GearJoint, _super);
    function b2GearJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_typeA = b2Joint_1.b2JointType.e_unknownJoint;
        _this.m_typeB = b2Joint_1.b2JointType.e_unknownJoint;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_localAnchorC = new b2Math_1.b2Vec2();
        _this.m_localAnchorD = new b2Math_1.b2Vec2();
        _this.m_localAxisC = new b2Math_1.b2Vec2();
        _this.m_localAxisD = new b2Math_1.b2Vec2();
        _this.m_referenceAngleA = 0;
        _this.m_referenceAngleB = 0;
        _this.m_constant = 0;
        _this.m_ratio = 0;
        _this.m_impulse = 0;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_indexC = 0;
        _this.m_indexD = 0;
        _this.m_lcA = new b2Math_1.b2Vec2();
        _this.m_lcB = new b2Math_1.b2Vec2();
        _this.m_lcC = new b2Math_1.b2Vec2();
        _this.m_lcD = new b2Math_1.b2Vec2();
        _this.m_mA = 0;
        _this.m_mB = 0;
        _this.m_mC = 0;
        _this.m_mD = 0;
        _this.m_iA = 0;
        _this.m_iB = 0;
        _this.m_iC = 0;
        _this.m_iD = 0;
        _this.m_JvAC = new b2Math_1.b2Vec2();
        _this.m_JvBD = new b2Math_1.b2Vec2();
        _this.m_JwA = 0;
        _this.m_JwB = 0;
        _this.m_JwC = 0;
        _this.m_JwD = 0;
        _this.m_mass = 0;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_qC = new b2Math_1.b2Rot();
        _this.m_qD = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_lalcC = new b2Math_1.b2Vec2();
        _this.m_lalcD = new b2Math_1.b2Vec2();
        _this.m_joint1 = def.joint1;
        _this.m_joint2 = def.joint2;
        _this.m_typeA = _this.m_joint1.GetType();
        _this.m_typeB = _this.m_joint2.GetType();
        // DEBUG: b2Assert(this.m_typeA === b2JointType.e_revoluteJoint || this.m_typeA === b2JointType.e_prismaticJoint);
        // DEBUG: b2Assert(this.m_typeB === b2JointType.e_revoluteJoint || this.m_typeB === b2JointType.e_prismaticJoint);
        var coordinateA, coordinateB;
        // TODO_ERIN there might be some problem with the joint edges in b2Joint.
        _this.m_bodyC = _this.m_joint1.GetBodyA();
        _this.m_bodyA = _this.m_joint1.GetBodyB();
        // Get geometry of joint1
        var xfA = _this.m_bodyA.m_xf;
        var aA = _this.m_bodyA.m_sweep.a;
        var xfC = _this.m_bodyC.m_xf;
        var aC = _this.m_bodyC.m_sweep.a;
        if (_this.m_typeA === b2Joint_1.b2JointType.e_revoluteJoint) {
            var revolute = def.joint1;
            _this.m_localAnchorC.Copy(revolute.m_localAnchorA);
            _this.m_localAnchorA.Copy(revolute.m_localAnchorB);
            _this.m_referenceAngleA = revolute.m_referenceAngle;
            _this.m_localAxisC.SetZero();
            coordinateA = aA - aC - _this.m_referenceAngleA;
        }
        else {
            var prismatic = def.joint1;
            _this.m_localAnchorC.Copy(prismatic.m_localAnchorA);
            _this.m_localAnchorA.Copy(prismatic.m_localAnchorB);
            _this.m_referenceAngleA = prismatic.m_referenceAngle;
            _this.m_localAxisC.Copy(prismatic.m_localXAxisA);
            // b2Vec2 pC = m_localAnchorC;
            var pC = _this.m_localAnchorC;
            // b2Vec2 pA = b2MulT(xfC.q, b2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
            var pA = b2Math_1.b2Rot.MulTRV(xfC.q, b2Math_1.b2Vec2.AddVV(b2Math_1.b2Rot.MulRV(xfA.q, _this.m_localAnchorA, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(xfA.p, xfC.p, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0); // pA uses s_t0
            // coordinateA = b2Dot(pA - pC, m_localAxisC);
            coordinateA = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pA, pC, b2Math_1.b2Vec2.s_t0), _this.m_localAxisC);
        }
        _this.m_bodyD = _this.m_joint2.GetBodyA();
        _this.m_bodyB = _this.m_joint2.GetBodyB();
        // Get geometry of joint2
        var xfB = _this.m_bodyB.m_xf;
        var aB = _this.m_bodyB.m_sweep.a;
        var xfD = _this.m_bodyD.m_xf;
        var aD = _this.m_bodyD.m_sweep.a;
        if (_this.m_typeB === b2Joint_1.b2JointType.e_revoluteJoint) {
            var revolute = def.joint2;
            _this.m_localAnchorD.Copy(revolute.m_localAnchorA);
            _this.m_localAnchorB.Copy(revolute.m_localAnchorB);
            _this.m_referenceAngleB = revolute.m_referenceAngle;
            _this.m_localAxisD.SetZero();
            coordinateB = aB - aD - _this.m_referenceAngleB;
        }
        else {
            var prismatic = def.joint2;
            _this.m_localAnchorD.Copy(prismatic.m_localAnchorA);
            _this.m_localAnchorB.Copy(prismatic.m_localAnchorB);
            _this.m_referenceAngleB = prismatic.m_referenceAngle;
            _this.m_localAxisD.Copy(prismatic.m_localXAxisA);
            // b2Vec2 pD = m_localAnchorD;
            var pD = _this.m_localAnchorD;
            // b2Vec2 pB = b2MulT(xfD.q, b2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
            var pB = b2Math_1.b2Rot.MulTRV(xfD.q, b2Math_1.b2Vec2.AddVV(b2Math_1.b2Rot.MulRV(xfB.q, _this.m_localAnchorB, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.SubVV(xfB.p, xfD.p, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0); // pB uses s_t0
            // coordinateB = b2Dot(pB - pD, m_localAxisD);
            coordinateB = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pB, pD, b2Math_1.b2Vec2.s_t0), _this.m_localAxisD);
        }
        _this.m_ratio = b2Settings_1.b2Maybe(def.ratio, 1);
        _this.m_constant = coordinateA + _this.m_ratio * coordinateB;
        _this.m_impulse = 0;
        return _this;
    }
    b2GearJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.Copy(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.Copy(this.m_bodyD.m_sweep.localCenter);
        this.m_mA = this.m_bodyA.m_invMass;
        this.m_mB = this.m_bodyB.m_invMass;
        this.m_mC = this.m_bodyC.m_invMass;
        this.m_mD = this.m_bodyD.m_invMass;
        this.m_iA = this.m_bodyA.m_invI;
        this.m_iB = this.m_bodyB.m_invI;
        this.m_iC = this.m_bodyC.m_invI;
        this.m_iD = this.m_bodyD.m_invI;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var aC = data.positions[this.m_indexC].a;
        var vC = data.velocities[this.m_indexC].v;
        var wC = data.velocities[this.m_indexC].w;
        var aD = data.positions[this.m_indexD].a;
        var vD = data.velocities[this.m_indexD].v;
        var wD = data.velocities[this.m_indexD].w;
        // b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB), qC = this.m_qC.SetAngle(aC), qD = this.m_qD.SetAngle(aD);
        this.m_mass = 0;
        if (this.m_typeA === b2Joint_1.b2JointType.e_revoluteJoint) {
            this.m_JvAC.SetZero();
            this.m_JwA = 1;
            this.m_JwC = 1;
            this.m_mass += this.m_iA + this.m_iC;
        }
        else {
            // b2Vec2 u = b2Mul(qC, m_localAxisC);
            var u = b2Math_1.b2Rot.MulRV(qC, this.m_localAxisC, b2GearJoint.InitVelocityConstraints_s_u);
            // b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorC, this.m_lcC, this.m_lalcC);
            var rC = b2Math_1.b2Rot.MulRV(qC, this.m_lalcC, b2GearJoint.InitVelocityConstraints_s_rC);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_lcA, this.m_lalcA);
            var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, b2GearJoint.InitVelocityConstraints_s_rA);
            // m_JvAC = u;
            this.m_JvAC.Copy(u);
            // m_JwC = b2Cross(rC, u);
            this.m_JwC = b2Math_1.b2Vec2.CrossVV(rC, u);
            // m_JwA = b2Cross(rA, u);
            this.m_JwA = b2Math_1.b2Vec2.CrossVV(rA, u);
            this.m_mass += this.m_mC + this.m_mA + this.m_iC * this.m_JwC * this.m_JwC + this.m_iA * this.m_JwA * this.m_JwA;
        }
        if (this.m_typeB === b2Joint_1.b2JointType.e_revoluteJoint) {
            this.m_JvBD.SetZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        }
        else {
            // b2Vec2 u = b2Mul(qD, m_localAxisD);
            var u = b2Math_1.b2Rot.MulRV(qD, this.m_localAxisD, b2GearJoint.InitVelocityConstraints_s_u);
            // b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorD, this.m_lcD, this.m_lalcD);
            var rD = b2Math_1.b2Rot.MulRV(qD, this.m_lalcD, b2GearJoint.InitVelocityConstraints_s_rD);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
            b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_lcB, this.m_lalcB);
            var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, b2GearJoint.InitVelocityConstraints_s_rB);
            // m_JvBD = m_ratio * u;
            b2Math_1.b2Vec2.MulSV(this.m_ratio, u, this.m_JvBD);
            // m_JwD = m_ratio * b2Cross(rD, u);
            this.m_JwD = this.m_ratio * b2Math_1.b2Vec2.CrossVV(rD, u);
            // m_JwB = m_ratio * b2Cross(rB, u);
            this.m_JwB = this.m_ratio * b2Math_1.b2Vec2.CrossVV(rB, u);
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * this.m_JwD * this.m_JwD + this.m_iB * this.m_JwB * this.m_JwB;
        }
        // Compute effective mass.
        this.m_mass = this.m_mass > 0 ? 1 / this.m_mass : 0;
        if (data.step.warmStarting) {
            // vA += (m_mA * m_impulse) * m_JvAC;
            vA.SelfMulAdd(this.m_mA * this.m_impulse, this.m_JvAC);
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            // vB += (m_mB * m_impulse) * m_JvBD;
            vB.SelfMulAdd(this.m_mB * this.m_impulse, this.m_JvBD);
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            // vC -= (m_mC * m_impulse) * m_JvAC;
            vC.SelfMulSub(this.m_mC * this.m_impulse, this.m_JvAC);
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            // vD -= (m_mD * m_impulse) * m_JvBD;
            vD.SelfMulSub(this.m_mD * this.m_impulse, this.m_JvBD);
            wD -= this.m_iD * this.m_impulse * this.m_JwD;
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    };
    b2GearJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var vC = data.velocities[this.m_indexC].v;
        var wC = data.velocities[this.m_indexC].w;
        var vD = data.velocities[this.m_indexD].v;
        var wD = data.velocities[this.m_indexD].w;
        // float32 Cdot = b2Dot(m_JvAC, vA - vC) + b2Dot(m_JvBD, vB - vD);
        var Cdot = b2Math_1.b2Vec2.DotVV(this.m_JvAC, b2Math_1.b2Vec2.SubVV(vA, vC, b2Math_1.b2Vec2.s_t0)) +
            b2Math_1.b2Vec2.DotVV(this.m_JvBD, b2Math_1.b2Vec2.SubVV(vB, vD, b2Math_1.b2Vec2.s_t0));
        Cdot += (this.m_JwA * wA - this.m_JwC * wC) + (this.m_JwB * wB - this.m_JwD * wD);
        var impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;
        // vA += (m_mA * impulse) * m_JvAC;
        vA.SelfMulAdd((this.m_mA * impulse), this.m_JvAC);
        wA += this.m_iA * impulse * this.m_JwA;
        // vB += (m_mB * impulse) * m_JvBD;
        vB.SelfMulAdd((this.m_mB * impulse), this.m_JvBD);
        wB += this.m_iB * impulse * this.m_JwB;
        // vC -= (m_mC * impulse) * m_JvAC;
        vC.SelfMulSub((this.m_mC * impulse), this.m_JvAC);
        wC -= this.m_iC * impulse * this.m_JwC;
        // vD -= (m_mD * impulse) * m_JvBD;
        vD.SelfMulSub((this.m_mD * impulse), this.m_JvBD);
        wD -= this.m_iD * impulse * this.m_JwD;
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    };
    b2GearJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var cC = data.positions[this.m_indexC].c;
        var aC = data.positions[this.m_indexC].a;
        var cD = data.positions[this.m_indexD].c;
        var aD = data.positions[this.m_indexD].a;
        // b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB), qC = this.m_qC.SetAngle(aC), qD = this.m_qD.SetAngle(aD);
        var linearError = 0;
        var coordinateA, coordinateB;
        var JvAC = this.m_JvAC, JvBD = this.m_JvBD;
        var JwA, JwB, JwC, JwD;
        var mass = 0;
        if (this.m_typeA === b2Joint_1.b2JointType.e_revoluteJoint) {
            JvAC.SetZero();
            JwA = 1;
            JwC = 1;
            mass += this.m_iA + this.m_iC;
            coordinateA = aA - aC - this.m_referenceAngleA;
        }
        else {
            // b2Vec2 u = b2Mul(qC, m_localAxisC);
            var u = b2Math_1.b2Rot.MulRV(qC, this.m_localAxisC, b2GearJoint.SolvePositionConstraints_s_u);
            // b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
            var rC = b2Math_1.b2Rot.MulRV(qC, this.m_lalcC, b2GearJoint.SolvePositionConstraints_s_rC);
            // b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
            var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, b2GearJoint.SolvePositionConstraints_s_rA);
            // JvAC = u;
            JvAC.Copy(u);
            // JwC = b2Cross(rC, u);
            JwC = b2Math_1.b2Vec2.CrossVV(rC, u);
            // JwA = b2Cross(rA, u);
            JwA = b2Math_1.b2Vec2.CrossVV(rA, u);
            mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;
            // b2Vec2 pC = m_localAnchorC - m_lcC;
            var pC = this.m_lalcC;
            // b2Vec2 pA = b2MulT(qC, rA + (cA - cC));
            var pA = b2Math_1.b2Rot.MulTRV(qC, b2Math_1.b2Vec2.AddVV(rA, b2Math_1.b2Vec2.SubVV(cA, cC, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0); // pA uses s_t0
            // coordinateA = b2Dot(pA - pC, m_localAxisC);
            coordinateA = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pA, pC, b2Math_1.b2Vec2.s_t0), this.m_localAxisC);
        }
        if (this.m_typeB === b2Joint_1.b2JointType.e_revoluteJoint) {
            JvBD.SetZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
            coordinateB = aB - aD - this.m_referenceAngleB;
        }
        else {
            // b2Vec2 u = b2Mul(qD, m_localAxisD);
            var u = b2Math_1.b2Rot.MulRV(qD, this.m_localAxisD, b2GearJoint.SolvePositionConstraints_s_u);
            // b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
            var rD = b2Math_1.b2Rot.MulRV(qD, this.m_lalcD, b2GearJoint.SolvePositionConstraints_s_rD);
            // b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
            var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, b2GearJoint.SolvePositionConstraints_s_rB);
            // JvBD = m_ratio * u;
            b2Math_1.b2Vec2.MulSV(this.m_ratio, u, JvBD);
            // JwD = m_ratio * b2Cross(rD, u);
            JwD = this.m_ratio * b2Math_1.b2Vec2.CrossVV(rD, u);
            // JwB = m_ratio * b2Cross(rB, u);
            JwB = this.m_ratio * b2Math_1.b2Vec2.CrossVV(rB, u);
            mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * JwD * JwD + this.m_iB * JwB * JwB;
            // b2Vec2 pD = m_localAnchorD - m_lcD;
            var pD = this.m_lalcD;
            // b2Vec2 pB = b2MulT(qD, rB + (cB - cD));
            var pB = b2Math_1.b2Rot.MulTRV(qD, b2Math_1.b2Vec2.AddVV(rB, b2Math_1.b2Vec2.SubVV(cB, cD, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0); // pB uses s_t0
            // coordinateB = b2Dot(pB - pD, m_localAxisD);
            coordinateB = b2Math_1.b2Vec2.DotVV(b2Math_1.b2Vec2.SubVV(pB, pD, b2Math_1.b2Vec2.s_t0), this.m_localAxisD);
        }
        var C = (coordinateA + this.m_ratio * coordinateB) - this.m_constant;
        var impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }
        // cA += m_mA * impulse * JvAC;
        cA.SelfMulAdd(this.m_mA * impulse, JvAC);
        aA += this.m_iA * impulse * JwA;
        // cB += m_mB * impulse * JvBD;
        cB.SelfMulAdd(this.m_mB * impulse, JvBD);
        aB += this.m_iB * impulse * JwB;
        // cC -= m_mC * impulse * JvAC;
        cC.SelfMulSub(this.m_mC * impulse, JvAC);
        aC -= this.m_iC * impulse * JwC;
        // cD -= m_mD * impulse * JvBD;
        cD.SelfMulSub(this.m_mD * impulse, JvBD);
        aD -= this.m_iD * impulse * JwD;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        // data.positions[this.m_indexC].c = cC;
        data.positions[this.m_indexC].a = aC;
        // data.positions[this.m_indexD].c = cD;
        data.positions[this.m_indexD].a = aD;
        // TODO_ERIN not implemented
        return linearError < b2Settings_1.b2_linearSlop;
    };
    b2GearJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2GearJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2GearJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // b2Vec2 P = m_impulse * m_JvAC;
        // return inv_dt * P;
        return b2Math_1.b2Vec2.MulSV(inv_dt * this.m_impulse, this.m_JvAC, out);
    };
    b2GearJoint.prototype.GetReactionTorque = function (inv_dt) {
        // float32 L = m_impulse * m_JwA;
        // return inv_dt * L;
        return inv_dt * this.m_impulse * this.m_JwA;
    };
    b2GearJoint.prototype.GetJoint1 = function () { return this.m_joint1; };
    b2GearJoint.prototype.GetJoint2 = function () { return this.m_joint2; };
    b2GearJoint.prototype.GetRatio = function () {
        return this.m_ratio;
    };
    b2GearJoint.prototype.SetRatio = function (ratio) {
        // DEBUG: b2Assert(b2IsValid(ratio));
        this.m_ratio = ratio;
    };
    b2GearJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        var index1 = this.m_joint1.m_index;
        var index2 = this.m_joint2.m_index;
        log("  const jd: b2GearJointDef = new b2GearJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.joint1 = joints[%d];\n", index1);
        log("  jd.joint2 = joints[%d];\n", index2);
        log("  jd.ratio = %.15f;\n", this.m_ratio);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2GearJoint.InitVelocityConstraints_s_u = new b2Math_1.b2Vec2();
    b2GearJoint.InitVelocityConstraints_s_rA = new b2Math_1.b2Vec2();
    b2GearJoint.InitVelocityConstraints_s_rB = new b2Math_1.b2Vec2();
    b2GearJoint.InitVelocityConstraints_s_rC = new b2Math_1.b2Vec2();
    b2GearJoint.InitVelocityConstraints_s_rD = new b2Math_1.b2Vec2();
    b2GearJoint.SolvePositionConstraints_s_u = new b2Math_1.b2Vec2();
    b2GearJoint.SolvePositionConstraints_s_rA = new b2Math_1.b2Vec2();
    b2GearJoint.SolvePositionConstraints_s_rB = new b2Math_1.b2Vec2();
    b2GearJoint.SolvePositionConstraints_s_rC = new b2Math_1.b2Vec2();
    b2GearJoint.SolvePositionConstraints_s_rD = new b2Math_1.b2Vec2();
    return b2GearJoint;
}(b2Joint_1.b2Joint));
exports.b2GearJoint = b2GearJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHZWFySm9pbnQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Kb2ludHMvYjJHZWFySm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7QUFFRiw2REFBNkQ7QUFDN0QsMERBQTBEO0FBQzFELHNEQUFpRTtBQUNqRSw4Q0FBcUU7QUFDckUscUNBQTBFO0FBYzFFLGdFQUFnRTtBQUNoRSw2REFBNkQ7QUFDN0Q7SUFBb0Msa0NBQVU7SUFPNUM7UUFBQSxZQUNFLGtCQUFNLHFCQUFXLENBQUMsV0FBVyxDQUFDLFNBQy9CO1FBSk0sV0FBSyxHQUFXLENBQUMsQ0FBQzs7SUFJekIsQ0FBQztJQUNILHFCQUFDO0FBQUQsQ0FBQyxBQVZELENBQW9DLG9CQUFVLEdBVTdDO0FBVlksd0NBQWM7QUFZM0I7SUFBaUMsK0JBQU87SUErRHRDLHFCQUFZLEdBQW9CO1FBQWhDLFlBQ0Usa0JBQU0sR0FBRyxDQUFDLFNBZ0dYO1FBNUpNLGFBQU8sR0FBZ0IscUJBQVcsQ0FBQyxjQUFjLENBQUM7UUFDbEQsYUFBTyxHQUFnQixxQkFBVyxDQUFDLGNBQWMsQ0FBQztRQU96RCxnQkFBZ0I7UUFDQSxvQkFBYyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDdEMsb0JBQWMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ3RDLG9CQUFjLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUN0QyxvQkFBYyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFFdEMsa0JBQVksR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ3BDLGtCQUFZLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUU3Qyx1QkFBaUIsR0FBVyxDQUFDLENBQUM7UUFDOUIsdUJBQWlCLEdBQVcsQ0FBQyxDQUFDO1FBRTlCLGdCQUFVLEdBQVcsQ0FBQyxDQUFDO1FBQ3ZC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LEdBQUcsSUFBSSxDQUFDLE9BQU8sR0FBRyxlQUFNLENBQUMsT0FBTyxDQUFDLEVBQUUsRUFBRSxDQUFDLENBQUMsQ0FBQztZQUNsRCxJQUFJLENBQUMsTUFBTSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLElBQUksQ0FBQyxJQUFJLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxHQUFHLElBQUksQ0FBQyxJQUFJLEdBQUcsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQyxJQUFJLEdBQUcsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUMsS0FBSyxDQUFDO1NBQ2xKO1FBRUQsMEJBQTBCO1FBQzFCLElBQUksQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsR0FBRyxJQUFJLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFcEQsSUFBSSxJQUFJLENBQUMsSUFBSSxDQUFDLFlBQVksRUFBRTtZQUMxQixxQ0FBcUM7WUFDckMsRUFBRSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxTQUFTLEVBQUUsSUFBSSxDQUFDLE1BQU0sQ0FBQyxDQUFDO1lBQ3ZELEVBQUUsSUFBSSxJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUM5QyxxQ0FBcUM7WUFDckMsRUFBRSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxTQUFTLEVBQUUsSUFBSSxDQUFDLE1BQU0sQ0FBQyxDQUFDO1lBQ3ZELEVBQUUsSUFBSSxJQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxDQUFDLEtBQUssQ0FBQztZQUM5QyxxQ0FBcUM7WUFDckMsRUFBRSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsSUFBSSxHQUFH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