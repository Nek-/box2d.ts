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
/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
var b2RopeJointDef = /** @class */ (function (_super) {
    __extends(b2RopeJointDef, _super);
    function b2RopeJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_ropeJoint) || this;
        _this.localAnchorA = new b2Math_1.b2Vec2(-1, 0);
        _this.localAnchorB = new b2Math_1.b2Vec2(1, 0);
        _this.maxLength = 0;
        return _this;
    }
    return b2RopeJointDef;
}(b2Joint_1.b2JointDef));
exports.b2RopeJointDef = b2RopeJointDef;
var b2RopeJoint = /** @class */ (function (_super) {
    __extends(b2RopeJoint, _super);
    function b2RopeJoint(def) {
        var _this = _super.call(this, def) || this;
        // Solver shared
        _this.m_localAnchorA = new b2Math_1.b2Vec2();
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_maxLength = 0;
        _this.m_length = 0;
        _this.m_impulse = 0;
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
        _this.m_state = b2Joint_1.b2LimitState.e_inactiveLimit;
        _this.m_qA = new b2Math_1.b2Rot();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcA = new b2Math_1.b2Vec2();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_localAnchorA.Copy(b2Settings_1.b2Maybe(def.localAnchorA, new b2Math_1.b2Vec2(-1, 0)));
        _this.m_localAnchorB.Copy(b2Settings_1.b2Maybe(def.localAnchorB, new b2Math_1.b2Vec2(1, 0)));
        _this.m_maxLength = b2Settings_1.b2Maybe(def.maxLength, 0);
        return _this;
    }
    b2RopeJoint.prototype.InitVelocityConstraints = function (data) {
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
        // this.m_rA = b2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // this.m_rB = b2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // this.m_u = cB + this.m_rB - cA - this.m_rA;
        this.m_u.Copy(cB).SelfAdd(this.m_rB).SelfSub(cA).SelfSub(this.m_rA);
        this.m_length = this.m_u.Length();
        var C = this.m_length - this.m_maxLength;
        if (C > 0) {
            this.m_state = b2Joint_1.b2LimitState.e_atUpperLimit;
        }
        else {
            this.m_state = b2Joint_1.b2LimitState.e_inactiveLimit;
        }
        if (this.m_length > b2Settings_1.b2_linearSlop) {
            this.m_u.SelfMul(1 / this.m_length);
        }
        else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            return;
        }
        // Compute effective mass.
        var crA = b2Math_1.b2Vec2.CrossVV(this.m_rA, this.m_u);
        var crB = b2Math_1.b2Vec2.CrossVV(this.m_rB, this.m_u);
        var invMass = this.m_invMassA + this.m_invIA * crA * crA + this.m_invMassB + this.m_invIB * crB * crB;
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            // b2Vec2 P = m_impulse * m_u;
            var P = b2Math_1.b2Vec2.MulSV(this.m_impulse, this.m_u, b2RopeJoint.InitVelocityConstraints_s_P);
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, P);
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
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
    b2RopeJoint.prototype.SolveVelocityConstraints = function (data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // Cdot = dot(u, v + cross(w, r))
        // b2Vec2 vpA = vA + b2Cross(wA, m_rA);
        var vpA = b2Math_1.b2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2RopeJoint.SolveVelocityConstraints_s_vpA);
        // b2Vec2 vpB = vB + b2Cross(wB, m_rB);
        var vpB = b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2RopeJoint.SolveVelocityConstraints_s_vpB);
        // float32 C = m_length - m_maxLength;
        var C = this.m_length - this.m_maxLength;
        // float32 Cdot = b2Dot(m_u, vpB - vpA);
        var Cdot = b2Math_1.b2Vec2.DotVV(this.m_u, b2Math_1.b2Vec2.SubVV(vpB, vpA, b2Math_1.b2Vec2.s_t0));
        // Predictive constraint.
        if (C < 0) {
            Cdot += data.step.inv_dt * C;
        }
        var impulse = -this.m_mass * Cdot;
        var oldImpulse = this.m_impulse;
        this.m_impulse = b2Math_1.b2Min(0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;
        // b2Vec2 P = impulse * m_u;
        var P = b2Math_1.b2Vec2.MulSV(impulse, this.m_u, b2RopeJoint.SolveVelocityConstraints_s_P);
        // vA -= m_invMassA * P;
        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(this.m_rA, P);
        // vB += m_invMassB * P;
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, P);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2RopeJoint.prototype.SolvePositionConstraints = function (data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // b2Vec2 rA = b2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        var rA = b2Math_1.b2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // b2Vec2 rB = b2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        var rB = b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // b2Vec2 u = cB + rB - cA - rA;
        var u = this.m_u.Copy(cB).SelfAdd(rB).SelfSub(cA).SelfSub(rA);
        var length = u.Normalize();
        var C = length - this.m_maxLength;
        C = b2Math_1.b2Clamp(C, 0, b2Settings_1.b2_maxLinearCorrection);
        var impulse = -this.m_mass * C;
        // b2Vec2 P = impulse * u;
        var P = b2Math_1.b2Vec2.MulSV(impulse, u, b2RopeJoint.SolvePositionConstraints_s_P);
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * b2Math_1.b2Vec2.CrossVV(rA, P);
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(rB, P);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return length - this.m_maxLength < b2Settings_1.b2_linearSlop;
    };
    b2RopeJoint.prototype.GetAnchorA = function (out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    };
    b2RopeJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2RopeJoint.prototype.GetReactionForce = function (inv_dt, out) {
        // return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
        return b2Math_1.b2Vec2.MulSV((inv_dt * this.m_impulse), this.m_u, out);
    };
    b2RopeJoint.prototype.GetReactionTorque = function (inv_dt) {
        return 0;
    };
    b2RopeJoint.prototype.GetLocalAnchorA = function () { return this.m_localAnchorA; };
    b2RopeJoint.prototype.GetLocalAnchorB = function () { return this.m_localAnchorB; };
    b2RopeJoint.prototype.SetMaxLength = function (length) { this.m_maxLength = length; };
    b2RopeJoint.prototype.GetMaxLength = function () {
        return this.m_maxLength;
    };
    b2RopeJoint.prototype.GetLimitState = function () {
        return this.m_state;
    };
    b2RopeJoint.prototype.Dump = function (log) {
        var indexA = this.m_bodyA.m_islandIndex;
        var indexB = this.m_bodyB.m_islandIndex;
        log("  const jd: b2RopeJointDef = new b2RopeJointDef();\n");
        log("  jd.bodyA = bodies[%d];\n", indexA);
        log("  jd.bodyB = bodies[%d];\n", indexB);
        log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
        log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
        log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
        log("  jd.maxLength = %.15f;\n", this.m_maxLength);
        log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
    };
    b2RopeJoint.InitVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2RopeJoint.SolveVelocityConstraints_s_vpA = new b2Math_1.b2Vec2();
    b2RopeJoint.SolveVelocityConstraints_s_vpB = new b2Math_1.b2Vec2();
    b2RopeJoint.SolveVelocityConstraints_s_P = new b2Math_1.b2Vec2();
    b2RopeJoint.SolvePositionConstraints_s_P = new b2Math_1.b2Vec2();
    return b2RopeJoint;
}(b2Joint_1.b2Joint));
exports.b2RopeJoint = b2RopeJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJSb3BlSm9pbnQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Kb2ludHMvYjJSb3BlSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7QUFFRixzREFBeUY7QUFDekYsOENBQXdFO0FBQ3hFLHFDQUF3RjtBQVd4RixtRUFBbUU7QUFDbkUsc0JBQXNCO0FBQ3RCLDREQUE0RDtBQUM1RCx1Q0FBdUM7QUFDdkM7SUFBb0Msa0NBQVU7SUFPNUM7UUFBQSxZQUNFLGtCQUFNLHFCQUFXLENBQUMsV0FBVyxDQUFDLFNBQy9CO1FBUmUsa0JBQVksR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUV6QyxrQkFBWSxHQUFXLElBQUksZUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUVqRCxlQUFTLEdBQVcsQ0FBQyxDQUFDOztJQUk3QixDQUFDO0lBQ0gscUJBQUM7QUFBRCxDQUFDLEFBVkQsQ0FBb0Msb0JBQVUsR0FVN0M7QUFWWSx3Q0FBYztBQVkzQjtJQUFpQywrQkFBTztJQTRCdEMscUJBQVksR0FBb0I7UUFBaEMsWUFDRSxrQkFBTSxHQUFHLENBQUMsU0FLWDtRQWpDRCxnQkFBZ0I7UUFDQSxvQkFBYyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDdEMsb0JBQWMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQy9DLGlCQUFXLEdBQVcsQ0FBQyxDQUFDO1FBQ3hCLGNBQVEsR0FBVyxDQUFDLENBQUM7UUFDckIsZUFBUyxHQUFXLENBQUMsQ0FBQztRQUU3QixjQUFjO1FBQ1AsY0FBUSxHQUFXLENBQUMsQ0FBQztRQUNyQixjQUFRLEdBQVcsQ0FBQyxDQUFDO1FBQ1osU0FBRyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDM0IsVUFBSSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDNUIsVUFBSSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDNUIsb0JBQWMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ3RDLG9CQUFjLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUMvQyxnQkFBVSxHQUFXLENBQUMsQ0FBQztRQUN2QixnQkFBVSxHQUFXLENBQUMsQ0FBQztRQUN2QixhQUFPLEdBQVcsQ0FBQyxDQUFDO1FBQ3BCLGFBQU8sR0FBVyxDQUFDLENBQUM7UUFDcEIsWUFBTSxHQUFXLENBQUMsQ0FBQztRQUNuQixhQUFPLEdBQUcsc0JBQVksQ0FBQyxlQUFlLENBQUM7UUFFOUIsVUFBSSxHQUFVLElBQUksY0FBSyxFQUFFLENBQUM7UUFDMUIsVUFBSSxHQUFVLElBQUksY0FBSyxFQUFFLENBQUM7UUFDMUIsYUFBTyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDL0IsYUFBTyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFLN0MsS0FBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsb0JBQU8sQ0FBQyxHQUFHLENBQUMsWUFBWSxFQUFFLElBQUksZUFBTSxDQUFDLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN2RSxLQUFJLENBQUMsY0FBYyxDQUFDLElBQUksQ0FBQyxvQkFBTyxDQUFDLEdBQUcsQ0FBQyxZQUFZLEVBQUUsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUN0RSxLQUFJLENBQUMsV0FBVyxHQUFHLG9CQUFPLENBQUMsR0FBRyxDQUFDLFNBQVMsRUFBRSxDQUFDLENBQUMsQ0FBQzs7SUFDL0MsQ0FBQztJQUdNLDZDQUF1QixHQUE5QixVQUErQixJQUFrQjtRQUMvQyxJQUFJLENBQUMsUUFBUSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsYUFBYSxDQUFDO1FBQzNDLElBQUksQ0FBQyxRQUFRLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxhQUFhLENBQUM7UUFDM0MsSUFBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsV0FBVyxDQUFDLENBQUM7UUFDM0QsSUFBSSxDQUFDLGNBQWMsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsV0FBVyxDQUFDLENBQUM7UUFDM0QsSUFBSSxDQUFDLFVBQVUsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLFNBQVMsQ0FBQztRQUN6QyxJQUFJLENBQUMsVUFBVSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsU0FBUyxDQUFDO1FBQ3pDLElBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxNQUFNLENBQUM7UUFDbkMsSUFBSSxDQUFDLE9BQU8sR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLE1BQU0sQ0FBQztRQUVuQyxJQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbkQsSUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ25ELElBQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNwRCxJQUFJLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFFbEQsSUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ25ELElBQU0sRUFBRSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNuRCxJQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBRWxELElBQU0sRUFBRSxHQUFVLElBQUksQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLEVBQUUsQ0FBQyxFQUFFLEVBQUUsR0FBVSxJQUFJLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUU3RSxvRUFBb0U7UUFDcEUsZUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3JFLGNBQUssQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLElBQUksQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBQ3pDLG9FQUFvRTtRQUNwRSxlQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLGNBQWMsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7UUFDckUsY0FBSyxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsSUFBSSxDQUFDLE9BQU8sRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFDekMsOENBQThDO1FBQzlDLElBQUksQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUMsT0FBTyxDQUFDLEVBQUUsQ0FBQyxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFFcEUsSUFBSSxDQUFDLFFBQVEsR0FBRyxJQUFJLENBQUMsR0FBRyxDQUFDLE1BQU0sRUFBRSxDQUFDO1FBRWxDLElBQU0sQ0FBQyxHQUFXLElBQUksQ0FBQyxRQUFRLEdBQUcsSUFBSSxDQUFDLFdBQVcsQ0FBQztRQUNuRCxJQUFJLENBQUMsR0FBRyxDQUFDLEVBQUU7WUFDVCxJQUFJLENBQUMsT0FBTyxHQUFHLHNCQUFZLENBQUMsY0FBYyxDQUFDO1NBQzVDO2FBQU07WUFDTCxJQUFJLENBQUMsT0FBTyxHQUFHLHNCQUFZLENBQUMsZUFBZSxDQUFDO1NBQzdDO1FBRUQsSUFBSSxJQUFJLENBQUMsUUFBUSxHQUFHLDBCQUFhLEVBQUU7WUFDakMsSUFBSSxDQUFDLEdBQUcsQ0FBQyxPQUFPLENBQUMsQ0FBQyxHQUFHLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQztTQUNyQzthQUFNO1lBQ0wsSUFBSSxDQUFDLEdBQUcsQ0FBQyxPQUFPLEVBQUUsQ0FBQztZQUNuQixJQUFJLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztZQUNoQixJQUFJLENBQUMsU0FBUyxHQUFHLENBQUMsQ0FBQztZQUNuQixPQUFPO1NBQ1I7UUFFRCwwQkFBMEI7UUFDMUIsSUFBTSxHQUFHLEdBQVcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLElBQUksQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUN4RCxJQUFNLEdBQUcsR0FBVyxlQUFNLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxJQUFJLEVBQUUsSUFBSSxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQ3hELElBQU0sT0FBTyxHQUFXLElBQUksQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDLE9BQU8sR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLElBQUksQ0FBQyxVQUFVLEdBQUcsSUFBSSxDQUFDLE9BQU8sR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDO1FBRWhILElBQUksQ0FBQyxNQUFNLEdBQUcsT0FBTyxLQUFLLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxHQUFHLE9BQU8sQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBRTlDLElBQUksSUFBSSxDQUFDLElBQUksQ0FBQyxZQUFZLEVBQUU7WUFDMUIscURBQXFEO1lBQ3JELElBQUksQ0FBQyxTQUFTLElBQUksSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUM7WUFFcEMsOEJBQThCO1lBQzlCLElBQU0sQ0FBQyxHQUFXLGVBQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLFNBQVMsRUFBRSxJQUFJLENBQUMsR0FBRyxFQUFFLFdBQVcsQ0FBQywyQkFBMkIsQ0FBQyxDQUFDO1lBQ2xHLHdCQUF3QjtZQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxDQUFDO1lBQ2xELHdCQUF3QjtZQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7WUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxDQUFDO1NBQ25EO2FBQU07WUFDTCxJQUFJLENBQUMsU0FBUyxHQUFHLENBQUMsQ0FBQztTQUNwQjtRQUVELHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO1FBQ3RDLHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO0lBQ3hDLENBQUM7SUFLTSw4Q0FBd0IsR0FBL0IsVUFBZ0MsSUFBa0I7UUFDaEQsSUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ3BELElBQUksRUFBRSxHQUFXLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNsRCxJQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDcEQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBRWxELGlDQUFpQztRQUNqQyx1Q0FBdUM7UUFDdkMsSUFBTSxHQUFHLEdBQVcsZUFBTSxDQUFDLFdBQVcsQ0FBQyxFQUFFLEVBQUUsRUFBRSxFQUFFLElBQUksQ0FBQyxJQUFJLEVBQUUsV0FBVyxDQUFDLDhCQUE4QixDQUFDLENBQUM7UUFDdEcsdUNBQXVDO1FBQ3ZDLElBQU0sR0FBRyxHQUFXLGVBQU0sQ0FBQyxXQUFXLENBQUMsRUFBRSxFQUFFLEVBQUUsRUFBRSxJQUFJLENBQUMsSUFBSSxFQUFFLFdBQVcsQ0FBQyw4QkFBOEIsQ0FBQyxDQUFDO1FBQ3RHLHNDQUFzQztRQUN0QyxJQUFNLENBQUMsR0FBVyxJQUFJLENBQUMsUUFBUSxHQUFHLElBQUksQ0FBQyxXQUFXLENBQUM7UUFDbkQsd0NBQXdDO1FBQ3hDLElBQUksSUFBSSxHQUFXLGVBQU0sQ0FBQyxLQUFLLENBQUMsSUFBSSxDQUFDLEdBQUcsRUFBRSxlQUFNLENBQUMsS0FBSyxDQUFDLEdBQUcsRUFBRSxHQUFHLEVBQUUsZUFBTSxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUM7UUFFL0UseUJBQXlCO1FBQ3pCLElBQUksQ0FBQyxHQUFHLENBQUMsRUFBRTtZQUNULElBQUksSUFBSSxJQUFJLENBQUMsSUFBSSxDQUFDLE1BQU0sR0FBRyxDQUFDLENBQUM7U0FDOUI7UUFFRCxJQUFJLE9BQU8sR0FBVyxDQUFDLElBQUksQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDO1FBQzFDLElBQU0sVUFBVSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUM7UUFDMUMsSUFBSSxDQUFDLFNBQVMsR0FBRyxjQUFLLENBQUMsQ0FBQyxFQUFFLElBQUksQ0FBQyxTQUFTLEdBQUcsT0FBTyxDQUFDLENBQUM7UUFDcEQsT0FBTyxHQUFHLElBQUksQ0FBQyxTQUFTLEdBQUcsVUFBVSxDQUFDO1FBRXRDLDRCQUE0QjtRQUM1QixJQUFNLENBQUMsR0FBVyxlQUFNLENBQUMsS0FBSyxDQUFDLE9BQU8sRUFBRSxJQUFJLENBQUMsR0FBRyxFQUFFLFdBQVcsQ0FBQyw0QkFBNEIsQ0FBQyxDQUFDO1FBQzVGLHdCQUF3QjtRQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ2xELHdCQUF3QjtRQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsSUFBSSxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRWxELHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO1FBQ3RDLHlDQUF5QztRQUN6QyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO0lBQ3hDLENBQUM7SUFHTSw4Q0FBd0IsR0FBL0IsVUFBZ0MsSUFBa0I7UUFDaEQsSUFBTSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBQ25ELElBQUksRUFBRSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNqRCxJQUFNLEVBQUUsR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDbkQsSUFBSSxFQUFFLEdBQVcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO1FBRWpELElBQU0sRUFBRSxHQUFVLElBQUksQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLEVBQUUsQ0FBQyxFQUFFLEVBQUUsR0FBVSxJQUFJLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUU3RSxvRUFBb0U7UUFDcEUsZUFBTSxDQUFDLEtBQUssQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3JFLElBQU0sRUFBRSxHQUFXLGNBQUssQ0FBQyxLQUFLLENBQUMsRUFBRSxFQUFFLElBQUksQ0FBQyxPQUFPLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO1FBQzVELG9FQUFvRTtRQUNwRSxlQUFNLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLGNBQWMsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7UUFDckUsSUFBTSxFQUFFLEdBQVcsY0FBSyxDQUFDLEtBQUssQ0FBQyxFQUFFLEVBQUUsSUFBSSxDQUFDLE9BQU8sRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFDNUQsZ0NBQWdDO1FBQ2hDLElBQU0sQ0FBQyxHQUFXLElBQUksQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLE9BQU8sQ0FBQyxFQUFFLENBQUMsQ0FBQyxPQUFPLENBQUMsRUFBRSxDQUFDLENBQUMsT0FBTyxDQUFDLEVBQUUsQ0FBQyxDQUFDO1FBRXhFLElBQU0sTUFBTSxHQUFXLENBQUMsQ0FBQyxTQUFTLEVBQUUsQ0FBQztRQUNyQyxJQUFJLENBQUMsR0FBVyxNQUFNLEdBQUcsSUFBSSxDQUFDLFdBQVcsQ0FBQztRQUUxQyxDQUFDLEdBQUcsZ0JBQU8sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxFQUFFLG1DQUFzQixDQUFDLENBQUM7UUFFMUMsSUFBTSxPQUFPLEdBQVcsQ0FBQyxJQUFJLENBQUMsTUFBTSxHQUFHLENBQUMsQ0FBQztRQUN6QywwQkFBMEI7UUFDMUIsSUFBTSxDQUFDLEdBQVcsZUFBTSxDQUFDLEtBQUssQ0FBQyxPQUFPLEVBQUUsQ0FBQyxFQUFFLFdBQVcsQ0FBQyw0QkFBNEIsQ0FBQyxDQUFDO1FBRXJGLHdCQUF3QjtRQUN4QixFQUFFLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxVQUFVLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDbEMsRUFBRSxJQUFJLElBQUksQ0FBQyxPQUFPLEdBQUcsZUFBTSxDQUFDLE9BQU8sQ0FBQyxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDM0Msd0JBQXdCO1FBQ3hCLEVBQUUsQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLFVBQVUsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNsQyxFQUFFLElBQUksSUFBSSxDQUFDLE9BQU8sR0FBRyxlQUFNLENBQUMsT0FBTyxDQUFDLEVBQUUsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUUzQyx3Q0FBd0M7UUFDeEMsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQztRQUNyQyx3Q0FBd0M7UUFDeEMsSUFBSSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQztRQUVyQyxPQUFPLE1BQU0sR0FBRyxJQUFJLENBQUMsV0FBVyxHQUFHLDBCQUFhLENBQUM7SUFDbkQsQ0FBQztJQUVNLGdDQUFVLEdBQWpCLFVBQWdDLEdBQU07UUFDcEMsT0FBTyxJQUFJLENBQUMsT0FBTyxDQUFDLGFBQWEsQ0FBQyxJQUFJLENBQUMsY0FBYyxFQUFFLEdBQUcsQ0FBQyxDQUFDO0lBQzlELENBQUM7SUFFTSxnQ0FBVSxHQUFqQixVQUFnQyxHQUFNO1FBQ3BDLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQyxhQUFhLENBQUMsSUFBSSxDQUFDLGNBQWMsRUFBRSxHQUFHLENBQUMsQ0FBQztJQUM5RCxDQUFDO0lBRU0sc0NBQWdCLEdBQXZCLFVBQXNDLE1BQWMsRUFBRSxHQUFNO1FBQzFELG9GQUFvRjtRQUNwRixPQUFPLGVBQU0sQ0FBQyxLQUFLLENBQUMsQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxFQUFFLElBQUksQ0FBQyxHQUFHLEVBQUUsR0FBRyxDQUFDLENBQUM7SUFDaEUsQ0FBQztJQUVNLHVDQUFpQixHQUF4QixVQUF5QixNQUFjO1FBQ3JDLE9BQU8sQ0FBQyxDQUFDO0lBQ1gsQ0FBQztJQUVNLHFDQUFlLEdBQXRCLGNBQTZDLE9BQU8sSUFBSSxDQUFDLGNBQWMsQ0FBQyxDQUFDLENBQUM7SUFFbkUscUNBQWUsR0FBdEIsY0FBNkMsT0FBTyxJQUFJLENBQUMsY0FBYyxDQUFDLENBQUMsQ0FBQztJQUVuRSxrQ0FBWSxHQUFuQixVQUFvQixNQUFjLElBQVUsSUFBSSxDQUFDLFdBQVcsR0FBRyxNQUFNLENBQUMsQ0FBQyxDQUFDO0lBQ2pFLGtDQUFZLEdBQW5CO1FBQ0UsT0FBTyxJQUFJLENBQUMsV0FBVyxDQUFDO0lBQzFCLENBQUM7SUFFTSxtQ0FBYSxHQUFwQjtRQUNFLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQztJQUN0QixDQUFDO0lBRU0sMEJBQUksR0FBWCxVQUFZLEdBQTZDO1FBQ3ZELElBQU0sTUFBTSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsYUFBYSxDQUFDO1FBQzFDLElBQU0sTUFBTSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsYUFBYSxDQUFDO1FBRTFDLEdBQUcsQ0FBQyxzREFBc0QsQ0FBQyxDQUFDO1FBQzVELEdBQUcsQ0FBQyw0QkFBNEIsRUFBRSxNQUFNLENBQUMsQ0FBQztRQUMxQyxHQUFHLENBQUMsNEJBQTRCLEVBQUUsTUFBTSxDQUFDLENBQUM7UUFDMUMsR0FBRyxDQUFDLCtCQUErQixFQUFFLENBQUMsSUFBSSxDQUFDLGtCQUFrQixDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsT0FBTyxDQUFDLENBQUMsQ0FBQztRQUN2RixHQUFHLENBQUMsd0NBQXdDLEVBQUUsSUFBSSxDQUFDLGNBQWMsQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLGNBQWMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUM1RixHQUFHLENBQUMsd0NBQXdDLEVBQUUsSUFBSSxDQUFDLGNBQWMsQ0FBQyxDQUFDLEVBQUUsSUFBSSxDQUFDLGNBQWMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUM1RixHQUFHLENBQUMsMkJBQTJCLEVBQUUsSUFBSSxDQUFDLFdBQVcsQ0FBQyxDQUFDO1FBQ25ELEdBQUcsQ0FBQyxnREFBZ0QsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7SUFDdEUsQ0FBQztJQS9NYyx1Q0FBMkIsR0FBRyxJQUFJLGVBQU0sRUFBRSxDQUFDO0lBK0UzQywwQ0FBOEIsR0FBRyxJQUFJLGVBQU0sRUFBRSxDQUFDO0lBQzlDLDBDQUE4QixHQUFHLElBQUksZUFBTSxFQUFFLENBQUM7SUFDOUMsd0NBQTRCLEdBQUcsSUFBSSxlQUFNLEVBQUUsQ0FBQztJQTBDNUMsd0NBQTRCLEdBQUcsSUFBSSxlQUFNLEVBQUUsQ0FBQztJQXFGN0Qsa0JBQUM7Q0FBQSxBQXBQRCxDQUFpQyxpQkFBTyxHQW9QdkM7QUFwUFksa0NBQVcifQ==