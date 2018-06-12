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
// DEBUG: import { b2Assert, b2_epsilon } from "../../Common/b2Settings";
// DEBUG: import { b2IsValid } from "../../Common/b2Math";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
var b2MouseJointDef = /** @class */ (function (_super) {
    __extends(b2MouseJointDef, _super);
    function b2MouseJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_mouseJoint) || this;
        _this.target = new b2Math_1.b2Vec2();
        _this.maxForce = 0;
        _this.frequencyHz = 5;
        _this.dampingRatio = 0.7;
        return _this;
    }
    return b2MouseJointDef;
}(b2Joint_1.b2JointDef));
exports.b2MouseJointDef = b2MouseJointDef;
var b2MouseJoint = /** @class */ (function (_super) {
    __extends(b2MouseJoint, _super);
    function b2MouseJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_localAnchorB = new b2Math_1.b2Vec2();
        _this.m_targetA = new b2Math_1.b2Vec2();
        _this.m_frequencyHz = 0;
        _this.m_dampingRatio = 0;
        _this.m_beta = 0;
        // Solver shared
        _this.m_impulse = new b2Math_1.b2Vec2();
        _this.m_maxForce = 0;
        _this.m_gamma = 0;
        // Solver temp
        _this.m_indexA = 0;
        _this.m_indexB = 0;
        _this.m_rB = new b2Math_1.b2Vec2();
        _this.m_localCenterB = new b2Math_1.b2Vec2();
        _this.m_invMassB = 0;
        _this.m_invIB = 0;
        _this.m_mass = new b2Math_1.b2Mat22();
        _this.m_C = new b2Math_1.b2Vec2();
        _this.m_qB = new b2Math_1.b2Rot();
        _this.m_lalcB = new b2Math_1.b2Vec2();
        _this.m_K = new b2Math_1.b2Mat22();
        _this.m_targetA.Copy(b2Settings_1.b2Maybe(def.target, b2Math_1.b2Vec2.ZERO));
        // DEBUG: b2Assert(this.m_targetA.IsValid());
        b2Math_1.b2Transform.MulTXV(_this.m_bodyB.GetTransform(), _this.m_targetA, _this.m_localAnchorB);
        _this.m_maxForce = b2Settings_1.b2Maybe(def.maxForce, 0);
        // DEBUG: b2Assert(b2IsValid(this.m_maxForce) && this.m_maxForce >= 0);
        _this.m_impulse.SetZero();
        _this.m_frequencyHz = b2Settings_1.b2Maybe(def.frequencyHz, 0);
        // DEBUG: b2Assert(b2IsValid(this.m_frequencyHz) && this.m_frequencyHz >= 0);
        _this.m_dampingRatio = b2Settings_1.b2Maybe(def.dampingRatio, 0);
        // DEBUG: b2Assert(b2IsValid(this.m_dampingRatio) && this.m_dampingRatio >= 0);
        _this.m_beta = 0;
        _this.m_gamma = 0;
        return _this;
    }
    b2MouseJoint.prototype.SetTarget = function (target) {
        if (!this.m_bodyB.IsAwake()) {
            this.m_bodyB.SetAwake(true);
        }
        this.m_targetA.Copy(target);
    };
    b2MouseJoint.prototype.GetTarget = function () {
        return this.m_targetA;
    };
    b2MouseJoint.prototype.SetMaxForce = function (maxForce) {
        this.m_maxForce = maxForce;
    };
    b2MouseJoint.prototype.GetMaxForce = function () {
        return this.m_maxForce;
    };
    b2MouseJoint.prototype.SetFrequency = function (hz) {
        this.m_frequencyHz = hz;
    };
    b2MouseJoint.prototype.GetFrequency = function () {
        return this.m_frequencyHz;
    };
    b2MouseJoint.prototype.SetDampingRatio = function (ratio) {
        this.m_dampingRatio = ratio;
    };
    b2MouseJoint.prototype.GetDampingRatio = function () {
        return this.m_dampingRatio;
    };
    b2MouseJoint.prototype.InitVelocityConstraints = function (data) {
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIB = this.m_bodyB.m_invI;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var qB = this.m_qB.SetAngle(aB);
        var mass = this.m_bodyB.GetMass();
        // Frequency
        var omega = 2 * b2Settings_1.b2_pi * this.m_frequencyHz;
        // Damping coefficient
        var d = 2 * mass * this.m_dampingRatio * omega;
        // Spring stiffness
        var k = mass * (omega * omega);
        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        var h = data.step.dt;
        // DEBUG: b2Assert(d + h * k > b2_epsilon);
        this.m_gamma = h * (d + h * k);
        if (this.m_gamma !== 0) {
            this.m_gamma = 1 / this.m_gamma;
        }
        this.m_beta = h * k * this.m_gamma;
        // Compute the effective mass matrix.
        b2Math_1.b2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.b2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        var K = this.m_K;
        K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
        K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;
        K.GetInverse(this.m_mass);
        // m_C = cB + m_rB - m_targetA;
        this.m_C.x = cB.x + this.m_rB.x - this.m_targetA.x;
        this.m_C.y = cB.y + this.m_rB.y - this.m_targetA.y;
        // m_C *= m_beta;
        this.m_C.SelfMul(this.m_beta);
        // Cheat with some damping
        wB *= 0.98;
        if (data.step.warmStarting) {
            this.m_impulse.SelfMul(data.step.dtRatio);
            // vB += m_invMassB * m_impulse;
            vB.x += this.m_invMassB * this.m_impulse.x;
            vB.y += this.m_invMassB * this.m_impulse.y;
            wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, this.m_impulse);
        }
        else {
            this.m_impulse.SetZero();
        }
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2MouseJoint.prototype.SolveVelocityConstraints = function (data) {
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        // Cdot = v + cross(w, r)
        // b2Vec2 Cdot = vB + b2Cross(wB, m_rB);
        var Cdot = b2Math_1.b2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2MouseJoint.SolveVelocityConstraints_s_Cdot);
        //  b2Vec2 impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));
        var impulse = b2Math_1.b2Mat22.MulMV(this.m_mass, b2Math_1.b2Vec2.AddVV(Cdot, b2Math_1.b2Vec2.AddVV(this.m_C, b2Math_1.b2Vec2.MulSV(this.m_gamma, this.m_impulse, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0).SelfNeg(), b2MouseJoint.SolveVelocityConstraints_s_impulse);
        // b2Vec2 oldImpulse = m_impulse;
        var oldImpulse = b2MouseJoint.SolveVelocityConstraints_s_oldImpulse.Copy(this.m_impulse);
        // m_impulse += impulse;
        this.m_impulse.SelfAdd(impulse);
        var maxImpulse = data.step.dt * this.m_maxForce;
        if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.m_impulse.SelfMul(maxImpulse / this.m_impulse.Length());
        }
        // impulse = m_impulse - oldImpulse;
        b2Math_1.b2Vec2.SubVV(this.m_impulse, oldImpulse, impulse);
        // vB += m_invMassB * impulse;
        vB.SelfMulAdd(this.m_invMassB, impulse);
        wB += this.m_invIB * b2Math_1.b2Vec2.CrossVV(this.m_rB, impulse);
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    };
    b2MouseJoint.prototype.SolvePositionConstraints = function (data) {
        return true;
    };
    b2MouseJoint.prototype.GetAnchorA = function (out) {
        out.x = this.m_targetA.x;
        out.y = this.m_targetA.y;
        return out;
    };
    b2MouseJoint.prototype.GetAnchorB = function (out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    };
    b2MouseJoint.prototype.GetReactionForce = function (inv_dt, out) {
        return b2Math_1.b2Vec2.MulSV(inv_dt, this.m_impulse, out);
    };
    b2MouseJoint.prototype.GetReactionTorque = function (inv_dt) {
        return 0;
    };
    b2MouseJoint.prototype.Dump = function (log) {
        log("Mouse joint dumping is not supported.\n");
    };
    b2MouseJoint.prototype.ShiftOrigin = function (newOrigin) {
        this.m_targetA.SelfSub(newOrigin);
    };
    b2MouseJoint.SolveVelocityConstraints_s_Cdot = new b2Math_1.b2Vec2();
    b2MouseJoint.SolveVelocityConstraints_s_impulse = new b2Math_1.b2Vec2();
    b2MouseJoint.SolveVelocityConstraints_s_oldImpulse = new b2Math_1.b2Vec2();
    return b2MouseJoint;
}(b2Joint_1.b2Joint));
exports.b2MouseJoint = b2MouseJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJNb3VzZUpvaW50LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvSm9pbnRzL2IyTW91c2VKb2ludC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUVGLHlFQUF5RTtBQUN6RSwwREFBMEQ7QUFDMUQsc0RBQXlEO0FBQ3pELDhDQUE4RTtBQUM5RSxxQ0FBMEU7QUFhMUUsK0RBQStEO0FBQy9ELHlDQUF5QztBQUN6QztJQUFxQyxtQ0FBVTtJQVM3QztRQUFBLFlBQ0Usa0JBQU0scUJBQVcsQ0FBQyxZQUFZLENBQUMsU0FDaEM7UUFWZSxZQUFNLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUV2QyxjQUFRLEdBQVcsQ0FBQyxDQUFDO1FBRXJCLGlCQUFXLEdBQVcsQ0FBQyxDQUFDO1FBRXhCLGtCQUFZLEdBQVcsR0FBRyxDQUFDOztJQUlsQyxDQUFDO0lBQ0gsc0JBQUM7QUFBRCxDQUFDLEFBWkQsQ0FBcUMsb0JBQVUsR0FZOUM7QUFaWSwwQ0FBZTtBQWM1QjtJQUFrQyxnQ0FBTztJQXlCdkMsc0JBQVksR0FBcUI7UUFBakMsWUFDRSxrQkFBTSxHQUFHLENBQUMsU0FpQlg7UUExQ2Usb0JBQWMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ3RDLGVBQVMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQzFDLG1CQUFhLEdBQVcsQ0FBQyxDQUFDO1FBQzFCLG9CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBQzNCLFlBQU0sR0FBVyxDQUFDLENBQUM7UUFFMUIsZ0JBQWdCO1FBQ0EsZUFBUyxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDMUMsZ0JBQVUsR0FBVyxDQUFDLENBQUM7UUFDdkIsYUFBTyxHQUFXLENBQUMsQ0FBQztRQUUzQixjQUFjO1FBQ1AsY0FBUSxHQUFXLENBQUMsQ0FBQztRQUNyQixjQUFRLEdBQVcs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