"use strict";
/*
 * Copyright (c) 2013 Google, Inc.
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
Object.defineProperty(exports, "__esModule", { value: true });
// #if B2_ENABLE_PARTICLE
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Math_1 = require("../Common/b2Math");
var b2Draw_1 = require("../Common/b2Draw");
var b2ParticleGroupFlag;
(function (b2ParticleGroupFlag) {
    /// Prevents overlapping or leaking.
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_solidParticleGroup"] = 1] = "b2_solidParticleGroup";
    /// Keeps its shape.
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_rigidParticleGroup"] = 2] = "b2_rigidParticleGroup";
    /// Won't be destroyed if it gets empty.
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_particleGroupCanBeEmpty"] = 4] = "b2_particleGroupCanBeEmpty";
    /// Will be destroyed on next simulation step.
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_particleGroupWillBeDestroyed"] = 8] = "b2_particleGroupWillBeDestroyed";
    /// Updates depth data on next simulation step.
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_particleGroupNeedsUpdateDepth"] = 16] = "b2_particleGroupNeedsUpdateDepth";
    b2ParticleGroupFlag[b2ParticleGroupFlag["b2_particleGroupInternalMask"] = 24] = "b2_particleGroupInternalMask";
})(b2ParticleGroupFlag = exports.b2ParticleGroupFlag || (exports.b2ParticleGroupFlag = {}));
var b2ParticleGroupDef = /** @class */ (function () {
    function b2ParticleGroupDef() {
        this.flags = 0;
        this.groupFlags = 0;
        this.position = new b2Math_1.b2Vec2();
        this.angle = 0.0;
        this.linearVelocity = new b2Math_1.b2Vec2();
        this.angularVelocity = 0.0;
        this.color = new b2Draw_1.b2Color();
        this.strength = 1.0;
        this.shapeCount = 0;
        this.stride = 0;
        this.particleCount = 0;
        this.lifetime = 0;
        this.userData = null;
        this.group = null;
    }
    return b2ParticleGroupDef;
}());
exports.b2ParticleGroupDef = b2ParticleGroupDef;
var b2ParticleGroup = /** @class */ (function () {
    function b2ParticleGroup(system) {
        this.m_firstIndex = 0;
        this.m_lastIndex = 0;
        this.m_groupFlags = 0;
        this.m_strength = 1.0;
        this.m_prev = null;
        this.m_next = null;
        this.m_timestamp = -1;
        this.m_mass = 0.0;
        this.m_inertia = 0.0;
        this.m_center = new b2Math_1.b2Vec2();
        this.m_linearVelocity = new b2Math_1.b2Vec2();
        this.m_angularVelocity = 0.0;
        this.m_transform = new b2Math_1.b2Transform();
        ///m_transform.SetIdentity();
        this.m_userData = null;
        this.m_system = system;
    }
    b2ParticleGroup.prototype.GetNext = function () {
        return this.m_next;
    };
    b2ParticleGroup.prototype.GetParticleSystem = function () {
        return this.m_system;
    };
    b2ParticleGroup.prototype.GetParticleCount = function () {
        return this.m_lastIndex - this.m_firstIndex;
    };
    b2ParticleGroup.prototype.GetBufferIndex = function () {
        return this.m_firstIndex;
    };
    b2ParticleGroup.prototype.ContainsParticle = function (index) {
        return this.m_firstIndex <= index && index < this.m_lastIndex;
    };
    b2ParticleGroup.prototype.GetAllParticleFlags = function () {
        if (!this.m_system.m_flagsBuffer.data) {
            throw new Error();
        }
        var flags = 0;
        for (var i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            flags |= this.m_system.m_flagsBuffer.data[i];
        }
        return flags;
    };
    b2ParticleGroup.prototype.GetGroupFlags = function () {
        return this.m_groupFlags;
    };
    b2ParticleGroup.prototype.SetGroupFlags = function (flags) {
        // DEBUG: b2Assert((flags & b2ParticleGroupFlag.b2_particleGroupInternalMask) === 0);
        flags |= this.m_groupFlags & b2ParticleGroupFlag.b2_particleGroupInternalMask;
        this.m_system.SetGroupFlags(this, flags);
    };
    b2ParticleGroup.prototype.GetMass = function () {
        this.UpdateStatistics();
        return this.m_mass;
    };
    b2ParticleGroup.prototype.GetInertia = function () {
        this.UpdateStatistics();
        return this.m_inertia;
    };
    b2ParticleGroup.prototype.GetCenter = function () {
        this.UpdateStatistics();
        return this.m_center;
    };
    b2ParticleGroup.prototype.GetLinearVelocity = function () {
        this.UpdateStatistics();
        return this.m_linearVelocity;
    };
    b2ParticleGroup.prototype.GetAngularVelocity = function () {
        this.UpdateStatistics();
        return this.m_angularVelocity;
    };
    b2ParticleGroup.prototype.GetTransform = function () {
        return this.m_transform;
    };
    b2ParticleGroup.prototype.GetPosition = function () {
        return this.m_transform.p;
    };
    b2ParticleGroup.prototype.GetAngle = function () {
        return this.m_transform.q.GetAngle();
    };
    b2ParticleGroup.prototype.GetLinearVelocityFromWorldPoint = function (worldPoint, out) {
        var s_t0 = b2ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0;
        this.UpdateStatistics();
        ///  return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_center);
        return b2Math_1.b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Math_1.b2Vec2.SubVV(worldPoint, this.m_center, s_t0), out);
    };
    b2ParticleGroup.prototype.GetUserData = function () {
        return this.m_userData;
    };
    b2ParticleGroup.prototype.SetUserData = function (data) {
        this.m_userData = data;
    };
    b2ParticleGroup.prototype.ApplyForce = function (force) {
        this.m_system.ApplyForce(this.m_firstIndex, this.m_lastIndex, force);
    };
    b2ParticleGroup.prototype.ApplyLinearImpulse = function (impulse) {
        this.m_system.ApplyLinearImpulse(this.m_firstIndex, this.m_lastIndex, impulse);
    };
    b2ParticleGroup.prototype.DestroyParticles = function (callDestructionListener) {
        if (this.m_system.m_world.IsLocked()) {
            throw new Error();
        }
        for (var i = this.m_firstIndex; i < this.m_lastIndex; i++) {
            this.m_system.DestroyParticle(i, callDestructionListener);
        }
    };
    b2ParticleGroup.prototype.UpdateStatistics = function () {
        if (!this.m_system.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_system.m_velocityBuffer.data) {
            throw new Error();
        }
        var p = new b2Math_1.b2Vec2();
        var v = new b2Math_1.b2Vec2();
        if (this.m_timestamp !== this.m_system.m_timestamp) {
            var m = this.m_system.GetParticleMass();
            ///  this.m_mass = 0;
            this.m_mass = m * (this.m_lastIndex - this.m_firstIndex);
            this.m_center.SetZero();
            this.m_linearVelocity.SetZero();
            for (var i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///  this.m_mass += m;
                ///  this.m_center += m * this.m_system.m_positionBuffer.data[i];
                this.m_center.SelfMulAdd(m, this.m_system.m_positionBuffer.data[i]);
                ///  this.m_linearVelocity += m * this.m_system.m_velocityBuffer.data[i];
                this.m_linearVelocity.SelfMulAdd(m, this.m_system.m_velocityBuffer.data[i]);
            }
            if (this.m_mass > 0) {
                var inv_mass = 1 / this.m_mass;
                ///this.m_center *= 1 / this.m_mass;
                this.m_center.SelfMul(inv_mass);
                ///this.m_linearVelocity *= 1 / this.m_mass;
                this.m_linearVelocity.SelfMul(inv_mass);
            }
            this.m_inertia = 0;
            this.m_angularVelocity = 0;
            for (var i = this.m_firstIndex; i < this.m_lastIndex; i++) {
                ///b2Vec2 p = this.m_system.m_positionBuffer.data[i] - this.m_center;
                b2Math_1.b2Vec2.SubVV(this.m_system.m_positionBuffer.data[i], this.m_center, p);
                ///b2Vec2 v = this.m_system.m_velocityBuffer.data[i] - this.m_linearVelocity;
                b2Math_1.b2Vec2.SubVV(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity, v);
                this.m_inertia += m * b2Math_1.b2Vec2.DotVV(p, p);
                this.m_angularVelocity += m * b2Math_1.b2Vec2.CrossVV(p, v);
            }
            if (this.m_inertia > 0) {
                this.m_angularVelocity *= 1 / this.m_inertia;
            }
            this.m_timestamp = this.m_system.m_timestamp;
        }
    };
    b2ParticleGroup.GetLinearVelocityFromWorldPoint_s_t0 = new b2Math_1.b2Vec2();
    return b2ParticleGroup;
}());
exports.b2ParticleGroup = b2ParticleGroup;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQYXJ0aWNsZUdyb3VwLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvUGFydGljbGUvYjJQYXJ0aWNsZUdyb3VwLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztHQWdCRzs7QUFFSCx5QkFBeUI7QUFFekIsMERBQTBEO0FBQzFELDJDQUEyRDtBQUMzRCwyQ0FBaUQ7QUFLakQsSUFBWSxtQkFhWDtBQWJELFdBQVksbUJBQW1CO0lBQzdCLG9DQUFvQztJQUNwQywrRkFBOEIsQ0FBQTtJQUM5QixvQkFBb0I7SUFDcEIsK0ZBQThCLENBQUE7SUFDOUIsd0NBQXdDO0lBQ3hDLHlHQUFtQyxDQUFBO0lBQ25DLDhDQUE4QztJQUM5QyxtSEFBd0MsQ0FBQTtJQUN4QywrQ0FBK0M7SUFDL0Msc0hBQXlDLENBQUE7SUFFekMsOEdBQWlHLENBQUE7QUFDbkcsQ0FBQyxFQWJXLG1CQUFtQixHQUFuQiwyQkFBbUIsS0FBbkIsMkJBQW1CLFFBYTlCO0FBc0JEO0lBQUE7UUFDUyxVQUFLLEdBQW1CLENBQUMsQ0FBQztRQUMxQixlQUFVLEdBQXdCLENBQUMsQ0FBQztRQUMzQixhQUFRLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUN6QyxVQUFLLEdBQVcsR0FBRyxDQUFDO1FBQ1gsbUJBQWMsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQy9DLG9CQUFlLEdBQVcsR0FBRyxDQUFDO1FBQ3JCLFVBQUssR0FBWSxJQUFJLGdCQUFPLEVBQUUsQ0FBQztRQUN4QyxhQUFRLEdBQVcsR0FBRyxDQUFDO1FBR3ZCLGVBQVUsR0FBVyxDQUFDLENBQUM7UUFDdkIsV0FBTSxHQUFXLENBQUMsQ0FBQztRQUNuQixrQkFBYSxHQUFXLENBQUMsQ0FBQztRQUUxQixhQUFRLEdBQVcsQ0FBQyxDQUFDO1FBQ3JCLGFBQVEsR0FBUSxJQUFJLENBQUM7UUFDckIsVUFBSyxHQUEyQixJQUFJLENBQUM7SUFDOUMs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