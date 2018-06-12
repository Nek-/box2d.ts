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
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Draw_1 = require("../Common/b2Draw");
/**
 * The particle type. Can be combined with the | operator.
 */
var b2ParticleFlag;
(function (b2ParticleFlag) {
    /// Water particle.
    b2ParticleFlag[b2ParticleFlag["b2_waterParticle"] = 0] = "b2_waterParticle";
    /// Removed after next simulation step.
    b2ParticleFlag[b2ParticleFlag["b2_zombieParticle"] = 2] = "b2_zombieParticle";
    /// Zero velocity.
    b2ParticleFlag[b2ParticleFlag["b2_wallParticle"] = 4] = "b2_wallParticle";
    /// With restitution from stretching.
    b2ParticleFlag[b2ParticleFlag["b2_springParticle"] = 8] = "b2_springParticle";
    /// With restitution from deformation.
    b2ParticleFlag[b2ParticleFlag["b2_elasticParticle"] = 16] = "b2_elasticParticle";
    /// With viscosity.
    b2ParticleFlag[b2ParticleFlag["b2_viscousParticle"] = 32] = "b2_viscousParticle";
    /// Without isotropic pressure.
    b2ParticleFlag[b2ParticleFlag["b2_powderParticle"] = 64] = "b2_powderParticle";
    /// With surface tension.
    b2ParticleFlag[b2ParticleFlag["b2_tensileParticle"] = 128] = "b2_tensileParticle";
    /// Mix color between contacting particles.
    b2ParticleFlag[b2ParticleFlag["b2_colorMixingParticle"] = 256] = "b2_colorMixingParticle";
    /// Call b2DestructionListener on destruction.
    b2ParticleFlag[b2ParticleFlag["b2_destructionListenerParticle"] = 512] = "b2_destructionListenerParticle";
    /// Prevents other particles from leaking.
    b2ParticleFlag[b2ParticleFlag["b2_barrierParticle"] = 1024] = "b2_barrierParticle";
    /// Less compressibility.
    b2ParticleFlag[b2ParticleFlag["b2_staticPressureParticle"] = 2048] = "b2_staticPressureParticle";
    /// Makes pairs or triads with other particles.
    b2ParticleFlag[b2ParticleFlag["b2_reactiveParticle"] = 4096] = "b2_reactiveParticle";
    /// With high repulsive force.
    b2ParticleFlag[b2ParticleFlag["b2_repulsiveParticle"] = 8192] = "b2_repulsiveParticle";
    /// Call b2ContactListener when this particle is about to interact with
    /// a rigid body or stops interacting with a rigid body.
    /// This results in an expensive operation compared to using
    /// b2_fixtureContactFilterParticle to detect collisions between
    /// particles.
    b2ParticleFlag[b2ParticleFlag["b2_fixtureContactListenerParticle"] = 16384] = "b2_fixtureContactListenerParticle";
    /// Call b2ContactListener when this particle is about to interact with
    /// another particle or stops interacting with another particle.
    /// This results in an expensive operation compared to using
    /// b2_particleContactFilterParticle to detect collisions between
    /// particles.
    b2ParticleFlag[b2ParticleFlag["b2_particleContactListenerParticle"] = 32768] = "b2_particleContactListenerParticle";
    /// Call b2ContactFilter when this particle interacts with rigid bodies.
    b2ParticleFlag[b2ParticleFlag["b2_fixtureContactFilterParticle"] = 65536] = "b2_fixtureContactFilterParticle";
    /// Call b2ContactFilter when this particle interacts with other
    /// particles.
    b2ParticleFlag[b2ParticleFlag["b2_particleContactFilterParticle"] = 131072] = "b2_particleContactFilterParticle";
})(b2ParticleFlag = exports.b2ParticleFlag || (exports.b2ParticleFlag = {}));
var b2ParticleDef = /** @class */ (function () {
    function b2ParticleDef() {
        this.flags = 0;
        this.position = new b2Math_1.b2Vec2();
        this.velocity = new b2Math_1.b2Vec2();
        this.color = new b2Draw_1.b2Color(0, 0, 0, 0);
        this.lifetime = 0.0;
        this.userData = null;
        this.group = null;
    }
    return b2ParticleDef;
}());
exports.b2ParticleDef = b2ParticleDef;
function b2CalculateParticleIterations(gravity, radius, timeStep) {
    // In some situations you may want more particle iterations than this,
    // but to avoid excessive cycle cost, don't recommend more than this.
    var B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8;
    var B2_RADIUS_THRESHOLD = 0.01;
    var iterations = Math.ceil(Math.sqrt(gravity / (B2_RADIUS_THRESHOLD * radius)) * timeStep);
    return b2Math_1.b2Clamp(iterations, 1, B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS);
}
exports.b2CalculateParticleIterations = b2CalculateParticleIterations;
var b2ParticleHandle = /** @class */ (function () {
    function b2ParticleHandle() {
        this.m_index = b2Settings_1.b2_invalidParticleIndex;
    }
    b2ParticleHandle.prototype.GetIndex = function () { return this.m_index; };
    b2ParticleHandle.prototype.SetIndex = function (index) { this.m_index = index; };
    return b2ParticleHandle;
}());
exports.b2ParticleHandle = b2ParticleHandle;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQYXJ0aWNsZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL1BhcnRpY2xlL2IyUGFydGljbGUudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOztBQUVILHlCQUF5QjtBQUV6QixtREFBK0Q7QUFDL0QsMkNBQXVEO0FBQ3ZELDJDQUFpRDtBQUdqRDs7R0FFRztBQUNILElBQVksY0E4Q1g7QUE5Q0QsV0FBWSxjQUFjO0lBQ3hCLG1CQUFtQjtJQUNuQiwyRUFBb0IsQ0FBQTtJQUNwQix1Q0FBdUM7SUFDdkMsNkVBQTBCLENBQUE7SUFDMUIsa0JBQWtCO0lBQ2xCLHlFQUF3QixDQUFBO0lBQ3hCLHFDQUFxQztJQUNyQyw2RUFBMEIsQ0FBQTtJQUMxQixzQ0FBc0M7SUFDdEMsZ0ZBQTJCLENBQUE7SUFDM0IsbUJBQW1CO0lBQ25CLGdGQUEyQixDQUFBO0lBQzNCLCtCQUErQjtJQUMvQiw4RUFBMEIsQ0FBQTtJQUMxQix5QkFBeUI7SUFDekIsaUZBQTJCLENBQUE7SUFDM0IsMkNBQTJDO0lBQzNDLHlGQUErQixDQUFBO0lBQy9CLDhDQUE4QztJQUM5Qyx5R0FBdUMsQ0FBQTtJQUN2QywwQ0FBMEM7SUFDMUMsa0ZBQTRCLENBQUE7SUFDNUIseUJBQXlCO0lBQ3pCLGdHQUFtQyxDQUFBO0lBQ25DLCtDQUErQztJQUMvQyxvRkFBNkIsQ0FBQTtJQUM3Qiw4QkFBOEI7SUFDOUIsc0ZBQThCLENBQUE7SUFDOUIsdUVBQXVFO0lBQ3ZFLHdEQUF3RDtJQUN4RCw0REFBNEQ7SUFDNUQsZ0VBQWdFO0lBQ2hFLGNBQWM7SUFDZCxpSEFBMkMsQ0FBQTtJQUMzQyx1RUFBdUU7SUFDdkUsZ0VBQWdFO0lBQ2hFLDREQUE0RDtJQUM1RCxpRUFBaUU7SUFDakUsY0FBYztJQUNkLG1IQUE0QyxDQUFBO0lBQzVDLHdFQUF3RTtJQUN4RSw2R0FBeUMsQ0FBQTtJQUN6QyxnRUFBZ0U7SUFDaEUsY0FBYztJQUNkLGdIQUEwQyxDQUFBO0FBQzVDLENBQUMsRUE5Q1csY0FBYyxHQUFkLHNCQUFjLEtBQWQsc0JBQWMsUUE4Q3pCO0FBWUQ7SUFBQTtRQUNTLFVBQUssR0FBbUIsQ0FBQyxDQUFDO1FBQ2pCLGFBQVEsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2hDLGFBQVEsR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQ2hDLFVBQUssR0FBWSxJQUFJLGdCQUFPLENBQUMsQ0FBQyxFQUFFLENBQUMsRUFBRSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDbEQsYUFBUSxHQUFXLEdBQUcsQ0FBQztRQUN2QixhQUFRLEdBQVEsSUFBSSxDQUFDO1FBQ3JCLFVBQUssR0FBMkIsSUFBSSxDQUFDO0lBQzlDLENBQUM7SUFBRCxvQkFBQztBQUFELENBQUMsQUFSRCxJQVFDO0FBUlksc0NBQWE7QUFVMUIsdUNBQThDLE9BQWUsRUFBRSxNQUFjLEVBQUUsUUFBZ0I7SUFDN0Ysc0VBQXNFO0lBQ3RFLHFFQUFxRTtJQUNyRSxJQUFNLHNDQUFzQyxHQUFHLENBQUMsQ0FBQztJQUNqRCxJQUFNLG1CQUFtQixHQUFHLElBQUksQ0FBQztJQUNqQyxJQUFNLFVBQVUsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsbUJBQW1CLEdBQUcsTUFBTSxDQUFDLENBQUMsR0FBRyxRQUFRLENBQUMsQ0FBQztJQUM3RixPQUFPLGdCQUFPLENBQUMsVUFBVSxFQUFFLENBQUMsRUFBRSxzQ0FBc0MsQ0FBQyxDQUFDO0FBQ3hFLENBQUM7QUFQRCxzRUFPQztBQUVEO0lBQUE7UUFDUyxZQUFPLEdBQVcsb0NBQXVCLENBQUM7SUFHbkQsQ0FBQztJQUZRLG1DQUFRLEdBQWYsY0FBNEIsT0FBTyxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUMsQ0FBQztJQUMzQyxtQ0FBUSxHQUFmLFVBQWdCLEtBQWEsSUFBVSxJQUFJLENBQUMsT0FBTyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUM7SUFDaEUsdUJBQUM7QUFBRCxDQUFDLEFBSkQsSUFJQztBQUpZLDRDQUFnQjtBQU03QixTQUFTIn0=