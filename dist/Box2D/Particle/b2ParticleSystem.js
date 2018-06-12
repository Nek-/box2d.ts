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
// #if B2_ENABLE_PARTICLE
// DEBUG: import { b2Assert, b2_maxParticleIndex } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Settings_2 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Draw_1 = require("../Common/b2Draw");
var b2Collision_1 = require("../Collision/b2Collision");
var b2Shape_1 = require("../Collision/Shapes/b2Shape");
var b2EdgeShape_1 = require("../Collision/Shapes/b2EdgeShape");
var b2TimeStep_1 = require("../Dynamics/b2TimeStep");
var b2WorldCallbacks_1 = require("../Dynamics/b2WorldCallbacks");
var b2Particle_1 = require("./b2Particle");
var b2ParticleGroup_1 = require("./b2ParticleGroup");
var b2VoronoiDiagram_1 = require("./b2VoronoiDiagram");
function std_iter_swap(array, a, b) {
    var tmp = array[a];
    array[a] = array[b];
    array[b] = tmp;
}
function default_compare(a, b) { return a < b; }
function std_sort(array, first, len, cmp) {
    if (first === void 0) { first = 0; }
    if (len === void 0) { len = array.length - first; }
    if (cmp === void 0) { cmp = default_compare; }
    var left = first;
    var stack = [];
    var pos = 0;
    for (;;) { /* outer loop */
        for (; left + 1 < len; len++) { /* sort left to len-1 */
            var pivot = array[left + Math.floor(Math.random() * (len - left))]; /* pick random pivot */
            stack[pos++] = len; /* sort right part later */
            for (var right = left - 1;;) { /* inner loop: partitioning */
                while (cmp(array[++right], pivot)) { } /* look for greater element */
                while (cmp(pivot, array[--len])) { } /* look for smaller element */
                if (right >= len) {
                    break;
                } /* partition point found? */
                std_iter_swap(array, right, len); /* the only swap */
            } /* partitioned, continue left part */
        }
        if (pos === 0) {
            break;
        } /* stack empty? */
        left = len; /* left to right is sorted */
        len = stack[--pos]; /* get next range to sort */
    }
    return array;
}
function std_stable_sort(array, first, len, cmp) {
    if (first === void 0) { first = 0; }
    if (len === void 0) { len = array.length - first; }
    if (cmp === void 0) { cmp = default_compare; }
    return std_sort(array, first, len, cmp);
}
function std_remove_if(array, predicate, length) {
    if (length === void 0) { length = array.length; }
    var l = 0;
    for (var c = 0; c < length; ++c) {
        // if we can be collapsed, keep l where it is.
        if (predicate(array[c])) {
            continue;
        }
        // this node can't be collapsed; push it back as far as we can.
        if (c === l) {
            ++l;
            continue; // quick exit if we're already in the right spot
        }
        // array[l++] = array[c];
        std_iter_swap(array, l++, c);
    }
    return l;
}
function std_lower_bound(array, first, last, val, cmp) {
    if (cmp === void 0) { cmp = default_compare; }
    var count = last - first;
    while (count > 0) {
        var step = Math.floor(count / 2);
        var it = first + step;
        if (cmp(array[it], val)) {
            first = ++it;
            count -= step + 1;
        }
        else {
            count = step;
        }
    }
    return first;
}
function std_upper_bound(array, first, last, val, cmp) {
    if (cmp === void 0) { cmp = default_compare; }
    var count = last - first;
    while (count > 0) {
        var step = Math.floor(count / 2);
        var it = first + step;
        if (!cmp(val, array[it])) {
            first = ++it;
            count -= step + 1;
        }
        else {
            count = step;
        }
    }
    return first;
}
function std_rotate(array, first, n_first, last) {
    var next = n_first;
    while (first !== next) {
        std_iter_swap(array, first++, next++);
        if (next === last) {
            next = n_first;
        }
        else if (first === n_first) {
            n_first = next;
        }
    }
}
function std_unique(array, first, last, cmp) {
    if (first === last) {
        return last;
    }
    var result = first;
    while (++first !== last) {
        if (!cmp(array[result], array[first])) {
            ///array[++result] = array[first];
            std_iter_swap(array, ++result, first);
        }
    }
    return ++result;
}
var b2GrowableBuffer = /** @class */ (function () {
    function b2GrowableBuffer(allocator) {
        this.data = [];
        this.count = 0;
        this.capacity = 0;
        this.allocator = allocator;
    }
    b2GrowableBuffer.prototype.Append = function () {
        if (this.count >= this.capacity) {
            this.Grow();
        }
        return this.count++;
    };
    b2GrowableBuffer.prototype.Reserve = function (newCapacity) {
        if (this.capacity >= newCapacity) {
            return;
        }
        // DEBUG: b2Assert(this.capacity === this.data.length);
        for (var i = this.capacity; i < newCapacity; ++i) {
            this.data[i] = this.allocator();
        }
        this.capacity = newCapacity;
    };
    b2GrowableBuffer.prototype.Grow = function () {
        // Double the capacity.
        var newCapacity = this.capacity ? 2 * this.capacity : b2Settings_1.b2_minParticleSystemBufferCapacity;
        // DEBUG: b2Assert(newCapacity > this.capacity);
        this.Reserve(newCapacity);
    };
    b2GrowableBuffer.prototype.Free = function () {
        if (this.data.length === 0) {
            return;
        }
        this.data = [];
        this.capacity = 0;
        this.count = 0;
    };
    b2GrowableBuffer.prototype.Shorten = function (newEnd) {
        // DEBUG: b2Assert(false);
    };
    b2GrowableBuffer.prototype.Data = function () {
        return this.data;
    };
    b2GrowableBuffer.prototype.GetCount = function () {
        return this.count;
    };
    b2GrowableBuffer.prototype.SetCount = function (newCount) {
        // DEBUG: b2Assert(0 <= newCount && newCount <= this.capacity);
        this.count = newCount;
    };
    b2GrowableBuffer.prototype.GetCapacity = function () {
        return this.capacity;
    };
    b2GrowableBuffer.prototype.RemoveIf = function (pred) {
        // DEBUG: let count = 0;
        // DEBUG: for (let i = 0; i < this.count; ++i) {
        // DEBUG:   if (!pred(this.data[i])) {
        // DEBUG:     count++;
        // DEBUG:   }
        // DEBUG: }
        this.count = std_remove_if(this.data, pred, this.count);
        // DEBUG: b2Assert(count === this.count);
    };
    b2GrowableBuffer.prototype.Unique = function (pred) {
        this.count = std_unique(this.data, 0, this.count, pred);
    };
    return b2GrowableBuffer;
}());
exports.b2GrowableBuffer = b2GrowableBuffer;
var b2FixtureParticleQueryCallback = /** @class */ (function (_super) {
    __extends(b2FixtureParticleQueryCallback, _super);
    function b2FixtureParticleQueryCallback(system) {
        var _this = _super.call(this) || this;
        _this.m_system = system;
        return _this;
    }
    b2FixtureParticleQueryCallback.prototype.ShouldQueryParticleSystem = function (system) {
        // Skip reporting particles.
        return false;
    };
    b2FixtureParticleQueryCallback.prototype.ReportFixture = function (fixture) {
        if (fixture.IsSensor()) {
            return true;
        }
        var shape = fixture.GetShape();
        var childCount = shape.GetChildCount();
        for (var childIndex = 0; childIndex < childCount; childIndex++) {
            var aabb = fixture.GetAABB(childIndex);
            var enumerator = this.m_system.GetInsideBoundsEnumerator(aabb);
            var index = void 0;
            while ((index = enumerator.GetNext()) >= 0) {
                this.ReportFixtureAndParticle(fixture, childIndex, index);
            }
        }
        return true;
    };
    b2FixtureParticleQueryCallback.prototype.ReportParticle = function (system, index) {
        return false;
    };
    b2FixtureParticleQueryCallback.prototype.ReportFixtureAndParticle = function (fixture, childIndex, index) {
        // DEBUG: b2Assert(false); // pure virtual
    };
    return b2FixtureParticleQueryCallback;
}(b2WorldCallbacks_1.b2QueryCallback));
exports.b2FixtureParticleQueryCallback = b2FixtureParticleQueryCallback;
var b2ParticleContact = /** @class */ (function () {
    function b2ParticleContact() {
        this.indexA = 0;
        this.indexB = 0;
        this.weight = 0;
        this.normal = new b2Math_1.b2Vec2();
        this.flags = 0;
    }
    b2ParticleContact.prototype.SetIndices = function (a, b) {
        // DEBUG: b2Assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
        this.indexA = a;
        this.indexB = b;
    };
    b2ParticleContact.prototype.SetWeight = function (w) {
        this.weight = w;
    };
    b2ParticleContact.prototype.SetNormal = function (n) {
        this.normal.Copy(n);
    };
    b2ParticleContact.prototype.SetFlags = function (f) {
        this.flags = f;
    };
    b2ParticleContact.prototype.GetIndexA = function () {
        return this.indexA;
    };
    b2ParticleContact.prototype.GetIndexB = function () {
        return this.indexB;
    };
    b2ParticleContact.prototype.GetWeight = function () {
        return this.weight;
    };
    b2ParticleContact.prototype.GetNormal = function () {
        return this.normal;
    };
    b2ParticleContact.prototype.GetFlags = function () {
        return this.flags;
    };
    b2ParticleContact.prototype.IsEqual = function (rhs) {
        return this.indexA === rhs.indexA && this.indexB === rhs.indexB && this.flags === rhs.flags && this.weight === rhs.weight && this.normal.x === rhs.normal.x && this.normal.y === rhs.normal.y;
    };
    b2ParticleContact.prototype.IsNotEqual = function (rhs) {
        return !this.IsEqual(rhs);
    };
    b2ParticleContact.prototype.ApproximatelyEqual = function (rhs) {
        var MAX_WEIGHT_DIFF = 0.01; // Weight 0 ~ 1, so about 1%
        var MAX_NORMAL_DIFF_SQ = 0.01 * 0.01; // Normal length = 1, so 1%
        return this.indexA === rhs.indexA && this.indexB === rhs.indexB && this.flags === rhs.flags && b2Math_1.b2Abs(this.weight - rhs.weight) < MAX_WEIGHT_DIFF && b2Math_1.b2Vec2.DistanceSquaredVV(this.normal, rhs.normal) < MAX_NORMAL_DIFF_SQ;
    };
    return b2ParticleContact;
}());
exports.b2ParticleContact = b2ParticleContact;
var b2ParticleBodyContact = /** @class */ (function () {
    function b2ParticleBodyContact() {
        this.index = 0; // Index of the particle making contact.
        this.weight = 0.0; // Weight of the contact. A value between 0.0f and 1.0f.
        this.normal = new b2Math_1.b2Vec2(); // The normalized direction from the particle to the body.
        this.mass = 0.0; // The effective mass used in calculating force.
    }
    return b2ParticleBodyContact;
}());
exports.b2ParticleBodyContact = b2ParticleBodyContact;
var b2ParticlePair = /** @class */ (function () {
    function b2ParticlePair() {
        this.indexA = 0; // Indices of the respective particles making pair.
        this.indexB = 0;
        this.flags = 0; // The logical sum of the particle flags. See the b2ParticleFlag enum.
        this.strength = 0.0; // The strength of cohesion among the particles.
        this.distance = 0.0; // The initial distance of the particles.
    }
    return b2ParticlePair;
}());
exports.b2ParticlePair = b2ParticlePair;
var b2ParticleTriad = /** @class */ (function () {
    function b2ParticleTriad() {
        this.indexA = 0; // Indices of the respective particles making triad.
        this.indexB = 0;
        this.indexC = 0;
        this.flags = 0; // The logical sum of the particle flags. See the b2ParticleFlag enum.
        this.strength = 0.0; // The strength of cohesion among the particles.
        this.pa = new b2Math_1.b2Vec2(0.0, 0.0); // Values used for calculation.
        this.pb = new b2Math_1.b2Vec2(0.0, 0.0);
        this.pc = new b2Math_1.b2Vec2(0.0, 0.0);
        this.ka = 0.0;
        this.kb = 0.0;
        this.kc = 0.0;
        this.s = 0.0;
    }
    return b2ParticleTriad;
}());
exports.b2ParticleTriad = b2ParticleTriad;
var b2ParticleSystemDef = /** @class */ (function () {
    function b2ParticleSystemDef() {
        // Initialize physical coefficients to the maximum values that
        // maintain numerical stability.
        /**
         * Enable strict Particle/Body contact check.
         * See SetStrictContactCheck for details.
         */
        this.strictContactCheck = false;
        /**
         * Set the particle density.
         * See SetDensity for details.
         */
        this.density = 1.0;
        /**
         * Change the particle gravity scale. Adjusts the effect of the
         * global gravity vector on particles. Default value is 1.0f.
         */
        this.gravityScale = 1.0;
        /**
         * Particles behave as circles with this radius. In Box2D units.
         */
        this.radius = 1.0;
        /**
         * Set the maximum number of particles.
         * By default, there is no maximum. The particle buffers can
         * continue to grow while b2World's block allocator still has
         * memory.
         * See SetMaxParticleCount for details.
         */
        this.maxCount = 0;
        /**
         * Increases pressure in response to compression
         * Smaller values allow more compression
         */
        this.pressureStrength = 0.005;
        /**
         * Reduces velocity along the collision normal
         * Smaller value reduces less
         */
        this.dampingStrength = 1.0;
        /**
         * Restores shape of elastic particle groups
         * Larger values increase elastic particle velocity
         */
        this.elasticStrength = 0.25;
        /**
         * Restores length of spring particle groups
         * Larger values increase spring particle velocity
         */
        this.springStrength = 0.25;
        /**
         * Reduces relative velocity of viscous particles
         * Larger values slow down viscous particles more
         */
        this.viscousStrength = 0.25;
        /**
         * Produces pressure on tensile particles
         * 0~0.2. Larger values increase the amount of surface tension.
         */
        this.surfaceTensionPressureStrength = 0.2;
        /**
         * Smoothes outline of tensile particles
         * 0~0.2. Larger values result in rounder, smoother,
         * water-drop-like clusters of particles.
         */
        this.surfaceTensionNormalStrength = 0.2;
        /**
         * Produces additional pressure on repulsive particles
         * Larger values repulse more
         * Negative values mean attraction. The range where particles
         * behave stably is about -0.2 to 2.0.
         */
        this.repulsiveStrength = 1.0;
        /**
         * Produces repulsion between powder particles
         * Larger values repulse more
         */
        this.powderStrength = 0.5;
        /**
         * Pushes particles out of solid particle group
         * Larger values repulse more
         */
        this.ejectionStrength = 0.5;
        /**
         * Produces static pressure
         * Larger values increase the pressure on neighboring partilces
         * For a description of static pressure, see
         * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
         */
        this.staticPressureStrength = 0.2;
        /**
         * Reduces instability in static pressure calculation
         * Larger values make stabilize static pressure with fewer
         * iterations
         */
        this.staticPressureRelaxation = 0.2;
        /**
         * Computes static pressure more precisely
         * See SetStaticPressureIterations for details
         */
        this.staticPressureIterations = 8;
        /**
         * Determines how fast colors are mixed
         * 1.0f ==> mixed immediately
         * 0.5f ==> mixed half way each simulation step (see
         * b2World::Step())
         */
        this.colorMixingStrength = 0.5;
        /**
         * Whether to destroy particles by age when no more particles
         * can be created.  See #b2ParticleSystem::SetDestructionByAge()
         * for more information.
         */
        this.destroyByAge = true;
        /**
         * Granularity of particle lifetimes in seconds.  By default
         * this is set to (1.0f / 60.0f) seconds.  b2ParticleSystem uses
         * a 32-bit signed value to track particle lifetimes so the
         * maximum lifetime of a particle is (2^32 - 1) / (1.0f /
         * lifetimeGranularity) seconds. With the value set to 1/60 the
         * maximum lifetime or age of a particle is 2.27 years.
         */
        this.lifetimeGranularity = 1.0 / 60.0;
    }
    b2ParticleSystemDef.prototype.Copy = function (def) {
        this.strictContactCheck = def.strictContactCheck;
        this.density = def.density;
        this.gravityScale = def.gravityScale;
        this.radius = def.radius;
        this.maxCount = def.maxCount;
        this.pressureStrength = def.pressureStrength;
        this.dampingStrength = def.dampingStrength;
        this.elasticStrength = def.elasticStrength;
        this.springStrength = def.springStrength;
        this.viscousStrength = def.viscousStrength;
        this.surfaceTensionPressureStrength = def.surfaceTensionPressureStrength;
        this.surfaceTensionNormalStrength = def.surfaceTensionNormalStrength;
        this.repulsiveStrength = def.repulsiveStrength;
        this.powderStrength = def.powderStrength;
        this.ejectionStrength = def.ejectionStrength;
        this.staticPressureStrength = def.staticPressureStrength;
        this.staticPressureRelaxation = def.staticPressureRelaxation;
        this.staticPressureIterations = def.staticPressureIterations;
        this.colorMixingStrength = def.colorMixingStrength;
        this.destroyByAge = def.destroyByAge;
        this.lifetimeGranularity = def.lifetimeGranularity;
        return this;
    };
    b2ParticleSystemDef.prototype.Clone = function () {
        return new b2ParticleSystemDef().Copy(this);
    };
    return b2ParticleSystemDef;
}());
exports.b2ParticleSystemDef = b2ParticleSystemDef;
var b2ParticleSystem = /** @class */ (function () {
    function b2ParticleSystem(def, world) {
        this.m_paused = false;
        this.m_timestamp = 0;
        this.m_allParticleFlags = 0;
        this.m_needsUpdateAllParticleFlags = false;
        this.m_allGroupFlags = 0;
        this.m_needsUpdateAllGroupFlags = false;
        this.m_hasForce = false;
        this.m_iterationIndex = 0;
        this.m_inverseDensity = 0.0;
        this.m_particleDiameter = 0.0;
        this.m_inverseDiameter = 0.0;
        this.m_squaredDiameter = 0.0;
        this.m_count = 0;
        this.m_internalAllocatedCapacity = 0;
        /**
         * Allocator for b2ParticleHandle instances.
         */
        ///m_handleAllocator: any = null;
        /**
         * Maps particle indicies to handles.
         */
        this.m_handleIndexBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_flagsBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_positionBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_velocityBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_forceBuffer = [];
        /**
         * this.m_weightBuffer is populated in ComputeWeight and used in
         * ComputeDepth(), SolveStaticPressure() and SolvePressure().
         */
        this.m_weightBuffer = [];
        /**
         * When any particles have the flag b2_staticPressureParticle,
         * this.m_staticPressureBuffer is first allocated and used in
         * SolveStaticPressure() and SolvePressure().  It will be
         * reallocated on subsequent CreateParticle() calls.
         */
        this.m_staticPressureBuffer = [];
        /**
         * this.m_accumulationBuffer is used in many functions as a temporary
         * buffer for scalar values.
         */
        this.m_accumulationBuffer = [];
        /**
         * When any particles have the flag b2_tensileParticle,
         * this.m_accumulation2Buffer is first allocated and used in
         * SolveTensile() as a temporary buffer for vector values.  It
         * will be reallocated on subsequent CreateParticle() calls.
         */
        this.m_accumulation2Buffer = [];
        /**
         * When any particle groups have the flag b2_solidParticleGroup,
         * this.m_depthBuffer is first allocated and populated in
         * ComputeDepth() and used in SolveSolid(). It will be
         * reallocated on subsequent CreateParticle() calls.
         */
        this.m_depthBuffer = [];
        this.m_colorBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_groupBuffer = [];
        this.m_userDataBuffer = new b2ParticleSystem.UserOverridableBuffer();
        /**
         * Stuck particle detection parameters and record keeping
         */
        this.m_stuckThreshold = 0;
        this.m_lastBodyContactStepBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_bodyContactCountBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_consecutiveContactStepsBuffer = new b2ParticleSystem.UserOverridableBuffer();
        this.m_stuckParticleBuffer = new b2GrowableBuffer(function () { return 0; });
        this.m_proxyBuffer = new b2GrowableBuffer(function () { return new b2ParticleSystem.Proxy(); });
        this.m_contactBuffer = new b2GrowableBuffer(function () { return new b2ParticleContact(); });
        this.m_bodyContactBuffer = new b2GrowableBuffer(function () { return new b2ParticleBodyContact(); });
        this.m_pairBuffer = new b2GrowableBuffer(function () { return new b2ParticlePair(); });
        this.m_triadBuffer = new b2GrowableBuffer(function () { return new b2ParticleTriad(); });
        /**
         * Time each particle should be destroyed relative to the last
         * time this.m_timeElapsed was initialized.  Each unit of time
         * corresponds to b2ParticleSystemDef::lifetimeGranularity
         * seconds.
         */
        this.m_expirationTimeBuffer = new b2ParticleSystem.UserOverridableBuffer();
        /**
         * List of particle indices sorted by expiration time.
         */
        this.m_indexByExpirationTimeBuffer = new b2ParticleSystem.UserOverridableBuffer();
        /**
         * Time elapsed in 32:32 fixed point.  Each non-fractional unit
         * of time corresponds to
         * b2ParticleSystemDef::lifetimeGranularity seconds.
         */
        this.m_timeElapsed = 0;
        /**
         * Whether the expiration time buffer has been modified and
         * needs to be resorted.
         */
        this.m_expirationTimeBufferRequiresSorting = false;
        this.m_groupCount = 0;
        this.m_groupList = null;
        this.m_def = new b2ParticleSystemDef();
        this.m_prev = null;
        this.m_next = null;
        this.SetStrictContactCheck(def.strictContactCheck);
        this.SetDensity(def.density);
        this.SetGravityScale(def.gravityScale);
        this.SetRadius(def.radius);
        this.SetMaxParticleCount(def.maxCount);
        // DEBUG: b2Assert(def.lifetimeGranularity > 0.0);
        this.m_def = def.Clone();
        this.m_world = world;
        this.SetDestructionByAge(this.m_def.destroyByAge);
    }
    b2ParticleSystem.computeTag = function (x, y) {
        ///return ((uint32)(y + yOffset) << yShift) + (uint32)(xScale * x + xOffset);
        return ((((y + b2ParticleSystem.yOffset) >>> 0) << b2ParticleSystem.yShift) + ((b2ParticleSystem.xScale * x + b2ParticleSystem.xOffset) >>> 0)) >>> 0;
    };
    b2ParticleSystem.computeRelativeTag = function (tag, x, y) {
        ///return tag + (y << yShift) + (x << xShift);
        return (tag + (y << b2ParticleSystem.yShift) + (x << b2ParticleSystem.xShift)) >>> 0;
    };
    b2ParticleSystem.prototype.Drop = function () {
        while (this.m_groupList) {
            this.DestroyParticleGroup(this.m_groupList);
        }
        this.FreeUserOverridableBuffer(this.m_handleIndexBuffer);
        this.FreeUserOverridableBuffer(this.m_flagsBuffer);
        this.FreeUserOverridableBuffer(this.m_lastBodyContactStepBuffer);
        this.FreeUserOverridableBuffer(this.m_bodyContactCountBuffer);
        this.FreeUserOverridableBuffer(this.m_consecutiveContactStepsBuffer);
        this.FreeUserOverridableBuffer(this.m_positionBuffer);
        this.FreeUserOverridableBuffer(this.m_velocityBuffer);
        this.FreeUserOverridableBuffer(this.m_colorBuffer);
        this.FreeUserOverridableBuffer(this.m_userDataBuffer);
        this.FreeUserOverridableBuffer(this.m_expirationTimeBuffer);
        this.FreeUserOverridableBuffer(this.m_indexByExpirationTimeBuffer);
        this.FreeBuffer(this.m_forceBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_weightBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_staticPressureBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_accumulationBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_accumulation2Buffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_depthBuffer, this.m_internalAllocatedCapacity);
        this.FreeBuffer(this.m_groupBuffer, this.m_internalAllocatedCapacity);
    };
    /**
     * Create a particle whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * A simulation step must occur before it's possible to interact
     * with a newly created particle.  For example,
     * DestroyParticleInShape() will not destroy a particle until
     * b2World::Step() has been called.
     *
     * warning: This function is locked during callbacks.
     */
    b2ParticleSystem.prototype.CreateParticle = function (def) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // Double the particle capacity.
            var capacity = this.m_count ? 2 * this.m_count : b2Settings_1.b2_minParticleSystemBufferCapacity;
            this.ReallocateInternalAllocatedBuffers(capacity);
        }
        if (this.m_count >= this.m_internalAllocatedCapacity) {
            // If the oldest particle should be destroyed...
            if (this.m_def.destroyByAge) {
                this.DestroyOldestParticle(0, false);
                // Need to destroy this particle *now* so that it's possible to
                // create a new particle.
                this.SolveZombie();
            }
            else {
                return b2Settings_1.b2_invalidParticleIndex;
            }
        }
        var index = this.m_count++;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        this.m_flagsBuffer.data[index] = 0;
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[index] = 0;
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[index] = 0;
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[index] = 0;
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        this.m_positionBuffer.data[index] = (this.m_positionBuffer.data[index] || new b2Math_1.b2Vec2()).Copy(b2Settings_1.b2Maybe(def.position, b2Math_1.b2Vec2.ZERO));
        this.m_velocityBuffer.data[index] = (this.m_velocityBuffer.data[index] || new b2Math_1.b2Vec2()).Copy(b2Settings_1.b2Maybe(def.velocity, b2Math_1.b2Vec2.ZERO));
        this.m_weightBuffer[index] = 0;
        this.m_forceBuffer[index] = (this.m_forceBuffer[index] || new b2Math_1.b2Vec2()).SetZero();
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[index] = 0;
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[index] = 0;
        }
        var color = new b2Draw_1.b2Color().Copy(b2Settings_1.b2Maybe(def.color, b2Draw_1.b2Color.ZERO));
        if (this.m_colorBuffer.data || !color.IsZero()) {
            this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            this.m_colorBuffer.data[index] = (this.m_colorBuffer.data[index] || new b2Draw_1.b2Color()).Copy(color);
        }
        if (this.m_userDataBuffer.data || def.userData) {
            this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
            this.m_userDataBuffer.data[index] = def.userData;
        }
        if (this.m_handleIndexBuffer.data) {
            this.m_handleIndexBuffer.data[index] = null;
        }
        ///Proxy& proxy = m_proxyBuffer.Append();
        var proxy = this.m_proxyBuffer.data[this.m_proxyBuffer.Append()];
        // If particle lifetimes are enabled or the lifetime is set in the particle
        // definition, initialize the lifetime.
        var lifetime = b2Settings_1.b2Maybe(def.lifetime, 0.0);
        var finiteLifetime = lifetime > 0.0;
        if (this.m_expirationTimeBuffer.data || finiteLifetime) {
            this.SetParticleLifetime(index, finiteLifetime ? lifetime :
                this.ExpirationTimeToLifetime(-this.GetQuantizedTimeElapsed()));
            // Add a reference to the newly added particle to the end of the
            // queue.
            if (!this.m_indexByExpirationTimeBuffer.data) {
                throw new Error();
            }
            this.m_indexByExpirationTimeBuffer.data[index] = index;
        }
        proxy.index = index;
        var group = b2Settings_1.b2Maybe(def.group, null);
        this.m_groupBuffer[index] = group;
        if (group) {
            if (group.m_firstIndex < group.m_lastIndex) {
                // Move particles in the group just before the new particle.
                this.RotateBuffer(group.m_firstIndex, group.m_lastIndex, index);
                // DEBUG: b2Assert(group.m_lastIndex === index);
                // Update the index range of the group to contain the new particle.
                group.m_lastIndex = index + 1;
            }
            else {
                // If the group is empty, reset the index range to contain only the
                // new particle.
                group.m_firstIndex = index;
                group.m_lastIndex = index + 1;
            }
        }
        this.SetParticleFlags(index, b2Settings_1.b2Maybe(def.flags, 0));
        return index;
    };
    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #b2ParticleHandle for why you might want a handle.
     */
    b2ParticleSystem.prototype.GetParticleHandleFromIndex = function (index) {
        // DEBUG: b2Assert(index >= 0 && index < this.GetParticleCount() && index !== b2_invalidParticleIndex);
        this.m_handleIndexBuffer.data = this.RequestBuffer(this.m_handleIndexBuffer.data);
        var handle = this.m_handleIndexBuffer.data[index];
        if (handle) {
            return handle;
        }
        // Create a handle.
        ///handle = m_handleAllocator.Allocate();
        handle = new b2Particle_1.b2ParticleHandle();
        // DEBUG: b2Assert(handle !== null);
        handle.SetIndex(index);
        this.m_handleIndexBuffer.data[index] = handle;
        return handle;
    };
    /**
     * Destroy a particle.
     *
     * The particle is removed after the next simulation step (see
     * b2World::Step()).
     *
     * @param index Index of the particle to destroy.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    b2ParticleSystem.prototype.DestroyParticle = function (index, callDestructionListener) {
        if (callDestructionListener === void 0) { callDestructionListener = false; }
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        var flags = b2Particle_1.b2ParticleFlag.b2_zombieParticle;
        if (callDestructionListener) {
            flags |= b2Particle_1.b2ParticleFlag.b2_destructionListenerParticle;
        }
        this.SetParticleFlags(index, this.m_flagsBuffer.data[index] | flags);
    };
    /**
     * Destroy the Nth oldest particle in the system.
     *
     * The particle is removed after the next b2World::Step().
     *
     * @param index Index of the Nth oldest particle to
     *      destroy, 0 will destroy the oldest particle in the
     *      system, 1 will destroy the next oldest particle etc.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    b2ParticleSystem.prototype.DestroyOldestParticle = function (index, callDestructionListener) {
        if (callDestructionListener === void 0) { callDestructionListener = false; }
        var particleCount = this.GetParticleCount();
        // DEBUG: b2Assert(index >= 0 && index < particleCount);
        // Make sure particle lifetime tracking is enabled.
        // DEBUG: b2Assert(this.m_indexByExpirationTimeBuffer.data !== null);
        if (!this.m_indexByExpirationTimeBuffer.data) {
            throw new Error();
        }
        if (!this.m_expirationTimeBuffer.data) {
            throw new Error();
        }
        // Destroy the oldest particle (preferring to destroy finite
        // lifetime particles first) to free a slot in the buffer.
        var oldestFiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[particleCount - (index + 1)];
        var oldestInfiniteLifetimeParticle = this.m_indexByExpirationTimeBuffer.data[index];
        this.DestroyParticle(this.m_expirationTimeBuffer.data[oldestFiniteLifetimeParticle] > 0.0 ?
            oldestFiniteLifetimeParticle : oldestInfiniteLifetimeParticle, callDestructionListener);
    };
    /**
     * Destroy particles inside a shape.
     *
     * warning: This function is locked during callbacks.
     *
     * In addition, this function immediately destroys particles in
     * the shape in constrast to DestroyParticle() which defers the
     * destruction until the next simulation step.
     *
     * @return Number of particles destroyed.
     * @param shape Shape which encloses particles
     *      that should be destroyed.
     * @param xf Transform applied to the shape.
     * @param callDestructionListener Whether to call the
     *      world b2DestructionListener for each particle
     *      destroyed.
     */
    b2ParticleSystem.prototype.DestroyParticlesInShape = function (shape, xf, callDestructionListener) {
        if (callDestructionListener === void 0) { callDestructionListener = false; }
        var s_aabb = b2ParticleSystem.DestroyParticlesInShape_s_aabb;
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        var callback = new b2ParticleSystem.DestroyParticlesInShapeCallback(this, shape, xf, callDestructionListener);
        var aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, 0);
        this.m_world.QueryAABB(callback, aabb);
        return callback.Destroyed();
    };
    /**
     * Create a particle group whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * warning: This function is locked during callbacks.
     */
    b2ParticleSystem.prototype.CreateParticleGroup = function (groupDef) {
        var s_transform = b2ParticleSystem.CreateParticleGroup_s_transform;
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        var transform = s_transform;
        transform.SetPositionAngle(b2Settings_1.b2Maybe(groupDef.position, b2Math_1.b2Vec2.ZERO), b2Settings_1.b2Maybe(groupDef.angle, 0));
        var firstIndex = this.m_count;
        if (groupDef.shape) {
            this.CreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
        }
        if (groupDef.shapes) {
            this.CreateParticlesWithShapesForGroup(groupDef.shapes, b2Settings_1.b2Maybe(groupDef.shapeCount, groupDef.shapes.length), groupDef, transform);
        }
        if (groupDef.positionData) {
            var count = b2Settings_1.b2Maybe(groupDef.particleCount, groupDef.positionData.length);
            for (var i = 0; i < count; i++) {
                var p = groupDef.positionData[i];
                this.CreateParticleForGroup(groupDef, transform, p);
            }
        }
        var lastIndex = this.m_count;
        var group = new b2ParticleGroup_1.b2ParticleGroup(this);
        group.m_firstIndex = firstIndex;
        group.m_lastIndex = lastIndex;
        group.m_strength = b2Settings_1.b2Maybe(groupDef.strength, 1);
        group.m_userData = groupDef.userData;
        group.m_transform.Copy(transform);
        group.m_prev = null;
        group.m_next = this.m_groupList;
        if (this.m_groupList) {
            this.m_groupList.m_prev = group;
        }
        this.m_groupList = group;
        ++this.m_groupCount;
        for (var i = firstIndex; i < lastIndex; i++) {
            this.m_groupBuffer[i] = group;
        }
        this.SetGroupFlags(group, b2Settings_1.b2Maybe(groupDef.groupFlags, 0));
        // Create pairs and triads between particles in the group.
        var filter = new b2ParticleSystem.ConnectionFilter();
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(firstIndex, lastIndex, filter);
        if (groupDef.group) {
            this.JoinParticleGroups(groupDef.group, group);
            group = groupDef.group;
        }
        return group;
    };
    /**
     * Join two particle groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param groupA the first group. Expands to encompass the second group.
     * @param groupB the second group. It is destroyed.
     */
    b2ParticleSystem.prototype.JoinParticleGroups = function (groupA, groupB) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        // DEBUG: b2Assert(groupA !== groupB);
        this.RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, this.m_count);
        // DEBUG: b2Assert(groupB.m_lastIndex === this.m_count);
        this.RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex, groupB.m_firstIndex);
        // DEBUG: b2Assert(groupA.m_lastIndex === groupB.m_firstIndex);
        // Create pairs and triads connecting groupA and groupB.
        var filter = new b2ParticleSystem.JoinParticleGroupsFilter(groupB.m_firstIndex);
        this.UpdateContacts(true);
        this.UpdatePairsAndTriads(groupA.m_firstIndex, groupB.m_lastIndex, filter);
        for (var i = groupB.m_firstIndex; i < groupB.m_lastIndex; i++) {
            this.m_groupBuffer[i] = groupA;
        }
        var groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
        this.SetGroupFlags(groupA, groupFlags);
        groupA.m_lastIndex = groupB.m_lastIndex;
        groupB.m_firstIndex = groupB.m_lastIndex;
        this.DestroyParticleGroup(groupB);
    };
    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group the group to be split.
     */
    b2ParticleSystem.prototype.SplitParticleGroup = function (group) {
        this.UpdateContacts(true);
        var particleCount = group.GetParticleCount();
        // We create several linked lists. Each list represents a set of connected particles.
        ///ParticleListNode* nodeBuffer = (ParticleListNode*) m_world.m_stackAllocator.Allocate(sizeof(ParticleListNode) * particleCount);
        var nodeBuffer = b2Settings_1.b2MakeArray(particleCount, function (index) { return new b2ParticleSystem.ParticleListNode(); });
        b2ParticleSystem.InitializeParticleLists(group, nodeBuffer);
        this.MergeParticleListsInContact(group, nodeBuffer);
        var survivingList = b2ParticleSystem.FindLongestParticleList(group, nodeBuffer);
        this.MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
        this.CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
        this.UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
        ///this.m_world.m_stackAllocator.Free(nodeBuffer);
    };
    /**
     * Get the world particle group list. With the returned group,
     * use b2ParticleGroup::GetNext to get the next group in the
     * world list.
     *
     * A null group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    b2ParticleSystem.prototype.GetParticleGroupList = function () {
        return this.m_groupList;
    };
    /**
     * Get the number of particle groups.
     */
    b2ParticleSystem.prototype.GetParticleGroupCount = function () {
        return this.m_groupCount;
    };
    /**
     * Get the number of particles.
     */
    b2ParticleSystem.prototype.GetParticleCount = function () {
        return this.m_count;
    };
    /**
     * Get the maximum number of particles.
     */
    b2ParticleSystem.prototype.GetMaxParticleCount = function () {
        return this.m_def.maxCount;
    };
    /**
     * Set the maximum number of particles.
     *
     * A value of 0 means there is no maximum. The particle buffers
     * can continue to grow while b2World's block allocator still
     * has memory.
     *
     * Note: If you try to CreateParticle() with more than this
     * count, b2_invalidParticleIndex is returned unless
     * SetDestructionByAge() is used to enable the destruction of
     * the oldest particles in the system.
     */
    b2ParticleSystem.prototype.SetMaxParticleCount = function (count) {
        // DEBUG: b2Assert(this.m_count <= count);
        this.m_def.maxCount = count;
    };
    /**
     * Get all existing particle flags.
     */
    b2ParticleSystem.prototype.GetAllParticleFlags = function () {
        return this.m_allParticleFlags;
    };
    /**
     * Get all existing particle group flags.
     */
    b2ParticleSystem.prototype.GetAllGroupFlags = function () {
        return this.m_allGroupFlags;
    };
    /**
     * Pause or unpause the particle system. When paused,
     * b2World::Step() skips over this particle system. All
     * b2ParticleSystem function calls still work.
     *
     * @param paused paused is true to pause, false to un-pause.
     */
    b2ParticleSystem.prototype.SetPaused = function (paused) {
        this.m_paused = paused;
    };
    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @return true if the particle system is being updated in b2World::Step().
     */
    b2ParticleSystem.prototype.GetPaused = function () {
        return this.m_paused;
    };
    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with b2Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    b2ParticleSystem.prototype.SetDensity = function (density) {
        this.m_def.density = density;
        this.m_inverseDensity = 1 / this.m_def.density;
    };
    /**
     * Get the particle density.
     */
    b2ParticleSystem.prototype.GetDensity = function () {
        return this.m_def.density;
    };
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    b2ParticleSystem.prototype.SetGravityScale = function (gravityScale) {
        this.m_def.gravityScale = gravityScale;
    };
    /**
     * Get the particle gravity scale.
     */
    b2ParticleSystem.prototype.GetGravityScale = function () {
        return this.m_def.gravityScale;
    };
    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1.0f but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    b2ParticleSystem.prototype.SetDamping = function (damping) {
        this.m_def.dampingStrength = damping;
    };
    /**
     * Get damping for particles
     */
    b2ParticleSystem.prototype.GetDamping = function () {
        return this.m_def.dampingStrength;
    };
    /**
     * Change the number of iterations when calculating the static
     * pressure of particles. By default, 8 iterations. You can
     * reduce the number of iterations down to 1 in some situations,
     * but this may cause instabilities when many particles come
     * together. If you see particles popping away from each other
     * like popcorn, you may have to increase the number of
     * iterations.
     *
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    b2ParticleSystem.prototype.SetStaticPressureIterations = function (iterations) {
        this.m_def.staticPressureIterations = iterations;
    };
    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    b2ParticleSystem.prototype.GetStaticPressureIterations = function () {
        return this.m_def.staticPressureIterations;
    };
    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    b2ParticleSystem.prototype.SetRadius = function (radius) {
        this.m_particleDiameter = 2 * radius;
        this.m_squaredDiameter = this.m_particleDiameter * this.m_particleDiameter;
        this.m_inverseDiameter = 1 / this.m_particleDiameter;
    };
    /**
     * Get the particle radius.
     */
    b2ParticleSystem.prototype.GetRadius = function () {
        return this.m_particleDiameter / 2;
    };
    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    b2ParticleSystem.prototype.GetPositionBuffer = function () {
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        return this.m_positionBuffer.data;
    };
    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle velocities array.
     */
    b2ParticleSystem.prototype.GetVelocityBuffer = function () {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        return this.m_velocityBuffer.data;
    };
    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle colors array.
     */
    b2ParticleSystem.prototype.GetColorBuffer = function () {
        this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
        return this.m_colorBuffer.data;
    };
    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle group array.
     */
    b2ParticleSystem.prototype.GetGroupBuffer = function () {
        return this.m_groupBuffer;
    };
    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    b2ParticleSystem.prototype.GetWeightBuffer = function () {
        return this.m_weightBuffer;
    };
    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle user-data array.
     */
    b2ParticleSystem.prototype.GetUserDataBuffer = function () {
        this.m_userDataBuffer.data = this.RequestBuffer(this.m_userDataBuffer.data);
        return this.m_userDataBuffer.data;
    };
    /**
     * Get the flags for each particle. See the b2ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle-flags array.
     */
    b2ParticleSystem.prototype.GetFlagsBuffer = function () {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        return this.m_flagsBuffer.data;
    };
    /**
     * Set flags for a particle. See the b2ParticleFlag enum.
     */
    b2ParticleSystem.prototype.SetParticleFlags = function (index, newFlags) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        var oldFlags = this.m_flagsBuffer.data[index];
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllParticleFlags = true;
        }
        if (~this.m_allParticleFlags & newFlags) {
            // If any flags were added
            if (newFlags & b2Particle_1.b2ParticleFlag.b2_tensileParticle) {
                this.m_accumulation2Buffer = this.RequestBuffer(this.m_accumulation2Buffer);
            }
            if (newFlags & b2Particle_1.b2ParticleFlag.b2_colorMixingParticle) {
                this.m_colorBuffer.data = this.RequestBuffer(this.m_colorBuffer.data);
            }
            this.m_allParticleFlags |= newFlags;
        }
        this.m_flagsBuffer.data[index] = newFlags;
    };
    /**
     * Get flags for a particle. See the b2ParticleFlag enum.
     */
    b2ParticleSystem.prototype.GetParticleFlags = function (index) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        return this.m_flagsBuffer.data[index];
    };
    /**
     * Set an external buffer for particle data.
     *
     * Normally, the b2World's block allocator is used for particle
     * data. However, sometimes you may have an OpenGL or Java
     * buffer for particle data. To avoid data duplication, you may
     * supply this external buffer.
     *
     * Note that, when b2World's block allocator is used, the
     * particle data buffers can grow as required. However, when
     * external buffers are used, the maximum number of particles is
     * clamped to the size of the smallest external buffer.
     *
     * @param buffer a pointer to a block of memory.
     * @param capacity the number of values in the block.
     */
    b2ParticleSystem.prototype.SetFlagsBuffer = function (buffer, capacity) {
        this.SetUserOverridableBuffer(this.m_flagsBuffer, buffer, capacity);
    };
    b2ParticleSystem.prototype.SetPositionBuffer = function (buffer, capacity) {
        ///if (buffer instanceof Float32Array) {
        ///let array = [];
        ///for (let i = 0; i < capacity; ++i) {
        ///  array[i] = new b2Vec2(buffer.subarray(i * 2, i * 2 + 2));
        ///}
        ///this.SetUserOverridableBuffer(this.m_positionBuffer, array, capacity);
        ///} else {
        this.SetUserOverridableBuffer(this.m_positionBuffer, buffer, capacity);
        ///}
    };
    b2ParticleSystem.prototype.SetVelocityBuffer = function (buffer, capacity) {
        ///if (buffer instanceof Float32Array) {
        ///let array = [];
        ///for (let i = 0; i < capacity; ++i) {
        ///  array[i] = new b2Vec2(buffer.subarray(i * 2, i * 2 + 2));
        ///}
        ///this.SetUserOverridableBuffer(this.m_velocityBuffer, array, capacity);
        ///} else {
        this.SetUserOverridableBuffer(this.m_velocityBuffer, buffer, capacity);
        ///}
    };
    b2ParticleSystem.prototype.SetColorBuffer = function (buffer, capacity) {
        ///if (buffer instanceof Uint8Array) {
        ///let array: b2Color[] = [];
        ///for (let i = 0; i < capacity; ++i) {
        ///  array[i] = new b2Color(buffer.subarray(i * 4, i * 4 + 4));
        ///}
        ///this.SetUserOverridableBuffer(this.m_colorBuffer, array, capacity);
        ///} else {
        this.SetUserOverridableBuffer(this.m_colorBuffer, buffer, capacity);
        ///}
    };
    b2ParticleSystem.prototype.SetUserDataBuffer = function (buffer, capacity) {
        this.SetUserOverridableBuffer(this.m_userDataBuffer, buffer, capacity);
    };
    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    b2ParticleSystem.prototype.GetContacts = function () {
        return this.m_contactBuffer.data;
    };
    b2ParticleSystem.prototype.GetContactCount = function () {
        return this.m_contactBuffer.count;
    };
    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    b2ParticleSystem.prototype.GetBodyContacts = function () {
        return this.m_bodyContactBuffer.data;
    };
    b2ParticleSystem.prototype.GetBodyContactCount = function () {
        return this.m_bodyContactBuffer.count;
    };
    /**
     * Get array of particle pairs. The particles in a pair:
     *   (1) are contacting,
     *   (2) are in the same particle group,
     *   (3) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (4) have at least one particle that is a spring or barrier
     *       particle (i.e. one of the types in k_pairFlags),
     *   (5) have at least one particle that returns true for
     *       ConnectionFilter::IsNecessary,
     *   (6) are not zombie particles.
     *
     * Essentially, this is an array of spring or barrier particles
     * that are interacting. The array is sorted by b2ParticlePair's
     * indexA, and then indexB. There are no duplicate entries.
     */
    b2ParticleSystem.prototype.GetPairs = function () {
        return this.m_pairBuffer.data;
    };
    b2ParticleSystem.prototype.GetPairCount = function () {
        return this.m_pairBuffer.count;
    };
    /**
     * Get array of particle triads. The particles in a triad:
     *   (1) are in the same particle group,
     *   (2) are in a Voronoi triangle together,
     *   (3) are within b2_maxTriadDistance particle diameters of each
     *       other,
     *   (4) return true for ConnectionFilter::ShouldCreateTriad
     *   (5) have at least one particle of type elastic (i.e. one of the
     *       types in k_triadFlags),
     *   (6) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (7) are not zombie particles.
     *
     * Essentially, this is an array of elastic particles that are
     * interacting. The array is sorted by b2ParticleTriad's indexA,
     * then indexB, then indexC. There are no duplicate entries.
     */
    b2ParticleSystem.prototype.GetTriads = function () {
        return this.m_triadBuffer.data;
    };
    b2ParticleSystem.prototype.GetTriadCount = function () {
        return this.m_triadBuffer.count;
    };
    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * "stuck". Setting to zero or less disables.
     */
    b2ParticleSystem.prototype.SetStuckThreshold = function (steps) {
        this.m_stuckThreshold = steps;
        if (steps > 0) {
            this.m_lastBodyContactStepBuffer.data = this.RequestBuffer(this.m_lastBodyContactStepBuffer.data);
            this.m_bodyContactCountBuffer.data = this.RequestBuffer(this.m_bodyContactCountBuffer.data);
            this.m_consecutiveContactStepsBuffer.data = this.RequestBuffer(this.m_consecutiveContactStepsBuffer.data);
        }
    };
    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    b2ParticleSystem.prototype.GetStuckCandidates = function () {
        ///return m_stuckParticleBuffer.Data();
        return this.m_stuckParticleBuffer.Data();
    };
    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    b2ParticleSystem.prototype.GetStuckCandidateCount = function () {
        ///return m_stuckParticleBuffer.GetCount();
        return this.m_stuckParticleBuffer.GetCount();
    };
    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    b2ParticleSystem.prototype.ComputeCollisionEnergy = function () {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var s_v = b2ParticleSystem.ComputeCollisionEnergy_s_v;
        var vel_data = this.m_velocityBuffer.data;
        var sum_v2 = 0;
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var n = contact.normal;
            ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
            var v = b2Math_1.b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
            var vn = b2Math_1.b2Vec2.DotVV(v, n);
            if (vn < 0) {
                sum_v2 += vn * vn;
            }
        }
        return 0.5 * this.GetParticleMass() * sum_v2;
    };
    /**
     * Set strict Particle/Body contact check.
     *
     * This is an option that will help ensure correct behavior if
     * there are corners in the world model where Particle/Body
     * contact is ambiguous. This option scales at n*log(n) of the
     * number of Particle/Body contacts, so it is best to only
     * enable if it is necessary for your geometry. Enable if you
     * see strange particle behavior around b2Body intersections.
     */
    b2ParticleSystem.prototype.SetStrictContactCheck = function (enabled) {
        this.m_def.strictContactCheck = enabled;
    };
    /**
     * Get the status of the strict contact check.
     */
    b2ParticleSystem.prototype.GetStrictContactCheck = function () {
        return this.m_def.strictContactCheck;
    };
    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0.0f
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    b2ParticleSystem.prototype.SetParticleLifetime = function (index, lifetime) {
        // DEBUG: b2Assert(this.ValidateParticleIndex(index));
        var initializeExpirationTimes = this.m_indexByExpirationTimeBuffer.data === null;
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);
        // Initialize the inverse mapping buffer.
        if (initializeExpirationTimes) {
            var particleCount = this.GetParticleCount();
            for (var i = 0; i < particleCount; ++i) {
                this.m_indexByExpirationTimeBuffer.data[i] = i;
            }
        }
        ///const int32 quantizedLifetime = (int32)(lifetime / m_def.lifetimeGranularity);
        var quantizedLifetime = lifetime / this.m_def.lifetimeGranularity;
        // Use a negative lifetime so that it's possible to track which
        // of the infinite lifetime particles are older.
        var newExpirationTime = quantizedLifetime > 0.0 ? this.GetQuantizedTimeElapsed() + quantizedLifetime : quantizedLifetime;
        if (newExpirationTime !== this.m_expirationTimeBuffer.data[index]) {
            this.m_expirationTimeBuffer.data[index] = newExpirationTime;
            this.m_expirationTimeBufferRequiresSorting = true;
        }
    };
    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0.0f is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0.0f
     * indicate the particle has an infinite lifetime.
     */
    b2ParticleSystem.prototype.GetParticleLifetime = function (index) {
        // DEBUG: b2Assert(this.ValidateParticleIndex(index));
        return this.ExpirationTimeToLifetime(this.GetExpirationTimeBuffer()[index]);
    };
    /**
     * Enable / disable destruction of particles in CreateParticle()
     * when no more particles can be created due to a prior call to
     * SetMaxParticleCount().  When this is enabled, the oldest
     * particle is destroyed in CreateParticle() favoring the
     * destruction of particles with a finite lifetime over
     * particles with infinite lifetimes. This feature is enabled by
     * default when particle lifetimes are tracked.  Explicitly
     * enabling this feature using this function enables particle
     * lifetime tracking.
     */
    b2ParticleSystem.prototype.SetDestructionByAge = function (enable) {
        if (enable) {
            this.GetExpirationTimeBuffer();
        }
        this.m_def.destroyByAge = enable;
    };
    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    b2ParticleSystem.prototype.GetDestructionByAge = function () {
        return this.m_def.destroyByAge;
    };
    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    b2ParticleSystem.prototype.GetExpirationTimeBuffer = function () {
        this.m_expirationTimeBuffer.data = this.RequestBuffer(this.m_expirationTimeBuffer.data);
        return this.m_expirationTimeBuffer.data;
    };
    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    b2ParticleSystem.prototype.ExpirationTimeToLifetime = function (expirationTime) {
        return (expirationTime > 0 ?
            expirationTime - this.GetQuantizedTimeElapsed() :
            expirationTime) * this.m_def.lifetimeGranularity;
    };
    /**
     * Get the array of particle indices ordered by reverse
     * lifetime. The oldest particle indexes are at the end of the
     * array with the newest at the start.  Particles with infinite
     * lifetimes (i.e expiration times less than or equal to 0) are
     * placed at the start of the array.
     * ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]) is
     * equivalent to GetParticleLifetime(index).
     *
     * GetParticleCount() items are in the returned array.
     */
    b2ParticleSystem.prototype.GetIndexByExpirationTimeBuffer = function () {
        // If particles are present, initialize / reinitialize the lifetime buffer.
        if (this.GetParticleCount()) {
            this.SetParticleLifetime(0, this.GetParticleLifetime(0));
        }
        else {
            this.m_indexByExpirationTimeBuffer.data = this.RequestBuffer(this.m_indexByExpirationTimeBuffer.data);
        }
        if (!this.m_indexByExpirationTimeBuffer.data) {
            throw new Error();
        }
        return this.m_indexByExpirationTimeBuffer.data;
    };
    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to b2Body::ApplyLinearImpulse.
     *
     * @param index the particle that will be modified.
     * @param impulse impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    b2ParticleSystem.prototype.ParticleApplyLinearImpulse = function (index, impulse) {
        this.ApplyLinearImpulse(index, index + 1, impulse);
    };
    /**
     * Apply an impulse to all particles between 'firstIndex' and
     * 'lastIndex'. This immediately modifies the velocity. Note
     * that the impulse is applied to the total mass of all
     * particles. So, calling ParticleApplyLinearImpulse(0, impulse)
     * and ParticleApplyLinearImpulse(1, impulse) will impart twice
     * as much velocity as calling just ApplyLinearImpulse(0, 1,
     * impulse).
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    b2ParticleSystem.prototype.ApplyLinearImpulse = function (firstIndex, lastIndex, impulse) {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        var numParticles = (lastIndex - firstIndex);
        var totalMass = numParticles * this.GetParticleMass();
        ///const b2Vec2 velocityDelta = impulse / totalMass;
        var velocityDelta = new b2Math_1.b2Vec2().Copy(impulse).SelfMul(1 / totalMass);
        for (var i = firstIndex; i < lastIndex; i++) {
            ///m_velocityBuffer.data[i] += velocityDelta;
            vel_data[i].SelfAdd(velocityDelta);
        }
    };
    b2ParticleSystem.IsSignificantForce = function (force) {
        return force.x !== 0 || force.y !== 0;
    };
    /**
     * Apply a force to the center of a particle.
     *
     * @param index the particle that will be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    b2ParticleSystem.prototype.ParticleApplyForce = function (index, force) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (b2ParticleSystem.IsSignificantForce(force) &&
            this.ForceCanBeApplied(this.m_flagsBuffer.data[index])) {
            this.PrepareForceBuffer();
            ///m_forceBuffer[index] += force;
            this.m_forceBuffer[index].SelfAdd(force);
        }
    };
    /**
     * Distribute a force across several particles. The particles
     * must not be wall particles. Note that the force is
     * distributed across all the particles, so calling this
     * function for indices 0..N is not the same as calling
     * ParticleApplyForce(i, force) for i in 0..N.
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    b2ParticleSystem.prototype.ApplyForce = function (firstIndex, lastIndex, force) {
        // Ensure we're not trying to apply force to particles that can't move,
        // such as wall particles.
        // DEBUG: if (!this.m_flagsBuffer.data) { throw new Error(); }
        // DEBUG: let flags = 0;
        // DEBUG: for (let i = firstIndex; i < lastIndex; i++) {
        // DEBUG:   flags |= this.m_flagsBuffer.data[i];
        // DEBUG: }
        // DEBUG: b2Assert(this.ForceCanBeApplied(flags));
        // Early out if force does nothing (optimization).
        ///const b2Vec2 distributedForce = force / (float32)(lastIndex - firstIndex);
        var distributedForce = new b2Math_1.b2Vec2().Copy(force).SelfMul(1 / (lastIndex - firstIndex));
        if (b2ParticleSystem.IsSignificantForce(distributedForce)) {
            this.PrepareForceBuffer();
            // Distribute the force over all the particles.
            for (var i = firstIndex; i < lastIndex; i++) {
                ///m_forceBuffer[i] += distributedForce;
                this.m_forceBuffer[i].SelfAdd(distributedForce);
            }
        }
    };
    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    b2ParticleSystem.prototype.GetNext = function () {
        return this.m_next;
    };
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     * b2QueryCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    b2ParticleSystem.prototype.QueryAABB = function (callback, aabb) {
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        var beginProxy = 0;
        var endProxy = this.m_proxyBuffer.count;
        var firstProxy = std_lower_bound(this.m_proxyBuffer.data, beginProxy, endProxy, b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.lowerBound.x, this.m_inverseDiameter * aabb.lowerBound.y), b2ParticleSystem.Proxy.CompareProxyTag);
        var lastProxy = std_upper_bound(this.m_proxyBuffer.data, firstProxy, endProxy, b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.upperBound.x, this.m_inverseDiameter * aabb.upperBound.y), b2ParticleSystem.Proxy.CompareTagProxy);
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        for (var k = firstProxy; k < lastProxy; ++k) {
            var proxy = this.m_proxyBuffer.data[k];
            var i = proxy.index;
            var p = pos_data[i];
            if (aabb.lowerBound.x < p.x && p.x < aabb.upperBound.x &&
                aabb.lowerBound.y < p.y && p.y < aabb.upperBound.y) {
                if (!callback.ReportParticle(this, i)) {
                    break;
                }
            }
        }
    };
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided shape's AABB. Calls QueryAABB
     * internally. b2QueryCallback::ShouldQueryParticleSystem is
     * ignored.
     *
     * @param callback a user implemented callback class.
     * @param shape the query shape
     * @param xf the transform of the AABB
     * @param childIndex
     */
    b2ParticleSystem.prototype.QueryShapeAABB = function (callback, shape, xf, childIndex) {
        if (childIndex === void 0) { childIndex = 0; }
        var s_aabb = b2ParticleSystem.QueryShapeAABB_s_aabb;
        var aabb = s_aabb;
        shape.ComputeAABB(aabb, xf, childIndex);
        this.QueryAABB(callback, aabb);
    };
    b2ParticleSystem.prototype.QueryPointAABB = function (callback, point, slop) {
        if (slop === void 0) { slop = b2Settings_1.b2_linearSlop; }
        var s_aabb = b2ParticleSystem.QueryPointAABB_s_aabb;
        var aabb = s_aabb;
        aabb.lowerBound.Set(point.x - slop, point.y - slop);
        aabb.upperBound.Set(point.x + slop, point.y + slop);
        this.QueryAABB(callback, aabb);
    };
    /**
     * Ray-cast the particle system for all particles in the path of
     * the ray. Your callback controls whether you get the closest
     * point, any point, or n-points. The ray-cast ignores particles
     * that contain the starting point.
     * b2RayCastCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    b2ParticleSystem.prototype.RayCast = function (callback, point1, point2) {
        var s_aabb = b2ParticleSystem.RayCast_s_aabb;
        var s_p = b2ParticleSystem.RayCast_s_p;
        var s_v = b2ParticleSystem.RayCast_s_v;
        var s_n = b2ParticleSystem.RayCast_s_n;
        var s_point = b2ParticleSystem.RayCast_s_point;
        if (this.m_proxyBuffer.count === 0) {
            return;
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var aabb = s_aabb;
        b2Math_1.b2Vec2.MinV(point1, point2, aabb.lowerBound);
        b2Math_1.b2Vec2.MaxV(point1, point2, aabb.upperBound);
        var fraction = 1;
        // solving the following equation:
        // ((1-t)*point1+t*point2-position)^2=diameter^2
        // where t is a potential fraction
        ///b2Vec2 v = point2 - point1;
        var v = b2Math_1.b2Vec2.SubVV(point2, point1, s_v);
        var v2 = b2Math_1.b2Vec2.DotVV(v, v);
        var enumerator = this.GetInsideBoundsEnumerator(aabb);
        var i;
        while ((i = enumerator.GetNext()) >= 0) {
            ///b2Vec2 p = point1 - m_positionBuffer.data[i];
            var p = b2Math_1.b2Vec2.SubVV(point1, pos_data[i], s_p);
            var pv = b2Math_1.b2Vec2.DotVV(p, v);
            var p2 = b2Math_1.b2Vec2.DotVV(p, p);
            var determinant = pv * pv - v2 * (p2 - this.m_squaredDiameter);
            if (determinant >= 0) {
                var sqrtDeterminant = b2Math_1.b2Sqrt(determinant);
                // find a solution between 0 and fraction
                var t = (-pv - sqrtDeterminant) / v2;
                if (t > fraction) {
                    continue;
                }
                if (t < 0) {
                    t = (-pv + sqrtDeterminant) / v2;
                    if (t < 0 || t > fraction) {
                        continue;
                    }
                }
                ///b2Vec2 n = p + t * v;
                var n = b2Math_1.b2Vec2.AddVMulSV(p, t, v, s_n);
                n.Normalize();
                ///float32 f = callback.ReportParticle(this, i, point1 + t * v, n, t);
                var f = callback.ReportParticle(this, i, b2Math_1.b2Vec2.AddVMulSV(point1, t, v, s_point), n, t);
                fraction = b2Math_1.b2Min(fraction, f);
                if (fraction <= 0) {
                    break;
                }
            }
        }
    };
    /**
     * Compute the axis-aligned bounding box for all particles
     * contained within this particle system.
     * @param aabb Returns the axis-aligned bounding box of the system.
     */
    b2ParticleSystem.prototype.ComputeAABB = function (aabb) {
        var particleCount = this.GetParticleCount();
        // DEBUG: b2Assert(aabb !== null);
        aabb.lowerBound.x = +b2Settings_1.b2_maxFloat;
        aabb.lowerBound.y = +b2Settings_1.b2_maxFloat;
        aabb.upperBound.x = -b2Settings_1.b2_maxFloat;
        aabb.upperBound.y = -b2Settings_1.b2_maxFloat;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        for (var i = 0; i < particleCount; i++) {
            var p = pos_data[i];
            b2Math_1.b2Vec2.MinV(aabb.lowerBound, p, aabb.lowerBound);
            b2Math_1.b2Vec2.MaxV(aabb.upperBound, p, aabb.upperBound);
        }
        aabb.lowerBound.x -= this.m_particleDiameter;
        aabb.lowerBound.y -= this.m_particleDiameter;
        aabb.upperBound.x += this.m_particleDiameter;
        aabb.upperBound.y += this.m_particleDiameter;
    };
    b2ParticleSystem.prototype.FreeBuffer = function (b, capacity) {
        if (b === null) {
            return;
        }
        b.length = 0;
    };
    b2ParticleSystem.prototype.FreeUserOverridableBuffer = function (b) {
        if (b.userSuppliedCapacity === 0) {
            this.FreeBuffer(b.data, this.m_internalAllocatedCapacity);
        }
    };
    /**
     * Reallocate a buffer
     */
    b2ParticleSystem.prototype.ReallocateBuffer3 = function (oldBuffer, oldCapacity, newCapacity) {
        // b2Assert(newCapacity > oldCapacity);
        if (newCapacity <= oldCapacity) {
            throw new Error();
        }
        var newBuffer = (oldBuffer) ? oldBuffer.slice() : [];
        newBuffer.length = newCapacity;
        return newBuffer;
    };
    /**
     * Reallocate a buffer
     */
    b2ParticleSystem.prototype.ReallocateBuffer5 = function (buffer, userSuppliedCapacity, oldCapacity, newCapacity, deferred) {
        // b2Assert(newCapacity > oldCapacity);
        if (newCapacity <= oldCapacity) {
            throw new Error();
        }
        // A 'deferred' buffer is reallocated only if it is not NULL.
        // If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
        // be kept.
        // b2Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
        if (!(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity)) {
            throw new Error();
        }
        if ((!deferred || buffer) && !userSuppliedCapacity) {
            buffer = this.ReallocateBuffer3(buffer, oldCapacity, newCapacity);
        }
        return buffer; // TODO: fix this
    };
    /**
     * Reallocate a buffer
     */
    b2ParticleSystem.prototype.ReallocateBuffer4 = function (buffer, oldCapacity, newCapacity, deferred) {
        // DEBUG: b2Assert(newCapacity > oldCapacity);
        return this.ReallocateBuffer5(buffer.data, buffer.userSuppliedCapacity, oldCapacity, newCapacity, deferred);
    };
    b2ParticleSystem.prototype.RequestBuffer = function (buffer) {
        if (!buffer) {
            if (this.m_internalAllocatedCapacity === 0) {
                this.ReallocateInternalAllocatedBuffers(b2Settings_1.b2_minParticleSystemBufferCapacity);
            }
            buffer = [];
            buffer.length = this.m_internalAllocatedCapacity;
        }
        return buffer;
    };
    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    b2ParticleSystem.prototype.ReallocateHandleBuffers = function (newCapacity) {
        // DEBUG: b2Assert(newCapacity > this.m_internalAllocatedCapacity);
        // Reallocate a new handle / index map buffer, copying old handle pointers
        // is fine since they're kept around.
        this.m_handleIndexBuffer.data = this.ReallocateBuffer4(this.m_handleIndexBuffer, this.m_internalAllocatedCapacity, newCapacity, true);
        // Set the size of the next handle allocation.
        ///this.m_handleAllocator.SetItemsPerSlab(newCapacity - this.m_internalAllocatedCapacity);
    };
    b2ParticleSystem.prototype.ReallocateInternalAllocatedBuffers = function (capacity) {
        function LimitCapacity(capacity, maxCount) {
            return maxCount && capacity > maxCount ? maxCount : capacity;
        }
        // Don't increase capacity beyond the smallest user-supplied buffer size.
        capacity = LimitCapacity(capacity, this.m_def.maxCount);
        capacity = LimitCapacity(capacity, this.m_flagsBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_positionBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_velocityBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_colorBuffer.userSuppliedCapacity);
        capacity = LimitCapacity(capacity, this.m_userDataBuffer.userSuppliedCapacity);
        if (this.m_internalAllocatedCapacity < capacity) {
            this.ReallocateHandleBuffers(capacity);
            this.m_flagsBuffer.data = this.ReallocateBuffer4(this.m_flagsBuffer, this.m_internalAllocatedCapacity, capacity, false);
            // Conditionally defer these as they are optional if the feature is
            // not enabled.
            var stuck = this.m_stuckThreshold > 0;
            this.m_lastBodyContactStepBuffer.data = this.ReallocateBuffer4(this.m_lastBodyContactStepBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_bodyContactCountBuffer.data = this.ReallocateBuffer4(this.m_bodyContactCountBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_consecutiveContactStepsBuffer.data = this.ReallocateBuffer4(this.m_consecutiveContactStepsBuffer, this.m_internalAllocatedCapacity, capacity, stuck);
            this.m_positionBuffer.data = this.ReallocateBuffer4(this.m_positionBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_velocityBuffer.data = this.ReallocateBuffer4(this.m_velocityBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_forceBuffer = this.ReallocateBuffer5(this.m_forceBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_weightBuffer = this.ReallocateBuffer5(this.m_weightBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_staticPressureBuffer = this.ReallocateBuffer5(this.m_staticPressureBuffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_accumulationBuffer = this.ReallocateBuffer5(this.m_accumulationBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_accumulation2Buffer = this.ReallocateBuffer5(this.m_accumulation2Buffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_depthBuffer = this.ReallocateBuffer5(this.m_depthBuffer, 0, this.m_internalAllocatedCapacity, capacity, true);
            this.m_colorBuffer.data = this.ReallocateBuffer4(this.m_colorBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_groupBuffer = this.ReallocateBuffer5(this.m_groupBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
            this.m_userDataBuffer.data = this.ReallocateBuffer4(this.m_userDataBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_expirationTimeBuffer.data = this.ReallocateBuffer4(this.m_expirationTimeBuffer, this.m_internalAllocatedCapacity, capacity, true);
            this.m_indexByExpirationTimeBuffer.data = this.ReallocateBuffer4(this.m_indexByExpirationTimeBuffer, this.m_internalAllocatedCapacity, capacity, false);
            this.m_internalAllocatedCapacity = capacity;
        }
    };
    b2ParticleSystem.prototype.CreateParticleForGroup = function (groupDef, xf, p) {
        var particleDef = new b2Particle_1.b2ParticleDef();
        particleDef.flags = b2Settings_1.b2Maybe(groupDef.flags, 0);
        ///particleDef.position = b2Mul(xf, p);
        b2Math_1.b2Transform.MulXV(xf, p, particleDef.position);
        ///particleDef.velocity =
        ///  groupDef.linearVelocity +
        ///  b2Cross(groupDef.angularVelocity,
        ///      particleDef.position - groupDef.position);
        b2Math_1.b2Vec2.AddVV(b2Settings_1.b2Maybe(groupDef.linearVelocity, b2Math_1.b2Vec2.ZERO), b2Math_1.b2Vec2.CrossSV(b2Settings_1.b2Maybe(groupDef.angularVelocity, 0), b2Math_1.b2Vec2.SubVV(particleDef.position, b2Settings_1.b2Maybe(groupDef.position, b2Math_1.b2Vec2.ZERO), b2Math_1.b2Vec2.s_t0), b2Math_1.b2Vec2.s_t0), particleDef.velocity);
        particleDef.color.Copy(b2Settings_1.b2Maybe(groupDef.color, b2Draw_1.b2Color.ZERO));
        particleDef.lifetime = b2Settings_1.b2Maybe(groupDef.lifetime, 0);
        particleDef.userData = groupDef.userData;
        this.CreateParticle(particleDef);
    };
    b2ParticleSystem.prototype.CreateParticlesStrokeShapeForGroup = function (shape, groupDef, xf) {
        var s_edge = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge;
        var s_d = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d;
        var s_p = b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p;
        var stride = b2Settings_1.b2Maybe(groupDef.stride, 0);
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        var positionOnEdge = 0;
        var childCount = shape.GetChildCount();
        for (var childIndex = 0; childIndex < childCount; childIndex++) {
            var edge = null;
            if (shape.GetType() === b2Shape_1.b2ShapeType.e_edgeShape) {
                edge = shape;
            }
            else {
                // DEBUG: b2Assert(shape.GetType() === b2ShapeType.e_chainShape);
                edge = s_edge;
                shape.GetChildEdge(edge, childIndex);
            }
            var d = b2Math_1.b2Vec2.SubVV(edge.m_vertex2, edge.m_vertex1, s_d);
            var edgeLength = d.Length();
            while (positionOnEdge < edgeLength) {
                ///b2Vec2 p = edge.m_vertex1 + positionOnEdge / edgeLength * d;
                var p = b2Math_1.b2Vec2.AddVMulSV(edge.m_vertex1, positionOnEdge / edgeLength, d, s_p);
                this.CreateParticleForGroup(groupDef, xf, p);
                positionOnEdge += stride;
            }
            positionOnEdge -= edgeLength;
        }
    };
    b2ParticleSystem.prototype.CreateParticlesFillShapeForGroup = function (shape, groupDef, xf) {
        var s_aabb = b2ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb;
        var s_p = b2ParticleSystem.CreateParticlesFillShapeForGroup_s_p;
        var stride = b2Settings_1.b2Maybe(groupDef.stride, 0);
        if (stride === 0) {
            stride = this.GetParticleStride();
        }
        ///b2Transform identity;
        /// identity.SetIdentity();
        var identity = b2Math_1.b2Transform.IDENTITY;
        var aabb = s_aabb;
        // DEBUG: b2Assert(shape.GetChildCount() === 1);
        shape.ComputeAABB(aabb, identity, 0);
        for (var y = Math.floor(aabb.lowerBound.y / stride) * stride; y < aabb.upperBound.y; y += stride) {
            for (var x = Math.floor(aabb.lowerBound.x / stride) * stride; x < aabb.upperBound.x; x += stride) {
                var p = s_p.Set(x, y);
                if (shape.TestPoint(identity, p)) {
                    this.CreateParticleForGroup(groupDef, xf, p);
                }
            }
        }
    };
    b2ParticleSystem.prototype.CreateParticlesWithShapeForGroup = function (shape, groupDef, xf) {
        switch (shape.GetType()) {
            case b2Shape_1.b2ShapeType.e_edgeShape:
            case b2Shape_1.b2ShapeType.e_chainShape:
                this.CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
                break;
            case b2Shape_1.b2ShapeType.e_polygonShape:
            case b2Shape_1.b2ShapeType.e_circleShape:
                this.CreateParticlesFillShapeForGroup(shape, groupDef, xf);
                break;
            default:
                // DEBUG: b2Assert(false);
                break;
        }
    };
    b2ParticleSystem.prototype.CreateParticlesWithShapesForGroup = function (shapes, shapeCount, groupDef, xf) {
        var compositeShape = new b2ParticleSystem.CompositeShape(shapes, shapeCount);
        this.CreateParticlesFillShapeForGroup(compositeShape, groupDef, xf);
    };
    b2ParticleSystem.prototype.CloneParticle = function (oldIndex, group) {
        var def = new b2Particle_1.b2ParticleDef();
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        def.flags = this.m_flagsBuffer.data[oldIndex];
        def.position.Copy(this.m_positionBuffer.data[oldIndex]);
        def.velocity.Copy(this.m_velocityBuffer.data[oldIndex]);
        if (this.m_colorBuffer.data) {
            def.color.Copy(this.m_colorBuffer.data[oldIndex]);
        }
        if (this.m_userDataBuffer.data) {
            def.userData = this.m_userDataBuffer.data[oldIndex];
        }
        def.group = group;
        var newIndex = this.CreateParticle(def);
        if (this.m_handleIndexBuffer.data) {
            var handle = this.m_handleIndexBuffer.data[oldIndex];
            if (handle) {
                handle.SetIndex(newIndex);
            }
            this.m_handleIndexBuffer.data[newIndex] = handle;
            this.m_handleIndexBuffer.data[oldIndex] = null;
        }
        if (this.m_lastBodyContactStepBuffer.data) {
            this.m_lastBodyContactStepBuffer.data[newIndex] =
                this.m_lastBodyContactStepBuffer.data[oldIndex];
        }
        if (this.m_bodyContactCountBuffer.data) {
            this.m_bodyContactCountBuffer.data[newIndex] =
                this.m_bodyContactCountBuffer.data[oldIndex];
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            this.m_consecutiveContactStepsBuffer.data[newIndex] =
                this.m_consecutiveContactStepsBuffer.data[oldIndex];
        }
        if (this.m_hasForce) {
            this.m_forceBuffer[newIndex].Copy(this.m_forceBuffer[oldIndex]);
        }
        if (this.m_staticPressureBuffer) {
            this.m_staticPressureBuffer[newIndex] = this.m_staticPressureBuffer[oldIndex];
        }
        if (this.m_depthBuffer) {
            this.m_depthBuffer[newIndex] = this.m_depthBuffer[oldIndex];
        }
        if (this.m_expirationTimeBuffer.data) {
            this.m_expirationTimeBuffer.data[newIndex] =
                this.m_expirationTimeBuffer.data[oldIndex];
        }
        return newIndex;
    };
    b2ParticleSystem.prototype.DestroyParticlesInGroup = function (group, callDestructionListener) {
        if (callDestructionListener === void 0) { callDestructionListener = false; }
        for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.DestroyParticle(i, callDestructionListener);
        }
    };
    b2ParticleSystem.prototype.DestroyParticleGroup = function (group) {
        // DEBUG: b2Assert(this.m_groupCount > 0);
        // DEBUG: b2Assert(group !== null);
        if (this.m_world.m_destructionListener) {
            this.m_world.m_destructionListener.SayGoodbyeParticleGroup(group);
        }
        this.SetGroupFlags(group, 0);
        for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
            this.m_groupBuffer[i] = null;
        }
        if (group.m_prev) {
            group.m_prev.m_next = group.m_next;
        }
        if (group.m_next) {
            group.m_next.m_prev = group.m_prev;
        }
        if (group === this.m_groupList) {
            this.m_groupList = group.m_next;
        }
        --this.m_groupCount;
    };
    b2ParticleSystem.ParticleCanBeConnected = function (flags, group) {
        return ((flags & (b2Particle_1.b2ParticleFlag.b2_wallParticle | b2Particle_1.b2ParticleFlag.b2_springParticle | b2Particle_1.b2ParticleFlag.b2_elasticParticle)) !== 0) ||
            ((group !== null) && ((group.GetGroupFlags() & b2ParticleGroup_1.b2ParticleGroupFlag.b2_rigidParticleGroup) !== 0));
    };
    b2ParticleSystem.prototype.UpdatePairsAndTriads = function (firstIndex, lastIndex, filter) {
        var s_dab = b2ParticleSystem.UpdatePairsAndTriads_s_dab;
        var s_dbc = b2ParticleSystem.UpdatePairsAndTriads_s_dbc;
        var s_dca = b2ParticleSystem.UpdatePairsAndTriads_s_dca;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        // Create pairs or triads.
        // All particles in each pair/triad should satisfy the following:
        // * firstIndex <= index < lastIndex
        // * don't have b2_zombieParticle
        // * ParticleCanBeConnected returns true
        // * ShouldCreatePair/ShouldCreateTriad returns true
        // Any particles in each pair/triad should satisfy the following:
        // * filter.IsNeeded returns true
        // * have one of k_pairFlags/k_triadsFlags
        // DEBUG: b2Assert(firstIndex <= lastIndex);
        var particleFlags = 0;
        for (var i = firstIndex; i < lastIndex; i++) {
            particleFlags |= this.m_flagsBuffer.data[i];
        }
        if (particleFlags & b2ParticleSystem.k_pairFlags) {
            for (var k = 0; k < this.m_contactBuffer.count; k++) {
                var contact = this.m_contactBuffer.data[k];
                var a = contact.indexA;
                var b = contact.indexB;
                var af = this.m_flagsBuffer.data[a];
                var bf = this.m_flagsBuffer.data[b];
                var groupA = this.m_groupBuffer[a];
                var groupB = this.m_groupBuffer[b];
                if (a >= firstIndex && a < lastIndex &&
                    b >= firstIndex && b < lastIndex &&
                    !((af | bf) & b2Particle_1.b2ParticleFlag.b2_zombieParticle) &&
                    ((af | bf) & b2ParticleSystem.k_pairFlags) &&
                    (filter.IsNecessary(a) || filter.IsNecessary(b)) &&
                    b2ParticleSystem.ParticleCanBeConnected(af, groupA) &&
                    b2ParticleSystem.ParticleCanBeConnected(bf, groupB) &&
                    filter.ShouldCreatePair(a, b)) {
                    ///b2ParticlePair& pair = m_pairBuffer.Append();
                    var pair = this.m_pairBuffer.data[this.m_pairBuffer.Append()];
                    pair.indexA = a;
                    pair.indexB = b;
                    pair.flags = contact.flags;
                    pair.strength = b2Math_1.b2Min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1);
                    ///pair.distance = b2Distance(pos_data[a], pos_data[b]); // TODO: this was wrong!
                    pair.distance = b2Math_1.b2Vec2.DistanceVV(pos_data[a], pos_data[b]);
                }
                ///std::stable_sort(m_pairBuffer.Begin(), m_pairBuffer.End(), ComparePairIndices);
                std_stable_sort(this.m_pairBuffer.data, 0, this.m_pairBuffer.count, b2ParticleSystem.ComparePairIndices);
                ///m_pairBuffer.Unique(MatchPairIndices);
                this.m_pairBuffer.Unique(b2ParticleSystem.MatchPairIndices);
            }
        }
        if (particleFlags & b2ParticleSystem.k_triadFlags) {
            var diagram = new b2VoronoiDiagram_1.b2VoronoiDiagram(lastIndex - firstIndex);
            ///let necessary_count = 0;
            for (var i = firstIndex; i < lastIndex; i++) {
                var flags = this.m_flagsBuffer.data[i];
                var group = this.m_groupBuffer[i];
                if (!(flags & b2Particle_1.b2ParticleFlag.b2_zombieParticle) &&
                    b2ParticleSystem.ParticleCanBeConnected(flags, group)) {
                    ///if (filter.IsNecessary(i)) {
                    ///++necessary_count;
                    ///}
                    diagram.AddGenerator(pos_data[i], i, filter.IsNecessary(i));
                }
            }
            ///if (necessary_count === 0) {
            /////debugger;
            ///for (let i = firstIndex; i < lastIndex; i++) {
            ///  filter.IsNecessary(i);
            ///}
            ///}
            var stride = this.GetParticleStride();
            diagram.Generate(stride / 2, stride * 2);
            var system_1 = this;
            var callback = /*UpdateTriadsCallback*/ function (a, b, c) {
                if (!system_1.m_flagsBuffer.data) {
                    throw new Error();
                }
                var af = system_1.m_flagsBuffer.data[a];
                var bf = system_1.m_flagsBuffer.data[b];
                var cf = system_1.m_flagsBuffer.data[c];
                if (((af | bf | cf) & b2ParticleSystem.k_triadFlags) &&
                    filter.ShouldCreateTriad(a, b, c)) {
                    var pa = pos_data[a];
                    var pb = pos_data[b];
                    var pc = pos_data[c];
                    var dab = b2Math_1.b2Vec2.SubVV(pa, pb, s_dab);
                    var dbc = b2Math_1.b2Vec2.SubVV(pb, pc, s_dbc);
                    var dca = b2Math_1.b2Vec2.SubVV(pc, pa, s_dca);
                    var maxDistanceSquared = b2Settings_1.b2_maxTriadDistanceSquared * system_1.m_squaredDiameter;
                    if (b2Math_1.b2Vec2.DotVV(dab, dab) > maxDistanceSquared ||
                        b2Math_1.b2Vec2.DotVV(dbc, dbc) > maxDistanceSquared ||
                        b2Math_1.b2Vec2.DotVV(dca, dca) > maxDistanceSquared) {
                        return;
                    }
                    var groupA = system_1.m_groupBuffer[a];
                    var groupB = system_1.m_groupBuffer[b];
                    var groupC = system_1.m_groupBuffer[c];
                    ///b2ParticleTriad& triad = m_system.m_triadBuffer.Append();
                    var triad = system_1.m_triadBuffer.data[system_1.m_triadBuffer.Append()];
                    triad.indexA = a;
                    triad.indexB = b;
                    triad.indexC = c;
                    triad.flags = af | bf | cf;
                    triad.strength = b2Math_1.b2Min(b2Math_1.b2Min(groupA ? groupA.m_strength : 1, groupB ? groupB.m_strength : 1), groupC ? groupC.m_strength : 1);
                    ///let midPoint = b2Vec2.MulSV(1.0 / 3.0, b2Vec2.AddVV(pa, b2Vec2.AddVV(pb, pc, new b2Vec2()), new b2Vec2()), new b2Vec2());
                    var midPoint_x = (pa.x + pb.x + pc.x) / 3.0;
                    var midPoint_y = (pa.y + pb.y + pc.y) / 3.0;
                    ///triad.pa = b2Vec2.SubVV(pa, midPoint, new b2Vec2());
                    triad.pa.x = pa.x - midPoint_x;
                    triad.pa.y = pa.y - midPoint_y;
                    ///triad.pb = b2Vec2.SubVV(pb, midPoint, new b2Vec2());
                    triad.pb.x = pb.x - midPoint_x;
                    triad.pb.y = pb.y - midPoint_y;
                    ///triad.pc = b2Vec2.SubVV(pc, midPoint, new b2Vec2());
                    triad.pc.x = pc.x - midPoint_x;
                    triad.pc.y = pc.y - midPoint_y;
                    triad.ka = -b2Math_1.b2Vec2.DotVV(dca, dab);
                    triad.kb = -b2Math_1.b2Vec2.DotVV(dab, dbc);
                    triad.kc = -b2Math_1.b2Vec2.DotVV(dbc, dca);
                    triad.s = b2Math_1.b2Vec2.CrossVV(pa, pb) + b2Math_1.b2Vec2.CrossVV(pb, pc) + b2Math_1.b2Vec2.CrossVV(pc, pa);
                }
            };
            diagram.GetNodes(callback);
            ///std::stable_sort(m_triadBuffer.Begin(), m_triadBuffer.End(), CompareTriadIndices);
            std_stable_sort(this.m_triadBuffer.data, 0, this.m_triadBuffer.count, b2ParticleSystem.CompareTriadIndices);
            ///m_triadBuffer.Unique(MatchTriadIndices);
            this.m_triadBuffer.Unique(b2ParticleSystem.MatchTriadIndices);
        }
    };
    b2ParticleSystem.prototype.UpdatePairsAndTriadsWithReactiveParticles = function () {
        var filter = new b2ParticleSystem.ReactiveFilter(this.m_flagsBuffer);
        this.UpdatePairsAndTriads(0, this.m_count, filter);
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        for (var i = 0; i < this.m_count; i++) {
            this.m_flagsBuffer.data[i] &= ~b2Particle_1.b2ParticleFlag.b2_reactiveParticle;
        }
        this.m_allParticleFlags &= ~b2Particle_1.b2ParticleFlag.b2_reactiveParticle;
    };
    b2ParticleSystem.ComparePairIndices = function (a, b) {
        var diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        return a.indexB < b.indexB;
    };
    b2ParticleSystem.MatchPairIndices = function (a, b) {
        return a.indexA === b.indexA && a.indexB === b.indexB;
    };
    b2ParticleSystem.CompareTriadIndices = function (a, b) {
        var diffA = a.indexA - b.indexA;
        if (diffA !== 0) {
            return diffA < 0;
        }
        var diffB = a.indexB - b.indexB;
        if (diffB !== 0) {
            return diffB < 0;
        }
        return a.indexC < b.indexC;
    };
    b2ParticleSystem.MatchTriadIndices = function (a, b) {
        return a.indexA === b.indexA && a.indexB === b.indexB && a.indexC === b.indexC;
    };
    b2ParticleSystem.InitializeParticleLists = function (group, nodeBuffer) {
        var bufferIndex = group.GetBufferIndex();
        var particleCount = group.GetParticleCount();
        for (var i = 0; i < particleCount; i++) {
            var node = nodeBuffer[i];
            node.list = node;
            node.next = null;
            node.count = 1;
            node.index = i + bufferIndex;
        }
    };
    b2ParticleSystem.prototype.MergeParticleListsInContact = function (group, nodeBuffer) {
        var bufferIndex = group.GetBufferIndex();
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            /*const b2ParticleContact&*/
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            if (!group.ContainsParticle(a) || !group.ContainsParticle(b)) {
                continue;
            }
            var listA = nodeBuffer[a - bufferIndex].list;
            var listB = nodeBuffer[b - bufferIndex].list;
            if (listA === listB) {
                continue;
            }
            // To minimize the cost of insertion, make sure listA is longer than
            // listB.
            if (listA.count < listB.count) {
                var _tmp = listA;
                listA = listB;
                listB = _tmp; ///b2Swap(listA, listB);
            }
            // DEBUG: b2Assert(listA.count >= listB.count);
            b2ParticleSystem.MergeParticleLists(listA, listB);
        }
    };
    b2ParticleSystem.MergeParticleLists = function (listA, listB) {
        // Insert listB between index 0 and 1 of listA
        // Example:
        //     listA => a1 => a2 => a3 => null
        //     listB => b1 => b2 => null
        // to
        //     listA => listB => b1 => b2 => a1 => a2 => a3 => null
        // DEBUG: b2Assert(listA !== listB);
        for (var b = listB;;) {
            b.list = listA;
            var nextB = b.next;
            if (nextB) {
                b = nextB;
            }
            else {
                b.next = listA.next;
                break;
            }
        }
        listA.next = listB;
        listA.count += listB.count;
        listB.count = 0;
    };
    b2ParticleSystem.FindLongestParticleList = function (group, nodeBuffer) {
        var particleCount = group.GetParticleCount();
        var result = nodeBuffer[0];
        for (var i = 0; i < particleCount; i++) {
            var node = nodeBuffer[i];
            if (result.count < node.count) {
                result = node;
            }
        }
        return result;
    };
    b2ParticleSystem.prototype.MergeZombieParticleListNodes = function (group, nodeBuffer, survivingList) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        var particleCount = group.GetParticleCount();
        for (var i = 0; i < particleCount; i++) {
            var node = nodeBuffer[i];
            if (node !== survivingList &&
                (this.m_flagsBuffer.data[node.index] & b2Particle_1.b2ParticleFlag.b2_zombieParticle)) {
                b2ParticleSystem.MergeParticleListAndNode(survivingList, node);
            }
        }
    };
    b2ParticleSystem.MergeParticleListAndNode = function (list, node) {
        // Insert node between index 0 and 1 of list
        // Example:
        //     list => a1 => a2 => a3 => null
        //     node => null
        // to
        //     list => node => a1 => a2 => a3 => null
        // DEBUG: b2Assert(node !== list);
        // DEBUG: b2Assert(node.list === node);
        // DEBUG: b2Assert(node.count === 1);
        node.list = list;
        node.next = list.next;
        list.next = node;
        list.count++;
        node.count = 0;
    };
    b2ParticleSystem.prototype.CreateParticleGroupsFromParticleList = function (group, nodeBuffer, survivingList) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        var particleCount = group.GetParticleCount();
        var def = new b2ParticleGroup_1.b2ParticleGroupDef();
        def.groupFlags = group.GetGroupFlags();
        def.userData = group.GetUserData();
        for (var i = 0; i < particleCount; i++) {
            var list = nodeBuffer[i];
            if (!list.count || list === survivingList) {
                continue;
            }
            // DEBUG: b2Assert(list.list === list);
            var newGroup = this.CreateParticleGroup(def);
            for (var node = list; node; node = node.next) {
                var oldIndex = node.index;
                // DEBUG: const flags = this.m_flagsBuffer.data[oldIndex];
                // DEBUG: b2Assert(!(flags & b2ParticleFlag.b2_zombieParticle));
                var newIndex = this.CloneParticle(oldIndex, newGroup);
                this.m_flagsBuffer.data[oldIndex] |= b2Particle_1.b2ParticleFlag.b2_zombieParticle;
                node.index = newIndex;
            }
        }
    };
    b2ParticleSystem.prototype.UpdatePairsAndTriadsWithParticleList = function (group, nodeBuffer) {
        var bufferIndex = group.GetBufferIndex();
        // Update indices in pairs and triads. If an index belongs to the group,
        // replace it with the corresponding value in nodeBuffer.
        // Note that nodeBuffer is allocated only for the group and the index should
        // be shifted by bufferIndex.
        for (var k = 0; k < this.m_pairBuffer.count; k++) {
            var pair = this.m_pairBuffer.data[k];
            var a = pair.indexA;
            var b = pair.indexB;
            if (group.ContainsParticle(a)) {
                pair.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                pair.indexB = nodeBuffer[b - bufferIndex].index;
            }
        }
        for (var k = 0; k < this.m_triadBuffer.count; k++) {
            var triad = this.m_triadBuffer.data[k];
            var a = triad.indexA;
            var b = triad.indexB;
            var c = triad.indexC;
            if (group.ContainsParticle(a)) {
                triad.indexA = nodeBuffer[a - bufferIndex].index;
            }
            if (group.ContainsParticle(b)) {
                triad.indexB = nodeBuffer[b - bufferIndex].index;
            }
            if (group.ContainsParticle(c)) {
                triad.indexC = nodeBuffer[c - bufferIndex].index;
            }
        }
    };
    b2ParticleSystem.prototype.ComputeDepth = function () {
        ///b2ParticleContact* contactGroups = (b2ParticleContact*) this.m_world.m_stackAllocator.Allocate(sizeof(b2ParticleContact) * this.m_contactBuffer.GetCount());
        var contactGroups = []; // TODO: static
        var contactGroupsCount = 0;
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var groupA = this.m_groupBuffer[a];
            var groupB = this.m_groupBuffer[b];
            if (groupA && groupA === groupB &&
                (groupA.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth)) {
                contactGroups[contactGroupsCount++] = contact;
            }
        }
        ///b2ParticleGroup** groupsToUpdate = (b2ParticleGroup**) this.m_world.m_stackAllocator.Allocate(sizeof(b2ParticleGroup*) * this.m_groupCount);
        var groupsToUpdate = []; // TODO: static
        var groupsToUpdateCount = 0;
        for (var group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth) {
                groupsToUpdate[groupsToUpdateCount++] = group;
                this.SetGroupFlags(group, group.m_groupFlags &
                    ~b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth);
                for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // Compute sum of weight of contacts except between different groups.
        for (var k = 0; k < contactGroupsCount; k++) {
            var contact = contactGroups[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var w = contact.weight;
            this.m_accumulationBuffer[a] += w;
            this.m_accumulationBuffer[b] += w;
        }
        // DEBUG: b2Assert(this.m_depthBuffer !== null);
        for (var i = 0; i < groupsToUpdateCount; i++) {
            var group = groupsToUpdate[i];
            for (var i_1 = group.m_firstIndex; i_1 < group.m_lastIndex; i_1++) {
                var w = this.m_accumulationBuffer[i_1];
                this.m_depthBuffer[i_1] = w < 0.8 ? 0 : b2Settings_1.b2_maxFloat;
            }
        }
        // The number of iterations is equal to particle number from the deepest
        // particle to the nearest surface particle, and in general it is smaller
        // than sqrt of total particle number.
        ///int32 iterationCount = (int32)b2Sqrt((float)m_count);
        var iterationCount = b2Math_1.b2Sqrt(this.m_count) >> 0;
        for (var t = 0; t < iterationCount; t++) {
            var updated = false;
            for (var k = 0; k < contactGroupsCount; k++) {
                var contact = contactGroups[k];
                var a = contact.indexA;
                var b = contact.indexB;
                var r = 1 - contact.weight;
                ///float32& ap0 = m_depthBuffer[a];
                var ap0 = this.m_depthBuffer[a];
                ///float32& bp0 = m_depthBuffer[b];
                var bp0 = this.m_depthBuffer[b];
                var ap1 = bp0 + r;
                var bp1 = ap0 + r;
                if (ap0 > ap1) {
                    ///ap0 = ap1;
                    this.m_depthBuffer[a] = ap1;
                    updated = true;
                }
                if (bp0 > bp1) {
                    ///bp0 = bp1;
                    this.m_depthBuffer[b] = bp1;
                    updated = true;
                }
            }
            if (!updated) {
                break;
            }
        }
        for (var i = 0; i < groupsToUpdateCount; i++) {
            var group = groupsToUpdate[i];
            for (var i_2 = group.m_firstIndex; i_2 < group.m_lastIndex; i_2++) {
                if (this.m_depthBuffer[i_2] < b2Settings_1.b2_maxFloat) {
                    this.m_depthBuffer[i_2] *= this.m_particleDiameter;
                }
                else {
                    this.m_depthBuffer[i_2] = 0;
                }
            }
        }
        ///this.m_world.m_stackAllocator.Free(groupsToUpdate);
        ///this.m_world.m_stackAllocator.Free(contactGroups);
    };
    b2ParticleSystem.prototype.GetInsideBoundsEnumerator = function (aabb) {
        var lowerTag = b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.lowerBound.x - 1, this.m_inverseDiameter * aabb.lowerBound.y - 1);
        var upperTag = b2ParticleSystem.computeTag(this.m_inverseDiameter * aabb.upperBound.x + 1, this.m_inverseDiameter * aabb.upperBound.y + 1);
        ///const Proxy* beginProxy = m_proxyBuffer.Begin();
        var beginProxy = 0;
        ///const Proxy* endProxy = m_proxyBuffer.End();
        var endProxy = this.m_proxyBuffer.count;
        ///const Proxy* firstProxy = std::lower_bound(beginProxy, endProxy, lowerTag);
        var firstProxy = std_lower_bound(this.m_proxyBuffer.data, beginProxy, endProxy, lowerTag, b2ParticleSystem.Proxy.CompareProxyTag);
        ///const Proxy* lastProxy = std::upper_bound(firstProxy, endProxy, upperTag);
        var lastProxy = std_upper_bound(this.m_proxyBuffer.data, beginProxy, endProxy, upperTag, b2ParticleSystem.Proxy.CompareTagProxy);
        // DEBUG: b2Assert(beginProxy <= firstProxy);
        // DEBUG: b2Assert(firstProxy <= lastProxy);
        // DEBUG: b2Assert(lastProxy <= endProxy);
        return new b2ParticleSystem.InsideBoundsEnumerator(this, lowerTag, upperTag, firstProxy, lastProxy);
    };
    b2ParticleSystem.prototype.UpdateAllParticleFlags = function () {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        this.m_allParticleFlags = 0;
        for (var i = 0; i < this.m_count; i++) {
            this.m_allParticleFlags |= this.m_flagsBuffer.data[i];
        }
        this.m_needsUpdateAllParticleFlags = false;
    };
    b2ParticleSystem.prototype.UpdateAllGroupFlags = function () {
        this.m_allGroupFlags = 0;
        for (var group = this.m_groupList; group; group = group.GetNext()) {
            this.m_allGroupFlags |= group.m_groupFlags;
        }
        this.m_needsUpdateAllGroupFlags = false;
    };
    b2ParticleSystem.prototype.AddContact = function (a, b, contacts) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var s_d = b2ParticleSystem.AddContact_s_d;
        var pos_data = this.m_positionBuffer.data;
        // DEBUG: b2Assert(contacts === this.m_contactBuffer);
        ///b2Vec2 d = m_positionBuffer.data[b] - m_positionBuffer.data[a];
        var d = b2Math_1.b2Vec2.SubVV(pos_data[b], pos_data[a], s_d);
        var distBtParticlesSq = b2Math_1.b2Vec2.DotVV(d, d);
        if (distBtParticlesSq < this.m_squaredDiameter) {
            var invD = b2Math_1.b2InvSqrt(distBtParticlesSq);
            if (!isFinite(invD)) {
                invD = 1.98177537e+019;
            }
            ///b2ParticleContact& contact = contacts.Append();
            var contact = this.m_contactBuffer.data[this.m_contactBuffer.Append()];
            contact.indexA = a;
            contact.indexB = b;
            contact.flags = this.m_flagsBuffer.data[a] | this.m_flagsBuffer.data[b];
            contact.weight = 1 - distBtParticlesSq * invD * this.m_inverseDiameter;
            ///contact.SetNormal(invD * d);
            b2Math_1.b2Vec2.MulSV(invD, d, contact.normal);
        }
    };
    b2ParticleSystem.prototype.FindContacts_Reference = function (contacts) {
        // DEBUG: b2Assert(contacts === this.m_contactBuffer);
        var beginProxy = 0;
        var endProxy = this.m_proxyBuffer.count;
        this.m_contactBuffer.count = 0;
        for (var a = beginProxy, c = beginProxy; a < endProxy; a++) {
            var rightTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 0);
            for (var b = a + 1; b < endProxy; b++) {
                if (rightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index, this.m_contactBuffer);
            }
            var bottomLeftTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, -1, 1);
            for (; c < endProxy; c++) {
                if (bottomLeftTag <= this.m_proxyBuffer.data[c].tag) {
                    break;
                }
            }
            var bottomRightTag = b2ParticleSystem.computeRelativeTag(this.m_proxyBuffer.data[a].tag, 1, 1);
            for (var b = c; b < endProxy; b++) {
                if (bottomRightTag < this.m_proxyBuffer.data[b].tag) {
                    break;
                }
                this.AddContact(this.m_proxyBuffer.data[a].index, this.m_proxyBuffer.data[b].index, this.m_contactBuffer);
            }
        }
    };
    ///void ReorderForFindContact(FindContactInput* reordered, int alignedCount) const;
    ///void GatherChecksOneParticle(const uint32 bound, const int startIndex, const int particleIndex, int* nextUncheckedIndex, b2GrowableBuffer<FindContactCheck>& checks) const;
    ///void GatherChecks(b2GrowableBuffer<FindContactCheck>& checks) const;
    ///void FindContacts_Simd(b2GrowableBuffer<b2ParticleContact>& contacts) const;
    b2ParticleSystem.prototype.FindContacts = function (contacts) {
        this.FindContacts_Reference(contacts);
    };
    ///static void UpdateProxyTags(const uint32* const tags, b2GrowableBuffer<Proxy>& proxies);
    ///static bool ProxyBufferHasIndex(int32 index, const Proxy* const a, int count);
    ///static int NumProxiesWithSameTag(const Proxy* const a, const Proxy* const b, int count);
    ///static bool AreProxyBuffersTheSame(const b2GrowableBuffer<Proxy>& a, const b2GrowableBuffer<Proxy>& b);
    b2ParticleSystem.prototype.UpdateProxies_Reference = function (proxies) {
        // DEBUG: b2Assert(proxies === this.m_proxyBuffer);
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var inv_diam = this.m_inverseDiameter;
        for (var k = 0; k < this.m_proxyBuffer.count; ++k) {
            var proxy = this.m_proxyBuffer.data[k];
            var i = proxy.index;
            var p = pos_data[i];
            proxy.tag = b2ParticleSystem.computeTag(inv_diam * p.x, inv_diam * p.y);
        }
    };
    ///void UpdateProxies_Simd(b2GrowableBuffer<Proxy>& proxies) const;
    b2ParticleSystem.prototype.UpdateProxies = function (proxies) {
        this.UpdateProxies_Reference(proxies);
    };
    b2ParticleSystem.prototype.SortProxies = function (proxies) {
        // DEBUG: b2Assert(proxies === this.m_proxyBuffer);
        ///std::sort(proxies.Begin(), proxies.End());
        std_sort(this.m_proxyBuffer.data, 0, this.m_proxyBuffer.count, b2ParticleSystem.Proxy.CompareProxyProxy);
    };
    b2ParticleSystem.prototype.FilterContacts = function (contacts) {
        // Optionally filter the contact.
        var contactFilter = this.GetParticleContactFilter();
        if (contactFilter === null) {
            return;
        }
        /// contacts.RemoveIf(b2ParticleContactRemovePredicate(this, contactFilter));
        // DEBUG: b2Assert(contacts === this.m_contactBuffer);
        var system = this;
        var predicate = function (contact) {
            return ((contact.flags & b2Particle_1.b2ParticleFlag.b2_particleContactFilterParticle) !== 0) && !contactFilter.ShouldCollideParticleParticle(system, contact.indexA, contact.indexB);
        };
        this.m_contactBuffer.RemoveIf(predicate);
    };
    b2ParticleSystem.prototype.NotifyContactListenerPreContact = function (particlePairs) {
        var contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }
        ///particlePairs.Initialize(m_contactBuffer.Begin(), m_contactBuffer.GetCount(), GetFlagsBuffer());
        particlePairs.Initialize(this.m_contactBuffer, this.m_flagsBuffer);
        throw new Error(); // TODO: notify
    };
    b2ParticleSystem.prototype.NotifyContactListenerPostContact = function (particlePairs) {
        var contactListener = this.GetParticleContactListener();
        if (contactListener === null) {
            return;
        }
        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        ///const b2ParticleContact* const endContact = m_contactBuffer.End();
        ///for (b2ParticleContact* contact = m_contactBuffer.Begin(); contact < endContact; ++contact)
        for (var k = 0; k < this.m_contactBuffer.count; ++k) {
            var contact = this.m_contactBuffer.data[k];
            ///ParticlePair pair;
            ///pair.first = contact.GetIndexA();
            ///pair.second = contact.GetIndexB();
            ///const int32 itemIndex = particlePairs.Find(pair);
            var itemIndex = -1; // TODO
            if (itemIndex >= 0) {
                // Already touching, ignore this contact.
                particlePairs.Invalidate(itemIndex);
            }
            else {
                // Just started touching, inform the listener.
                contactListener.BeginContactParticleParticle(this, contact);
            }
        }
        // Report particles that are no longer touching.
        // That is, any pairs that were not invalidated above.
        ///const int32 pairCount = particlePairs.GetCount();
        ///const ParticlePair* const pairs = particlePairs.GetBuffer();
        ///const int8* const valid = particlePairs.GetValidBuffer();
        ///for (int32 i = 0; i < pairCount; ++i)
        ///{
        ///  if (valid[i])
        ///  {
        ///    contactListener.EndContactParticleParticle(this, pairs[i].first, pairs[i].second);
        ///  }
        ///}
        throw new Error(); // TODO: notify
    };
    b2ParticleSystem.b2ParticleContactIsZombie = function (contact) {
        return (contact.flags & b2Particle_1.b2ParticleFlag.b2_zombieParticle) === b2Particle_1.b2ParticleFlag.b2_zombieParticle;
    };
    b2ParticleSystem.prototype.UpdateContacts = function (exceptZombie) {
        this.UpdateProxies(this.m_proxyBuffer);
        this.SortProxies(this.m_proxyBuffer);
        ///b2ParticlePairSet particlePairs(&this.m_world.m_stackAllocator);
        var particlePairs = new b2ParticleSystem.b2ParticlePairSet(); // TODO: static
        this.NotifyContactListenerPreContact(particlePairs);
        this.FindContacts(this.m_contactBuffer);
        this.FilterContacts(this.m_contactBuffer);
        this.NotifyContactListenerPostContact(particlePairs);
        if (exceptZombie) {
            this.m_contactBuffer.RemoveIf(b2ParticleSystem.b2ParticleContactIsZombie);
        }
    };
    b2ParticleSystem.prototype.NotifyBodyContactListenerPreContact = function (fixtureSet) {
        var contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }
        ///fixtureSet.Initialize(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.GetCount(), GetFlagsBuffer());
        fixtureSet.Initialize(this.m_bodyContactBuffer, this.m_flagsBuffer);
        throw new Error(); // TODO: notify
    };
    b2ParticleSystem.prototype.NotifyBodyContactListenerPostContact = function (fixtureSet) {
        var contactListener = this.GetFixtureContactListener();
        if (contactListener === null) {
            return;
        }
        // Loop through all new contacts, reporting any new ones, and
        // "invalidating" the ones that still exist.
        ///for (b2ParticleBodyContact* contact = m_bodyContactBuffer.Begin(); contact !== m_bodyContactBuffer.End(); ++contact)
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            // DEBUG: b2Assert(contact !== null);
            ///FixtureParticle fixtureParticleToFind;
            ///fixtureParticleToFind.first = contact.fixture;
            ///fixtureParticleToFind.second = contact.index;
            ///const int32 index = fixtureSet.Find(fixtureParticleToFind);
            var index = -1; // TODO
            if (index >= 0) {
                // Already touching remove this from the set.
                fixtureSet.Invalidate(index);
            }
            else {
                // Just started touching, report it!
                contactListener.BeginContactFixtureParticle(this, contact);
            }
        }
        // If the contact listener is enabled, report all fixtures that are no
        // longer in contact with particles.
        ///const FixtureParticle* const fixtureParticles = fixtureSet.GetBuffer();
        ///const int8* const fixtureParticlesValid = fixtureSet.GetValidBuffer();
        ///const int32 fixtureParticleCount = fixtureSet.GetCount();
        ///for (int32 i = 0; i < fixtureParticleCount; ++i)
        ///{
        ///  if (fixtureParticlesValid[i])
        ///  {
        ///    const FixtureParticle* const fixtureParticle = &fixtureParticles[i];
        ///    contactListener.EndContactFixtureParticle(fixtureParticle.first, this, fixtureParticle.second);
        ///  }
        ///}
        throw new Error(); // TODO: notify
    };
    b2ParticleSystem.prototype.UpdateBodyContacts = function () {
        var s_aabb = b2ParticleSystem.UpdateBodyContacts_s_aabb;
        // If the particle contact listener is enabled, generate a set of
        // fixture / particle contacts.
        ///FixtureParticleSet fixtureSet(&m_world.m_stackAllocator);
        var fixtureSet = new b2ParticleSystem.FixtureParticleSet(); // TODO: static
        this.NotifyBodyContactListenerPreContact(fixtureSet);
        if (this.m_stuckThreshold > 0) {
            if (!this.m_bodyContactCountBuffer.data) {
                throw new Error();
            }
            if (!this.m_lastBodyContactStepBuffer.data) {
                throw new Error();
            }
            if (!this.m_consecutiveContactStepsBuffer.data) {
                throw new Error();
            }
            var particleCount = this.GetParticleCount();
            for (var i = 0; i < particleCount; i++) {
                // Detect stuck particles, see comment in
                // b2ParticleSystem::DetectStuckParticle()
                this.m_bodyContactCountBuffer.data[i] = 0;
                if (this.m_timestamp > (this.m_lastBodyContactStepBuffer.data[i] + 1)) {
                    this.m_consecutiveContactStepsBuffer.data[i] = 0;
                }
            }
        }
        this.m_bodyContactBuffer.SetCount(0);
        this.m_stuckParticleBuffer.SetCount(0);
        var aabb = s_aabb;
        this.ComputeAABB(aabb);
        var callback = new b2ParticleSystem.UpdateBodyContactsCallback(this, this.GetFixtureContactFilter());
        this.m_world.QueryAABB(callback, aabb);
        if (this.m_def.strictContactCheck) {
            this.RemoveSpuriousBodyContacts();
        }
        this.NotifyBodyContactListenerPostContact(fixtureSet);
    };
    b2ParticleSystem.prototype.Solve = function (step) {
        var s_subStep = b2ParticleSystem.Solve_s_subStep;
        if (this.m_count === 0) {
            return;
        }
        // If particle lifetimes are enabled, destroy particles that are too old.
        if (this.m_expirationTimeBuffer.data) {
            this.SolveLifetimes(step);
        }
        if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_zombieParticle) {
            this.SolveZombie();
        }
        if (this.m_needsUpdateAllParticleFlags) {
            this.UpdateAllParticleFlags();
        }
        if (this.m_needsUpdateAllGroupFlags) {
            this.UpdateAllGroupFlags();
        }
        if (this.m_paused) {
            return;
        }
        for (this.m_iterationIndex = 0; this.m_iterationIndex < step.particleIterations; this.m_iterationIndex++) {
            ++this.m_timestamp;
            var subStep = s_subStep.Copy(step);
            subStep.dt /= step.particleIterations;
            subStep.inv_dt *= step.particleIterations;
            this.UpdateContacts(false);
            this.UpdateBodyContacts();
            this.ComputeWeight();
            if (this.m_allGroupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth) {
                this.ComputeDepth();
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_reactiveParticle) {
                this.UpdatePairsAndTriadsWithReactiveParticles();
            }
            if (this.m_hasForce) {
                this.SolveForce(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_viscousParticle) {
                this.SolveViscous();
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_repulsiveParticle) {
                this.SolveRepulsive(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_powderParticle) {
                this.SolvePowder(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_tensileParticle) {
                this.SolveTensile(subStep);
            }
            if (this.m_allGroupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_solidParticleGroup) {
                this.SolveSolid(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_colorMixingParticle) {
                this.SolveColorMixing();
            }
            this.SolveGravity(subStep);
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_staticPressureParticle) {
                this.SolveStaticPressure(subStep);
            }
            this.SolvePressure(subStep);
            this.SolveDamping(subStep);
            if (this.m_allParticleFlags & b2ParticleSystem.k_extraDampingFlags) {
                this.SolveExtraDamping();
            }
            // SolveElastic and SolveSpring refer the current velocities for
            // numerical stability, they should be called as late as possible.
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_elasticParticle) {
                this.SolveElastic(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_springParticle) {
                this.SolveSpring(subStep);
            }
            this.LimitVelocity(subStep);
            if (this.m_allGroupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_rigidParticleGroup) {
                this.SolveRigidDamping();
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_barrierParticle) {
                this.SolveBarrier(subStep);
            }
            // SolveCollision, SolveRigid and SolveWall should be called after
            // other force functions because they may require particles to have
            // specific velocities.
            this.SolveCollision(subStep);
            if (this.m_allGroupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_rigidParticleGroup) {
                this.SolveRigid(subStep);
            }
            if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_wallParticle) {
                this.SolveWall();
            }
            // The particle positions can be updated only at the end of substep.
            if (!this.m_positionBuffer.data) {
                throw new Error();
            }
            if (!this.m_velocityBuffer.data) {
                throw new Error();
            }
            for (var i = 0; i < this.m_count; i++) {
                ///m_positionBuffer.data[i] += subStep.dt * m_velocityBuffer.data[i];
                this.m_positionBuffer.data[i].SelfMulAdd(subStep.dt, this.m_velocityBuffer.data[i]);
            }
        }
    };
    b2ParticleSystem.prototype.SolveCollision = function (step) {
        var s_aabb = b2ParticleSystem.SolveCollision_s_aabb;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        // This function detects particles which are crossing boundary of bodies
        // and modifies velocities of them so that they will move just in front of
        // boundary. This function function also applies the reaction force to
        // bodies as precisely as the numerical stability is kept.
        var aabb = s_aabb;
        aabb.lowerBound.x = +b2Settings_1.b2_maxFloat;
        aabb.lowerBound.y = +b2Settings_1.b2_maxFloat;
        aabb.upperBound.x = -b2Settings_1.b2_maxFloat;
        aabb.upperBound.y = -b2Settings_1.b2_maxFloat;
        for (var i = 0; i < this.m_count; i++) {
            var v = vel_data[i];
            var p1 = pos_data[i];
            ///let p2 = p1 + step.dt * v;
            var p2_x = p1.x + step.dt * v.x;
            var p2_y = p1.y + step.dt * v.y;
            ///aabb.lowerBound = b2Min(aabb.lowerBound, b2Min(p1, p2));
            aabb.lowerBound.x = b2Math_1.b2Min(aabb.lowerBound.x, b2Math_1.b2Min(p1.x, p2_x));
            aabb.lowerBound.y = b2Math_1.b2Min(aabb.lowerBound.y, b2Math_1.b2Min(p1.y, p2_y));
            ///aabb.upperBound = b2Max(aabb.upperBound, b2Max(p1, p2));
            aabb.upperBound.x = b2Math_1.b2Max(aabb.upperBound.x, b2Math_1.b2Max(p1.x, p2_x));
            aabb.upperBound.y = b2Math_1.b2Max(aabb.upperBound.y, b2Math_1.b2Max(p1.y, p2_y));
        }
        var callback = new b2ParticleSystem.SolveCollisionCallback(this, step);
        this.m_world.QueryAABB(callback, aabb);
    };
    b2ParticleSystem.prototype.LimitVelocity = function (step) {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        var criticalVelocitySquared = this.GetCriticalVelocitySquared(step);
        for (var i = 0; i < this.m_count; i++) {
            var v = vel_data[i];
            var v2 = b2Math_1.b2Vec2.DotVV(v, v);
            if (v2 > criticalVelocitySquared) {
                ///v *= b2Sqrt(criticalVelocitySquared / v2);
                v.SelfMul(b2Math_1.b2Sqrt(criticalVelocitySquared / v2));
            }
        }
    };
    b2ParticleSystem.prototype.SolveGravity = function (step) {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var s_gravity = b2ParticleSystem.SolveGravity_s_gravity;
        var vel_data = this.m_velocityBuffer.data;
        ///b2Vec2 gravity = step.dt * m_def.gravityScale * m_world.GetGravity();
        var gravity = b2Math_1.b2Vec2.MulSV(step.dt * this.m_def.gravityScale, this.m_world.GetGravity(), s_gravity);
        for (var i = 0; i < this.m_count; i++) {
            vel_data[i].SelfAdd(gravity);
        }
    };
    b2ParticleSystem.prototype.SolveBarrier = function (step) {
        var s_aabb = b2ParticleSystem.SolveBarrier_s_aabb;
        var s_va = b2ParticleSystem.SolveBarrier_s_va;
        var s_vb = b2ParticleSystem.SolveBarrier_s_vb;
        var s_pba = b2ParticleSystem.SolveBarrier_s_pba;
        var s_vba = b2ParticleSystem.SolveBarrier_s_vba;
        var s_vc = b2ParticleSystem.SolveBarrier_s_vc;
        var s_pca = b2ParticleSystem.SolveBarrier_s_pca;
        var s_vca = b2ParticleSystem.SolveBarrier_s_vca;
        var s_qba = b2ParticleSystem.SolveBarrier_s_qba;
        var s_qca = b2ParticleSystem.SolveBarrier_s_qca;
        var s_dv = b2ParticleSystem.SolveBarrier_s_dv;
        var s_f = b2ParticleSystem.SolveBarrier_s_f;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        // If a particle is passing between paired barrier particles,
        // its velocity will be decelerated to avoid passing.
        for (var i = 0; i < this.m_count; i++) {
            var flags = this.m_flagsBuffer.data[i];
            ///if ((flags & b2ParticleSystem.k_barrierWallFlags) === b2ParticleSystem.k_barrierWallFlags)
            if ((flags & b2ParticleSystem.k_barrierWallFlags) !== 0) {
                vel_data[i].SetZero();
            }
        }
        var tmax = b2Settings_1.b2_barrierCollisionTime * step.dt;
        var mass = this.GetParticleMass();
        for (var k = 0; k < this.m_pairBuffer.count; k++) {
            var pair = this.m_pairBuffer.data[k];
            if (pair.flags & b2Particle_1.b2ParticleFlag.b2_barrierParticle) {
                var a = pair.indexA;
                var b = pair.indexB;
                var pa = pos_data[a];
                var pb = pos_data[b];
                /// b2AABB aabb;
                var aabb = s_aabb;
                ///aabb.lowerBound = b2Min(pa, pb);
                b2Math_1.b2Vec2.MinV(pa, pb, aabb.lowerBound);
                ///aabb.upperBound = b2Max(pa, pb);
                b2Math_1.b2Vec2.MaxV(pa, pb, aabb.upperBound);
                var aGroup = this.m_groupBuffer[a];
                var bGroup = this.m_groupBuffer[b];
                ///b2Vec2 va = GetLinearVelocity(aGroup, a, pa);
                var va = this.GetLinearVelocity(aGroup, a, pa, s_va);
                ///b2Vec2 vb = GetLinearVelocity(bGroup, b, pb);
                var vb = this.GetLinearVelocity(bGroup, b, pb, s_vb);
                ///b2Vec2 pba = pb - pa;
                var pba = b2Math_1.b2Vec2.SubVV(pb, pa, s_pba);
                ///b2Vec2 vba = vb - va;
                var vba = b2Math_1.b2Vec2.SubVV(vb, va, s_vba);
                ///InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
                var enumerator = this.GetInsideBoundsEnumerator(aabb);
                var c = void 0;
                while ((c = enumerator.GetNext()) >= 0) {
                    var pc = pos_data[c];
                    var cGroup = this.m_groupBuffer[c];
                    if (aGroup !== cGroup && bGroup !== cGroup) {
                        ///b2Vec2 vc = GetLinearVelocity(cGroup, c, pc);
                        var vc = this.GetLinearVelocity(cGroup, c, pc, s_vc);
                        // Solve the equation below:
                        //   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
                        // which expresses that the particle c will pass a line
                        // connecting the particles a and b at the time of t.
                        // if s is between 0 and 1, c will pass between a and b.
                        ///b2Vec2 pca = pc - pa;
                        var pca = b2Math_1.b2Vec2.SubVV(pc, pa, s_pca);
                        ///b2Vec2 vca = vc - va;
                        var vca = b2Math_1.b2Vec2.SubVV(vc, va, s_vca);
                        var e2 = b2Math_1.b2Vec2.CrossVV(vba, vca);
                        var e1 = b2Math_1.b2Vec2.CrossVV(pba, vca) - b2Math_1.b2Vec2.CrossVV(pca, vba);
                        var e0 = b2Math_1.b2Vec2.CrossVV(pba, pca);
                        var s = void 0, t = void 0;
                        ///b2Vec2 qba, qca;
                        var qba = s_qba, qca = s_qca;
                        if (e2 === 0) {
                            if (e1 === 0) {
                                continue;
                            }
                            t = -e0 / e1;
                            if (!(t >= 0 && t < tmax)) {
                                continue;
                            }
                            ///qba = pba + t * vba;
                            b2Math_1.b2Vec2.AddVMulSV(pba, t, vba, qba);
                            ///qca = pca + t * vca;
                            b2Math_1.b2Vec2.AddVMulSV(pca, t, vca, qca);
                            s = b2Math_1.b2Vec2.DotVV(qba, qca) / b2Math_1.b2Vec2.DotVV(qba, qba);
                            if (!(s >= 0 && s <= 1)) {
                                continue;
                            }
                        }
                        else {
                            var det = e1 * e1 - 4 * e0 * e2;
                            if (det < 0) {
                                continue;
                            }
                            var sqrtDet = b2Math_1.b2Sqrt(det);
                            var t1 = (-e1 - sqrtDet) / (2 * e2);
                            var t2 = (-e1 + sqrtDet) / (2 * e2);
                            ///if (t1 > t2) b2Swap(t1, t2);
                            if (t1 > t2) {
                                var tmp = t1;
                                t1 = t2;
                                t2 = tmp;
                            }
                            t = t1;
                            ///qba = pba + t * vba;
                            b2Math_1.b2Vec2.AddVMulSV(pba, t, vba, qba);
                            ///qca = pca + t * vca;
                            b2Math_1.b2Vec2.AddVMulSV(pca, t, vca, qca);
                            ///s = b2Dot(qba, qca) / b2Dot(qba, qba);
                            s = b2Math_1.b2Vec2.DotVV(qba, qca) / b2Math_1.b2Vec2.DotVV(qba, qba);
                            if (!(t >= 0 && t < tmax && s >= 0 && s <= 1)) {
                                t = t2;
                                if (!(t >= 0 && t < tmax)) {
                                    continue;
                                }
                                ///qba = pba + t * vba;
                                b2Math_1.b2Vec2.AddVMulSV(pba, t, vba, qba);
                                ///qca = pca + t * vca;
                                b2Math_1.b2Vec2.AddVMulSV(pca, t, vca, qca);
                                ///s = b2Dot(qba, qca) / b2Dot(qba, qba);
                                s = b2Math_1.b2Vec2.DotVV(qba, qca) / b2Math_1.b2Vec2.DotVV(qba, qba);
                                if (!(s >= 0 && s <= 1)) {
                                    continue;
                                }
                            }
                        }
                        // Apply a force to particle c so that it will have the
                        // interpolated velocity at the collision point on line ab.
                        ///b2Vec2 dv = va + s * vba - vc;
                        var dv = s_dv;
                        dv.x = va.x + s * vba.x - vc.x;
                        dv.y = va.y + s * vba.y - vc.y;
                        ///b2Vec2 f = GetParticleMass() * dv;
                        var f = b2Math_1.b2Vec2.MulSV(mass, dv, s_f);
                        if (cGroup && this.IsRigidGroup(cGroup)) {
                            // If c belongs to a rigid group, the force will be
                            // distributed in the group.
                            var mass_1 = cGroup.GetMass();
                            var inertia = cGroup.GetInertia();
                            if (mass_1 > 0) {
                                ///cGroup.m_linearVelocity += 1 / mass * f;
                                cGroup.m_linearVelocity.SelfMulAdd(1 / mass_1, f);
                            }
                            if (inertia > 0) {
                                ///cGroup.m_angularVelocity += b2Cross(pc - cGroup.GetCenter(), f) / inertia;
                                cGroup.m_angularVelocity += b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.SubVV(pc, cGroup.GetCenter(), b2Math_1.b2Vec2.s_t0), f) / inertia;
                            }
                        }
                        else {
                            ///m_velocityBuffer.data[c] += dv;
                            vel_data[c].SelfAdd(dv);
                        }
                        // Apply a reversed force to particle c after particle
                        // movement so that momentum will be preserved.
                        ///ParticleApplyForce(c, -step.inv_dt * f);
                        this.ParticleApplyForce(c, f.SelfMul(-step.inv_dt));
                    }
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveStaticPressure = function (step) {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        this.m_staticPressureBuffer = this.RequestBuffer(this.m_staticPressureBuffer);
        var criticalPressure = this.GetCriticalPressure(step);
        var pressurePerWeight = this.m_def.staticPressureStrength * criticalPressure;
        var maxPressure = b2Settings_2.b2_maxParticlePressure * criticalPressure;
        var relaxation = this.m_def.staticPressureRelaxation;
        /// Compute pressure satisfying the modified Poisson equation:
        ///   Sum_for_j((p_i - p_j) * w_ij) + relaxation * p_i =
        ///   pressurePerWeight * (w_i - b2_minParticleWeight)
        /// by iterating the calculation:
        ///   p_i = (Sum_for_j(p_j * w_ij) + pressurePerWeight *
        ///         (w_i - b2_minParticleWeight)) / (w_i + relaxation)
        /// where
        ///   p_i and p_j are static pressure of particle i and j
        ///   w_ij is contact weight between particle i and j
        ///   w_i is sum of contact weight of particle i
        for (var t = 0; t < this.m_def.staticPressureIterations; t++) {
            ///memset(m_accumulationBuffer, 0, sizeof(*m_accumulationBuffer) * m_count);
            for (var i = 0; i < this.m_count; i++) {
                this.m_accumulationBuffer[i] = 0;
            }
            for (var k = 0; k < this.m_contactBuffer.count; k++) {
                var contact = this.m_contactBuffer.data[k];
                if (contact.flags & b2Particle_1.b2ParticleFlag.b2_staticPressureParticle) {
                    var a = contact.indexA;
                    var b = contact.indexB;
                    var w = contact.weight;
                    this.m_accumulationBuffer[a] += w * this.m_staticPressureBuffer[b]; // a <- b
                    this.m_accumulationBuffer[b] += w * this.m_staticPressureBuffer[a]; // b <- a
                }
            }
            for (var i = 0; i < this.m_count; i++) {
                var w = this.m_weightBuffer[i];
                if (this.m_flagsBuffer.data[i] & b2Particle_1.b2ParticleFlag.b2_staticPressureParticle) {
                    var wh = this.m_accumulationBuffer[i];
                    var h = (wh + pressurePerWeight * (w - b2Settings_2.b2_minParticleWeight)) /
                        (w + relaxation);
                    this.m_staticPressureBuffer[i] = b2Math_1.b2Clamp(h, 0.0, maxPressure);
                }
                else {
                    this.m_staticPressureBuffer[i] = 0;
                }
            }
        }
    };
    b2ParticleSystem.prototype.ComputeWeight = function () {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        ///memset(m_weightBuffer, 0, sizeof(*m_weightBuffer) * m_count);
        for (var k = 0; k < this.m_count; k++) {
            this.m_weightBuffer[k] = 0;
        }
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            var w = contact.weight;
            this.m_weightBuffer[a] += w;
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var w = contact.weight;
            this.m_weightBuffer[a] += w;
            this.m_weightBuffer[b] += w;
        }
    };
    b2ParticleSystem.prototype.SolvePressure = function (step) {
        var s_f = b2ParticleSystem.SolvePressure_s_f;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        // calculates pressure as a linear function of density
        var criticalPressure = this.GetCriticalPressure(step);
        var pressurePerWeight = this.m_def.pressureStrength * criticalPressure;
        var maxPressure = b2Settings_2.b2_maxParticlePressure * criticalPressure;
        for (var i = 0; i < this.m_count; i++) {
            var w = this.m_weightBuffer[i];
            var h = pressurePerWeight * b2Math_1.b2Max(0.0, w - b2Settings_2.b2_minParticleWeight);
            this.m_accumulationBuffer[i] = b2Math_1.b2Min(h, maxPressure);
        }
        // ignores particles which have their own repulsive force
        if (this.m_allParticleFlags & b2ParticleSystem.k_noPressureFlags) {
            for (var i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & b2ParticleSystem.k_noPressureFlags) {
                    this.m_accumulationBuffer[i] = 0;
                }
            }
        }
        // static pressure
        if (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_staticPressureParticle) {
            // DEBUG: b2Assert(this.m_staticPressureBuffer !== null);
            for (var i = 0; i < this.m_count; i++) {
                if (this.m_flagsBuffer.data[i] & b2Particle_1.b2ParticleFlag.b2_staticPressureParticle) {
                    this.m_accumulationBuffer[i] += this.m_staticPressureBuffer[i];
                }
            }
        }
        // applies pressure between each particles in contact
        var velocityPerPressure = step.dt / (this.m_def.density * this.m_particleDiameter);
        var inv_mass = this.GetParticleInvMass();
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            var b = contact.body;
            var w = contact.weight;
            var m = contact.mass;
            var n = contact.normal;
            var p = pos_data[a];
            var h = this.m_accumulationBuffer[a] + pressurePerWeight * w;
            ///b2Vec2 f = velocityPerPressure * w * m * h * n;
            var f = b2Math_1.b2Vec2.MulSV(velocityPerPressure * w * m * h, n, s_f);
            ///m_velocityBuffer.data[a] -= GetParticleInvMass() * f;
            vel_data[a].SelfMulSub(inv_mass, f);
            b.ApplyLinearImpulse(f, p, true);
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var w = contact.weight;
            var n = contact.normal;
            var h = this.m_accumulationBuffer[a] + this.m_accumulationBuffer[b];
            ///b2Vec2 f = velocityPerPressure * w * h * n;
            var f = b2Math_1.b2Vec2.MulSV(velocityPerPressure * w * h, n, s_f);
            ///m_velocityBuffer.data[a] -= f;
            vel_data[a].SelfSub(f);
            ///m_velocityBuffer.data[b] += f;
            vel_data[b].SelfAdd(f);
        }
    };
    b2ParticleSystem.prototype.SolveDamping = function (step) {
        var s_v = b2ParticleSystem.SolveDamping_s_v;
        var s_f = b2ParticleSystem.SolveDamping_s_f;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        // reduces normal velocity of each contact
        var linearDamping = this.m_def.dampingStrength;
        var quadraticDamping = 1 / this.GetCriticalVelocity(step);
        var inv_mass = this.GetParticleInvMass();
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            var b = contact.body;
            var w = contact.weight;
            var m = contact.mass;
            var n = contact.normal;
            var p = pos_data[a];
            ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
            var v = b2Math_1.b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Math_1.b2Vec2.s_t0), vel_data[a], s_v);
            var vn = b2Math_1.b2Vec2.DotVV(v, n);
            if (vn < 0) {
                var damping = b2Math_1.b2Max(linearDamping * w, b2Math_1.b2Min(-quadraticDamping * vn, 0.5));
                ///b2Vec2 f = damping * m * vn * n;
                var f = b2Math_1.b2Vec2.MulSV(damping * m * vn, n, s_f);
                ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                vel_data[a].SelfMulAdd(inv_mass, f);
                ///b.ApplyLinearImpulse(-f, p, true);
                b.ApplyLinearImpulse(f.SelfNeg(), p, true);
            }
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var w = contact.weight;
            var n = contact.normal;
            ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
            var v = b2Math_1.b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
            var vn = b2Math_1.b2Vec2.DotVV(v, n);
            if (vn < 0) {
                ///float32 damping = b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
                var damping = b2Math_1.b2Max(linearDamping * w, b2Math_1.b2Min(-quadraticDamping * vn, 0.5));
                ///b2Vec2 f = damping * vn * n;
                var f = b2Math_1.b2Vec2.MulSV(damping * vn, n, s_f);
                ///this.m_velocityBuffer.data[a] += f;
                vel_data[a].SelfAdd(f);
                ///this.m_velocityBuffer.data[b] -= f;
                vel_data[b].SelfSub(f);
            }
        }
    };
    b2ParticleSystem.prototype.SolveRigidDamping = function () {
        var s_t0 = b2ParticleSystem.SolveRigidDamping_s_t0;
        var s_t1 = b2ParticleSystem.SolveRigidDamping_s_t1;
        var s_p = b2ParticleSystem.SolveRigidDamping_s_p;
        var s_v = b2ParticleSystem.SolveRigidDamping_s_v;
        var invMassA = [0.0], invInertiaA = [0.0], tangentDistanceA = [0.0]; // TODO: static
        var invMassB = [0.0], invInertiaB = [0.0], tangentDistanceB = [0.0]; // TODO: static
        // Apply impulse to rigid particle groups colliding with other objects
        // to reduce relative velocity at the colliding point.
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var damping = this.m_def.dampingStrength;
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            var aGroup = this.m_groupBuffer[a];
            if (aGroup && this.IsRigidGroup(aGroup)) {
                var b = contact.body;
                var n = contact.normal;
                var w = contact.weight;
                var p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - aGroup.GetLinearVelocityFromWorldPoint(p);
                var v = b2Math_1.b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, s_t0), aGroup.GetLinearVelocityFromWorldPoint(p, s_t1), s_v);
                var vn = b2Math_1.b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    // The group's average velocity at particle position 'p' is pushing
                    // the particle into the body.
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassA, &invInertiaA, &tangentDistanceA, true, aGroup, a, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassA, invInertiaA, tangentDistanceA, true, aGroup, a, p, n);
                    // Calculate b.m_I from public functions of b2Body.
                    ///this.InitDampingParameter(&invMassB, &invInertiaB, &tangentDistanceB, b.GetMass(), b.GetInertia() - b.GetMass() * b.GetLocalCenter().LengthSquared(), b.GetWorldCenter(), p, n);
                    this.InitDampingParameter(invMassB, invInertiaB, tangentDistanceB, b.GetMass(), b.GetInertia() - b.GetMass() * b.GetLocalCenter().LengthSquared(), b.GetWorldCenter(), p, n);
                    ///float32 f = damping * b2Min(w, 1.0) * this.ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                    var f = damping * b2Math_1.b2Min(w, 1.0) * this.ComputeDampingImpulse(invMassA[0], invInertiaA[0], tangentDistanceA[0], invMassB[0], invInertiaB[0], tangentDistanceB[0], vn);
                    ///this.ApplyDamping(invMassA, invInertiaA, tangentDistanceA, true, aGroup, a, f, n);
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], true, aGroup, a, f, n);
                    ///b.ApplyLinearImpulse(-f * n, p, true);
                    b.ApplyLinearImpulse(b2Math_1.b2Vec2.MulSV(-f, n, b2Math_1.b2Vec2.s_t0), p, true);
                }
            }
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            var n = contact.normal;
            var w = contact.weight;
            var aGroup = this.m_groupBuffer[a];
            var bGroup = this.m_groupBuffer[b];
            var aRigid = this.IsRigidGroup(aGroup);
            var bRigid = this.IsRigidGroup(bGroup);
            if (aGroup !== bGroup && (aRigid || bRigid)) {
                ///b2Vec2 p = 0.5f * (this.m_positionBuffer.data[a] + this.m_positionBuffer.data[b]);
                var p = b2Math_1.b2Vec2.MidVV(pos_data[a], pos_data[b], s_p);
                ///b2Vec2 v = GetLinearVelocity(bGroup, b, p) - GetLinearVelocity(aGroup, a, p);
                var v = b2Math_1.b2Vec2.SubVV(this.GetLinearVelocity(bGroup, b, p, s_t0), this.GetLinearVelocity(aGroup, a, p, s_t1), s_v);
                var vn = b2Math_1.b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassA, &invInertiaA, &tangentDistanceA, aRigid, aGroup, a, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassA, invInertiaA, tangentDistanceA, aRigid, aGroup, a, p, n);
                    ///this.InitDampingParameterWithRigidGroupOrParticle(&invMassB, &invInertiaB, &tangentDistanceB, bRigid, bGroup, b, p, n);
                    this.InitDampingParameterWithRigidGroupOrParticle(invMassB, invInertiaB, tangentDistanceB, bRigid, bGroup, b, p, n);
                    ///float32 f = damping * w * this.ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                    var f = damping * w * this.ComputeDampingImpulse(invMassA[0], invInertiaA[0], tangentDistanceA[0], invMassB[0], invInertiaB[0], tangentDistanceB[0], vn);
                    ///this.ApplyDamping(invMassA, invInertiaA, tangentDistanceA, aRigid, aGroup, a, f, n);
                    this.ApplyDamping(invMassA[0], invInertiaA[0], tangentDistanceA[0], aRigid, aGroup, a, f, n);
                    ///this.ApplyDamping(invMassB, invInertiaB, tangentDistanceB, bRigid, bGroup, b, -f, n);
                    this.ApplyDamping(invMassB[0], invInertiaB[0], tangentDistanceB[0], bRigid, bGroup, b, -f, n);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveExtraDamping = function () {
        var s_v = b2ParticleSystem.SolveExtraDamping_s_v;
        var s_f = b2ParticleSystem.SolveExtraDamping_s_f;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        // Applies additional damping force between bodies and particles which can
        // produce strong repulsive force. Applying damping force multiple times
        // is effective in suppressing vibration.
        var pos_data = this.m_positionBuffer.data;
        var inv_mass = this.GetParticleInvMass();
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            if (this.m_flagsBuffer.data[a] & b2ParticleSystem.k_extraDampingFlags) {
                var b = contact.body;
                var m = contact.mass;
                var n = contact.normal;
                var p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
                var v = b2Math_1.b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Math_1.b2Vec2.s_t0), vel_data[a], s_v);
                ///float32 vn = b2Dot(v, n);
                var vn = b2Math_1.b2Vec2.DotVV(v, n);
                if (vn < 0) {
                    ///b2Vec2 f = 0.5f * m * vn * n;
                    var f = b2Math_1.b2Vec2.MulSV(0.5 * m * vn, n, s_f);
                    ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                    vel_data[a].SelfMulAdd(inv_mass, f);
                    ///b.ApplyLinearImpulse(-f, p, true);
                    b.ApplyLinearImpulse(f.SelfNeg(), p, true);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveWall = function () {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        for (var i = 0; i < this.m_count; i++) {
            if (this.m_flagsBuffer.data[i] & b2Particle_1.b2ParticleFlag.b2_wallParticle) {
                vel_data[i].SetZero();
            }
        }
    };
    b2ParticleSystem.prototype.SolveRigid = function (step) {
        var s_position = b2ParticleSystem.SolveRigid_s_position;
        var s_rotation = b2ParticleSystem.SolveRigid_s_rotation;
        var s_transform = b2ParticleSystem.SolveRigid_s_transform;
        var s_velocityTransform = b2ParticleSystem.SolveRigid_s_velocityTransform;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        for (var group = this.m_groupList; group; group = group.GetNext()) {
            if (group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_rigidParticleGroup) {
                group.UpdateStatistics();
                ///b2Rot rotation(step.dt * group.m_angularVelocity);
                var rotation = s_rotation;
                rotation.SetAngle(step.dt * group.m_angularVelocity);
                ///b2Transform transform(group.m_center + step.dt * group.m_linearVelocity - b2Mul(rotation, group.m_center), rotation);
                var position = b2Math_1.b2Vec2.AddVV(group.m_center, b2Math_1.b2Vec2.SubVV(b2Math_1.b2Vec2.MulSV(step.dt, group.m_linearVelocity, b2Math_1.b2Vec2.s_t0), b2Math_1.b2Rot.MulRV(rotation, group.m_center, b2Math_1.b2Vec2.s_t1), b2Math_1.b2Vec2.s_t0), s_position);
                var transform = s_transform;
                transform.SetPositionRotation(position, rotation);
                ///group.m_transform = b2Mul(transform, group.m_transform);
                b2Math_1.b2Transform.MulXX(transform, group.m_transform, group.m_transform);
                var velocityTransform = s_velocityTransform;
                velocityTransform.p.x = step.inv_dt * transform.p.x;
                velocityTransform.p.y = step.inv_dt * transform.p.y;
                velocityTransform.q.s = step.inv_dt * transform.q.s;
                velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
                for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                    ///m_velocityBuffer.data[i] = b2Mul(velocityTransform, m_positionBuffer.data[i]);
                    b2Math_1.b2Transform.MulXV(velocityTransform, pos_data[i], vel_data[i]);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveElastic = function (step) {
        var s_pa = b2ParticleSystem.SolveElastic_s_pa;
        var s_pb = b2ParticleSystem.SolveElastic_s_pb;
        var s_pc = b2ParticleSystem.SolveElastic_s_pc;
        var s_r = b2ParticleSystem.SolveElastic_s_r;
        var s_t0 = b2ParticleSystem.SolveElastic_s_t0;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        var elasticStrength = step.inv_dt * this.m_def.elasticStrength;
        for (var k = 0; k < this.m_triadBuffer.count; k++) {
            var triad = this.m_triadBuffer.data[k];
            if (triad.flags & b2Particle_1.b2ParticleFlag.b2_elasticParticle) {
                var a = triad.indexA;
                var b = triad.indexB;
                var c = triad.indexC;
                var oa = triad.pa;
                var ob = triad.pb;
                var oc = triad.pc;
                ///b2Vec2 pa = m_positionBuffer.data[a];
                var pa = s_pa.Copy(pos_data[a]);
                ///b2Vec2 pb = m_positionBuffer.data[b];
                var pb = s_pb.Copy(pos_data[b]);
                ///b2Vec2 pc = m_positionBuffer.data[c];
                var pc = s_pc.Copy(pos_data[c]);
                var va = vel_data[a];
                var vb = vel_data[b];
                var vc = vel_data[c];
                ///pa += step.dt * va;
                pa.SelfMulAdd(step.dt, va);
                ///pb += step.dt * vb;
                pb.SelfMulAdd(step.dt, vb);
                ///pc += step.dt * vc;
                pc.SelfMulAdd(step.dt, vc);
                ///b2Vec2 midPoint = (float32) 1 / 3 * (pa + pb + pc);
                var midPoint_x = (pa.x + pb.x + pc.x) / 3.0;
                var midPoint_y = (pa.y + pb.y + pc.y) / 3.0;
                ///pa -= midPoint;
                pa.x -= midPoint_x;
                pa.y -= midPoint_y;
                ///pb -= midPoint;
                pb.x -= midPoint_x;
                pb.y -= midPoint_y;
                ///pc -= midPoint;
                pc.x -= midPoint_x;
                pc.y -= midPoint_y;
                ///b2Rot r;
                var r = s_r;
                r.s = b2Math_1.b2Vec2.CrossVV(oa, pa) + b2Math_1.b2Vec2.CrossVV(ob, pb) + b2Math_1.b2Vec2.CrossVV(oc, pc);
                r.c = b2Math_1.b2Vec2.DotVV(oa, pa) + b2Math_1.b2Vec2.DotVV(ob, pb) + b2Math_1.b2Vec2.DotVV(oc, pc);
                var r2 = r.s * r.s + r.c * r.c;
                var invR = b2Math_1.b2InvSqrt(r2);
                if (!isFinite(invR)) {
                    invR = 1.98177537e+019;
                }
                r.s *= invR;
                r.c *= invR;
                ///r.angle = Math.atan2(r.s, r.c); // TODO: optimize
                var strength = elasticStrength * triad.strength;
                ///va += strength * (b2Mul(r, oa) - pa);
                b2Math_1.b2Rot.MulRV(r, oa, s_t0);
                b2Math_1.b2Vec2.SubVV(s_t0, pa, s_t0);
                b2Math_1.b2Vec2.MulSV(strength, s_t0, s_t0);
                va.SelfAdd(s_t0);
                ///vb += strength * (b2Mul(r, ob) - pb);
                b2Math_1.b2Rot.MulRV(r, ob, s_t0);
                b2Math_1.b2Vec2.SubVV(s_t0, pb, s_t0);
                b2Math_1.b2Vec2.MulSV(strength, s_t0, s_t0);
                vb.SelfAdd(s_t0);
                ///vc += strength * (b2Mul(r, oc) - pc);
                b2Math_1.b2Rot.MulRV(r, oc, s_t0);
                b2Math_1.b2Vec2.SubVV(s_t0, pc, s_t0);
                b2Math_1.b2Vec2.MulSV(strength, s_t0, s_t0);
                vc.SelfAdd(s_t0);
            }
        }
    };
    b2ParticleSystem.prototype.SolveSpring = function (step) {
        var s_pa = b2ParticleSystem.SolveSpring_s_pa;
        var s_pb = b2ParticleSystem.SolveSpring_s_pb;
        var s_d = b2ParticleSystem.SolveSpring_s_d;
        var s_f = b2ParticleSystem.SolveSpring_s_f;
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        var springStrength = step.inv_dt * this.m_def.springStrength;
        for (var k = 0; k < this.m_pairBuffer.count; k++) {
            var pair = this.m_pairBuffer.data[k];
            if (pair.flags & b2Particle_1.b2ParticleFlag.b2_springParticle) {
                ///int32 a = pair.indexA;
                var a = pair.indexA;
                ///int32 b = pair.indexB;
                var b = pair.indexB;
                ///b2Vec2 pa = m_positionBuffer.data[a];
                var pa = s_pa.Copy(pos_data[a]);
                ///b2Vec2 pb = m_positionBuffer.data[b];
                var pb = s_pb.Copy(pos_data[b]);
                ///b2Vec2& va = m_velocityBuffer.data[a];
                var va = vel_data[a];
                ///b2Vec2& vb = m_velocityBuffer.data[b];
                var vb = vel_data[b];
                ///pa += step.dt * va;
                pa.SelfMulAdd(step.dt, va);
                ///pb += step.dt * vb;
                pb.SelfMulAdd(step.dt, vb);
                ///b2Vec2 d = pb - pa;
                var d = b2Math_1.b2Vec2.SubVV(pb, pa, s_d);
                ///float32 r0 = pair.distance;
                var r0 = pair.distance;
                ///float32 r1 = d.Length();
                var r1 = d.Length();
                ///float32 strength = springStrength * pair.strength;
                var strength = springStrength * pair.strength;
                ///b2Vec2 f = strength * (r0 - r1) / r1 * d;
                var f = b2Math_1.b2Vec2.MulSV(strength * (r0 - r1) / r1, d, s_f);
                ///va -= f;
                va.SelfSub(f);
                ///vb += f;
                vb.SelfAdd(f);
            }
        }
    };
    b2ParticleSystem.prototype.SolveTensile = function (step) {
        var s_weightedNormal = b2ParticleSystem.SolveTensile_s_weightedNormal;
        var s_s = b2ParticleSystem.SolveTensile_s_s;
        var s_f = b2ParticleSystem.SolveTensile_s_f;
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        // DEBUG: b2Assert(this.m_accumulation2Buffer !== null);
        for (var i = 0; i < this.m_count; i++) {
            this.m_accumulation2Buffer[i] = new b2Math_1.b2Vec2();
            this.m_accumulation2Buffer[i].SetZero();
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            if (contact.flags & b2Particle_1.b2ParticleFlag.b2_tensileParticle) {
                var a = contact.indexA;
                var b = contact.indexB;
                var w = contact.weight;
                var n = contact.normal;
                ///b2Vec2 weightedNormal = (1 - w) * w * n;
                var weightedNormal = b2Math_1.b2Vec2.MulSV((1 - w) * w, n, s_weightedNormal);
                ///m_accumulation2Buffer[a] -= weightedNormal;
                this.m_accumulation2Buffer[a].SelfSub(weightedNormal);
                ///m_accumulation2Buffer[b] += weightedNormal;
                this.m_accumulation2Buffer[b].SelfAdd(weightedNormal);
            }
        }
        var criticalVelocity = this.GetCriticalVelocity(step);
        var pressureStrength = this.m_def.surfaceTensionPressureStrength * criticalVelocity;
        var normalStrength = this.m_def.surfaceTensionNormalStrength * criticalVelocity;
        var maxVelocityVariation = b2Settings_2.b2_maxParticleForce * criticalVelocity;
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            if (contact.flags & b2Particle_1.b2ParticleFlag.b2_tensileParticle) {
                var a = contact.indexA;
                var b = contact.indexB;
                var w = contact.weight;
                var n = contact.normal;
                var h = this.m_weightBuffer[a] + this.m_weightBuffer[b];
                ///b2Vec2 s = m_accumulation2Buffer[b] - m_accumulation2Buffer[a];
                var s = b2Math_1.b2Vec2.SubVV(this.m_accumulation2Buffer[b], this.m_accumulation2Buffer[a], s_s);
                var fn = b2Math_1.b2Min(pressureStrength * (h - 2) + normalStrength * b2Math_1.b2Vec2.DotVV(s, n), maxVelocityVariation) * w;
                ///b2Vec2 f = fn * n;
                var f = b2Math_1.b2Vec2.MulSV(fn, n, s_f);
                ///m_velocityBuffer.data[a] -= f;
                vel_data[a].SelfSub(f);
                ///m_velocityBuffer.data[b] += f;
                vel_data[b].SelfAdd(f);
            }
        }
    };
    b2ParticleSystem.prototype.SolveViscous = function () {
        var s_v = b2ParticleSystem.SolveViscous_s_v;
        var s_f = b2ParticleSystem.SolveViscous_s_f;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        var viscousStrength = this.m_def.viscousStrength;
        var inv_mass = this.GetParticleInvMass();
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            if (this.m_flagsBuffer.data[a] & b2Particle_1.b2ParticleFlag.b2_viscousParticle) {
                var b = contact.body;
                var w = contact.weight;
                var m = contact.mass;
                var p = pos_data[a];
                ///b2Vec2 v = b.GetLinearVelocityFromWorldPoint(p) - m_velocityBuffer.data[a];
                var v = b2Math_1.b2Vec2.SubVV(b.GetLinearVelocityFromWorldPoint(p, b2Math_1.b2Vec2.s_t0), vel_data[a], s_v);
                ///b2Vec2 f = viscousStrength * m * w * v;
                var f = b2Math_1.b2Vec2.MulSV(viscousStrength * m * w, v, s_f);
                ///m_velocityBuffer.data[a] += GetParticleInvMass() * f;
                vel_data[a].SelfMulAdd(inv_mass, f);
                ///b.ApplyLinearImpulse(-f, p, true);
                b.ApplyLinearImpulse(f.SelfNeg(), p, true);
            }
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            if (contact.flags & b2Particle_1.b2ParticleFlag.b2_viscousParticle) {
                var a = contact.indexA;
                var b = contact.indexB;
                var w = contact.weight;
                ///b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
                var v = b2Math_1.b2Vec2.SubVV(vel_data[b], vel_data[a], s_v);
                ///b2Vec2 f = viscousStrength * w * v;
                var f = b2Math_1.b2Vec2.MulSV(viscousStrength * w, v, s_f);
                ///m_velocityBuffer.data[a] += f;
                vel_data[a].SelfAdd(f);
                ///m_velocityBuffer.data[b] -= f;
                vel_data[b].SelfSub(f);
            }
        }
    };
    b2ParticleSystem.prototype.SolveRepulsive = function (step) {
        var s_f = b2ParticleSystem.SolveRepulsive_s_f;
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        var repulsiveStrength = this.m_def.repulsiveStrength * this.GetCriticalVelocity(step);
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            if (contact.flags & b2Particle_1.b2ParticleFlag.b2_repulsiveParticle) {
                var a = contact.indexA;
                var b = contact.indexB;
                if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                    var w = contact.weight;
                    var n = contact.normal;
                    ///b2Vec2 f = repulsiveStrength * w * n;
                    var f = b2Math_1.b2Vec2.MulSV(repulsiveStrength * w, n, s_f);
                    ///m_velocityBuffer.data[a] -= f;
                    vel_data[a].SelfSub(f);
                    ///m_velocityBuffer.data[b] += f;
                    vel_data[b].SelfAdd(f);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolvePowder = function (step) {
        var s_f = b2ParticleSystem.SolvePowder_s_f;
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var pos_data = this.m_positionBuffer.data;
        var vel_data = this.m_velocityBuffer.data;
        var powderStrength = this.m_def.powderStrength * this.GetCriticalVelocity(step);
        var minWeight = 1.0 - b2Settings_2.b2_particleStride;
        var inv_mass = this.GetParticleInvMass();
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            var a = contact.index;
            if (this.m_flagsBuffer.data[a] & b2Particle_1.b2ParticleFlag.b2_powderParticle) {
                var w = contact.weight;
                if (w > minWeight) {
                    var b = contact.body;
                    var m = contact.mass;
                    var p = pos_data[a];
                    var n = contact.normal;
                    var f = b2Math_1.b2Vec2.MulSV(powderStrength * m * (w - minWeight), n, s_f);
                    vel_data[a].SelfMulSub(inv_mass, f);
                    b.ApplyLinearImpulse(f, p, true);
                }
            }
        }
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            if (contact.flags & b2Particle_1.b2ParticleFlag.b2_powderParticle) {
                var w = contact.weight;
                if (w > minWeight) {
                    var a = contact.indexA;
                    var b = contact.indexB;
                    var n = contact.normal;
                    var f = b2Math_1.b2Vec2.MulSV(powderStrength * (w - minWeight), n, s_f);
                    vel_data[a].SelfSub(f);
                    vel_data[b].SelfAdd(f);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveSolid = function (step) {
        var s_f = b2ParticleSystem.SolveSolid_s_f;
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        // applies extra repulsive force from solid particle groups
        this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
        var ejectionStrength = step.inv_dt * this.m_def.ejectionStrength;
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            var a = contact.indexA;
            var b = contact.indexB;
            if (this.m_groupBuffer[a] !== this.m_groupBuffer[b]) {
                var w = contact.weight;
                var n = contact.normal;
                var h = this.m_depthBuffer[a] + this.m_depthBuffer[b];
                var f = b2Math_1.b2Vec2.MulSV(ejectionStrength * h * w, n, s_f);
                vel_data[a].SelfSub(f);
                vel_data[b].SelfAdd(f);
            }
        }
    };
    b2ParticleSystem.prototype.SolveForce = function (step) {
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        var vel_data = this.m_velocityBuffer.data;
        var velocityPerForce = step.dt * this.GetParticleInvMass();
        for (var i = 0; i < this.m_count; i++) {
            ///m_velocityBuffer.data[i] += velocityPerForce * m_forceBuffer[i];
            vel_data[i].SelfMulAdd(velocityPerForce, this.m_forceBuffer[i]);
        }
        this.m_hasForce = false;
    };
    b2ParticleSystem.prototype.SolveColorMixing = function () {
        // mixes color between contacting particles
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_colorBuffer.data) {
            throw new Error();
        }
        var colorMixing = 0.5 * this.m_def.colorMixingStrength;
        if (colorMixing) {
            for (var k = 0; k < this.m_contactBuffer.count; k++) {
                var contact = this.m_contactBuffer.data[k];
                var a = contact.indexA;
                var b = contact.indexB;
                if (this.m_flagsBuffer.data[a] & this.m_flagsBuffer.data[b] &
                    b2Particle_1.b2ParticleFlag.b2_colorMixingParticle) {
                    var colorA = this.m_colorBuffer.data[a];
                    var colorB = this.m_colorBuffer.data[b];
                    // Use the static method to ensure certain compilers inline
                    // this correctly.
                    b2Draw_1.b2Color.MixColors(colorA, colorB, colorMixing);
                }
            }
        }
    };
    b2ParticleSystem.prototype.SolveZombie = function () {
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        // removes particles with zombie flag
        var newCount = 0;
        ///int32* newIndices = (int32*) this.m_world.m_stackAllocator.Allocate(sizeof(int32) * this.m_count);
        var newIndices = []; // TODO: static
        for (var i = 0; i < this.m_count; i++) {
            newIndices[i] = b2Settings_1.b2_invalidParticleIndex;
        }
        // DEBUG: b2Assert(newIndices.length === this.m_count);
        var allParticleFlags = 0;
        for (var i = 0; i < this.m_count; i++) {
            var flags = this.m_flagsBuffer.data[i];
            if (flags & b2Particle_1.b2ParticleFlag.b2_zombieParticle) {
                var destructionListener = this.m_world.m_destructionListener;
                if ((flags & b2Particle_1.b2ParticleFlag.b2_destructionListenerParticle) && destructionListener) {
                    destructionListener.SayGoodbyeParticle(this, i);
                }
                // Destroy particle handle.
                if (this.m_handleIndexBuffer.data) {
                    var handle = this.m_handleIndexBuffer.data[i];
                    if (handle) {
                        handle.SetIndex(b2Settings_1.b2_invalidParticleIndex);
                        this.m_handleIndexBuffer.data[i] = null;
                        ///m_handleAllocator.Free(handle);
                    }
                }
                newIndices[i] = b2Settings_1.b2_invalidParticleIndex;
            }
            else {
                newIndices[i] = newCount;
                if (i !== newCount) {
                    // Update handle to reference new particle index.
                    if (this.m_handleIndexBuffer.data) {
                        var handle = this.m_handleIndexBuffer.data[i];
                        if (handle) {
                            handle.SetIndex(newCount);
                        }
                        this.m_handleIndexBuffer.data[newCount] = handle;
                    }
                    this.m_flagsBuffer.data[newCount] = this.m_flagsBuffer.data[i];
                    if (this.m_lastBodyContactStepBuffer.data) {
                        this.m_lastBodyContactStepBuffer.data[newCount] = this.m_lastBodyContactStepBuffer.data[i];
                    }
                    if (this.m_bodyContactCountBuffer.data) {
                        this.m_bodyContactCountBuffer.data[newCount] = this.m_bodyContactCountBuffer.data[i];
                    }
                    if (this.m_consecutiveContactStepsBuffer.data) {
                        this.m_consecutiveContactStepsBuffer.data[newCount] = this.m_consecutiveContactStepsBuffer.data[i];
                    }
                    this.m_positionBuffer.data[newCount].Copy(this.m_positionBuffer.data[i]);
                    this.m_velocityBuffer.data[newCount].Copy(this.m_velocityBuffer.data[i]);
                    this.m_groupBuffer[newCount] = this.m_groupBuffer[i];
                    if (this.m_hasForce) {
                        this.m_forceBuffer[newCount].Copy(this.m_forceBuffer[i]);
                    }
                    if (this.m_staticPressureBuffer) {
                        this.m_staticPressureBuffer[newCount] = this.m_staticPressureBuffer[i];
                    }
                    if (this.m_depthBuffer) {
                        this.m_depthBuffer[newCount] = this.m_depthBuffer[i];
                    }
                    if (this.m_colorBuffer.data) {
                        this.m_colorBuffer.data[newCount].Copy(this.m_colorBuffer.data[i]);
                    }
                    if (this.m_userDataBuffer.data) {
                        this.m_userDataBuffer.data[newCount] = this.m_userDataBuffer.data[i];
                    }
                    if (this.m_expirationTimeBuffer.data) {
                        this.m_expirationTimeBuffer.data[newCount] = this.m_expirationTimeBuffer.data[i];
                    }
                }
                newCount++;
                allParticleFlags |= flags;
            }
        }
        // predicate functions
        var Test = {
            ///static bool IsProxyInvalid(const Proxy& proxy)
            IsProxyInvalid: function (proxy) {
                return proxy.index < 0;
            },
            ///static bool IsContactInvalid(const b2ParticleContact& contact)
            IsContactInvalid: function (contact) {
                return contact.indexA < 0 || contact.indexB < 0;
            },
            ///static bool IsBodyContactInvalid(const b2ParticleBodyContact& contact)
            IsBodyContactInvalid: function (contact) {
                return contact.index < 0;
            },
            ///static bool IsPairInvalid(const b2ParticlePair& pair)
            IsPairInvalid: function (pair) {
                return pair.indexA < 0 || pair.indexB < 0;
            },
            ///static bool IsTriadInvalid(const b2ParticleTriad& triad)
            IsTriadInvalid: function (triad) {
                return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
            },
        };
        // update proxies
        for (var k = 0; k < this.m_proxyBuffer.count; k++) {
            var proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndices[proxy.index];
        }
        this.m_proxyBuffer.RemoveIf(Test.IsProxyInvalid);
        // update contacts
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndices[contact.indexA];
            contact.indexB = newIndices[contact.indexB];
        }
        this.m_contactBuffer.RemoveIf(Test.IsContactInvalid);
        // update particle-body contacts
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndices[contact.index];
        }
        this.m_bodyContactBuffer.RemoveIf(Test.IsBodyContactInvalid);
        // update pairs
        for (var k = 0; k < this.m_pairBuffer.count; k++) {
            var pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndices[pair.indexA];
            pair.indexB = newIndices[pair.indexB];
        }
        this.m_pairBuffer.RemoveIf(Test.IsPairInvalid);
        // update triads
        for (var k = 0; k < this.m_triadBuffer.count; k++) {
            var triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndices[triad.indexA];
            triad.indexB = newIndices[triad.indexB];
            triad.indexC = newIndices[triad.indexC];
        }
        this.m_triadBuffer.RemoveIf(Test.IsTriadInvalid);
        // Update lifetime indices.
        if (this.m_indexByExpirationTimeBuffer.data) {
            var writeOffset = 0;
            for (var readOffset = 0; readOffset < this.m_count; readOffset++) {
                var newIndex = newIndices[this.m_indexByExpirationTimeBuffer.data[readOffset]];
                if (newIndex !== b2Settings_1.b2_invalidParticleIndex) {
                    this.m_indexByExpirationTimeBuffer.data[writeOffset++] = newIndex;
                }
            }
        }
        // update groups
        for (var group = this.m_groupList; group; group = group.GetNext()) {
            var firstIndex = newCount;
            var lastIndex = 0;
            var modified = false;
            for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
                var j = newIndices[i];
                if (j >= 0) {
                    firstIndex = b2Math_1.b2Min(firstIndex, j);
                    lastIndex = b2Math_1.b2Max(lastIndex, j + 1);
                }
                else {
                    modified = true;
                }
            }
            if (firstIndex < lastIndex) {
                group.m_firstIndex = firstIndex;
                group.m_lastIndex = lastIndex;
                if (modified) {
                    if (group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_solidParticleGroup) {
                        this.SetGroupFlags(group, group.m_groupFlags | b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth);
                    }
                }
            }
            else {
                group.m_firstIndex = 0;
                group.m_lastIndex = 0;
                if (!(group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupCanBeEmpty)) {
                    this.SetGroupFlags(group, group.m_groupFlags | b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupWillBeDestroyed);
                }
            }
        }
        // update particle count
        this.m_count = newCount;
        ///m_world.m_stackAllocator.Free(newIndices);
        this.m_allParticleFlags = allParticleFlags;
        this.m_needsUpdateAllParticleFlags = false;
        // destroy bodies with no particles
        for (var group = this.m_groupList; group;) {
            var next = group.GetNext();
            if (group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupWillBeDestroyed) {
                this.DestroyParticleGroup(group);
            }
            group = next;
        }
    };
    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    b2ParticleSystem.prototype.SolveLifetimes = function (step) {
        if (!this.m_expirationTimeBuffer.data) {
            throw new Error();
        }
        if (!this.m_indexByExpirationTimeBuffer.data) {
            throw new Error();
        }
        // Update the time elapsed.
        this.m_timeElapsed = this.LifetimeToExpirationTime(step.dt);
        // Get the floor (non-fractional component) of the elapsed time.
        var quantizedTimeElapsed = this.GetQuantizedTimeElapsed();
        var expirationTimes = this.m_expirationTimeBuffer.data;
        var expirationTimeIndices = this.m_indexByExpirationTimeBuffer.data;
        var particleCount = this.GetParticleCount();
        // Sort the lifetime buffer if it's required.
        if (this.m_expirationTimeBufferRequiresSorting) {
            ///const ExpirationTimeComparator expirationTimeComparator(expirationTimes);
            ///std::sort(expirationTimeIndices, expirationTimeIndices + particleCount, expirationTimeComparator);
            /**
             * Compare the lifetime of particleIndexA and particleIndexB
             * returning true if the lifetime of A is greater than B for
             * particles that will expire.  If either particle's lifetime is
             * infinite (<= 0.0f) this function return true if the lifetime
             * of A is lesser than B. When used with std::sort() this
             * results in an array of particle indicies sorted in reverse
             * order by particle lifetime.
             *
             * For example, the set of lifetimes
             * (1.0, 0.7, 0.3, 0.0, -1.0, 2.0)
             * would be sorted as
             * (0.0, 1.0, -2.0, 1.0, 0.7, 0.3)
             */
            var ExpirationTimeComparator = function (particleIndexA, particleIndexB) {
                var expirationTimeA = expirationTimes[particleIndexA];
                var expirationTimeB = expirationTimes[particleIndexB];
                var infiniteExpirationTimeA = expirationTimeA <= 0.0;
                var infiniteExpirationTimeB = expirationTimeB <= 0.0;
                return infiniteExpirationTimeA === infiniteExpirationTimeB ?
                    expirationTimeA > expirationTimeB : infiniteExpirationTimeA;
            };
            std_sort(expirationTimeIndices, 0, particleCount, ExpirationTimeComparator);
            this.m_expirationTimeBufferRequiresSorting = false;
        }
        // Destroy particles which have expired.
        for (var i = particleCount - 1; i >= 0; --i) {
            var particleIndex = expirationTimeIndices[i];
            var expirationTime = expirationTimes[particleIndex];
            // If no particles need to be destroyed, skip this.
            if (quantizedTimeElapsed < expirationTime || expirationTime <= 0) {
                break;
            }
            // Destroy this particle.
            this.DestroyParticle(particleIndex);
        }
    };
    b2ParticleSystem.prototype.RotateBuffer = function (start, mid, end) {
        // move the particles assigned to the given group toward the end of array
        if (start === mid || mid === end) {
            return;
        }
        // DEBUG: b2Assert(mid >= start && mid <= end);
        function newIndices(i) {
            if (i < start) {
                return i;
            }
            else if (i < mid) {
                return i + end - mid;
            }
            else if (i < end) {
                return i + start - mid;
            }
            else {
                return i;
            }
        }
        if (!this.m_flagsBuffer.data) {
            throw new Error();
        }
        if (!this.m_positionBuffer.data) {
            throw new Error();
        }
        if (!this.m_velocityBuffer.data) {
            throw new Error();
        }
        ///std::rotate(m_flagsBuffer.data + start, m_flagsBuffer.data + mid, m_flagsBuffer.data + end);
        std_rotate(this.m_flagsBuffer.data, start, mid, end);
        if (this.m_lastBodyContactStepBuffer.data) {
            ///std::rotate(m_lastBodyContactStepBuffer.data + start, m_lastBodyContactStepBuffer.data + mid, m_lastBodyContactStepBuffer.data + end);
            std_rotate(this.m_lastBodyContactStepBuffer.data, start, mid, end);
        }
        if (this.m_bodyContactCountBuffer.data) {
            ///std::rotate(m_bodyContactCountBuffer.data + start, m_bodyContactCountBuffer.data + mid, m_bodyContactCountBuffer.data + end);
            std_rotate(this.m_bodyContactCountBuffer.data, start, mid, end);
        }
        if (this.m_consecutiveContactStepsBuffer.data) {
            ///std::rotate(m_consecutiveContactStepsBuffer.data + start, m_consecutiveContactStepsBuffer.data + mid, m_consecutiveContactStepsBuffer.data + end);
            std_rotate(this.m_consecutiveContactStepsBuffer.data, start, mid, end);
        }
        ///std::rotate(m_positionBuffer.data + start, m_positionBuffer.data + mid, m_positionBuffer.data + end);
        std_rotate(this.m_positionBuffer.data, start, mid, end);
        ///std::rotate(m_velocityBuffer.data + start, m_velocityBuffer.data + mid, m_velocityBuffer.data + end);
        std_rotate(this.m_velocityBuffer.data, start, mid, end);
        ///std::rotate(m_groupBuffer + start, m_groupBuffer + mid, m_groupBuffer + end);
        std_rotate(this.m_groupBuffer, start, mid, end);
        if (this.m_hasForce) {
            ///std::rotate(m_forceBuffer + start, m_forceBuffer + mid, m_forceBuffer + end);
            std_rotate(this.m_forceBuffer, start, mid, end);
        }
        if (this.m_staticPressureBuffer) {
            ///std::rotate(m_staticPressureBuffer + start, m_staticPressureBuffer + mid, m_staticPressureBuffer + end);
            std_rotate(this.m_staticPressureBuffer, start, mid, end);
        }
        if (this.m_depthBuffer) {
            ///std::rotate(m_depthBuffer + start, m_depthBuffer + mid, m_depthBuffer + end);
            std_rotate(this.m_depthBuffer, start, mid, end);
        }
        if (this.m_colorBuffer.data) {
            ///std::rotate(m_colorBuffer.data + start, m_colorBuffer.data + mid, m_colorBuffer.data + end);
            std_rotate(this.m_colorBuffer.data, start, mid, end);
        }
        if (this.m_userDataBuffer.data) {
            ///std::rotate(m_userDataBuffer.data + start, m_userDataBuffer.data + mid, m_userDataBuffer.data + end);
            std_rotate(this.m_userDataBuffer.data, start, mid, end);
        }
        // Update handle indices.
        if (this.m_handleIndexBuffer.data) {
            ///std::rotate(m_handleIndexBuffer.data + start, m_handleIndexBuffer.data + mid, m_handleIndexBuffer.data + end);
            std_rotate(this.m_handleIndexBuffer.data, start, mid, end);
            for (var i = start; i < end; ++i) {
                var handle = this.m_handleIndexBuffer.data[i];
                if (handle) {
                    handle.SetIndex(newIndices(handle.GetIndex()));
                }
            }
        }
        if (this.m_expirationTimeBuffer.data) {
            ///std::rotate(m_expirationTimeBuffer.data + start, m_expirationTimeBuffer.data + mid, m_expirationTimeBuffer.data + end);
            std_rotate(this.m_expirationTimeBuffer.data, start, mid, end);
            // Update expiration time buffer indices.
            var particleCount = this.GetParticleCount();
            if (!this.m_indexByExpirationTimeBuffer.data) {
                throw new Error();
            }
            var indexByExpirationTime = this.m_indexByExpirationTimeBuffer.data;
            for (var i = 0; i < particleCount; ++i) {
                indexByExpirationTime[i] = newIndices(indexByExpirationTime[i]);
            }
        }
        // update proxies
        for (var k = 0; k < this.m_proxyBuffer.count; k++) {
            var proxy = this.m_proxyBuffer.data[k];
            proxy.index = newIndices(proxy.index);
        }
        // update contacts
        for (var k = 0; k < this.m_contactBuffer.count; k++) {
            var contact = this.m_contactBuffer.data[k];
            contact.indexA = newIndices(contact.indexA);
            contact.indexB = newIndices(contact.indexB);
        }
        // update particle-body contacts
        for (var k = 0; k < this.m_bodyContactBuffer.count; k++) {
            var contact = this.m_bodyContactBuffer.data[k];
            contact.index = newIndices(contact.index);
        }
        // update pairs
        for (var k = 0; k < this.m_pairBuffer.count; k++) {
            var pair = this.m_pairBuffer.data[k];
            pair.indexA = newIndices(pair.indexA);
            pair.indexB = newIndices(pair.indexB);
        }
        // update triads
        for (var k = 0; k < this.m_triadBuffer.count; k++) {
            var triad = this.m_triadBuffer.data[k];
            triad.indexA = newIndices(triad.indexA);
            triad.indexB = newIndices(triad.indexB);
            triad.indexC = newIndices(triad.indexC);
        }
        // update groups
        for (var group = this.m_groupList; group; group = group.GetNext()) {
            group.m_firstIndex = newIndices(group.m_firstIndex);
            group.m_lastIndex = newIndices(group.m_lastIndex - 1) + 1;
        }
    };
    b2ParticleSystem.prototype.GetCriticalVelocity = function (step) {
        return this.m_particleDiameter * step.inv_dt;
    };
    b2ParticleSystem.prototype.GetCriticalVelocitySquared = function (step) {
        var velocity = this.GetCriticalVelocity(step);
        return velocity * velocity;
    };
    b2ParticleSystem.prototype.GetCriticalPressure = function (step) {
        return this.m_def.density * this.GetCriticalVelocitySquared(step);
    };
    b2ParticleSystem.prototype.GetParticleStride = function () {
        return b2Settings_2.b2_particleStride * this.m_particleDiameter;
    };
    b2ParticleSystem.prototype.GetParticleMass = function () {
        var stride = this.GetParticleStride();
        return this.m_def.density * stride * stride;
    };
    b2ParticleSystem.prototype.GetParticleInvMass = function () {
        ///return 1.777777 * this.m_inverseDensity * this.m_inverseDiameter * this.m_inverseDiameter;
        // mass = density * stride^2, so we take the inverse of this.
        var inverseStride = this.m_inverseDiameter * (1.0 / b2Settings_2.b2_particleStride);
        return this.m_inverseDensity * inverseStride * inverseStride;
    };
    /**
     * Get the world's contact filter if any particles with the
     * b2_contactFilterParticle flag are present in the system.
     */
    b2ParticleSystem.prototype.GetFixtureContactFilter = function () {
        return (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_fixtureContactFilterParticle) ?
            this.m_world.m_contactManager.m_contactFilter : null;
    };
    /**
     * Get the world's contact filter if any particles with the
     * b2_particleContactFilterParticle flag are present in the
     * system.
     */
    b2ParticleSystem.prototype.GetParticleContactFilter = function () {
        return (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_particleContactFilterParticle) ?
            this.m_world.m_contactManager.m_contactFilter : null;
    };
    /**
     * Get the world's contact listener if any particles with the
     * b2_fixtureContactListenerParticle flag are present in the
     * system.
     */
    b2ParticleSystem.prototype.GetFixtureContactListener = function () {
        return (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_fixtureContactListenerParticle) ?
            this.m_world.m_contactManager.m_contactListener : null;
    };
    /**
     * Get the world's contact listener if any particles with the
     * b2_particleContactListenerParticle flag are present in the
     * system.
     */
    b2ParticleSystem.prototype.GetParticleContactListener = function () {
        return (this.m_allParticleFlags & b2Particle_1.b2ParticleFlag.b2_particleContactListenerParticle) ?
            this.m_world.m_contactManager.m_contactListener : null;
    };
    b2ParticleSystem.prototype.SetUserOverridableBuffer = function (buffer, newData, newCapacity) {
        // DEBUG: b2Assert(((newData !== null) && (newCapacity > 0)) || ((newData === null) && (newCapacity === 0)));
        ///if (!buffer.userSuppliedCapacity)
        ///{
        ///this.m_world.m_blockAllocator.Free(buffer.data, sizeof(T) * m_internalAllocatedCapacity);
        ///}
        buffer.data = newData;
        buffer.userSuppliedCapacity = newCapacity;
    };
    b2ParticleSystem.prototype.SetGroupFlags = function (group, newFlags) {
        var oldFlags = group.m_groupFlags;
        if ((oldFlags ^ newFlags) & b2ParticleGroup_1.b2ParticleGroupFlag.b2_solidParticleGroup) {
            // If the b2_solidParticleGroup flag changed schedule depth update.
            newFlags |= b2ParticleGroup_1.b2ParticleGroupFlag.b2_particleGroupNeedsUpdateDepth;
        }
        if (oldFlags & ~newFlags) {
            // If any flags might be removed
            this.m_needsUpdateAllGroupFlags = true;
        }
        if (~this.m_allGroupFlags & newFlags) {
            // If any flags were added
            if (newFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_solidParticleGroup) {
                this.m_depthBuffer = this.RequestBuffer(this.m_depthBuffer);
            }
            this.m_allGroupFlags |= newFlags;
        }
        group.m_groupFlags = newFlags;
    };
    b2ParticleSystem.BodyContactCompare = function (lhs, rhs) {
        if (lhs.index === rhs.index) {
            // Subsort by weight, decreasing.
            return lhs.weight > rhs.weight;
        }
        return lhs.index < rhs.index;
    };
    b2ParticleSystem.prototype.RemoveSpuriousBodyContacts = function () {
        // At this point we have a list of contact candidates based on AABB
        // overlap.The AABB query that  generated this returns all collidable
        // fixtures overlapping particle bounding boxes.  This breaks down around
        // vertices where two shapes intersect, such as a "ground" surface made
        // of multiple b2PolygonShapes; it potentially applies a lot of spurious
        // impulses from normals that should not actually contribute.  See the
        // Ramp example in Testbed.
        //
        // To correct for this, we apply this algorithm:
        //   * sort contacts by particle and subsort by weight (nearest to farthest)
        //   * for each contact per particle:
        //      - project a point at the contact distance along the inverse of the
        //        contact normal
        //      - if this intersects the fixture that generated the contact, apply
        //         it, otherwise discard as impossible
        //      - repeat for up to n nearest contacts, currently we get good results
        //        from n=3.
        ///std::sort(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(), b2ParticleSystem::BodyContactCompare);
        std_sort(this.m_bodyContactBuffer.data, 0, this.m_bodyContactBuffer.count, b2ParticleSystem.BodyContactCompare);
        ///int32 discarded = 0;
        ///std::remove_if(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(), b2ParticleBodyContactRemovePredicate(this, &discarded));
        ///
        ///m_bodyContactBuffer.SetCount(m_bodyContactBuffer.GetCount() - discarded);
        var s_n = b2ParticleSystem.RemoveSpuriousBodyContacts_s_n;
        var s_pos = b2ParticleSystem.RemoveSpuriousBodyContacts_s_pos;
        var s_normal = b2ParticleSystem.RemoveSpuriousBodyContacts_s_normal;
        // Max number of contacts processed per particle, from nearest to farthest.
        // This must be at least 2 for correctness with concave shapes; 3 was
        // experimentally arrived at as looking reasonable.
        var k_maxContactsPerPoint = 3;
        var system = this;
        // Index of last particle processed.
        var lastIndex = -1;
        // Number of contacts processed for the current particle.
        var currentContacts = 0;
        // Output the number of discarded contacts.
        // let discarded = 0;
        var b2ParticleBodyContactRemovePredicate = function (contact) {
            // This implements the selection criteria described in
            // RemoveSpuriousBodyContacts().
            // This functor is iterating through a list of Body contacts per
            // Particle, ordered from near to far.  For up to the maximum number of
            // contacts we allow per point per step, we verify that the contact
            // normal of the Body that genenerated the contact makes physical sense
            // by projecting a point back along that normal and seeing if it
            // intersects the fixture generating the contact.
            if (contact.index !== lastIndex) {
                currentContacts = 0;
                lastIndex = contact.index;
            }
            if (currentContacts++ > k_maxContactsPerPoint) {
                // ++discarded;
                return true;
            }
            // Project along inverse normal (as returned in the contact) to get the
            // point to check.
            ///b2Vec2 n = contact.normal;
            var n = s_n.Copy(contact.normal);
            // weight is 1-(inv(diameter) * distance)
            ///n *= system.m_particleDiameter * (1 - contact.weight);
            n.SelfMul(system.m_particleDiameter * (1 - contact.weight));
            ///b2Vec2 pos = system.m_positionBuffer.data[contact.index] + n;
            if (!system.m_positionBuffer.data) {
                throw new Error();
            }
            var pos = b2Math_1.b2Vec2.AddVV(system.m_positionBuffer.data[contact.index], n, s_pos);
            // pos is now a point projected back along the contact normal to the
            // contact distance. If the surface makes sense for a contact, pos will
            // now lie on or in the fixture generating
            if (!contact.fixture.TestPoint(pos)) {
                var childCount = contact.fixture.GetShape().GetChildCount();
                for (var childIndex = 0; childIndex < childCount; childIndex++) {
                    var normal = s_normal;
                    var distance = contact.fixture.ComputeDistance(pos, normal, childIndex);
                    if (distance < b2Settings_1.b2_linearSlop) {
                        return false;
                    }
                }
                // ++discarded;
                return true;
            }
            return false;
        };
        this.m_bodyContactBuffer.count = std_remove_if(this.m_bodyContactBuffer.data, b2ParticleBodyContactRemovePredicate, this.m_bodyContactBuffer.count);
    };
    b2ParticleSystem.prototype.DetectStuckParticle = function (particle) {
        // Detect stuck particles
        //
        // The basic algorithm is to allow the user to specify an optional
        // threshold where we detect whenever a particle is contacting
        // more than one fixture for more than threshold consecutive
        // steps. This is considered to be "stuck", and these are put
        // in a list the user can query per step, if enabled, to deal with
        // such particles.
        if (this.m_stuckThreshold <= 0) {
            return;
        }
        if (!this.m_bodyContactCountBuffer.data) {
            throw new Error();
        }
        if (!this.m_consecutiveContactStepsBuffer.data) {
            throw new Error();
        }
        if (!this.m_lastBodyContactStepBuffer.data) {
            throw new Error();
        }
        // Get the state variables for this particle.
        ///int32 * const consecutiveCount = &m_consecutiveContactStepsBuffer.data[particle];
        ///int32 * const lastStep = &m_lastBodyContactStepBuffer.data[particle];
        ///int32 * const bodyCount = &m_bodyContactCountBuffer.data[particle];
        // This is only called when there is a body contact for this particle.
        ///++(*bodyCount);
        ++this.m_bodyContactCountBuffer.data[particle];
        // We want to only trigger detection once per step, the first time we
        // contact more than one fixture in a step for a given particle.
        ///if (*bodyCount === 2)
        if (this.m_bodyContactCountBuffer.data[particle] === 2) {
            ///++(*consecutiveCount);
            ++this.m_consecutiveContactStepsBuffer.data[particle];
            ///if (*consecutiveCount > m_stuckThreshold)
            if (this.m_consecutiveContactStepsBuffer.data[particle] > this.m_stuckThreshold) {
                ///int32& newStuckParticle = m_stuckParticleBuffer.Append();
                ///newStuckParticle = particle;
                this.m_stuckParticleBuffer.data[this.m_stuckParticleBuffer.Append()] = particle;
            }
        }
        ///*lastStep = m_timestamp;
        this.m_lastBodyContactStepBuffer.data[particle] = this.m_timestamp;
    };
    /**
     * Determine whether a particle index is valid.
     */
    b2ParticleSystem.prototype.ValidateParticleIndex = function (index) {
        return index >= 0 && index < this.GetParticleCount() &&
            index !== b2Settings_1.b2_invalidParticleIndex;
    };
    /**
     * Get the time elapsed in
     * b2ParticleSystemDef::lifetimeGranularity.
     */
    b2ParticleSystem.prototype.GetQuantizedTimeElapsed = function () {
        ///return (int32)(m_timeElapsed >> 32);
        return Math.floor(this.m_timeElapsed / 0x100000000);
    };
    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    b2ParticleSystem.prototype.LifetimeToExpirationTime = function (lifetime) {
        ///return m_timeElapsed + (int64)((lifetime / m_def.lifetimeGranularity) * (float32)(1LL << 32));
        return this.m_timeElapsed + Math.floor(((lifetime / this.m_def.lifetimeGranularity) * 0x100000000));
    };
    b2ParticleSystem.prototype.ForceCanBeApplied = function (flags) {
        return !(flags & b2Particle_1.b2ParticleFlag.b2_wallParticle);
    };
    b2ParticleSystem.prototype.PrepareForceBuffer = function () {
        if (!this.m_hasForce) {
            ///memset(m_forceBuffer, 0, sizeof(*m_forceBuffer) * m_count);
            for (var i = 0; i < this.m_count; i++) {
                this.m_forceBuffer[i].SetZero();
            }
            this.m_hasForce = true;
        }
    };
    b2ParticleSystem.prototype.IsRigidGroup = function (group) {
        return (group !== null) && ((group.m_groupFlags & b2ParticleGroup_1.b2ParticleGroupFlag.b2_rigidParticleGroup) !== 0);
    };
    b2ParticleSystem.prototype.GetLinearVelocity = function (group, particleIndex, point, out) {
        if (group && this.IsRigidGroup(group)) {
            return group.GetLinearVelocityFromWorldPoint(point, out);
        }
        else {
            if (!this.m_velocityBuffer.data) {
                throw new Error();
            }
            ///return m_velocityBuffer.data[particleIndex];
            return out.Copy(this.m_velocityBuffer.data[particleIndex]);
        }
    };
    b2ParticleSystem.prototype.InitDampingParameter = function (invMass, invInertia, tangentDistance, mass, inertia, center, point, normal) {
        ///*invMass = mass > 0 ? 1 / mass : 0;
        invMass[0] = mass > 0 ? 1 / mass : 0;
        ///*invInertia = inertia > 0 ? 1 / inertia : 0;
        invInertia[0] = inertia > 0 ? 1 / inertia : 0;
        ///*tangentDistance = b2Cross(point - center, normal);
        tangentDistance[0] = b2Math_1.b2Vec2.CrossVV(b2Math_1.b2Vec2.SubVV(point, center, b2Math_1.b2Vec2.s_t0), normal);
    };
    b2ParticleSystem.prototype.InitDampingParameterWithRigidGroupOrParticle = function (invMass, invInertia, tangentDistance, isRigidGroup, group, particleIndex, point, normal) {
        if (group && isRigidGroup) {
            this.InitDampingParameter(invMass, invInertia, tangentDistance, group.GetMass(), group.GetInertia(), group.GetCenter(), point, normal);
        }
        else {
            if (!this.m_flagsBuffer.data) {
                throw new Error();
            }
            var flags = this.m_flagsBuffer.data[particleIndex];
            this.InitDampingParameter(invMass, invInertia, tangentDistance, flags & b2Particle_1.b2ParticleFlag.b2_wallParticle ? 0 : this.GetParticleMass(), 0, point, point, normal);
        }
    };
    b2ParticleSystem.prototype.ComputeDampingImpulse = function (invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, normalVelocity) {
        var invMass = invMassA + invInertiaA * tangentDistanceA * tangentDistanceA +
            invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
        return invMass > 0 ? normalVelocity / invMass : 0;
    };
    b2ParticleSystem.prototype.ApplyDamping = function (invMass, invInertia, tangentDistance, isRigidGroup, group, particleIndex, impulse, normal) {
        if (group && isRigidGroup) {
            ///group.m_linearVelocity += impulse * invMass * normal;
            group.m_linearVelocity.SelfMulAdd(impulse * invMass, normal);
            ///group.m_angularVelocity += impulse * tangentDistance * invInertia;
            group.m_angularVelocity += impulse * tangentDistance * invInertia;
        }
        else {
            if (!this.m_velocityBuffer.data) {
                throw new Error();
            }
            ///m_velocityBuffer.data[particleIndex] += impulse * invMass * normal;
            this.m_velocityBuffer.data[particleIndex].SelfMulAdd(impulse * invMass, normal);
        }
    };
    b2ParticleSystem.xTruncBits = 12;
    b2ParticleSystem.yTruncBits = 12;
    b2ParticleSystem.tagBits = 8 * 4; // 8u * sizeof(uint32);
    b2ParticleSystem.yOffset = 1 << (b2ParticleSystem.yTruncBits - 1);
    b2ParticleSystem.yShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits;
    b2ParticleSystem.xShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits - b2ParticleSystem.xTruncBits;
    b2ParticleSystem.xScale = 1 << b2ParticleSystem.xShift;
    b2ParticleSystem.xOffset = b2ParticleSystem.xScale * (1 << (b2ParticleSystem.xTruncBits - 1));
    b2ParticleSystem.yMask = ((1 << b2ParticleSystem.yTruncBits) - 1) << b2ParticleSystem.yShift;
    b2ParticleSystem.xMask = ~b2ParticleSystem.yMask;
    b2ParticleSystem.DestroyParticlesInShape_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.CreateParticleGroup_s_transform = new b2Math_1.b2Transform();
    b2ParticleSystem.ComputeCollisionEnergy_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.QueryShapeAABB_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.QueryPointAABB_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.RayCast_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.RayCast_s_p = new b2Math_1.b2Vec2();
    b2ParticleSystem.RayCast_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.RayCast_s_n = new b2Math_1.b2Vec2();
    b2ParticleSystem.RayCast_s_point = new b2Math_1.b2Vec2();
    /**
     * All particle types that require creating pairs
     */
    b2ParticleSystem.k_pairFlags = b2Particle_1.b2ParticleFlag.b2_springParticle;
    /**
     * All particle types that require creating triads
     */
    b2ParticleSystem.k_triadFlags = b2Particle_1.b2ParticleFlag.b2_elasticParticle;
    /**
     * All particle types that do not produce dynamic pressure
     */
    b2ParticleSystem.k_noPressureFlags = b2Particle_1.b2ParticleFlag.b2_powderParticle | b2Particle_1.b2ParticleFlag.b2_tensileParticle;
    /**
     * All particle types that apply extra damping force with bodies
     */
    b2ParticleSystem.k_extraDampingFlags = b2Particle_1.b2ParticleFlag.b2_staticPressureParticle;
    b2ParticleSystem.k_barrierWallFlags = b2Particle_1.b2ParticleFlag.b2_barrierParticle | b2Particle_1.b2ParticleFlag.b2_wallParticle;
    b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_edge = new b2EdgeShape_1.b2EdgeShape();
    b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_d = new b2Math_1.b2Vec2();
    b2ParticleSystem.CreateParticlesStrokeShapeForGroup_s_p = new b2Math_1.b2Vec2();
    b2ParticleSystem.CreateParticlesFillShapeForGroup_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.CreateParticlesFillShapeForGroup_s_p = new b2Math_1.b2Vec2();
    b2ParticleSystem.UpdatePairsAndTriads_s_dab = new b2Math_1.b2Vec2();
    b2ParticleSystem.UpdatePairsAndTriads_s_dbc = new b2Math_1.b2Vec2();
    b2ParticleSystem.UpdatePairsAndTriads_s_dca = new b2Math_1.b2Vec2();
    b2ParticleSystem.AddContact_s_d = new b2Math_1.b2Vec2();
    b2ParticleSystem.UpdateBodyContacts_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.Solve_s_subStep = new b2TimeStep_1.b2TimeStep();
    b2ParticleSystem.SolveCollision_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.SolveGravity_s_gravity = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_aabb = new b2Collision_1.b2AABB();
    b2ParticleSystem.SolveBarrier_s_va = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_vb = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_pba = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_vba = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_vc = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_pca = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_vca = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_qba = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_qca = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_dv = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveBarrier_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolvePressure_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveDamping_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveDamping_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigidDamping_s_t0 = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigidDamping_s_t1 = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigidDamping_s_p = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigidDamping_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveExtraDamping_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveExtraDamping_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigid_s_position = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRigid_s_rotation = new b2Math_1.b2Rot();
    b2ParticleSystem.SolveRigid_s_transform = new b2Math_1.b2Transform();
    b2ParticleSystem.SolveRigid_s_velocityTransform = new b2Math_1.b2Transform();
    b2ParticleSystem.SolveElastic_s_pa = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveElastic_s_pb = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveElastic_s_pc = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveElastic_s_r = new b2Math_1.b2Rot();
    b2ParticleSystem.SolveElastic_s_t0 = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveSpring_s_pa = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveSpring_s_pb = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveSpring_s_d = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveSpring_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveTensile_s_weightedNormal = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveTensile_s_s = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveTensile_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveViscous_s_v = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveViscous_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveRepulsive_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolvePowder_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.SolveSolid_s_f = new b2Math_1.b2Vec2();
    b2ParticleSystem.RemoveSpuriousBodyContacts_s_n = new b2Math_1.b2Vec2();
    b2ParticleSystem.RemoveSpuriousBodyContacts_s_pos = new b2Math_1.b2Vec2();
    b2ParticleSystem.RemoveSpuriousBodyContacts_s_normal = new b2Math_1.b2Vec2();
    return b2ParticleSystem;
}());
exports.b2ParticleSystem = b2ParticleSystem;
(function (b2ParticleSystem) {
    var UserOverridableBuffer = /** @class */ (function () {
        function UserOverridableBuffer() {
            this.data = null;
            this.userSuppliedCapacity = 0;
        }
        return UserOverridableBuffer;
    }());
    b2ParticleSystem.UserOverridableBuffer = UserOverridableBuffer;
    var Proxy = /** @class */ (function () {
        function Proxy() {
            this.index = b2Settings_1.b2_invalidParticleIndex;
            this.tag = 0;
        }
        Proxy.CompareProxyProxy = function (a, b) {
            return a.tag < b.tag;
        };
        Proxy.CompareTagProxy = function (a, b) {
            return a < b.tag;
        };
        Proxy.CompareProxyTag = function (a, b) {
            return a.tag < b;
        };
        return Proxy;
    }());
    b2ParticleSystem.Proxy = Proxy;
    var InsideBoundsEnumerator = /** @class */ (function () {
        /**
         * InsideBoundsEnumerator enumerates all particles inside the
         * given bounds.
         *
         * Construct an enumerator with bounds of tags and a range of
         * proxies.
         */
        function InsideBoundsEnumerator(system, lower, upper, first, last) {
            this.m_system = system;
            this.m_xLower = (lower & b2ParticleSystem.xMask) >>> 0;
            this.m_xUpper = (upper & b2ParticleSystem.xMask) >>> 0;
            this.m_yLower = (lower & b2ParticleSystem.yMask) >>> 0;
            this.m_yUpper = (upper & b2ParticleSystem.yMask) >>> 0;
            this.m_first = first;
            this.m_last = last;
            // DEBUG: b2Assert(this.m_first <= this.m_last);
        }
        /**
         * Get index of the next particle. Returns
         * b2_invalidParticleIndex if there are no more particles.
         */
        InsideBoundsEnumerator.prototype.GetNext = function () {
            while (this.m_first < this.m_last) {
                var xTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & b2ParticleSystem.xMask) >>> 0;
                // #if B2_ASSERT_ENABLED
                // DEBUG: const yTag = (this.m_system.m_proxyBuffer.data[this.m_first].tag & b2ParticleSystem.yMask) >>> 0;
                // DEBUG: b2Assert(yTag >= this.m_yLower);
                // DEBUG: b2Assert(yTag <= this.m_yUpper);
                // #endif
                if (xTag >= this.m_xLower && xTag <= this.m_xUpper) {
                    return (this.m_system.m_proxyBuffer.data[this.m_first++]).index;
                }
                this.m_first++;
            }
            return b2Settings_1.b2_invalidParticleIndex;
        };
        return InsideBoundsEnumerator;
    }());
    b2ParticleSystem.InsideBoundsEnumerator = InsideBoundsEnumerator;
    var ParticleListNode = /** @class */ (function () {
        function ParticleListNode() {
            /**
             * The next node in the list.
             */
            this.next = null;
            /**
             * Number of entries in the list. Valid only for the node at the
             * head of the list.
             */
            this.count = 0;
            /**
             * Particle index.
             */
            this.index = 0;
        }
        return ParticleListNode;
    }());
    b2ParticleSystem.ParticleListNode = ParticleListNode;
    /**
     * @constructor
     */
    var FixedSetAllocator = /** @class */ (function () {
        function FixedSetAllocator() {
        }
        FixedSetAllocator.prototype.Allocate = function (itemSize, count) {
            // TODO
            return count;
        };
        FixedSetAllocator.prototype.Clear = function () {
            // TODO
        };
        FixedSetAllocator.prototype.GetCount = function () {
            // TODO
            return 0;
        };
        FixedSetAllocator.prototype.Invalidate = function (itemIndex) {
            // TODO
        };
        FixedSetAllocator.prototype.GetValidBuffer = function () {
            // TODO
            return [];
        };
        FixedSetAllocator.prototype.GetBuffer = function () {
            // TODO
            return [];
        };
        FixedSetAllocator.prototype.SetCount = function (count) {
            // TODO
        };
        return FixedSetAllocator;
    }());
    b2ParticleSystem.FixedSetAllocator = FixedSetAllocator;
    var FixtureParticle = /** @class */ (function () {
        function FixtureParticle(fixture, particle) {
            this.second = b2Settings_1.b2_invalidParticleIndex;
            this.first = fixture;
            this.second = particle;
        }
        return FixtureParticle;
    }());
    b2ParticleSystem.FixtureParticle = FixtureParticle;
    var FixtureParticleSet = /** @class */ (function (_super) {
        __extends(FixtureParticleSet, _super);
        function FixtureParticleSet() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        FixtureParticleSet.prototype.Initialize = function (bodyContactBuffer, flagsBuffer) {
            // TODO
        };
        FixtureParticleSet.prototype.Find = function (pair) {
            // TODO
            return b2Settings_1.b2_invalidParticleIndex;
        };
        return FixtureParticleSet;
    }(b2ParticleSystem.FixedSetAllocator));
    b2ParticleSystem.FixtureParticleSet = FixtureParticleSet;
    var ParticlePair = /** @class */ (function () {
        function ParticlePair(particleA, particleB) {
            this.first = b2Settings_1.b2_invalidParticleIndex;
            this.second = b2Settings_1.b2_invalidParticleIndex;
            this.first = particleA;
            this.second = particleB;
        }
        return ParticlePair;
    }());
    b2ParticleSystem.ParticlePair = ParticlePair;
    var b2ParticlePairSet = /** @class */ (function (_super) {
        __extends(b2ParticlePairSet, _super);
        function b2ParticlePairSet() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        b2ParticlePairSet.prototype.Initialize = function (contactBuffer, flagsBuffer) {
            // TODO
        };
        b2ParticlePairSet.prototype.Find = function (pair) {
            // TODO
            return b2Settings_1.b2_invalidParticleIndex;
        };
        return b2ParticlePairSet;
    }(b2ParticleSystem.FixedSetAllocator));
    b2ParticleSystem.b2ParticlePairSet = b2ParticlePairSet;
    var ConnectionFilter = /** @class */ (function () {
        function ConnectionFilter() {
        }
        /**
         * Is the particle necessary for connection?
         * A pair or a triad should contain at least one 'necessary'
         * particle.
         */
        ConnectionFilter.prototype.IsNecessary = function (index) {
            return true;
        };
        /**
         * An additional condition for creating a pair.
         */
        ConnectionFilter.prototype.ShouldCreatePair = function (a, b) {
            return true;
        };
        /**
         * An additional condition for creating a triad.
         */
        ConnectionFilter.prototype.ShouldCreateTriad = function (a, b, c) {
            return true;
        };
        return ConnectionFilter;
    }());
    b2ParticleSystem.ConnectionFilter = ConnectionFilter;
    var DestroyParticlesInShapeCallback = /** @class */ (function (_super) {
        __extends(DestroyParticlesInShapeCallback, _super);
        function DestroyParticlesInShapeCallback(system, shape, xf, callDestructionListener) {
            var _this = _super.call(this) || this;
            _this.m_callDestructionListener = false;
            _this.m_destroyed = 0;
            _this.m_system = system;
            _this.m_shape = shape;
            _this.m_xf = xf;
            _this.m_callDestructionListener = callDestructionListener;
            _this.m_destroyed = 0;
            return _this;
        }
        DestroyParticlesInShapeCallback.prototype.ReportFixture = function (fixture) {
            return false;
        };
        DestroyParticlesInShapeCallback.prototype.ReportParticle = function (particleSystem, index) {
            if (particleSystem !== this.m_system) {
                return false;
            }
            // DEBUG: b2Assert(index >= 0 && index < this.m_system.m_count);
            if (!this.m_system.m_positionBuffer.data) {
                throw new Error();
            }
            if (this.m_shape.TestPoint(this.m_xf, this.m_system.m_positionBuffer.data[index])) {
                this.m_system.DestroyParticle(index, this.m_callDestructionListener);
                this.m_destroyed++;
            }
            return true;
        };
        DestroyParticlesInShapeCallback.prototype.Destroyed = function () {
            return this.m_destroyed;
        };
        return DestroyParticlesInShapeCallback;
    }(b2WorldCallbacks_1.b2QueryCallback));
    b2ParticleSystem.DestroyParticlesInShapeCallback = DestroyParticlesInShapeCallback;
    var JoinParticleGroupsFilter = /** @class */ (function (_super) {
        __extends(JoinParticleGroupsFilter, _super);
        function JoinParticleGroupsFilter(threshold) {
            var _this = _super.call(this) || this;
            _this.m_threshold = 0;
            _this.m_threshold = threshold;
            return _this;
        }
        /**
         * An additional condition for creating a pair.
         */
        JoinParticleGroupsFilter.prototype.ShouldCreatePair = function (a, b) {
            return (a < this.m_threshold && this.m_threshold <= b) ||
                (b < this.m_threshold && this.m_threshold <= a);
        };
        /**
         * An additional condition for creating a triad.
         */
        JoinParticleGroupsFilter.prototype.ShouldCreateTriad = function (a, b, c) {
            return (a < this.m_threshold || b < this.m_threshold || c < this.m_threshold) &&
                (this.m_threshold <= a || this.m_threshold <= b || this.m_threshold <= c);
        };
        return JoinParticleGroupsFilter;
    }(b2ParticleSystem.ConnectionFilter));
    b2ParticleSystem.JoinParticleGroupsFilter = JoinParticleGroupsFilter;
    var CompositeShape = /** @class */ (function (_super) {
        __extends(CompositeShape, _super);
        function CompositeShape(shapes, shapeCount) {
            if (shapeCount === void 0) { shapeCount = shapes.length; }
            var _this = _super.call(this, b2Shape_1.b2ShapeType.e_unknown, 0) || this;
            _this.m_shapeCount = 0;
            _this.m_shapes = shapes;
            _this.m_shapeCount = shapeCount;
            return _this;
        }
        CompositeShape.prototype.Clone = function () {
            // DEBUG: b2Assert(false);
            throw new Error();
        };
        CompositeShape.prototype.GetChildCount = function () {
            return 1;
        };
        /**
         * @see b2Shape::TestPoint
         */
        CompositeShape.prototype.TestPoint = function (xf, p) {
            for (var i = 0; i < this.m_shapeCount; i++) {
                if (this.m_shapes[i].TestPoint(xf, p)) {
                    return true;
                }
            }
            return false;
        };
        /**
         * @see b2Shape::ComputeDistance
         */
        CompositeShape.prototype.ComputeDistance = function (xf, p, normal, childIndex) {
            // DEBUG: b2Assert(false);
            return 0;
        };
        /**
         * Implement b2Shape.
         */
        CompositeShape.prototype.RayCast = function (output, input, xf, childIndex) {
            // DEBUG: b2Assert(false);
            return false;
        };
        /**
         * @see b2Shape::ComputeAABB
         */
        CompositeShape.prototype.ComputeAABB = function (aabb, xf, childIndex) {
            var s_subaabb = new b2Collision_1.b2AABB();
            aabb.lowerBound.x = +b2Settings_1.b2_maxFloat;
            aabb.lowerBound.y = +b2Settings_1.b2_maxFloat;
            aabb.upperBound.x = -b2Settings_1.b2_maxFloat;
            aabb.upperBound.y = -b2Settings_1.b2_maxFloat;
            // DEBUG: b2Assert(childIndex === 0);
            for (var i = 0; i < this.m_shapeCount; i++) {
                var childCount = this.m_shapes[i].GetChildCount();
                for (var j = 0; j < childCount; j++) {
                    var subaabb = s_subaabb;
                    this.m_shapes[i].ComputeAABB(subaabb, xf, j);
                    aabb.Combine1(subaabb);
                }
            }
        };
        /**
         * @see b2Shape::ComputeMass
         */
        CompositeShape.prototype.ComputeMass = function (massData, density) {
            // DEBUG: b2Assert(false);
        };
        CompositeShape.prototype.SetupDistanceProxy = function (proxy, index) {
            // DEBUG: b2Assert(false);
        };
        CompositeShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
            // DEBUG: b2Assert(false);
            return 0;
        };
        CompositeShape.prototype.Dump = function (log) {
            // DEBUG: b2Assert(false);
        };
        return CompositeShape;
    }(b2Shape_1.b2Shape));
    b2ParticleSystem.CompositeShape = CompositeShape;
    var ReactiveFilter = /** @class */ (function (_super) {
        __extends(ReactiveFilter, _super);
        function ReactiveFilter(flagsBuffer) {
            var _this = _super.call(this) || this;
            _this.m_flagsBuffer = flagsBuffer;
            return _this;
        }
        ReactiveFilter.prototype.IsNecessary = function (index) {
            if (!this.m_flagsBuffer.data) {
                throw new Error();
            }
            return (this.m_flagsBuffer.data[index] & b2Particle_1.b2ParticleFlag.b2_reactiveParticle) !== 0;
        };
        return ReactiveFilter;
    }(b2ParticleSystem.ConnectionFilter));
    b2ParticleSystem.ReactiveFilter = ReactiveFilter;
    var UpdateBodyContactsCallback = /** @class */ (function (_super) {
        __extends(UpdateBodyContactsCallback, _super);
        function UpdateBodyContactsCallback(system, contactFilter) {
            var _this = _super.call(this, system) || this;
            _this.m_contactFilter = contactFilter;
            return _this;
        }
        UpdateBodyContactsCallback.prototype.ShouldCollideFixtureParticle = function (fixture, particleSystem, particleIndex) {
            // Call the contact filter if it's set, to determine whether to
            // filter this contact.  Returns true if contact calculations should
            // be performed, false otherwise.
            if (this.m_contactFilter) {
                var flags = this.m_system.GetFlagsBuffer();
                if (flags[particleIndex] & b2Particle_1.b2ParticleFlag.b2_fixtureContactFilterParticle) {
                    return this.m_contactFilter.ShouldCollideFixtureParticle(fixture, this.m_system, particleIndex);
                }
            }
            return true;
        };
        UpdateBodyContactsCallback.prototype.ReportFixtureAndParticle = function (fixture, childIndex, a) {
            var s_n = b2ParticleSystem.UpdateBodyContactsCallback.ReportFixtureAndParticle_s_n;
            var s_rp = b2ParticleSystem.UpdateBodyContactsCallback.ReportFixtureAndParticle_s_rp;
            if (!this.m_system.m_flagsBuffer.data) {
                throw new Error();
            }
            if (!this.m_system.m_positionBuffer.data) {
                throw new Error();
            }
            var ap = this.m_system.m_positionBuffer.data[a];
            var n = s_n;
            var d = fixture.ComputeDistance(ap, n, childIndex);
            if (d < this.m_system.m_particleDiameter && this.ShouldCollideFixtureParticle(fixture, this.m_system, a)) {
                var b = fixture.GetBody();
                var bp = b.GetWorldCenter();
                var bm = b.GetMass();
                var bI = b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
                var invBm = bm > 0 ? 1 / bm : 0;
                var invBI = bI > 0 ? 1 / bI : 0;
                var invAm = this.m_system.m_flagsBuffer.data[a] &
                    b2Particle_1.b2ParticleFlag.b2_wallParticle ? 0 : this.m_system.GetParticleInvMass();
                ///b2Vec2 rp = ap - bp;
                var rp = b2Math_1.b2Vec2.SubVV(ap, bp, s_rp);
                var rpn = b2Math_1.b2Vec2.CrossVV(rp, n);
                var invM = invAm + invBm + invBI * rpn * rpn;
                ///b2ParticleBodyContact& contact = m_system.m_bodyContactBuffer.Append();
                var contact = this.m_system.m_bodyContactBuffer.data[this.m_system.m_bodyContactBuffer.Append()];
                contact.index = a;
                contact.body = b;
                contact.fixture = fixture;
                contact.weight = 1 - d * this.m_system.m_inverseDiameter;
                ///contact.normal = -n;
                contact.normal.Copy(n.SelfNeg());
                contact.mass = invM > 0 ? 1 / invM : 0;
                this.m_system.DetectStuckParticle(a);
            }
        };
        UpdateBodyContactsCallback.ReportFixtureAndParticle_s_n = new b2Math_1.b2Vec2();
        UpdateBodyContactsCallback.ReportFixtureAndParticle_s_rp = new b2Math_1.b2Vec2();
        return UpdateBodyContactsCallback;
    }(b2FixtureParticleQueryCallback));
    b2ParticleSystem.UpdateBodyContactsCallback = UpdateBodyContactsCallback;
    var SolveCollisionCallback = /** @class */ (function (_super) {
        __extends(SolveCollisionCallback, _super);
        function SolveCollisionCallback(system, step) {
            var _this = _super.call(this, system) || this;
            _this.m_step = step;
            return _this;
        }
        SolveCollisionCallback.prototype.ReportFixtureAndParticle = function (fixture, childIndex, a) {
            var s_p1 = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_p1;
            var s_output = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_output;
            var s_input = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_input;
            var s_p = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_p;
            var s_v = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_v;
            var s_f = b2ParticleSystem.SolveCollisionCallback.ReportFixtureAndParticle_s_f;
            var body = fixture.GetBody();
            if (!this.m_system.m_positionBuffer.data) {
                throw new Error();
            }
            if (!this.m_system.m_velocityBuffer.data) {
                throw new Error();
            }
            var ap = this.m_system.m_positionBuffer.data[a];
            var av = this.m_system.m_velocityBuffer.data[a];
            var output = s_output;
            var input = s_input;
            if (this.m_system.m_iterationIndex === 0) {
                // Put 'ap' in the local space of the previous frame
                ///b2Vec2 p1 = b2MulT(body.m_xf0, ap);
                var p1 = b2Math_1.b2Transform.MulTXV(body.m_xf0, ap, s_p1);
                if (fixture.GetShape().GetType() === b2Shape_1.b2ShapeType.e_circleShape) {
                    // Make relative to the center of the circle
                    ///p1 -= body.GetLocalCenter();
                    p1.SelfSub(body.GetLocalCenter());
                    // Re-apply rotation about the center of the circle
                    ///p1 = b2Mul(body.m_xf0.q, p1);
                    b2Math_1.b2Rot.MulRV(body.m_xf0.q, p1, p1);
                    // Subtract rotation of the current frame
                    ///p1 = b2MulT(body.m_xf.q, p1);
                    b2Math_1.b2Rot.MulTRV(body.m_xf.q, p1, p1);
                    // Return to local space
                    ///p1 += body.GetLocalCenter();
                    p1.SelfAdd(body.GetLocalCenter());
                }
                // Return to global space and apply rotation of current frame
                ///input.p1 = b2Mul(body.m_xf, p1);
                b2Math_1.b2Transform.MulXV(body.m_xf, p1, input.p1);
            }
            else {
                ///input.p1 = ap;
                input.p1.Copy(ap);
            }
            ///input.p2 = ap + m_step.dt * av;
            b2Math_1.b2Vec2.AddVMulSV(ap, this.m_step.dt, av, input.p2);
            input.maxFraction = 1;
            if (fixture.RayCast(output, input, childIndex)) {
                var n = output.normal;
                ///b2Vec2 p = (1 - output.fraction) * input.p1 + output.fraction * input.p2 + b2_linearSlop * n;
                var p = s_p;
                p.x = (1 - output.fraction) * input.p1.x + output.fraction * input.p2.x + b2Settings_1.b2_linearSlop * n.x;
                p.y = (1 - output.fraction) * input.p1.y + output.fraction * input.p2.y + b2Settings_1.b2_linearSlop * n.y;
                ///b2Vec2 v = m_step.inv_dt * (p - ap);
                var v = s_v;
                v.x = this.m_step.inv_dt * (p.x - ap.x);
                v.y = this.m_step.inv_dt * (p.y - ap.y);
                ///m_system.m_velocityBuffer.data[a] = v;
                this.m_system.m_velocityBuffer.data[a].Copy(v);
                ///b2Vec2 f = m_step.inv_dt * m_system.GetParticleMass() * (av - v);
                var f = s_f;
                f.x = this.m_step.inv_dt * this.m_system.GetParticleMass() * (av.x - v.x);
                f.y = this.m_step.inv_dt * this.m_system.GetParticleMass() * (av.y - v.y);
                this.m_system.ParticleApplyForce(a, f);
            }
        };
        SolveCollisionCallback.prototype.ReportParticle = function (system, index) {
            return false;
        };
        SolveCollisionCallback.ReportFixtureAndParticle_s_p1 = new b2Math_1.b2Vec2();
        SolveCollisionCallback.ReportFixtureAndParticle_s_output = new b2Collision_1.b2RayCastOutput();
        SolveCollisionCallback.ReportFixtureAndParticle_s_input = new b2Collision_1.b2RayCastInput();
        SolveCollisionCallback.ReportFixtureAndParticle_s_p = new b2Math_1.b2Vec2();
        SolveCollisionCallback.ReportFixtureAndParticle_s_v = new b2Math_1.b2Vec2();
        SolveCollisionCallback.ReportFixtureAndParticle_s_f = new b2Math_1.b2Vec2();
        return SolveCollisionCallback;
    }(b2FixtureParticleQueryCallback));
    b2ParticleSystem.SolveCollisionCallback = SolveCollisionCallback;
})(b2ParticleSystem = exports.b2ParticleSystem || (exports.b2ParticleSystem = {}));
exports.b2ParticleSystem = b2ParticleSystem;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQYXJ0aWNsZVN5c3RlbS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL1BhcnRpY2xlL2IyUGFydGljbGVTeXN0ZW0udHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOzs7Ozs7Ozs7Ozs7QUFFSCx5QkFBeUI7QUFFekIsK0VBQStFO0FBQy9FLG1EQUEwTTtBQUMxTSxtREFBNEg7QUFDNUgsMkNBQW1IO0FBQ25ILDJDQUEyQztBQUMzQyx3REFBbUY7QUFDbkYsdURBQStFO0FBQy9FLCtEQUE4RDtBQUU5RCxxREFBb0Q7QUFJcEQsaUVBQXNIO0FBQ3RILDJDQUErRjtBQUMvRixxREFBa0g7QUFDbEgsdURBQXNEO0FBR3RELHVCQUEwQixLQUFVLEVBQUUsQ0FBUyxFQUFFLENBQVM7SUFDeEQsSUFBTSxHQUFHLEdBQU0sS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ3hCLEtBQUssQ0FBQyxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsQ0FBQyxDQUFDLENBQUM7SUFDcEIsS0FBSyxDQUFDLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztBQUNqQixDQUFDO0FBRUQseUJBQTRCLENBQUksRUFBRSxDQUFJLElBQWEsT0FBTyxDQUFDLEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQztBQUVsRSxrQkFBcUIsS0FBVSxFQUFFLEtBQWlCLEVBQUUsR0FBa0MsRUFBRSxHQUE4QztJQUFyRyxzQkFBQSxFQUFBLFNBQWlCO0lBQUUsb0JBQUEsRUFBQSxNQUFjLEtBQUssQ0FBQyxNQUFNLEdBQUcsS0FBSztJQUFFLG9CQUFBLEVBQUEscUJBQThDO0lBQ3BJLElBQUksSUFBSSxHQUFHLEtBQUssQ0FBQztJQUNqQixJQUFNLEtBQUssR0FBYSxFQUFFLENBQUM7SUFDM0IsSUFBSSxHQUFHLEdBQUcsQ0FBQyxDQUFDO0lBRVosU0FBVyxFQUFFLGdCQUFnQjtRQUMzQixPQUFPLElBQUksR0FB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