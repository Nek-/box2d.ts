"use strict";
/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
function b2Assert(condition) {
    var args = [];
    for (var _i = 1; _i < arguments.length; _i++) {
        args[_i - 1] = arguments[_i];
    }
    if (!condition) {
        // debugger;
        throw new (Error.bind.apply(Error, [void 0].concat(args)))();
    }
}
exports.b2Assert = b2Assert;
function b2Maybe(value, def) {
    return value !== undefined ? value : def;
}
exports.b2Maybe = b2Maybe;
exports.b2_maxFloat = 1E+37; // FLT_MAX instead of Number.MAX_VALUE;
exports.b2_epsilon = 1E-5; // FLT_EPSILON instead of Number.MIN_VALUE;
exports.b2_epsilon_sq = (exports.b2_epsilon * exports.b2_epsilon);
exports.b2_pi = 3.14159265359; // Math.PI;
/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///
// Collision
/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
exports.b2_maxManifoldPoints = 2;
/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
exports.b2_maxPolygonVertices = 8;
/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
exports.b2_aabbExtension = 0.1;
/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
exports.b2_aabbMultiplier = 2;
/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
exports.b2_linearSlop = 0.008; // 0.005;
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
exports.b2_angularSlop = 2 / 180 * exports.b2_pi;
/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
exports.b2_polygonRadius = 2 * exports.b2_linearSlop;
/// Maximum number of sub-steps per contact in continuous physics simulation.
exports.b2_maxSubSteps = 8;
// Dynamics
/// Maximum number of contacts to be handled to solve a TOI impact.
exports.b2_maxTOIContacts = 32;
/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
exports.b2_velocityThreshold = 1;
/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
exports.b2_maxLinearCorrection = 0.2;
/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
exports.b2_maxAngularCorrection = 8 / 180 * exports.b2_pi;
/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
exports.b2_maxTranslation = 2;
exports.b2_maxTranslationSquared = exports.b2_maxTranslation * exports.b2_maxTranslation;
/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
exports.b2_maxRotation = 0.5 * exports.b2_pi;
exports.b2_maxRotationSquared = exports.b2_maxRotation * exports.b2_maxRotation;
/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
exports.b2_baumgarte = 0.2;
exports.b2_toiBaumgarte = 0.75;
// #if B2_ENABLE_PARTICLE
// Particle
/// A symbolic constant that stands for particle allocation error.
exports.b2_invalidParticleIndex = -1;
exports.b2_maxParticleIndex = 0x7FFFFFFF;
/// The default distance between particles, multiplied by the particle diameter.
exports.b2_particleStride = 0.75;
/// The minimum particle weight that produces pressure.
exports.b2_minParticleWeight = 1.0;
/// The upper limit for particle pressure.
exports.b2_maxParticlePressure = 0.25;
/// The upper limit for force between particles.
exports.b2_maxParticleForce = 0.5;
/// The maximum distance between particles in a triad, multiplied by the particle diameter.
exports.b2_maxTriadDistance = 2.0;
exports.b2_maxTriadDistanceSquared = (exports.b2_maxTriadDistance * exports.b2_maxTriadDistance);
/// The initial size of particle data buffers.
exports.b2_minParticleSystemBufferCapacity = 256;
/// The time into the future that collisions against barrier particles will be detected.
exports.b2_barrierCollisionTime = 2.5;
// #endif
// Sleep
/// The time that a body must be still before it will go to sleep.
exports.b2_timeToSleep = 0.5;
/// A body cannot sleep if its linear velocity is above this tolerance.
exports.b2_linearSleepTolerance = 0.01;
/// A body cannot sleep if its angular velocity is above this tolerance.
exports.b2_angularSleepTolerance = 2 / 180 * exports.b2_pi;
// Memory Allocation
/// Implement this function to use your own memory allocator.
function b2Alloc(size) {
    return null;
}
exports.b2Alloc = b2Alloc;
/// If you implement b2Alloc, you should also implement this function.
function b2Free(mem) {
}
exports.b2Free = b2Free;
/// Logging function.
function b2Log(message) {
    var args = [];
    for (var _i = 1; _i < arguments.length; _i++) {
        args[_i - 1] = arguments[_i];
    }
    // console.log(message, ...args);
}
exports.b2Log = b2Log;
/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
var b2Version = /** @class */ (function () {
    function b2Version(major, minor, revision) {
        if (major === void 0) { major = 0; }
        if (minor === void 0) { minor = 0; }
        if (revision === void 0) { revision = 0; }
        this.major = 0; ///< significant changes
        this.minor = 0; ///< incremental changes
        this.revision = 0; ///< bug fixes
        this.major = major;
        this.minor = minor;
        this.revision = revision;
    }
    b2Version.prototype.toString = function () {
        return this.major + "." + this.minor + "." + this.revision;
    };
    return b2Version;
}());
exports.b2Version = b2Version;
/// Current version.
exports.b2_version = new b2Version(2, 3, 2);
exports.b2_changelist = 313;
function b2ParseInt(v) {
    return parseInt(v, 10);
}
exports.b2ParseInt = b2ParseInt;
function b2ParseUInt(v) {
    return Math.abs(parseInt(v, 10));
}
exports.b2ParseUInt = b2ParseUInt;
function b2MakeArray(length, init) {
    var a = [];
    for (var i = 0; i < length; ++i) {
        a.push(init(i));
    }
    return a;
}
exports.b2MakeArray = b2MakeArray;
function b2MakeNullArray(length) {
    var a = [];
    for (var i = 0; i < length; ++i) {
        a.push(null);
    }
    return a;
}
exports.b2MakeNullArray = b2MakeNullArray;
function b2MakeNumberArray(length, init) {
    if (init === void 0) { init = 0; }
    var a = [];
    for (var i = 0; i < length; ++i) {
        a.push(init);
    }
    return a;
}
exports.b2MakeNumberArray = b2MakeNumberArray;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJTZXR0aW5ncy5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbW1vbi9iMlNldHRpbmdzLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRixrQkFBeUIsU0FBa0I7SUFBRSxjQUFjO1NBQWQsVUFBYyxFQUFkLHFCQUFjLEVBQWQsSUFBYztRQUFkLDZCQUFjOztJQUN6RCxJQUFJLENBQUMsU0FBUyxFQUFFO1FBQ2QsWUFBWTtRQUNaLFdBQVUsS0FBSyxZQUFMLEtBQUssa0JBQUksSUFBSSxNQUFFO0tBQzFCO0FBQ0gsQ0FBQztBQUxELDRCQUtDO0FBRUQsaUJBQTJCLEtBQW9CLEVBQUUsR0FBTTtJQUNyRCxPQUFPLEtBQUssS0FBSyxTQUFTLENBQUMsQ0FBQyxDQUFDLEtBQUssQ0FBQyxDQUFDLENBQUMsR0FBRyxDQUFDO0FBQzNDLENBQUM7QUFGRCwwQkFFQztBQUVZLFFBQUEsV0FBVyxHQUFXLEtBQUssQ0FBQyxDQUFDLHVDQUF1QztBQUNwRSxRQUFBLFVBQVUsR0FBVyxJQUFJLENBQUMsQ0FBQywyQ0FBMkM7QUFDdEUsUUFBQSxhQUFhLEdBQVcsQ0FBQyxrQkFBVSxHQUFHLGtCQUFVLENBQUMsQ0FBQztBQUNsRCxRQUFBLEtBQUssR0FBVyxhQUFhLENBQUMsQ0FBQyxXQUFXO0FBRXZELFNBQVM7QUFDVCwwRUFBMEU7QUFDMUUsR0FBRztBQUVILFlBQVk7QUFFWixzRUFBc0U7QUFDdEUsMEJBQTBCO0FBQ2IsUUFBQSxvQkFBb0IsR0FBVyxDQUFDLENBQUM7QUFFOUMsMkVBQTJFO0FBQzNFLHFFQUFxRTtBQUN4RCxRQUFBLHFCQUFxQixHQUFXLENBQUMsQ0FBQztBQUUvQyx5RUFBeUU7QUFDekUsbUVBQW1FO0FBQ25FLHNCQUFzQjtBQUNULFFBQUEsZ0JBQWdCLEdBQVcsR0FBRyxDQUFDO0FBRTVDLDZFQUE2RTtBQUM3RSwwREFBMEQ7QUFDMUQsdUNB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