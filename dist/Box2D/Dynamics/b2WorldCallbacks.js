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
var b2Settings_1 = require("../Common/b2Settings");
var b2Body_1 = require("./b2Body");
// #endif
/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
var b2DestructionListener = /** @class */ (function () {
    function b2DestructionListener() {
    }
    /// Called when any joint is about to be destroyed due
    /// to the destruction of one of its attached bodies.
    b2DestructionListener.prototype.SayGoodbyeJoint = function (joint) { };
    /// Called when any fixture is about to be destroyed due
    /// to the destruction of its parent body.
    b2DestructionListener.prototype.SayGoodbyeFixture = function (fixture) { };
    // #if B2_ENABLE_PARTICLE
    /// Called when any particle group is about to be destroyed.
    b2DestructionListener.prototype.SayGoodbyeParticleGroup = function (group) { };
    /// Called when a particle is about to be destroyed.
    /// The index can be used in conjunction with
    /// b2ParticleSystem::GetUserDataBuffer() or
    /// b2ParticleSystem::GetParticleHandleFromIndex() to determine which
    /// particle has been destroyed.
    b2DestructionListener.prototype.SayGoodbyeParticle = function (system, index) { };
    return b2DestructionListener;
}());
exports.b2DestructionListener = b2DestructionListener;
/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
var b2ContactFilter = /** @class */ (function () {
    function b2ContactFilter() {
    }
    /// Return true if contact calculations should be performed between these two shapes.
    /// @warning for performance reasons this is only called when the AABBs begin to overlap.
    b2ContactFilter.prototype.ShouldCollide = function (fixtureA, fixtureB) {
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        // At least one body should be dynamic or kinematic.
        if (bodyB.GetType() === b2Body_1.b2BodyType.b2_staticBody && bodyA.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
            return false;
        }
        // Does a joint prevent collision?
        if (!bodyB.ShouldCollideConnected(bodyA)) {
            return false;
        }
        var filter1 = fixtureA.GetFilterData();
        var filter2 = fixtureB.GetFilterData();
        if (filter1.groupIndex === filter2.groupIndex && filter1.groupIndex !== 0) {
            return (filter1.groupIndex > 0);
        }
        var collide = (((filter1.maskBits & filter2.categoryBits) !== 0) && ((filter1.categoryBits & filter2.maskBits) !== 0));
        return collide;
    };
    // #if B2_ENABLE_PARTICLE
    b2ContactFilter.prototype.ShouldCollideFixtureParticle = function (fixture, system, index) {
        return true;
    };
    b2ContactFilter.prototype.ShouldCollideParticleParticle = function (system, indexA, indexB) {
        return true;
    };
    // #endif
    b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
    return b2ContactFilter;
}());
exports.b2ContactFilter = b2ContactFilter;
/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in b2Manifold.
var b2ContactImpulse = /** @class */ (function () {
    function b2ContactImpulse() {
        this.normalImpulses = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxManifoldPoints);
        this.tangentImpulses = b2Settings_1.b2MakeNumberArray(b2Settings_1.b2_maxManifoldPoints);
        this.count = 0;
    }
    return b2ContactImpulse;
}());
exports.b2ContactImpulse = b2ContactImpulse;
/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
var b2ContactListener = /** @class */ (function () {
    function b2ContactListener() {
    }
    /// Called when two fixtures begin to touch.
    b2ContactListener.prototype.BeginContact = function (contact) { };
    /// Called when two fixtures cease to touch.
    b2ContactListener.prototype.EndContact = function (contact) { };
    // #if B2_ENABLE_PARTICLE
    b2ContactListener.prototype.BeginContactFixtureParticle = function (system, contact) { };
    b2ContactListener.prototype.EndContactFixtureParticle = function (system, contact) { };
    b2ContactListener.prototype.BeginContactParticleParticle = function (system, contact) { };
    b2ContactListener.prototype.EndContactParticleParticle = function (system, contact) { };
    // #endif
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. disable contact).
    /// A copy of the old manifold is provided so that you can detect changes.
    /// Note: this is called only for awake bodies.
    /// Note: this is called even when the number of contact points is zero.
    /// Note: this is not called for sensors.
    /// Note: if you set the number of contact points to zero, you will not
    /// get an EndContact callback. However, you may get a BeginContact callback
    /// the next step.
    b2ContactListener.prototype.PreSolve = function (contact, oldManifold) { };
    /// This lets you inspect a contact after the solver is finished. This is useful
    /// for inspecting impulses.
    /// Note: the contact manifold does not include time of impact impulses, which can be
    /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    /// in a separate data structure.
    /// Note: this is only called for contacts that are touching, solid, and awake.
    b2ContactListener.prototype.PostSolve = function (contact, impulse) { };
    b2ContactListener.b2_defaultListener = new b2ContactListener();
    return b2ContactListener;
}());
exports.b2ContactListener = b2ContactListener;
/// Callback class for AABB queries.
/// See b2World::Query
var b2QueryCallback = /** @class */ (function () {
    function b2QueryCallback() {
    }
    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    b2QueryCallback.prototype.ReportFixture = function (fixture) {
        return true;
    };
    // #if B2_ENABLE_PARTICLE
    b2QueryCallback.prototype.ReportParticle = function (system, index) {
        return false;
    };
    b2QueryCallback.prototype.ShouldQueryParticleSystem = function (system) {
        return true;
    };
    return b2QueryCallback;
}());
exports.b2QueryCallback = b2QueryCallback;
/// Callback class for ray casts.
/// See b2World::RayCast
var b2RayCastCallback = /** @class */ (function () {
    function b2RayCastCallback() {
    }
    /// Called for each fixture found in the query. You control how the ray cast
    /// proceeds by returning a float:
    /// return -1: ignore this fixture and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    /// @param fixture the fixture hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection
    /// @return -1 to filter, 0 to terminate, fraction to clip the ray for
    /// closest hit, 1 to continue
    b2RayCastCallback.prototype.ReportFixture = function (fixture, point, normal, fraction) {
        return fraction;
    };
    // #if B2_ENABLE_PARTICLE
    b2RayCastCallback.prototype.ReportParticle = function (system, index, point, normal, fraction) {
        return 0;
    };
    b2RayCastCallback.prototype.ShouldQueryParticleSystem = function (system) {
        return true;
    };
    return b2RayCastCallback;
}());
exports.b2RayCastCallback = b2RayCastCallback;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJXb3JsZENhbGxiYWNrcy5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL2IyV29ybGRDYWxsYmFja3MudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLG1EQUErRTtBQUkvRSxtQ0FBOEM7QUFNOUMsU0FBUztBQUVULDJEQUEyRDtBQUMzRCwwREFBMEQ7QUFDMUQsc0RBQXNEO0FBQ3REO0lBQUE7SUFvQkEsQ0FBQztJQW5CQyxzREFBc0Q7SUFDdEQscURBQXFEO0lBQzlDLCtDQUFlLEdBQXRCLFVBQXVCLEtBQWMsSUFBUyxDQUFDO0lBRS9DLHdEQUF3RDtJQUN4RCwwQ0FBMEM7SUFDbkMsaURBQWlCLEdBQXhCLFVBQXlCLE9BQWtCLElBQVMsQ0FBQztJQUVyRCx5QkFBeUI7SUFDekIsNERBQTREO0lBQ3JELHVEQUF1QixHQUE5QixVQUErQixLQUFzQixJQUFTLENBQUM7SUFFL0Qsb0RBQW9EO0lBQ3BELDZDQUE2QztJQUM3Qyw0Q0FBNEM7SUFDNUMscUVBQXFFO0lBQ3JFLGdDQUFnQztJQUN6QixrREFBa0IsR0FBekIsVUFBMEIsTUFBd0IsRUFBRSxLQUFhLElBQVMsQ0FBQztJQUU3RSw0QkFBQztBQUFELENBQUMsQUFwQkQsSUFvQkM7QUFwQlksc0RBQXFCO0FBc0JsQywwRkFBMEY7QUFDMUYsK0RBQStEO0FBQy9EO0lBQUE7SUF1Q0EsQ0FBQztJQXRDQyxxRkFBcUY7SUFDckYseUZBQXlGO0lBQ2xGLHVDQUFhLEdBQXBCLFVBQXFCLFFBQW1CLEVBQUUsUUFBbUI7UUFDM0QsSUFBTSxLQUFLLEdBQVcsUUFBUSxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQ3pDLElBQU0sS0FBSyxHQUFXLFFBQVEsQ0FBQyxPQUFPLEVBQUUsQ0FBQztRQUV6QyxvREFBb0Q7UUFDcEQsSUFBSSxLQUFLLENBQUMsT0FBTyxFQUFFLEtBQUssbUJBQVUsQ0FBQyxh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