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
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Timer_1 = require("../Common/b2Timer");
var b2Draw_1 = require("../Common/b2Draw");
var b2Collision_1 = require("../Collision/b2Collision");
var b2TimeOfImpact_1 = require("../Collision/b2TimeOfImpact");
var b2Shape_1 = require("../Collision/Shapes/b2Shape");
var b2Joint_1 = require("./Joints/b2Joint");
var b2JointFactory_1 = require("./Joints/b2JointFactory");
var b2Body_1 = require("./b2Body");
var b2ContactManager_1 = require("./b2ContactManager");
var b2Island_1 = require("./b2Island");
var b2TimeStep_1 = require("./b2TimeStep");
var b2WorldCallbacks_1 = require("./b2WorldCallbacks");
// #if B2_ENABLE_PARTICLE
var b2Settings_2 = require("../Common/b2Settings");
var b2Particle_1 = require("../Particle/b2Particle");
var b2ParticleSystem_1 = require("../Particle/b2ParticleSystem");
// #endif
/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
var b2World = /** @class */ (function () {
    // #endif
    /// Construct a world object.
    /// @param gravity the world gravity vector.
    function b2World(gravity) {
        // b2BlockAllocator m_blockAllocator;
        // b2StackAllocator m_stackAllocator;
        this.m_newFixture = false;
        this.m_locked = false;
        this.m_clearForces = true;
        this.m_contactManager = new b2ContactManager_1.b2ContactManager();
        this.m_bodyList = null;
        this.m_jointList = null;
        // #if B2_ENABLE_PARTICLE
        this.m_particleSystemList = null;
        // #endif
        this.m_bodyCount = 0;
        this.m_jointCount = 0;
        this.m_gravity = new b2Math_1.b2Vec2();
        this.m_allowSleep = true;
        this.m_destructionListener = null;
        this.m_debugDraw = null;
        // This is used to compute the time step ratio to
        // support a variable time step.
        this.m_inv_dt0 = 0;
        // These are for debugging the solver.
        this.m_warmStarting = true;
        this.m_continuousPhysics = true;
        this.m_subStepping = false;
        this.m_stepComplete = true;
        this.m_profile = new b2TimeStep_1.b2Profile();
        this.m_island = new b2Island_1.b2Island();
        this.s_stack = [];
        // #if B2_ENABLE_CONTROLLER
        this.m_controllerList = null;
        this.m_controllerCount = 0;
        this.m_gravity.Copy(gravity);
    }
    /// Register a destruction listener. The listener is owned by you and must
    /// remain in scope.
    b2World.prototype.SetDestructionListener = function (listener) {
        this.m_destructionListener = listener;
    };
    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used (b2_defaultFilter). The listener is
    /// owned by you and must remain in scope.
    b2World.prototype.SetContactFilter = function (filter) {
        this.m_contactManager.m_contactFilter = filter;
    };
    /// Register a contact event listener. The listener is owned by you and must
    /// remain in scope.
    b2World.prototype.SetContactListener = function (listener) {
        this.m_contactManager.m_contactListener = listener;
    };
    /// Register a routine for debug drawing. The debug draw functions are called
    /// inside with b2World::DrawDebugData method. The debug draw object is owned
    /// by you and must remain in scope.
    b2World.prototype.SetDebugDraw = function (debugDraw) {
        this.m_debugDraw = debugDraw;
    };
    /// Create a rigid body given a definition. No reference to the definition
    /// is retained.
    /// @warning This function is locked during callbacks.
    b2World.prototype.CreateBody = function (def) {
        if (def === void 0) { def = {}; }
        if (this.IsLocked()) {
            throw new Error();
        }
        var b = new b2Body_1.b2Body(def, this);
        // Add to world doubly linked list.
        b.m_prev = null;
        b.m_next = this.m_bodyList;
        if (this.m_bodyList) {
            this.m_bodyList.m_prev = b;
        }
        this.m_bodyList = b;
        ++this.m_bodyCount;
        return b;
    };
    /// Destroy a rigid body given a definition. No reference to the definition
    /// is retained. This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    b2World.prototype.DestroyBody = function (b) {
        // DEBUG: b2Assert(this.m_bodyCount > 0);
        if (this.IsLocked()) {
            throw new Error();
        }
        // Delete the attached joints.
        var je = b.m_jointList;
        while (je) {
            var je0 = je;
            je = je.next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeJoint(je0.joint);
            }
            this.DestroyJoint(je0.joint);
            b.m_jointList = je;
        }
        b.m_jointList = null;
        // #if B2_ENABLE_CONTROLLER
        // @see b2Controller list
        var coe = b.m_controllerList;
        while (coe) {
            var coe0 = coe;
            coe = coe.nextController;
            coe0.controller.RemoveBody(b);
        }
        // #endif
        // Delete the attached contacts.
        var ce = b.m_contactList;
        while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.m_contactManager.Destroy(ce0.contact);
        }
        b.m_contactList = null;
        // Delete the attached fixtures. This destroys broad-phase proxies.
        var f = b.m_fixtureList;
        while (f) {
            var f0 = f;
            f = f.m_next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeFixture(f0);
            }
            f0.DestroyProxies(this.m_contactManager.m_broadPhase);
            f0.Destroy();
            b.m_fixtureList = f;
            b.m_fixtureCount -= 1;
        }
        b.m_fixtureList = null;
        b.m_fixtureCount = 0;
        // Remove world body list.
        if (b.m_prev) {
            b.m_prev.m_next = b.m_next;
        }
        if (b.m_next) {
            b.m_next.m_prev = b.m_prev;
        }
        if (b === this.m_bodyList) {
            this.m_bodyList = b.m_next;
        }
        --this.m_bodyCount;
    };
    /// Create a joint to constrain bodies together. No reference to the definition
    /// is retained. This may cause the connected bodies to cease colliding.
    /// @warning This function is locked during callbacks.
    b2World.prototype.CreateJoint = function (def) {
        if (this.IsLocked()) {
            throw new Error();
        }
        var j = b2JointFactory_1.b2JointFactory.Create(def, null);
        // Connect to the world list.
        j.m_prev = null;
        j.m_next = this.m_jointList;
        if (this.m_jointList) {
            this.m_jointList.m_prev = j;
        }
        this.m_jointList = j;
        ++this.m_jointCount;
        // Connect to the bodies' doubly linked lists.
        // j.m_edgeA.joint = j;
        // j.m_edgeA.other = j.m_bodyB;
        j.m_edgeA.prev = null;
        j.m_edgeA.next = j.m_bodyA.m_jointList;
        if (j.m_bodyA.m_jointList) {
            j.m_bodyA.m_jointList.prev = j.m_edgeA;
        }
        j.m_bodyA.m_jointList = j.m_edgeA;
        // j.m_edgeB.joint = j;
        // j.m_edgeB.other = j.m_bodyA;
        j.m_edgeB.prev = null;
        j.m_edgeB.next = j.m_bodyB.m_jointList;
        if (j.m_bodyB.m_jointList) {
            j.m_bodyB.m_jointList.prev = j.m_edgeB;
        }
        j.m_bodyB.m_jointList = j.m_edgeB;
        var bodyA = def.bodyA;
        var bodyB = def.bodyB;
        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!def.collideConnected) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
        // Note: creating a joint doesn't wake the bodies.
        return j;
    };
    /// Destroy a joint. This may cause the connected bodies to begin colliding.
    /// @warning This function is locked during callbacks.
    b2World.prototype.DestroyJoint = function (j) {
        if (this.IsLocked()) {
            throw new Error();
        }
        var collideConnected = j.m_collideConnected;
        // Remove from the doubly linked list.
        if (j.m_prev) {
            j.m_prev.m_next = j.m_next;
        }
        if (j.m_next) {
            j.m_next.m_prev = j.m_prev;
        }
        if (j === this.m_jointList) {
            this.m_jointList = j.m_next;
        }
        // Disconnect from island graph.
        var bodyA = j.m_bodyA;
        var bodyB = j.m_bodyB;
        // Wake up connected bodies.
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
        // Remove from body 1.
        if (j.m_edgeA.prev) {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }
        if (j.m_edgeA.next) {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }
        if (j.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next;
        }
        j.m_edgeA.prev = null;
        j.m_edgeA.next = null;
        // Remove from body 2
        if (j.m_edgeB.prev) {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }
        if (j.m_edgeB.next) {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }
        if (j.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next;
        }
        j.m_edgeB.prev = null;
        j.m_edgeB.next = null;
        b2JointFactory_1.b2JointFactory.Destroy(j, null);
        // DEBUG: b2Assert(this.m_jointCount > 0);
        --this.m_jointCount;
        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
    };
    // #if B2_ENABLE_PARTICLE
    b2World.prototype.CreateParticleSystem = function (def) {
        if (this.IsLocked()) {
            throw new Error();
        }
        var p = new b2ParticleSystem_1.b2ParticleSystem(def, this);
        // Add to world doubly linked list.
        p.m_prev = null;
        p.m_next = this.m_particleSystemList;
        if (this.m_particleSystemList) {
            this.m_particleSystemList.m_prev = p;
        }
        this.m_particleSystemList = p;
        return p;
    };
    b2World.prototype.DestroyParticleSystem = function (p) {
        if (this.IsLocked()) {
            throw new Error();
        }
        // Remove world particleSystem list.
        if (p.m_prev) {
            p.m_prev.m_next = p.m_next;
        }
        if (p.m_next) {
            p.m_next.m_prev = p.m_prev;
        }
        if (p === this.m_particleSystemList) {
            this.m_particleSystemList = p.m_next;
        }
    };
    b2World.prototype.CalculateReasonableParticleIterations = function (timeStep) {
        if (this.m_particleSystemList === null) {
            return 1;
        }
        function GetSmallestRadius(world) {
            var smallestRadius = b2Settings_2.b2_maxFloat;
            for (var system = world.GetParticleSystemList(); system !== null; system = system.m_next) {
                smallestRadius = b2Math_1.b2Min(smallestRadius, system.GetRadius());
            }
            return smallestRadius;
        }
        // Use the smallest radius, since that represents the worst-case.
        return b2Particle_1.b2CalculateParticleIterations(this.m_gravity.Length(), GetSmallestRadius(this), timeStep);
    };
    // #if B2_ENABLE_PARTICLE
    b2World.prototype.Step = function (dt, velocityIterations, positionIterations, particleIterations) {
        if (particleIterations === void 0) { particleIterations = this.CalculateReasonableParticleIterations(dt); }
        // #else
        // public Step(dt: number, velocityIterations: number, positionIterations: number): void {
        // #endif
        var stepTimer = b2World.Step_s_stepTimer.Reset();
        // If new fixtures were added, we need to find the new contacts.
        if (this.m_newFixture) {
            this.m_contactManager.FindNewContacts();
            this.m_newFixture = false;
        }
        this.m_locked = true;
        var step = b2World.Step_s_step;
        step.dt = dt;
        step.velocityIterations = velocityIterations;
        step.positionIterations = positionIterations;
        // #if B2_ENABLE_PARTICLE
        step.particleIterations = particleIterations;
        // #endif
        if (dt > 0) {
            step.inv_dt = 1 / dt;
        }
        else {
            step.inv_dt = 0;
        }
        step.dtRatio = this.m_inv_dt0 * dt;
        step.warmStarting = this.m_warmStarting;
        // Update contacts. This is where some contacts are destroyed.
        var timer = b2World.Step_s_timer.Reset();
        this.m_contactManager.Collide();
        this.m_profile.collide = timer.GetMilliseconds();
        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (this.m_stepComplete && step.dt > 0) {
            var timer_1 = b2World.Step_s_timer.Reset();
            // #if B2_ENABLE_PARTICLE
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                p.Solve(step); // Particle Simulation
            }
            // #endif
            this.Solve(step);
            this.m_profile.solve = timer_1.GetMilliseconds();
        }
        // Handle TOI events.
        if (this.m_continuousPhysics && step.dt > 0) {
            var timer_2 = b2World.Step_s_timer.Reset();
            this.SolveTOI(step);
            this.m_profile.solveTOI = timer_2.GetMilliseconds();
        }
        if (step.dt > 0) {
            this.m_inv_dt0 = step.inv_dt;
        }
        if (this.m_clearForces) {
            this.ClearForces();
        }
        this.m_locked = false;
        this.m_profile.step = stepTimer.GetMilliseconds();
    };
    /// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
    /// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
    /// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
    /// a fixed sized time step under a variable frame-rate.
    /// When you perform sub-stepping you will disable auto clearing of forces and instead call
    /// ClearForces after all sub-steps are complete in one pass of your game loop.
    /// @see SetAutoClearForces
    b2World.prototype.ClearForces = function () {
        for (var body = this.m_bodyList; body; body = body.m_next) {
            body.m_force.SetZero();
            body.m_torque = 0;
        }
    };
    // #if B2_ENABLE_PARTICLE
    b2World.prototype.DrawParticleSystem = function (system) {
        if (this.m_debugDraw === null) {
            return;
        }
        var particleCount = system.GetParticleCount();
        if (particleCount) {
            var radius = system.GetRadius();
            var positionBuffer = system.GetPositionBuffer();
            if (system.m_colorBuffer.data) {
                var colorBuffer = system.GetColorBuffer();
                this.m_debugDraw.DrawParticles(positionBuffer, radius, colorBuffer, particleCount);
            }
            else {
                this.m_debugDraw.DrawParticles(positionBuffer, radius, null, particleCount);
            }
        }
    };
    b2World.prototype.DrawDebugData = function () {
        if (this.m_debugDraw === null) {
            return;
        }
        var flags = this.m_debugDraw.GetFlags();
        var color = b2World.DrawDebugData_s_color.SetRGB(0, 0, 0);
        if (flags & b2Draw_1.b2DrawFlags.e_shapeBit) {
            for (var b = this.m_bodyList; b; b = b.m_next) {
                var xf = b.m_xf;
                this.m_debugDraw.PushTransform(xf);
                for (var f = b.GetFixtureList(); f; f = f.m_next) {
                    if (!b.IsActive()) {
                        color.SetRGB(0.5, 0.5, 0.3);
                        this.DrawShape(f, color);
                    }
                    else if (b.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                        color.SetRGB(0.5, 0.9, 0.5);
                        this.DrawShape(f, color);
                    }
                    else if (b.GetType() === b2Body_1.b2BodyType.b2_kinematicBody) {
                        color.SetRGB(0.5, 0.5, 0.9);
                        this.DrawShape(f, color);
                    }
                    else if (!b.IsAwake()) {
                        color.SetRGB(0.6, 0.6, 0.6);
                        this.DrawShape(f, color);
                    }
                    else {
                        color.SetRGB(0.9, 0.7, 0.7);
                        this.DrawShape(f, color);
                    }
                }
                this.m_debugDraw.PopTransform(xf);
            }
        }
        // #if B2_ENABLE_PARTICLE
        if (flags & b2Draw_1.b2DrawFlags.e_particleBit) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                this.DrawParticleSystem(p);
            }
        }
        // #endif
        if (flags & b2Draw_1.b2DrawFlags.e_jointBit) {
            for (var j = this.m_jointList; j; j = j.m_next) {
                this.DrawJoint(j);
            }
        }
        /*
        if (flags & b2DrawFlags.e_pairBit) {
          color.SetRGB(0.3, 0.9, 0.9);
          for (let contact = this.m_contactManager.m_contactList; contact; contact = contact.m_next) {
            const fixtureA = contact.GetFixtureA();
            const fixtureB = contact.GetFixtureB();
    
            const cA = fixtureA.GetAABB().GetCenter();
            const cB = fixtureB.GetAABB().GetCenter();
    
            this.m_debugDraw.DrawSegment(cA, cB, color);
          }
        }
        */
        if (flags & b2Draw_1.b2DrawFlags.e_aabbBit) {
            color.SetRGB(0.9, 0.3, 0.9);
            var bp = this.m_contactManager.m_broadPhase;
            var vs = b2World.DrawDebugData_s_vs;
            for (var b = this.m_bodyList; b; b = b.m_next) {
                if (!b.IsActive()) {
                    continue;
                }
                for (var f = b.GetFixtureList(); f; f = f.m_next) {
                    for (var i = 0; i < f.m_proxyCount; ++i) {
                        var proxy = f.m_proxies[i];
                        var aabb = bp.GetFatAABB(proxy.treeNode);
                        vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                        vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                        vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                        vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
                        this.m_debugDraw.DrawPolygon(vs, 4, color);
                    }
                }
            }
        }
        if (flags & b2Draw_1.b2DrawFlags.e_centerOfMassBit) {
            for (var b = this.m_bodyList; b; b = b.m_next) {
                var xf = b2World.DrawDebugData_s_xf;
                xf.q.Copy(b.m_xf.q);
                xf.p.Copy(b.GetWorldCenter());
                this.m_debugDraw.DrawTransform(xf);
            }
        }
        // #if B2_ENABLE_CONTROLLER
        // @see b2Controller list
        if (flags & b2Draw_1.b2DrawFlags.e_controllerBit) {
            for (var c = this.m_controllerList; c; c = c.m_next) {
                c.Draw(this.m_debugDraw);
            }
        }
        // #endif
    };
    /// Query the world for all fixtures that potentially overlap the
    /// provided AABB.
    /// @param callback a user implemented callback class.
    /// @param aabb the query box.
    b2World.prototype.QueryAABB = function (callback, aabb, fn) {
        var broadPhase = this.m_contactManager.m_broadPhase;
        broadPhase.Query(aabb, function (proxy) {
            var fixture_proxy = broadPhase.GetUserData(proxy);
            // DEBUG: b2Assert(fixture_proxy instanceof b2FixtureProxy);
            var fixture = fixture_proxy.fixture;
            if (callback) {
                return callback.ReportFixture(fixture);
            }
            else if (fn) {
                return fn(fixture);
            }
            return true;
        });
        // #if B2_ENABLE_PARTICLE
        if (callback instanceof b2WorldCallbacks_1.b2QueryCallback) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryAABB(callback, aabb);
                }
            }
        }
        // #endif
    };
    b2World.prototype.QueryAllAABB = function (aabb, out) {
        if (out === void 0) { out = []; }
        this.QueryAABB(null, aabb, function (fixture) { out.push(fixture); return true; });
        return out;
    };
    /// Query the world for all fixtures that potentially overlap the
    /// provided point.
    /// @param callback a user implemented callback class.
    /// @param point the query point.
    b2World.prototype.QueryPointAABB = function (callback, point, fn) {
        var broadPhase = this.m_contactManager.m_broadPhase;
        broadPhase.QueryPoint(point, function (proxy) {
            var fixture_proxy = broadPhase.GetUserData(proxy);
            // DEBUG: b2Assert(fixture_proxy instanceof b2FixtureProxy);
            var fixture = fixture_proxy.fixture;
            if (callback) {
                return callback.ReportFixture(fixture);
            }
            else if (fn) {
                return fn(fixture);
            }
            return true;
        });
        // #if B2_ENABLE_PARTICLE
        if (callback instanceof b2WorldCallbacks_1.b2QueryCallback) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryPointAABB(callback, point);
                }
            }
        }
        // #endif
    };
    b2World.prototype.QueryAllPointAABB = function (point, out) {
        if (out === void 0) { out = []; }
        this.QueryPointAABB(null, point, function (fixture) { out.push(fixture); return true; });
        return out;
    };
    b2World.prototype.QueryFixtureShape = function (callback, shape, index, transform, fn) {
        var broadPhase = this.m_contactManager.m_broadPhase;
        var aabb = b2World.QueryFixtureShape_s_aabb;
        shape.ComputeAABB(aabb, transform, index);
        broadPhase.Query(aabb, function (proxy) {
            var fixture_proxy = broadPhase.GetUserData(proxy);
            // DEBUG: b2Assert(fixture_proxy instanceof b2FixtureProxy);
            var fixture = fixture_proxy.fixture;
            if (b2Collision_1.b2TestOverlapShape(shape, index, fixture.GetShape(), fixture_proxy.childIndex, transform, fixture.GetBody().GetTransform())) {
                if (callback) {
                    return callback.ReportFixture(fixture);
                }
                else if (fn) {
                    return fn(fixture);
                }
            }
            return true;
        });
        // #if B2_ENABLE_PARTICLE
        if (callback instanceof b2WorldCallbacks_1.b2QueryCallback) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryAABB(callback, aabb);
                }
            }
        }
        // #endif
    };
    b2World.prototype.QueryAllFixtureShape = function (shape, index, transform, out) {
        if (out === void 0) { out = []; }
        this.QueryFixtureShape(null, shape, index, transform, function (fixture) { out.push(fixture); return true; });
        return out;
    };
    b2World.prototype.QueryFixturePoint = function (callback, point, fn) {
        var broadPhase = this.m_contactManager.m_broadPhase;
        broadPhase.QueryPoint(point, function (proxy) {
            var fixture_proxy = broadPhase.GetUserData(proxy);
            // DEBUG: b2Assert(fixture_proxy instanceof b2FixtureProxy);
            var fixture = fixture_proxy.fixture;
            if (fixture.TestPoint(point)) {
                if (callback) {
                    return callback.ReportFixture(fixture);
                }
                else if (fn) {
                    return fn(fixture);
                }
            }
            return true;
        });
        // #if B2_ENABLE_PARTICLE
        if (callback) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.QueryPointAABB(callback, point);
                }
            }
        }
        // #endif
    };
    b2World.prototype.QueryAllFixturePoint = function (point, out) {
        if (out === void 0) { out = []; }
        this.QueryFixturePoint(null, point, function (fixture) { out.push(fixture); return true; });
        return out;
    };
    b2World.prototype.RayCast = function (callback, point1, point2, fn) {
        var broadPhase = this.m_contactManager.m_broadPhase;
        var input = b2World.RayCast_s_input;
        input.maxFraction = 1;
        input.p1.Copy(point1);
        input.p2.Copy(point2);
        broadPhase.RayCast(input, function (input, proxy) {
            var fixture_proxy = broadPhase.GetUserData(proxy);
            // DEBUG: b2Assert(fixture_proxy instanceof b2FixtureProxy);
            var fixture = fixture_proxy.fixture;
            var index = fixture_proxy.childIndex;
            var output = b2World.RayCast_s_output;
            var hit = fixture.RayCast(output, input, index);
            if (hit) {
                var fraction = output.fraction;
                var point = b2World.RayCast_s_point;
                point.Set((1 - fraction) * point1.x + fraction * point2.x, (1 - fraction) * point1.y + fraction * point2.y);
                if (callback) {
                    return callback.ReportFixture(fixture, point, output.normal, fraction);
                }
                else if (fn) {
                    return fn(fixture, point, output.normal, fraction);
                }
            }
            return input.maxFraction;
        });
        // #if B2_ENABLE_PARTICLE
        if (callback) {
            for (var p = this.m_particleSystemList; p; p = p.m_next) {
                if (callback.ShouldQueryParticleSystem(p)) {
                    p.RayCast(callback, point1, point2);
                }
            }
        }
        // #endif
    };
    b2World.prototype.RayCastOne = function (point1, point2) {
        var result = null;
        var min_fraction = 1;
        this.RayCast(null, point1, point2, function (fixture, point, normal, fraction) {
            if (fraction < min_fraction) {
                min_fraction = fraction;
                result = fixture;
            }
            return min_fraction;
        });
        return result;
    };
    b2World.prototype.RayCastAll = function (point1, point2, out) {
        if (out === void 0) { out = []; }
        this.RayCast(null, point1, point2, function (fixture, point, normal, fraction) {
            out.push(fixture);
            return 1;
        });
        return out;
    };
    /// Get the world body list. With the returned body, use b2Body::GetNext to get
    /// the next body in the world list. A NULL body indicates the end of the list.
    /// @return the head of the world body list.
    b2World.prototype.GetBodyList = function () {
        return this.m_bodyList;
    };
    /// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
    /// the next joint in the world list. A NULL joint indicates the end of the list.
    /// @return the head of the world joint list.
    b2World.prototype.GetJointList = function () {
        return this.m_jointList;
    };
    // #if B2_ENABLE_PARTICLE
    b2World.prototype.GetParticleSystemList = function () {
        return this.m_particleSystemList;
    };
    // #endif
    /// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
    /// the next contact in the world list. A NULL contact indicates the end of the list.
    /// @return the head of the world contact list.
    /// @warning contacts are created and destroyed in the middle of a time step.
    /// Use b2ContactListener to avoid missing contacts.
    b2World.prototype.GetContactList = function () {
        return this.m_contactManager.m_contactList;
    };
    /// Enable/disable sleep.
    b2World.prototype.SetAllowSleeping = function (flag) {
        if (flag === this.m_allowSleep) {
            return;
        }
        this.m_allowSleep = flag;
        if (!this.m_allowSleep) {
            for (var b = this.m_bodyList; b; b = b.m_next) {
                b.SetAwake(true);
            }
        }
    };
    b2World.prototype.GetAllowSleeping = function () {
        return this.m_allowSleep;
    };
    /// Enable/disable warm starting. For testing.
    b2World.prototype.SetWarmStarting = function (flag) {
        this.m_warmStarting = flag;
    };
    b2World.prototype.GetWarmStarting = function () {
        return this.m_warmStarting;
    };
    /// Enable/disable continuous physics. For testing.
    b2World.prototype.SetContinuousPhysics = function (flag) {
        this.m_continuousPhysics = flag;
    };
    b2World.prototype.GetContinuousPhysics = function () {
        return this.m_continuousPhysics;
    };
    /// Enable/disable single stepped continuous physics. For testing.
    b2World.prototype.SetSubStepping = function (flag) {
        this.m_subStepping = flag;
    };
    b2World.prototype.GetSubStepping = function () {
        return this.m_subStepping;
    };
    /// Get the number of broad-phase proxies.
    b2World.prototype.GetProxyCount = function () {
        return this.m_contactManager.m_broadPhase.GetProxyCount();
    };
    /// Get the number of bodies.
    b2World.prototype.GetBodyCount = function () {
        return this.m_bodyCount;
    };
    /// Get the number of joints.
    b2World.prototype.GetJointCount = function () {
        return this.m_jointCount;
    };
    /// Get the number of contacts (each may have 0 or more contact points).
    b2World.prototype.GetContactCount = function () {
        return this.m_contactManager.m_contactCount;
    };
    /// Get the height of the dynamic tree.
    b2World.prototype.GetTreeHeight = function () {
        return this.m_contactManager.m_broadPhase.GetTreeHeight();
    };
    /// Get the balance of the dynamic tree.
    b2World.prototype.GetTreeBalance = function () {
        return this.m_contactManager.m_broadPhase.GetTreeBalance();
    };
    /// Get the quality metric of the dynamic tree. The smaller the better.
    /// The minimum is 1.
    b2World.prototype.GetTreeQuality = function () {
        return this.m_contactManager.m_broadPhase.GetTreeQuality();
    };
    /// Change the global gravity vector.
    b2World.prototype.SetGravity = function (gravity, wake) {
        if (wake === void 0) { wake = true; }
        if (!b2Math_1.b2Vec2.IsEqualToV(this.m_gravity, gravity)) {
            this.m_gravity.Copy(gravity);
            if (wake) {
                for (var b = this.m_bodyList; b; b = b.m_next) {
                    b.SetAwake(true);
                }
            }
        }
    };
    /// Get the global gravity vector.
    b2World.prototype.GetGravity = function () {
        return this.m_gravity;
    };
    /// Is the world locked (in the middle of a time step).
    b2World.prototype.IsLocked = function () {
        return this.m_locked;
    };
    /// Set flag to control automatic clearing of forces after each time step.
    b2World.prototype.SetAutoClearForces = function (flag) {
        this.m_clearForces = flag;
    };
    /// Get the flag that controls automatic clearing of forces after each time step.
    b2World.prototype.GetAutoClearForces = function () {
        return this.m_clearForces;
    };
    /// Shift the world origin. Useful for large worlds.
    /// The body shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    b2World.prototype.ShiftOrigin = function (newOrigin) {
        if (this.IsLocked()) {
            throw new Error();
        }
        for (var b = this.m_bodyList; b; b = b.m_next) {
            b.m_xf.p.SelfSub(newOrigin);
            b.m_sweep.c0.SelfSub(newOrigin);
            b.m_sweep.c.SelfSub(newOrigin);
        }
        for (var j = this.m_jointList; j; j = j.m_next) {
            j.ShiftOrigin(newOrigin);
        }
        this.m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
    };
    /// Get the contact manager for testing.
    b2World.prototype.GetContactManager = function () {
        return this.m_contactManager;
    };
    /// Get the current profile.
    b2World.prototype.GetProfile = function () {
        return this.m_profile;
    };
    /// Dump the world into the log file.
    /// @warning this should be called outside of a time step.
    b2World.prototype.Dump = function (log) {
        if (this.m_locked) {
            return;
        }
        log("const g: b2Vec2 = new b2Vec2(%.15f, %.15f);\n", this.m_gravity.x, this.m_gravity.y);
        log("this.m_world.SetGravity(g);\n");
        log("const bodies: b2Body[] = [];\n");
        log("const joints: b2Joint[] = [];\n");
        var i = 0;
        for (var b = this.m_bodyList; b; b = b.m_next) {
            b.m_islandIndex = i;
            b.Dump(log);
            ++i;
        }
        i = 0;
        for (var j = this.m_jointList; j; j = j.m_next) {
            j.m_index = i;
            ++i;
        }
        // First pass on joints, skip gear joints.
        for (var j = this.m_jointList; j; j = j.m_next) {
            if (j.m_type === b2Joint_1.b2JointType.e_gearJoint) {
                continue;
            }
            log("{\n");
            j.Dump(log);
            log("}\n");
        }
        // Second pass on joints, only gear joints.
        for (var j = this.m_jointList; j; j = j.m_next) {
            if (j.m_type !== b2Joint_1.b2JointType.e_gearJoint) {
                continue;
            }
            log("{\n");
            j.Dump(log);
            log("}\n");
        }
    };
    b2World.prototype.DrawJoint = function (joint) {
        if (this.m_debugDraw === null) {
            return;
        }
        var bodyA = joint.GetBodyA();
        var bodyB = joint.GetBodyB();
        var xf1 = bodyA.m_xf;
        var xf2 = bodyB.m_xf;
        var x1 = xf1.p;
        var x2 = xf2.p;
        var p1 = joint.GetAnchorA(b2World.DrawJoint_s_p1);
        var p2 = joint.GetAnchorB(b2World.DrawJoint_s_p2);
        var color = b2World.DrawJoint_s_color.SetRGB(0.5, 0.8, 0.8);
        switch (joint.m_type) {
            case b2Joint_1.b2JointType.e_distanceJoint:
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
            case b2Joint_1.b2JointType.e_pulleyJoint:
                {
                    var pulley = joint;
                    var s1 = pulley.GetGroundAnchorA();
                    var s2 = pulley.GetGroundAnchorB();
                    this.m_debugDraw.DrawSegment(s1, p1, color);
                    this.m_debugDraw.DrawSegment(s2, p2, color);
                    this.m_debugDraw.DrawSegment(s1, s2, color);
                }
                break;
            case b2Joint_1.b2JointType.e_mouseJoint:
                // don't draw this
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
            default:
                this.m_debugDraw.DrawSegment(x1, p1, color);
                this.m_debugDraw.DrawSegment(p1, p2, color);
                this.m_debugDraw.DrawSegment(x2, p2, color);
        }
    };
    b2World.prototype.DrawShape = function (fixture, color) {
        if (this.m_debugDraw === null) {
            return;
        }
        var shape = fixture.GetShape();
        switch (shape.m_type) {
            case b2Shape_1.b2ShapeType.e_circleShape:
                {
                    var circle = shape;
                    var center = circle.m_p;
                    var radius = circle.m_radius;
                    var axis = b2Math_1.b2Vec2.UNITX;
                    this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
                }
                break;
            case b2Shape_1.b2ShapeType.e_edgeShape:
                {
                    var edge = shape;
                    var v1 = edge.m_vertex1;
                    var v2 = edge.m_vertex2;
                    this.m_debugDraw.DrawSegment(v1, v2, color);
                }
                break;
            case b2Shape_1.b2ShapeType.e_chainShape:
                {
                    var chain = shape;
                    var count = chain.m_count;
                    var vertices = chain.m_vertices;
                    var v1 = vertices[0];
                    this.m_debugDraw.DrawCircle(v1, 0.05, color);
                    for (var i = 1; i < count; ++i) {
                        var v2 = vertices[i];
                        this.m_debugDraw.DrawSegment(v1, v2, color);
                        this.m_debugDraw.DrawCircle(v2, 0.05, color);
                        v1 = v2;
                    }
                }
                break;
            case b2Shape_1.b2ShapeType.e_polygonShape:
                {
                    var poly = shape;
                    var vertexCount = poly.m_count;
                    var vertices = poly.m_vertices;
                    this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
                }
                break;
        }
    };
    b2World.prototype.Solve = function (step) {
        // #if B2_ENABLE_PARTICLE
        // update previous transforms
        for (var b = this.m_bodyList; b; b = b.m_next) {
            b.m_xf0.Copy(b.m_xf);
        }
        // #endif
        // #if B2_ENABLE_CONTROLLER
        // @see b2Controller list
        for (var controller = this.m_controllerList; controller; controller = controller.m_next) {
            controller.Step(step);
        }
        // #endif
        this.m_profile.solveInit = 0;
        this.m_profile.solveVelocity = 0;
        this.m_profile.solvePosition = 0;
        // Size the island for the worst case.
        var island = this.m_island;
        island.Initialize(this.m_bodyCount, this.m_contactManager.m_contactCount, this.m_jointCount, null, // this.m_stackAllocator,
        this.m_contactManager.m_contactListener);
        // Clear all the island flags.
        for (var b = this.m_bodyList; b; b = b.m_next) {
            b.m_islandFlag = false;
        }
        for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
            c.m_islandFlag = false;
        }
        for (var j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }
        // Build and simulate all awake islands.
        // DEBUG: const stackSize: number = this.m_bodyCount;
        var stack = this.s_stack;
        for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
            if (seed.m_islandFlag) {
                continue;
            }
            if (!seed.IsAwake() || !seed.IsActive()) {
                continue;
            }
            // The seed can be dynamic or kinematic.
            if (seed.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                continue;
            }
            // Reset island and stack.
            island.Clear();
            var stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_islandFlag = true;
            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                var b = stack[--stackCount];
                if (!b) {
                    throw new Error();
                }
                // DEBUG: b2Assert(b.IsActive());
                island.AddBody(b);
                // Make sure the body is awake.
                b.SetAwake(true);
                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                    continue;
                }
                // Search all contacts connected to this body.
                for (var ce = b.m_contactList; ce; ce = ce.next) {
                    var contact = ce.contact;
                    // Has this contact already been added to an island?
                    if (contact.m_islandFlag) {
                        continue;
                    }
                    // Is this contact solid and touching?
                    if (!contact.IsEnabled() || !contact.IsTouching()) {
                        continue;
                    }
                    // Skip sensors.
                    var sensorA = contact.m_fixtureA.m_isSensor;
                    var sensorB = contact.m_fixtureB.m_isSensor;
                    if (sensorA || sensorB) {
                        continue;
                    }
                    island.AddContact(contact);
                    contact.m_islandFlag = true;
                    var other = ce.other;
                    if (!other) {
                        throw new Error();
                    }
                    // Was the other body already added to this island?
                    if (other.m_islandFlag) {
                        continue;
                    }
                    // DEBUG: b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
                // Search all joints connect to this body.
                for (var je = b.m_jointList; je; je = je.next) {
                    if (je.joint.m_islandFlag) {
                        continue;
                    }
                    var other = je.other;
                    // Don't simulate joints connected to inactive bodies.
                    if (!other.IsActive()) {
                        continue;
                    }
                    island.AddJoint(je.joint);
                    je.joint.m_islandFlag = true;
                    if (other.m_islandFlag) {
                        continue;
                    }
                    // DEBUG: b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_islandFlag = true;
                }
            }
            var profile = new b2TimeStep_1.b2Profile();
            island.Solve(profile, step, this.m_gravity, this.m_allowSleep);
            this.m_profile.solveInit += profile.solveInit;
            this.m_profile.solveVelocity += profile.solveVelocity;
            this.m_profile.solvePosition += profile.solvePosition;
            // Post solve cleanup.
            for (var i = 0; i < island.m_bodyCount; ++i) {
                // Allow static bodies to participate in other islands.
                var b = island.m_bodies[i];
                if (b.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                    b.m_islandFlag = false;
                }
            }
        }
        for (var i = 0; i < stack.length; ++i) {
            if (!stack[i]) {
                break;
            }
            stack[i] = null;
        }
        var timer = new b2Timer_1.b2Timer();
        // Synchronize fixtures, check for out of range bodies.
        for (var b = this.m_bodyList; b; b = b.m_next) {
            // If a body was not in an island then it did not move.
            if (!b.m_islandFlag) {
                continue;
            }
            if (b.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                continue;
            }
            // Update fixtures (for broad-phase).
            b.SynchronizeFixtures();
        }
        // Look for new contacts.
        this.m_contactManager.FindNewContacts();
        this.m_profile.broadphase = timer.GetMilliseconds();
    };
    b2World.prototype.SolveTOI = function (step) {
        // b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);
        var island = this.m_island;
        island.Initialize(2 * b2Settings_1.b2_maxTOIContacts, b2Settings_1.b2_maxTOIContacts, 0, null, this.m_contactManager.m_contactListener);
        if (this.m_stepComplete) {
            for (var b = this.m_bodyList; b; b = b.m_next) {
                b.m_islandFlag = false;
                b.m_sweep.alpha0 = 0;
            }
            for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Invalidate TOI
                c.m_toiFlag = false;
                c.m_islandFlag = false;
                c.m_toiCount = 0;
                c.m_toi = 1;
            }
        }
        // Find TOI events and solve them.
        for (;;) {
            // Find the first TOI.
            var minContact = null;
            var minAlpha = 1;
            for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                // Is this contact disabled?
                if (!c.IsEnabled()) {
                    continue;
                }
                // Prevent excessive sub-stepping.
                if (c.m_toiCount > b2Settings_1.b2_maxSubSteps) {
                    continue;
                }
                var alpha = 1;
                if (c.m_toiFlag) {
                    // This contact has a valid cached TOI.
                    alpha = c.m_toi;
                }
                else {
                    var fA_1 = c.GetFixtureA();
                    var fB_1 = c.GetFixtureB();
                    // Is there a sensor?
                    if (fA_1.IsSensor() || fB_1.IsSensor()) {
                        continue;
                    }
                    var bA_1 = fA_1.GetBody();
                    var bB_1 = fB_1.GetBody();
                    var typeA = bA_1.m_type;
                    var typeB = bB_1.m_type;
                    // DEBUG: b2Assert(typeA !== b2BodyType.b2_staticBody || typeB !== b2BodyType.b2_staticBody);
                    var activeA = bA_1.IsAwake() && typeA !== b2Body_1.b2BodyType.b2_staticBody;
                    var activeB = bB_1.IsAwake() && typeB !== b2Body_1.b2BodyType.b2_staticBody;
                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (!activeA && !activeB) {
                        continue;
                    }
                    var collideA = bA_1.IsBullet() || typeA !== b2Body_1.b2BodyType.b2_dynamicBody;
                    var collideB = bB_1.IsBullet() || typeB !== b2Body_1.b2BodyType.b2_dynamicBody;
                    // Are these two non-bullet dynamic bodies?
                    if (!collideA && !collideB) {
                        continue;
                    }
                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    var alpha0 = bA_1.m_sweep.alpha0;
                    if (bA_1.m_sweep.alpha0 < bB_1.m_sweep.alpha0) {
                        alpha0 = bB_1.m_sweep.alpha0;
                        bA_1.m_sweep.Advance(alpha0);
                    }
                    else if (bB_1.m_sweep.alpha0 < bA_1.m_sweep.alpha0) {
                        alpha0 = bA_1.m_sweep.alpha0;
                        bB_1.m_sweep.Advance(alpha0);
                    }
                    // DEBUG: b2Assert(alpha0 < 1);
                    var indexA = c.GetChildIndexA();
                    var indexB = c.GetChildIndexB();
                    // Compute the time of impact in interval [0, minTOI]
                    var input = b2World.SolveTOI_s_toi_input;
                    input.proxyA.SetShape(fA_1.GetShape(), indexA);
                    input.proxyB.SetShape(fB_1.GetShape(), indexB);
                    input.sweepA.Copy(bA_1.m_sweep);
                    input.sweepB.Copy(bB_1.m_sweep);
                    input.tMax = 1;
                    var output = b2World.SolveTOI_s_toi_output;
                    b2TimeOfImpact_1.b2TimeOfImpact(output, input);
                    // Beta is the fraction of the remaining portion of the .
                    var beta = output.t;
                    if (output.state === b2TimeOfImpact_1.b2TOIOutputState.e_touching) {
                        alpha = b2Math_1.b2Min(alpha0 + (1 - alpha0) * beta, 1);
                    }
                    else {
                        alpha = 1;
                    }
                    c.m_toi = alpha;
                    c.m_toiFlag = true;
                }
                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha = alpha;
                }
            }
            if (minContact === null || 1 - 10 * b2Settings_1.b2_epsilon < minAlpha) {
                // No more TOI events. Done!
                this.m_stepComplete = true;
                break;
            }
            // Advance the bodies to the TOI.
            var fA = minContact.GetFixtureA();
            var fB = minContact.GetFixtureB();
            var bA = fA.GetBody();
            var bB = fB.GetBody();
            var backup1 = b2World.SolveTOI_s_backup1.Copy(bA.m_sweep);
            var backup2 = b2World.SolveTOI_s_backup2.Copy(bB.m_sweep);
            bA.Advance(minAlpha);
            bB.Advance(minAlpha);
            // The TOI contact likely has some new contact points.
            minContact.Update(this.m_contactManager.m_contactListener);
            minContact.m_toiFlag = false;
            ++minContact.m_toiCount;
            // Is the contact solid?
            if (!minContact.IsEnabled() || !minContact.IsTouching()) {
                // Restore the sweeps.
                minContact.SetEnabled(false);
                bA.m_sweep.Copy(backup1);
                bB.m_sweep.Copy(backup2);
                bA.SynchronizeTransform();
                bB.SynchronizeTransform();
                continue;
            }
            bA.SetAwake(true);
            bB.SetAwake(true);
            // Build the island
            island.Clear();
            island.AddBody(bA);
            island.AddBody(bB);
            island.AddContact(minContact);
            bA.m_islandFlag = true;
            bB.m_islandFlag = true;
            minContact.m_islandFlag = true;
            // Get contacts on bodyA and bodyB.
            // const bodies: b2Body[] = [bA, bB];
            for (var i = 0; i < 2; ++i) {
                var body = (i === 0) ? (bA) : (bB); // bodies[i];
                if (body.m_type === b2Body_1.b2BodyType.b2_dynamicBody) {
                    for (var ce = body.m_contactList; ce; ce = ce.next) {
                        if (island.m_bodyCount === island.m_bodyCapacity) {
                            break;
                        }
                        if (island.m_contactCount === island.m_contactCapacity) {
                            break;
                        }
                        var contact = ce.contact;
                        // Has this contact already been added to the island?
                        if (contact.m_islandFlag) {
                            continue;
                        }
                        // Only add static, kinematic, or bullet bodies.
                        var other = ce.other;
                        if (other.m_type === b2Body_1.b2BodyType.b2_dynamicBody &&
                            !body.IsBullet() && !other.IsBullet()) {
                            continue;
                        }
                        // Skip sensors.
                        var sensorA = contact.m_fixtureA.m_isSensor;
                        var sensorB = contact.m_fixtureB.m_isSensor;
                        if (sensorA || sensorB) {
                            continue;
                        }
                        // Tentatively advance the body to the TOI.
                        var backup = b2World.SolveTOI_s_backup.Copy(other.m_sweep);
                        if (!other.m_islandFlag) {
                            other.Advance(minAlpha);
                        }
                        // Update the contact points
                        contact.Update(this.m_contactManager.m_contactListener);
                        // Was the contact disabled by the user?
                        if (!contact.IsEnabled()) {
                            other.m_sweep.Copy(backup);
                            other.SynchronizeTransform();
                            continue;
                        }
                        // Are there contact points?
                        if (!contact.IsTouching()) {
                            other.m_sweep.Copy(backup);
                            other.SynchronizeTransform();
                            continue;
                        }
                        // Add the contact to the island
                        contact.m_islandFlag = true;
                        island.AddContact(contact);
                        // Has the other body already been added to the island?
                        if (other.m_islandFlag) {
                            continue;
                        }
                        // Add the other body to the island.
                        other.m_islandFlag = true;
                        if (other.m_type !== b2Body_1.b2BodyType.b2_staticBody) {
                            other.SetAwake(true);
                        }
                        island.AddBody(other);
                    }
                }
            }
            var subStep = b2World.SolveTOI_s_subStep;
            subStep.dt = (1 - minAlpha) * step.dt;
            subStep.inv_dt = 1 / subStep.dt;
            subStep.dtRatio = 1;
            subStep.positionIterations = 20;
            subStep.velocityIterations = step.velocityIterations;
            // #if B2_ENABLE_PARTICLE
            subStep.particleIterations = step.particleIterations;
            // #endif
            subStep.warmStarting = false;
            island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);
            // Reset island flags and synchronize broad-phase proxies.
            for (var i = 0; i < island.m_bodyCount; ++i) {
                var body = island.m_bodies[i];
                body.m_islandFlag = false;
                if (body.m_type !== b2Body_1.b2BodyType.b2_dynamicBody) {
                    continue;
                }
                body.SynchronizeFixtures();
                // Invalidate all contact TOIs on this displaced body.
                for (var ce = body.m_contactList; ce; ce = ce.next) {
                    ce.contact.m_toiFlag = false;
                    ce.contact.m_islandFlag = false;
                }
            }
            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            this.m_contactManager.FindNewContacts();
            if (this.m_subStepping) {
                this.m_stepComplete = false;
                break;
            }
        }
    };
    // #if B2_ENABLE_CONTROLLER
    b2World.prototype.AddController = function (controller) {
        // b2Assert(controller.m_world === null, "Controller can only be a member of one world");
        // controller.m_world = this;
        controller.m_next = this.m_controllerList;
        controller.m_prev = null;
        if (this.m_controllerList) {
            this.m_controllerList.m_prev = controller;
        }
        this.m_controllerList = controller;
        ++this.m_controllerCount;
        return controller;
    };
    b2World.prototype.RemoveController = function (controller) {
        // b2Assert(controller.m_world === this, "Controller is not a member of this world");
        if (controller.m_prev) {
            controller.m_prev.m_next = controller.m_next;
        }
        if (controller.m_next) {
            controller.m_next.m_prev = controller.m_prev;
        }
        if (this.m_controllerList === controller) {
            this.m_controllerList = controller.m_next;
        }
        --this.m_controllerCount;
        controller.m_prev = null;
        controller.m_next = null;
        // delete controller.m_world; // = null;
        return controller;
    };
    // #endif
    /// Take a time step. This performs collision detection, integration,
    /// and constraint solution.
    /// @param timeStep the amount of time to simulate, this should not vary.
    /// @param velocityIterations for the velocity constraint solver.
    /// @param positionIterations for the position constraint solver.
    b2World.Step_s_step = new b2TimeStep_1.b2TimeStep();
    b2World.Step_s_stepTimer = new b2Timer_1.b2Timer();
    b2World.Step_s_timer = new b2Timer_1.b2Timer();
    // #endif
    /// Call this to draw shapes and other debug draw data.
    b2World.DrawDebugData_s_color = new b2Draw_1.b2Color(0, 0, 0);
    b2World.DrawDebugData_s_vs = b2Math_1.b2Vec2.MakeArray(4);
    b2World.DrawDebugData_s_xf = new b2Math_1.b2Transform();
    b2World.QueryFixtureShape_s_aabb = new b2Collision_1.b2AABB();
    /// Ray-cast the world for all fixtures in the path of the ray. Your callback
    /// controls whether you get the closest point, any point, or n-points.
    /// The ray-cast ignores shapes that contain the starting point.
    /// @param callback a user implemented callback class.
    /// @param point1 the ray starting point
    /// @param point2 the ray ending point
    b2World.RayCast_s_input = new b2Collision_1.b2RayCastInput();
    b2World.RayCast_s_output = new b2Collision_1.b2RayCastOutput();
    b2World.RayCast_s_point = new b2Math_1.b2Vec2();
    b2World.DrawJoint_s_p1 = new b2Math_1.b2Vec2();
    b2World.DrawJoint_s_p2 = new b2Math_1.b2Vec2();
    b2World.DrawJoint_s_color = new b2Draw_1.b2Color(0.5, 0.8, 0.8);
    b2World.SolveTOI_s_subStep = new b2TimeStep_1.b2TimeStep();
    b2World.SolveTOI_s_backup = new b2Math_1.b2Sweep();
    b2World.SolveTOI_s_backup1 = new b2Math_1.b2Sweep();
    b2World.SolveTOI_s_backup2 = new b2Math_1.b2Sweep();
    b2World.SolveTOI_s_toi_input = new b2TimeOfImpact_1.b2TOIInput();
    b2World.SolveTOI_s_toi_output = new b2TimeOfImpact_1.b2TOIOutput();
    return b2World;
}());
exports.b2World = b2World;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJXb3JsZC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL2IyV29ybGQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLDBEQUEwRDtBQUMxRCxtREFBcUY7QUFDckYsMkNBQTJFO0FBQzNFLDZDQUE0QztBQUM1QywyQ0FBZ0U7QUFFaEUsd0RBQXVHO0FBRXZHLDhEQUF3RztBQUN4Ryx1REFBbUU7QUFNbkUsNENBQWtGO0FBQ2xGLDBEQUF5RDtBQUV6RCxtQ0FBMEQ7QUFDMUQsdURBQXNEO0FBRXRELHVDQUFzQztBQUN0QywyQ0FBcUQ7QUFJckQsdURBQThFO0FBRTlFLHlCQUF5QjtBQUN6QixtREFBbUQ7QUFDbkQscURBQXVFO0FBQ3ZFLGlFQUFxRjtBQUlyRixTQUFTO0FBRVQscUVBQXFFO0FBQ3JFLHNFQUFzRTtBQUN0RSwwQkFBMEI7QUFDMUI7SUE4Q0UsU0FBUztJQUVULDZCQUE2QjtJQUM3Qiw0Q0FBNEM7SUFDNUMsaUJBQVksT0FBVztRQWpEdkIscUNBQXFDO1FBQ3JDLHFDQUFxQztRQUU5QixpQkFBWSxHQUFZLEtBQUssQ0FBQztRQUM5QixhQUFRLEdBQVksS0FBSyxDQUFDO1FBQzFCLGtCQUFhLEdBQVksSUFBSSxDQUFDO1FBRXJCLHFCQUFnQixHQUFxQixJQUFJLG1DQUFnQixFQUFFLENBQUM7UUFFckUsZUFBVSxHQUFrQixJQUFJLENBQUM7UUFDakMsZ0JBQVcsR0FBbUIsSUFBSSxDQUFDO1FBRTFDLHlCQUF5QjtRQUNsQix5QkFBb0IsR0FBNEIsSUFBSSxDQUFDO1FBQzVELFNBQVM7UUFFRixnQkFBVyxHQUFXLENBQUMsQ0FBQztRQUN4QixpQkFBWSxHQUFXLENBQUMsQ0FBQztRQUVoQixjQUFTLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUMxQyxpQkFBWSxHQUFZLElBQUksQ0FBQztRQUU3QiwwQkFBcUIsR0FBaUMs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