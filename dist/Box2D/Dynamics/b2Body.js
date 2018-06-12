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
// DEBUG: import { b2IsValid } from "../Common/b2Math";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Shape_1 = require("../Collision/Shapes/b2Shape");
var b2Fixture_1 = require("./b2Fixture");
// #endif
/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
var b2BodyType;
(function (b2BodyType) {
    b2BodyType[b2BodyType["b2_unknown"] = -1] = "b2_unknown";
    b2BodyType[b2BodyType["b2_staticBody"] = 0] = "b2_staticBody";
    b2BodyType[b2BodyType["b2_kinematicBody"] = 1] = "b2_kinematicBody";
    b2BodyType[b2BodyType["b2_dynamicBody"] = 2] = "b2_dynamicBody";
    // TODO_ERIN
    // b2_bulletBody = 3
})(b2BodyType = exports.b2BodyType || (exports.b2BodyType = {}));
/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
var b2BodyDef = /** @class */ (function () {
    function b2BodyDef() {
        /// The body type: static, kinematic, or dynamic.
        /// Note: if a dynamic body would have zero mass, the mass is set to one.
        this.type = b2BodyType.b2_staticBody;
        /// The world position of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        this.position = new b2Math_1.b2Vec2(0, 0);
        /// The world angle of the body in radians.
        this.angle = 0;
        /// The linear velocity of the body's origin in world co-ordinates.
        this.linearVelocity = new b2Math_1.b2Vec2(0, 0);
        /// The angular velocity of the body.
        this.angularVelocity = 0;
        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        this.linearDamping = 0;
        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        this.angularDamping = 0;
        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        this.allowSleep = true;
        /// Is this body initially awake or sleeping?
        this.awake = true;
        /// Should this body be prevented from rotating? Useful for characters.
        this.fixedRotation = false;
        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// kinematic and static bodies. This setting is only considered on dynamic bodies.
        /// @warning You should use this flag sparingly since it increases processing time.
        this.bullet = false;
        /// Does this body start out active?
        this.active = true;
        /// Use this to store application specific body data.
        this.userData = null;
        /// Scale the gravity applied to this body.
        this.gravityScale = 1;
    }
    return b2BodyDef;
}());
exports.b2BodyDef = b2BodyDef;
/// A rigid body. These are created via b2World::CreateBody.
var b2Body = /** @class */ (function () {
    // #endif
    function b2Body(bd, world) {
        this.m_type = b2BodyType.b2_staticBody;
        this.m_islandFlag = false;
        this.m_awakeFlag = false;
        this.m_autoSleepFlag = false;
        this.m_bulletFlag = false;
        this.m_fixedRotationFlag = false;
        this.m_activeFlag = false;
        this.m_toiFlag = false;
        this.m_islandIndex = 0;
        this.m_xf = new b2Math_1.b2Transform(); // the body origin transform
        // #if B2_ENABLE_PARTICLE
        this.m_xf0 = new b2Math_1.b2Transform();
        // #endif
        this.m_sweep = new b2Math_1.b2Sweep(); // the swept motion for CCD
        this.m_linearVelocity = new b2Math_1.b2Vec2();
        this.m_angularVelocity = 0;
        this.m_force = new b2Math_1.b2Vec2();
        this.m_torque = 0;
        this.m_prev = null;
        this.m_next = null;
        this.m_fixtureList = null;
        this.m_fixtureCount = 0;
        this.m_jointList = null;
        this.m_contactList = null;
        this.m_mass = 1;
        this.m_invMass = 1;
        // Rotational inertia about the center of mass.
        this.m_I = 0;
        this.m_invI = 0;
        this.m_linearDamping = 0;
        this.m_angularDamping = 0;
        this.m_gravityScale = 1;
        this.m_sleepTime = 0;
        this.m_userData = null;
        // #if B2_ENABLE_CONTROLLER
        this.m_controllerList = null;
        this.m_controllerCount = 0;
        this.m_bulletFlag = b2Settings_1.b2Maybe(bd.bullet, false);
        this.m_fixedRotationFlag = b2Settings_1.b2Maybe(bd.fixedRotation, false);
        this.m_autoSleepFlag = b2Settings_1.b2Maybe(bd.allowSleep, true);
        this.m_awakeFlag = b2Settings_1.b2Maybe(bd.awake, true);
        this.m_activeFlag = b2Settings_1.b2Maybe(bd.active, true);
        this.m_world = world;
        this.m_xf.p.Copy(b2Settings_1.b2Maybe(bd.position, b2Math_1.b2Vec2.ZERO));
        // DEBUG: b2Assert(this.m_xf.p.IsValid());
        this.m_xf.q.SetAngle(b2Settings_1.b2Maybe(bd.angle, 0));
        // DEBUG: b2Assert(b2IsValid(this.m_xf.q.GetAngle()));
        // #if B2_ENABLE_PARTICLE
        this.m_xf0.Copy(this.m_xf);
        // #endif
        this.m_sweep.localCenter.SetZero();
        this.m_sweep.c0.Copy(this.m_xf.p);
        this.m_sweep.c.Copy(this.m_xf.p);
        this.m_sweep.a0 = this.m_sweep.a = this.m_xf.q.GetAngle();
        this.m_sweep.alpha0 = 0;
        this.m_linearVelocity.Copy(b2Settings_1.b2Maybe(bd.linearVelocity, b2Math_1.b2Vec2.ZERO));
        // DEBUG: b2Assert(this.m_linearVelocity.IsValid());
        this.m_angularVelocity = b2Settings_1.b2Maybe(bd.angularVelocity, 0);
        // DEBUG: b2Assert(b2IsValid(this.m_angularVelocity));
        this.m_linearDamping = b2Settings_1.b2Maybe(bd.linearDamping, 0);
        this.m_angularDamping = b2Settings_1.b2Maybe(bd.angularDamping, 0);
        this.m_gravityScale = b2Settings_1.b2Maybe(bd.gravityScale, 1);
        // DEBUG: b2Assert(b2IsValid(this.m_gravityScale) && this.m_gravityScale >= 0);
        // DEBUG: b2Assert(b2IsValid(this.m_angularDamping) && this.m_angularDamping >= 0);
        // DEBUG: b2Assert(b2IsValid(this.m_linearDamping) && this.m_linearDamping >= 0);
        this.m_force.SetZero();
        this.m_torque = 0;
        this.m_sleepTime = 0;
        this.m_type = b2Settings_1.b2Maybe(bd.type, b2BodyType.b2_staticBody);
        if (bd.type === b2BodyType.b2_dynamicBody) {
            this.m_mass = 1;
            this.m_invMass = 1;
        }
        else {
            this.m_mass = 0;
            this.m_invMass = 0;
        }
        this.m_I = 0;
        this.m_invI = 0;
        this.m_userData = bd.userData;
        this.m_fixtureList = null;
        this.m_fixtureCount = 0;
        // #if B2_ENABLE_CONTROLLER
        this.m_controllerList = null;
        this.m_controllerCount = 0;
        // #endif
    }
    b2Body.prototype.CreateFixture = function (a, b) {
        if (b === void 0) { b = 0; }
        if (a instanceof b2Shape_1.b2Shape) {
            return this.CreateFixtureShapeDensity(a, b);
        }
        else {
            return this.CreateFixtureDef(a);
        }
    };
    /// Creates a fixture and attach it to this body. Use this function if you need
    /// to set some fixture parameters, like friction. Otherwise you can create the
    /// fixture directly from a shape.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// Contacts are not created until the next time step.
    /// @param def the fixture definition.
    /// @warning This function is locked during callbacks.
    b2Body.prototype.CreateFixtureDef = function (def) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        var fixture = new b2Fixture_1.b2Fixture(def, this);
        fixture.Create(def);
        if (this.m_activeFlag) {
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.CreateProxies(broadPhase, this.m_xf);
        }
        fixture.m_next = this.m_fixtureList;
        this.m_fixtureList = fixture;
        ++this.m_fixtureCount;
        // fixture.m_body = this;
        // Adjust mass properties if needed.
        if (fixture.m_density > 0) {
            this.ResetMassData();
        }
        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        this.m_world.m_newFixture = true;
        return fixture;
    };
    b2Body.prototype.CreateFixtureShapeDensity = function (shape, density) {
        if (density === void 0) { density = 0; }
        var def = b2Body.CreateFixtureShapeDensity_s_def;
        def.shape = shape;
        def.density = density;
        return this.CreateFixtureDef(def);
    };
    /// Destroy a fixture. This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture. This will
    /// automatically adjust the mass of the body if the body is dynamic and the
    /// fixture has positive density.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    /// @param fixture the fixture to be removed.
    /// @warning This function is locked during callbacks.
    b2Body.prototype.DestroyFixture = function (fixture) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        // DEBUG: b2Assert(fixture.m_body === this);
        // Remove the fixture from this body's singly linked list.
        // DEBUG: b2Assert(this.m_fixtureCount > 0);
        var node = this.m_fixtureList;
        var ppF = null;
        // DEBUG: let found: boolean = false;
        while (node !== null) {
            if (node === fixture) {
                if (ppF) {
                    ppF.m_next = fixture.m_next;
                }
                else {
                    this.m_fixtureList = fixture.m_next;
                }
                // DEBUG: found = true;
                break;
            }
            ppF = node;
            node = node.m_next;
        }
        // You tried to remove a shape that is not attached to this body.
        // DEBUG: b2Assert(found);
        // Destroy any contacts associated with the fixture.
        var edge = this.m_contactList;
        while (edge) {
            var c = edge.contact;
            edge = edge.next;
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                this.m_world.m_contactManager.Destroy(c);
            }
        }
        if (this.m_activeFlag) {
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.DestroyProxies(broadPhase);
        }
        fixture.Destroy();
        // fixture.m_body = null;
        fixture.m_next = null;
        --this.m_fixtureCount;
        // Reset the mass data.
        this.ResetMassData();
    };
    /// Set the position of the body's origin and rotation.
    /// This breaks any contacts and wakes the other bodies.
    /// Manipulating a body's transform may cause non-physical behavior.
    /// @param position the world position of the body's local origin.
    /// @param angle the world rotation in radians.
    b2Body.prototype.SetTransformVec = function (position, angle) {
        this.SetTransformXY(position.x, position.y, angle);
    };
    b2Body.prototype.SetTransformXY = function (x, y, angle) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        this.m_xf.q.SetAngle(angle);
        this.m_xf.p.Set(x, y);
        // #if B2_ENABLE_PARTICLE
        this.m_xf0.Copy(this.m_xf);
        // #endif
        b2Math_1.b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.a = angle;
        this.m_sweep.c0.Copy(this.m_sweep.c);
        this.m_sweep.a0 = angle;
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            f.Synchronize(broadPhase, this.m_xf, this.m_xf);
        }
        this.m_world.m_contactManager.FindNewContacts();
    };
    b2Body.prototype.SetTransform = function (xf) {
        this.SetTransformVec(xf.p, xf.GetAngle());
    };
    /// Get the body transform for the body's origin.
    /// @return the world transform of the body's origin.
    b2Body.prototype.GetTransform = function () {
        return this.m_xf;
    };
    /// Get the world body origin position.
    /// @return the world position of the body's origin.
    b2Body.prototype.GetPosition = function () {
        return this.m_xf.p;
    };
    b2Body.prototype.SetPosition = function (position) {
        this.SetTransformVec(position, this.GetAngle());
    };
    b2Body.prototype.SetPositionXY = function (x, y) {
        this.SetTransformXY(x, y, this.GetAngle());
    };
    /// Get the angle in radians.
    /// @return the current world rotation angle in radians.
    b2Body.prototype.GetAngle = function () {
        return this.m_sweep.a;
    };
    b2Body.prototype.SetAngle = function (angle) {
        this.SetTransformVec(this.GetPosition(), angle);
    };
    /// Get the world position of the center of mass.
    b2Body.prototype.GetWorldCenter = function () {
        return this.m_sweep.c;
    };
    /// Get the local position of the center of mass.
    b2Body.prototype.GetLocalCenter = function () {
        return this.m_sweep.localCenter;
    };
    /// Set the linear velocity of the center of mass.
    /// @param v the new linear velocity of the center of mass.
    b2Body.prototype.SetLinearVelocity = function (v) {
        if (this.m_type === b2BodyType.b2_staticBody) {
            return;
        }
        if (b2Math_1.b2Vec2.DotVV(v, v) > 0) {
            this.SetAwake(true);
        }
        this.m_linearVelocity.Copy(v);
    };
    /// Get the linear velocity of the center of mass.
    /// @return the linear velocity of the center of mass.
    b2Body.prototype.GetLinearVelocity = function () {
        return this.m_linearVelocity;
    };
    /// Set the angular velocity.
    /// @param omega the new angular velocity in radians/second.
    b2Body.prototype.SetAngularVelocity = function (w) {
        if (this.m_type === b2BodyType.b2_staticBody) {
            return;
        }
        if (w * w > 0) {
            this.SetAwake(true);
        }
        this.m_angularVelocity = w;
    };
    /// Get the angular velocity.
    /// @return the angular velocity in radians/second.
    b2Body.prototype.GetAngularVelocity = function () {
        return this.m_angularVelocity;
    };
    b2Body.prototype.GetDefinition = function (bd) {
        bd.type = this.GetType();
        bd.allowSleep = this.m_autoSleepFlag;
        bd.angle = this.GetAngle();
        bd.angularDamping = this.m_angularDamping;
        bd.gravityScale = this.m_gravityScale;
        bd.angularVelocity = this.m_angularVelocity;
        bd.fixedRotation = this.m_fixedRotationFlag;
        bd.bullet = this.m_bulletFlag;
        bd.awake = this.m_awakeFlag;
        bd.linearDamping = this.m_linearDamping;
        bd.linearVelocity.Copy(this.GetLinearVelocity());
        bd.position.Copy(this.GetPosition());
        bd.userData = this.GetUserData();
        return bd;
    };
    /// Apply a force at a world point. If the force is not
    /// applied at the center of mass, it will generate a torque and
    /// affect the angular velocity. This wakes up the body.
    /// @param force the world force vector, usually in Newtons (N).
    /// @param point the world position of the point of application.
    /// @param wake also wake up the body
    b2Body.prototype.ApplyForce = function (force, point, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
            this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
        }
    };
    /// Apply a force to the center of mass. This wakes up the body.
    /// @param force the world force vector, usually in Newtons (N).
    /// @param wake also wake up the body
    b2Body.prototype.ApplyForceToCenter = function (force, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_force.x += force.x;
            this.m_force.y += force.y;
        }
    };
    /// Apply a torque. This affects the angular velocity
    /// without affecting the linear velocity of the center of mass.
    /// This wakes up the body.
    /// @param torque about the z-axis (out of the screen), usually in N-m.
    /// @param wake also wake up the body
    b2Body.prototype.ApplyTorque = function (torque, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_torque += torque;
        }
    };
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// It also modifies the angular velocity if the point of application
    /// is not at the center of mass. This wakes up the body.
    /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    /// @param point the world position of the point of application.
    /// @param wake also wake up the body
    b2Body.prototype.ApplyLinearImpulse = function (impulse, point, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
            this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
        }
    };
    /// Apply an impulse at the center of gravity. This immediately modifies the velocity.
    /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    /// @param wake also wake up the body
    b2Body.prototype.ApplyLinearImpulseToCenter = function (impulse, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
        }
    };
    /// Apply an angular impulse.
    /// @param impulse the angular impulse in units of kg*m*m/s
    /// @param wake also wake up the body
    b2Body.prototype.ApplyAngularImpulse = function (impulse, wake) {
        if (wake === void 0) { wake = true; }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        if (wake && !this.m_awakeFlag) {
            this.SetAwake(true);
        }
        // Don't accumulate a force if the body is sleeping.
        if (this.m_awakeFlag) {
            this.m_angularVelocity += this.m_invI * impulse;
        }
    };
    /// Get the total mass of the body.
    /// @return the mass, usually in kilograms (kg).
    b2Body.prototype.GetMass = function () {
        return this.m_mass;
    };
    /// Get the rotational inertia of the body about the local origin.
    /// @return the rotational inertia, usually in kg-m^2.
    b2Body.prototype.GetInertia = function () {
        return this.m_I + this.m_mass * b2Math_1.b2Vec2.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter);
    };
    /// Get the mass data of the body.
    /// @return a struct containing the mass, inertia and center of the body.
    b2Body.prototype.GetMassData = function (data) {
        data.mass = this.m_mass;
        data.I = this.m_I + this.m_mass * b2Math_1.b2Vec2.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter);
        data.center.Copy(this.m_sweep.localCenter);
        return data;
    };
    b2Body.prototype.SetMassData = function (massData) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_type !== b2BodyType.b2_dynamicBody) {
            return;
        }
        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;
        this.m_mass = massData.mass;
        if (this.m_mass <= 0) {
            this.m_mass = 1;
        }
        this.m_invMass = 1 / this.m_mass;
        if (massData.I > 0 && !this.m_fixedRotationFlag) {
            this.m_I = massData.I - this.m_mass * b2Math_1.b2Vec2.DotVV(massData.center, massData.center);
            // DEBUG: b2Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        }
        // Move center of mass.
        var oldCenter = b2Body.SetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(massData.center);
        b2Math_1.b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);
        // Update center of mass velocity.
        b2Math_1.b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Math_1.b2Vec2.SubVV(this.m_sweep.c, oldCenter, b2Math_1.b2Vec2.s_t0), this.m_linearVelocity);
    };
    b2Body.prototype.ResetMassData = function () {
        // Compute mass data from shapes. Each shape has its own density.
        this.m_mass = 0;
        this.m_invMass = 0;
        this.m_I = 0;
        this.m_invI = 0;
        this.m_sweep.localCenter.SetZero();
        // Static and kinematic bodies have zero mass.
        if (this.m_type === b2BodyType.b2_staticBody || this.m_type === b2BodyType.b2_kinematicBody) {
            this.m_sweep.c0.Copy(this.m_xf.p);
            this.m_sweep.c.Copy(this.m_xf.p);
            this.m_sweep.a0 = this.m_sweep.a;
            return;
        }
        // DEBUG: b2Assert(this.m_type === b2BodyType.b2_dynamicBody);
        // Accumulate mass over all fixtures.
        var localCenter = b2Body.ResetMassData_s_localCenter.SetZero();
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            if (f.m_density === 0) {
                continue;
            }
            var massData = f.GetMassData(b2Body.ResetMassData_s_massData);
            this.m_mass += massData.mass;
            localCenter.x += massData.center.x * massData.mass;
            localCenter.y += massData.center.y * massData.mass;
            this.m_I += massData.I;
        }
        // Compute center of mass.
        if (this.m_mass > 0) {
            this.m_invMass = 1 / this.m_mass;
            localCenter.x *= this.m_invMass;
            localCenter.y *= this.m_invMass;
        }
        else {
            // Force all dynamic bodies to have a positive mass.
            this.m_mass = 1;
            this.m_invMass = 1;
        }
        if (this.m_I > 0 && !this.m_fixedRotationFlag) {
            // Center the inertia about the center of mass.
            this.m_I -= this.m_mass * b2Math_1.b2Vec2.DotVV(localCenter, localCenter);
            // DEBUG: b2Assert(this.m_I > 0);
            this.m_invI = 1 / this.m_I;
        }
        else {
            this.m_I = 0;
            this.m_invI = 0;
        }
        // Move center of mass.
        var oldCenter = b2Body.ResetMassData_s_oldCenter.Copy(this.m_sweep.c);
        this.m_sweep.localCenter.Copy(localCenter);
        b2Math_1.b2Transform.MulXV(this.m_xf, this.m_sweep.localCenter, this.m_sweep.c);
        this.m_sweep.c0.Copy(this.m_sweep.c);
        // Update center of mass velocity.
        b2Math_1.b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Math_1.b2Vec2.SubVV(this.m_sweep.c, oldCenter, b2Math_1.b2Vec2.s_t0), this.m_linearVelocity);
    };
    /// Get the world coordinates of a point given the local coordinates.
    /// @param localPoint a point on the body measured relative the the body's origin.
    /// @return the same point expressed in world coordinates.
    b2Body.prototype.GetWorldPoint = function (localPoint, out) {
        return b2Math_1.b2Transform.MulXV(this.m_xf, localPoint, out);
    };
    /// Get the world coordinates of a vector given the local coordinates.
    /// @param localVector a vector fixed in the body.
    /// @return the same vector expressed in world coordinates.
    b2Body.prototype.GetWorldVector = function (localVector, out) {
        return b2Math_1.b2Rot.MulRV(this.m_xf.q, localVector, out);
    };
    /// Gets a local point relative to the body's origin given a world point.
    /// @param a point in world coordinates.
    /// @return the corresponding local point relative to the body's origin.
    b2Body.prototype.GetLocalPoint = function (worldPoint, out) {
        return b2Math_1.b2Transform.MulTXV(this.m_xf, worldPoint, out);
    };
    /// Gets a local vector given a world vector.
    /// @param a vector in world coordinates.
    /// @return the corresponding local vector.
    b2Body.prototype.GetLocalVector = function (worldVector, out) {
        return b2Math_1.b2Rot.MulTRV(this.m_xf.q, worldVector, out);
    };
    /// Get the world linear velocity of a world point attached to this body.
    /// @param a point in world coordinates.
    /// @return the world velocity of a point.
    b2Body.prototype.GetLinearVelocityFromWorldPoint = function (worldPoint, out) {
        return b2Math_1.b2Vec2.AddVCrossSV(this.m_linearVelocity, this.m_angularVelocity, b2Math_1.b2Vec2.SubVV(worldPoint, this.m_sweep.c, b2Math_1.b2Vec2.s_t0), out);
    };
    /// Get the world velocity of a local point.
    /// @param a point in local coordinates.
    /// @return the world velocity of a point.
    b2Body.prototype.GetLinearVelocityFromLocalPoint = function (localPoint, out) {
        return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint, out), out);
    };
    /// Get the linear damping of the body.
    b2Body.prototype.GetLinearDamping = function () {
        return this.m_linearDamping;
    };
    /// Set the linear damping of the body.
    b2Body.prototype.SetLinearDamping = function (linearDamping) {
        this.m_linearDamping = linearDamping;
    };
    /// Get the angular damping of the body.
    b2Body.prototype.GetAngularDamping = function () {
        return this.m_angularDamping;
    };
    /// Set the angular damping of the body.
    b2Body.prototype.SetAngularDamping = function (angularDamping) {
        this.m_angularDamping = angularDamping;
    };
    /// Get the gravity scale of the body.
    b2Body.prototype.GetGravityScale = function () {
        return this.m_gravityScale;
    };
    /// Set the gravity scale of the body.
    b2Body.prototype.SetGravityScale = function (scale) {
        this.m_gravityScale = scale;
    };
    /// Set the type of this body. This may alter the mass and velocity.
    b2Body.prototype.SetType = function (type) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (this.m_type === type) {
            return;
        }
        this.m_type = type;
        this.ResetMassData();
        if (this.m_type === b2BodyType.b2_staticBody) {
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_sweep.a0 = this.m_sweep.a;
            this.m_sweep.c0.Copy(this.m_sweep.c);
            this.SynchronizeFixtures();
        }
        this.SetAwake(true);
        this.m_force.SetZero();
        this.m_torque = 0;
        // Delete the attached contacts.
        var ce = this.m_contactList;
        while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.m_world.m_contactManager.Destroy(ce0.contact);
        }
        this.m_contactList = null;
        // Touch the proxies so that new contacts will be created (when appropriate)
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            var proxyCount = f.m_proxyCount;
            for (var i = 0; i < proxyCount; ++i) {
                broadPhase.TouchProxy(f.m_proxies[i].treeNode);
            }
        }
    };
    /// Get the type of this body.
    b2Body.prototype.GetType = function () {
        return this.m_type;
    };
    /// Should this body be treated like a bullet for continuous collision detection?
    b2Body.prototype.SetBullet = function (flag) {
        this.m_bulletFlag = flag;
    };
    /// Is this body treated like a bullet for continuous collision detection?
    b2Body.prototype.IsBullet = function () {
        return this.m_bulletFlag;
    };
    /// You can disable sleeping on this body. If you disable sleeping, the
    /// body will be woken.
    b2Body.prototype.SetSleepingAllowed = function (flag) {
        this.m_autoSleepFlag = flag;
        if (!flag) {
            this.SetAwake(true);
        }
    };
    /// Is this body allowed to sleep
    b2Body.prototype.IsSleepingAllowed = function () {
        return this.m_autoSleepFlag;
    };
    /// Set the sleep state of the body. A sleeping body has very
    /// low CPU cost.
    /// @param flag set to true to wake the body, false to put it to sleep.
    b2Body.prototype.SetAwake = function (flag) {
        if (flag) {
            if (!this.m_awakeFlag) {
                this.m_awakeFlag = true;
                this.m_sleepTime = 0;
            }
        }
        else {
            this.m_awakeFlag = false;
            this.m_sleepTime = 0;
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0;
            this.m_force.SetZero();
            this.m_torque = 0;
        }
    };
    /// Get the sleeping state of this body.
    /// @return true if the body is sleeping.
    b2Body.prototype.IsAwake = function () {
        return this.m_awakeFlag;
    };
    /// Set the active state of the body. An inactive body is not
    /// simulated and cannot be collided with or woken up.
    /// If you pass a flag of true, all fixtures will be added to the
    /// broad-phase.
    /// If you pass a flag of false, all fixtures will be removed from
    /// the broad-phase and all contacts will be destroyed.
    /// Fixtures and joints are otherwise unaffected. You may continue
    /// to create/destroy fixtures and joints on inactive bodies.
    /// Fixtures on an inactive body are implicitly inactive and will
    /// not participate in collisions, ray-casts, or queries.
    /// Joints connected to an inactive body are implicitly inactive.
    /// An inactive body is still owned by a b2World object and remains
    /// in the body list.
    b2Body.prototype.SetActive = function (flag) {
        if (this.m_world.IsLocked()) {
            throw new Error();
        }
        if (flag === this.IsActive()) {
            return;
        }
        this.m_activeFlag = flag;
        if (flag) {
            // Create all proxies.
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (var f = this.m_fixtureList; f; f = f.m_next) {
                f.CreateProxies(broadPhase, this.m_xf);
            }
            // Contacts are created the next time step.
        }
        else {
            // Destroy all proxies.
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (var f = this.m_fixtureList; f; f = f.m_next) {
                f.DestroyProxies(broadPhase);
            }
            // Destroy the attached contacts.
            var ce = this.m_contactList;
            while (ce) {
                var ce0 = ce;
                ce = ce.next;
                this.m_world.m_contactManager.Destroy(ce0.contact);
            }
            this.m_contactList = null;
        }
    };
    /// Get the active state of the body.
    b2Body.prototype.IsActive = function () {
        return this.m_activeFlag;
    };
    /// Set this body to have fixed rotation. This causes the mass
    /// to be reset.
    b2Body.prototype.SetFixedRotation = function (flag) {
        if (this.m_fixedRotationFlag === flag) {
            return;
        }
        this.m_fixedRotationFlag = flag;
        this.m_angularVelocity = 0;
        this.ResetMassData();
    };
    /// Does this body have fixed rotation?
    b2Body.prototype.IsFixedRotation = function () {
        return this.m_fixedRotationFlag;
    };
    /// Get the list of all fixtures attached to this body.
    b2Body.prototype.GetFixtureList = function () {
        return this.m_fixtureList;
    };
    /// Get the list of all joints attached to this body.
    b2Body.prototype.GetJointList = function () {
        return this.m_jointList;
    };
    /// Get the list of all contacts attached to this body.
    /// @warning this list changes during the time step and you may
    /// miss some collisions if you don't use b2ContactListener.
    b2Body.prototype.GetContactList = function () {
        return this.m_contactList;
    };
    /// Get the next body in the world's body list.
    b2Body.prototype.GetNext = function () {
        return this.m_next;
    };
    /// Get the user data pointer that was provided in the body definition.
    b2Body.prototype.GetUserData = function () {
        return this.m_userData;
    };
    /// Set the user data. Use this to store your application specific data.
    b2Body.prototype.SetUserData = function (data) {
        this.m_userData = data;
    };
    /// Get the parent world of this body.
    b2Body.prototype.GetWorld = function () {
        return this.m_world;
    };
    /// Dump this body to a log file
    b2Body.prototype.Dump = function (log) {
        var bodyIndex = this.m_islandIndex;
        log("{\n");
        log("  const bd: b2BodyDef = new b2BodyDef();\n");
        var type_str = "";
        switch (this.m_type) {
            case b2BodyType.b2_staticBody:
                type_str = "b2BodyType.b2_staticBody";
                break;
            case b2BodyType.b2_kinematicBody:
                type_str = "b2BodyType.b2_kinematicBody";
                break;
            case b2BodyType.b2_dynamicBody:
                type_str = "b2BodyType.b2_dynamicBody";
                break;
            default:
                // DEBUG: b2Assert(false);
                break;
        }
        log("  bd.type = %s;\n", type_str);
        log("  bd.position.Set(%.15f, %.15f);\n", this.m_xf.p.x, this.m_xf.p.y);
        log("  bd.angle = %.15f;\n", this.m_sweep.a);
        log("  bd.linearVelocity.Set(%.15f, %.15f);\n", this.m_linearVelocity.x, this.m_linearVelocity.y);
        log("  bd.angularVelocity = %.15f;\n", this.m_angularVelocity);
        log("  bd.linearDamping = %.15f;\n", this.m_linearDamping);
        log("  bd.angularDamping = %.15f;\n", this.m_angularDamping);
        log("  bd.allowSleep = %s;\n", (this.m_autoSleepFlag) ? ("true") : ("false"));
        log("  bd.awake = %s;\n", (this.m_awakeFlag) ? ("true") : ("false"));
        log("  bd.fixedRotation = %s;\n", (this.m_fixedRotationFlag) ? ("true") : ("false"));
        log("  bd.bullet = %s;\n", (this.m_bulletFlag) ? ("true") : ("false"));
        log("  bd.active = %s;\n", (this.m_activeFlag) ? ("true") : ("false"));
        log("  bd.gravityScale = %.15f;\n", this.m_gravityScale);
        log("\n");
        log("  bodies[%d] = this.m_world.CreateBody(bd);\n", this.m_islandIndex);
        log("\n");
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            log("  {\n");
            f.Dump(log, bodyIndex);
            log("  }\n");
        }
        log("}\n");
    };
    b2Body.prototype.SynchronizeFixtures = function () {
        var xf1 = b2Body.SynchronizeFixtures_s_xf1;
        xf1.q.SetAngle(this.m_sweep.a0);
        b2Math_1.b2Rot.MulRV(xf1.q, this.m_sweep.localCenter, xf1.p);
        b2Math_1.b2Vec2.SubVV(this.m_sweep.c0, xf1.p, xf1.p);
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            f.Synchronize(broadPhase, xf1, this.m_xf);
        }
    };
    b2Body.prototype.SynchronizeTransform = function () {
        this.m_xf.q.SetAngle(this.m_sweep.a);
        b2Math_1.b2Rot.MulRV(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        b2Math_1.b2Vec2.SubVV(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    };
    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    b2Body.prototype.ShouldCollide = function (other) {
        // At least one body should be dynamic or kinematic.
        if (this.m_type === b2BodyType.b2_staticBody && other.m_type === b2BodyType.b2_staticBody) {
            return false;
        }
        return this.ShouldCollideConnected(other);
    };
    b2Body.prototype.ShouldCollideConnected = function (other) {
        // Does a joint prevent collision?
        for (var jn = this.m_jointList; jn; jn = jn.next) {
            if (jn.other === other) {
                if (!jn.joint.m_collideConnected) {
                    return false;
                }
            }
        }
        return true;
    };
    b2Body.prototype.Advance = function (alpha) {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        this.m_sweep.Advance(alpha);
        this.m_sweep.c.Copy(this.m_sweep.c0);
        this.m_sweep.a = this.m_sweep.a0;
        this.m_xf.q.SetAngle(this.m_sweep.a);
        b2Math_1.b2Rot.MulRV(this.m_xf.q, this.m_sweep.localCenter, this.m_xf.p);
        b2Math_1.b2Vec2.SubVV(this.m_sweep.c, this.m_xf.p, this.m_xf.p);
    };
    // #if B2_ENABLE_CONTROLLER
    b2Body.prototype.GetControllerList = function () {
        return this.m_controllerList;
    };
    b2Body.prototype.GetControllerCount = function () {
        return this.m_controllerCount;
    };
    /// Creates a fixture from a shape and attach it to this body.
    /// This is a convenience function. Use b2FixtureDef if you need to set parameters
    /// like friction, restitution, user data, or filtering.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// @param shape the shape to be cloned.
    /// @param density the shape density (set to zero for static bodies).
    /// @warning This function is locked during callbacks.
    b2Body.CreateFixtureShapeDensity_s_def = new b2Fixture_1.b2FixtureDef();
    /// Set the mass properties to override the mass properties of the fixtures.
    /// Note that this changes the center of mass position.
    /// Note that creating or destroying fixtures can also alter the mass.
    /// This function has no effect if the body isn't dynamic.
    /// @param massData the mass properties.
    b2Body.SetMassData_s_oldCenter = new b2Math_1.b2Vec2();
    /// This resets the mass properties to the sum of the mass properties of the fixtures.
    /// This normally does not need to be called unless you called SetMassData to override
    /// the mass and you later want to reset the mass.
    b2Body.ResetMassData_s_localCenter = new b2Math_1.b2Vec2();
    b2Body.ResetMassData_s_oldCenter = new b2Math_1.b2Vec2();
    b2Body.ResetMassData_s_massData = new b2Shape_1.b2MassData();
    b2Body.SynchronizeFixtures_s_xf1 = new b2Math_1.b2Transform();
    return b2Body;
}());
exports.b2Body = b2Body;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCb2R5LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvYjJCb2R5LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiwwREFBMEQ7QUFDMUQsdURBQXVEO0FBQ3ZELG1EQUErQztBQUMvQywyQ0FBMkU7QUFFM0UsdURBQWtFO0FBR2xFLHlDQUFxRTtBQUlyRSxTQUFTO0FBRVQsa0JBQWtCO0FBQ2xCLDJEQUEyRDtBQUMzRCx3RUFBd0U7QUFDeEUsbUZBQW1GO0FBQ25GLElBQVksVUFRWDtBQVJELFdBQVksVUFBVTtJQUNwQix3REFBZSxDQUFBO0lBQ2YsNkRBQWlCLENBQUE7SUFDakIsbUVBQW9CLENBQUE7SUFDcEIsK0RBQWtCLENBQUE7SUFFbEIsWUFBWTtJQUNaLG9CQUFvQjtBQUN0QixDQUFDLEVBUlcsVUFBVSxHQUFWLGtCQUFVLEtBQVYsa0JBQVUsUUFRckI7QUF3REQsMEVBQTBFO0FBQzFFLDBGQUEwRjtBQUMxRjtJQUFBO1FBQ0UsaURBQWlEO1FBQ2pELHlFQUF5RTtRQUNsRSxTQUFJLEdBQWUsVUFBVSxDQUFDLGFBQWEsQ0FBQztRQUVuRCx1RUFBdUU7UUFDdkUsbURBQW1EO1FBQ25DLGFBQVEsR0FBVyxJQUFJLGVBQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFFcEQsMkNBQTJDO1FBQ3BDLFVBQUssR0FBVyxDQUFDLENBQUM7UUFFekIsbUVBQW1FO1FBQ25ELG1CQUFjLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRTFELHFDQUFxQztRQUM5QixvQkFBZSxHQUFXLENBQUMsQ0FBQztRQUVuQyw4RUFBOEU7UUFDOUUsMkVBQTJFO1FBQzNFLGtEQUFrRDtRQUMzQyxrQkFBYSxHQUFXLENBQUMsQ0FBQztRQUVqQyxnRkFBZ0Y7UUFDaEYsMkVBQTJFO1FBQzNFLGtEQUFrRDtR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