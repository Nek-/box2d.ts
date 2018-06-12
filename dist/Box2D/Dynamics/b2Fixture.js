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
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Collision_1 = require("../Collision/b2Collision");
var b2Shape_1 = require("../Collision/Shapes/b2Shape");
/// This holds contact filtering data.
var b2Filter = /** @class */ (function () {
    function b2Filter() {
        /// The collision category bits. Normally you would just set one bit.
        this.categoryBits = 0x0001;
        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        this.maskBits = 0xFFFF;
        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). Zero means no collision group. Non-zero group
        /// filtering always wins against the mask bits.
        this.groupIndex = 0;
    }
    b2Filter.prototype.Clone = function () {
        return new b2Filter().Copy(this);
    };
    b2Filter.prototype.Copy = function (other) {
        // DEBUG: b2Assert(this !== other);
        this.categoryBits = other.categoryBits;
        this.maskBits = other.maskBits;
        this.groupIndex = other.groupIndex || 0;
        return this;
    };
    b2Filter.DEFAULT = new b2Filter();
    return b2Filter;
}());
exports.b2Filter = b2Filter;
/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
var b2FixtureDef = /** @class */ (function () {
    function b2FixtureDef() {
        /// Use this to store application specific fixture data.
        this.userData = null;
        /// The friction coefficient, usually in the range [0,1].
        this.friction = 0.2;
        /// The restitution (elasticity) usually in the range [0,1].
        this.restitution = 0;
        /// The density, usually in kg/m^2.
        this.density = 0;
        /// A sensor shape collects contact information but never generates a collision
        /// response.
        this.isSensor = false;
        /// Contact filtering data.
        this.filter = new b2Filter();
    }
    return b2FixtureDef;
}());
exports.b2FixtureDef = b2FixtureDef;
/// This proxy is used internally to connect fixtures to the broad-phase.
var b2FixtureProxy = /** @class */ (function () {
    function b2FixtureProxy(fixture) {
        this.aabb = new b2Collision_1.b2AABB();
        this.childIndex = 0;
        this.fixture = fixture;
    }
    return b2FixtureProxy;
}());
exports.b2FixtureProxy = b2FixtureProxy;
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
var b2Fixture = /** @class */ (function () {
    function b2Fixture(def, body) {
        this.m_density = 0;
        this.m_next = null;
        this.m_friction = 0;
        this.m_restitution = 0;
        this.m_proxies = [];
        this.m_proxyCount = 0;
        this.m_filter = new b2Filter();
        this.m_isSensor = false;
        this.m_userData = null;
        this.m_body = body;
        this.m_shape = def.shape.Clone();
    }
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    b2Fixture.prototype.GetType = function () {
        return this.m_shape.GetType();
    };
    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    b2Fixture.prototype.GetShape = function () {
        return this.m_shape;
    };
    /// Set if this fixture is a sensor.
    b2Fixture.prototype.SetSensor = function (sensor) {
        if (sensor !== this.m_isSensor) {
            this.m_body.SetAwake(true);
            this.m_isSensor = sensor;
        }
    };
    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    b2Fixture.prototype.IsSensor = function () {
        return this.m_isSensor;
    };
    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    b2Fixture.prototype.SetFilterData = function (filter) {
        this.m_filter.Copy(filter);
        this.Refilter();
    };
    /// Get the contact filtering data.
    b2Fixture.prototype.GetFilterData = function () {
        return this.m_filter;
    };
    /// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
    b2Fixture.prototype.Refilter = function () {
        // Flag associated contacts for filtering.
        var edge = this.m_body.GetContactList();
        while (edge) {
            var contact = edge.contact;
            var fixtureA = contact.GetFixtureA();
            var fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) {
                contact.FlagForFiltering();
            }
            edge = edge.next;
        }
        var world = this.m_body.GetWorld();
        if (world === null) {
            return;
        }
        // Touch each proxy so that new pairs may be created
        var broadPhase = world.m_contactManager.m_broadPhase;
        for (var i = 0; i < this.m_proxyCount; ++i) {
            broadPhase.TouchProxy(this.m_proxies[i].treeNode);
        }
    };
    /// Get the parent body of this fixture. This is NULL if the fixture is not attached.
    /// @return the parent body.
    b2Fixture.prototype.GetBody = function () {
        return this.m_body;
    };
    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    b2Fixture.prototype.GetNext = function () {
        return this.m_next;
    };
    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    b2Fixture.prototype.GetUserData = function () {
        return this.m_userData;
    };
    /// Set the user data. Use this to store your application specific data.
    b2Fixture.prototype.SetUserData = function (data) {
        this.m_userData = data;
    };
    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    b2Fixture.prototype.TestPoint = function (p) {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    };
    // #if B2_ENABLE_PARTICLE
    b2Fixture.prototype.ComputeDistance = function (p, normal, childIndex) {
        return this.m_shape.ComputeDistance(this.m_body.GetTransform(), p, normal, childIndex);
    };
    // #endif
    /// Cast a ray against this shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    b2Fixture.prototype.RayCast = function (output, input, childIndex) {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
    };
    /// Get the mass data for this fixture. The mass data is based on the density and
    /// the shape. The rotational inertia is about the shape's origin. This operation
    /// may be expensive.
    b2Fixture.prototype.GetMassData = function (massData) {
        if (massData === void 0) { massData = new b2Shape_1.b2MassData(); }
        this.m_shape.ComputeMass(massData, this.m_density);
        return massData;
    };
    /// Set the density of this fixture. This will _not_ automatically adjust the mass
    /// of the body. You must call b2Body::ResetMassData to update the body's mass.
    b2Fixture.prototype.SetDensity = function (density) {
        this.m_density = density;
    };
    /// Get the density of this fixture.
    b2Fixture.prototype.GetDensity = function () {
        return this.m_density;
    };
    /// Get the coefficient of friction.
    b2Fixture.prototype.GetFriction = function () {
        return this.m_friction;
    };
    /// Set the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    b2Fixture.prototype.SetFriction = function (friction) {
        this.m_friction = friction;
    };
    /// Get the coefficient of restitution.
    b2Fixture.prototype.GetRestitution = function () {
        return this.m_restitution;
    };
    /// Set the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    b2Fixture.prototype.SetRestitution = function (restitution) {
        this.m_restitution = restitution;
    };
    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    b2Fixture.prototype.GetAABB = function (childIndex) {
        // DEBUG: b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
        return this.m_proxies[childIndex].aabb;
    };
    /// Dump this fixture to the log file.
    b2Fixture.prototype.Dump = function (log, bodyIndex) {
        log("    const fd: b2FixtureDef = new b2FixtureDef();\n");
        log("    fd.friction = %.15f;\n", this.m_friction);
        log("    fd.restitution = %.15f;\n", this.m_restitution);
        log("    fd.density = %.15f;\n", this.m_density);
        log("    fd.isSensor = %s;\n", (this.m_isSensor) ? ("true") : ("false"));
        log("    fd.filter.categoryBits = %d;\n", this.m_filter.categoryBits);
        log("    fd.filter.maskBits = %d;\n", this.m_filter.maskBits);
        log("    fd.filter.groupIndex = %d;\n", this.m_filter.groupIndex);
        this.m_shape.Dump(log);
        log("\n");
        log("    fd.shape = shape;\n");
        log("\n");
        log("    bodies[%d].CreateFixture(fd);\n", bodyIndex);
    };
    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
    b2Fixture.prototype.Create = function (def) {
        var _this = this;
        this.m_userData = def.userData;
        this.m_friction = b2Settings_1.b2Maybe(def.friction, 0.2);
        this.m_restitution = b2Settings_1.b2Maybe(def.restitution, 0);
        // this.m_body = body;
        this.m_next = null;
        this.m_filter.Copy(b2Settings_1.b2Maybe(def.filter, b2Filter.DEFAULT));
        this.m_isSensor = b2Settings_1.b2Maybe(def.isSensor, false);
        // Reserve proxy space
        // const childCount = m_shape->GetChildCount();
        // m_proxies = (b2FixtureProxy*)allocator->Allocate(childCount * sizeof(b2FixtureProxy));
        // for (int32 i = 0; i < childCount; ++i)
        // {
        //   m_proxies[i].fixture = NULL;
        //   m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
        // }
        // this.m_proxies = b2FixtureProxy.MakeArray(this.m_shape.GetChildCount());
        this.m_proxies = b2Settings_1.b2MakeArray(this.m_shape.GetChildCount(), function (i) { return new b2FixtureProxy(_this); });
        this.m_proxyCount = 0;
        this.m_density = b2Settings_1.b2Maybe(def.density, 0);
    };
    b2Fixture.prototype.Destroy = function () {
        // The proxies must be destroyed before calling this.
        // DEBUG: b2Assert(this.m_proxyCount === 0);
        // Free the proxy array.
        // int32 childCount = m_shape->GetChildCount();
        // allocator->Free(m_proxies, childCount * sizeof(b2FixtureProxy));
        // m_proxies = NULL;
        // this.m_shape = null;
    };
    // These support body activation/deactivation.
    b2Fixture.prototype.CreateProxies = function (broadPhase, xf) {
        // DEBUG: b2Assert(this.m_proxyCount === 0);
        // Create proxies in the broad-phase.
        this.m_proxyCount = this.m_shape.GetChildCount();
        for (var i = 0; i < this.m_proxyCount; ++i) {
            var proxy = this.m_proxies[i] = new b2FixtureProxy(this);
            this.m_shape.ComputeAABB(proxy.aabb, xf, i);
            proxy.treeNode = broadPhase.CreateProxy(proxy.aabb, proxy);
            proxy.childIndex = i;
        }
    };
    b2Fixture.prototype.DestroyProxies = function (broadPhase) {
        // Destroy proxies in the broad-phase.
        for (var i = 0; i < this.m_proxyCount; ++i) {
            var proxy = this.m_proxies[i];
            proxy.treeNode.userData = null;
            broadPhase.DestroyProxy(proxy.treeNode);
            delete proxy.treeNode; // = null;
        }
        this.m_proxyCount = 0;
    };
    b2Fixture.prototype.Synchronize = function (broadPhase, transform1, transform2) {
        if (this.m_proxyCount === 0) {
            return;
        }
        for (var i = 0; i < this.m_proxyCount; ++i) {
            var proxy = this.m_proxies[i];
            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            var aabb1 = b2Fixture.Synchronize_s_aabb1;
            var aabb2 = b2Fixture.Synchronize_s_aabb2;
            this.m_shape.ComputeAABB(aabb1, transform1, i);
            this.m_shape.ComputeAABB(aabb2, transform2, i);
            proxy.aabb.Combine2(aabb1, aabb2);
            var displacement = b2Math_1.b2Vec2.SubVV(transform2.p, transform1.p, b2Fixture.Synchronize_s_displacement);
            broadPhase.MoveProxy(proxy.treeNode, proxy.aabb, displacement);
        }
    };
    b2Fixture.Synchronize_s_aabb1 = new b2Collision_1.b2AABB();
    b2Fixture.Synchronize_s_aabb2 = new b2Collision_1.b2AABB();
    b2Fixture.Synchronize_s_displacement = new b2Math_1.b2Vec2();
    return b2Fixture;
}());
exports.b2Fixture = b2Fixture;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJGaXh0dXJlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvYjJGaXh0dXJlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiwwREFBMEQ7QUFDMUQsbURBQTREO0FBQzVELDJDQUF1RDtBQUV2RCx3REFBbUY7QUFFbkYsdURBQStFO0FBa0IvRSxzQ0FBc0M7QUFDdEM7SUFBQTtRQUdFLHFFQUFxRTtRQUM5RCxpQkFBWSxHQUFXLE1BQU0sQ0FBQztRQUVyQyxpRUFBaUU7UUFDakUscUNBQXFDO1FBQzlCLGFBQVEsR0FBVyxNQUFNLENBQUM7UUFFakMsaUZBQWlGO1FBQ2pGLCtFQUErRTtRQUMvRSxnREFBZ0Q7UUFDekMsZUFBVSxHQUFXLENBQUMsQ0FBQztJQWFoQyxDQUFDO0lBWFEsd0JBQUssR0FBWjtRQUNFLE9BQU8sSUFBSSxRQUFRLEVBQUUsQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7SUFDbkMsQ0FBQztJQUVNLHVCQUFJLEdBQVgsVUFBWSxLQUFnQjtRQUMxQixtQ0FBbUM7UUFDbkMsSUFBSSxDQUFDLFlBQVksR0FBRyxLQUFLLENBQUMsWUFBWSxDQUFDO1FBQ3ZDLElBQUksQ0FBQyxRQUFRLEdBQUcsS0FBSyxDQUFDLFFBQVEsQ0FBQztRQUMvQixJQUFJLENBQUMsVUFBVSxHQUFHLEtBQUssQ0FBQyxVQUFVLElBQUksQ0FBQyxDQUFDO1FBQ3hDLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQXhCc0IsZ0JBQU8sR0FBdUIsSUFBSSxRQUFRLEVBQUUsQ0FBQztJQXlCdEUsZUFBQztDQUFBLEFBMUJELElBMEJDO0FBMUJZLDRCQUFRO0FBdURyQiwyRUFBMkU7QUFDM0UsMEVBQTBFO0FBQzFFO0lBQUE7UUFLRSx3REFBd0Q7UUFDakQsYUFBUSxHQUFRLElBQUksQ0FBQztRQUU1Qix5REFBeUQ7UUFDbEQsYUFBUSxHQUFX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