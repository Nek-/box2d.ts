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
System.register(["../Common/b2Settings", "../Common/b2Math", "../Collision/b2Collision", "../Collision/Shapes/b2Shape"], function (exports_1, context_1) {
    "use strict";
    var b2Settings_1, b2Math_1, b2Collision_1, b2Shape_1, b2Filter, b2FixtureDef, b2FixtureProxy, b2Fixture;
    var __moduleName = context_1 && context_1.id;
    return {
        setters: [
            function (b2Settings_1_1) {
                b2Settings_1 = b2Settings_1_1;
            },
            function (b2Math_1_1) {
                b2Math_1 = b2Math_1_1;
            },
            function (b2Collision_1_1) {
                b2Collision_1 = b2Collision_1_1;
            },
            function (b2Shape_1_1) {
                b2Shape_1 = b2Shape_1_1;
            }
        ],
        execute: function () {
            /// This holds contact filtering data.
            b2Filter = class b2Filter {
                constructor() {
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
                Clone() {
                    return new b2Filter().Copy(this);
                }
                Copy(other) {
                    ///b2Assert(this !== other);
                    this.categoryBits = other.categoryBits;
                    this.maskBits = other.maskBits;
                    this.groupIndex = other.groupIndex || 0;
                    return this;
                }
            };
            b2Filter.DEFAULT = new b2Filter();
            exports_1("b2Filter", b2Filter);
            /// A fixture definition is used to create a fixture. This class defines an
            /// abstract fixture definition. You can reuse fixture definitions safely.
            b2FixtureDef = class b2FixtureDef {
                constructor() {
                    /// The shape, this must be set. The shape will be cloned, so you
                    /// can create the shape on the stack.
                    this.shape = null;
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
            };
            exports_1("b2FixtureDef", b2FixtureDef);
            /// This proxy is used internally to connect fixtures to the broad-phase.
            b2FixtureProxy = class b2FixtureProxy {
                // public static MakeArray(length: number): b2FixtureProxy[] {
                //   return b2MakeArray(length, (i) => new b2FixtureProxy());
                // }
                constructor(fixture) {
                    this.aabb = new b2Collision_1.b2AABB();
                    this.childIndex = 0;
                    this.treeNode = null;
                    this.fixture = fixture;
                }
            };
            exports_1("b2FixtureProxy", b2FixtureProxy);
            /// A fixture is used to attach a shape to a body for collision detection. A fixture
            /// inherits its transform from its parent. Fixtures hold additional non-geometric data
            /// such as friction, collision filters, etc.
            /// Fixtures are created via b2Body::CreateFixture.
            /// @warning you cannot reuse fixtures.
            b2Fixture = class b2Fixture {
                constructor(def, body) {
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
                GetType() {
                    return this.m_shape.GetType();
                }
                /// Get the child shape. You can modify the child shape, however you should not change the
                /// number of vertices because this will crash some collision caching mechanisms.
                /// Manipulating the shape may lead to non-physical behavior.
                GetShape() {
                    return this.m_shape;
                }
                /// Set if this fixture is a sensor.
                SetSensor(sensor) {
                    if (sensor !== this.m_isSensor) {
                        this.m_body.SetAwake(true);
                        this.m_isSensor = sensor;
                    }
                }
                /// Is this fixture a sensor (non-solid)?
                /// @return the true if the shape is a sensor.
                IsSensor() {
                    return this.m_isSensor;
                }
                /// Set the contact filtering data. This will not update contacts until the next time
                /// step when either parent body is active and awake.
                /// This automatically calls Refilter.
                SetFilterData(filter) {
                    this.m_filter.Copy(filter);
                    this.Refilter();
                }
                /// Get the contact filtering data.
                GetFilterData() {
                    return this.m_filter;
                }
                /// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
                Refilter() {
                    // Flag associated contacts for filtering.
                    let edge = this.m_body.GetContactList();
                    while (edge) {
                        const contact = edge.contact;
                        const fixtureA = contact.GetFixtureA();
                        const fixtureB = contact.GetFixtureB();
                        if (fixtureA === this || fixtureB === this) {
                            contact.FlagForFiltering();
                        }
                        edge = edge.next;
                    }
                    const world = this.m_body.GetWorld();
                    if (world === null) {
                        return;
                    }
                    // Touch each proxy so that new pairs may be created
                    const broadPhase = world.m_contactManager.m_broadPhase;
                    for (let i = 0; i < this.m_proxyCount; ++i) {
                        broadPhase.TouchProxy(this.m_proxies[i].treeNode);
                    }
                }
                /// Get the parent body of this fixture. This is NULL if the fixture is not attached.
                /// @return the parent body.
                GetBody() {
                    return this.m_body;
                }
                /// Get the next fixture in the parent body's fixture list.
                /// @return the next shape.
                GetNext() {
                    return this.m_next;
                }
                /// Get the user data that was assigned in the fixture definition. Use this to
                /// store your application specific data.
                GetUserData() {
                    return this.m_userData;
                }
                /// Set the user data. Use this to store your application specific data.
                SetUserData(data) {
                    this.m_userData = data;
                }
                /// Test a point for containment in this fixture.
                /// @param p a point in world coordinates.
                TestPoint(p) {
                    return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
                }
                // #if B2_ENABLE_PARTICLE
                ComputeDistance(p, normal, childIndex) {
                    return this.m_shape.ComputeDistance(this.m_body.GetTransform(), p, normal, childIndex);
                }
                // #endif
                /// Cast a ray against this shape.
                /// @param output the ray-cast results.
                /// @param input the ray-cast input parameters.
                RayCast(output, input, childIndex) {
                    return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
                }
                /// Get the mass data for this fixture. The mass data is based on the density and
                /// the shape. The rotational inertia is about the shape's origin. This operation
                /// may be expensive.
                GetMassData(massData = new b2Shape_1.b2MassData()) {
                    this.m_shape.ComputeMass(massData, this.m_density);
                    return massData;
                }
                /// Set the density of this fixture. This will _not_ automatically adjust the mass
                /// of the body. You must call b2Body::ResetMassData to update the body's mass.
                SetDensity(density) {
                    this.m_density = density;
                }
                /// Get the density of this fixture.
                GetDensity() {
                    return this.m_density;
                }
                /// Get the coefficient of friction.
                GetFriction() {
                    return this.m_friction;
                }
                /// Set the coefficient of friction. This will _not_ change the friction of
                /// existing contacts.
                SetFriction(friction) {
                    this.m_friction = friction;
                }
                /// Get the coefficient of restitution.
                GetRestitution() {
                    return this.m_restitution;
                }
                /// Set the coefficient of restitution. This will _not_ change the restitution of
                /// existing contacts.
                SetRestitution(restitution) {
                    this.m_restitution = restitution;
                }
                /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
                /// If you need a more accurate AABB, compute it using the shape and
                /// the body transform.
                GetAABB(childIndex) {
                    ///b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
                    return this.m_proxies[childIndex].aabb;
                }
                /// Dump this fixture to the log file.
                Dump(log, bodyIndex) {
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
                }
                // We need separation create/destroy functions from the constructor/destructor because
                // the destructor cannot access the allocator (no destructor arguments allowed by C++).
                Create(/*body: b2Body,*/ def) {
                    function maybe(value, _default) {
                        return value !== undefined ? value : _default;
                    }
                    this.m_userData = def.userData;
                    this.m_friction = maybe(def.friction, 0.2);
                    this.m_restitution = maybe(def.restitution, 0);
                    // this.m_body = body;
                    this.m_next = null;
                    this.m_filter.Copy(maybe(def.filter, b2Filter.DEFAULT));
                    this.m_isSensor = maybe(def.isSensor, false);
                    // this.m_shape = def.shape.Clone();
                    // Reserve proxy space
                    // const childCount = m_shape->GetChildCount();
                    // m_proxies = (b2FixtureProxy*)allocator->Allocate(childCount * sizeof(b2FixtureProxy));
                    // for (int32 i = 0; i < childCount; ++i)
                    // {
                    //   m_proxies[i].fixture = NULL;
                    //   m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
                    // }
                    // this.m_proxies = b2FixtureProxy.MakeArray(this.m_shape.GetChildCount());
                    this.m_proxies = b2Settings_1.b2MakeArray(this.m_shape.GetChildCount(), (i) => new b2FixtureProxy(this));
                    this.m_proxyCount = 0;
                    this.m_density = maybe(def.density, 0);
                }
                Destroy() {
                    // The proxies must be destroyed before calling this.
                    ///b2Assert(this.m_proxyCount === 0);
                    // Free the proxy array.
                    // int32 childCount = m_shape->GetChildCount();
                    // allocator->Free(m_proxies, childCount * sizeof(b2FixtureProxy));
                    // m_proxies = NULL;
                    // this.m_shape = null;
                }
                // These support body activation/deactivation.
                CreateProxies(broadPhase, xf) {
                    ///b2Assert(this.m_proxyCount === 0);
                    // Create proxies in the broad-phase.
                    this.m_proxyCount = this.m_shape.GetChildCount();
                    for (let i = 0; i < this.m_proxyCount; ++i) {
                        const proxy = this.m_proxies[i] = new b2FixtureProxy(this);
                        this.m_shape.ComputeAABB(proxy.aabb, xf, i);
                        proxy.treeNode = broadPhase.CreateProxy(proxy.aabb, proxy);
                        // proxy.fixture = this;
                        proxy.childIndex = i;
                    }
                }
                DestroyProxies(broadPhase) {
                    // Destroy proxies in the broad-phase.
                    for (let i = 0; i < this.m_proxyCount; ++i) {
                        const proxy = this.m_proxies[i];
                        proxy.treeNode.userData = null;
                        broadPhase.DestroyProxy(proxy.treeNode);
                        proxy.treeNode = null;
                    }
                    this.m_proxyCount = 0;
                }
                Synchronize(broadPhase, transform1, transform2) {
                    if (this.m_proxyCount === 0) {
                        return;
                    }
                    for (let i = 0; i < this.m_proxyCount; ++i) {
                        const proxy = this.m_proxies[i];
                        // Compute an AABB that covers the swept shape (may miss some rotation effect).
                        const aabb1 = b2Fixture.Synchronize_s_aabb1;
                        const aabb2 = b2Fixture.Synchronize_s_aabb2;
                        this.m_shape.ComputeAABB(aabb1, transform1, i);
                        this.m_shape.ComputeAABB(aabb2, transform2, i);
                        proxy.aabb.Combine2(aabb1, aabb2);
                        const displacement = b2Math_1.b2Vec2.SubVV(transform2.p, transform1.p, b2Fixture.Synchronize_s_displacement);
                        broadPhase.MoveProxy(proxy.treeNode, proxy.aabb, displacement);
                    }
                }
            };
            b2Fixture.Synchronize_s_aabb1 = new b2Collision_1.b2AABB();
            b2Fixture.Synchronize_s_aabb2 = new b2Collision_1.b2AABB();
            b2Fixture.Synchronize_s_displacement = new b2Math_1.b2Vec2();
            exports_1("b2Fixture", b2Fixture);
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJGaXh0dXJlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiYjJGaXh0dXJlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7WUF5QkYsc0NBQXNDO1lBQ3RDLFdBQUE7Z0JBQUE7b0JBR0UscUVBQXFFO29CQUM5RCxpQkFBWSxHQUFXLE1BQU0sQ0FBQztvQkFFckMsaUVBQWlFO29CQUNqRSxxQ0FBcUM7b0JBQzlCLGFBQVEsR0FBVyxNQUFNLENBQUM7b0JBRWpDLGlGQUFpRjtvQkFDakYsK0VBQStFO29CQUMvRSxnREFBZ0Q7b0JBQ3pDLGVBQVUsR0FBVyxDQUFDLENBQUM7Z0JBYWhDLENBQUM7Z0JBWFEsS0FBSztvQkFDVixPQUFPLElBQUksUUFBUSxFQUFFLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDO2dCQUNuQyxDQUFDO2dCQUVNLElBQUksQ0FBQyxLQUFnQjtvQkFDMUIsNEJBQTRCO29CQUM1QixJQUFJLENBQUMsWUFBWSxHQUFHLEtBQUssQ0FBQyxZQUFZLENBQUM7b0JBQ3ZDLElBQUksQ0FBQyxRQUFRLEdBQUcsS0FBSyxDQUFDLFFBQVEsQ0FBQztvQkFDL0IsSUFBSSxDQUFDLFVBQVUsR0FBRyxLQUFLLENBQUMsVUFBVSxJQUFJLENBQUMsQ0FBQztvQkFDeEMsT0FBTyxJQUFJLENBQUM7Z0JBQ2QsQ0FBQzthQUNGLENBQUE7WUF6QndCLGdCQUFPLEdBQXVCLElBQUksUUFBUSxFQUFFLENBQUM7O1lBc0R0RSwyRUFBMkU7WUFDM0UsMEVBQTBFO1lBQzFFLGVBQUE7Z0JBQUE7b0JBQ0UsaUVBQWlFO29CQUNqRSxzQ0FBc0M7b0JBQy9CLFVBQUssR0FBWSxJQUFJLENBQUM7b0JBRTdCLHdEQUF3RDtvQkFDakQsYUFBUSxHQUFRLElBQUksQ0FBQztvQkFFNUIseURBQXlEO29CQUNsRCxhQUFRLEdBQVcsR0FBRyxDQUFDO29CQUU5Qiw0REFBNEQ7b0JB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