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
var b2BroadPhase_1 = require("../Collision/b2BroadPhase");
var b2ContactFactory_1 = require("./Contacts/b2ContactFactory");
var b2Body_1 = require("./b2Body");
var b2WorldCallbacks_1 = require("./b2WorldCallbacks");
// Delegate of b2World.
var b2ContactManager = /** @class */ (function () {
    function b2ContactManager() {
        this.m_broadPhase = new b2BroadPhase_1.b2BroadPhase();
        this.m_contactList = null;
        this.m_contactCount = 0;
        this.m_contactFilter = b2WorldCallbacks_1.b2ContactFilter.b2_defaultFilter;
        this.m_contactListener = b2WorldCallbacks_1.b2ContactListener.b2_defaultListener;
        this.m_allocator = null;
        this.m_contactFactory = new b2ContactFactory_1.b2ContactFactory(this.m_allocator);
    }
    // Broad-phase callback.
    b2ContactManager.prototype.AddPair = function (proxyA, proxyB) {
        // DEBUG: b2Assert(proxyA instanceof b2FixtureProxy);
        // DEBUG: b2Assert(proxyB instanceof b2FixtureProxy);
        var fixtureA = proxyA.fixture;
        var fixtureB = proxyB.fixture;
        var indexA = proxyA.childIndex;
        var indexB = proxyB.childIndex;
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        // Are the fixtures on the same body?
        if (bodyA === bodyB) {
            return;
        }
        // TODO_ERIN use a hash table to remove a potential bottleneck when both
        // bodies have a lot of contacts.
        // Does a contact already exist?
        var edge = bodyB.GetContactList();
        while (edge) {
            if (edge.other === bodyA) {
                var fA = edge.contact.GetFixtureA();
                var fB = edge.contact.GetFixtureB();
                var iA = edge.contact.GetChildIndexA();
                var iB = edge.contact.GetChildIndexB();
                if (fA === fixtureA && fB === fixtureB && iA === indexA && iB === indexB) {
                    // A contact already exists.
                    return;
                }
                if (fA === fixtureB && fB === fixtureA && iA === indexB && iB === indexA) {
                    // A contact already exists.
                    return;
                }
            }
            edge = edge.next;
        }
        // Check user filtering.
        if (this.m_contactFilter && !this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)) {
            return;
        }
        // Call the factory.
        var c = this.m_contactFactory.Create(fixtureA, indexA, fixtureB, indexB);
        if (c === null) {
            return;
        }
        // Contact creation may swap fixtures.
        fixtureA = c.GetFixtureA();
        fixtureB = c.GetFixtureB();
        indexA = c.GetChildIndexA();
        indexB = c.GetChildIndexB();
        bodyA = fixtureA.m_body;
        bodyB = fixtureB.m_body;
        // Insert into the world.
        c.m_prev = null;
        c.m_next = this.m_contactList;
        if (this.m_contactList !== null) {
            this.m_contactList.m_prev = c;
        }
        this.m_contactList = c;
        // Connect to island graph.
        // Connect to body A
        c.m_nodeA.contact = c;
        c.m_nodeA.other = bodyB;
        c.m_nodeA.prev = null;
        c.m_nodeA.next = bodyA.m_contactList;
        if (bodyA.m_contactList !== null) {
            bodyA.m_contactList.prev = c.m_nodeA;
        }
        bodyA.m_contactList = c.m_nodeA;
        // Connect to body B
        c.m_nodeB.contact = c;
        c.m_nodeB.other = bodyA;
        c.m_nodeB.prev = null;
        c.m_nodeB.next = bodyB.m_contactList;
        if (bodyB.m_contactList !== null) {
            bodyB.m_contactList.prev = c.m_nodeB;
        }
        bodyB.m_contactList = c.m_nodeB;
        // Wake up the bodies
        if (!fixtureA.IsSensor() && !fixtureB.IsSensor()) {
            bodyA.SetAwake(true);
            bodyB.SetAwake(true);
        }
        ++this.m_contactCount;
    };
    b2ContactManager.prototype.FindNewContacts = function () {
        this.m_broadPhase.UpdatePairs(this);
    };
    b2ContactManager.prototype.Destroy = function (c) {
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        if (this.m_contactListener && c.IsTouching()) {
            this.m_contactListener.EndContact(c);
        }
        // Remove from the world.
        if (c.m_prev) {
            c.m_prev.m_next = c.m_next;
        }
        if (c.m_next) {
            c.m_next.m_prev = c.m_prev;
        }
        if (c === this.m_contactList) {
            this.m_contactList = c.m_next;
        }
        // Remove from body 1
        if (c.m_nodeA.prev) {
            c.m_nodeA.prev.next = c.m_nodeA.next;
        }
        if (c.m_nodeA.next) {
            c.m_nodeA.next.prev = c.m_nodeA.prev;
        }
        if (c.m_nodeA === bodyA.m_contactList) {
            bodyA.m_contactList = c.m_nodeA.next;
        }
        // Remove from body 2
        if (c.m_nodeB.prev) {
            c.m_nodeB.prev.next = c.m_nodeB.next;
        }
        if (c.m_nodeB.next) {
            c.m_nodeB.next.prev = c.m_nodeB.prev;
        }
        if (c.m_nodeB === bodyB.m_contactList) {
            bodyB.m_contactList = c.m_nodeB.next;
        }
        // Call the factory.
        this.m_contactFactory.Destroy(c);
        --this.m_contactCount;
    };
    // This is the top level collision call for the time step. Here
    // all the narrow phase collision is processed for the world
    // contact list.
    b2ContactManager.prototype.Collide = function () {
        // Update awake contacts.
        var c = this.m_contactList;
        while (c) {
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            var indexA = c.GetChildIndexA();
            var indexB = c.GetChildIndexB();
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();
            // Is this contact flagged for filtering?
            if (c.m_filterFlag) {
                // Check user filtering.
                if (this.m_contactFilter && !this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)) {
                    var cNuke = c;
                    c = cNuke.m_next;
                    this.Destroy(cNuke);
                    continue;
                }
                // Clear the filtering flag.
                c.m_filterFlag = false;
            }
            var activeA = bodyA.IsAwake() && bodyA.m_type !== b2Body_1.b2BodyType.b2_staticBody;
            var activeB = bodyB.IsAwake() && bodyB.m_type !== b2Body_1.b2BodyType.b2_staticBody;
            // At least one body must be awake and it must be dynamic or kinematic.
            if (!activeA && !activeB) {
                c = c.m_next;
                continue;
            }
            var proxyA = fixtureA.m_proxies[indexA].treeNode;
            var proxyB = fixtureB.m_proxies[indexB].treeNode;
            var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
            // Here we destroy contacts that cease to overlap in the broad-phase.
            if (!overlap) {
                var cNuke = c;
                c = cNuke.m_next;
                this.Destroy(cNuke);
                continue;
            }
            // The contact persists.
            c.Update(this.m_contactListener);
            c = c.m_next;
        }
    };
    return b2ContactManager;
}());
exports.b2ContactManager = b2ContactManager;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250YWN0TWFuYWdlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL2IyQ29udGFjdE1hbmFnZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOztBQUVGLDBEQUEwRDtBQUMxRCwwREFBeUQ7QUFHekQsZ0VBQStEO0FBQy9ELG1DQUE4QztBQUU5Qyx1REFBd0U7QUFFeEUsdUJBQXVCO0FBQ3ZCO0lBVUU7UUFUZ0IsaUJBQVksR0FBaUIsSUFBSSwyQkFBWSxFQUFFLENBQUM7UUFDekQsa0JBQWEsR0FBcUIsSUFBSSxDQUFDO1FBQ3ZDLG1CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBQzNCLG9CQUFlLEdBQW9CLGtDQUFlLENBQUMsZ0JBQWdCLENBQUM7UUFDcEUsc0JBQWlCLEdBQXNCLG9DQUFpQixDQUFDLGtCQUFrQixDQUFDO1FBQzVFLGdCQUFXLEdBQVEsSUFBSSxDQUFDO1FBSzdCLElBQUksQ0FBQyxnQkFBZ0IsR0FBRyxJQUFJLG1DQUFnQixDQUFDLElBQUksQ0FBQyxXQUFXLENBQUMsQ0FBQztJQUNqRSxDQUFDO0lBRUQsd0JBQXdCO0lBQ2pCLGtDQUFPLEdBQWQsVUFBZSxNQUFzQixFQUFFLE1BQXNCO1FBQzNELHFEQUFxRDtRQUNyRCxxREFBcUQ7UUFFckQsSUFBSSxRQUFRLEdBQWMsTUFBTSxDQUFDLE9BQU8sQ0FBQztRQUN6QyxJQUFJLFFBQVEsR0FBYyxNQUFNLENBQUMsT0FBTyxDQUFDO1FBRXpDLElBQUksTUFBTSxHQUFXLE1BQU0sQ0FBQyxVQUFVLENBQUM7UUFDdkMsSUFBSSxNQUFNLEdBQVcsTUFBTSxDQUFDLFVBQVUsQ0FBQztRQUV2QyxJQUFJLEtBQUssR0FBVyxRQUFRLENBQUMsT0FBTyxFQUFFLENBQUM7UUFDdkMsSUFBSSxLQUFLLEdBQVcsUUFBUSxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBRXZDLHFDQUFxQztRQUNyQyxJ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