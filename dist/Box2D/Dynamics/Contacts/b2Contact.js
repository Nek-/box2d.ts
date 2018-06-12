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
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Collision_1 = require("../../Collision/b2Collision");
var b2Collision_2 = require("../../Collision/b2Collision");
var b2TimeOfImpact_1 = require("../../Collision/b2TimeOfImpact");
/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
function b2MixFriction(friction1, friction2) {
    return b2Math_1.b2Sqrt(friction1 * friction2);
}
exports.b2MixFriction = b2MixFriction;
/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
function b2MixRestitution(restitution1, restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}
exports.b2MixRestitution = b2MixRestitution;
var b2ContactEdge = /** @class */ (function () {
    function b2ContactEdge(contact) {
        this.prev = null; ///< the previous contact edge in the body's contact list
        this.next = null; ///< the next contact edge in the body's contact list
        this.contact = contact;
    }
    return b2ContactEdge;
}());
exports.b2ContactEdge = b2ContactEdge;
var b2Contact = /** @class */ (function () {
    function b2Contact() {
        this.m_islandFlag = false; /// Used when crawling contact graph when forming islands.
        this.m_touchingFlag = false; /// Set when the shapes are touching.
        this.m_enabledFlag = false; /// This contact can be disabled (by user)
        this.m_filterFlag = false; /// This contact needs filtering because a fixture filter was changed.
        this.m_bulletHitFlag = false; /// This bullet contact had a TOI event
        this.m_toiFlag = false; /// This contact has a valid TOI in m_toi
        this.m_prev = null;
        this.m_next = null;
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_manifold = new b2Collision_1.b2Manifold(); // TODO: readonly
        this.m_toiCount = 0;
        this.m_toi = 0;
        this.m_friction = 0;
        this.m_restitution = 0;
        this.m_tangentSpeed = 0;
        this.m_oldManifold = new b2Collision_1.b2Manifold(); // TODO: readonly
        this.m_nodeA = new b2ContactEdge(this);
        this.m_nodeB = new b2ContactEdge(this);
    }
    b2Contact.prototype.GetManifold = function () {
        return this.m_manifold;
    };
    b2Contact.prototype.GetWorldManifold = function (worldManifold) {
        var bodyA = this.m_fixtureA.GetBody();
        var bodyB = this.m_fixtureB.GetBody();
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
    };
    b2Contact.prototype.IsTouching = function () {
        return this.m_touchingFlag;
    };
    b2Contact.prototype.SetEnabled = function (flag) {
        this.m_enabledFlag = flag;
    };
    b2Contact.prototype.IsEnabled = function () {
        return this.m_enabledFlag;
    };
    b2Contact.prototype.GetNext = function () {
        return this.m_next;
    };
    b2Contact.prototype.GetFixtureA = function () {
        return this.m_fixtureA;
    };
    b2Contact.prototype.GetChildIndexA = function () {
        return this.m_indexA;
    };
    b2Contact.prototype.GetFixtureB = function () {
        return this.m_fixtureB;
    };
    b2Contact.prototype.GetChildIndexB = function () {
        return this.m_indexB;
    };
    b2Contact.prototype.FlagForFiltering = function () {
        this.m_filterFlag = true;
    };
    b2Contact.prototype.SetFriction = function (friction) {
        this.m_friction = friction;
    };
    b2Contact.prototype.GetFriction = function () {
        return this.m_friction;
    };
    b2Contact.prototype.ResetFriction = function () {
        this.m_friction = b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
    };
    b2Contact.prototype.SetRestitution = function (restitution) {
        this.m_restitution = restitution;
    };
    b2Contact.prototype.GetRestitution = function () {
        return this.m_restitution;
    };
    b2Contact.prototype.ResetRestitution = function () {
        this.m_restitution = b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    };
    b2Contact.prototype.SetTangentSpeed = function (speed) {
        this.m_tangentSpeed = speed;
    };
    b2Contact.prototype.GetTangentSpeed = function () {
        return this.m_tangentSpeed;
    };
    b2Contact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        this.m_islandFlag = false;
        this.m_touchingFlag = false;
        this.m_enabledFlag = true;
        this.m_filterFlag = false;
        this.m_bulletHitFlag = false;
        this.m_toiFlag = false;
        this.m_fixtureA = fixtureA;
        this.m_fixtureB = fixtureB;
        this.m_indexA = indexA;
        this.m_indexB = indexB;
        this.m_manifold.pointCount = 0;
        this.m_prev = null;
        this.m_next = null;
        delete this.m_nodeA.contact; // = null;
        this.m_nodeA.prev = null;
        this.m_nodeA.next = null;
        delete this.m_nodeA.other; // = null;
        delete this.m_nodeB.contact; // = null;
        this.m_nodeB.prev = null;
        this.m_nodeB.next = null;
        delete this.m_nodeB.other; // = null;
        this.m_toiCount = 0;
        this.m_friction = b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
        this.m_restitution = b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    };
    b2Contact.prototype.Update = function (listener) {
        var tManifold = this.m_oldManifold;
        this.m_oldManifold = this.m_manifold;
        this.m_manifold = tManifold;
        // Re-enable this contact.
        this.m_enabledFlag = true;
        var touching = false;
        var wasTouching = this.m_touchingFlag;
        var sensorA = this.m_fixtureA.IsSensor();
        var sensorB = this.m_fixtureB.IsSensor();
        var sensor = sensorA || sensorB;
        var bodyA = this.m_fixtureA.GetBody();
        var bodyB = this.m_fixtureB.GetBody();
        var xfA = bodyA.GetTransform();
        var xfB = bodyB.GetTransform();
        ///const aabbOverlap = b2TestOverlapAABB(this.m_fixtureA.GetAABB(0), this.m_fixtureB.GetAABB(0));
        // Is this contact a sensor?
        if (sensor) {
            ///if (aabbOverlap)
            ///{
            var shapeA = this.m_fixtureA.GetShape();
            var shapeB = this.m_fixtureB.GetShape();
            touching = b2Collision_2.b2TestOverlapShape(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);
            ///}
            // Sensors don't generate manifolds.
            this.m_manifold.pointCount = 0;
        }
        else {
            ///if (aabbOverlap)
            ///{
            this.Evaluate(this.m_manifold, xfA, xfB);
            touching = this.m_manifold.pointCount > 0;
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (var i = 0; i < this.m_manifold.pointCount; ++i) {
                var mp2 = this.m_manifold.points[i];
                mp2.normalImpulse = 0;
                mp2.tangentImpulse = 0;
                var id2 = mp2.id;
                for (var j = 0; j < this.m_oldManifold.pointCount; ++j) {
                    var mp1 = this.m_oldManifold.points[j];
                    if (mp1.id.key === id2.key) {
                        mp2.normalImpulse = mp1.normalImpulse;
                        mp2.tangentImpulse = mp1.tangentImpulse;
                        break;
                    }
                }
            }
            ///}
            ///else
            ///{
            ///  this.m_manifold.pointCount = 0;
            ///}
            if (touching !== wasTouching) {
                bodyA.SetAwake(true);
                bodyB.SetAwake(true);
            }
        }
        this.m_touchingFlag = touching;
        if (!wasTouching && touching && listener) {
            listener.BeginContact(this);
        }
        if (wasTouching && !touching && listener) {
            listener.EndContact(this);
        }
        if (!sensor && touching && listener) {
            listener.PreSolve(this, this.m_oldManifold);
        }
    };
    b2Contact.prototype.ComputeTOI = function (sweepA, sweepB) {
        var input = b2Contact.ComputeTOI_s_input;
        input.proxyA.SetShape(this.m_fixtureA.GetShape(), this.m_indexA);
        input.proxyB.SetShape(this.m_fixtureB.GetShape(), this.m_indexB);
        input.sweepA.Copy(sweepA);
        input.sweepB.Copy(sweepB);
        input.tMax = b2Settings_1.b2_linearSlop;
        var output = b2Contact.ComputeTOI_s_output;
        b2TimeOfImpact_1.b2TimeOfImpact(output, input);
        return output.t;
    };
    b2Contact.ComputeTOI_s_input = new b2TimeOfImpact_1.b2TOIInput();
    b2Contact.ComputeTOI_s_output = new b2TimeOfImpact_1.b2TOIOutput();
    return b2Contact;
}());
exports.b2Contact = b2Contact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250YWN0LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvQ29udGFjdHMvYjJDb250YWN0LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRixzREFBd0Q7QUFDeEQsOENBQW1FO0FBQ25FLDJEQUF3RztBQUN4RywyREFBaUU7QUFDakUsaUVBQXlGO0FBTXpGLDhGQUE4RjtBQUM5Rix3Q0FBd0M7QUFDeEMsdUJBQThCLFNBQWlCLEVBQUUsU0FBaUI7SUFDaEUsT0FBTyxlQUFNLENBQUMsU0FBUyxHQUFHLFNBQVMsQ0FBQyxDQUFDO0FBQ3ZDLENBQUM7QUFGRCxzQ0FFQztBQUVELDhGQUE4RjtBQUM5RixpREFBaUQ7QUFDakQsMEJBQWlDLFlBQW9CLEVBQUUsWUFBb0I7SUFDekUsT0FBTyxZQUFZLEdBQUcsWUFBWSxDQUFDLENBQUMsQ0FBQyxZQUFZLENBQUMsQ0FBQyxDQUFDLFlBQVksQ0FBQztBQUNuRSxDQUFDO0FBRkQsNENBRUM7QUFFRDtJQUtFLHVCQUFZLE9BQWtCO1FBRnZCLFNBQUksR0FBeUIsSUFBSSxDQUFDLENBQUMseURBQXlEO1FBQzVGLFNBQUksR0FBeUIsSUFBSSxDQUFDLENBQUMscURBQXFEO1FBRTdGLElBQUksQ0FBQyxPQUFPLEdBQUcsT0FBTyxDQUFDO0lBQ3pCLENBQUM7SUFDSCxvQkFBQztBQUFELENBQUMsQUFSRCxJQVFDO0FBUlksc0NBQWE7QUFVMUI7SUFnQ0U7UUEvQk8saUJBQVksR0FBWSxLQUFLLENBQUMsQ0FBQywwREFBMEQ7UUFDekYsbUJBQWMsR0FBWSxLQUFLLENBQUMsQ0FBQyxxQ0FBcUM7UUFDdEUsa0JBQWEsR0FBWSxLQUFLLENBQUMsQ0FBQywwQ0FBMEM7UUFDMUUsaUJBQVksR0FBWSxLQUFLLENBQUMsQ0FBQyxzRUFBc0U7UUFDckcsb0JBQWUsR0FBWSxLQUFLLENBQUMsQ0FBQyx1Q0FB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