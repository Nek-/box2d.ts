"use strict";
/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2JointType;
(function (b2JointType) {
    b2JointType[b2JointType["e_unknownJoint"] = 0] = "e_unknownJoint";
    b2JointType[b2JointType["e_revoluteJoint"] = 1] = "e_revoluteJoint";
    b2JointType[b2JointType["e_prismaticJoint"] = 2] = "e_prismaticJoint";
    b2JointType[b2JointType["e_distanceJoint"] = 3] = "e_distanceJoint";
    b2JointType[b2JointType["e_pulleyJoint"] = 4] = "e_pulleyJoint";
    b2JointType[b2JointType["e_mouseJoint"] = 5] = "e_mouseJoint";
    b2JointType[b2JointType["e_gearJoint"] = 6] = "e_gearJoint";
    b2JointType[b2JointType["e_wheelJoint"] = 7] = "e_wheelJoint";
    b2JointType[b2JointType["e_weldJoint"] = 8] = "e_weldJoint";
    b2JointType[b2JointType["e_frictionJoint"] = 9] = "e_frictionJoint";
    b2JointType[b2JointType["e_ropeJoint"] = 10] = "e_ropeJoint";
    b2JointType[b2JointType["e_motorJoint"] = 11] = "e_motorJoint";
    b2JointType[b2JointType["e_areaJoint"] = 12] = "e_areaJoint";
})(b2JointType = exports.b2JointType || (exports.b2JointType = {}));
var b2LimitState;
(function (b2LimitState) {
    b2LimitState[b2LimitState["e_inactiveLimit"] = 0] = "e_inactiveLimit";
    b2LimitState[b2LimitState["e_atLowerLimit"] = 1] = "e_atLowerLimit";
    b2LimitState[b2LimitState["e_atUpperLimit"] = 2] = "e_atUpperLimit";
    b2LimitState[b2LimitState["e_equalLimits"] = 3] = "e_equalLimits";
})(b2LimitState = exports.b2LimitState || (exports.b2LimitState = {}));
var b2Jacobian = /** @class */ (function () {
    function b2Jacobian() {
        this.linear = new b2Math_1.b2Vec2();
        this.angularA = 0;
        this.angularB = 0;
    }
    b2Jacobian.prototype.SetZero = function () {
        this.linear.SetZero();
        this.angularA = 0;
        this.angularB = 0;
        return this;
    };
    b2Jacobian.prototype.Set = function (x, a1, a2) {
        this.linear.Copy(x);
        this.angularA = a1;
        this.angularB = a2;
        return this;
    };
    return b2Jacobian;
}());
exports.b2Jacobian = b2Jacobian;
/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
var b2JointEdge = /** @class */ (function () {
    function b2JointEdge(joint, other) {
        this.prev = null; ///< the previous joint edge in the body's joint list
        this.next = null; ///< the next joint edge in the body's joint list
        this.joint = joint;
        this.other = other;
    }
    return b2JointEdge;
}());
exports.b2JointEdge = b2JointEdge;
/// Joint definitions are used to construct joints.
var b2JointDef = /** @class */ (function () {
    function b2JointDef(type) {
        /// The joint type is set automatically for concrete joint types.
        this.type = b2JointType.e_unknownJoint;
        /// Use this to attach application specific data to your joints.
        this.userData = null;
        /// Set this flag to true if the attached bodies should collide.
        this.collideConnected = false;
        this.type = type;
    }
    return b2JointDef;
}());
exports.b2JointDef = b2JointDef;
/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
var b2Joint = /** @class */ (function () {
    function b2Joint(def) {
        // DEBUG: b2Assert(def.bodyA !== def.bodyB);
        this.m_type = b2JointType.e_unknownJoint;
        this.m_prev = null;
        this.m_next = null;
        this.m_index = 0;
        this.m_islandFlag = false;
        this.m_collideConnected = false;
        this.m_userData = null;
        this.m_type = def.type;
        this.m_edgeA = new b2JointEdge(this, def.bodyB);
        this.m_edgeB = new b2JointEdge(this, def.bodyA);
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;
        this.m_collideConnected = b2Settings_1.b2Maybe(def.collideConnected, false);
        this.m_userData = def.userData;
    }
    /// Get the type of the concrete joint.
    b2Joint.prototype.GetType = function () {
        return this.m_type;
    };
    /// Get the first body attached to this joint.
    b2Joint.prototype.GetBodyA = function () {
        return this.m_bodyA;
    };
    /// Get the second body attached to this joint.
    b2Joint.prototype.GetBodyB = function () {
        return this.m_bodyB;
    };
    /// Get the next joint the world joint list.
    b2Joint.prototype.GetNext = function () {
        return this.m_next;
    };
    /// Get the user data pointer.
    b2Joint.prototype.GetUserData = function () {
        return this.m_userData;
    };
    /// Set the user data pointer.
    b2Joint.prototype.SetUserData = function (data) {
        this.m_userData = data;
    };
    /// Short-cut function to determine if either body is inactive.
    b2Joint.prototype.IsActive = function () {
        return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
    };
    /// Get collide connected.
    /// Note: modifying the collide connect flag won't work correctly because
    /// the flag is only checked when fixture AABBs begin to overlap.
    b2Joint.prototype.GetCollideConnected = function () {
        return this.m_collideConnected;
    };
    /// Dump this joint to the log file.
    b2Joint.prototype.Dump = function (log) {
        log("// Dump is not supported for this joint type.\n");
    };
    /// Shift the origin for any points stored in world coordinates.
    b2Joint.prototype.ShiftOrigin = function (newOrigin) {
    };
    return b2Joint;
}());
exports.b2Joint = b2Joint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJKb2ludC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0pvaW50cy9iMkpvaW50LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiw2REFBNkQ7QUFDN0Qsc0RBQWtEO0FBQ2xELDhDQUFpRDtBQUlqRCxJQUFZLFdBY1g7QUFkRCxXQUFZLFdBQVc7SUFDckIsaUVBQWtCLENBQUE7SUFDbEIsbUVBQW1CLENBQUE7SUFDbkIscUVBQW9CLENBQUE7SUFDcEIsbUVBQW1CLENBQUE7SUFDbkIsK0RBQWlCLENBQUE7SUFDakIsNkRBQWdCLENBQUE7SUFDaEIsMkRBQWUsQ0FBQTtJQUNmLDZEQUFnQixDQUFBO0lBQ2hCLDJEQUFlLENBQUE7SUFDZixtRUFBbUIsQ0FBQTtJQUNuQiw0REFBZ0IsQ0FBQTtJQUNoQiw4REFBaUIsQ0FBQTtJQUNqQiw0REFBZ0IsQ0FBQTtBQUNsQixDQUFDLEVBZFcsV0FBVyxHQUFYLG1CQUFXLEtBQVgsbUJBQVcsUUFjdEI7QUFFRCxJQUFZLFlBS1g7QUFMRCxXQUFZLFlBQVk7SUFDdEIscUVBQW1CLENBQUE7SUFDbkIsbUVBQWtCLENBQUE7SUFDbEIsbUVBQWtCLENBQUE7SUFDbEIsaUVBQWlCLENBQUE7QUFDbkIsQ0FBQyxFQUxXLFlBQVksR0FBWixvQkFBWSxLQUFaLG9CQUFZLFFBS3ZCO0FBRUQ7SUFBQTtRQUNrQixXQUFNLEdBQVcsSUFBSSxlQUFNLEVBQUUsQ0FBQztRQUN2QyxhQUFRLEdBQVcsQ0FBQyxDQUFDO1FBQ3JCLGFBQVEsR0FBVyxDQUFDLENBQUM7SUFlOUIsQ0FBQztJQWJRLDRCQUFPLEdBQWQ7UUFDRSxJQUFJLENBQUMsTUFBTSxDQUFDLE9BQU8sRUFBRSxDQUFDO1FBQ3RCLElBQUksQ0FBQyxRQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2xCLElBQUksQ0FBQyxRQUFRLEdBQUcsQ0FBQyxDQUFD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