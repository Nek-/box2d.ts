"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var b2Joint_1 = require("./b2Joint");
var b2AreaJoint_1 = require("./b2AreaJoint");
var b2DistanceJoint_1 = require("./b2DistanceJoint");
var b2FrictionJoint_1 = require("./b2FrictionJoint");
var b2GearJoint_1 = require("./b2GearJoint");
var b2MotorJoint_1 = require("./b2MotorJoint");
var b2MouseJoint_1 = require("./b2MouseJoint");
var b2PrismaticJoint_1 = require("./b2PrismaticJoint");
var b2PulleyJoint_1 = require("./b2PulleyJoint");
var b2RevoluteJoint_1 = require("./b2RevoluteJoint");
var b2RopeJoint_1 = require("./b2RopeJoint");
var b2WeldJoint_1 = require("./b2WeldJoint");
var b2WheelJoint_1 = require("./b2WheelJoint");
var b2JointFactory = /** @class */ (function () {
    function b2JointFactory() {
    }
    b2JointFactory.Create = function (def, allocator) {
        switch (def.type) {
            case b2Joint_1.b2JointType.e_distanceJoint: return new b2DistanceJoint_1.b2DistanceJoint(def);
            case b2Joint_1.b2JointType.e_mouseJoint: return new b2MouseJoint_1.b2MouseJoint(def);
            case b2Joint_1.b2JointType.e_prismaticJoint: return new b2PrismaticJoint_1.b2PrismaticJoint(def);
            case b2Joint_1.b2JointType.e_revoluteJoint: return new b2RevoluteJoint_1.b2RevoluteJoint(def);
            case b2Joint_1.b2JointType.e_pulleyJoint: return new b2PulleyJoint_1.b2PulleyJoint(def);
            case b2Joint_1.b2JointType.e_gearJoint: return new b2GearJoint_1.b2GearJoint(def);
            case b2Joint_1.b2JointType.e_wheelJoint: return new b2WheelJoint_1.b2WheelJoint(def);
            case b2Joint_1.b2JointType.e_weldJoint: return new b2WeldJoint_1.b2WeldJoint(def);
            case b2Joint_1.b2JointType.e_frictionJoint: return new b2FrictionJoint_1.b2FrictionJoint(def);
            case b2Joint_1.b2JointType.e_ropeJoint: return new b2RopeJoint_1.b2RopeJoint(def);
            case b2Joint_1.b2JointType.e_motorJoint: return new b2MotorJoint_1.b2MotorJoint(def);
            case b2Joint_1.b2JointType.e_areaJoint: return new b2AreaJoint_1.b2AreaJoint(def);
        }
        throw new Error();
    };
    b2JointFactory.Destroy = function (joint, allocator) {
    };
    return b2JointFactory;
}());
exports.b2JointFactory = b2JointFactory;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJKb2ludEZhY3RvcnkuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Kb2ludHMvYjJKb2ludEZhY3RvcnkudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7QUFBQSxxQ0FBOEQ7QUFDOUQsNkNBQTZEO0FBQzdELHFEQUF5RTtBQUN6RSxxREFBeUU7QUFDekUsNkNBQTZEO0FBQzdELCtDQUFnRTtBQUNoRSwrQ0FBZ0U7QUFDaEUsdURBQTRFO0FBQzVFLGlEQUFtRTtBQUNuRSxxREFBeUU7QUFDekUsNkNBQTZEO0FBQzdELDZDQUE2RDtBQUM3RCwrQ0FBZ0U7QUFFaEU7SUFBQTtJQXFCQSxDQUFDO0lBcEJlLHFCQUFNLEdBQXBCLFVBQXFCLEdBQWdCLEVBQUUsU0FBYztRQUNuRCxRQUFRLEdBQUcsQ0FBQyxJQUFJLEVBQUU7WUFDbEIsS0FBSyxxQkFBVyxDQUFDLGVBQWUsQ0FBQyxDQUFDLE9BQU8sSUFBSSxpQ0FBZSxDQUFDLEdBQTBCLENBQUMsQ0FBQztZQUN6RixLQUFLLHFCQUFXLENBQUMsWUFBWSxDQUFDLENBQUMsT0FBTyxJQUFJLDJCQUFZLENBQUMsR0FBdUIsQ0FBQyxDQUFDO1lBQ2hGLEtBQUsscUJBQVcsQ0FBQyxnQkFBZ0IsQ0FBQyxDQUFDLE9BQU8sSUFBSSxtQ0FBZ0IsQ0FBQyxHQUEyQixDQUFDLENBQUM7WUFDNUYsS0FBSyxxQkFBVyxDQUFDLGVBQWUsQ0FBQyxDQUFDLE9BQU8sSUFBSSxpQ0FBZSxDQUFDLEdBQTBCLENBQUMsQ0FBQztZQUN6RixLQUFLLHFCQUFXLENBQUMsYUFBYSxDQUFDLENBQUMsT0FBTyxJQUFJLDZCQUFhLENBQUMsR0FBd0IsQ0FBQyxDQUFDO1lBQ25GLEtBQUsscUJBQVcsQ0FBQyxXQUFXLENBQUMsQ0FBQyxPQUFPLElBQUkseUJBQVcsQ0FBQyxHQUFzQixDQUFDLENBQUM7WUFDN0UsS0FBSyxxQkFBVyxDQUFDLFlBQVksQ0FBQyxDQUFDLE9BQU8sSUFBSSwyQkFBWSxDQUFDLEdBQXVCLENBQUMsQ0FBQztZQUNoRixLQUFLLHFCQUFXLENBQUMsV0FBVyxDQUFDLENBQUMsT0FBTyxJQUFJLHlCQUFXLENBQUMsR0FBc0IsQ0FBQyxDQUFDO1lBQzdFLEtBQUsscUJBQVcsQ0FBQyxlQUFlLENBQUMsQ0FBQyxPQUFPLElBQUksaUNBQWUsQ0FBQyxHQUEwQixDQUFDLENBQUM7WUFDekYsS0FBSyxxQkFBVyxDQUFDLFdBQVcsQ0FBQyxDQUFDLE9BQU8sSUFBSSx5QkFBVyxDQUFDLEdBQXNCLENBQUMsQ0FBQztZQUM3RSxLQUFLLHFCQUFXLENBQUMsWUFBWSxDQUFDLENBQUMsT0FBTyxJQUFJLDJCQUFZLENBQUMsR0FBdUIsQ0FBQyxDQUFDO1lBQ2hGLEtBQUsscUJBQVcsQ0FBQyxXQUFXLENBQUMsQ0FBQyxPQUFPLElBQUkseUJBQVcsQ0FBQyxHQUFzQixDQUFDLENBQUM7U0FDNUU7UUFDQyxNQUFNLElBQUksS0FBSyxFQUFFLENBQUM7SUFDcEIsQ0FBQztJQUVXLHNCQUFPLEdBQXJCLFVBQXNCLEtBQWMsRUFBRSxTQUFjO0lBQ3BELENBQUM7SUFDSCxxQkFBQztBQUFELENBQUMsQUFyQkQsSUFxQkM7QUFyQlksd0NBQWMifQ==