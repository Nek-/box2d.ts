"use strict";
var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Joint_1 = require("./b2Joint");
var b2DistanceJoint_1 = require("./b2DistanceJoint");
var b2AreaJointDef = /** @class */ (function (_super) {
    __extends(b2AreaJointDef, _super);
    function b2AreaJointDef() {
        var _this = _super.call(this, b2Joint_1.b2JointType.e_areaJoint) || this;
        _this.bodies = [];
        _this.frequencyHz = 0;
        _this.dampingRatio = 0;
        return _this;
    }
    b2AreaJointDef.prototype.AddBody = function (body) {
        this.bodies.push(body);
        if (this.bodies.length === 1) {
            this.bodyA = body;
        }
        else if (this.bodies.length === 2) {
            this.bodyB = body;
        }
    };
    return b2AreaJointDef;
}(b2Joint_1.b2JointDef));
exports.b2AreaJointDef = b2AreaJointDef;
var b2AreaJoint = /** @class */ (function (_super) {
    __extends(b2AreaJoint, _super);
    function b2AreaJoint(def) {
        var _this = _super.call(this, def) || this;
        _this.m_frequencyHz = 0;
        _this.m_dampingRatio = 0;
        // Solver shared
        _this.m_impulse = 0;
        _this.m_targetArea = 0;
        // DEBUG: b2Assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");
        _this.m_bodies = def.bodies;
        _this.m_frequencyHz = b2Settings_1.b2Maybe(def.frequencyHz, 0);
        _this.m_dampingRatio = b2Settings_1.b2Maybe(def.dampingRatio, 0);
        _this.m_targetLengths = b2Settings_1.b2MakeNumberArray(def.bodies.length);
        _this.m_normals = b2Math_1.b2Vec2.MakeArray(def.bodies.length);
        _this.m_joints = []; // b2MakeNullArray(def.bodies.length);
        _this.m_deltas = b2Math_1.b2Vec2.MakeArray(def.bodies.length);
        _this.m_delta = new b2Math_1.b2Vec2();
        var djd = new b2DistanceJoint_1.b2DistanceJointDef();
        djd.frequencyHz = _this.m_frequencyHz;
        djd.dampingRatio = _this.m_dampingRatio;
        _this.m_targetArea = 0;
        for (var i = 0; i < _this.m_bodies.length; ++i) {
            var body = _this.m_bodies[i];
            var next = _this.m_bodies[(i + 1) % _this.m_bodies.length];
            var body_c = body.GetWorldCenter();
            var next_c = next.GetWorldCenter();
            _this.m_targetLengths[i] = b2Math_1.b2Vec2.DistanceVV(body_c, next_c);
            _this.m_targetArea += b2Math_1.b2Vec2.CrossVV(body_c, next_c);
            djd.Initialize(body, next, body_c, next_c);
            _this.m_joints[i] = body.GetWorld().CreateJoint(djd);
        }
        _this.m_targetArea *= 0.5;
        return _this;
    }
    b2AreaJoint.prototype.GetAnchorA = function (out) {
        return out;
    };
    b2AreaJoint.prototype.GetAnchorB = function (out) {
        return out;
    };
    b2AreaJoint.prototype.GetReactionForce = function (inv_dt, out) {
        return out;
    };
    b2AreaJoint.prototype.GetReactionTorque = function (inv_dt) {
        return 0;
    };
    b2AreaJoint.prototype.SetFrequency = function (hz) {
        this.m_frequencyHz = hz;
        for (var i = 0; i < this.m_joints.length; ++i) {
            this.m_joints[i].SetFrequency(hz);
        }
    };
    b2AreaJoint.prototype.GetFrequency = function () {
        return this.m_frequencyHz;
    };
    b2AreaJoint.prototype.SetDampingRatio = function (ratio) {
        this.m_dampingRatio = ratio;
        for (var i = 0; i < this.m_joints.length; ++i) {
            this.m_joints[i].SetDampingRatio(ratio);
        }
    };
    b2AreaJoint.prototype.GetDampingRatio = function () {
        return this.m_dampingRatio;
    };
    b2AreaJoint.prototype.Dump = function (log) {
        log("Area joint dumping is not supported.\n");
    };
    b2AreaJoint.prototype.InitVelocityConstraints = function (data) {
        for (var i = 0; i < this.m_bodies.length; ++i) {
            var prev = this.m_bodies[(i + this.m_bodies.length - 1) % this.m_bodies.length];
            var next = this.m_bodies[(i + 1) % this.m_bodies.length];
            var prev_c = data.positions[prev.m_islandIndex].c;
            var next_c = data.positions[next.m_islandIndex].c;
            var delta = this.m_deltas[i];
            b2Math_1.b2Vec2.SubVV(next_c, prev_c, delta);
        }
        if (data.step.warmStarting) {
            this.m_impulse *= data.step.dtRatio;
            for (var i = 0; i < this.m_bodies.length; ++i) {
                var body = this.m_bodies[i];
                var body_v = data.velocities[body.m_islandIndex].v;
                var delta = this.m_deltas[i];
                body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse;
                body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse;
            }
        }
        else {
            this.m_impulse = 0;
        }
    };
    b2AreaJoint.prototype.SolveVelocityConstraints = function (data) {
        var dotMassSum = 0;
        var crossMassSum = 0;
        for (var i = 0; i < this.m_bodies.length; ++i) {
            var body = this.m_bodies[i];
            var body_v = data.velocities[body.m_islandIndex].v;
            var delta = this.m_deltas[i];
            dotMassSum += delta.LengthSquared() / body.GetMass();
            crossMassSum += b2Math_1.b2Vec2.CrossVV(body_v, delta);
        }
        var lambda = -2 * crossMassSum / dotMassSum;
        // lambda = b2Clamp(lambda, -b2_maxLinearCorrection, b2_maxLinearCorrection);
        this.m_impulse += lambda;
        for (var i = 0; i < this.m_bodies.length; ++i) {
            var body = this.m_bodies[i];
            var body_v = data.velocities[body.m_islandIndex].v;
            var delta = this.m_deltas[i];
            body_v.x += body.m_invMass * delta.y * 0.5 * lambda;
            body_v.y += body.m_invMass * -delta.x * 0.5 * lambda;
        }
    };
    b2AreaJoint.prototype.SolvePositionConstraints = function (data) {
        var perimeter = 0;
        var area = 0;
        for (var i = 0; i < this.m_bodies.length; ++i) {
            var body = this.m_bodies[i];
            var next = this.m_bodies[(i + 1) % this.m_bodies.length];
            var body_c = data.positions[body.m_islandIndex].c;
            var next_c = data.positions[next.m_islandIndex].c;
            var delta = b2Math_1.b2Vec2.SubVV(next_c, body_c, this.m_delta);
            var dist = delta.Length();
            if (dist < b2Settings_1.b2_epsilon) {
                dist = 1;
            }
            this.m_normals[i].x = delta.y / dist;
            this.m_normals[i].y = -delta.x / dist;
            perimeter += dist;
            area += b2Math_1.b2Vec2.CrossVV(body_c, next_c);
        }
        area *= 0.5;
        var deltaArea = this.m_targetArea - area;
        var toExtrude = 0.5 * deltaArea / perimeter;
        var done = true;
        for (var i = 0; i < this.m_bodies.length; ++i) {
            var body = this.m_bodies[i];
            var body_c = data.positions[body.m_islandIndex].c;
            var next_i = (i + 1) % this.m_bodies.length;
            var delta = b2Math_1.b2Vec2.AddVV(this.m_normals[i], this.m_normals[next_i], this.m_delta);
            delta.SelfMul(toExtrude);
            var norm_sq = delta.LengthSquared();
            if (norm_sq > b2Math_1.b2Sq(b2Settings_1.b2_maxLinearCorrection)) {
                delta.SelfMul(b2Settings_1.b2_maxLinearCorrection / b2Math_1.b2Sqrt(norm_sq));
            }
            if (norm_sq > b2Math_1.b2Sq(b2Settings_1.b2_linearSlop)) {
                done = false;
            }
            body_c.x += delta.x;
            body_c.y += delta.y;
        }
        return done;
    };
    return b2AreaJoint;
}(b2Joint_1.b2Joint));
exports.b2AreaJoint = b2AreaJoint;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJBcmVhSm9pbnQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Kb2ludHMvYjJBcmVhSm9pbnQudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7Ozs7O0FBQUEsNkRBQTZEO0FBQzdELHNEQUF3SDtBQUN4SCw4Q0FBK0Q7QUFDL0QscUNBQTBFO0FBQzFFLHFEQUF3RTtBQWN4RTtJQUFvQyxrQ0FBVTtJQU81QztRQUFBLFlBQ0Usa0JBQU0scUJBQVcsQ0FBQyxXQUFXLENBQUMsU0FDL0I7UUFSTSxZQUFNLEdBQWEsRUFBRSxDQUFDO1FBRXRCLGlCQUFXLEdBQVcsQ0FBQyxDQUFDO1FBRXhCLGtCQUFZLEdBQVcsQ0FBQyxDQUFDOztJQUloQyxDQUFDO0lBRU0sZ0NBQU8sR0FBZCxVQUFlLElBQVk7UUFDekIsSUFBSSxDQUFDLE1BQU0sQ0FBQyxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7UUFFdkIsSUFBSSxJQUFJLENBQUMsTUFBTSxDQUFDLE1BQU0sS0FBSyxDQUFDLEVBQUU7WUFDNUIsSUFBSSxDQUFDLEtBQUssR0FBRyxJQUFJLENBQUM7U0FDbkI7YUFBTSxJQUFJLElBQUksQ0FBQyxNQUFNLENBQUMsTUFBTSxLQUFLLENBQUMsRUFBRTtZQUNuQyxJQUFJLENBQUMsS0FBSyxHQUFHLElBQUksQ0FBQztTQUNuQjtJQUNILENBQUM7SUFDSCxxQkFBQztBQUFELENBQUMsQUFwQkQsQ0FBb0Msb0JBQVUsR0FvQjdDO0FBcEJZLHdDQUFjO0FBc0IzQjtJQUFpQywrQkFBTztJQWdCdEMscUJBQVksR0FBb0I7UUFBaEMsWUFDRSxrQkFBTSxHQUFHLENBQUMsU0FvQ1g7UUFuRE0sbUJBQWEsR0FBVyxDQUFDLENBQUM7UUFDMUIsb0JBQWMsR0FBVyxDQUFDLENBQUM7UUFFbEMsZ0JBQWdCO1FBQ1QsZUFBUyxHQUFXLENBQUMsQ0FBQztRQUl0QixrQkFBWSxHQUFXLENBQUMsQ0FBQztR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QU0sTUFBTSxHQUFXLElBQUksQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLGFBQWEsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUM1RCxJQUFNLE1BQU0sR0FBVyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxhQUFhLENBQUMsQ0FBQyxDQUFDLENBQUM7WUFDNUQsSUFBTSxLQUFLLEdBQVcsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsQ0FBQztZQUV2QyxlQUFNLENBQUMsS0FBSyxDQUFDLE1BQU0sRUFBRSxNQUFNLEVBQUUsS0FBSyxDQUFDLENBQUM7U0FDckM7UUFFRCxJQUFJLElBQUksQ0FBQyxJQUFJLENBQUMsWUFBWSxFQUFFO1lBQzFCLElBQUksQ0FBQyxTQUFTLElBQUksSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUM7WUFFcEMsS0FBSyxJQUFJLENBQUMsR0FBVyxDQUFDLEVBQUUsQ0FBQyxHQUFHLElBQUksQ0FBQyxRQUFRLENBQUMsTUFBTSxFQUFFLEVBQUUsQ0FBQyxFQUFFO2dCQUNyRCxJQUFNLElBQUksR0FBVyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDO2dCQUN0QyxJQUFNLE1BQU0sR0FBVyxJQUFJLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxhQUFhLENBQUMsQ0FBQyxDQUFDLENBQUM7Z0JBQzdELElBQU0sS0FBSyxHQUFXLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQyxDQUFDLENBQUM7Z0JBRXZDLE1BQU0sQ0FBQyxDQUFDLElBQUksSUFBSSxDQUFDLFNBQVMsR0FBSSxLQUFLLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDO2dCQUM3RCxNQUFNLENBQUMsQ0FBQyxJQUFJLElBQUksQ0FBQyxTQUFTLEdBQUcsQ0FBQyxLQUFLLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxJQUFJLENBQUMsU0FBUyxDQUFDO2FBQzlEO1NBQ0Y7YUFBTTtZQUNMLElBQUksQ0FBQyxTQUFTLEdBQUcsQ0FBQyxDQUFDO1NBQ3BC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