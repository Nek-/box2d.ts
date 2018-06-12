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
var b2Settings_2 = require("../Common/b2Settings");
var b2Settings_3 = require("../Common/b2Settings");
var b2Settings_4 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2Timer_1 = require("../Common/b2Timer");
var b2ContactSolver_1 = require("./Contacts/b2ContactSolver");
var b2Body_1 = require("./b2Body");
var b2TimeStep_1 = require("./b2TimeStep");
var b2WorldCallbacks_1 = require("./b2WorldCallbacks");
/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/
/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/
/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/
var b2Island = /** @class */ (function () {
    function b2Island() {
        this.m_allocator = null;
        this.m_bodies = [ /*1024*/]; // TODO: b2Settings
        this.m_contacts = [ /*1024*/]; // TODO: b2Settings
        this.m_joints = [ /*1024*/]; // TODO: b2Settings
        this.m_positions = b2TimeStep_1.b2Position.MakeArray(1024); // TODO: b2Settings
        this.m_velocities = b2TimeStep_1.b2Velocity.MakeArray(1024); // TODO: b2Settings
        this.m_bodyCount = 0;
        this.m_jointCount = 0;
        this.m_contactCount = 0;
        this.m_bodyCapacity = 0;
        this.m_contactCapacity = 0;
        this.m_jointCapacity = 0;
    }
    b2Island.prototype.Initialize = function (bodyCapacity, contactCapacity, jointCapacity, allocator, listener) {
        this.m_bodyCapacity = bodyCapacity;
        this.m_contactCapacity = contactCapacity;
        this.m_jointCapacity = jointCapacity;
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;
        this.m_allocator = allocator;
        this.m_listener = listener;
        // TODO:
        // while (this.m_bodies.length < bodyCapacity) {
        //   this.m_bodies[this.m_bodies.length] = null;
        // }
        // TODO:
        // while (this.m_contacts.length < contactCapacity) {
        //   this.m_contacts[this.m_contacts.length] = null;
        // }
        // TODO:
        // while (this.m_joints.length < jointCapacity) {
        //   this.m_joints[this.m_joints.length] = null;
        // }
        // TODO:
        if (this.m_positions.length < bodyCapacity) {
            var new_length = b2Math_1.b2Max(this.m_positions.length * 2, bodyCapacity);
            while (this.m_positions.length < new_length) {
                this.m_positions[this.m_positions.length] = new b2TimeStep_1.b2Position();
            }
        }
        // TODO:
        if (this.m_velocities.length < bodyCapacity) {
            var new_length = b2Math_1.b2Max(this.m_velocities.length * 2, bodyCapacity);
            while (this.m_velocities.length < new_length) {
                this.m_velocities[this.m_velocities.length] = new b2TimeStep_1.b2Velocity();
            }
        }
    };
    b2Island.prototype.Clear = function () {
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;
    };
    b2Island.prototype.AddBody = function (body) {
        // DEBUG: b2Assert(this.m_bodyCount < this.m_bodyCapacity);
        body.m_islandIndex = this.m_bodyCount;
        this.m_bodies[this.m_bodyCount++] = body;
    };
    b2Island.prototype.AddContact = function (contact) {
        // DEBUG: b2Assert(this.m_contactCount < this.m_contactCapacity);
        this.m_contacts[this.m_contactCount++] = contact;
    };
    b2Island.prototype.AddJoint = function (joint) {
        // DEBUG: b2Assert(this.m_jointCount < this.m_jointCapacity);
        this.m_joints[this.m_jointCount++] = joint;
    };
    b2Island.prototype.Solve = function (profile, step, gravity, allowSleep) {
        var timer = b2Island.s_timer.Reset();
        var h = step.dt;
        // Integrate velocities and apply damping. Initialize the body state.
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var b = this.m_bodies[i];
            // const c: b2Vec2 =
            this.m_positions[i].c.Copy(b.m_sweep.c);
            var a = b.m_sweep.a;
            var v = this.m_velocities[i].v.Copy(b.m_linearVelocity);
            var w = b.m_angularVelocity;
            // Store positions for continuous collision.
            b.m_sweep.c0.Copy(b.m_sweep.c);
            b.m_sweep.a0 = b.m_sweep.a;
            if (b.m_type === b2Body_1.b2BodyType.b2_dynamicBody) {
                // Integrate velocities.
                v.x += h * (b.m_gravityScale * gravity.x + b.m_invMass * b.m_force.x);
                v.y += h * (b.m_gravityScale * gravity.y + b.m_invMass * b.m_force.y);
                w += h * b.m_invI * b.m_torque;
                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v.SelfMul(1.0 / (1.0 + h * b.m_linearDamping));
                w *= 1.0 / (1.0 + h * b.m_angularDamping);
            }
            // this.m_positions[i].c = c;
            this.m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            this.m_velocities[i].w = w;
        }
        timer.Reset();
        // Solver data
        var solverData = b2Island.s_solverData;
        solverData.step.Copy(step);
        solverData.positions = this.m_positions;
        solverData.velocities = this.m_velocities;
        // Initialize velocity constraints.
        var contactSolverDef = b2Island.s_contactSolverDef;
        contactSolverDef.step.Copy(step);
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocities;
        contactSolverDef.allocator = this.m_allocator;
        var contactSolver = b2Island.s_contactSolver.Initialize(contactSolverDef);
        contactSolver.InitializeVelocityConstraints();
        if (step.warmStarting) {
            contactSolver.WarmStart();
        }
        for (var i = 0; i < this.m_jointCount; ++i) {
            this.m_joints[i].InitVelocityConstraints(solverData);
        }
        profile.solveInit = timer.GetMilliseconds();
        // Solve velocity constraints.
        timer.Reset();
        for (var i = 0; i < step.velocityIterations; ++i) {
            for (var j = 0; j < this.m_jointCount; ++j) {
                this.m_joints[j].SolveVelocityConstraints(solverData);
            }
            contactSolver.SolveVelocityConstraints();
        }
        // Store impulses for warm starting
        contactSolver.StoreImpulses();
        profile.solveVelocity = timer.GetMilliseconds();
        // Integrate positions.
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var c = this.m_positions[i].c;
            var a = this.m_positions[i].a;
            var v = this.m_velocities[i].v;
            var w = this.m_velocities[i].w;
            // Check for large velocities
            var translation = b2Math_1.b2Vec2.MulSV(h, v, b2Island.s_translation);
            if (b2Math_1.b2Vec2.DotVV(translation, translation) > b2Settings_2.b2_maxTranslationSquared) {
                var ratio = b2Settings_2.b2_maxTranslation / translation.Length();
                v.SelfMul(ratio);
            }
            var rotation = h * w;
            if (rotation * rotation > b2Settings_3.b2_maxRotationSquared) {
                var ratio = b2Settings_3.b2_maxRotation / b2Math_1.b2Abs(rotation);
                w *= ratio;
            }
            // Integrate
            c.x += h * v.x;
            c.y += h * v.y;
            a += h * w;
            // this.m_positions[i].c = c;
            this.m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            this.m_velocities[i].w = w;
        }
        // Solve position constraints
        timer.Reset();
        var positionSolved = false;
        for (var i = 0; i < step.positionIterations; ++i) {
            var contactsOkay = contactSolver.SolvePositionConstraints();
            var jointsOkay = true;
            for (var j = 0; j < this.m_jointCount; ++j) {
                var jointOkay = this.m_joints[j].SolvePositionConstraints(solverData);
                jointsOkay = jointsOkay && jointOkay;
            }
            if (contactsOkay && jointsOkay) {
                // Exit early if the position errors are small.
                positionSolved = true;
                break;
            }
        }
        // Copy state buffers back to the bodies
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var body = this.m_bodies[i];
            body.m_sweep.c.Copy(this.m_positions[i].c);
            body.m_sweep.a = this.m_positions[i].a;
            body.m_linearVelocity.Copy(this.m_velocities[i].v);
            body.m_angularVelocity = this.m_velocities[i].w;
            body.SynchronizeTransform();
        }
        profile.solvePosition = timer.GetMilliseconds();
        this.Report(contactSolver.m_velocityConstraints);
        if (allowSleep) {
            var minSleepTime = b2Settings_1.b2_maxFloat;
            var linTolSqr = b2Settings_4.b2_linearSleepTolerance * b2Settings_4.b2_linearSleepTolerance;
            var angTolSqr = b2Settings_4.b2_angularSleepTolerance * b2Settings_4.b2_angularSleepTolerance;
            for (var i = 0; i < this.m_bodyCount; ++i) {
                var b = this.m_bodies[i];
                if (b.GetType() === b2Body_1.b2BodyType.b2_staticBody) {
                    continue;
                }
                if (!b.m_autoSleepFlag ||
                    b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
                    b2Math_1.b2Vec2.DotVV(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
                    b.m_sleepTime = 0;
                    minSleepTime = 0;
                }
                else {
                    b.m_sleepTime += h;
                    minSleepTime = b2Math_1.b2Min(minSleepTime, b.m_sleepTime);
                }
            }
            if (minSleepTime >= b2Settings_1.b2_timeToSleep && positionSolved) {
                for (var i = 0; i < this.m_bodyCount; ++i) {
                    var b = this.m_bodies[i];
                    b.SetAwake(false);
                }
            }
        }
    };
    b2Island.prototype.SolveTOI = function (subStep, toiIndexA, toiIndexB) {
        // DEBUG: b2Assert(toiIndexA < this.m_bodyCount);
        // DEBUG: b2Assert(toiIndexB < this.m_bodyCount);
        // Initialize the body state.
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var b = this.m_bodies[i];
            this.m_positions[i].c.Copy(b.m_sweep.c);
            this.m_positions[i].a = b.m_sweep.a;
            this.m_velocities[i].v.Copy(b.m_linearVelocity);
            this.m_velocities[i].w = b.m_angularVelocity;
        }
        var contactSolverDef = b2Island.s_contactSolverDef;
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.allocator = this.m_allocator;
        contactSolverDef.step.Copy(subStep);
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocities;
        var contactSolver = b2Island.s_contactSolver.Initialize(contactSolverDef);
        // Solve position constraints.
        for (var i = 0; i < subStep.positionIterations; ++i) {
            var contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
            if (contactsOkay) {
                break;
            }
        }
        /*
        #if 0
          // Is the new position really safe?
          for (int32 i = 0; i < this.m_contactCount; ++i) {
            b2Contact* c = this.m_contacts[i];
            b2Fixture* fA = c.GetFixtureA();
            b2Fixture* fB = c.GetFixtureB();
      
            b2Body* bA = fA.GetBody();
            b2Body* bB = fB.GetBody();
      
            int32 indexA = c.GetChildIndexA();
            int32 indexB = c.GetChildIndexB();
      
            b2DistanceInput input;
            input.proxyA.Set(fA.GetShape(), indexA);
            input.proxyB.Set(fB.GetShape(), indexB);
            input.transformA = bA.GetTransform();
            input.transformB = bB.GetTransform();
            input.useRadii = false;
      
            b2DistanceOutput output;
            b2SimplexCache cache;
            cache.count = 0;
            b2Distance(&output, &cache, &input);
      
            if (output.distance === 0 || cache.count === 3) {
              cache.count += 0;
            }
          }
        #endif
        */
        // Leap of faith to new safe state.
        this.m_bodies[toiIndexA].m_sweep.c0.Copy(this.m_positions[toiIndexA].c);
        this.m_bodies[toiIndexA].m_sweep.a0 = this.m_positions[toiIndexA].a;
        this.m_bodies[toiIndexB].m_sweep.c0.Copy(this.m_positions[toiIndexB].c);
        this.m_bodies[toiIndexB].m_sweep.a0 = this.m_positions[toiIndexB].a;
        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        contactSolver.InitializeVelocityConstraints();
        // Solve velocity constraints.
        for (var i = 0; i < subStep.velocityIterations; ++i) {
            contactSolver.SolveVelocityConstraints();
        }
        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.
        var h = subStep.dt;
        // Integrate positions
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var c = this.m_positions[i].c;
            var a = this.m_positions[i].a;
            var v = this.m_velocities[i].v;
            var w = this.m_velocities[i].w;
            // Check for large velocities
            var translation = b2Math_1.b2Vec2.MulSV(h, v, b2Island.s_translation);
            if (b2Math_1.b2Vec2.DotVV(translation, translation) > b2Settings_2.b2_maxTranslationSquared) {
                var ratio = b2Settings_2.b2_maxTranslation / translation.Length();
                v.SelfMul(ratio);
            }
            var rotation = h * w;
            if (rotation * rotation > b2Settings_3.b2_maxRotationSquared) {
                var ratio = b2Settings_3.b2_maxRotation / b2Math_1.b2Abs(rotation);
                w *= ratio;
            }
            // Integrate
            c.SelfMulAdd(h, v);
            a += h * w;
            // this.m_positions[i].c = c;
            this.m_positions[i].a = a;
            // this.m_velocities[i].v = v;
            this.m_velocities[i].w = w;
            // Sync bodies
            var body = this.m_bodies[i];
            body.m_sweep.c.Copy(c);
            body.m_sweep.a = a;
            body.m_linearVelocity.Copy(v);
            body.m_angularVelocity = w;
            body.SynchronizeTransform();
        }
        this.Report(contactSolver.m_velocityConstraints);
    };
    b2Island.prototype.Report = function (constraints) {
        if (this.m_listener === null) {
            return;
        }
        for (var i = 0; i < this.m_contactCount; ++i) {
            var c = this.m_contacts[i];
            if (!c) {
                continue;
            }
            var vc = constraints[i];
            var impulse = b2Island.s_impulse;
            impulse.count = vc.pointCount;
            for (var j = 0; j < vc.pointCount; ++j) {
                impulse.normalImpulses[j] = vc.points[j].normalImpulse;
                impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
            }
            this.m_listener.PostSolve(c, impulse);
        }
    };
    b2Island.s_timer = new b2Timer_1.b2Timer();
    b2Island.s_solverData = new b2TimeStep_1.b2SolverData();
    b2Island.s_contactSolverDef = new b2ContactSolver_1.b2ContactSolverDef();
    b2Island.s_contactSolver = new b2ContactSolver_1.b2ContactSolver();
    b2Island.s_translation = new b2Math_1.b2Vec2();
    b2Island.s_impulse = new b2WorldCallbacks_1.b2ContactImpulse();
    return b2Island;
}());
exports.b2Island = b2Island;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJJc2xhbmQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9iMklzbGFuZC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBRUYsMERBQTBEO0FBQzFELG1EQUFtRTtBQUNuRSxtREFBbUY7QUFDbkYsbURBQTZFO0FBQzdFLG1EQUF5RjtBQUN6RiwyQ0FBK0Q7QUFDL0QsNkNBQTRDO0FBRTVDLDhEQUFpRjtBQUdqRixtQ0FBOEM7QUFDOUMsMkNBQTJGO0FBQzNGLHVEQUF5RTtBQUV6RTs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7RUE2RUU7QUFFRjs7Ozs7Ozs7Ozs7Ozs7RUFjRTtBQUVGOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7RUFxQkU7QUFFRjtJQUFBO1FBQ1MsZ0JBQVcsR0FBUSxJQUFJLENBQUM7UUFHeEIsYUFBUSxHQUFhLEVBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxtQkFBbUI7UUFDcEQsZUFBVSxHQUFnQixFQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsbUJBQW1CO1FBQ3pELGFBQVEsR0FBYyxFQUFDLFFBQVEsQ0FBQyxDQUFDLENBQUMsbUJBQW1CO1FBRXJELGdCQUFXLEdBQWlCLHVCQUFVLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUMsbUJBQW1CO1FBQzNFLGlCQUFZLEdBQWlCLHVCQUFVLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxDQUFDLENBQUMsbUJBQW1CO1FBRTVFLGdCQUFXLEdBQVcsQ0FBQyxDQUFDO1FBQ3hCLGlCQUFZLEdBQVcsQ0FBQyxDQUFDO1FBQ3pCLG1CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBRTNCLG1CQUFjLEdBQVcsQ0FBQyxDQUFDO1FBQzNCLHNCQUFpQixHQUFXLENBQUMsQ0FBQztRQUM5QixvQkFB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