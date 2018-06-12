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
function __export(m) {
    for (var p in m) if (!exports.hasOwnProperty(p)) exports[p] = m[p];
}
Object.defineProperty(exports, "__esModule", { value: true });
/**
 * \mainpage Box2D API Documentation
 * \section intro_sec Getting Started
 * For documentation please see http://box2d.org/documentation.html
 * For discussion please visit http://box2d.org/forum
 */
// These include files constitute the main Box2D API
__export(require("./Common/b2Settings"));
__export(require("./Common/b2Math"));
__export(require("./Common/b2Draw"));
__export(require("./Common/b2Timer"));
__export(require("./Common/b2GrowableStack"));
__export(require("./Common/b2BlockAllocator"));
__export(require("./Common/b2StackAllocator"));
__export(require("./Collision/b2Collision"));
__export(require("./Collision/b2Distance"));
__export(require("./Collision/b2BroadPhase"));
__export(require("./Collision/b2DynamicTree"));
__export(require("./Collision/b2TimeOfImpact"));
__export(require("./Collision/b2CollideCircle"));
__export(require("./Collision/b2CollidePolygon"));
__export(require("./Collision/b2CollideEdge"));
__export(require("./Collision/Shapes/b2Shape"));
__export(require("./Collision/Shapes/b2CircleShape"));
__export(require("./Collision/Shapes/b2PolygonShape"));
__export(require("./Collision/Shapes/b2EdgeShape"));
__export(require("./Collision/Shapes/b2ChainShape"));
__export(require("./Dynamics/b2Fixture"));
__export(require("./Dynamics/b2Body"));
__export(require("./Dynamics/b2World"));
__export(require("./Dynamics/b2WorldCallbacks"));
__export(require("./Dynamics/b2Island"));
__export(require("./Dynamics/b2TimeStep"));
__export(require("./Dynamics/b2ContactManager"));
__export(require("./Dynamics/Contacts/b2Contact"));
__export(require("./Dynamics/Contacts/b2ContactFactory"));
__export(require("./Dynamics/Contacts/b2ContactSolver"));
__export(require("./Dynamics/Contacts/b2CircleContact"));
__export(require("./Dynamics/Contacts/b2PolygonContact"));
__export(require("./Dynamics/Contacts/b2PolygonAndCircleContact"));
__export(require("./Dynamics/Contacts/b2EdgeAndCircleContact"));
__export(require("./Dynamics/Contacts/b2EdgeAndPolygonContact"));
__export(require("./Dynamics/Contacts/b2ChainAndCircleContact"));
__export(require("./Dynamics/Contacts/b2ChainAndPolygonContact"));
__export(require("./Dynamics/Joints/b2Joint"));
__export(require("./Dynamics/Joints/b2JointFactory"));
__export(require("./Dynamics/Joints/b2AreaJoint"));
__export(require("./Dynamics/Joints/b2DistanceJoint"));
__export(require("./Dynamics/Joints/b2FrictionJoint"));
__export(require("./Dynamics/Joints/b2GearJoint"));
__export(require("./Dynamics/Joints/b2MotorJoint"));
__export(require("./Dynamics/Joints/b2MouseJoint"));
__export(require("./Dynamics/Joints/b2PrismaticJoint"));
__export(require("./Dynamics/Joints/b2PulleyJoint"));
__export(require("./Dynamics/Joints/b2RevoluteJoint"));
__export(require("./Dynamics/Joints/b2RopeJoint"));
__export(require("./Dynamics/Joints/b2WeldJoint"));
__export(require("./Dynamics/Joints/b2WheelJoint"));
// #if B2_ENABLE_CONTROLLER
__export(require("./Controllers/b2Controller"));
__export(require("./Controllers/b2BuoyancyController"));
__export(require("./Controllers/b2ConstantAccelController"));
__export(require("./Controllers/b2ConstantForceController"));
__export(require("./Controllers/b2GravityController"));
__export(require("./Controllers/b2TensorDampingController"));
// #endif
// #if B2_ENABLE_PARTICLE
__export(require("./Particle/b2Particle"));
__export(require("./Particle/b2ParticleGroup"));
__export(require("./Particle/b2ParticleSystem"));
// #endif
__export(require("./Rope/b2Rope"));
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiQm94MkQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi9Cb3gyRC9Cb3gyRC9Cb3gyRC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7O0FBRUY7Ozs7O0dBS0c7QUFFSCxvREFBb0Q7QUFFcEQseUNBQW9DO0FBQ3BDLHFDQUFnQztBQUNoQyxxQ0FBZ0M7QUFDaEMsc0NBQWlDO0FBQ2pDLDhDQUF5QztBQUN6QywrQ0FBMEM7QUFDMUMsK0NBQTBDO0FBRTFDLDZDQUF3QztBQUN4Qyw0Q0FBdUM7QUFDdkMsOENBQXlDO0FBQ3pDLCtDQUEwQztBQUMxQyxnREFBMkM7QUFDM0MsaURBQTRDO0FBQzVDLGtEQUE2QztBQUM3QywrQ0FBMEM7QUFFMUMsZ0RBQTJDO0FBQzNDLHNEQUFpRDtBQUNqRCx1REFBa0Q7QUFDbEQsb0RBQStDO0FBQy9DLHFEQUFnRDtBQUVoRCwwQ0FBcUM7QUFDckMsdUNBQWtDO0FBQ2xDLHdDQUFtQztBQUNuQyxpREFBNEM7QUFDNUMseUNBQW9DO0FBQ3BDLDJDQUFzQztBQUN0QyxpREFBNEM7QUFFNUMsbURBQThDO0FBQzlDLDBEQUFxRDtBQUNyRCx5REFBb0Q7QUFDcEQseURBQW9EO0FBQ3BELDBEQUFxRDtBQUNyRCxtRUFBOEQ7QUFDOUQsZ0VBQTJEO0FBQzNELGlFQUE0RDtBQUM1RCxpRUFBNEQ7QUFDNUQsa0VBQTZEO0FBRTdELCtDQUEwQztBQUMxQyxzREFBaUQ7QUFDakQsbURBQThDO0FBQzlDLHVEQUFrRDtBQUNsRCx1REFBa0Q7QUFDbEQsbURBQThDO0FBQzlDLG9EQUErQztBQUMvQyxvREFBK0M7QUFDL0Msd0RBQW1EO0FBQ25ELHFEQUFnRDtBQUNoRCx1REFBa0Q7QUFDbEQsbURBQThDO0FBQzlDLG1EQUE4QztBQUM5QyxvREFBK0M7QUFFL0MsMkJBQTJCO0FBQzNCLGdEQUEyQztBQUMzQyx3REFBbUQ7QUFDbkQsNkRBQXdEO0FBQ3hELDZEQUF3RDtBQUN4RCx1REFBa0Q7QUFDbEQsNkRBQXdEO0FBQ3hELFNBQVM7QUFFVCx5QkFBeUI7QUFDekIsMkNBQXNDO0FBQ3RDLGdEQUEyQztBQUMzQyxpREFBNEM7QUFDNUMsU0FBUztBQUVULG1DQUE4QiJ9