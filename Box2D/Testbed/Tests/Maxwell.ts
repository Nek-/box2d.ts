/*
 * Copyright (c) 2014 Google, Inc.
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

// #if B2_ENABLE_PARTICLE

import * as box2d from "../../Box2D/Box2D";
import * as testbed from "../Testbed";

/**
 * Game which adds some fun to Maxwell's demon.
 *
 * http://en.wikipedia.org/wiki/Maxwell's_demon
 *
 * The user's goal is to try to catch as many particles as
 * possible in the bottom half of the container by splitting the
 * container using a barrier with the 'a' key.
 *
 * See Maxwell::Keyboard() for other controls.
 */

export class Maxwell extends testbed.Test {
  m_density = 0.0;
  m_position = 0.0;
  m_temperature = 0.0;
  m_barrierBody: box2d.b2Body;
  m_particleGroup: box2d.b2ParticleGroup;

  static readonly k_containerWidth = 2.0;
  static readonly k_containerHeight = 4.0;
  static readonly k_containerHalfWidth = Maxwell.k_containerWidth / 2.0;
  static readonly k_containerHalfHeight = Maxwell.k_containerHeight / 2.0;
  static readonly k_barrierHeight = Maxwell.k_containerHalfHeight / 100.0;
  static readonly k_barrierMovementIncrement = Maxwell.k_containerHalfHeight * 0.1;
  static readonly k_densityStep = 1.25;
  static readonly k_densityMin = 0.01;
  static readonly k_densityMax = 0.8;
  static readonly k_densityDefault = 0.25;
  static readonly k_temperatureStep = 0.2;
  static readonly k_temperatureMin = 0.4;
  static readonly k_temperatureMax = 10.0;
  static readonly k_temperatureDefault = 5.0;

  constructor() {
    super();

    this.m_density = Maxwell.k_densityDefault;
    this.m_position = Maxwell.k_containerHalfHeight;
    this.m_particleGroup = null;
    this.m_temperature = Maxwell.k_temperatureDefault;
    this.m_barrierBody = null;

    this.m_world.SetGravity(new box2d.b2Vec2(0, 0));

    // Create the container.
    {
      var bd = new box2d.b2BodyDef();
      var ground = this.m_world.CreateBody(bd);
      var shape = new box2d.b2ChainShape();
      var vertices = [
        new box2d.b2Vec2(-Maxwell.k_containerHalfWidth, 0),
        new box2d.b2Vec2(Maxwell.k_containerHalfWidth, 0),
        new box2d.b2Vec2(Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight),
        new box2d.b2Vec2(-Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight)
      ];
      shape.CreateLoop(vertices, 4);
      var def = new box2d.b2FixtureDef();
      def.shape = shape;
      def.density = 0;
      def.restitution = 1.0;
      ground.CreateFixture(def);
    }

    // Enable the barrier.
    this.EnableBarrier();
    // Create the particles.
    this.ResetParticles();
  }

  /**
   * Disable the barrier.
   */
  DisableBarrier() {
    if (this.m_barrierBody) {
      this.m_world.DestroyBody(this.m_barrierBody);
      this.m_barrierBody = null;
    }
  }

  /**
   * Enable the barrier.
   */
  EnableBarrier() {
    if (!this.m_barrierBody) {
      var bd = new box2d.b2BodyDef();
      this.m_barrierBody = this.m_world.CreateBody(bd);
      var barrierShape = new box2d.b2PolygonShape();
      barrierShape.SetAsBox(Maxwell.k_containerHalfWidth, Maxwell.k_barrierHeight,
        new box2d.b2Vec2(0, this.m_position), 0);
      var def = new box2d.b2FixtureDef();
      def.shape = barrierShape;
      def.density = 0;
      def.restitution = 1.0;
      this.m_barrierBody.CreateFixture(def);
    }
  }

  /**
   * Enable / disable the barrier.
   */
  ToggleBarrier() {
    if (this.m_barrierBody) {
      this.DisableBarrier();
    } else {
      this.EnableBarrier();
    }
  }

  /**
   * Destroy and recreate all particles.
   */
  ResetParticles() {
    if (this.m_particleGroup !== null) {
      this.m_particleGroup.DestroyParticles(false);
      this.m_particleGroup = null;
    }

    this.m_particleSystem.SetRadius(Maxwell.k_containerHalfWidth / 20.0); {
      var shape = new box2d.b2PolygonShape();
      shape.SetAsBox(this.m_density * Maxwell.k_containerHalfWidth,
        this.m_density * Maxwell.k_containerHalfHeight,
        new box2d.b2Vec2(0, Maxwell.k_containerHalfHeight), 0);
      var pd = new box2d.b2ParticleGroupDef();
      pd.flags = box2d.b2ParticleFlag.b2_powderParticle;
      pd.shape = shape;
      this.m_particleGroup = this.m_particleSystem.CreateParticleGroup(pd);
      ///  b2Vec2* velocities =
      ///    this.m_particleSystem.GetVelocityBuffer() +
      ///    this.m_particleGroup.GetBufferIndex();
      var velocities = this.m_particleSystem.GetVelocityBuffer();
      var index = this.m_particleGroup.GetBufferIndex();

      for (var i = 0; i < this.m_particleGroup.GetParticleCount(); ++i) {
        ///  b2Vec2& v = *(velocities + i);
        var v = velocities[index + i];
        v.Set(testbed.RandomFloat() + 1.0, testbed.RandomFloat() + 1.0);
        v.Normalize();
        ///  v *= this.m_temperature;
        v.SelfMul(this.m_temperature);
      }
    }
  }

  Keyboard(key: string) {
    switch (key) {
      case "a":
        // Enable / disable the barrier.
        this.ToggleBarrier();
        break;
      case "=":
        // Increase the particle density.
        this.m_density = box2d.b2Min(this.m_density * Maxwell.k_densityStep, Maxwell.k_densityMax);
        this.Reset();
        break;
      case "-":
        // Reduce the particle density.
        this.m_density = box2d.b2Max(this.m_density / Maxwell.k_densityStep, Maxwell.k_densityMin);
        this.Reset();
        break;
      case ".":
        // Move the location of the divider up.
        this.MoveDivider(this.m_position + Maxwell.k_barrierMovementIncrement);
        break;
      case ",":
        // Move the location of the divider down.
        this.MoveDivider(this.m_position - Maxwell.k_barrierMovementIncrement);
        break;
      case ";":
        // Reduce the temperature (velocity of particles).
        this.m_temperature = box2d.b2Max(this.m_temperature - Maxwell.k_temperatureStep,
          Maxwell.k_temperatureMin);
        this.Reset();
        break;
      case "'":
        // Increase the temperature (velocity of particles).
        this.m_temperature = box2d.b2Min(this.m_temperature + Maxwell.k_temperatureStep,
          Maxwell.k_temperatureMax);
        this.Reset();
        break;
      default:
        super.Keyboard(key);
        break;
    }
  }

  /**
   * Determine whether a point is in the container.
   */
  InContainer(p: box2d.b2Vec2) {
    return p.x >= -Maxwell.k_containerHalfWidth && p.x <= Maxwell.k_containerHalfWidth &&
      p.y >= 0.0 && p.y <= Maxwell.k_containerHalfHeight * 2.0;
  }

  MouseDown(p: box2d.b2Vec2) {
    if (!this.InContainer(p)) {
      super.MouseDown(p);
    }
  }

  MouseUp(p: box2d.b2Vec2) {
    // If the pointer is in the container.
    if (this.InContainer(p)) {
      // Enable / disable the barrier.
      this.ToggleBarrier();
    } else {
      // Move the barrier to the touch position.
      this.MoveDivider(p.y);

      super.MouseUp(p);
    }
  }

  Step(settings: testbed.Settings) {
    super.Step(settings);

    // Number of particles above (top) and below (bottom) the barrier.
    var top = 0;
    var bottom = 0;
    var index = this.m_particleGroup.GetBufferIndex();
    ///  b2Vec2* const velocities = this.m_particleSystem.GetVelocityBuffer() + index;
    var velocities = this.m_particleSystem.GetVelocityBuffer();
    ///  b2Vec2* const positions = this.m_particleSystem.GetPositionBuffer() + index;
    var positions = this.m_particleSystem.GetPositionBuffer();

    for (var i = 0; i < this.m_particleGroup.GetParticleCount(); i++) {
      // Add energy to particles based upon the temperature.
      ///  b2Vec2& v = velocities[i];
      var v = velocities[index + i];
      v.Normalize();
      ///  v *= this.m_temperature;
      v.SelfMul(this.m_temperature);

      // Keep track of the number of particles above / below the
      // divider / barrier position.
      ///  b2Vec2& p = positions[i];
      var p = positions[index + i];
      if (p.y > this.m_position)
        top++;
      else
        bottom++;
    }

    // Calculate a score based upon the difference in pressure between the
    // upper and lower divisions of the container.
    var topPressure = top / (Maxwell.k_containerHeight - this.m_position);
    var botPressure = bottom / this.m_position;
    testbed.g_debugDraw.DrawString(
      10, 75, `Score: ${topPressure > 0.0 ? botPressure / topPressure - 1.0 : 0.0}`);
  }

  /**
   * Reset the particles and the barrier.
   */
  Reset() {
    this.DisableBarrier();
    this.ResetParticles();
    this.EnableBarrier();
  }

  /**
   * Move the divider / barrier.
   */
  MoveDivider(newPosition: number) {
    this.m_position = box2d.b2Clamp(newPosition, Maxwell.k_barrierMovementIncrement,
      Maxwell.k_containerHeight - Maxwell.k_barrierMovementIncrement);
    this.Reset();
  }

  GetDefaultViewZoom() {
    return 0.1;
  }
  
  static Create() {
    return new Maxwell();
  }
}

// #endif

// //#if B2_ENABLE_PARTICLE

// goog.provide('box2d.Testbed.Maxwell');

// goog.require('box2d.Testbed.Test');

// /**
//  * Game which adds some fun to Maxwell's demon.
//  *
//  * http://en.wikipedia.org/wiki/Maxwell's_demon
//  *
//  * The user's goal is to try to catch as many particles as
//  * possible in the bottom half of the container by splitting the
//  * container using a barrier with the 'a' key.
//  *
//  * See Maxwell::Keyboard() for other controls.
//  *
//  * @export
//  * @constructor
//  * @extends {box2d.Testbed.Test}
//  * @param {HTMLCanvasElement} canvas
//  * @param {box2d.Testbed.Settings} settings
//  */
// box2d.Testbed.Maxwell(canvas, settings) {
//   box2d.Testbed.Test.call(this, canvas, settings); // base class constructor
//   this.m_density = Maxwell.k_densityDefault;
//   this.m_position = Maxwell.k_containerHalfHeight;
//   this.m_particleGroup = null;
//   this.m_temperature = Maxwell.k_temperatureDefault;
//   this.m_barrierBody = null;

//   this.m_world.SetGravity(new box2d.b2Vec2(0, 0));

//   // Create the container.
//   {
//     var bd = new box2d.b2BodyDef();
//     var ground = this.m_world.CreateBody(bd);
//     var shape = new box2d.b2ChainShape();
//     var vertices = [
//       new box2d.b2Vec2(-Maxwell.k_containerHalfWidth, 0),
//       new box2d.b2Vec2(Maxwell.k_containerHalfWidth, 0),
//       new box2d.b2Vec2(Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight),
//       new box2d.b2Vec2(-Maxwell.k_containerHalfWidth, Maxwell.k_containerHeight)
//     ];
//     shape.CreateLoop(vertices, 4);
//     var def = new box2d.b2FixtureDef();
//     def.shape = shape;
//     def.density = 0;
//     def.restitution = 1.0;
//     ground.CreateFixture(def);
//   }

//   // Enable the barrier.
//   this.EnableBarrier();
//   // Create the particles.
//   this.ResetParticles();
// }

// goog.inherits(box2d.Testbed.Maxwell, box2d.Testbed.Test);

// /**
//  * @type {number}
//  */
// m_density = 0.0;
// /**
//  * @type {number}
//  */
// m_position = 0.0;
// /**
//  * @type {number}
//  */
// m_temperature = 0.0;
// /**
//  * @type {box2d.b2Body}
//  */
// m_barrierBody = null;
// /**
//  * @type {box2d.b2ParticleGroup}
//  */
// m_particleGroup = null;

// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_containerWidth = 2.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_containerHeight = 4.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_containerHalfWidth = Maxwell.k_containerWidth / 2.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_containerHalfHeight = Maxwell.k_containerHeight / 2.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_barrierHeight = Maxwell.k_containerHalfHeight / 100.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_barrierMovementIncrement = Maxwell.k_containerHalfHeight * 0.1;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_densityStep = 1.25;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_densityMin = 0.01;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_densityMax = 0.8;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_densityDefault = 0.25;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_temperatureStep = 0.2;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_temperatureMin = 0.4;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_temperatureMax = 10.0;
// /**
//  * @const
//  * @type {number}
//  */
// Maxwell.k_temperatureDefault = 5.0;


// /**
//  * @export
//  * @return {box2d.Testbed.Test}
//  * @param {HTMLCanvasElement} canvas
//  * @param {box2d.Testbed.Settings} settings
//  */
// box2d.Testbed.Maxwell.Create(canvas, settings) {
//   return new box2d.Testbed.Maxwell(canvas, settings);
// }

// //#endif
