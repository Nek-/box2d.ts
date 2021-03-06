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
System.register(["Testbed/Framework/Test", "./AddPair", "./ApplyForce", "./BodyTypes", "./Breakable", "./Bridge", "./BulletTest", "./Cantilever", "./Car", "./ContinuousTest", "./Chain", "./CharacterCollision", "./CollisionFiltering", "./CollisionProcessing", "./CompoundShapes", "./Confined", "./ConvexHull", "./ConveyorBelt", "./DistanceTest", "./Dominos", "./DumpShell", "./DynamicTreeTest", "./EdgeShapes", "./EdgeTest", "./Gears", "./Mobile", "./MobileBalanced", "./MotorJoint", "./OneSidedPlatform", "./Pinball", "./PolyCollision", "./PolyShapes", "./Prismatic", "./Pulleys", "./Pyramid", "./RayCast", "./Revolute", "./RopeJoint", "./SensorTest", "./ShapeEditing", "./SliderCrank", "./SphereStack", "./TheoJansen", "./Tiles", "./TimeOfImpact", "./Tumbler", "./VaryingFriction", "./VaryingRestitution", "./VerticalStack", "./Web", "./Rope", "./MotorJoint2", "./BlobTest", "./TestCCD", "./TestRagdoll", "./TestStack", "./BasicSliderCrank", "./PyramidTopple", "./DominoTower", "./HeavyOnLight", "./HeavyOnLightTwo", "./TopdownCar", "./BuoyancyTest", "./Sandbox", "./Sparky", "./DamBreak", "./LiquidTimer", "./WaveMachine", "./Particles", "./Faucet", "./DrawingParticles", "./Soup", "./ParticlesSurfaceTension", "./ElasticParticles", "./RigidParticles", "./MultipleParticleSystems", "./Impulse", "./SoupStirrer", "./Fracker", "./Maxwell", "./Ramp", "./Pointy", "./AntiPointy", "./CornerCase", "./ParticleCollisionFilter", "./EyeCandy"], function (exports_1, context_1) {
    "use strict";
    var Test_1, AddPair_1, ApplyForce_1, BodyTypes_1, Breakable_1, Bridge_1, BulletTest_1, Cantilever_1, Car_1, ContinuousTest_1, Chain_1, CharacterCollision_1, CollisionFiltering_1, CollisionProcessing_1, CompoundShapes_1, Confined_1, ConvexHull_1, ConveyorBelt_1, DistanceTest_1, Dominos_1, DumpShell_1, DynamicTreeTest_1, EdgeShapes_1, EdgeTest_1, Gears_1, Mobile_1, MobileBalanced_1, MotorJoint_1, OneSidedPlatform_1, Pinball_1, PolyCollision_1, PolyShapes_1, Prismatic_1, Pulleys_1, Pyramid_1, RayCast_1, Revolute_1, RopeJoint_1, SensorTest_1, ShapeEditing_1, SliderCrank_1, SphereStack_1, TheoJansen_1, Tiles_1, TimeOfImpact_1, Tumbler_1, VaryingFriction_1, VaryingRestitution_1, VerticalStack_1, Web_1, Rope_1, MotorJoint2_1, BlobTest_1, TestCCD_1, TestRagdoll_1, TestStack_1, BasicSliderCrank_1, PyramidTopple_1, DominoTower_1, HeavyOnLight_1, HeavyOnLightTwo_1, TopdownCar_1, BuoyancyTest_1, Sandbox_1, Sparky_1, DamBreak_1, LiquidTimer_1, WaveMachine_1, Particles_1, Faucet_1, DrawingParticles_1, Soup_1, ParticlesSurfaceTension_1, ElasticParticles_1, RigidParticles_1, MultipleParticleSystems_1, Impulse_1, SoupStirrer_1, Fracker_1, Maxwell_1, Ramp_1, Pointy_1, AntiPointy_1, CornerCase_1, ParticleCollisionFilter_1, EyeCandy_1, g_testEntries;
    var __moduleName = context_1 && context_1.id;
    return {
        setters: [
            function (Test_1_1) {
                Test_1 = Test_1_1;
            },
            function (AddPair_1_1) {
                AddPair_1 = AddPair_1_1;
            },
            function (ApplyForce_1_1) {
                ApplyForce_1 = ApplyForce_1_1;
            },
            function (BodyTypes_1_1) {
                BodyTypes_1 = BodyTypes_1_1;
            },
            function (Breakable_1_1) {
                Breakable_1 = Breakable_1_1;
            },
            function (Bridge_1_1) {
                Bridge_1 = Bridge_1_1;
            },
            function (BulletTest_1_1) {
                BulletTest_1 = BulletTest_1_1;
            },
            function (Cantilever_1_1) {
                Cantilever_1 = Cantilever_1_1;
            },
            function (Car_1_1) {
                Car_1 = Car_1_1;
            },
            function (ContinuousTest_1_1) {
                ContinuousTest_1 = ContinuousTest_1_1;
            },
            function (Chain_1_1) {
                Chain_1 = Chain_1_1;
            },
            function (CharacterCollision_1_1) {
                CharacterCollision_1 = CharacterCollision_1_1;
            },
            function (CollisionFiltering_1_1) {
                CollisionFiltering_1 = CollisionFiltering_1_1;
            },
            function (CollisionProcessing_1_1) {
                CollisionProcessing_1 = CollisionProcessing_1_1;
            },
            function (CompoundShapes_1_1) {
                CompoundShapes_1 = CompoundShapes_1_1;
            },
            function (Confined_1_1) {
                Confined_1 = Confined_1_1;
            },
            function (ConvexHull_1_1) {
                ConvexHull_1 = ConvexHull_1_1;
            },
            function (ConveyorBelt_1_1) {
                ConveyorBelt_1 = ConveyorBelt_1_1;
            },
            function (DistanceTest_1_1) {
                DistanceTest_1 = DistanceTest_1_1;
            },
            function (Dominos_1_1) {
                Dominos_1 = Dominos_1_1;
            },
            function (DumpShell_1_1) {
                DumpShell_1 = DumpShell_1_1;
            },
            function (DynamicTreeTest_1_1) {
                DynamicTreeTest_1 = DynamicTreeTest_1_1;
            },
            function (EdgeShapes_1_1) {
                EdgeShapes_1 = EdgeShapes_1_1;
            },
            function (EdgeTest_1_1) {
                EdgeTest_1 = EdgeTest_1_1;
            },
            function (Gears_1_1) {
                Gears_1 = Gears_1_1;
            },
            function (Mobile_1_1) {
                Mobile_1 = Mobile_1_1;
            },
            function (MobileBalanced_1_1) {
                MobileBalanced_1 = MobileBalanced_1_1;
            },
            function (MotorJoint_1_1) {
                MotorJoint_1 = MotorJoint_1_1;
            },
            function (OneSidedPlatform_1_1) {
                OneSidedPlatform_1 = OneSidedPlatform_1_1;
            },
            function (Pinball_1_1) {
                Pinball_1 = Pinball_1_1;
            },
            function (PolyCollision_1_1) {
                PolyCollision_1 = PolyCollision_1_1;
            },
            function (PolyShapes_1_1) {
                PolyShapes_1 = PolyShapes_1_1;
            },
            function (Prismatic_1_1) {
                Prismatic_1 = Prismatic_1_1;
            },
            function (Pulleys_1_1) {
                Pulleys_1 = Pulleys_1_1;
            },
            function (Pyramid_1_1) {
                Pyramid_1 = Pyramid_1_1;
            },
            function (RayCast_1_1) {
                RayCast_1 = RayCast_1_1;
            },
            function (Revolute_1_1) {
                Revolute_1 = Revolute_1_1;
            },
            function (RopeJoint_1_1) {
                RopeJoint_1 = RopeJoint_1_1;
            },
            function (SensorTest_1_1) {
                SensorTest_1 = SensorTest_1_1;
            },
            function (ShapeEditing_1_1) {
                ShapeEditing_1 = ShapeEditing_1_1;
            },
            function (SliderCrank_1_1) {
                SliderCrank_1 = SliderCrank_1_1;
            },
            function (SphereStack_1_1) {
                SphereStack_1 = SphereStack_1_1;
            },
            function (TheoJansen_1_1) {
                TheoJansen_1 = TheoJansen_1_1;
            },
            function (Tiles_1_1) {
                Tiles_1 = Tiles_1_1;
            },
            function (TimeOfImpact_1_1) {
                TimeOfImpact_1 = TimeOfImpact_1_1;
            },
            function (Tumbler_1_1) {
                Tumbler_1 = Tumbler_1_1;
            },
            function (VaryingFriction_1_1) {
                VaryingFriction_1 = VaryingFriction_1_1;
            },
            function (VaryingRestitution_1_1) {
                VaryingRestitution_1 = VaryingRestitution_1_1;
            },
            function (VerticalStack_1_1) {
                VerticalStack_1 = VerticalStack_1_1;
            },
            function (Web_1_1) {
                Web_1 = Web_1_1;
            },
            function (Rope_1_1) {
                Rope_1 = Rope_1_1;
            },
            function (MotorJoint2_1_1) {
                MotorJoint2_1 = MotorJoint2_1_1;
            },
            function (BlobTest_1_1) {
                BlobTest_1 = BlobTest_1_1;
            },
            function (TestCCD_1_1) {
                TestCCD_1 = TestCCD_1_1;
            },
            function (TestRagdoll_1_1) {
                TestRagdoll_1 = TestRagdoll_1_1;
            },
            function (TestStack_1_1) {
                TestStack_1 = TestStack_1_1;
            },
            function (BasicSliderCrank_1_1) {
                BasicSliderCrank_1 = BasicSliderCrank_1_1;
            },
            function (PyramidTopple_1_1) {
                PyramidTopple_1 = PyramidTopple_1_1;
            },
            function (DominoTower_1_1) {
                DominoTower_1 = DominoTower_1_1;
            },
            function (HeavyOnLight_1_1) {
                HeavyOnLight_1 = HeavyOnLight_1_1;
            },
            function (HeavyOnLightTwo_1_1) {
                HeavyOnLightTwo_1 = HeavyOnLightTwo_1_1;
            },
            function (TopdownCar_1_1) {
                TopdownCar_1 = TopdownCar_1_1;
            },
            function (BuoyancyTest_1_1) {
                BuoyancyTest_1 = BuoyancyTest_1_1;
            },
            function (Sandbox_1_1) {
                Sandbox_1 = Sandbox_1_1;
            },
            function (Sparky_1_1) {
                Sparky_1 = Sparky_1_1;
            },
            function (DamBreak_1_1) {
                DamBreak_1 = DamBreak_1_1;
            },
            function (LiquidTimer_1_1) {
                LiquidTimer_1 = LiquidTimer_1_1;
            },
            function (WaveMachine_1_1) {
                WaveMachine_1 = WaveMachine_1_1;
            },
            function (Particles_1_1) {
                Particles_1 = Particles_1_1;
            },
            function (Faucet_1_1) {
                Faucet_1 = Faucet_1_1;
            },
            function (DrawingParticles_1_1) {
                DrawingParticles_1 = DrawingParticles_1_1;
            },
            function (Soup_1_1) {
                Soup_1 = Soup_1_1;
            },
            function (ParticlesSurfaceTension_1_1) {
                ParticlesSurfaceTension_1 = ParticlesSurfaceTension_1_1;
            },
            function (ElasticParticles_1_1) {
                ElasticParticles_1 = ElasticParticles_1_1;
            },
            function (RigidParticles_1_1) {
                RigidParticles_1 = RigidParticles_1_1;
            },
            function (MultipleParticleSystems_1_1) {
                MultipleParticleSystems_1 = MultipleParticleSystems_1_1;
            },
            function (Impulse_1_1) {
                Impulse_1 = Impulse_1_1;
            },
            function (SoupStirrer_1_1) {
                SoupStirrer_1 = SoupStirrer_1_1;
            },
            function (Fracker_1_1) {
                Fracker_1 = Fracker_1_1;
            },
            function (Maxwell_1_1) {
                Maxwell_1 = Maxwell_1_1;
            },
            function (Ramp_1_1) {
                Ramp_1 = Ramp_1_1;
            },
            function (Pointy_1_1) {
                Pointy_1 = Pointy_1_1;
            },
            function (AntiPointy_1_1) {
                AntiPointy_1 = AntiPointy_1_1;
            },
            function (CornerCase_1_1) {
                CornerCase_1 = CornerCase_1_1;
            },
            function (ParticleCollisionFilter_1_1) {
                ParticleCollisionFilter_1 = ParticleCollisionFilter_1_1;
            },
            function (EyeCandy_1_1) {
                EyeCandy_1 = EyeCandy_1_1;
            }
        ],
        execute: function () {
            // #endif
            exports_1("g_testEntries", g_testEntries = [
                // #if B2_ENABLE_PARTICLE
                new Test_1.TestEntry("Sparky", Sparky_1.Sparky.Create),
                // #endif
                new Test_1.TestEntry("Character Collision", CharacterCollision_1.CharacterCollision.Create),
                new Test_1.TestEntry("Tiles", Tiles_1.Tiles.Create),
                new Test_1.TestEntry("Heavy on Light", HeavyOnLight_1.HeavyOnLight.Create),
                new Test_1.TestEntry("Heavy on Light Two", HeavyOnLightTwo_1.HeavyOnLightTwo.Create),
                new Test_1.TestEntry("Vertical Stack", VerticalStack_1.VerticalStack.Create),
                new Test_1.TestEntry("Basic Slider Crank", BasicSliderCrank_1.BasicSliderCrank.Create),
                new Test_1.TestEntry("Slider Crank", SliderCrank_1.SliderCrank.Create),
                new Test_1.TestEntry("Sphere Stack", SphereStack_1.SphereStack.Create),
                new Test_1.TestEntry("Convex Hull", ConvexHull_1.ConvexHull.Create),
                new Test_1.TestEntry("Tumbler", Tumbler_1.Tumbler.Create),
                new Test_1.TestEntry("Ray-Cast", RayCast_1.RayCast.Create),
                new Test_1.TestEntry("Dump Shell", DumpShell_1.DumpShell.Create),
                new Test_1.TestEntry("Apply Force", ApplyForce_1.ApplyForce.Create),
                new Test_1.TestEntry("Continuous Test", ContinuousTest_1.ContinuousTest.Create),
                new Test_1.TestEntry("Time of Impact", TimeOfImpact_1.TimeOfImpact.Create),
                new Test_1.TestEntry("Motor Joint", MotorJoint_1.MotorJoint.Create),
                new Test_1.TestEntry("One-Sided Platform", OneSidedPlatform_1.OneSidedPlatform.Create),
                new Test_1.TestEntry("Mobile", Mobile_1.Mobile.Create),
                new Test_1.TestEntry("MobileBalanced", MobileBalanced_1.MobileBalanced.Create),
                new Test_1.TestEntry("Conveyor Belt", ConveyorBelt_1.ConveyorBelt.Create),
                new Test_1.TestEntry("Gears", Gears_1.Gears.Create),
                new Test_1.TestEntry("Varying Restitution", VaryingRestitution_1.VaryingRestitution.Create),
                new Test_1.TestEntry("Cantilever", Cantilever_1.Cantilever.Create),
                new Test_1.TestEntry("Edge Test", EdgeTest_1.EdgeTest.Create),
                new Test_1.TestEntry("Body Types", BodyTypes_1.BodyTypes.Create),
                new Test_1.TestEntry("Shape Editing", ShapeEditing_1.ShapeEditing.Create),
                new Test_1.TestEntry("Car", Car_1.Car.Create),
                new Test_1.TestEntry("Prismatic", Prismatic_1.Prismatic.Create),
                new Test_1.TestEntry("Revolute", Revolute_1.Revolute.Create),
                new Test_1.TestEntry("Pulleys", Pulleys_1.Pulleys.Create),
                new Test_1.TestEntry("Polygon Shapes", PolyShapes_1.PolyShapes.Create),
                new Test_1.TestEntry("Web", Web_1.Web.Create),
                new Test_1.TestEntry("RopeJoint", RopeJoint_1.RopeJoint.Create),
                new Test_1.TestEntry("Pinball", Pinball_1.Pinball.Create),
                new Test_1.TestEntry("Bullet Test", BulletTest_1.BulletTest.Create),
                new Test_1.TestEntry("Confined", Confined_1.Confined.Create),
                new Test_1.TestEntry("Pyramid", Pyramid_1.Pyramid.Create),
                new Test_1.TestEntry("Theo Jansen's Walker", TheoJansen_1.TheoJansen.Create),
                new Test_1.TestEntry("Edge Shapes", EdgeShapes_1.EdgeShapes.Create),
                new Test_1.TestEntry("PolyCollision", PolyCollision_1.PolyCollision.Create),
                new Test_1.TestEntry("Bridge", Bridge_1.Bridge.Create),
                new Test_1.TestEntry("Breakable", Breakable_1.Breakable.Create),
                new Test_1.TestEntry("Chain", Chain_1.Chain.Create),
                new Test_1.TestEntry("Collision Filtering", CollisionFiltering_1.CollisionFiltering.Create),
                new Test_1.TestEntry("Collision Processing", CollisionProcessing_1.CollisionProcessing.Create),
                new Test_1.TestEntry("Compound Shapes", CompoundShapes_1.CompoundShapes.Create),
                new Test_1.TestEntry("Distance Test", DistanceTest_1.DistanceTest.Create),
                new Test_1.TestEntry("Dominos", Dominos_1.Dominos.Create),
                new Test_1.TestEntry("Dynamic Tree", DynamicTreeTest_1.DynamicTreeTest.Create),
                new Test_1.TestEntry("Sensor Test", SensorTest_1.SensorTest.Create),
                new Test_1.TestEntry("Varying Friction", VaryingFriction_1.VaryingFriction.Create),
                new Test_1.TestEntry("Add Pair Stress Test", AddPair_1.AddPair.Create),
                new Test_1.TestEntry("Rope", Rope_1.Rope.Create),
                new Test_1.TestEntry("Motor Joint (Bug #487)", MotorJoint2_1.MotorJoint2.Create),
                new Test_1.TestEntry("Blob Test", BlobTest_1.BlobTest.Create),
                new Test_1.TestEntry("Continuous Collision", TestCCD_1.TestCCD.Create),
                new Test_1.TestEntry("Ragdolls", TestRagdoll_1.TestRagdoll.Create),
                new Test_1.TestEntry("Stacked Boxes", TestStack_1.TestStack.Create),
                new Test_1.TestEntry("Basic Slider Crank", BasicSliderCrank_1.BasicSliderCrank.Create),
                new Test_1.TestEntry("Pyramid Topple", PyramidTopple_1.PyramidTopple.Create),
                new Test_1.TestEntry("Domino Tower", DominoTower_1.DominoTower.Create),
                new Test_1.TestEntry("Heavy on Light", HeavyOnLight_1.HeavyOnLight.Create),
                new Test_1.TestEntry("Heavy on Light 2", HeavyOnLightTwo_1.HeavyOnLightTwo.Create),
                new Test_1.TestEntry("TopDown Car", TopdownCar_1.TopdownCar.Create),
                // #if B2_ENABLE_CONTROLLER
                new Test_1.TestEntry("Buoyancy Test", BuoyancyTest_1.BuoyancyTest.Create),
                // #endif
                // #if B2_ENABLE_PARTICLE
                new Test_1.TestEntry("Sandbox", Sandbox_1.Sandbox.Create),
                // new TestEntry("Sparky", Sparky.Create),
                new Test_1.TestEntry("DamBreak", DamBreak_1.DamBreak.Create),
                new Test_1.TestEntry("Liquid Timer", LiquidTimer_1.LiquidTimer.Create),
                new Test_1.TestEntry("Wave Machine", WaveMachine_1.WaveMachine.Create),
                new Test_1.TestEntry("Particles", Particles_1.Particles.Create),
                new Test_1.TestEntry("Faucet", Faucet_1.Faucet.Create),
                new Test_1.TestEntry("Particle Drawing", DrawingParticles_1.DrawingParticles.Create),
                new Test_1.TestEntry("Soup", Soup_1.Soup.Create),
                new Test_1.TestEntry("Surface Tension", ParticlesSurfaceTension_1.ParticlesSurfaceTension.Create),
                new Test_1.TestEntry("Elastic Particles", ElasticParticles_1.ElasticParticles.Create),
                new Test_1.TestEntry("Rigid Particles", RigidParticles_1.RigidParticles.Create),
                new Test_1.TestEntry("Multiple Systems", MultipleParticleSystems_1.MultipleParticleSystems.Create),
                new Test_1.TestEntry("Impulse", Impulse_1.Impulse.Create),
                new Test_1.TestEntry("Soup Stirrer", SoupStirrer_1.SoupStirrer.Create),
                new Test_1.TestEntry("Fracker", Fracker_1.Fracker.Create),
                new Test_1.TestEntry("Maxwell", Maxwell_1.Maxwell.Create),
                new Test_1.TestEntry("Ramp", Ramp_1.Ramp.Create),
                new Test_1.TestEntry("Pointy", Pointy_1.Pointy.Create),
                new Test_1.TestEntry("AntiPointy", AntiPointy_1.AntiPointy.Create),
                new Test_1.TestEntry("Corner Case", CornerCase_1.CornerCase.Create),
                new Test_1.TestEntry("Particle Collisions", ParticleCollisionFilter_1.ParticleCollisionFilter.Create),
                new Test_1.TestEntry("Eye Candy", EyeCandy_1.EyeCandy.Create),
            ]);
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiVGVzdEVudHJpZXMuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyJUZXN0RW50cmllcy50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7O1lBZ0dGLFNBQVM7WUFFVCwyQkFBYSxhQUFhLEdBQWdCO2dCQUN4Qyx5QkFBeUI7Z0JBQ3pCLElBQUksZ0JBQVMsQ0FBQyxRQUFRLEVBQUUsZUFBTSxDQUFDLE1BQU0sQ0FBQztnQkFDdEMsU0FBUztnQkFFVCxJQUFJLGdCQUFTLENBQUMscUJBQXFCLEVBQUUsdUNBQWtCLENBQUMsTUFBTSxDQUFDO2dCQUMvRCxJQUFJLGdCQUFTLENBQUMsT0FBTyxFQUFFLGFBQUssQ0FBQyxNQUFNLENBQUM7Z0JBQ3BDLElBQUksZ0JBQVMsQ0FBQyxnQkFBZ0IsRUFBRSwyQkFBWSxDQUFDLE1BQU0sQ0FBQztnQkFDcEQsSUFBSSxnQkFBUyxDQUFDLG9CQUFvQixFQUFFLGlDQUFlLENBQUMsTUFBTSxDQUFDO2dCQUMzRCxJQUFJLGdCQUFTLENBQUMsZ0JBQWdCLEVBQUUsNkJBQWEsQ0FBQyxNQUFNLENBQUM7Z0JBQ3JELElBQUksZ0JBQVMsQ0FBQyxvQkFBb0IsRUFBRSxtQ0FBZ0IsQ0FBQyxNQUFNLENBQUM7Z0JBQzVELElBQUksZ0JBQVMsQ0FBQyxjQUFjLEVBQUUseUJBQVcsQ0FBQyxNQUFNLENBQUM7Z0JB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