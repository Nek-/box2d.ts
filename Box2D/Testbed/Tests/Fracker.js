/*
 * Copyright (c) 2014 Google, Inc
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
System.register(["../../Box2D/Box2D", "../Testbed"], function (exports_1, context_1) {
    "use strict";
    var box2d, testbed, EmitterTracker, ParticleGroupTracker, FrackerSettings, Fracker;
    var __moduleName = context_1 && context_1.id;
    return {
        setters: [
            function (box2d_1) {
                box2d = box2d_1;
            },
            function (testbed_1) {
                testbed = testbed_1;
            }
        ],
        execute: function () {
            /**
             * Tracks instances of RadialEmitter and destroys them after a
             * specified period of time.
             */
            EmitterTracker = class EmitterTracker {
                constructor() {
                    this.m_emitterLifetime = [];
                }
                /**
                 * Delete all emitters.
                 */
                __dtor__() {
                    ///  for (std.map<RadialEmitter*, float32>.const_iterator it = m_emitterLifetime.begin(); it !== m_emitterLifetime.end(); ++it)
                    for (let it = 0; it < this.m_emitterLifetime.length; ++it) {
                        ///  delete it.first;
                        this.m_emitterLifetime[it].emitter.__dtor__();
                    }
                }
                /**
                 * Add an emitter to the tracker.
                 * This assumes emitter was allocated using "new" and ownership
                 * of the object is handed to this class.
                 */
                Add(emitter, lifetime) {
                    ///  m_emitterLifetime[emitter] = lifetime;
                    this.m_emitterLifetime.push({
                        emitter: emitter,
                        lifetime: lifetime
                    });
                }
                /**
                 * Update all emitters destroying those who are too old.
                 */
                Step(dt) {
                    ///  std.vector<RadialEmitter*> emittersToDestroy;
                    const emittersToDestroy = [];
                    ///  for (std.map<RadialEmitter*, float32>.const_iterator it = m_emitterLifetime.begin(); it !== m_emitterLifetime.end(); ++it)
                    for (let it = 0; it < this.m_emitterLifetime.length; ++it) {
                        ///  RadialEmitter * const emitter = it.first;
                        const emitter = this.m_emitterLifetime[it].emitter;
                        ///  const float32 lifetime = it.second - dt;
                        const lifetime = this.m_emitterLifetime[it].lifetime - dt;
                        if (lifetime <= 0.0) {
                            emittersToDestroy.push(emitter);
                        }
                        ///  m_emitterLifetime[emitter] = lifetime;
                        this.m_emitterLifetime[it].lifetime = lifetime;
                        emitter.Step(dt);
                    }
                    ///  for (std.vector<RadialEmitter*>.const_iterator it = emittersToDestroy.begin(); it !== emittersToDestroy.end(); ++it)
                    for (let it = 0; it < emittersToDestroy.length; ++it) {
                        ///  RadialEmitter *emitter = *it;
                        const emitter = emittersToDestroy[it];
                        /// delete emitter;
                        emitter.__dtor__();
                        ///  m_emitterLifetime.erase(m_emitterLifetime.find(emitter));
                        this.m_emitterLifetime = this.m_emitterLifetime.filter(function (value) {
                            return value.emitter !== emitter;
                        });
                    }
                }
            };
            exports_1("EmitterTracker", EmitterTracker);
            /**
             * Keep track of particle groups in a set, removing them when
             * they're destroyed.
             */
            ParticleGroupTracker = class ParticleGroupTracker extends box2d.b2DestructionListener {
                constructor() {
                    super(...arguments);
                    this.m_particleGroups = [];
                }
                /**
                 * Called when any particle group is about to be destroyed.
                 */
                SayGoodbyeParticleGroup(group) {
                    this.RemoveParticleGroup(group);
                }
                /**
                 * Add a particle group to the tracker.
                 */
                AddParticleGroup(group) {
                    this.m_particleGroups.push(group);
                }
                /**
                 * Remove a particle group from the tracker.
                 */
                RemoveParticleGroup(group) {
                    this.m_particleGroups.splice(this.m_particleGroups.indexOf(group), 1);
                }
                GetParticleGroups() {
                    return this.m_particleGroups;
                }
            };
            exports_1("ParticleGroupTracker", ParticleGroupTracker);
            FrackerSettings = class FrackerSettings {
            };
            /**
             * Width and height of the world in tiles.
             */
            FrackerSettings.k_worldWidthTiles = 24;
            FrackerSettings.k_worldHeightTiles = 16;
            /**
             * Total number of tiles.
             */
            FrackerSettings.k_worldTiles = FrackerSettings.k_worldWidthTiles * FrackerSettings.k_worldHeightTiles;
            /**
             * Center of the world in world coordinates.
             */
            FrackerSettings.k_worldCenterX = 0.0;
            FrackerSettings.k_worldCenterY = 2.0;
            /**
             * Size of each tile in world units.
             */
            FrackerSettings.k_tileWidth = 0.2;
            FrackerSettings.k_tileHeight = 0.2;
            /**
             * Half width and height of tiles in world units.
             */
            FrackerSettings.k_tileHalfWidth = FrackerSettings.k_tileWidth * 0.5;
            FrackerSettings.k_tileHalfHeight = FrackerSettings.k_tileHeight * 0.5;
            /**
             * Half width and height of the world in world coordinates.
             */
            FrackerSettings.k_worldHalfWidth = (FrackerSettings.k_worldWidthTiles * FrackerSettings.k_tileWidth) * 0.5;
            FrackerSettings.k_worldHalfHeight = (FrackerSettings.k_worldHeightTiles * FrackerSettings.k_tileHeight) * 0.5;
            /**
             * Colors of tiles.
             */
            FrackerSettings.k_playerColor = new box2d.b2Color(1.0, 1.0, 1.0);
            FrackerSettings.k_playerFrackColor = new box2d.b2Color(1.0, 0.5, 0.5);
            FrackerSettings.k_wellColor = new box2d.b2Color(0.5, 0.5, 0.5);
            FrackerSettings.k_oilColor = new box2d.b2Color(1.0, 0.0, 0.0);
            FrackerSettings.k_waterColor = new box2d.b2Color(0.0, 0.2, 1.0);
            FrackerSettings.k_frackingFluidColor = new box2d.b2Color(0.8, 0.4, 0.0);
            /**
             * Default density of each body.
             */
            FrackerSettings.k_density = 0.1;
            /**
             * Radius of oil / water / fracking fluid particles.
             */
            FrackerSettings.k_particleRadius = ((FrackerSettings.k_tileWidth + FrackerSettings.k_tileHeight) * 0.5) * 0.2;
            /**
             * Probability (0..100%) of generating each tile (must sum to
             * 1.0).
             */
            FrackerSettings.k_dirtProbability = 80;
            FrackerSettings.k_emptyProbability = 10;
            FrackerSettings.k_oilProbability = 7;
            FrackerSettings.k_waterProbability = 3;
            /**
             * Lifetime of a fracking fluid emitter in seconds.
             */
            FrackerSettings.k_frackingFluidEmitterLifetime = 5.0;
            /**
             * Speed particles are sucked up the well.
             */
            FrackerSettings.k_wellSuckSpeedInside = FrackerSettings.k_tileHeight * 5.0;
            /**
             * Speed particle are sucket towards the well bottom.
             */
            FrackerSettings.k_wellSuckSpeedOutside = FrackerSettings.k_tileWidth * 1.0;
            /**
             * Time mouse button must be held before emitting fracking
             * fluid.
             */
            FrackerSettings.k_frackingFluidChargeTime = 1.0;
            /**
             * Scores.
             */
            FrackerSettings.k_scorePerOilParticle = 1;
            FrackerSettings.k_scorePerWaterParticle = -1;
            FrackerSettings.k_scorePerFrackingParticle = 0;
            FrackerSettings.k_scorePerFrackingDeployment = -10;
            exports_1("FrackerSettings", FrackerSettings);
            /**
             * Oil Fracking simulator.
             *
             * Dig down to move the oil (red) to the well (gray). Try not to
             * contaminate the ground water (blue). To deploy fracking fluid
             * press 'space'.  Fracking fluid can be used to push other
             * fluids to the well head and ultimately score points.
             */
            Fracker = class Fracker extends testbed.Test {
                constructor() {
                    super();
                    this.m_wellX = FrackerSettings.k_worldWidthTiles - (FrackerSettings.k_worldWidthTiles / 4);
                    this.m_wellTop = FrackerSettings.k_worldHeightTiles - 1;
                    this.m_wellBottom = FrackerSettings.k_worldHeightTiles / 8;
                    this.m_tracker = new EmitterTracker();
                    this.m_allowInput = false;
                    this.m_frackingFluidChargeTime = -1.0;
                    this.m_material = [];
                    this.m_bodies = [];
                    /**
                     * Set of particle groups the well has influence over.
                     */
                    this.m_listener = new Fracker.DestructionListener();
                    this.m_listener.Initialize(this.m_world);
                    this.m_particleSystem.SetRadius(FrackerSettings.k_particleRadius);
                    this.InitializeLayout();
                    // Create the boundaries of the play area.
                    this.CreateGround();
                    // Create the well.
                    this.CreateWell();
                    // Create the geography / features (tiles of the world).
                    this.CreateGeo();
                    // Create the player.
                    this.CreatePlayer();
                }
                __dtor__() {
                    this.m_listener.__dtor__();
                }
                /**
                 * Initialize the data structures used to track the material in
                 * each tile and the bodies associated with each tile.
                 */
                InitializeLayout() {
                    for (let i = 0; i < FrackerSettings.k_worldTiles; ++i) {
                        this.m_material[i] = Fracker.Material.EMPTY;
                        this.m_bodies[i] = null;
                    }
                }
                /**
                 * Get the material of the tile at the specified tile position.
                 */
                GetMaterial(x, y) {
                    ///  return *const_cast<Fracker*>(this).GetMaterialStorage(x, y);
                    return this.m_material[Fracker.TileToArrayOffset(x, y)];
                }
                /**
                 * Set the material of the tile at the specified tile position.
                 */
                SetMaterial(x, y, material) {
                    ///  *GetMaterialStorage(x, y) = material;
                    this.m_material[Fracker.TileToArrayOffset(x, y)] = material;
                }
                /**
                 * Get the body associated with the specified tile position.
                 */
                GetBody(x, y) {
                    ///  return *const_cast<Fracker*>(this).GetBodyStorage(x, y);
                    return this.m_bodies[Fracker.TileToArrayOffset(x, y)];
                }
                /**
                 * Set the body associated with the specified tile position.
                 */
                SetBody(x, y, body) {
                    ///  b2Body** const currentBody = GetBodyStorage(x, y);
                    const currentBody = this.m_bodies[Fracker.TileToArrayOffset(x, y)];
                    if (currentBody) {
                        this.m_world.DestroyBody(currentBody);
                    }
                    this.m_bodies[Fracker.TileToArrayOffset(x, y)] = body;
                }
                /**
                 * Create the player.
                 */
                CreatePlayer() {
                    const bd = new box2d.b2BodyDef();
                    bd.type = box2d.b2BodyType.b2_kinematicBody;
                    this.m_player = this.m_world.CreateBody(bd);
                    const shape = new box2d.b2PolygonShape();
                    shape.SetAsBox(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight, new box2d.b2Vec2(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight), 0);
                    this.m_player.CreateFixture(shape, FrackerSettings.k_density);
                    this.m_player.SetTransformVec(Fracker.TileToWorld(FrackerSettings.k_worldWidthTiles / 2, FrackerSettings.k_worldHeightTiles / 2), 0);
                }
                /**
                 * Create the geography / features of the world.
                 */
                CreateGeo() {
                    box2d.b2Assert(FrackerSettings.k_dirtProbability +
                        FrackerSettings.k_emptyProbability +
                        FrackerSettings.k_oilProbability +
                        FrackerSettings.k_waterProbability === 100);
                    for (let x = 0; x < FrackerSettings.k_worldWidthTiles; x++) {
                        for (let y = 0; y < FrackerSettings.k_worldHeightTiles; y++) {
                            if (this.GetMaterial(x, y) !== Fracker.Material.EMPTY) {
                                continue;
                            }
                            // Choose a tile at random.
                            const chance = Math.random() * 100.0;
                            // Create dirt if this is the bottom row or chance dictates it.
                            if (chance < FrackerSettings.k_dirtProbability || y === 0) {
                                this.CreateDirtBlock(x, y);
                            }
                            else if (chance < FrackerSettings.k_dirtProbability +
                                FrackerSettings.k_emptyProbability) {
                                this.SetMaterial(x, y, Fracker.Material.EMPTY);
                            }
                            else if (chance < FrackerSettings.k_dirtProbability +
                                FrackerSettings.k_emptyProbability +
                                FrackerSettings.k_oilProbability) {
                                this.CreateReservoirBlock(x, y, Fracker.Material.OIL);
                            }
                            else {
                                this.CreateReservoirBlock(x, y, Fracker.Material.WATER);
                            }
                        }
                    }
                }
                /**
                 * Create the boundary of the world.
                 */
                CreateGround() {
                    const bd = new box2d.b2BodyDef();
                    const ground = this.m_world.CreateBody(bd);
                    const shape = new box2d.b2ChainShape();
                    const bottomLeft = new box2d.b2Vec2(), topRight = new box2d.b2Vec2();
                    Fracker.GetExtents(bottomLeft, topRight);
                    const vertices = [
                        new box2d.b2Vec2(bottomLeft.x, bottomLeft.y),
                        new box2d.b2Vec2(topRight.x, bottomLeft.y),
                        new box2d.b2Vec2(topRight.x, topRight.y),
                        new box2d.b2Vec2(bottomLeft.x, topRight.y)
                    ];
                    shape.CreateLoop(vertices, 4);
                    ground.CreateFixture(shape, 0.0);
                }
                /**
                 * Create a dirt block at the specified world position.
                 */
                CreateDirtBlock(x, y) {
                    const position = Fracker.TileToWorld(x, y);
                    const bd = new box2d.b2BodyDef();
                    const body = this.m_world.CreateBody(bd);
                    const shape = new box2d.b2PolygonShape();
                    shape.SetAsBox(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight, Fracker.CenteredPosition(position), 0);
                    body.CreateFixture(shape, FrackerSettings.k_density);
                    this.SetBody(x, y, body);
                    this.SetMaterial(x, y, Fracker.Material.DIRT);
                }
                /**
                 * Create particles in a tile with resources.
                 */
                CreateReservoirBlock(x, y, material) {
                    const position = Fracker.TileToWorld(x, y);
                    const shape = new box2d.b2PolygonShape();
                    this.SetMaterial(x, y, material);
                    shape.SetAsBox(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight, Fracker.CenteredPosition(position), 0);
                    const pd = new box2d.b2ParticleGroupDef();
                    pd.flags = box2d.b2ParticleFlag.b2_tensileParticle | box2d.b2ParticleFlag.b2_viscousParticle | box2d.b2ParticleFlag.b2_destructionListenerParticle;
                    pd.shape = shape;
                    pd.color.Copy(material === Fracker.Material.OIL ?
                        FrackerSettings.k_oilColor : FrackerSettings.k_waterColor);
                    const group = this.m_particleSystem.CreateParticleGroup(pd);
                    this.m_listener.AddParticleGroup(group);
                    // Tag each particle with its type.
                    const particleCount = group.GetParticleCount();
                    ///  void** const userDataBuffer = m_particleSystem.GetUserDataBuffer() + group.GetBufferIndex();;
                    const userDataBuffer = this.m_particleSystem.GetUserDataBuffer();
                    const index = group.GetBufferIndex();
                    for (let i = 0; i < particleCount; ++i) {
                        ///  userDataBuffer[i] = GetMaterialStorage(x, y);
                        userDataBuffer[index + i] = this.m_material[Fracker.TileToArrayOffset(x, y)];
                    }
                    // Keep track of the total available oil.
                    if (material === Fracker.Material.OIL) {
                        this.m_listener.AddOil(particleCount);
                    }
                }
                /**
                 * Create a well and the region which applies negative pressure
                 * to suck out fluid.
                 */
                CreateWell() {
                    for (let y = this.m_wellBottom; y <= this.m_wellTop; y++) {
                        this.SetMaterial(this.m_wellX, y, Fracker.Material.WELL);
                    }
                }
                /**
                 * Create a fracking fluid emitter.
                 */
                CreateFrackingFluidEmitter(position) {
                    const groupDef = new box2d.b2ParticleGroupDef();
                    const group = this.m_particleSystem.CreateParticleGroup(groupDef);
                    this.m_listener.AddParticleGroup(group);
                    const emitter = new testbed.RadialEmitter();
                    emitter.SetGroup(group);
                    emitter.SetParticleSystem(this.m_particleSystem);
                    emitter.SetPosition(Fracker.CenteredPosition(position));
                    emitter.SetVelocity(new box2d.b2Vec2(0.0, -FrackerSettings.k_tileHalfHeight));
                    emitter.SetSpeed(FrackerSettings.k_tileHalfWidth * 0.1);
                    emitter.SetSize(new box2d.b2Vec2(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight));
                    emitter.SetEmitRate(20.0);
                    emitter.SetColor(FrackerSettings.k_frackingFluidColor);
                    emitter.SetParticleFlags(box2d.b2ParticleFlag.b2_tensileParticle | box2d.b2ParticleFlag.b2_viscousParticle);
                    this.m_tracker.Add(emitter, FrackerSettings.k_frackingFluidEmitterLifetime);
                    this.m_listener.AddScore(FrackerSettings.k_scorePerFrackingDeployment);
                }
                /**
                 * Update the player's position.
                 */
                SetPlayerPosition(playerX, playerY) {
                    const playerPosition = this.m_player.GetTransform().p;
                    const currentPlayerX = [0], currentPlayerY = [0];
                    Fracker.WorldToTile(playerPosition, currentPlayerX, currentPlayerY);
                    playerX = box2d.b2Clamp(playerX, 0, FrackerSettings.k_worldWidthTiles - 1);
                    playerY = box2d.b2Clamp(playerY, 0, FrackerSettings.k_worldHeightTiles - 1);
                    // Only update if the player has moved and isn't attempting to
                    // move through the well.
                    if (this.GetMaterial(playerX, playerY) !== Fracker.Material.WELL &&
                        (currentPlayerX[0] !== playerX ||
                            currentPlayerY[0] !== playerY)) {
                        // Try to deploy any fracking fluid that was charging.
                        this.DeployFrackingFluid();
                        // Move the player.
                        this.m_player.SetTransformVec(Fracker.TileToWorld(playerX, playerY), 0);
                    }
                }
                /**
                 * Try to deploy fracking fluid at the player's position,
                 * returning true if successful.
                 */
                DeployFrackingFluid() {
                    let deployed = false;
                    const playerPosition = this.m_player.GetTransform().p;
                    if (this.m_frackingFluidChargeTime > FrackerSettings.k_frackingFluidChargeTime) {
                        this.CreateFrackingFluidEmitter(playerPosition);
                        deployed = true;
                    }
                    this.m_frackingFluidChargeTime = -1.0;
                    return deployed;
                }
                /**
                 * Destroy all particles in the box specified by a set of tile
                 * coordinates.
                 */
                DestroyParticlesInTiles(startX, startY, endX, endY) {
                    const shape = new box2d.b2PolygonShape();
                    const width = endX - startX + 1;
                    const height = endY - startY + 1;
                    const centerX = startX + width / 2;
                    const centerY = startY + height / 2;
                    shape.SetAsBox(FrackerSettings.k_tileHalfWidth * width, FrackerSettings.k_tileHalfHeight * height);
                    const killLocation = new box2d.b2Transform();
                    killLocation.SetPositionAngle(Fracker.CenteredPosition(Fracker.TileToWorld(centerX, centerY)), 0);
                    this.m_particleSystem.DestroyParticlesInShape(shape, killLocation);
                }
                JointDestroyed(joint) {
                    super.JointDestroyed(joint);
                }
                ParticleGroupDestroyed(group) {
                    super.ParticleGroupDestroyed(group);
                }
                BeginContact(contact) {
                    super.BeginContact(contact);
                }
                EndContact(contact) {
                    super.EndContact(contact);
                }
                PreSolve(contact, oldManifold) {
                    super.PreSolve(contact, oldManifold);
                }
                PostSolve(contact, impulse) {
                    super.PostSolve(contact, impulse);
                }
                /**
                 * a = left, d = right, a = up, s = down, e = deploy fracking
                 * fluid.
                 */
                Keyboard(key) {
                    // Only allow 1 move per simulation step.
                    if (!this.m_allowInput) {
                        return;
                    }
                    const playerPosition = this.m_player.GetTransform().p;
                    const playerX = [0], playerY = [0];
                    Fracker.WorldToTile(playerPosition, playerX, playerY);
                    switch (key) {
                        case "a":
                            playerX[0]--;
                            break;
                        case "s":
                            playerY[0]--;
                            break;
                        case "d":
                            playerX[0]++;
                            break;
                        case "w":
                            playerY[0]++;
                            break;
                        case "e":
                            // Start charging the fracking fluid.
                            if (this.m_frackingFluidChargeTime < 0.0) {
                                this.m_frackingFluidChargeTime = 0.0;
                            }
                            else {
                                // KeyboardUp() in freeglut (at least on OSX) is called
                                // repeatedly while a key is held.  This means there isn't
                                // a way for fracking fluid to be deployed when the user
                                // releases 'e'.  This works around the issue by attempting
                                // to deploy the fluid when 'e' is pressed again.
                                this.DeployFrackingFluid();
                            }
                            break;
                        default:
                            super.Keyboard(key);
                            break;
                    }
                    this.SetPlayerPosition(playerX[0], playerY[0]);
                    this.m_allowInput = false;
                }
                KeyboardUp(key) {
                    super.KeyboardUp(key);
                }
                MouseDown(p) {
                    super.MouseDown(p);
                    this.m_frackingFluidChargeTime = 0.0;
                }
                /**
                 * Try to deploy the fracking fluid or move the player.
                 */
                MouseUp(p) {
                    super.MouseUp(p);
                    if (!this.m_allowInput) {
                        return;
                    }
                    // If fracking fluid isn't being released, move the player.
                    if (!this.DeployFrackingFluid()) {
                        const playerPosition = this.m_player.GetTransform().p;
                        const playerX = [0], playerY = [0];
                        Fracker.WorldToTile(playerPosition, playerX, playerY);
                        // Move the player towards the mouse position, preferring to move
                        // along the axis with the maximal distance from the cursor.
                        const distance = box2d.b2Vec2.SubVV(p, Fracker.CenteredPosition(playerPosition), new box2d.b2Vec2());
                        const absDistX = Math.abs(distance.x);
                        const absDistY = Math.abs(distance.y);
                        if (absDistX > absDistY &&
                            absDistX >= FrackerSettings.k_tileHalfWidth) {
                            playerX[0] += distance.x > 0.0 ? 1 : -1;
                        }
                        else if (absDistY >= FrackerSettings.k_tileHalfWidth) {
                            playerY[0] += distance.y > 0.0 ? 1 : -1;
                        }
                        this.SetPlayerPosition(playerX[0], playerY[0]);
                    }
                    this.m_allowInput = false;
                }
                MouseMove(p) {
                    super.MouseMove(p);
                }
                Step(settings) {
                    let dt = settings.hz > 0.0 ? 1.0 / settings.hz : 0.0;
                    if (settings.pause && !settings.singleStep) {
                        dt = 0.0;
                    }
                    super.Step(settings);
                    this.m_tracker.Step(dt);
                    // Allow the user to move again.
                    this.m_allowInput = true;
                    // Charge up fracking fluid.
                    if (this.m_frackingFluidChargeTime >= 0.0) {
                        this.m_frackingFluidChargeTime += dt;
                    }
                    const playerPosition = this.m_player.GetTransform().p;
                    const playerX = [0], playerY = [0];
                    Fracker.WorldToTile(playerPosition, playerX, playerY);
                    // If the player is moved to a square with dirt, remove it.
                    if (this.GetMaterial(playerX[0], playerY[0]) === Fracker.Material.DIRT) {
                        this.SetMaterial(playerX[0], playerY[0], Fracker.Material.EMPTY);
                        this.SetBody(playerX[0], playerY[0], null);
                    }
                    // Destroy particles at the top of the well.
                    this.DestroyParticlesInTiles(this.m_wellX, this.m_wellTop, this.m_wellX, this.m_wellTop);
                    // Only move particles in the groups being tracked.
                    ///  const std.set<b2ParticleGroup*> &particleGroups = m_listener.GetParticleGroups();
                    const particleGroups = this.m_listener.GetParticleGroups();
                    ///  for (std.set<b2ParticleGroup*>.const_iterator it = particleGroups.begin(); it !== particleGroups.end(); ++it)
                    for (let it = 0; it < particleGroups.length; ++it) {
                        ///  b2ParticleGroup * const particleGroup = *it;
                        const particleGroup = particleGroups[it];
                        const index = particleGroup.GetBufferIndex();
                        ///  const b2Vec2* const positionBuffer = m_particleSystem.GetPositionBuffer() + index;
                        const positionBuffer = this.m_particleSystem.GetPositionBuffer();
                        ///  b2Vec2* const velocityBuffer = m_particleSystem.GetVelocityBuffer() + index;
                        const velocityBuffer = this.m_particleSystem.GetVelocityBuffer();
                        const particleCount = particleGroup.GetParticleCount();
                        for (let i = 0; i < particleCount; ++i) {
                            // Apply velocity to particles near the bottom or in the well
                            // sucking them up to the top.
                            const wellEnd = Fracker.CenteredPosition(Fracker.TileToWorld(this.m_wellX, this.m_wellBottom - 2));
                            const particlePosition = positionBuffer[index + i];
                            // Distance from the well's bottom.
                            ///  const b2Vec2 distance = particlePosition - wellEnd;
                            const distance = box2d.b2Vec2.SubVV(particlePosition, wellEnd, new box2d.b2Vec2());
                            // Distance from either well side wall.
                            const absDistX = Math.abs(distance.x);
                            if (absDistX < FrackerSettings.k_tileWidth &&
                                // If the particles are just below the well bottom.
                                distance.y > FrackerSettings.k_tileWidth * -2.0 &&
                                distance.y < 0.0) {
                                // Suck the particles towards the end of the well.
                                ///  b2Vec2 velocity = wellEnd - particlePosition;
                                const velocity = box2d.b2Vec2.SubVV(wellEnd, particlePosition, new box2d.b2Vec2());
                                velocity.Normalize();
                                ///  velocityBuffer[i] = velocity * FrackerSettings.k_wellSuckSpeedOutside;
                                velocityBuffer[index + i].Copy(velocity.SelfMul(FrackerSettings.k_wellSuckSpeedOutside));
                            }
                            else if (absDistX <= FrackerSettings.k_tileHalfWidth && distance.y > 0.0) {
                                // Suck the particles up the well with a random
                                // x component moving them side to side in the well.
                                const randomX = (Math.random() * FrackerSettings.k_tileHalfWidth) - distance.x;
                                const velocity = new box2d.b2Vec2(randomX, FrackerSettings.k_tileHeight);
                                velocity.Normalize();
                                ///  velocityBuffer[i] = velocity * FrackerSettings.k_wellSuckSpeedInside;
                                velocityBuffer[index + i].Copy(velocity.SelfMul(FrackerSettings.k_wellSuckSpeedInside));
                            }
                        }
                    }
                    // Draw everything.
                    this.DrawPlayer();
                    this.DrawWell();
                    this.DrawScore();
                }
                /**
                 * Render the well.
                 */
                DrawWell() {
                    for (let y = this.m_wellBottom; y <= this.m_wellTop; ++y) {
                        this.DrawQuad(Fracker.TileToWorld(this.m_wellX, y), FrackerSettings.k_wellColor);
                    }
                }
                /**
                 * Render the player / fracker.
                 */
                DrawPlayer() {
                    this.DrawQuad(this.m_player.GetTransform().p, Fracker.LerpColor(FrackerSettings.k_playerColor, FrackerSettings.k_playerFrackColor, box2d.b2Max(this.m_frackingFluidChargeTime /
                        FrackerSettings.k_frackingFluidChargeTime, 0.0)), true);
                }
                /**
                 * Render the score and the instructions / keys.
                 */
                DrawScore() {
                    ///  char score[512];
                    ///  sprintf(score, "Score: %d, Remaining Oil %d",
                    ///          m_listener.GetScore(), m_listener.GetOil());
                    ///  const char *lines[] = { score,  "Move: a,s,d,w   Fracking Fluid: e" };
                    ///  for (uint32 i = 0; i < B2_ARRAY_SIZE(lines); ++i)
                    ///  {
                    ///    m_debugDraw.DrawString(5, m_textLine, lines[i]);
                    ///    m_textLine += DRAW_STRING_NEW_LINE;
                    ///  }
                    testbed.g_debugDraw.DrawString(5, this.m_textLine, `Score: ${this.m_listener.GetScore()}, Remaining Oil ${this.m_listener.GetOil()}`);
                    this.m_textLine += testbed.DRAW_STRING_NEW_LINE;
                    testbed.g_debugDraw.DrawString(5, this.m_textLine, "Move: a,s,d,w   Fracking Fluid: e");
                    this.m_textLine += testbed.DRAW_STRING_NEW_LINE;
                }
                /**
                 * Draw a quad at position of color that is either just an
                 * outline (fill = false) or solid (fill = true).
                 */
                DrawQuad(position, color, fill = false) {
                    ///  b2Vec2 verts[4];
                    const verts = box2d.b2Vec2.MakeArray(4);
                    const maxX = position.x + FrackerSettings.k_tileWidth;
                    const maxY = position.y + FrackerSettings.k_tileHeight;
                    verts[0].Set(position.x, maxY);
                    verts[1].Set(position.x, position.y);
                    verts[2].Set(maxX, position.y);
                    verts[3].Set(maxX, maxY);
                    if (fill) {
                        testbed.g_debugDraw.DrawPolygon(verts, 4, color);
                    }
                    else {
                        testbed.g_debugDraw.DrawSolidPolygon(verts, 4, color);
                    }
                }
                ///  // Get a pointer to the material of the tile at the specified position.
                ///  Material* GetMaterialStorage(const int32 x, const int32 y)
                ///  {
                ///    return &m_material[Fracker.TileToArrayOffset(x, y)];
                ///  }
                ///  // A pointer to the body storage associated with the specified tile
                ///  // position.
                ///  b2Body** GetBodyStorage(const int32 x, const int32 y)
                ///  {
                ///    return &m_bodies[Fracker.TileToArrayOffset(x, y)];
                ///  }
                GetDefaultViewZoom() {
                    return 0.1;
                }
                static Create() {
                    return new Fracker();
                }
                /**
                 * Get the bottom left position of the world in world units.
                 */
                static GetBottomLeft(bottomLeft) {
                    bottomLeft.Set(FrackerSettings.k_worldCenterX -
                        FrackerSettings.k_worldHalfWidth, FrackerSettings.k_worldCenterY -
                        FrackerSettings.k_worldHalfHeight);
                }
                /**
                 * Get the extents of the world in world units.
                 */
                static GetExtents(bottomLeft, topRight) {
                    Fracker.GetBottomLeft(bottomLeft);
                    topRight.Set(FrackerSettings.k_worldCenterX +
                        FrackerSettings.k_worldHalfWidth, FrackerSettings.k_worldCenterY +
                        FrackerSettings.k_worldHalfHeight);
                }
                // Convert a point in world coordintes to a tile location
                static WorldToTile(position, x, y) {
                    // Translate relative to the world center and scale based upon the
                    // tile size.
                    const bottomLeft = new box2d.b2Vec2();
                    Fracker.GetBottomLeft(bottomLeft);
                    x[0] = Math.floor(((position.x - bottomLeft.x) /
                        FrackerSettings.k_tileWidth) +
                        FrackerSettings.k_tileHalfWidth);
                    y[0] = Math.floor(((position.y - bottomLeft.y) /
                        FrackerSettings.k_tileHeight) +
                        FrackerSettings.k_tileHalfHeight);
                }
                /**
                 * Convert a tile position to a point  in world coordinates.
                 */
                static TileToWorld(x, y, out = new box2d.b2Vec2()) {
                    // Scale based upon the tile size and translate relative to the world
                    // center.
                    const bottomLeft = new box2d.b2Vec2();
                    Fracker.GetBottomLeft(bottomLeft);
                    return out.Set((x * FrackerSettings.k_tileWidth) + bottomLeft.x, (y * FrackerSettings.k_tileHeight) + bottomLeft.y);
                }
                /**
                 * Calculate the offset within an array of all world tiles using
                 * the specified tile coordinates.
                 */
                static TileToArrayOffset(x, y) {
                    box2d.b2Assert(x >= 0);
                    box2d.b2Assert(x < FrackerSettings.k_worldWidthTiles);
                    box2d.b2Assert(y >= 0);
                    box2d.b2Assert(y < FrackerSettings.k_worldHeightTiles);
                    return x + (y * FrackerSettings.k_worldWidthTiles);
                }
                /**
                 * Calculate the center of a tile position in world units.
                 */
                static CenteredPosition(position, out = new box2d.b2Vec2()) {
                    return out.Set(position.x + FrackerSettings.k_tileHalfWidth, position.y + FrackerSettings.k_tileHalfHeight);
                }
                /**
                 * Interpolate between color a and b using t.
                 */
                static LerpColor(a, b, t) {
                    return new box2d.b2Color(Fracker.Lerp(a.r, b.r, t), Fracker.Lerp(a.g, b.g, t), Fracker.Lerp(a.b, b.b, t));
                }
                /**
                 * Interpolate between a and b using t.
                 */
                static Lerp(a, b, t) {
                    return a * (1.0 - t) + b * t;
                }
            };
            exports_1("Fracker", Fracker);
            (function (Fracker) {
                /**
                 * Type of material in a tile.
                 */
                let Material;
                (function (Material) {
                    Material[Material["EMPTY"] = 0] = "EMPTY";
                    Material[Material["DIRT"] = 1] = "DIRT";
                    Material[Material["ROCK"] = 2] = "ROCK";
                    Material[Material["OIL"] = 3] = "OIL";
                    Material[Material["WATER"] = 4] = "WATER";
                    Material[Material["WELL"] = 5] = "WELL";
                    Material[Material["PUMP"] = 6] = "PUMP";
                })(Material = Fracker.Material || (Fracker.Material = {}));
                /**
                 * Keep track of particle groups which are drawn up the well and
                 * tracks the score of the game.
                 */
                class DestructionListener extends ParticleGroupTracker {
                    constructor() {
                        super(...arguments);
                        this.m_score = 0;
                        this.m_oil = 0;
                    }
                    /**
                     * Initialize the score.
                     */
                    __ctor__() { }
                    __dtor__() {
                        if (this.m_world) {
                            this.m_world.SetDestructionListener(this.m_previousListener);
                        }
                    }
                    /**
                     * Initialize the particle system and world, setting this class
                     * as a destruction listener for the world.
                     */
                    Initialize(world) {
                        box2d.b2Assert(world !== null);
                        this.m_world = world;
                        this.m_world.SetDestructionListener(this);
                    }
                    /**
                     * Add to the current score.
                     */
                    AddScore(score) {
                        this.m_score += score;
                    }
                    /**
                     * Get the current score.
                     */
                    GetScore() {
                        return this.m_score;
                    }
                    /**
                     * Add to the remaining oil.
                     */
                    AddOil(oil) {
                        this.m_oil += oil;
                    }
                    /**
                     * Get the total oil.
                     */
                    GetOil() {
                        return this.m_oil;
                    }
                    /**
                     * Update the score when certain particles are destroyed.
                     */
                    SayGoodbyeParticle(particleSystem, index) {
                        box2d.b2Assert(particleSystem !== null);
                        ///  const void * const userData = particleSystem.GetUserDataBuffer()[index];
                        const userData = particleSystem.GetUserDataBuffer()[index];
                        if (userData) {
                            ///  const Material material = *((Material*)userData);
                            const material = userData;
                            switch (material) {
                                case Fracker.Material.OIL:
                                    this.AddScore(FrackerSettings.k_scorePerOilParticle);
                                    this.AddOil(-1);
                                    break;
                                case Fracker.Material.WATER:
                                    this.AddScore(FrackerSettings.k_scorePerWaterParticle);
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
                Fracker.DestructionListener = DestructionListener;
            })(Fracker || (Fracker = {}));
            exports_1("Fracker", Fracker);
        }
    };
});
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiRnJhY2tlci5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIkZyYWNrZXIudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7R0FnQkc7Ozs7Ozs7Ozs7Ozs7OztZQU9IOzs7ZUFHRztZQUNILGlCQUFBO2dCQUFBO29CQUNFLHNCQUFpQixHQUEyRCxFQUFFLENBQUM7Z0JBMERqRixDQUFDO2dCQXhEQzs7bUJBRUc7Z0JBQ0gsUUFBUTtvQkFDTiwrSEFBK0g7b0JBQy9ILEtBQUssSUFBSSxFQUFFLEdBQUcsQ0FBQyxFQUFFLEVBQUUsR0FBRyxJQUFJLENBQUMsaUJBQWlCLENBQUMsTUFBTSxFQUFFLEVBQUUsRUFBRSxFQUFFO3dCQUN6RCxxQkFBcUI7d0JBQ3JCLElBQUksQ0FBQyxpQkFBaUIsQ0FBQyxFQUFFLENBQUMsQ0FBQyxPQUFPLENBQUMsUUFBUSxFQUFFLENBQUM7cUJBQy9DO2dCQUNILENBQUM7Z0JBRUQ7Ozs7bUJBSUc7Z0JBQ0gsR0FBRyxDQUFDLE9BQThCLEVBQUUsUUFBZ0I7b0JBQ2xELDJDQUEyQztvQkFDM0MsSUFBSSxDQUFDLGlCQUFpQixDQUFDLElBQUksQ0FBQzt3QkFDMUIsT0FBTyxFQUFFLE9BQU87d0JBQ2hCLFFBQVEsRUFBRSxRQUFRO3FCQUNuQixDQUFDLENBQUM7Z0JBQ0wsQ0FBQztnQkFFRDs7bUJBRUc7Z0JBQ0gsSUFBSSxDQUFDLEVBQVU7b0JBQ2Isa0RBQWtEO29CQUNsRCxNQUFNLGlCQUFpQixHQUE0QixFQUFFLENBQUM7b0JBQ3RELCtIQUErSDtvQkFDL0gsS0FBSyxJQUFJLEVBQUUsR0FBRyxDQUFDLEVBQUUsRUFBRSxHQUFHLElBQUksQ0FBQyxpQkFBaUIsQ0FBQyxNQUFNLEVBQUUsRUFBRSxFQUFFLEVBQUU7d0JBQ3pELDhDQUE4Qzt3QkFDOUMsTUFBTSxPQUFPLEdBQUcsSUFBSSxDQUFDLGlCQUFpQixDQUFDLEVBQUUsQ0FBQyxDQUFDLE9BQU8sQ0FBQzt3QkFDbkQsNkNBQTZD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