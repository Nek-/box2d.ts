import { b2Vec2, b2Transform, XY } from "../Common/b2Math";
import { b2Color, b2Draw } from "../Common/b2Draw";
import { b2AABB } from "../Collision/b2Collision";
import { b2Shape } from "../Collision/Shapes/b2Shape";
import { b2Contact } from "./Contacts/b2Contact";
import { b2Joint, b2IJointDef } from "./Joints/b2Joint";
import { b2Body, b2IBodyDef } from "./b2Body";
import { b2ContactManager } from "./b2ContactManager";
import { b2Fixture } from "./b2Fixture";
import { b2Island } from "./b2Island";
import { b2Profile, b2TimeStep } from "./b2TimeStep";
import { b2ContactFilter } from "./b2WorldCallbacks";
import { b2ContactListener } from "./b2WorldCallbacks";
import { b2DestructionListener } from "./b2WorldCallbacks";
import { b2QueryCallback, b2QueryCallbackFunction } from "./b2WorldCallbacks";
import { b2RayCastCallback, b2RayCastCallbackFunction } from "./b2WorldCallbacks";
import { b2ParticleSystemDef, b2ParticleSystem } from "../Particle/b2ParticleSystem";
import { b2Controller } from "../Controllers/b2Controller";
export declare class b2World {
    m_newFixture: boolean;
    m_locked: boolean;
    m_clearForces: boolean;
    readonly m_contactManager: b2ContactManager;
    m_bodyList: b2Body | null;
    m_jointList: b2Joint | null;
    m_particleSystemList: b2ParticleSystem | null;
    m_bodyCount: number;
    m_jointCount: number;
    readonly m_gravity: b2Vec2;
    m_allowSleep: boolean;
    m_destructionListener: b2DestructionListener | null;
    m_debugDraw: b2Draw | null;
    m_inv_dt0: number;
    m_warmStarting: boolean;
    m_continuousPhysics: boolean;
    m_subStepping: boolean;
    m_stepComplete: boolean;
    readonly m_profile: b2Profile;
    readonly m_island: b2Island;
    readonly s_stack: Array<b2Body | null>;
    m_controllerList: b2Controller | null;
    m_controllerCount: number;
    constructor(gravity: XY);
    SetDestructionListener(listener: b2DestructionListener | null): void;
    SetContactFilter(filter: b2ContactFilter): void;
    SetContactListener(listener: b2ContactListener): void;
    SetDebugDraw(debugDraw: b2Draw): void;
    CreateBody(def?: b2IBodyDef): b2Body;
    DestroyBody(b: b2Body): void;
    CreateJoint<T extends b2Joint>(def: b2IJointDef): T;
    DestroyJoint(j: b2Joint): void;
    CreateParticleSystem(def: b2ParticleSystemDef): b2ParticleSystem;
    DestroyParticleSystem(p: b2ParticleSystem): void;
    CalculateReasonableParticleIterations(timeStep: number): number;
    private static Step_s_step;
    private static Step_s_stepTimer;
    private static Step_s_timer;
    Step(dt: number, velocityIterations: number, positionIterations: number, particleIterations?: number): void;
    ClearForces(): void;
    DrawParticleSystem(system: b2ParticleSystem): void;
    private static DrawDebugData_s_color;
    private static DrawDebugData_s_vs;
    private static DrawDebugData_s_xf;
    DrawDebugData(): void;
    QueryAABB(callback: b2QueryCallback | null, aabb: b2AABB, fn?: b2QueryCallbackFunction): void;
    QueryAllAABB(aabb: b2AABB, out?: b2Fixture[]): b2Fixture[];
    QueryPointAABB(callback: b2QueryCallback | null, point: b2Vec2, fn?: b2QueryCallbackFunction): void;
    QueryAllPointAABB(point: b2Vec2, out?: b2Fixture[]): b2Fixture[];
    private static QueryFixtureShape_s_aabb;
    QueryFixtureShape(callback: b2QueryCallback | null, shape: b2Shape, index: number, transform: b2Transform, fn?: b2QueryCallbackFunction): void;
    QueryAllFixtureShape(shape: b2Shape, index: number, transform: b2Transform, out?: b2Fixture[]): b2Fixture[];
    QueryFixturePoint(callback: b2QueryCallback | null, point: b2Vec2, fn?: b2QueryCallbackFunction): void;
    QueryAllFixturePoint(point: b2Vec2, out?: b2Fixture[]): b2Fixture[];
    private static RayCast_s_input;
    private static RayCast_s_output;
    private static RayCast_s_point;
    RayCast(callback: b2RayCastCallback | null, point1: b2Vec2, point2: b2Vec2, fn?: b2RayCastCallbackFunction): void;
    RayCastOne(point1: b2Vec2, point2: b2Vec2): b2Fixture | null;
    RayCastAll(point1: b2Vec2, point2: b2Vec2, out?: b2Fixture[]): b2Fixture[];
    GetBodyList(): b2Body | null;
    GetJointList(): b2Joint | null;
    GetParticleSystemList(): b2ParticleSystem | null;
    GetContactList(): b2Contact | null;
    SetAllowSleeping(flag: boolean): void;
    GetAllowSleeping(): boolean;
    SetWarmStarting(flag: boolean): void;
    GetWarmStarting(): boolean;
    SetContinuousPhysics(flag: boolean): void;
    GetContinuousPhysics(): boolean;
    SetSubStepping(flag: boolean): void;
    GetSubStepping(): boolean;
    GetProxyCount(): number;
    GetBodyCount(): number;
    GetJointCount(): number;
    GetContactCount(): number;
    GetTreeHeight(): number;
    GetTreeBalance(): number;
    GetTreeQuality(): number;
    SetGravity(gravity: XY, wake?: boolean): void;
    GetGravity(): Readonly<b2Vec2>;
    IsLocked(): boolean;
    SetAutoClearForces(flag: boolean): void;
    GetAutoClearForces(): boolean;
    ShiftOrigin(newOrigin: XY): void;
    GetContactManager(): b2ContactManager;
    GetProfile(): b2Profile;
    Dump(log: (format: string, ...args: any[]) => void): void;
    private static DrawJoint_s_p1;
    private static DrawJoint_s_p2;
    private static DrawJoint_s_color;
    DrawJoint(joint: b2Joint): void;
    DrawShape(fixture: b2Fixture, color: b2Color): void;
    Solve(step: b2TimeStep): void;
    private static SolveTOI_s_subStep;
    private static SolveTOI_s_backup;
    private static SolveTOI_s_backup1;
    private static SolveTOI_s_backup2;
    private static SolveTOI_s_toi_input;
    private static SolveTOI_s_toi_output;
    SolveTOI(step: b2TimeStep): void;
    AddController(controller: b2Controller): b2Controller;
    RemoveController(controller: b2Controller): b2Controller;
}
