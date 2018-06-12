import { b2Vec2, b2Mat22, b2Vec3, b2Mat33, b2Rot, XY } from "../../Common/b2Math";
import { b2Body } from "../b2Body";
import { b2Joint, b2JointDef, b2LimitState, b2IJointDef } from "./b2Joint";
import { b2SolverData } from "../b2TimeStep";
export interface b2IRevoluteJointDef extends b2IJointDef {
    localAnchorA?: XY;
    localAnchorB?: XY;
    referenceAngle?: number;
    enableLimit?: boolean;
    lowerAngle?: number;
    upperAngle?: number;
    enableMotor?: boolean;
    motorSpeed?: number;
    maxMotorTorque?: number;
}
export declare class b2RevoluteJointDef extends b2JointDef implements b2IRevoluteJointDef {
    readonly localAnchorA: b2Vec2;
    readonly localAnchorB: b2Vec2;
    referenceAngle: number;
    enableLimit: boolean;
    lowerAngle: number;
    upperAngle: number;
    enableMotor: boolean;
    motorSpeed: number;
    maxMotorTorque: number;
    constructor();
    Initialize(bA: b2Body, bB: b2Body, anchor: b2Vec2): void;
}
export declare class b2RevoluteJoint extends b2Joint {
    readonly m_localAnchorA: b2Vec2;
    readonly m_localAnchorB: b2Vec2;
    readonly m_impulse: b2Vec3;
    m_motorImpulse: number;
    m_enableMotor: boolean;
    m_maxMotorTorque: number;
    m_motorSpeed: number;
    m_enableLimit: boolean;
    m_referenceAngle: number;
    m_lowerAngle: number;
    m_upperAngle: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_rA: b2Vec2;
    readonly m_rB: b2Vec2;
    readonly m_localCenterA: b2Vec2;
    readonly m_localCenterB: b2Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    readonly m_mass: b2Mat33;
    m_motorMass: number;
    m_limitState: b2LimitState;
    readonly m_qA: b2Rot;
    readonly m_qB: b2Rot;
    readonly m_lalcA: b2Vec2;
    readonly m_lalcB: b2Vec2;
    readonly m_K: b2Mat22;
    constructor(def: b2IRevoluteJointDef);
    private static InitVelocityConstraints_s_P;
    InitVelocityConstraints(data: b2SolverData): void;
    private static SolveVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_Cdot1;
    private static SolveVelocityConstraints_s_impulse_v3;
    private static SolveVelocityConstraints_s_reduced_v2;
    private static SolveVelocityConstraints_s_impulse_v2;
    SolveVelocityConstraints(data: b2SolverData): void;
    private static SolvePositionConstraints_s_C_v2;
    private static SolvePositionConstraints_s_impulse;
    SolvePositionConstraints(data: b2SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<b2Vec2>;
    GetLocalAnchorB(): Readonly<b2Vec2>;
    GetReferenceAngle(): number;
    GetJointAngle(): number;
    GetJointSpeed(): number;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    GetMotorTorque(inv_dt: number): number;
    GetMotorSpeed(): number;
    SetMaxMotorTorque(torque: number): void;
    GetMaxMotorTorque(): number;
    IsLimitEnabled(): boolean;
    EnableLimit(flag: boolean): void;
    GetLowerLimit(): number;
    GetUpperLimit(): number;
    SetLimits(lower: number, upper: number): void;
    SetMotorSpeed(speed: number): void;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
