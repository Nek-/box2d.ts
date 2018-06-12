import { b2Vec2, b2Transform, XY } from "../../Common/b2Math";
import { b2AABB, b2RayCastInput, b2RayCastOutput } from "../b2Collision";
import { b2DistanceProxy } from "../b2Distance";
import { b2MassData } from "./b2Shape";
import { b2Shape } from "./b2Shape";
export declare class b2PolygonShape extends b2Shape {
    readonly m_centroid: b2Vec2;
    m_vertices: b2Vec2[];
    m_normals: b2Vec2[];
    m_count: number;
    constructor();
    Clone(): b2PolygonShape;
    Copy(other: b2PolygonShape): b2PolygonShape;
    GetChildCount(): number;
    private static Set_s_ps;
    private static Set_s_hull;
    private static Set_s_r;
    private static Set_s_v;
    Set(vertices: XY[], count?: number, start?: number): b2PolygonShape;
    SetAsArray(vertices: XY[], count?: number): b2PolygonShape;
    SetAsBox(hx: number, hy: number, center?: XY, angle?: number): b2PolygonShape;
    private static TestPoint_s_pLocal;
    TestPoint(xf: b2Transform, p: b2Vec2): boolean;
    private static ComputeDistance_s_pLocal;
    private static ComputeDistance_s_normalForMaxDistance;
    private static ComputeDistance_s_minDistance;
    private static ComputeDistance_s_distance;
    ComputeDistance(xf: b2Transform, p: b2Vec2, normal: b2Vec2, childIndex: number): number;
    private static RayCast_s_p1;
    private static RayCast_s_p2;
    private static RayCast_s_d;
    RayCast(output: b2RayCastOutput, input: b2RayCastInput, xf: b2Transform, childIndex: number): boolean;
    private static ComputeAABB_s_v;
    ComputeAABB(aabb: b2AABB, xf: b2Transform, childIndex: number): void;
    private static ComputeMass_s_center;
    private static ComputeMass_s_s;
    private static ComputeMass_s_e1;
    private static ComputeMass_s_e2;
    ComputeMass(massData: b2MassData, density: number): void;
    private static Validate_s_e;
    private static Validate_s_v;
    Validate(): boolean;
    SetupDistanceProxy(proxy: b2DistanceProxy, index: number): void;
    private static ComputeSubmergedArea_s_normalL;
    private static ComputeSubmergedArea_s_depths;
    private static ComputeSubmergedArea_s_md;
    private static ComputeSubmergedArea_s_intoVec;
    private static ComputeSubmergedArea_s_outoVec;
    private static ComputeSubmergedArea_s_center;
    ComputeSubmergedArea(normal: b2Vec2, offset: number, xf: b2Transform, c: b2Vec2): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    private static ComputeCentroid_s_pRef;
    private static ComputeCentroid_s_e1;
    private static ComputeCentroid_s_e2;
    static ComputeCentroid(vs: b2Vec2[], count: number, out: b2Vec2): b2Vec2;
}
