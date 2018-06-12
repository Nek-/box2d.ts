import { b2Vec2, XY } from "../Common/b2Math";
import { b2GrowableStack } from "../Common/b2GrowableStack";
import { b2AABB, b2RayCastInput } from "./b2Collision";
export declare class b2TreeNode {
    m_id: number;
    aabb: b2AABB;
    userData: any;
    parent: b2TreeNode | null;
    child1: b2TreeNode | null;
    child2: b2TreeNode | null;
    height: number;
    constructor(id?: number);
    IsLeaf(): boolean;
}
export declare class b2DynamicTree {
    m_root: b2TreeNode | null;
    m_freeList: b2TreeNode | null;
    m_path: number;
    m_insertionCount: number;
    static s_stack: b2GrowableStack<b2TreeNode>;
    static s_r: b2Vec2;
    static s_v: b2Vec2;
    static s_abs_v: b2Vec2;
    static s_segmentAABB: b2AABB;
    static s_subInput: b2RayCastInput;
    static s_combinedAABB: b2AABB;
    static s_aabb: b2AABB;
    GetUserData(proxy: b2TreeNode): any;
    GetFatAABB(proxy: b2TreeNode): b2AABB;
    Query(callback: (node: b2TreeNode) => boolean, aabb: b2AABB): void;
    RayCast(callback: (input: b2RayCastInput, node: b2TreeNode) => number, input: b2RayCastInput): void;
    static s_node_id: number;
    AllocateNode(): b2TreeNode;
    FreeNode(node: b2TreeNode): void;
    CreateProxy(aabb: b2AABB, userData: any): b2TreeNode;
    DestroyProxy(proxy: b2TreeNode): void;
    MoveProxy(proxy: b2TreeNode, aabb: b2AABB, displacement: b2Vec2): boolean;
    InsertLeaf(leaf: b2TreeNode): void;
    RemoveLeaf(leaf: b2TreeNode): void;
    Balance(A: b2TreeNode): b2TreeNode;
    GetHeight(): number;
    private static GetAreaNode;
    GetAreaRatio(): number;
    ComputeHeightNode(node: b2TreeNode): number;
    ComputeHeight(): number;
    ValidateStructure(index: b2TreeNode): void;
    ValidateMetrics(index: b2TreeNode): void;
    Validate(): void;
    private static GetMaxBalanceNode;
    GetMaxBalance(): number;
    RebuildBottomUp(): void;
    private static ShiftOriginNode;
    ShiftOrigin(newOrigin: XY): void;
}
