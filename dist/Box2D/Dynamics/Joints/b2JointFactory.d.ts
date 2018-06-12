import { b2Joint, b2IJointDef } from "./b2Joint";
export declare class b2JointFactory {
    static Create(def: b2IJointDef, allocator: any): b2Joint;
    static Destroy(joint: b2Joint, allocator: any): void;
}
