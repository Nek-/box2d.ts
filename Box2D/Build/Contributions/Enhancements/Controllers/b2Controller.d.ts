import { b2TimeStep, b2Draw, b2Body, b2World } from "../../../Box2D/Box2D/Box2D";
/**
 * A controller edge is used to connect bodies and controllers
 * together in a bipartite graph.
 */
export declare class b2ControllerEdge {
    controller: b2Controller;
    body: b2Body;
    prevBody: b2ControllerEdge;
    nextBody: b2ControllerEdge;
    prevController: b2ControllerEdge;
    nextController: b2ControllerEdge;
}
/**
 * Base class for controllers. Controllers are a convience for
 * encapsulating common per-step functionality.
 */
export declare abstract class b2Controller {
    m_world: b2World;
    m_bodyList: b2ControllerEdge;
    m_bodyCount: number;
    m_prev: b2Controller | null;
    m_next: b2Controller | null;
    /**
     * Controllers override this to implement per-step functionality.
     */
    abstract Step(step: b2TimeStep): void;
    /**
     * Controllers override this to provide debug drawing.
     */
    abstract Draw(debugDraw: b2Draw): void;
    /**
     * Get the next controller in the world's body list.
     */
    GetNext(): b2Controller;
    /**
     * Get the previous controller in the world's body list.
     */
    GetPrev(): b2Controller;
    /**
     * Get the parent world of this body.
     */
    GetWorld(): b2World;
    /**
     * Get the attached body list
     */
    GetBodyList(): b2ControllerEdge;
    /**
     * Adds a body to the controller list.
     */
    AddBody(body: b2Body): void;
    /**
     * Removes a body from the controller list.
     */
    RemoveBody(body: b2Body): void;
    /**
     * Removes all bodies from the controller list.
     */
    Clear(): void;
}
