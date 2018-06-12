"use strict";
/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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
var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert, b2_linearSlop } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Math_1 = require("../../Common/b2Math");
var b2Shape_1 = require("./b2Shape");
var b2EdgeShape_1 = require("./b2EdgeShape");
/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
var b2ChainShape = /** @class */ (function (_super) {
    __extends(b2ChainShape, _super);
    function b2ChainShape() {
        var _this = _super.call(this, b2Shape_1.b2ShapeType.e_chainShape, b2Settings_1.b2_polygonRadius) || this;
        _this.m_vertices = [];
        _this.m_count = 0;
        _this.m_prevVertex = new b2Math_1.b2Vec2();
        _this.m_nextVertex = new b2Math_1.b2Vec2();
        _this.m_hasPrevVertex = false;
        _this.m_hasNextVertex = false;
        return _this;
    }
    /// Create a loop. This automatically adjusts connectivity.
    /// @param vertices an array of vertices, these are copied
    /// @param count the vertex count
    b2ChainShape.prototype.CreateLoop = function (vertices, count, start) {
        // DEBUG: b2Assert(count >= 3);
        // DEBUG: for (let i: number = 1; i < count; ++i) {
        // DEBUG:   const v1 = vertices[start + i - 1];
        // DEBUG:   const v2 = vertices[start + i];
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   b2Assert(b2Vec2.DistanceSquaredVV(v1, v2) > b2_linearSlop * b2_linearSlop);
        // DEBUG: }
        if (count === void 0) { count = vertices.length; }
        if (start === void 0) { start = 0; }
        this.m_count = count + 1;
        this.m_vertices = b2Math_1.b2Vec2.MakeArray(this.m_count);
        for (var i = 0; i < count; ++i) {
            this.m_vertices[i].Copy(vertices[start + i]);
        }
        this.m_vertices[count].Copy(this.m_vertices[0]);
        this.m_prevVertex.Copy(this.m_vertices[this.m_count - 2]);
        this.m_nextVertex.Copy(this.m_vertices[1]);
        this.m_hasPrevVertex = true;
        this.m_hasNextVertex = true;
        return this;
    };
    /// Create a chain with isolated end vertices.
    /// @param vertices an array of vertices, these are copied
    /// @param count the vertex count
    b2ChainShape.prototype.CreateChain = function (vertices, count, start) {
        // DEBUG: b2Assert(count >= 2);
        // DEBUG: for (let i: number = 1; i < count; ++i) {
        // DEBUG:   const v1 = vertices[start + i - 1];
        // DEBUG:   const v2 = vertices[start + i];
        // DEBUG:   // If the code crashes here, it means your vertices are too close together.
        // DEBUG:   b2Assert(b2Vec2.DistanceSquaredVV(v1, v2) > b2_linearSlop * b2_linearSlop);
        // DEBUG: }
        if (count === void 0) { count = vertices.length; }
        if (start === void 0) { start = 0; }
        this.m_count = count;
        this.m_vertices = b2Math_1.b2Vec2.MakeArray(count);
        for (var i = 0; i < count; ++i) {
            this.m_vertices[i].Copy(vertices[start + i]);
        }
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
        this.m_prevVertex.SetZero();
        this.m_nextVertex.SetZero();
        return this;
    };
    /// Establish connectivity to a vertex that precedes the first vertex.
    /// Don't call this for loops.
    b2ChainShape.prototype.SetPrevVertex = function (prevVertex) {
        this.m_prevVertex.Copy(prevVertex);
        this.m_hasPrevVertex = true;
        return this;
    };
    /// Establish connectivity to a vertex that follows the last vertex.
    /// Don't call this for loops.
    b2ChainShape.prototype.SetNextVertex = function (nextVertex) {
        this.m_nextVertex.Copy(nextVertex);
        this.m_hasNextVertex = true;
        return this;
    };
    /// Implement b2Shape. Vertices are cloned using b2Alloc.
    b2ChainShape.prototype.Clone = function () {
        return new b2ChainShape().Copy(this);
    };
    b2ChainShape.prototype.Copy = function (other) {
        _super.prototype.Copy.call(this, other);
        // DEBUG: b2Assert(other instanceof b2ChainShape);
        this.CreateChain(other.m_vertices, other.m_count);
        this.m_prevVertex.Copy(other.m_prevVertex);
        this.m_nextVertex.Copy(other.m_nextVertex);
        this.m_hasPrevVertex = other.m_hasPrevVertex;
        this.m_hasNextVertex = other.m_hasNextVertex;
        return this;
    };
    /// @see b2Shape::GetChildCount
    b2ChainShape.prototype.GetChildCount = function () {
        // edge count = vertex count - 1
        return this.m_count - 1;
    };
    /// Get a child edge.
    b2ChainShape.prototype.GetChildEdge = function (edge, index) {
        // DEBUG: b2Assert(0 <= index && index < this.m_count - 1);
        edge.m_type = b2Shape_1.b2ShapeType.e_edgeShape;
        edge.m_radius = this.m_radius;
        edge.m_vertex1.Copy(this.m_vertices[index]);
        edge.m_vertex2.Copy(this.m_vertices[index + 1]);
        if (index > 0) {
            edge.m_vertex0.Copy(this.m_vertices[index - 1]);
            edge.m_hasVertex0 = true;
        }
        else {
            edge.m_vertex0.Copy(this.m_prevVertex);
            edge.m_hasVertex0 = this.m_hasPrevVertex;
        }
        if (index < this.m_count - 2) {
            edge.m_vertex3.Copy(this.m_vertices[index + 2]);
            edge.m_hasVertex3 = true;
        }
        else {
            edge.m_vertex3.Copy(this.m_nextVertex);
            edge.m_hasVertex3 = this.m_hasNextVertex;
        }
    };
    /// This always return false.
    /// @see b2Shape::TestPoint
    b2ChainShape.prototype.TestPoint = function (xf, p) {
        return false;
    };
    b2ChainShape.prototype.ComputeDistance = function (xf, p, normal, childIndex) {
        var edge = b2ChainShape.ComputeDistance_s_edgeShape;
        this.GetChildEdge(edge, childIndex);
        return edge.ComputeDistance(xf, p, normal, 0);
    };
    b2ChainShape.prototype.RayCast = function (output, input, xf, childIndex) {
        // DEBUG: b2Assert(childIndex < this.m_count);
        var edgeShape = b2ChainShape.RayCast_s_edgeShape;
        edgeShape.m_vertex1.Copy(this.m_vertices[childIndex]);
        edgeShape.m_vertex2.Copy(this.m_vertices[(childIndex + 1) % this.m_count]);
        return edgeShape.RayCast(output, input, xf, 0);
    };
    b2ChainShape.prototype.ComputeAABB = function (aabb, xf, childIndex) {
        // DEBUG: b2Assert(childIndex < this.m_count);
        var vertexi1 = this.m_vertices[childIndex];
        var vertexi2 = this.m_vertices[(childIndex + 1) % this.m_count];
        var v1 = b2Math_1.b2Transform.MulXV(xf, vertexi1, b2ChainShape.ComputeAABB_s_v1);
        var v2 = b2Math_1.b2Transform.MulXV(xf, vertexi2, b2ChainShape.ComputeAABB_s_v2);
        b2Math_1.b2Vec2.MinV(v1, v2, aabb.lowerBound);
        b2Math_1.b2Vec2.MaxV(v1, v2, aabb.upperBound);
    };
    /// Chains have zero mass.
    /// @see b2Shape::ComputeMass
    b2ChainShape.prototype.ComputeMass = function (massData, density) {
        massData.mass = 0;
        massData.center.SetZero();
        massData.I = 0;
    };
    b2ChainShape.prototype.SetupDistanceProxy = function (proxy, index) {
        // DEBUG: b2Assert(0 <= index && index < this.m_count);
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_vertices[index]);
        if (index + 1 < this.m_count) {
            proxy.m_vertices[1].Copy(this.m_vertices[index + 1]);
        }
        else {
            proxy.m_vertices[1].Copy(this.m_vertices[0]);
        }
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    };
    b2ChainShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
        c.SetZero();
        return 0;
    };
    b2ChainShape.prototype.Dump = function (log) {
        log("    const shape: b2ChainShape = new b2ChainShape();\n");
        log("    const vs: b2Vec2[] = b2Vec2.MakeArray(%d);\n", b2Settings_1.b2_maxPolygonVertices);
        for (var i = 0; i < this.m_count; ++i) {
            log("    vs[%d].Set(%.15f, %.15f);\n", i, this.m_vertices[i].x, this.m_vertices[i].y);
        }
        log("    shape.CreateChain(vs, %d);\n", this.m_count);
        log("    shape.m_prevVertex.Set(%.15f, %.15f);\n", this.m_prevVertex.x, this.m_prevVertex.y);
        log("    shape.m_nextVertex.Set(%.15f, %.15f);\n", this.m_nextVertex.x, this.m_nextVertex.y);
        log("    shape.m_hasPrevVertex = %s;\n", (this.m_hasPrevVertex) ? ("true") : ("false"));
        log("    shape.m_hasNextVertex = %s;\n", (this.m_hasNextVertex) ? ("true") : ("false"));
    };
    // #if B2_ENABLE_PARTICLE
    /// @see b2Shape::ComputeDistance
    b2ChainShape.ComputeDistance_s_edgeShape = new b2EdgeShape_1.b2EdgeShape();
    // #endif
    /// Implement b2Shape.
    b2ChainShape.RayCast_s_edgeShape = new b2EdgeShape_1.b2EdgeShape();
    /// @see b2Shape::ComputeAABB
    b2ChainShape.ComputeAABB_s_v1 = new b2Math_1.b2Vec2();
    b2ChainShape.ComputeAABB_s_v2 = new b2Math_1.b2Vec2();
    return b2ChainShape;
}(b2Shape_1.b2Shape));
exports.b2ChainShape = b2ChainShape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDaGFpblNoYXBlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvQ29sbGlzaW9uL1NoYXBlcy9iMkNoYWluU2hhcGUudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7QUFFRiw0RUFBNEU7QUFDNUUsc0RBQWtGO0FBQ2xGLDhDQUE4RDtBQUk5RCxxQ0FBaUQ7QUFDakQsNkNBQTRDO0FBRTVDLDJEQUEyRDtBQUMzRCxtRkFBbUY7QUFDbkYsNkNBQTZDO0FBQzdDLHVFQUF1RTtBQUN2RSxpRUFBaUU7QUFDakUsaUZBQWlGO0FBQ2pGO0lBQWtDLGdDQUFPO0lBUXZDO1FBQUEsWUFDRSxrQkFBTSxxQkFBVyxDQUFDLFlBQVksRUFBRSw2QkFBZ0IsQ0FBQyxTQUNsRDtRQVRNLGdCQUFVLEdBQWEsRUFBRSxDQUFDO1FBQzFCLGFBQU8sR0FBVyxDQUFDLENBQUM7UUFDWCxrQkFBWSxHQUFXLElBQUksZUFBTSxFQUFFLENBQUM7UUFDcEMsa0JBQVksR0FBVyxJQUFJLGVBQU0sRUFBRSxDQUFDO1FBQzdDLHFCQUFlLEdBQVksS0FBSyxDQUFDO1FBQ2pDLHFCQUFlLEdBQVksS0FBSyxDQUFDOztJQUl4QyxDQUFDO0lBRUQsMkRBQTJEO0lBQzNELDBEQUEwRDtJQUMxRCxpQ0FBaUM7SUFDMUIsaUNBQVUsR0FBakIsVUFBa0IsUUFBYyxFQUFFLEtBQStCLEVBQUUsS0FBaUI7UUFDbEYsK0JBQStCO1FBQy9CLG1EQUFtRDtRQUNuRCwrQ0FBK0M7UUFDL0MsMkNBQTJDO1FBQzNDLHVGQUF1RjtRQUN2Rix1RkFBdUY7UUFDdkYsV0FBVztRQVBxQixzQkFBQSxFQUFBLFFBQWdCLFFBQVEsQ0FBQyxNQUFNO1FBQUUsc0JBQUEsRUFBQSxTQUFpQjtRQVNsRixJQUFJLENBQUMsT0FBTyxHQUFHLEtBQUssR0FBRyxDQUFDLENBQUM7UUFDekIsSUFBSSxDQUFDLFVB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