"use strict";
/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2GrowableStack_1 = require("../Common/b2GrowableStack");
var b2Collision_1 = require("./b2Collision");
function verify(value) {
    if (value === null) {
        throw new Error();
    }
    return value;
}
/// A node in the dynamic tree. The client does not interact with this directly.
var b2TreeNode = /** @class */ (function () {
    function b2TreeNode(id) {
        if (id === void 0) { id = 0; }
        this.m_id = 0;
        this.aabb = new b2Collision_1.b2AABB();
        this.userData = null;
        this.parent = null; // or next
        this.child1 = null;
        this.child2 = null;
        this.height = 0; // leaf = 0, free node = -1
        this.m_id = id;
    }
    b2TreeNode.prototype.IsLeaf = function () {
        return this.child1 === null;
    };
    return b2TreeNode;
}());
exports.b2TreeNode = b2TreeNode;
var b2DynamicTree = /** @class */ (function () {
    function b2DynamicTree() {
        this.m_root = null;
        // b2TreeNode* public m_nodes;
        // int32 public m_nodeCount;
        // int32 public m_nodeCapacity;
        this.m_freeList = null;
        this.m_path = 0;
        this.m_insertionCount = 0;
    }
    b2DynamicTree.prototype.GetUserData = function (proxy) {
        // DEBUG: b2Assert(proxy !== null);
        return proxy.userData;
    };
    b2DynamicTree.prototype.GetFatAABB = function (proxy) {
        // DEBUG: b2Assert(proxy !== null);
        return proxy.aabb;
    };
    b2DynamicTree.prototype.Query = function (aabb, callback) {
        if (this.m_root === null) {
            return;
        }
        var stack = b2DynamicTree.s_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            var node = stack.Pop();
            // if (node === null) {
            //   continue;
            // }
            if (node.aabb.TestOverlap(aabb)) {
                if (node.IsLeaf()) {
                    var proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                }
                else {
                    stack.Push(verify(node.child1));
                    stack.Push(verify(node.child2));
                }
            }
        }
    };
    b2DynamicTree.prototype.QueryPoint = function (point, callback) {
        if (this.m_root === null) {
            return;
        }
        var stack = b2DynamicTree.s_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            var node = stack.Pop();
            // if (node === null) {
            //   continue;
            // }
            if (node.aabb.TestContain(point)) {
                if (node.IsLeaf()) {
                    var proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                }
                else {
                    stack.Push(verify(node.child1));
                    stack.Push(verify(node.child2));
                }
            }
        }
    };
    b2DynamicTree.prototype.RayCast = function (input, callback) {
        if (this.m_root === null) {
            return;
        }
        var p1 = input.p1;
        var p2 = input.p2;
        var r = b2Math_1.b2Vec2.SubVV(p2, p1, b2DynamicTree.s_r);
        // DEBUG: b2Assert(r.LengthSquared() > 0);
        r.Normalize();
        // v is perpendicular to the segment.
        var v = b2Math_1.b2Vec2.CrossOneV(r, b2DynamicTree.s_v);
        var abs_v = b2Math_1.b2Vec2.AbsV(v, b2DynamicTree.s_abs_v);
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        var maxFraction = input.maxFraction;
        // Build a bounding box for the segment.
        var segmentAABB = b2DynamicTree.s_segmentAABB;
        var t_x = p1.x + maxFraction * (p2.x - p1.x);
        var t_y = p1.y + maxFraction * (p2.y - p1.y);
        segmentAABB.lowerBound.x = b2Math_1.b2Min(p1.x, t_x);
        segmentAABB.lowerBound.y = b2Math_1.b2Min(p1.y, t_y);
        segmentAABB.upperBound.x = b2Math_1.b2Max(p1.x, t_x);
        segmentAABB.upperBound.y = b2Math_1.b2Max(p1.y, t_y);
        var stack = b2DynamicTree.s_stack.Reset();
        stack.Push(this.m_root);
        while (stack.GetCount() > 0) {
            var node = stack.Pop();
            // if (node === null) {
            //   continue;
            // }
            if (!b2Collision_1.b2TestOverlapAABB(node.aabb, segmentAABB)) {
                continue;
            }
            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            var c = node.aabb.GetCenter();
            var h = node.aabb.GetExtents();
            var separation = b2Math_1.b2Abs(b2Math_1.b2Vec2.DotVV(v, b2Math_1.b2Vec2.SubVV(p1, c, b2Math_1.b2Vec2.s_t0))) - b2Math_1.b2Vec2.DotVV(abs_v, h);
            if (separation > 0) {
                continue;
            }
            if (node.IsLeaf()) {
                var subInput = b2DynamicTree.s_subInput;
                subInput.p1.Copy(input.p1);
                subInput.p2.Copy(input.p2);
                subInput.maxFraction = maxFraction;
                var value = callback(subInput, node);
                if (value === 0) {
                    // The client has terminated the ray cast.
                    return;
                }
                if (value > 0) {
                    // Update segment bounding box.
                    maxFraction = value;
                    t_x = p1.x + maxFraction * (p2.x - p1.x);
                    t_y = p1.y + maxFraction * (p2.y - p1.y);
                    segmentAABB.lowerBound.x = b2Math_1.b2Min(p1.x, t_x);
                    segmentAABB.lowerBound.y = b2Math_1.b2Min(p1.y, t_y);
                    segmentAABB.upperBound.x = b2Math_1.b2Max(p1.x, t_x);
                    segmentAABB.upperBound.y = b2Math_1.b2Max(p1.y, t_y);
                }
            }
            else {
                stack.Push(verify(node.child1));
                stack.Push(verify(node.child2));
            }
        }
    };
    b2DynamicTree.prototype.AllocateNode = function () {
        // Expand the node pool as needed.
        if (this.m_freeList) {
            var node = this.m_freeList;
            this.m_freeList = node.parent; // this.m_freeList = node.next;
            node.parent = null;
            node.child1 = null;
            node.child2 = null;
            node.height = 0;
            node.userData = null;
            return node;
        }
        return new b2TreeNode(b2DynamicTree.s_node_id++);
    };
    b2DynamicTree.prototype.FreeNode = function (node) {
        node.parent = this.m_freeList; // node.next = this.m_freeList;
        node.child1 = null;
        node.child2 = null;
        node.height = -1;
        node.userData = null;
        this.m_freeList = node;
    };
    b2DynamicTree.prototype.CreateProxy = function (aabb, userData) {
        var node = this.AllocateNode();
        // Fatten the aabb.
        var r_x = b2Settings_1.b2_aabbExtension;
        var r_y = b2Settings_1.b2_aabbExtension;
        node.aabb.lowerBound.x = aabb.lowerBound.x - r_x;
        node.aabb.lowerBound.y = aabb.lowerBound.y - r_y;
        node.aabb.upperBound.x = aabb.upperBound.x + r_x;
        node.aabb.upperBound.y = aabb.upperBound.y + r_y;
        node.userData = userData;
        node.height = 0;
        this.InsertLeaf(node);
        return node;
    };
    b2DynamicTree.prototype.DestroyProxy = function (proxy) {
        // DEBUG: b2Assert(proxy.IsLeaf());
        this.RemoveLeaf(proxy);
        this.FreeNode(proxy);
    };
    b2DynamicTree.prototype.MoveProxy = function (proxy, aabb, displacement) {
        // DEBUG: b2Assert(proxy.IsLeaf());
        if (proxy.aabb.Contains(aabb)) {
            return false;
        }
        this.RemoveLeaf(proxy);
        // Extend AABB.
        // Predict AABB displacement.
        var r_x = b2Settings_1.b2_aabbExtension + b2Settings_1.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
        var r_y = b2Settings_1.b2_aabbExtension + b2Settings_1.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));
        proxy.aabb.lowerBound.x = aabb.lowerBound.x - r_x;
        proxy.aabb.lowerBound.y = aabb.lowerBound.y - r_y;
        proxy.aabb.upperBound.x = aabb.upperBound.x + r_x;
        proxy.aabb.upperBound.y = aabb.upperBound.y + r_y;
        this.InsertLeaf(proxy);
        return true;
    };
    b2DynamicTree.prototype.InsertLeaf = function (leaf) {
        ++this.m_insertionCount;
        if (this.m_root === null) {
            this.m_root = leaf;
            this.m_root.parent = null;
            return;
        }
        // Find the best sibling for this node
        var leafAABB = leaf.aabb;
        ///const center: b2Vec2 = leafAABB.GetCenter();
        var index = this.m_root;
        while (!index.IsLeaf()) {
            var child1 = verify(index.child1);
            var child2 = verify(index.child2);
            var area = index.aabb.GetPerimeter();
            var combinedAABB = b2DynamicTree.s_combinedAABB;
            combinedAABB.Combine2(index.aabb, leafAABB);
            var combinedArea = combinedAABB.GetPerimeter();
            // Cost of creating a new parent for this node and the new leaf
            var cost = 2 * combinedArea;
            // Minimum cost of pushing the leaf further down the tree
            var inheritanceCost = 2 * (combinedArea - area);
            // Cost of descending into child1
            var cost1 = void 0;
            var aabb = b2DynamicTree.s_aabb;
            var oldArea = void 0;
            var newArea = void 0;
            if (child1.IsLeaf()) {
                aabb.Combine2(leafAABB, child1.aabb);
                cost1 = aabb.GetPerimeter() + inheritanceCost;
            }
            else {
                aabb.Combine2(leafAABB, child1.aabb);
                oldArea = child1.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
                cost1 = (newArea - oldArea) + inheritanceCost;
            }
            // Cost of descending into child2
            var cost2 = void 0;
            if (child2.IsLeaf()) {
                aabb.Combine2(leafAABB, child2.aabb);
                cost2 = aabb.GetPerimeter() + inheritanceCost;
            }
            else {
                aabb.Combine2(leafAABB, child2.aabb);
                oldArea = child2.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
                cost2 = newArea - oldArea + inheritanceCost;
            }
            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2) {
                break;
            }
            // Descend
            if (cost1 < cost2) {
                index = child1;
            }
            else {
                index = child2;
            }
        }
        var sibling = index;
        // Create a parent for the siblings.
        var oldParent = sibling.parent;
        var newParent = this.AllocateNode();
        newParent.parent = oldParent;
        newParent.userData = null;
        newParent.aabb.Combine2(leafAABB, sibling.aabb);
        newParent.height = sibling.height + 1;
        if (oldParent) {
            // The sibling was not the root.
            if (oldParent.child1 === sibling) {
                oldParent.child1 = newParent;
            }
            else {
                oldParent.child2 = newParent;
            }
            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
        }
        else {
            // The sibling was the root.
            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
            this.m_root = newParent;
        }
        // Walk back up the tree fixing heights and AABBs
        var index2 = leaf.parent;
        while (index2 !== null) {
            index2 = this.Balance(index2);
            var child1 = verify(index2.child1);
            var child2 = verify(index2.child2);
            index2.height = 1 + b2Math_1.b2Max(child1.height, child2.height);
            index2.aabb.Combine2(child1.aabb, child2.aabb);
            index2 = index2.parent;
        }
        // this.Validate();
    };
    b2DynamicTree.prototype.RemoveLeaf = function (leaf) {
        if (leaf === this.m_root) {
            this.m_root = null;
            return;
        }
        var parent = verify(leaf.parent);
        var grandParent = parent && parent.parent;
        var sibling;
        if (parent.child1 === leaf) {
            sibling = verify(parent.child2);
        }
        else {
            sibling = verify(parent.child1);
        }
        if (grandParent) {
            // Destroy parent and connect sibling to grandParent.
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling;
            }
            else {
                grandParent.child2 = sibling;
            }
            sibling.parent = grandParent;
            this.FreeNode(parent);
            // Adjust ancestor bounds.
            var index = grandParent;
            while (index) {
                index = this.Balance(index);
                var child1 = verify(index.child1);
                var child2 = verify(index.child2);
                index.aabb.Combine2(child1.aabb, child2.aabb);
                index.height = 1 + b2Math_1.b2Max(child1.height, child2.height);
                index = index.parent;
            }
        }
        else {
            this.m_root = sibling;
            sibling.parent = null;
            this.FreeNode(parent);
        }
        // this.Validate();
    };
    b2DynamicTree.prototype.Balance = function (A) {
        // DEBUG: b2Assert(A !== null);
        if (A.IsLeaf() || A.height < 2) {
            return A;
        }
        var B = verify(A.child1);
        var C = verify(A.child2);
        var balance = C.height - B.height;
        // Rotate C up
        if (balance > 1) {
            var F = verify(C.child1);
            var G = verify(C.child2);
            // Swap A and C
            C.child1 = A;
            C.parent = A.parent;
            A.parent = C;
            // A's old parent should point to C
            if (C.parent !== null) {
                if (C.parent.child1 === A) {
                    C.parent.child1 = C;
                }
                else {
                    // DEBUG: b2Assert(C.parent.child2 === A);
                    C.parent.child2 = C;
                }
            }
            else {
                this.m_root = C;
            }
            // Rotate
            if (F.height > G.height) {
                C.child2 = F;
                A.child2 = G;
                G.parent = A;
                A.aabb.Combine2(B.aabb, G.aabb);
                C.aabb.Combine2(A.aabb, F.aabb);
                A.height = 1 + b2Math_1.b2Max(B.height, G.height);
                C.height = 1 + b2Math_1.b2Max(A.height, F.height);
            }
            else {
                C.child2 = G;
                A.child2 = F;
                F.parent = A;
                A.aabb.Combine2(B.aabb, F.aabb);
                C.aabb.Combine2(A.aabb, G.aabb);
                A.height = 1 + b2Math_1.b2Max(B.height, F.height);
                C.height = 1 + b2Math_1.b2Max(A.height, G.height);
            }
            return C;
        }
        // Rotate B up
        if (balance < -1) {
            var D = verify(B.child1);
            var E = verify(B.child2);
            // Swap A and B
            B.child1 = A;
            B.parent = A.parent;
            A.parent = B;
            // A's old parent should point to B
            if (B.parent !== null) {
                if (B.parent.child1 === A) {
                    B.parent.child1 = B;
                }
                else {
                    // DEBUG: b2Assert(B.parent.child2 === A);
                    B.parent.child2 = B;
                }
            }
            else {
                this.m_root = B;
            }
            // Rotate
            if (D.height > E.height) {
                B.child2 = D;
                A.child1 = E;
                E.parent = A;
                A.aabb.Combine2(C.aabb, E.aabb);
                B.aabb.Combine2(A.aabb, D.aabb);
                A.height = 1 + b2Math_1.b2Max(C.height, E.height);
                B.height = 1 + b2Math_1.b2Max(A.height, D.height);
            }
            else {
                B.child2 = E;
                A.child1 = D;
                D.parent = A;
                A.aabb.Combine2(C.aabb, D.aabb);
                B.aabb.Combine2(A.aabb, E.aabb);
                A.height = 1 + b2Math_1.b2Max(C.height, D.height);
                B.height = 1 + b2Math_1.b2Max(A.height, E.height);
            }
            return B;
        }
        return A;
    };
    b2DynamicTree.prototype.GetHeight = function () {
        if (this.m_root === null) {
            return 0;
        }
        return this.m_root.height;
    };
    b2DynamicTree.GetAreaNode = function (node) {
        if (node === null) {
            return 0;
        }
        if (node.IsLeaf()) {
            return 0;
        }
        var area = node.aabb.GetPerimeter();
        area += b2DynamicTree.GetAreaNode(node.child1);
        area += b2DynamicTree.GetAreaNode(node.child2);
        return area;
    };
    b2DynamicTree.prototype.GetAreaRatio = function () {
        if (this.m_root === null) {
            return 0;
        }
        var root = this.m_root;
        var rootArea = root.aabb.GetPerimeter();
        var totalArea = b2DynamicTree.GetAreaNode(this.m_root);
        /*
        float32 totalArea = 0.0;
        for (int32 i = 0; i < m_nodeCapacity; ++i) {
          const b2TreeNode* node = m_nodes + i;
          if (node.height < 0) {
            // Free node in pool
            continue;
          }
    
          totalArea += node.aabb.GetPerimeter();
        }
        */
        return totalArea / rootArea;
    };
    b2DynamicTree.prototype.ComputeHeightNode = function (node) {
        if (!node || node.IsLeaf()) {
            return 0;
        }
        var height1 = this.ComputeHeightNode(node.child1);
        var height2 = this.ComputeHeightNode(node.child2);
        return 1 + b2Math_1.b2Max(height1, height2);
    };
    b2DynamicTree.prototype.ComputeHeight = function () {
        var height = this.ComputeHeightNode(this.m_root);
        return height;
    };
    b2DynamicTree.prototype.ValidateStructure = function (index) {
        if (index === null) {
            return;
        }
        if (index === this.m_root) {
            // DEBUG: b2Assert(index.parent === null);
        }
        var node = index;
        if (node.IsLeaf()) {
            // DEBUG: b2Assert(node.child1 === null);
            // DEBUG: b2Assert(node.child2 === null);
            // DEBUG: b2Assert(node.height === 0);
            return;
        }
        var child1 = verify(node.child1);
        var child2 = verify(node.child2);
        // DEBUG: b2Assert(child1.parent === index);
        // DEBUG: b2Assert(child2.parent === index);
        this.ValidateStructure(child1);
        this.ValidateStructure(child2);
    };
    b2DynamicTree.prototype.ValidateMetrics = function (index) {
        if (index === null) {
            return;
        }
        var node = index;
        if (node.IsLeaf()) {
            // DEBUG: b2Assert(node.child1 === null);
            // DEBUG: b2Assert(node.child2 === null);
            // DEBUG: b2Assert(node.height === 0);
            return;
        }
        var child1 = verify(node.child1);
        var child2 = verify(node.child2);
        // DEBUG: const height1: number = child1.height;
        // DEBUG: const height2: number = child2.height;
        // DEBUG: const height: number = 1 + b2Max(height1, height2);
        // DEBUG: b2Assert(node.height === height);
        var aabb = b2DynamicTree.s_aabb;
        aabb.Combine2(child1.aabb, child2.aabb);
        // DEBUG: b2Assert(aabb.lowerBound === node.aabb.lowerBound);
        // DEBUG: b2Assert(aabb.upperBound === node.aabb.upperBound);
        this.ValidateMetrics(child1);
        this.ValidateMetrics(child2);
    };
    b2DynamicTree.prototype.Validate = function () {
        this.ValidateStructure(this.m_root);
        this.ValidateMetrics(this.m_root);
        // let freeCount: number = 0;
        var freeIndex = this.m_freeList;
        while (freeIndex !== null) {
            freeIndex = freeIndex.parent; // freeIndex = freeIndex.next;
            // ++freeCount;
        }
        // DEBUG: b2Assert(this.GetHeight() === this.ComputeHeight());
    };
    b2DynamicTree.GetMaxBalanceNode = function (node, maxBalance) {
        if (node === null) {
            return maxBalance;
        }
        if (node.height <= 1) {
            return maxBalance;
        }
        // DEBUG: b2Assert(!node.IsLeaf());
        var child1 = verify(node.child1);
        var child2 = verify(node.child2);
        var balance = b2Math_1.b2Abs(child2.height - child1.height);
        return b2Math_1.b2Max(maxBalance, balance);
    };
    b2DynamicTree.prototype.GetMaxBalance = function () {
        var maxBalance = b2DynamicTree.GetMaxBalanceNode(this.m_root, 0);
        /*
        int32 maxBalance = 0;
        for (int32 i = 0; i < m_nodeCapacity; ++i) {
          const b2TreeNode* node = m_nodes + i;
          if (node.height <= 1) {
            continue;
          }
    
          b2Assert(!node.IsLeaf());
    
          int32 child1 = node.child1;
          int32 child2 = node.child2;
          int32 balance = b2Abs(m_nodes[child2].height - m_nodes[child1].height);
          maxBalance = b2Max(maxBalance, balance);
        }
        */
        return maxBalance;
    };
    b2DynamicTree.prototype.RebuildBottomUp = function () {
        /*
        int32* nodes = (int32*)b2Alloc(m_nodeCount * sizeof(int32));
        int32 count = 0;
    
        // Build array of leaves. Free the rest.
        for (int32 i = 0; i < m_nodeCapacity; ++i) {
          if (m_nodes[i].height < 0) {
            // free node in pool
            continue;
          }
    
          if (m_nodes[i].IsLeaf()) {
            m_nodes[i].parent = b2_nullNode;
            nodes[count] = i;
            ++count;
          } else {
            FreeNode(i);
          }
        }
    
        while (count > 1) {
          float32 minCost = b2_maxFloat;
          int32 iMin = -1, jMin = -1;
          for (int32 i = 0; i < count; ++i) {
            b2AABB aabbi = m_nodes[nodes[i]].aabb;
    
            for (int32 j = i + 1; j < count; ++j) {
              b2AABB aabbj = m_nodes[nodes[j]].aabb;
              b2AABB b;
              b.Combine(aabbi, aabbj);
              float32 cost = b.GetPerimeter();
              if (cost < minCost) {
                iMin = i;
                jMin = j;
                minCost = cost;
              }
            }
          }
    
          int32 index1 = nodes[iMin];
          int32 index2 = nodes[jMin];
          b2TreeNode* child1 = m_nodes + index1;
          b2TreeNode* child2 = m_nodes + index2;
    
          int32 parentIndex = AllocateNode();
          b2TreeNode* parent = m_nodes + parentIndex;
          parent.child1 = index1;
          parent.child2 = index2;
          parent.height = 1 + b2Max(child1.height, child2.height);
          parent.aabb.Combine(child1.aabb, child2.aabb);
          parent.parent = b2_nullNode;
    
          child1.parent = parentIndex;
          child2.parent = parentIndex;
    
          nodes[jMin] = nodes[count-1];
          nodes[iMin] = parentIndex;
          --count;
        }
    
        m_root = nodes[0];
        b2Free(nodes);
        */
        this.Validate();
    };
    b2DynamicTree.ShiftOriginNode = function (node, newOrigin) {
        if (node === null) {
            return;
        }
        if (node.height <= 1) {
            return;
        }
        // DEBUG: b2Assert(!node.IsLeaf());
        var child1 = node.child1;
        var child2 = node.child2;
        b2DynamicTree.ShiftOriginNode(child1, newOrigin);
        b2DynamicTree.ShiftOriginNode(child2, newOrigin);
        node.aabb.lowerBound.SelfSub(newOrigin);
        node.aabb.upperBound.SelfSub(newOrigin);
    };
    b2DynamicTree.prototype.ShiftOrigin = function (newOrigin) {
        b2DynamicTree.ShiftOriginNode(this.m_root, newOrigin);
        /*
        // Build array of leaves. Free the rest.
        for (int32 i = 0; i < m_nodeCapacity; ++i) {
          m_nodes[i].aabb.lowerBound -= newOrigin;
          m_nodes[i].aabb.upperBound -= newOrigin;
        }
        */
    };
    b2DynamicTree.s_stack = new b2GrowableStack_1.b2GrowableStack(256);
    b2DynamicTree.s_r = new b2Math_1.b2Vec2();
    b2DynamicTree.s_v = new b2Math_1.b2Vec2();
    b2DynamicTree.s_abs_v = new b2Math_1.b2Vec2();
    b2DynamicTree.s_segmentAABB = new b2Collision_1.b2AABB();
    b2DynamicTree.s_subInput = new b2Collision_1.b2RayCastInput();
    b2DynamicTree.s_combinedAABB = new b2Collision_1.b2AABB();
    b2DynamicTree.s_aabb = new b2Collision_1.b2AABB();
    b2DynamicTree.s_node_id = 0;
    return b2DynamicTree;
}());
exports.b2DynamicTree = b2DynamicTree;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEeW5hbWljVHJlZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9iMkR5bmFtaWNUcmVlLnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7QUFFRiwwREFBMEQ7QUFDMUQsbURBQTJFO0FBQzNFLDJDQUFtRTtBQUNuRSw2REFBNEQ7QUFDNUQsNkNBQTBFO0FBRTFFLGdCQUFtQixLQUFlO0lBQ2hDLElBQUksS0FBSyxLQUFLLElBQUksRUFBRTtRQUFFLE1BQU0sSUFBSSxLQUFLLEVBQUUsQ0FBQztLQUFFO0lBQzFDLE9BQU8sS0FBSyxDQUFDO0FBQ2YsQ0FBQztBQUVELGdGQUFnRjtBQUNoRjtJQVNFLG9CQUFZLEVBQWM7UUFBZCxtQkFBQSxFQUFBLE1BQWM7UUFSbkIsU0FBSSxHQUFXLENBQUMsQ0FBQztRQUNSLFNBQUksR0FBVyxJQUFJLG9CQUFNLEVBQUUsQ0FBQztRQUNyQyxhQUFRLEdBQVEsSUFBSSxDQUFDO1FBQ3JCLFdBQU0sR0FBc0IsSUFBSSxDQUFDLENBQUMsVUFBVTtRQUM1QyxXQUFNLEdBQXNCLElBQUksQ0FBQztRQUNqQyxXQUFNLEdBQXNCLElBQUksQ0FBQztRQUNqQyxXQUFNLEdBQVcsQ0FBQyxDQUFDLENBQUMsMkJBQTJCO1FBR3BELElBQUksQ0FBQyxJQUFJLEdBQUcsRUFBRSxDQUFDO0lBQ2pCLENBQUM7SUFFTSwyQkFBTSxHQUFiO1FBQ0UsT0FBTyxJQUFJLENBQUMsTUFBTSxLQUFLLElBQUksQ0FBQztJQUM5QixDQUFDO0lBQ0gsaUJBQUM7QUFBRCxDQUFDLEFBaEJELElBZ0JDO0FBaEJZLGdDQUFVO0FBa0J2QjtJQUFBO1FBQ1MsV0FBTSxHQUFzQixJQUFJLENBQUM7UUFFeEMsOEJBQThCO1FBQzlCLDRCQUE0QjtRQUM1QiwrQkFBK0I7UUFFeEIsZUFBVSxHQUFzQixJQUFJLENBQUM7UUFFckMsV0FBTSxHQUFXLENB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