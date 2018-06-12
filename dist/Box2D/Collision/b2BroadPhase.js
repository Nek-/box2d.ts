"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
var b2Collision_1 = require("./b2Collision");
var b2DynamicTree_1 = require("./b2DynamicTree");
var b2Pair = /** @class */ (function () {
    function b2Pair(proxyA, proxyB) {
        this.proxyA = proxyA;
        this.proxyB = proxyB;
    }
    return b2Pair;
}());
exports.b2Pair = b2Pair;
/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
var b2BroadPhase = /** @class */ (function () {
    function b2BroadPhase() {
        this.m_tree = new b2DynamicTree_1.b2DynamicTree();
        this.m_proxyCount = 0;
        // public m_moveCapacity: number = 16;
        this.m_moveCount = 0;
        this.m_moveBuffer = [];
        // public m_pairCapacity: number = 16;
        this.m_pairCount = 0;
        this.m_pairBuffer = [];
    }
    // public m_queryProxyId: number = 0;
    /// Create a proxy with an initial AABB. Pairs are not reported until
    /// UpdatePairs is called.
    b2BroadPhase.prototype.CreateProxy = function (aabb, userData) {
        var proxy = this.m_tree.CreateProxy(aabb, userData);
        ++this.m_proxyCount;
        this.BufferMove(proxy);
        return proxy;
    };
    /// Destroy a proxy. It is up to the client to remove any pairs.
    b2BroadPhase.prototype.DestroyProxy = function (proxy) {
        this.UnBufferMove(proxy);
        --this.m_proxyCount;
        this.m_tree.DestroyProxy(proxy);
    };
    /// Call MoveProxy as many times as you like, then when you are done
    /// call UpdatePairs to finalized the proxy pairs (for your time step).
    b2BroadPhase.prototype.MoveProxy = function (proxy, aabb, displacement) {
        var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
        if (buffer) {
            this.BufferMove(proxy);
        }
    };
    /// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
    b2BroadPhase.prototype.TouchProxy = function (proxy) {
        this.BufferMove(proxy);
    };
    /// Get the fat AABB for a proxy.
    b2BroadPhase.prototype.GetFatAABB = function (proxy) {
        return this.m_tree.GetFatAABB(proxy);
    };
    /// Get user data from a proxy. Returns NULL if the id is invalid.
    b2BroadPhase.prototype.GetUserData = function (proxy) {
        return this.m_tree.GetUserData(proxy);
    };
    /// Test overlap of fat AABBs.
    b2BroadPhase.prototype.TestOverlap = function (proxyA, proxyB) {
        var aabbA = this.m_tree.GetFatAABB(proxyA);
        var aabbB = this.m_tree.GetFatAABB(proxyB);
        return b2Collision_1.b2TestOverlapAABB(aabbA, aabbB);
    };
    /// Get the number of proxies.
    b2BroadPhase.prototype.GetProxyCount = function () {
        return this.m_proxyCount;
    };
    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    b2BroadPhase.prototype.UpdatePairs = function (contactManager) {
        var _this = this;
        // Reset pair buffer
        this.m_pairCount = 0;
        var _loop_1 = function (i_1) {
            var queryProxy = this_1.m_moveBuffer[i_1];
            if (queryProxy === null) {
                return "continue";
            }
            // This is called from box2d.b2DynamicTree::Query when we are gathering pairs.
            // boolean b2BroadPhase::QueryCallback(int32 proxyId);
            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            var fatAABB = this_1.m_tree.GetFatAABB(queryProxy);
            // Query tree, create pairs and add them pair buffer.
            this_1.m_tree.Query(fatAABB, function (proxy) {
                // A proxy cannot form a pair with itself.
                if (proxy.m_id === queryProxy.m_id) {
                    return true;
                }
                // const proxyA = proxy < queryProxy ? proxy : queryProxy;
                // const proxyB = proxy >= queryProxy ? proxy : queryProxy;
                var proxyA;
                var proxyB;
                if (proxy.m_id < queryProxy.m_id) {
                    proxyA = proxy;
                    proxyB = queryProxy;
                }
                else {
                    proxyA = queryProxy;
                    proxyB = proxy;
                }
                // Grow the pair buffer as needed.
                if (_this.m_pairCount === _this.m_pairBuffer.length) {
                    _this.m_pairBuffer[_this.m_pairCount] = new b2Pair(proxyA, proxyB);
                }
                else {
                    var pair = _this.m_pairBuffer[_this.m_pairCount];
                    pair.proxyA = proxyA;
                    pair.proxyB = proxyB;
                }
                ++_this.m_pairCount;
                return true;
            });
        };
        var this_1 = this;
        // Perform tree queries for all moving proxies.
        for (var i_1 = 0; i_1 < this.m_moveCount; ++i_1) {
            _loop_1(i_1);
        }
        // Reset move buffer
        this.m_moveCount = 0;
        // Sort the pair buffer to expose duplicates.
        this.m_pairBuffer.length = this.m_pairCount;
        this.m_pairBuffer.sort(b2PairLessThan);
        // Send the pairs back to the client.
        var i = 0;
        while (i < this.m_pairCount) {
            var primaryPair = this.m_pairBuffer[i];
            var userDataA = this.m_tree.GetUserData(primaryPair.proxyA);
            var userDataB = this.m_tree.GetUserData(primaryPair.proxyB);
            contactManager.AddPair(userDataA, userDataB);
            ++i;
            // Skip any duplicate pairs.
            while (i < this.m_pairCount) {
                var pair = this.m_pairBuffer[i];
                if (pair.proxyA.m_id !== primaryPair.proxyA.m_id || pair.proxyB.m_id !== primaryPair.proxyB.m_id) {
                    break;
                }
                ++i;
            }
        }
        // Try to keep the tree balanced.
        // this.m_tree.Rebalance(4);
    };
    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    b2BroadPhase.prototype.Query = function (aabb, callback) {
        this.m_tree.Query(aabb, callback);
    };
    b2BroadPhase.prototype.QueryPoint = function (point, callback) {
        this.m_tree.QueryPoint(point, callback);
    };
    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback a callback class that is called for each proxy that is hit by the ray.
    b2BroadPhase.prototype.RayCast = function (input, callback) {
        this.m_tree.RayCast(input, callback);
    };
    /// Get the height of the embedded tree.
    b2BroadPhase.prototype.GetTreeHeight = function () {
        return this.m_tree.GetHeight();
    };
    /// Get the balance of the embedded tree.
    b2BroadPhase.prototype.GetTreeBalance = function () {
        return this.m_tree.GetMaxBalance();
    };
    /// Get the quality metric of the embedded tree.
    b2BroadPhase.prototype.GetTreeQuality = function () {
        return this.m_tree.GetAreaRatio();
    };
    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    b2BroadPhase.prototype.ShiftOrigin = function (newOrigin) {
        this.m_tree.ShiftOrigin(newOrigin);
    };
    b2BroadPhase.prototype.BufferMove = function (proxy) {
        this.m_moveBuffer[this.m_moveCount] = proxy;
        ++this.m_moveCount;
    };
    b2BroadPhase.prototype.UnBufferMove = function (proxy) {
        var i = this.m_moveBuffer.indexOf(proxy);
        this.m_moveBuffer[i] = null;
    };
    return b2BroadPhase;
}());
exports.b2BroadPhase = b2BroadPhase;
/// This is used to sort pairs.
function b2PairLessThan(pair1, pair2) {
    if (pair1.proxyA.m_id === pair2.proxyA.m_id) {
        return pair1.proxyB.m_id - pair2.proxyB.m_id;
    }
    return pair1.proxyA.m_id - pair2.proxyA.m_id;
}
exports.b2PairLessThan = b2PairLessThan;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJCcm9hZFBoYXNlLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvQ29sbGlzaW9uL2IyQnJvYWRQaGFzZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBR0YsNkNBQTBFO0FBQzFFLGlEQUE0RDtBQUk1RDtJQUNFLGdCQUFtQixNQUFrQixFQUFTLE1BQWtCO1FBQTdDLFdBQU0sR0FBTixNQUFNLENBQVk7UUFBUyxXQUFNLEdBQU4sTUFBTSxDQUFZO0lBQUcsQ0FBQztJQUN0RSxhQUFDO0FBQUQsQ0FBQyxBQUZELElBRUM7QUFGWSx3QkFBTTtBQUluQiw0RkFBNEY7QUFDNUYseUZBQXlGO0FBQ3pGLG9GQUFvRjtBQUNwRjtJQUFBO1FBQ2tCLFdBQU0sR0FBa0IsSUFBSSw2QkFBYSxFQUFFLENBQUM7UUFDckQsaUJBQVksR0FBVyxDQUFDLENBQUM7UUFDaEMsc0NBQXNDO1FBQy9CLGdCQUFXLEdBQVcsQ0FBQyxDQUFDO1FBQ3hCLGlCQUFZLEdBQTZCLEVBQUUsQ0FBQztRQUNuRCxzQ0FBc0M7UUFDL0IsZ0JBQVcsR0FBVyxDQUFDLENBQUM7UUFDeEIsaUJBQVksR0FBYSxFQUFFLENBQUM7SUErTHJDLENBQUM7SUE5TEMscUNBQXFDO0lBRXJDLHFFQUFxRTtJQUNyRSwwQkFBMEI7SUFDbkIsa0NBQVcsR0FBbEIsVUFBbUIsSUFBWSxFQUFFLFFBQWE7UUFDNUMsSUFBTSxLQUFLLEdBQWUsSUFBSSxDQUFDLE1BQU0sQ0FBQyxXQUFXLENBQUMsSUFBSSxFQUFFLFFBQVEsQ0FBQyxDQUFDO1FBQ2xFLEVBQUUsSUFBSSxDQUFDLFlBQVksQ0FBQztRQUNwQixJQUFJLENBQUMsVUFBVSxDQUFDLEtBQUssQ0FBQyxDQUFDO1FBQ3ZCLE9BQU8sS0FBSyxDQUFDO0lBQ2YsQ0FBQztJQUVELGdFQUFnRTtJQUN6RCxtQ0FBWSxHQUFuQixVQUFvQixLQUFpQjtRQUNuQyxJQUFJ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