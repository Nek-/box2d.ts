"use strict";
/*
 * Copyright (c) 2013 Google, Inc.
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
// #if B2_ENABLE_PARTICLE
// DEBUG: import { b2Assert } from "../Common/b2Settings";
var b2Settings_1 = require("../Common/b2Settings");
var b2Math_1 = require("../Common/b2Math");
var b2StackQueue_1 = require("./b2StackQueue");
/**
 * A field representing the nearest generator from each point.
 */
var b2VoronoiDiagram = /** @class */ (function () {
    function b2VoronoiDiagram(generatorCapacity) {
        this.m_generatorCapacity = 0;
        this.m_generatorCount = 0;
        this.m_countX = 0;
        this.m_countY = 0;
        this.m_diagram = [];
        this.m_generatorBuffer = b2Settings_1.b2MakeArray(generatorCapacity, function (index) { return new b2VoronoiDiagram.Generator(); });
        this.m_generatorCapacity = generatorCapacity;
    }
    /**
     * Add a generator.
     *
     * @param center the position of the generator.
     * @param tag a tag used to identify the generator in callback functions.
     * @param necessary whether to callback for nodes associated with the generator.
     */
    b2VoronoiDiagram.prototype.AddGenerator = function (center, tag, necessary) {
        // DEBUG: b2Assert(this.m_generatorCount < this.m_generatorCapacity);
        var g = this.m_generatorBuffer[this.m_generatorCount++];
        g.center.Copy(center);
        g.tag = tag;
        g.necessary = necessary;
    };
    /**
     * Generate the Voronoi diagram. It is rasterized with a given
     * interval in the same range as the necessary generators exist.
     *
     * @param radius the interval of the diagram.
     * @param margin margin for which the range of the diagram is extended.
     */
    b2VoronoiDiagram.prototype.Generate = function (radius, margin) {
        var inverseRadius = 1 / radius;
        var lower = new b2Math_1.b2Vec2(+b2Settings_1.b2_maxFloat, +b2Settings_1.b2_maxFloat);
        var upper = new b2Math_1.b2Vec2(-b2Settings_1.b2_maxFloat, -b2Settings_1.b2_maxFloat);
        var necessary_count = 0;
        for (var k = 0; k < this.m_generatorCount; k++) {
            var g = this.m_generatorBuffer[k];
            if (g.necessary) {
                b2Math_1.b2Vec2.MinV(lower, g.center, lower);
                b2Math_1.b2Vec2.MaxV(upper, g.center, upper);
                ++necessary_count;
            }
        }
        if (necessary_count === 0) {
            ///debugger;
            this.m_countX = 0;
            this.m_countY = 0;
            return;
        }
        lower.x -= margin;
        lower.y -= margin;
        upper.x += margin;
        upper.y += margin;
        this.m_countX = 1 + Math.floor(inverseRadius * (upper.x - lower.x));
        this.m_countY = 1 + Math.floor(inverseRadius * (upper.y - lower.y));
        ///  m_diagram = (Generator**) m_allocator->Allocate(sizeof(Generator*) * m_countX * m_countY);
        ///  for (int32 i = 0; i < m_countX * m_countY; i++)
        ///  {
        ///    m_diagram[i] = NULL;
        ///  }
        this.m_diagram = []; // b2MakeArray(this.m_countX * this.m_countY, (index) => null);
        // (4 * m_countX * m_countY) is the queue capacity that is experimentally
        // known to be necessary and sufficient for general particle distributions.
        var queue = new b2StackQueue_1.b2StackQueue(4 * this.m_countX * this.m_countY);
        for (var k = 0; k < this.m_generatorCount; k++) {
            var g = this.m_generatorBuffer[k];
            ///  g.center = inverseRadius * (g.center - lower);
            g.center.SelfSub(lower).SelfMul(inverseRadius);
            var x = Math.floor(g.center.x);
            var y = Math.floor(g.center.y);
            if (x >= 0 && y >= 0 && x < this.m_countX && y < this.m_countY) {
                queue.Push(new b2VoronoiDiagram.Task(x, y, x + y * this.m_countX, g));
            }
        }
        while (!queue.Empty()) {
            var task = queue.Front();
            var x = task.m_x;
            var y = task.m_y;
            var i = task.m_i;
            var g = task.m_generator;
            queue.Pop();
            if (!this.m_diagram[i]) {
                this.m_diagram[i] = g;
                if (x > 0) {
                    queue.Push(new b2VoronoiDiagram.Task(x - 1, y, i - 1, g));
                }
                if (y > 0) {
                    queue.Push(new b2VoronoiDiagram.Task(x, y - 1, i - this.m_countX, g));
                }
                if (x < this.m_countX - 1) {
                    queue.Push(new b2VoronoiDiagram.Task(x + 1, y, i + 1, g));
                }
                if (y < this.m_countY - 1) {
                    queue.Push(new b2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, g));
                }
            }
        }
        for (var y = 0; y < this.m_countY; y++) {
            for (var x = 0; x < this.m_countX - 1; x++) {
                var i = x + y * this.m_countX;
                var a = this.m_diagram[i];
                var b = this.m_diagram[i + 1];
                if (a !== b) {
                    queue.Push(new b2VoronoiDiagram.Task(x, y, i, b));
                    queue.Push(new b2VoronoiDiagram.Task(x + 1, y, i + 1, a));
                }
            }
        }
        for (var y = 0; y < this.m_countY - 1; y++) {
            for (var x = 0; x < this.m_countX; x++) {
                var i = x + y * this.m_countX;
                var a = this.m_diagram[i];
                var b = this.m_diagram[i + this.m_countX];
                if (a !== b) {
                    queue.Push(new b2VoronoiDiagram.Task(x, y, i, b));
                    queue.Push(new b2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, a));
                }
            }
        }
        while (!queue.Empty()) {
            var task = queue.Front();
            var x = task.m_x;
            var y = task.m_y;
            var i = task.m_i;
            var k = task.m_generator;
            queue.Pop();
            var a = this.m_diagram[i];
            var b = k;
            if (a !== b) {
                var ax = a.center.x - x;
                var ay = a.center.y - y;
                var bx = b.center.x - x;
                var by = b.center.y - y;
                var a2 = ax * ax + ay * ay;
                var b2 = bx * bx + by * by;
                if (a2 > b2) {
                    this.m_diagram[i] = b;
                    if (x > 0) {
                        queue.Push(new b2VoronoiDiagram.Task(x - 1, y, i - 1, b));
                    }
                    if (y > 0) {
                        queue.Push(new b2VoronoiDiagram.Task(x, y - 1, i - this.m_countX, b));
                    }
                    if (x < this.m_countX - 1) {
                        queue.Push(new b2VoronoiDiagram.Task(x + 1, y, i + 1, b));
                    }
                    if (y < this.m_countY - 1) {
                        queue.Push(new b2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, b));
                    }
                }
            }
        }
    };
    /**
     * Enumerate all nodes that contain at least one necessary
     * generator.
     */
    b2VoronoiDiagram.prototype.GetNodes = function (callback) {
        for (var y = 0; y < this.m_countY - 1; y++) {
            for (var x = 0; x < this.m_countX - 1; x++) {
                var i = x + y * this.m_countX;
                var a = this.m_diagram[i];
                var b = this.m_diagram[i + 1];
                var c = this.m_diagram[i + this.m_countX];
                var d = this.m_diagram[i + 1 + this.m_countX];
                if (b !== c) {
                    if (a !== b && a !== c &&
                        (a.necessary || b.necessary || c.necessary)) {
                        callback(a.tag, b.tag, c.tag);
                    }
                    if (d !== b && d !== c &&
                        (a.necessary || b.necessary || c.necessary)) {
                        callback(b.tag, d.tag, c.tag);
                    }
                }
            }
        }
    };
    return b2VoronoiDiagram;
}());
exports.b2VoronoiDiagram = b2VoronoiDiagram;
(function (b2VoronoiDiagram) {
    var Generator = /** @class */ (function () {
        function Generator() {
            this.center = new b2Math_1.b2Vec2();
            this.tag = 0;
            this.necessary = false;
        }
        return Generator;
    }());
    b2VoronoiDiagram.Generator = Generator;
    var Task = /** @class */ (function () {
        function Task(x, y, i, g) {
            this.m_x = x;
            this.m_y = y;
            this.m_i = i;
            this.m_generator = g;
        }
        return Task;
    }());
    b2VoronoiDiagram.Task = Task;
})(b2VoronoiDiagram = exports.b2VoronoiDiagram || (exports.b2VoronoiDiagram = {})); // namespace b2VoronoiDiagram
exports.b2VoronoiDiagram = b2VoronoiDiagram;
// #endif
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJWb3Jvbm9pRGlhZ3JhbS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0JveDJEL0JveDJEL1BhcnRpY2xlL2IyVm9yb25vaURpYWdyYW0udHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0dBZ0JHOztBQUVILHlCQUF5QjtBQUV6QiwwREFBMEQ7QUFDMUQsbURBQWdFO0FBQ2hFLDJDQUEwQztBQUMxQywrQ0FBOEM7QUFFOUM7O0dBRUc7QUFDSDtJQVFFLDBCQUFZLGlCQUF5QjtRQU45Qix3QkFBbUIsR0FBRyxDQUFDLENBQUM7UUFDeEIscUJBQWdCLEdBQUcsQ0FBQyxDQUFDO1FBQ3JCLGFBQVEsR0FBRyxDQUFDLENBQUM7UUFDYixhQUFRLEdBQUcsQ0FBQyxDQUFDO1FBQ2IsY0FBUyxHQUFpQyxFQUFFLENBQUM7UUFHbEQsSUFBSSxDQUFDLGlCQUFpQixHQUFHLHdCQUFXLENBQUMsaUJBQWlCLEVBQUUsVUFBQyxLQUFLLElBQUssT0FBQSxJQUFJLGdCQUFnQixDQUFDLFNBQVMsRUFBRSxFQUFoQyxDQUFnQyxDQUFDLENBQUM7UUFDckcsSUFBSSxDQUFDLG1CQUFtQixHQUFHLGlCQUFpQixDQUFDO0lBQy9DLENBQUM7SUFFRDs7Ozs7O09BTUc7SUFDSSx1Q0FBWSxHQUFuQixVQUFvQixNQUFjLEVBQUUsR0FBVyxFQUFFLFNBQWtCO1FBQ2pFLHFFQUFxRTtRQUNyRSxJQUFNLENBQUMsR0FBRyxJQUFJLENBQUMsaUJBQWlCLENBQUMsSUFBSSxDQUFDLGdCQUFnQixFQUFFLENBQUMsQ0FBQztRQUMxRCxDQUFDLENBQUMsTUFBTSxDQUFDLElBQUksQ0FBQyxNQUFNLENBQUMsQ0FBQztRQUN0QixDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQztRQUNaLENBQUMsQ0FBQyxTQUFTLEdBQUcsU0FBUyxDQUFDO0lBQzFCLENBQUM7SUFFRDs7Ozs7O09BTUc7SUFDSSxtQ0FBUSxHQUFmLFVBQWdCLE1B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