"use strict";
/*
* Copyright (c) 2010 Erin Catto http://www.box2d.org
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
// DEBUG: import { b2Assert } from "./b2Settings";
var b2Settings_1 = require("./b2Settings");
/// This is a growable LIFO stack with an initial capacity of N.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
var b2GrowableStack = /** @class */ (function () {
    function b2GrowableStack(N) {
        this.m_stack = [];
        this.m_count = 0;
        this.m_stack = b2Settings_1.b2MakeArray(N, function (index) { return null; });
        this.m_count = 0;
    }
    b2GrowableStack.prototype.Reset = function () {
        this.m_count = 0;
        return this;
    };
    b2GrowableStack.prototype.Push = function (element) {
        this.m_stack[this.m_count] = element;
        this.m_count++;
    };
    b2GrowableStack.prototype.Pop = function () {
        // DEBUG: b2Assert(this.m_count > 0);
        this.m_count--;
        var element = this.m_stack[this.m_count];
        this.m_stack[this.m_count] = null;
        if (element === null) {
            throw new Error();
        }
        return element;
    };
    b2GrowableStack.prototype.GetCount = function () {
        return this.m_count;
    };
    return b2GrowableStack;
}());
exports.b2GrowableStack = b2GrowableStack;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJHcm93YWJsZVN0YWNrLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvQ29tbW9uL2IyR3Jvd2FibGVTdGFjay50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBRUYsa0RBQWtEO0FBQ2xELDJDQUEyQztBQUUzQyxnRUFBZ0U7QUFDaEUsb0VBQW9FO0FBQ3BFLHNDQUFzQztBQUV0QztJQUlFLHlCQUFZLENBQVM7UUFIZCxZQUFPLEdBQW9CLEVBQUUsQ0FBQztRQUM5QixZQUFPLEdBQVcsQ0FBQyxDQUFDO1FBR3pCLElBQUksQ0FBQyxPQUFPLEdBQUcsd0JBQVcsQ0FBQyxDQUFDLEVBQUUsVUFBQyxLQUFLLElBQUssT0FBQSxJQUFJLEVBQUosQ0FBSSxDQUFDLENBQUM7UUFDL0MsSUFBSSxDQUFDLE9BQU8sR0FBRyxDQUFDLENBQUM7SUFDbkIsQ0FBQztJQUVNLCtCQUFLLEdBQVo7UUFDRSxJQUFJLENBQUMsT0FBTyxHQUFHLENBQUMsQ0FBQztRQUNqQixPQUFPLElBQUksQ0FBQztJQUNkLENBQUM7SUFFTSw4QkFBSSxHQUFYLFVBQVksT0FBVTtRQUNwQixJQUFJLENBQUMsT0FBTyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsR0FBRyxPQUFPLENBQUM7UUFDckMsSUFBSSxDQUFDLE9BQU8sRUFBRSxDQUFDO0lBQ2pCLENBQUM7SUFFTSw2QkFBRyxHQUFWO1FBQ0UscUNBQXFDO1FBQ3JDLElBQUksQ0FBQyxPQUFPLEVBQUUsQ0FBQztRQUNmLElBQU0sT0FBTyxHQUFhLElBQUksQ0FBQyxPQUFPLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO1FBQ3JELElBQUksQ0FBQyxPQUFPLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxHQUFHLElBQUksQ0FBQztRQUNsQyxJQUFJLE9BQU8sS0FBSyxJQUFJLEVBQUU7WUFBRSxNQUFNLElBQUksS0FBSyxFQUFFLENBQUM7U0FBRTtRQUM1QyxPQUFPLE9BQU8sQ0FBQztJQUNqQixDQUFDO0lBRU0sa0NBQVEsR0FBZjtRQUNFLE9BQU8sSUFBSSxDQUFDLE9BQU8sQ0FBQztJQUN0QixDQUFDO0lBQ0gsc0JBQUM7QUFBRCxDQUFDLEFBL0JELElBK0JDO0FBL0JZLDBDQUFlIn0=