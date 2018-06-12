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
var b2CollideEdge_1 = require("../../Collision/b2CollideEdge");
var b2Contact_1 = require("./b2Contact");
var b2EdgeAndPolygonContact = /** @class */ (function (_super) {
    __extends(b2EdgeAndPolygonContact, _super);
    function b2EdgeAndPolygonContact() {
        return _super.call(this) || this;
    }
    b2EdgeAndPolygonContact.Create = function (allocator) {
        return new b2EdgeAndPolygonContact();
    };
    b2EdgeAndPolygonContact.Destroy = function (contact, allocator) {
    };
    b2EdgeAndPolygonContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
        // DEBUG: b2Assert(fixtureA.GetType() === b2ShapeType.e_edgeShape);
        // DEBUG: b2Assert(fixtureB.GetType() === b2ShapeType.e_polygonShape);
    };
    b2EdgeAndPolygonContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA instanceof b2EdgeShape);
        // DEBUG: b2Assert(shapeB instanceof b2PolygonShape);
        b2CollideEdge_1.b2CollideEdgeAndPolygon(manifold, shapeA, xfA, shapeB, xfB);
    };
    return b2EdgeAndPolygonContact;
}(b2Contact_1.b2Contact));
exports.b2EdgeAndPolygonContact = b2EdgeAndPolygonContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJFZGdlQW5kUG9seWdvbkNvbnRhY3QuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Db250YWN0cy9iMkVkZ2VBbmRQb2x5Z29uQ29udGFjdC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUtGLCtEQUF3RTtBQUt4RSx5Q0FBd0M7QUFHeEM7SUFBNkMsMkNBQVM7SUFDcEQ7ZUFDRSxpQkFBTztJQUNULENBQUM7SUFFYSw4QkFBTSxHQUFwQixVQUFxQixTQUFjO1FBQ2pDLE9BQU8sSUFBSSx1QkFBdUIsRUFBRSxDQUFDO0lBQ3ZDLENBQUM7SUFFYSwrQkFBTyxHQUFyQixVQUFzQixPQUFrQixFQUFFLFNBQWM7SUFDeEQsQ0FBQztJQUVNLHVDQUFLLEdBQVosVUFBYSxRQUFtQixFQUFFLE1BQWMsRUFBRSxRQUFtQixFQUFFLE1BQWM7UUFDbkYsaUJBQU0sS0FBSyxZQUFDLFFBQVEsRUFBRSxNQUFNLEVBQUUsUUFBUSxFQUFFLE1BQU0sQ0FBQyxDQUFDO1FBQ2hELG1FQUFtRTtRQUNuRSxzRUFBc0U7SUFDeEUsQ0FBQztJQUVNLDBDQUFRLEdBQWYsVUFBZ0IsUUFBb0IsRUFBRSxHQUFnQixFQUFFLEdBQWdCO1FBQ3RFLElBQU0sTUFBTSxHQUFZLElBQUksQ0FBQyxVQUFVLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDbkQsSUFBTSxNQUFNLEdBQVksSUFBSSxDQUFDLFVBQVUsQ0FBQyxRQUFRLEVBQUUsQ0FBQztRQUNuRCxrREFBa0Q7UUFDbEQscURBQXFEO1FBQ3JELHVDQUF1QixDQUNyQixRQUFRLEVBQ1IsTUFBcUIsRUFBRSxHQUFHLEVBQzFCLE1BQXdCLEVBQUUsR0FBRyxDQUFDLENBQUM7SUFDbkMsQ0FBQztJQUNILDhCQUFDO0FBQUQsQ0FBQyxBQTVCRCxDQUE2QyxxQkFBUyxHQTRCckQ7QUE1QlksMERBQXVCIn0=