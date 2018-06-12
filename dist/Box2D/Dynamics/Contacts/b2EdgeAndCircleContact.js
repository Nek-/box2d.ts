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
var b2EdgeAndCircleContact = /** @class */ (function (_super) {
    __extends(b2EdgeAndCircleContact, _super);
    function b2EdgeAndCircleContact() {
        return _super.call(this) || this;
    }
    b2EdgeAndCircleContact.Create = function (allocator) {
        return new b2EdgeAndCircleContact();
    };
    b2EdgeAndCircleContact.Destroy = function (contact, allocator) {
    };
    b2EdgeAndCircleContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
        // DEBUG: b2Assert(fixtureA.GetType() === b2ShapeType.e_edgeShape);
        // DEBUG: b2Assert(fixtureB.GetType() === b2ShapeType.e_circleShape);
    };
    b2EdgeAndCircleContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA instanceof b2EdgeShape);
        // DEBUG: b2Assert(shapeB instanceof b2CircleShape);
        b2CollideEdge_1.b2CollideEdgeAndCircle(manifold, shapeA, xfA, shapeB, xfB);
    };
    return b2EdgeAndCircleContact;
}(b2Contact_1.b2Contact));
exports.b2EdgeAndCircleContact = b2EdgeAndCircleContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJFZGdlQW5kQ2lyY2xlQ29udGFjdC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0NvbnRhY3RzL2IyRWRnZUFuZENpcmNsZUNvbnRhY3QudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7QUFLRiwrREFBdUU7QUFLdkUseUNBQXdDO0FBR3hDO0lBQTRDLDBDQUFTO0lBQ25EO2VBQ0UsaUJBQU87SUFDVCxDQUFDO0lBRWEsNkJBQU0sR0FBcEIsVUFBcUIsU0FBYztRQUNqQyxPQUFPLElBQUksc0JBQXNCLEVBQUUsQ0FBQztJQUN0QyxDQUFDO0lBRWEsOEJBQU8sR0FBckIsVUFBc0IsT0FBa0IsRUFBRSxTQUFjO0lBQ3hELENBQUM7SUFFTSxzQ0FBSyxHQUFaLFVBQWEsUUFBbUIsRUFBRSxNQUFjLEVBQUUsUUFBbUIsRUFBRSxNQUFjO1FBQ25GLGlCQUFNLEtBQUssWUFBQyxRQUFRLEVBQUUsTUFBTSxFQUFFLFFBQVEsRUFBRSxNQUFNLENBQUMsQ0FBQztRQUNoRCxtRUFBbUU7UUFDbkUscUVBQXFFO0lBQ3ZFLENBQUM7SUFFTSx5Q0FBUSxHQUFmLFVBQWdCLFFBQW9CLEVBQUUsR0FBZ0IsRUFBRSxHQUFnQjtRQUN0RSxJQUFNLE1BQU0sR0FBWSxJQUFJLENBQUMsVUFBVSxDQUFDLFFBQVEsRUFBRSxDQUFDO1FBQ25ELElBQU0sTUFBTSxHQUFZLElBQUksQ0FBQyxVQUFVLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDbkQsa0RBQWtEO1FBQ2xELG9EQUFvRDtRQUNwRCxzQ0FBc0IsQ0FDcEIsUUFBUSxFQUNSLE1BQXFCLEVBQUUsR0FBRyxFQUMxQixNQUF1QixFQUFFLEdBQUcsQ0FBQyxDQUFDO0lBQ2xDLENBQUM7SUFDSCw2QkFBQztBQUFELENBQUMsQUE1QkQsQ0FBNEMscUJBQVMsR0E0QnBEO0FBNUJZLHdEQUFzQiJ9