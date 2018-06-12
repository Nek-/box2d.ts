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
var b2EdgeShape_1 = require("../../Collision/Shapes/b2EdgeShape");
var b2Contact_1 = require("./b2Contact");
var b2ChainAndPolygonContact = /** @class */ (function (_super) {
    __extends(b2ChainAndPolygonContact, _super);
    function b2ChainAndPolygonContact() {
        return _super.call(this) || this;
    }
    b2ChainAndPolygonContact.Create = function (allocator) {
        return new b2ChainAndPolygonContact();
    };
    b2ChainAndPolygonContact.Destroy = function (contact, allocator) {
    };
    b2ChainAndPolygonContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
        // DEBUG: b2Assert(fixtureA.GetType() === b2ShapeType.e_chainShape);
        // DEBUG: b2Assert(fixtureB.GetType() === b2ShapeType.e_polygonShape);
    };
    b2ChainAndPolygonContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA instanceof b2ChainShape);
        // DEBUG: b2Assert(shapeB instanceof b2PolygonShape);
        var chain = shapeA;
        var edge = b2ChainAndPolygonContact.Evaluate_s_edge;
        chain.GetChildEdge(edge, this.m_indexA);
        b2CollideEdge_1.b2CollideEdgeAndPolygon(manifold, edge, xfA, shapeB, xfB);
    };
    b2ChainAndPolygonContact.Evaluate_s_edge = new b2EdgeShape_1.b2EdgeShape();
    return b2ChainAndPolygonContact;
}(b2Contact_1.b2Contact));
exports.b2ChainAndPolygonContact = b2ChainAndPolygonContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDaGFpbkFuZFBvbHlnb25Db250YWN0LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvQ29udGFjdHMvYjJDaGFpbkFuZFBvbHlnb25Db250YWN0LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBS0YsK0RBQXdFO0FBSXhFLGtFQUFpRTtBQUVqRSx5Q0FBd0M7QUFHeEM7SUFBOEMsNENBQVM7SUFDckQ7ZUFDRSxpQkFBTztJQUNULENBQUM7SUFFYSwrQkFBTSxHQUFwQixVQUFxQixTQUFjO1FBQ2pDLE9BQU8sSUFBSSx3QkFBd0IsRUFBRSxDQUFDO0lBQ3hDLENBQUM7SUFFYSxnQ0FBTyxHQUFyQixVQUFzQixPQUFrQixFQUFFLFNBQWM7SUFDeEQsQ0FBQztJQUVNLHdDQUFLLEdBQVosVUFBYSxRQUFtQixFQUFFLE1BQWMsRUFBRSxRQUFtQixFQUFFLE1BQWM7UUFDbkYsaUJBQU0sS0FBSyxZQUFDLFFBQVEsRUFBRSxNQUFNLEVBQUUsUUFBUSxFQUFFLE1BQU0sQ0FBQyxDQUFDO1FBQ2hELG9FQUFvRTtRQUNwRSxzRUFBc0U7SUFDeEUsQ0FBQztJQUdNLDJDQUFRLEdBQWYsVUFBZ0IsUUFBb0IsRUFBRSxHQUFnQixFQUFFLEdBQWdCO1FBQ3RFLElBQU0sTUFBTSxHQUFZLElBQUksQ0FBQyxVQUFVLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDbkQsSUFBTSxNQUFNLEdBQVksSUFBSSxDQUFDLFVBQVUsQ0FBQyxRQUFRLEVBQUUsQ0FBQztRQUNuRCxtREFBbUQ7UUFDbkQscURBQXFEO1FBQ3JELElBQU0sS0FBSyxHQUFpQixNQUFzQixDQUFDO1FBQ25ELElBQU0sSUFBSSxHQUFnQix3QkFBd0IsQ0FBQyxlQUFlLENBQUM7UUFDbkUsS0FBSyxDQUFDLFlBQVksQ0FBQyxJQUFJLEVBQUUsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDO1FBQ3hDLHVDQUF1QixDQUNyQixRQUFRLEVBQ1IsSUFBSSxFQUFFLEdBQUcsRUFDVCxNQUF3QixFQUFFLEdBQUcsQ0FBQyxDQUFDO0lBQ25DLENBQUM7SUFiYyx3Q0FBZSxHQUFHLElBQUkseUJBQVcsRUFBRSxDQUFDO0lBY3JELCtCQUFDO0NBQUEsQUFoQ0QsQ0FBOEMscUJBQVMsR0FnQ3REO0FBaENZLDREQUF3QiJ9