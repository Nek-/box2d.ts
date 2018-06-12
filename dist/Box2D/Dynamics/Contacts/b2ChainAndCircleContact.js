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
var b2ChainAndCircleContact = /** @class */ (function (_super) {
    __extends(b2ChainAndCircleContact, _super);
    function b2ChainAndCircleContact() {
        return _super.call(this) || this;
    }
    b2ChainAndCircleContact.Create = function (allocator) {
        return new b2ChainAndCircleContact();
    };
    b2ChainAndCircleContact.Destroy = function (contact, allocator) {
    };
    b2ChainAndCircleContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
        // DEBUG: b2Assert(fixtureA.GetType() === b2ShapeType.e_chainShape);
        // DEBUG: b2Assert(fixtureB.GetType() === b2ShapeType.e_circleShape);
    };
    b2ChainAndCircleContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA instanceof b2ChainShape);
        // DEBUG: b2Assert(shapeB instanceof b2CircleShape);
        var chain = shapeA;
        var edge = b2ChainAndCircleContact.Evaluate_s_edge;
        chain.GetChildEdge(edge, this.m_indexA);
        b2CollideEdge_1.b2CollideEdgeAndCircle(manifold, edge, xfA, shapeB, xfB);
    };
    b2ChainAndCircleContact.Evaluate_s_edge = new b2EdgeShape_1.b2EdgeShape();
    return b2ChainAndCircleContact;
}(b2Contact_1.b2Contact));
exports.b2ChainAndCircleContact = b2ChainAndCircleContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDaGFpbkFuZENpcmNsZUNvbnRhY3QuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlcyI6WyIuLi8uLi8uLi8uLi9Cb3gyRC9Cb3gyRC9EeW5hbWljcy9Db250YWN0cy9iMkNoYWluQW5kQ2lyY2xlQ29udGFjdC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7Ozs7Ozs7Ozs7OztBQUtGLCtEQUF1RTtBQUt2RSxrRUFBaUU7QUFDakUseUNBQXdDO0FBR3hDO0lBQTZDLDJDQUFTO0lBQ3BEO2VBQ0UsaUJBQU87SUFDVCxDQUFDO0lBRWEsOEJBQU0sR0FBcEIsVUFBcUIsU0FBYztRQUNqQyxPQUFPLElBQUksdUJBQXVCLEVBQUUsQ0FBQztJQUN2QyxDQUFDO0lBRWEsK0JBQU8sR0FBckIsVUFBc0IsT0FBa0IsRUFBRSxTQUFjO0lBQ3hELENBQUM7SUFFTSx1Q0FBSyxHQUFaLFVBQWEsUUFBbUIsRUFBRSxNQUFjLEVBQUUsUUFBbUIsRUFBRSxNQUFjO1FBQ25GLGlCQUFNLEtBQUssWUFBQyxRQUFRLEVBQUUsTUFBTSxFQUFFLFFBQVEsRUFBRSxNQUFNLENBQUMsQ0FBQztRQUNoRCxvRUFBb0U7UUFDcEUscUVBQXFFO0lBQ3ZFLENBQUM7SUFHTSwwQ0FBUSxHQUFmLFVBQWdCLFFBQW9CLEVBQUUsR0FBZ0IsRUFBRSxHQUFnQjtRQUN0RSxJQUFNLE1BQU0sR0FBWSxJQUFJLENBQUMsVUFBVSxDQUFDLFFBQVEsRUFBRSxDQUFDO1FBQ25ELElBQU0sTUFBTSxHQUFZLElBQUksQ0FBQyxVQUFVLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDbkQsbURBQW1EO1FBQ25ELG9EQUFvRDtRQUNwRCxJQUFNLEtBQUssR0FBaUIsTUFBc0IsQ0FBQztRQUNuRCxJQUFNLElBQUksR0FBZ0IsdUJBQXVCLENBQUMsZUFBZSxDQUFDO1FBQ2xFLEtBQUssQ0FBQyxZQUFZLENBQUMsSUFBSSxFQUFFLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQztRQUN4QyxzQ0FBc0IsQ0FDcEIsUUFBUSxFQUNSLElBQUksRUFBRSxHQUFHLEVBQ1QsTUFBdUIsRUFBRSxHQUFHLENBQUMsQ0FBQztJQUNsQyxDQUFDO0lBYmMsdUNBQWUsR0FBRyxJQUFJLHlCQUFXLEVBQUUsQ0FBQztJQWNyRCw4QkFBQztDQUFBLEFBaENELENBQTZDLHFCQUFTLEdBZ0NyRDtBQWhDWSwwREFBdUIifQ==