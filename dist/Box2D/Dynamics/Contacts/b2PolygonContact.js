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
var b2CollidePolygon_1 = require("../../Collision/b2CollidePolygon");
var b2Contact_1 = require("./b2Contact");
var b2PolygonContact = /** @class */ (function (_super) {
    __extends(b2PolygonContact, _super);
    function b2PolygonContact() {
        return _super.call(this) || this;
    }
    b2PolygonContact.Create = function (allocator) {
        return new b2PolygonContact();
    };
    b2PolygonContact.Destroy = function (contact, allocator) {
    };
    b2PolygonContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
    };
    b2PolygonContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA.GetType() === b2ShapeType.e_polygonShape);
        // DEBUG: b2Assert(shapeB.GetType() === b2ShapeType.e_polygonShape);
        b2CollidePolygon_1.b2CollidePolygons(manifold, shapeA, xfA, shapeB, xfB);
    };
    return b2PolygonContact;
}(b2Contact_1.b2Contact));
exports.b2PolygonContact = b2PolygonContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJQb2x5Z29uQ29udGFjdC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0NvbnRhY3RzL2IyUG9seWdvbkNvbnRhY3QudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IjtBQUFBOzs7Ozs7Ozs7Ozs7Ozs7O0VBZ0JFOzs7Ozs7Ozs7Ozs7QUFLRixxRUFBcUU7QUFJckUseUNBQXdDO0FBR3hDO0lBQXNDLG9DQUFTO0lBQzdDO2VBQ0UsaUJBQU87SUFDVCxDQUFDO0lBRWEsdUJBQU0sR0FBcEIsVUFBcUIsU0FBYztRQUNqQyxPQUFPLElBQUksZ0JBQWdCLEVBQUUsQ0FBQztJQUNoQyxDQUFDO0lBRWEsd0JBQU8sR0FBckIsVUFBc0IsT0FBa0IsRUFBRSxTQUFjO0lBQ3hELENBQUM7SUFFTSxnQ0FBSyxHQUFaLFVBQWEsUUFBbUIsRUFBRSxNQUFjLEVBQUUsUUFBbUIsRUFBRSxNQUFjO1FBQ25GLGlCQUFNLEtBQUssWUFBQyxRQUFRLEVBQUUsTUFBTSxFQUFFLFFBQVEsRUFBRSxNQUFNLENBQUMsQ0FBQztJQUNsRCxDQUFDO0lBRU0sbUNBQVEsR0FBZixVQUFnQixRQUFvQixFQUFFLEdBQWdCLEVBQUUsR0FBZ0I7UUFDdEUsSUFBTSxNQUFNLEdBQVksSUFBSSxDQUFDLFVBQVUsQ0FBQyxRQUFRLEVBQUUsQ0FBQztRQUNuRCxJQUFNLE1BQU0sR0FBWSxJQUFJLENBQUMsVUFBVSxDQUFDLFFBQVEsRUFBRSxDQUFDO1FBQ25ELG9FQUFvRTtRQUNwRSxvRUFBb0U7UUFDcEUsb0NBQWlCLENBQ2YsUUFBUSxFQUNSLE1BQXdCLEVBQUUsR0FBRyxFQUM3QixNQUF3QixFQUFFLEdBQUcsQ0FBQyxDQUFDO0lBQ25DLENBQUM7SUFDSCx1QkFBQztBQUFELENBQUMsQUExQkQsQ0FBc0MscUJBQVMsR0EwQjlDO0FBMUJZLDRDQUFnQiJ9