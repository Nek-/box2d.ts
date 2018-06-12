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
var b2CollideCircle_1 = require("../../Collision/b2CollideCircle");
var b2Contact_1 = require("./b2Contact");
var b2CircleContact = /** @class */ (function (_super) {
    __extends(b2CircleContact, _super);
    function b2CircleContact() {
        return _super.call(this) || this;
    }
    b2CircleContact.Create = function (allocator) {
        return new b2CircleContact();
    };
    b2CircleContact.Destroy = function (contact, allocator) {
    };
    b2CircleContact.prototype.Reset = function (fixtureA, indexA, fixtureB, indexB) {
        _super.prototype.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
    };
    b2CircleContact.prototype.Evaluate = function (manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        // DEBUG: b2Assert(shapeA.GetType() === b2ShapeType.e_circleShape);
        // DEBUG: b2Assert(shapeB.GetType() === b2ShapeType.e_circleShape);
        b2CollideCircle_1.b2CollideCircles(manifold, shapeA, xfA, shapeB, xfB);
    };
    return b2CircleContact;
}(b2Contact_1.b2Contact));
exports.b2CircleContact = b2CircleContact;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDaXJjbGVDb250YWN0LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vLi4vQm94MkQvQm94MkQvRHluYW1pY3MvQ29udGFjdHMvYjJDaXJjbGVDb250YWN0LnRzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7QUFBQTs7Ozs7Ozs7Ozs7Ozs7OztFQWdCRTs7Ozs7Ozs7Ozs7O0FBS0YsbUVBQW1FO0FBSW5FLHlDQUF3QztBQUd4QztJQUFxQyxtQ0FBUztJQUM1QztlQUNFLGlCQUFPO0lBQ1QsQ0FBQztJQUVhLHNCQUFNLEdBQXBCLFVBQXFCLFNBQWM7UUFDakMsT0FBTyxJQUFJLGVBQWUsRUFBRSxDQUFDO0lBQy9CLENBQUM7SUFFYSx1QkFBTyxHQUFyQixVQUFzQixPQUFrQixFQUFFLFNBQWM7SUFDeEQsQ0FBQztJQUVNLCtCQUFLLEdBQVosVUFBYSxRQUFtQixFQUFFLE1BQWMsRUFBRSxRQUFtQixFQUFFLE1BQWM7UUFDbkYsaUJBQU0sS0FBSyxZQUFDLFFBQVEsRUFBRSxNQUFNLEVBQUUsUUFBUSxFQUFFLE1BQU0sQ0FBQyxDQUFDO0lBQ2xELENBQUM7SUFFTSxrQ0FBUSxHQUFmLFVBQWdCLFFBQW9CLEVBQUUsR0FBZ0IsRUFBRSxHQUFnQjtRQUN0RSxJQUFNLE1BQU0sR0FBWSxJQUFJLENBQUMsVUFBVSxDQUFDLFFBQVEsRUFBRSxDQUFDO1FBQ25ELElBQU0sTUFBTSxHQUFZLElBQUksQ0FBQyxVQUFVLENBQUMsUUFBUSxFQUFFLENBQUM7UUFDbkQsbUVBQW1FO1FBQ25FLG1FQUFtRTtRQUNuRSxrQ0FBZ0IsQ0FDZCxRQUFRLEVBQ1IsTUFBdUIsRUFBRSxHQUFHLEVBQzVCLE1BQXVCLEVBQUUsR0FBRyxDQUFDLENBQUM7SUFDbEMsQ0FBQztJQUNILHNCQUFDO0FBQUQsQ0FBQyxBQTFCRCxDQUFxQyxxQkFBUyxHQTBCN0M7QUExQlksMENBQWUifQ==