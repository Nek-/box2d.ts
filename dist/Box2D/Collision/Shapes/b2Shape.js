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
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Math_1 = require("../../Common/b2Math");
/// This holds the mass data computed for a shape.
var b2MassData = /** @class */ (function () {
    function b2MassData() {
        /// The mass of the shape, usually in kilograms.
        this.mass = 0;
        /// The position of the shape's centroid relative to the shape's origin.
        this.center = new b2Math_1.b2Vec2(0, 0);
        /// The rotational inertia of the shape about the local origin.
        this.I = 0;
    }
    return b2MassData;
}());
exports.b2MassData = b2MassData;
var b2ShapeType;
(function (b2ShapeType) {
    b2ShapeType[b2ShapeType["e_unknown"] = -1] = "e_unknown";
    b2ShapeType[b2ShapeType["e_circleShape"] = 0] = "e_circleShape";
    b2ShapeType[b2ShapeType["e_edgeShape"] = 1] = "e_edgeShape";
    b2ShapeType[b2ShapeType["e_polygonShape"] = 2] = "e_polygonShape";
    b2ShapeType[b2ShapeType["e_chainShape"] = 3] = "e_chainShape";
    b2ShapeType[b2ShapeType["e_shapeTypeCount"] = 4] = "e_shapeTypeCount";
})(b2ShapeType = exports.b2ShapeType || (exports.b2ShapeType = {}));
/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
var b2Shape = /** @class */ (function () {
    function b2Shape(type, radius) {
        this.m_type = b2ShapeType.e_unknown;
        this.m_radius = 0;
        this.m_type = type;
        this.m_radius = radius;
    }
    b2Shape.prototype.Copy = function (other) {
        // DEBUG: b2Assert(this.m_type === other.m_type);
        this.m_radius = other.m_radius;
        return this;
    };
    /// Get the type of this shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    b2Shape.prototype.GetType = function () {
        return this.m_type;
    };
    return b2Shape;
}());
exports.b2Shape = b2Shape;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJTaGFwZS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0NvbGxpc2lvbi9TaGFwZXMvYjJTaGFwZS50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBRUYsNkRBQTZEO0FBQzdELDhDQUEwRDtBQUkxRCxrREFBa0Q7QUFDbEQ7SUFBQTtRQUNFLGdEQUFnRDtRQUN6QyxTQUFJLEdBQVcsQ0FBQyxDQUFDO1FBRXhCLHdFQUF3RTtRQUN4RCxXQUFNLEdBQVcsSUFBSSxlQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBRWxELCtEQUErRDtRQUN4RCxNQUFDLEdBQVcsQ0FBQyxDQUFDO0lBQ3ZCLENBQUM7SUFBRCxpQkFBQztBQUFELENBQUMsQUFURCxJQVNDO0FBVFksZ0NBQVU7QUFXdkIsSUFBWSxXQU9YO0FBUEQsV0FBWSxXQUFXO0lBQ3JCLHdEQUFjLENBQUE7SUFDZCwrREFBaUIsQ0FBQTtJQUNqQiwyREFBZSxDQUFBO0lBQ2YsaUVBQWtCLENBQUE7SUFDbEIsNkRBQWdCLENBQUE7SUFDaEIscUVBQW9CLENBQUE7QUFDdEIsQ0FBQyxFQVBXLFdBQVcsR0FBWCxtQkFBVyxLQUFYLG1CQUFXLFFBT3RCO0FBRUQscUZBQXFGO0FBQ3JGLG9GQUFvRjtBQUNwRixrRUFBa0U7QUFDbEU7SUFJRSxpQkFBWSxJQUFpQixFQUFFLE1BQWM7UUFIdEMsV0FBTSxHQUFnQixXQUFXLENBQUMsU0FBUyxDQUFDO1FBQzVDLGFBQVEsR0FBVyxDQUFDLENBQUM7UUFHMUIsSUFBSSxDQUFDLE1BQU0sR0FBRyxJQUFJLENBQUM7UUFDbkIsSUFBSSxDQUFDLFFBQVEsR0FBRyxNQUFNLENBQUM7SUFDekIsQ0FBQztJQUtNLHNCQUFJLEdBQVgsVUFBWSxLQUFjO1FBQ3hCLGlEQUFpRDtRQUNqRCxJQUFJLENBQUMsUUFBUSxHQUFHLEtBQUssQ0FBQyxRQUFRLENBQUM7UUFDL0IsT0FBTyxJQUFJLENBQUM7SUFDZCxDQUFDO0lBRUQsb0ZBQW9GO0lBQ3BGLDJCQUEyQjtJQUNwQix5QkFBTyxHQUFkO1FBQ0UsT0FBTyxJQUFJLENBQUMsTUFBTSxDQUFDO0lBQ3JCLENBQUM7SUEyQ0gsY0FBQztBQUFELENBQUMsQUFqRUQsSUFpRUM7QUFqRXFCLDBCQUFPIn0=