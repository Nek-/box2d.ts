"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
// DEBUG: import { b2Assert } from "../../Common/b2Settings";
var b2Settings_1 = require("../../Common/b2Settings");
var b2Shape_1 = require("../../Collision/Shapes/b2Shape");
var b2CircleContact_1 = require("./b2CircleContact");
var b2PolygonContact_1 = require("./b2PolygonContact");
var b2PolygonAndCircleContact_1 = require("./b2PolygonAndCircleContact");
var b2EdgeAndCircleContact_1 = require("./b2EdgeAndCircleContact");
var b2EdgeAndPolygonContact_1 = require("./b2EdgeAndPolygonContact");
var b2ChainAndCircleContact_1 = require("./b2ChainAndCircleContact");
var b2ChainAndPolygonContact_1 = require("./b2ChainAndPolygonContact");
var b2ContactRegister = /** @class */ (function () {
    function b2ContactRegister() {
        // public pool: b2Contact[];
        this.createFcn = null;
        this.destroyFcn = null;
        this.primary = false;
    }
    return b2ContactRegister;
}());
exports.b2ContactRegister = b2ContactRegister;
var b2ContactFactory = /** @class */ (function () {
    function b2ContactFactory(allocator) {
        this.m_allocator = null;
        this.m_allocator = allocator;
        this.InitializeRegisters();
    }
    b2ContactFactory.prototype.AddType = function (createFcn, destroyFcn, type1, type2) {
        var _this = this;
        var pool = b2Settings_1.b2MakeArray(256, function (i) { return createFcn(_this.m_allocator); }); // TODO: b2Settings
        function poolCreateFcn(allocator) {
            // if (pool.length > 0) {
            //   return pool.pop();
            // }
            // return createFcn(allocator);
            return pool.pop() || createFcn(allocator);
        }
        function poolDestroyFcn(contact, allocator) {
            pool.push(contact);
        }
        // this.m_registers[type1][type2].pool = pool;
        this.m_registers[type1][type2].createFcn = poolCreateFcn;
        this.m_registers[type1][type2].destroyFcn = poolDestroyFcn;
        this.m_registers[type1][type2].primary = true;
        if (type1 !== type2) {
            // this.m_registers[type2][type1].pool = pool;
            this.m_registers[type2][type1].createFcn = poolCreateFcn;
            this.m_registers[type2][type1].destroyFcn = poolDestroyFcn;
            this.m_registers[type2][type1].primary = false;
        }
        /*
        this.m_registers[type1][type2].createFcn = createFcn;
        this.m_registers[type1][type2].destroyFcn = destroyFcn;
        this.m_registers[type1][type2].primary = true;
    
        if (type1 !== type2) {
          this.m_registers[type2][type1].createFcn = createFcn;
          this.m_registers[type2][type1].destroyFcn = destroyFcn;
          this.m_registers[type2][type1].primary = false;
        }
        */
    };
    b2ContactFactory.prototype.InitializeRegisters = function () {
        this.m_registers = [ /*b2ShapeType.e_shapeTypeCount*/];
        for (var i = 0; i < b2Shape_1.b2ShapeType.e_shapeTypeCount; i++) {
            this.m_registers[i] = [ /*b2ShapeType.e_shapeTypeCount*/];
            for (var j = 0; j < b2Shape_1.b2ShapeType.e_shapeTypeCount; j++) {
                this.m_registers[i][j] = new b2ContactRegister();
            }
        }
        this.AddType(b2CircleContact_1.b2CircleContact.Create, b2CircleContact_1.b2CircleContact.Destroy, b2Shape_1.b2ShapeType.e_circleShape, b2Shape_1.b2ShapeType.e_circleShape);
        this.AddType(b2PolygonAndCircleContact_1.b2PolygonAndCircleContact.Create, b2PolygonAndCircleContact_1.b2PolygonAndCircleContact.Destroy, b2Shape_1.b2ShapeType.e_polygonShape, b2Shape_1.b2ShapeType.e_circleShape);
        this.AddType(b2PolygonContact_1.b2PolygonContact.Create, b2PolygonContact_1.b2PolygonContact.Destroy, b2Shape_1.b2ShapeType.e_polygonShape, b2Shape_1.b2ShapeType.e_polygonShape);
        this.AddType(b2EdgeAndCircleContact_1.b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact_1.b2EdgeAndCircleContact.Destroy, b2Shape_1.b2ShapeType.e_edgeShape, b2Shape_1.b2ShapeType.e_circleShape);
        this.AddType(b2EdgeAndPolygonContact_1.b2EdgeAndPolygonContact.Create, b2EdgeAndPolygonContact_1.b2EdgeAndPolygonContact.Destroy, b2Shape_1.b2ShapeType.e_edgeShape, b2Shape_1.b2ShapeType.e_polygonShape);
        this.AddType(b2ChainAndCircleContact_1.b2ChainAndCircleContact.Create, b2ChainAndCircleContact_1.b2ChainAndCircleContact.Destroy, b2Shape_1.b2ShapeType.e_chainShape, b2Shape_1.b2ShapeType.e_circleShape);
        this.AddType(b2ChainAndPolygonContact_1.b2ChainAndPolygonContact.Create, b2ChainAndPolygonContact_1.b2ChainAndPolygonContact.Destroy, b2Shape_1.b2ShapeType.e_chainShape, b2Shape_1.b2ShapeType.e_polygonShape);
    };
    b2ContactFactory.prototype.Create = function (fixtureA, indexA, fixtureB, indexB) {
        var type1 = fixtureA.GetType();
        var type2 = fixtureB.GetType();
        // DEBUG: b2Assert(0 <= type1 && type1 < b2ShapeType.e_shapeTypeCount);
        // DEBUG: b2Assert(0 <= type2 && type2 < b2ShapeType.e_shapeTypeCount);
        var reg = this.m_registers[type1][type2];
        if (reg.createFcn) {
            var c = reg.createFcn(this.m_allocator);
            if (reg.primary) {
                c.Reset(fixtureA, indexA, fixtureB, indexB);
            }
            else {
                c.Reset(fixtureB, indexB, fixtureA, indexA);
            }
            return c;
        }
        else {
            return null;
        }
    };
    b2ContactFactory.prototype.Destroy = function (contact) {
        var fixtureA = contact.m_fixtureA;
        var fixtureB = contact.m_fixtureB;
        if (contact.m_manifold.pointCount > 0 &&
            !fixtureA.IsSensor() &&
            !fixtureB.IsSensor()) {
            fixtureA.GetBody().SetAwake(true);
            fixtureB.GetBody().SetAwake(true);
        }
        var typeA = fixtureA.GetType();
        var typeB = fixtureB.GetType();
        // DEBUG: b2Assert(0 <= typeA && typeB < b2ShapeType.e_shapeTypeCount);
        // DEBUG: b2Assert(0 <= typeA && typeB < b2ShapeType.e_shapeTypeCount);
        var reg = this.m_registers[typeA][typeB];
        if (reg.destroyFcn) {
            reg.destroyFcn(contact, this.m_allocator);
        }
    };
    return b2ContactFactory;
}());
exports.b2ContactFactory = b2ContactFactory;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJDb250YWN0RmFjdG9yeS5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzIjpbIi4uLy4uLy4uLy4uL0JveDJEL0JveDJEL0R5bmFtaWNzL0NvbnRhY3RzL2IyQ29udGFjdEZhY3RvcnkudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7QUFBQSw2REFBNkQ7QUFDN0Qsc0RBQXNEO0FBQ3RELDBEQUE2RDtBQUU3RCxxREFBb0Q7QUFDcEQsdURBQXNEO0FBQ3RELHlFQUF3RTtBQUN4RSxtRUFBa0U7QUFDbEUscUVBQW9FO0FBQ3BFLHFFQUFvRTtBQUNwRSx1RUFBc0U7QUFHdEU7SUFBQTtRQUNFLDRCQUE0QjtRQUNyQixjQUFTLEdBQTJDLElBQUksQ0FBQztRQUN6RCxlQUFVLEdBQTBELElBQUksQ0FBQztRQUN6RSxZQUFPLEdBQVksS0FBSyxDQUFDO0lBQ2xDLENBQUM7SUFBRCx3QkFBQztBQUFELENBQUMsQUFMRCxJQUtDO0FBTFksOENBQWlCO0FBTzlCO0lBSUUsMEJBQVksU0FBYztRQUhuQixnQkFBVyxHQUFRLElBQUksQ0FBQztRQUk3QixJQUFJLENBQUMsV0FBVyxHQUFHLFNBQVMsQ0FBQztRQUM3QixJQUFJLENBQUMsbUJBQW1CLEVBQUUsQ0FBQztJQUM3QixDQUFDO0lBRU8sa0NBQU8sR0FBZixVQUFnQixTQUF3QyxFQUFFLFVBQXdELEVBQUUsS0FBa0IsRUFBRSxLQUFrQjtRQUExSixpQkF1Q0M7UUF0Q0MsSUFBTSxJQUFJLEdBQWdCLHdCQUFXLENBQUMsR0FBRyxFQUFFLFVBQUMsQ0FBUyxJQUFLLE9BQUEsU0FBUyxDQUFDLEtBQUksQ0FBQyxXQUFXLENBQUMsRUFBM0IsQ0FBMkIsQ0FBQyxDQUFDLENBQUMsbUJBQW1CO1FBRTNHLHVCQUF1QixTQUFjO1lBQ25DLHlCQUF5QjtZQUN6Qix1QkFBdUI7WUFDdkIsSUFBSTtZQUVKLCtCQUErQjtZQUMvQixPQUFPLElBQUksQ0FBQyxHQUFHLEVBQUUsSUFBSSxTQUFTLENBQUMsU0FBUyxDQUFDLENB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