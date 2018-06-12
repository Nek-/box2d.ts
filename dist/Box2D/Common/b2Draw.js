"use strict";
/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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
/// Color for debug drawing. Each value has the range [0,1].
var b2Color = /** @class */ (function () {
    function b2Color(rr, gg, bb, aa) {
        if (rr === void 0) { rr = 0.5; }
        if (gg === void 0) { gg = 0.5; }
        if (bb === void 0) { bb = 0.5; }
        if (aa === void 0) { aa = 1.0; }
        this.r = rr;
        this.g = gg;
        this.b = bb;
        this.a = aa;
    }
    b2Color.prototype.Clone = function () {
        return new b2Color().Copy(this);
    };
    b2Color.prototype.Copy = function (other) {
        this.r = other.r;
        this.g = other.g;
        this.b = other.b;
        this.a = other.a;
        return this;
    };
    b2Color.prototype.IsEqual = function (color) {
        return (this.r === color.r) && (this.g === color.g) && (this.b === color.b) && (this.a === color.a);
    };
    b2Color.prototype.IsZero = function () {
        return (this.r === 0) && (this.g === 0) && (this.b === 0) && (this.a === 0);
    };
    b2Color.prototype.Set = function (r, g, b, a) {
        if (a === void 0) { a = this.a; }
        this.SetRGBA(r, g, b, a);
    };
    b2Color.prototype.SetByteRGB = function (r, g, b) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        return this;
    };
    b2Color.prototype.SetByteRGBA = function (r, g, b, a) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        this.a = a / 0xff;
        return this;
    };
    b2Color.prototype.SetRGB = function (rr, gg, bb) {
        this.r = rr;
        this.g = gg;
        this.b = bb;
        return this;
    };
    b2Color.prototype.SetRGBA = function (rr, gg, bb, aa) {
        this.r = rr;
        this.g = gg;
        this.b = bb;
        this.a = aa;
        return this;
    };
    b2Color.prototype.SelfAdd = function (color) {
        this.r += color.r;
        this.g += color.g;
        this.b += color.b;
        this.a += color.a;
        return this;
    };
    b2Color.prototype.Add = function (color, out) {
        out.r = this.r + color.r;
        out.g = this.g + color.g;
        out.b = this.b + color.b;
        out.a = this.a + color.a;
        return out;
    };
    b2Color.prototype.SelfSub = function (color) {
        this.r -= color.r;
        this.g -= color.g;
        this.b -= color.b;
        this.a -= color.a;
        return this;
    };
    b2Color.prototype.Sub = function (color, out) {
        out.r = this.r - color.r;
        out.g = this.g - color.g;
        out.b = this.b - color.b;
        out.a = this.a - color.a;
        return out;
    };
    b2Color.prototype.SelfMul = function (s) {
        this.r *= s;
        this.g *= s;
        this.b *= s;
        this.a *= s;
        return this;
    };
    b2Color.prototype.Mul = function (s, out) {
        out.r = this.r * s;
        out.g = this.g * s;
        out.b = this.b * s;
        out.a = this.a * s;
        return out;
    };
    b2Color.prototype.Mix = function (mixColor, strength) {
        b2Color.MixColors(this, mixColor, strength);
    };
    b2Color.MixColors = function (colorA, colorB, strength) {
        var dr = (strength * (colorB.r - colorA.r));
        var dg = (strength * (colorB.g - colorA.g));
        var db = (strength * (colorB.b - colorA.b));
        var da = (strength * (colorB.a - colorA.a));
        colorA.r += dr;
        colorA.g += dg;
        colorA.b += db;
        colorA.a += da;
        colorB.r -= dr;
        colorB.g -= dg;
        colorB.b -= db;
        colorB.a -= da;
    };
    b2Color.prototype.MakeStyleString = function (alpha) {
        if (alpha === void 0) { alpha = this.a; }
        return b2Color.MakeStyleString(this.r, this.g, this.b, alpha);
    };
    b2Color.MakeStyleString = function (r, g, b, a) {
        if (a === void 0) { a = 1.0; }
        // function clamp(x: number, lo: number, hi: number) { return x < lo ? lo : hi < x ? hi : x; }
        r *= 255; // r = clamp(r, 0, 255);
        g *= 255; // g = clamp(g, 0, 255);
        b *= 255; // b = clamp(b, 0, 255);
        // a = clamp(a, 0, 1);
        if (a < 1) {
            return "rgba(" + r + "," + g + "," + b + "," + a + ")";
        }
        else {
            return "rgb(" + r + "," + g + "," + b + ")";
        }
    };
    b2Color.ZERO = new b2Color(0, 0, 0, 0);
    b2Color.RED = new b2Color(1, 0, 0);
    b2Color.GREEN = new b2Color(0, 1, 0);
    b2Color.BLUE = new b2Color(0, 0, 1);
    return b2Color;
}());
exports.b2Color = b2Color;
var b2DrawFlags;
(function (b2DrawFlags) {
    b2DrawFlags[b2DrawFlags["e_none"] = 0] = "e_none";
    b2DrawFlags[b2DrawFlags["e_shapeBit"] = 1] = "e_shapeBit";
    b2DrawFlags[b2DrawFlags["e_jointBit"] = 2] = "e_jointBit";
    b2DrawFlags[b2DrawFlags["e_aabbBit"] = 4] = "e_aabbBit";
    b2DrawFlags[b2DrawFlags["e_pairBit"] = 8] = "e_pairBit";
    b2DrawFlags[b2DrawFlags["e_centerOfMassBit"] = 16] = "e_centerOfMassBit";
    // #if B2_ENABLE_PARTICLE
    b2DrawFlags[b2DrawFlags["e_particleBit"] = 32] = "e_particleBit";
    // #endif
    b2DrawFlags[b2DrawFlags["e_controllerBit"] = 64] = "e_controllerBit";
    b2DrawFlags[b2DrawFlags["e_all"] = 63] = "e_all";
})(b2DrawFlags = exports.b2DrawFlags || (exports.b2DrawFlags = {}));
/// Implement and register this class with a b2World to provide debug drawing of physics
/// entities in your game.
var b2Draw = /** @class */ (function () {
    function b2Draw() {
        this.m_drawFlags = 0;
    }
    b2Draw.prototype.SetFlags = function (flags) {
        this.m_drawFlags = flags;
    };
    b2Draw.prototype.GetFlags = function () {
        return this.m_drawFlags;
    };
    b2Draw.prototype.AppendFlags = function (flags) {
        this.m_drawFlags |= flags;
    };
    b2Draw.prototype.ClearFlags = function (flags) {
        this.m_drawFlags &= ~flags;
    };
    b2Draw.prototype.PushTransform = function (xf) { };
    b2Draw.prototype.PopTransform = function (xf) { };
    b2Draw.prototype.DrawPolygon = function (vertices, vertexCount, color) { };
    b2Draw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) { };
    b2Draw.prototype.DrawCircle = function (center, radius, color) { };
    b2Draw.prototype.DrawSolidCircle = function (center, radius, axis, color) { };
    // #if B2_ENABLE_PARTICLE
    b2Draw.prototype.DrawParticles = function (centers, radius, colors, count) { };
    // #endif
    b2Draw.prototype.DrawSegment = function (p1, p2, color) { };
    b2Draw.prototype.DrawTransform = function (xf) { };
    return b2Draw;
}());
exports.b2Draw = b2Draw;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJEcmF3LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvQ29tbW9uL2IyRHJhdy50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBY0YsNERBQTREO0FBQzVEO0lBWUUsaUJBQVksRUFBZ0IsRUFBRSxFQUFnQixFQUFFLEVBQWdCLEVBQUUsRUFBZ0I7UUFBdEUsbUJBQUEsRUFBQSxRQUFnQjtRQUFFLG1CQUFBLEVBQUEsUUFBZ0I7UUFBRSxtQkFBQSxFQUFBLFFBQWdCO1FBQUUsbUJBQUEsRUFBQSxRQUFnQjtRQUNoRixJQUFJLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQztRQUNaLElBQUksQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDO1FBQ1osSUFBSSxDQUFDLENBQUMsR0FBRyxFQUFFLENBQUM7UUFDWixJQUFJLENBQUMsQ0FBQyxHQUFHLEVBQUUsQ0FBQztJQUNkLENBQUM7SUFFTSx1QkFBSyxHQUFaO1FBQ0UsT0FBTyxJQUFJLE9BQU8sRUFBRSxDQUFDLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQztJQUNsQyxDQUFDO0lBRU0sc0JBQUksR0FBWCxVQUFZLEtBQVc7UUFDckIsSUFBSSxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsQ0FBQyxDQUFDO1FBQ2pCLElBQUksQ0FBQyxDQUFDLEdBQUcsS0FBSyxDQUFDLENBQUMsQ0FBQztRQUNqQixJQUFJLENBQUMsQ0FBQyxHQUFHLEtBQUssQ0FBQyxDQUFDLENBQUM7UUFDakIsSUFBSSxDQUFDLENBQUMsR0FBRyxLQUFLLENBQUMsQ0FBQyxDQUFDO1FBQ2pCLE9BQU8sSUFBSSxDQUFDO0lBQ2QsQ0FBQztJQUVNLHlCQUFPLEdBQWQsVUFBZSxLQUFXO1FBQ3hCLE9BQU8sQ0FBQyxJQUFJLENBQUMsQ0FBQyxLQUFLLEtBQUssQ0FBQyxDQUFDLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDLEtBQUssS0FBSyxD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