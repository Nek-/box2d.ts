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
// DEBUG: import { b2Assert } from "./b2Settings";
var b2Settings_1 = require("./b2Settings");
exports.b2_pi_over_180 = b2Settings_1.b2_pi / 180;
exports.b2_180_over_pi = 180 / b2Settings_1.b2_pi;
exports.b2_two_pi = 2 * b2Settings_1.b2_pi;
exports.b2Abs = Math.abs;
exports.b2Min = Math.min;
exports.b2Max = Math.max;
function b2Clamp(a, lo, hi) {
    return (a < lo) ? (lo) : ((a > hi) ? (hi) : (a));
}
exports.b2Clamp = b2Clamp;
function b2Swap(a, b) {
    // DEBUG: b2Assert(false);
    var tmp = a[0];
    a[0] = b[0];
    b[0] = tmp;
}
exports.b2Swap = b2Swap;
/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
exports.b2IsValid = isFinite;
function b2Sq(n) {
    return n * n;
}
exports.b2Sq = b2Sq;
/// This is a approximate yet fast inverse square-root.
function b2InvSqrt(n) {
    return 1 / Math.sqrt(n);
}
exports.b2InvSqrt = b2InvSqrt;
exports.b2Sqrt = Math.sqrt;
exports.b2Pow = Math.pow;
function b2DegToRad(degrees) {
    return degrees * exports.b2_pi_over_180;
}
exports.b2DegToRad = b2DegToRad;
function b2RadToDeg(radians) {
    return radians * exports.b2_180_over_pi;
}
exports.b2RadToDeg = b2RadToDeg;
exports.b2Cos = Math.cos;
exports.b2Sin = Math.sin;
exports.b2Acos = Math.acos;
exports.b2Asin = Math.asin;
exports.b2Atan2 = Math.atan2;
function b2NextPowerOfTwo(x) {
    x |= (x >> 1) & 0x7FFFFFFF;
    x |= (x >> 2) & 0x3FFFFFFF;
    x |= (x >> 4) & 0x0FFFFFFF;
    x |= (x >> 8) & 0x00FFFFFF;
    x |= (x >> 16) & 0x0000FFFF;
    return x + 1;
}
exports.b2NextPowerOfTwo = b2NextPowerOfTwo;
function b2IsPowerOfTwo(x) {
    return x > 0 && (x & (x - 1)) === 0;
}
exports.b2IsPowerOfTwo = b2IsPowerOfTwo;
function b2Random() {
    return Math.random() * 2 - 1;
}
exports.b2Random = b2Random;
function b2RandomRange(lo, hi) {
    return (hi - lo) * Math.random() + lo;
}
exports.b2RandomRange = b2RandomRange;
/// A 2D column vector.
var b2Vec2 = /** @class */ (function () {
    function b2Vec2(x, y) {
        if (x === void 0) { x = 0; }
        if (y === void 0) { y = 0; }
        this.x = x;
        this.y = y;
    }
    b2Vec2.prototype.Clone = function () {
        return new b2Vec2(this.x, this.y);
    };
    b2Vec2.prototype.SetZero = function () {
        this.x = 0;
        this.y = 0;
        return this;
    };
    b2Vec2.prototype.Set = function (x, y) {
        this.x = x;
        this.y = y;
        return this;
    };
    b2Vec2.prototype.Copy = function (other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    };
    b2Vec2.prototype.SelfAdd = function (v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    };
    b2Vec2.prototype.SelfAddXY = function (x, y) {
        this.x += x;
        this.y += y;
        return this;
    };
    b2Vec2.prototype.SelfSub = function (v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    };
    b2Vec2.prototype.SelfSubXY = function (x, y) {
        this.x -= x;
        this.y -= y;
        return this;
    };
    b2Vec2.prototype.SelfMul = function (s) {
        this.x *= s;
        this.y *= s;
        return this;
    };
    b2Vec2.prototype.SelfMulAdd = function (s, v) {
        this.x += s * v.x;
        this.y += s * v.y;
        return this;
    };
    b2Vec2.prototype.SelfMulSub = function (s, v) {
        this.x -= s * v.x;
        this.y -= s * v.y;
        return this;
    };
    b2Vec2.prototype.Dot = function (v) {
        return this.x * v.x + this.y * v.y;
    };
    b2Vec2.prototype.Cross = function (v) {
        return this.x * v.y - this.y * v.x;
    };
    b2Vec2.prototype.Length = function () {
        var x = this.x, y = this.y;
        return Math.sqrt(x * x + y * y);
    };
    b2Vec2.prototype.LengthSquared = function () {
        var x = this.x, y = this.y;
        return (x * x + y * y);
    };
    b2Vec2.prototype.Normalize = function () {
        var length = this.Length();
        if (length >= b2Settings_1.b2_epsilon) {
            var inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return length;
    };
    b2Vec2.prototype.SelfNormalize = function () {
        var length = this.Length();
        if (length >= b2Settings_1.b2_epsilon) {
            var inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return this;
    };
    b2Vec2.prototype.SelfRotate = function (radians) {
        var c = Math.cos(radians);
        var s = Math.sin(radians);
        var x = this.x;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    };
    b2Vec2.prototype.IsValid = function () {
        return isFinite(this.x) && isFinite(this.y);
    };
    b2Vec2.prototype.SelfCrossVS = function (s) {
        var x = this.x;
        this.x = s * this.y;
        this.y = -s * x;
        return this;
    };
    b2Vec2.prototype.SelfCrossSV = function (s) {
        var x = this.x;
        this.x = -s * this.y;
        this.y = s * x;
        return this;
    };
    b2Vec2.prototype.SelfMinV = function (v) {
        this.x = exports.b2Min(this.x, v.x);
        this.y = exports.b2Min(this.y, v.y);
        return this;
    };
    b2Vec2.prototype.SelfMaxV = function (v) {
        this.x = exports.b2Max(this.x, v.x);
        this.y = exports.b2Max(this.y, v.y);
        return this;
    };
    b2Vec2.prototype.SelfAbs = function () {
        this.x = exports.b2Abs(this.x);
        this.y = exports.b2Abs(this.y);
        return this;
    };
    b2Vec2.prototype.SelfNeg = function () {
        this.x = (-this.x);
        this.y = (-this.y);
        return this;
    };
    b2Vec2.prototype.SelfSkew = function () {
        var x = this.x;
        this.x = -this.y;
        this.y = x;
        return this;
    };
    b2Vec2.MakeArray = function (length) {
        return b2Settings_1.b2MakeArray(length, function (i) { return new b2Vec2(); });
    };
    b2Vec2.AbsV = function (v, out) {
        out.x = exports.b2Abs(v.x);
        out.y = exports.b2Abs(v.y);
        return out;
    };
    b2Vec2.MinV = function (a, b, out) {
        out.x = exports.b2Min(a.x, b.x);
        out.y = exports.b2Min(a.y, b.y);
        return out;
    };
    b2Vec2.MaxV = function (a, b, out) {
        out.x = exports.b2Max(a.x, b.x);
        out.y = exports.b2Max(a.y, b.y);
        return out;
    };
    b2Vec2.ClampV = function (v, lo, hi, out) {
        out.x = b2Clamp(v.x, lo.x, hi.x);
        out.y = b2Clamp(v.y, lo.y, hi.y);
        return out;
    };
    b2Vec2.RotateV = function (v, radians, out) {
        var v_x = v.x, v_y = v.y;
        var c = Math.cos(radians);
        var s = Math.sin(radians);
        out.x = c * v_x - s * v_y;
        out.y = s * v_x + c * v_y;
        return out;
    };
    b2Vec2.DotVV = function (a, b) {
        return a.x * b.x + a.y * b.y;
    };
    b2Vec2.CrossVV = function (a, b) {
        return a.x * b.y - a.y * b.x;
    };
    b2Vec2.CrossVS = function (v, s, out) {
        var v_x = v.x;
        out.x = s * v.y;
        out.y = -s * v_x;
        return out;
    };
    b2Vec2.CrossVOne = function (v, out) {
        var v_x = v.x;
        out.x = v.y;
        out.y = -v_x;
        return out;
    };
    b2Vec2.CrossSV = function (s, v, out) {
        var v_x = v.x;
        out.x = -s * v.y;
        out.y = s * v_x;
        return out;
    };
    b2Vec2.CrossOneV = function (v, out) {
        var v_x = v.x;
        out.x = -v.y;
        out.y = v_x;
        return out;
    };
    b2Vec2.AddVV = function (a, b, out) { out.x = a.x + b.x; out.y = a.y + b.y; return out; };
    b2Vec2.SubVV = function (a, b, out) { out.x = a.x - b.x; out.y = a.y - b.y; return out; };
    b2Vec2.MulSV = function (s, v, out) { out.x = v.x * s; out.y = v.y * s; return out; };
    b2Vec2.MulVS = function (v, s, out) { out.x = v.x * s; out.y = v.y * s; return out; };
    b2Vec2.AddVMulSV = function (a, s, b, out) { out.x = a.x + (s * b.x); out.y = a.y + (s * b.y); return out; };
    b2Vec2.SubVMulSV = function (a, s, b, out) { out.x = a.x - (s * b.x); out.y = a.y - (s * b.y); return out; };
    b2Vec2.AddVCrossSV = function (a, s, v, out) {
        var v_x = v.x;
        out.x = a.x - (s * v.y);
        out.y = a.y + (s * v_x);
        return out;
    };
    b2Vec2.MidVV = function (a, b, out) { out.x = (a.x + b.x) * 0.5; out.y = (a.y + b.y) * 0.5; return out; };
    b2Vec2.ExtVV = function (a, b, out) { out.x = (b.x - a.x) * 0.5; out.y = (b.y - a.y) * 0.5; return out; };
    b2Vec2.IsEqualToV = function (a, b) {
        return a.x === b.x && a.y === b.y;
    };
    b2Vec2.DistanceVV = function (a, b) {
        var c_x = a.x - b.x;
        var c_y = a.y - b.y;
        return Math.sqrt(c_x * c_x + c_y * c_y);
    };
    b2Vec2.DistanceSquaredVV = function (a, b) {
        var c_x = a.x - b.x;
        var c_y = a.y - b.y;
        return (c_x * c_x + c_y * c_y);
    };
    b2Vec2.NegV = function (v, out) { out.x = -v.x; out.y = -v.y; return out; };
    b2Vec2.ZERO = new b2Vec2(0, 0);
    b2Vec2.UNITX = new b2Vec2(1, 0);
    b2Vec2.UNITY = new b2Vec2(0, 1);
    b2Vec2.s_t0 = new b2Vec2();
    b2Vec2.s_t1 = new b2Vec2();
    b2Vec2.s_t2 = new b2Vec2();
    b2Vec2.s_t3 = new b2Vec2();
    return b2Vec2;
}());
exports.b2Vec2 = b2Vec2;
exports.b2Vec2_zero = new b2Vec2(0, 0);
/// A 2D column vector with 3 elements.
var b2Vec3 = /** @class */ (function () {
    function b2Vec3(x, y, z) {
        if (x === void 0) { x = 0; }
        if (y === void 0) { y = 0; }
        if (z === void 0) { z = 0; }
        this.x = x;
        this.y = y;
        this.z = z;
    }
    b2Vec3.prototype.Clone = function () {
        return new b2Vec3(this.x, this.y, this.z);
    };
    b2Vec3.prototype.SetZero = function () {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    };
    b2Vec3.prototype.SetXYZ = function (x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    };
    b2Vec3.prototype.Copy = function (other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        return this;
    };
    b2Vec3.prototype.SelfNeg = function () {
        this.x = (-this.x);
        this.y = (-this.y);
        this.z = (-this.z);
        return this;
    };
    b2Vec3.prototype.SelfAdd = function (v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    };
    b2Vec3.prototype.SelfAddXYZ = function (x, y, z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    };
    b2Vec3.prototype.SelfSub = function (v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    };
    b2Vec3.prototype.SelfSubXYZ = function (x, y, z) {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    };
    b2Vec3.prototype.SelfMul = function (s) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    };
    b2Vec3.DotV3V3 = function (a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    };
    b2Vec3.CrossV3V3 = function (a, b, out) {
        var a_x = a.x, a_y = a.y, a_z = a.z;
        var b_x = b.x, b_y = b.y, b_z = b.z;
        out.x = a_y * b_z - a_z * b_y;
        out.y = a_z * b_x - a_x * b_z;
        out.z = a_x * b_y - a_y * b_x;
        return out;
    };
    b2Vec3.ZERO = new b2Vec3(0, 0, 0);
    b2Vec3.s_t0 = new b2Vec3();
    return b2Vec3;
}());
exports.b2Vec3 = b2Vec3;
/// A 2-by-2 matrix. Stored in column-major order.
var b2Mat22 = /** @class */ (function () {
    function b2Mat22() {
        this.ex = new b2Vec2(1, 0);
        this.ey = new b2Vec2(0, 1);
    }
    b2Mat22.prototype.Clone = function () {
        return new b2Mat22().Copy(this);
    };
    b2Mat22.FromVV = function (c1, c2) {
        return new b2Mat22().SetVV(c1, c2);
    };
    b2Mat22.FromSSSS = function (r1c1, r1c2, r2c1, r2c2) {
        return new b2Mat22().SetSSSS(r1c1, r1c2, r2c1, r2c2);
    };
    b2Mat22.FromAngle = function (radians) {
        return new b2Mat22().SetAngle(radians);
    };
    b2Mat22.prototype.SetSSSS = function (r1c1, r1c2, r2c1, r2c2) {
        this.ex.Set(r1c1, r2c1);
        this.ey.Set(r1c2, r2c2);
        return this;
    };
    b2Mat22.prototype.SetVV = function (c1, c2) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        return this;
    };
    b2Mat22.prototype.SetAngle = function (radians) {
        var c = Math.cos(radians);
        var s = Math.sin(radians);
        this.ex.Set(c, s);
        this.ey.Set(-s, c);
        return this;
    };
    b2Mat22.prototype.Copy = function (other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        return this;
    };
    b2Mat22.prototype.SetIdentity = function () {
        this.ex.Set(1, 0);
        this.ey.Set(0, 1);
        return this;
    };
    b2Mat22.prototype.SetZero = function () {
        this.ex.SetZero();
        this.ey.SetZero();
        return this;
    };
    b2Mat22.prototype.GetAngle = function () {
        return Math.atan2(this.ex.y, this.ex.x);
    };
    b2Mat22.prototype.GetInverse = function (out) {
        var a = this.ex.x;
        var b = this.ey.x;
        var c = this.ex.y;
        var d = this.ey.y;
        var det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        out.ex.x = det * d;
        out.ey.x = (-det * b);
        out.ex.y = (-det * c);
        out.ey.y = det * a;
        return out;
    };
    b2Mat22.prototype.Solve = function (b_x, b_y, out) {
        var a11 = this.ex.x, a12 = this.ey.x;
        var a21 = this.ex.y, a22 = this.ey.y;
        var det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    };
    b2Mat22.prototype.SelfAbs = function () {
        this.ex.SelfAbs();
        this.ey.SelfAbs();
        return this;
    };
    b2Mat22.prototype.SelfInv = function () {
        this.GetInverse(this);
        return this;
    };
    b2Mat22.prototype.SelfAddM = function (M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        return this;
    };
    b2Mat22.prototype.SelfSubM = function (M) {
        this.ex.SelfSub(M.ex);
        this.ey.SelfSub(M.ey);
        return this;
    };
    b2Mat22.AbsM = function (M, out) {
        var M_ex = M.ex, M_ey = M.ey;
        out.ex.x = exports.b2Abs(M_ex.x);
        out.ex.y = exports.b2Abs(M_ex.y);
        out.ey.x = exports.b2Abs(M_ey.x);
        out.ey.y = exports.b2Abs(M_ey.y);
        return out;
    };
    b2Mat22.MulMV = function (M, v, out) {
        var M_ex = M.ex, M_ey = M.ey;
        var v_x = v.x, v_y = v.y;
        out.x = M_ex.x * v_x + M_ey.x * v_y;
        out.y = M_ex.y * v_x + M_ey.y * v_y;
        return out;
    };
    b2Mat22.MulTMV = function (M, v, out) {
        var M_ex = M.ex, M_ey = M.ey;
        var v_x = v.x, v_y = v.y;
        out.x = M_ex.x * v_x + M_ex.y * v_y;
        out.y = M_ey.x * v_x + M_ey.y * v_y;
        return out;
    };
    b2Mat22.AddMM = function (A, B, out) {
        var A_ex = A.ex, A_ey = A.ey;
        var B_ex = B.ex, B_ey = B.ey;
        out.ex.x = A_ex.x + B_ex.x;
        out.ex.y = A_ex.y + B_ex.y;
        out.ey.x = A_ey.x + B_ey.x;
        out.ey.y = A_ey.y + B_ey.y;
        return out;
    };
    b2Mat22.MulMM = function (A, B, out) {
        var A_ex_x = A.ex.x, A_ex_y = A.ex.y;
        var A_ey_x = A.ey.x, A_ey_y = A.ey.y;
        var B_ex_x = B.ex.x, B_ex_y = B.ex.y;
        var B_ey_x = B.ey.x, B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y;
        out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y;
        out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y;
        return out;
    };
    b2Mat22.MulTMM = function (A, B, out) {
        var A_ex_x = A.ex.x, A_ex_y = A.ex.y;
        var A_ey_x = A.ey.x, A_ey_y = A.ey.y;
        var B_ex_x = B.ex.x, B_ex_y = B.ex.y;
        var B_ey_x = B.ey.x, B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y;
        out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y;
        out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y;
        return out;
    };
    b2Mat22.IDENTITY = new b2Mat22();
    return b2Mat22;
}());
exports.b2Mat22 = b2Mat22;
/// A 3-by-3 matrix. Stored in column-major order.
var b2Mat33 = /** @class */ (function () {
    function b2Mat33() {
        this.ex = new b2Vec3(1, 0, 0);
        this.ey = new b2Vec3(0, 1, 0);
        this.ez = new b2Vec3(0, 0, 1);
    }
    b2Mat33.prototype.Clone = function () {
        return new b2Mat33().Copy(this);
    };
    b2Mat33.prototype.SetVVV = function (c1, c2, c3) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        this.ez.Copy(c3);
        return this;
    };
    b2Mat33.prototype.Copy = function (other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        this.ez.Copy(other.ez);
        return this;
    };
    b2Mat33.prototype.SetIdentity = function () {
        this.ex.SetXYZ(1, 0, 0);
        this.ey.SetXYZ(0, 1, 0);
        this.ez.SetXYZ(0, 0, 1);
        return this;
    };
    b2Mat33.prototype.SetZero = function () {
        this.ex.SetZero();
        this.ey.SetZero();
        this.ez.SetZero();
        return this;
    };
    b2Mat33.prototype.SelfAddM = function (M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        this.ez.SelfAdd(M.ez);
        return this;
    };
    b2Mat33.prototype.Solve33 = function (b_x, b_y, b_z, out) {
        var a11 = this.ex.x, a21 = this.ex.y, a31 = this.ex.z;
        var a12 = this.ey.x, a22 = this.ey.y, a32 = this.ey.z;
        var a13 = this.ez.x, a23 = this.ez.y, a33 = this.ez.z;
        var det = a11 * (a22 * a33 - a32 * a23) + a21 * (a32 * a13 - a12 * a33) + a31 * (a12 * a23 - a22 * a13);
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (b_x * (a22 * a33 - a32 * a23) + b_y * (a32 * a13 - a12 * a33) + b_z * (a12 * a23 - a22 * a13));
        out.y = det * (a11 * (b_y * a33 - b_z * a23) + a21 * (b_z * a13 - b_x * a33) + a31 * (b_x * a23 - b_y * a13));
        out.z = det * (a11 * (a22 * b_z - a32 * b_y) + a21 * (a32 * b_x - a12 * b_z) + a31 * (a12 * b_y - a22 * b_x));
        return out;
    };
    b2Mat33.prototype.Solve22 = function (b_x, b_y, out) {
        var a11 = this.ex.x, a12 = this.ey.x;
        var a21 = this.ex.y, a22 = this.ey.y;
        var det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    };
    b2Mat33.prototype.GetInverse22 = function (M) {
        var a = this.ex.x, b = this.ey.x, c = this.ex.y, d = this.ey.y;
        var det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0;
        M.ez.x = 0;
        M.ez.y = 0;
        M.ez.z = 0;
    };
    b2Mat33.prototype.GetSymInverse33 = function (M) {
        var det = b2Vec3.DotV3V3(this.ex, b2Vec3.CrossV3V3(this.ey, this.ez, b2Vec3.s_t0));
        if (det !== 0) {
            det = 1 / det;
        }
        var a11 = this.ex.x, a12 = this.ey.x, a13 = this.ez.x;
        var a22 = this.ey.y, a23 = this.ez.y;
        var a33 = this.ez.z;
        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);
        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);
        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    };
    b2Mat33.MulM33V3 = function (A, v, out) {
        var v_x = v.x, v_y = v.y, v_z = v.z;
        out.x = A.ex.x * v_x + A.ey.x * v_y + A.ez.x * v_z;
        out.y = A.ex.y * v_x + A.ey.y * v_y + A.ez.y * v_z;
        out.z = A.ex.z * v_x + A.ey.z * v_y + A.ez.z * v_z;
        return out;
    };
    b2Mat33.MulM33XYZ = function (A, x, y, z, out) {
        out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z;
        out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z;
        out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z;
        return out;
    };
    b2Mat33.MulM33V2 = function (A, v, out) {
        var v_x = v.x, v_y = v.y;
        out.x = A.ex.x * v_x + A.ey.x * v_y;
        out.y = A.ex.y * v_x + A.ey.y * v_y;
        return out;
    };
    b2Mat33.MulM33XY = function (A, x, y, out) {
        out.x = A.ex.x * x + A.ey.x * y;
        out.y = A.ex.y * x + A.ey.y * y;
        return out;
    };
    b2Mat33.IDENTITY = new b2Mat33();
    return b2Mat33;
}());
exports.b2Mat33 = b2Mat33;
/// Rotation
var b2Rot = /** @class */ (function () {
    function b2Rot(angle) {
        if (angle === void 0) { angle = 0; }
        this.s = 0;
        this.c = 1;
        if (angle) {
            this.s = Math.sin(angle);
            this.c = Math.cos(angle);
        }
    }
    b2Rot.prototype.Clone = function () {
        return new b2Rot().Copy(this);
    };
    b2Rot.prototype.Copy = function (other) {
        this.s = other.s;
        this.c = other.c;
        return this;
    };
    b2Rot.prototype.SetAngle = function (angle) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
        return this;
    };
    b2Rot.prototype.SetIdentity = function () {
        this.s = 0;
        this.c = 1;
        return this;
    };
    b2Rot.prototype.GetAngle = function () {
        return Math.atan2(this.s, this.c);
    };
    b2Rot.prototype.GetXAxis = function (out) {
        out.x = this.c;
        out.y = this.s;
        return out;
    };
    b2Rot.prototype.GetYAxis = function (out) {
        out.x = -this.s;
        out.y = this.c;
        return out;
    };
    b2Rot.MulRR = function (q, r, out) {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        var q_c = q.c, q_s = q.s;
        var r_c = r.c, r_s = r.s;
        out.s = q_s * r_c + q_c * r_s;
        out.c = q_c * r_c - q_s * r_s;
        return out;
    };
    b2Rot.MulTRR = function (q, r, out) {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        var q_c = q.c, q_s = q.s;
        var r_c = r.c, r_s = r.s;
        out.s = q_c * r_s - q_s * r_c;
        out.c = q_c * r_c + q_s * r_s;
        return out;
    };
    b2Rot.MulRV = function (q, v, out) {
        var q_c = q.c, q_s = q.s;
        var v_x = v.x, v_y = v.y;
        out.x = q_c * v_x - q_s * v_y;
        out.y = q_s * v_x + q_c * v_y;
        return out;
    };
    b2Rot.MulTRV = function (q, v, out) {
        var q_c = q.c, q_s = q.s;
        var v_x = v.x, v_y = v.y;
        out.x = q_c * v_x + q_s * v_y;
        out.y = -q_s * v_x + q_c * v_y;
        return out;
    };
    b2Rot.IDENTITY = new b2Rot();
    return b2Rot;
}());
exports.b2Rot = b2Rot;
/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
var b2Transform = /** @class */ (function () {
    function b2Transform() {
        this.p = new b2Vec2();
        this.q = new b2Rot();
    }
    b2Transform.prototype.Clone = function () {
        return new b2Transform().Copy(this);
    };
    b2Transform.prototype.Copy = function (other) {
        this.p.Copy(other.p);
        this.q.Copy(other.q);
        return this;
    };
    b2Transform.prototype.SetIdentity = function () {
        this.p.SetZero();
        this.q.SetIdentity();
        return this;
    };
    b2Transform.prototype.SetPositionRotation = function (position, q) {
        this.p.Copy(position);
        this.q.Copy(q);
        return this;
    };
    b2Transform.prototype.SetPositionAngle = function (pos, a) {
        this.p.Copy(pos);
        this.q.SetAngle(a);
        return this;
    };
    b2Transform.prototype.SetPosition = function (position) {
        this.p.Copy(position);
        return this;
    };
    b2Transform.prototype.SetPositionXY = function (x, y) {
        this.p.Set(x, y);
        return this;
    };
    b2Transform.prototype.SetRotation = function (rotation) {
        this.q.Copy(rotation);
        return this;
    };
    b2Transform.prototype.SetRotationAngle = function (radians) {
        this.q.SetAngle(radians);
        return this;
    };
    b2Transform.prototype.GetPosition = function () {
        return this.p;
    };
    b2Transform.prototype.GetRotation = function () {
        return this.q;
    };
    b2Transform.prototype.GetRotationAngle = function () {
        return this.q.GetAngle();
    };
    b2Transform.prototype.GetAngle = function () {
        return this.q.GetAngle();
    };
    b2Transform.MulXV = function (T, v, out) {
        // float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        // float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
        // return b2Vec2(x, y);
        var T_q_c = T.q.c, T_q_s = T.q.s;
        var v_x = v.x, v_y = v.y;
        out.x = (T_q_c * v_x - T_q_s * v_y) + T.p.x;
        out.y = (T_q_s * v_x + T_q_c * v_y) + T.p.y;
        return out;
    };
    b2Transform.MulTXV = function (T, v, out) {
        // float32 px = v.x - T.p.x;
        // float32 py = v.y - T.p.y;
        // float32 x = (T.q.c * px + T.q.s * py);
        // float32 y = (-T.q.s * px + T.q.c * py);
        // return b2Vec2(x, y);
        var T_q_c = T.q.c, T_q_s = T.q.s;
        var p_x = v.x - T.p.x;
        var p_y = v.y - T.p.y;
        out.x = (T_q_c * p_x + T_q_s * p_y);
        out.y = (-T_q_s * p_x + T_q_c * p_y);
        return out;
    };
    b2Transform.MulXX = function (A, B, out) {
        b2Rot.MulRR(A.q, B.q, out.q);
        b2Vec2.AddVV(b2Rot.MulRV(A.q, B.p, out.p), A.p, out.p);
        return out;
    };
    b2Transform.MulTXX = function (A, B, out) {
        b2Rot.MulTRR(A.q, B.q, out.q);
        b2Rot.MulTRV(A.q, b2Vec2.SubVV(B.p, A.p, out.p), out.p);
        return out;
    };
    b2Transform.IDENTITY = new b2Transform();
    return b2Transform;
}());
exports.b2Transform = b2Transform;
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
var b2Sweep = /** @class */ (function () {
    function b2Sweep() {
        this.localCenter = new b2Vec2();
        this.c0 = new b2Vec2();
        this.c = new b2Vec2();
        this.a0 = 0;
        this.a = 0;
        this.alpha0 = 0;
    }
    b2Sweep.prototype.Clone = function () {
        return new b2Sweep().Copy(this);
    };
    b2Sweep.prototype.Copy = function (other) {
        this.localCenter.Copy(other.localCenter);
        this.c0.Copy(other.c0);
        this.c.Copy(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.alpha0 = other.alpha0;
        return this;
    };
    b2Sweep.prototype.GetTransform = function (xf, beta) {
        var one_minus_beta = (1 - beta);
        xf.p.x = one_minus_beta * this.c0.x + beta * this.c.x;
        xf.p.y = one_minus_beta * this.c0.y + beta * this.c.y;
        var angle = one_minus_beta * this.a0 + beta * this.a;
        xf.q.SetAngle(angle);
        xf.p.SelfSub(b2Rot.MulRV(xf.q, this.localCenter, b2Vec2.s_t0));
        return xf;
    };
    b2Sweep.prototype.Advance = function (alpha) {
        // DEBUG: b2Assert(this.alpha0 < 1);
        var beta = (alpha - this.alpha0) / (1 - this.alpha0);
        var one_minus_beta = (1 - beta);
        this.c0.x = one_minus_beta * this.c0.x + beta * this.c.x;
        this.c0.y = one_minus_beta * this.c0.y + beta * this.c.y;
        this.a0 = one_minus_beta * this.a0 + beta * this.a;
        this.alpha0 = alpha;
    };
    b2Sweep.prototype.Normalize = function () {
        var d = exports.b2_two_pi * Math.floor(this.a0 / exports.b2_two_pi);
        this.a0 -= d;
        this.a -= d;
    };
    return b2Sweep;
}());
exports.b2Sweep = b2Sweep;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYjJNYXRoLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vQm94MkQvQm94MkQvQ29tbW9uL2IyTWF0aC50cyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiO0FBQUE7Ozs7Ozs7Ozs7Ozs7Ozs7RUFnQkU7O0FBRUYsa0RBQWtEO0FBQ2xELDJDQUE4RDtBQUVqRCxRQUFBLGNBQWMsR0FBVyxrQkFBSyxHQUFHLEdBQUcsQ0FBQztBQUNyQyxRQUFBLGNBQWMsR0FBVyxHQUFHLEdBQUcsa0JBQUssQ0FBQztBQUNyQyxRQUFBLFNBQVMsR0FBVyxDQUFDLEdBQUcsa0JBQUssQ0FBQztBQUU5QixRQUFBLEtBQUssR0FBRyxJQUFJLENBQUMsR0FBRyxDQUFDO0FBRWpCLFFBQUEsS0FBSyxHQUFHLElBQUksQ0FBQyxHQUFHLENBQUM7QUFDakIsUUFBQSxLQUFLLEdBQUcsSUFBSSxDQUFDLEdBQUcsQ0FBQztBQUU5QixpQkFBd0IsQ0FBUyxFQUFFLEVBQVUsRUFBRSxFQUFVO0lBQ3ZELE9BQU8sQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLEdBQUcsRUFBRSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztBQUNuRCxDQUFDO0FBRkQsMEJBRUM7QUFFRCxnQkFBMEIsQ0FBTSxFQUFFLENBQU07SUFDdEMsMEJBQTBCO0lBQzFCLElBQU0sR0FBRyxHQUFNLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztJQUNwQixDQUFDLENBQUMsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDO0lBQ1osQ0FBQyxDQUFDLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztBQUNiLENBQUM7QUFMRCx3QkFLQztBQUVELG1FQUFtRTtBQUNuRSwwQkFBMEI7QUFDYixR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LENBQUMsQ0FBQyxDQUFDO1FBQ2hGLElBQU0sR0FBRyxHQUFXLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxFQUFFLEdBQUcsR0FBVyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsRUFBRSxHQUFHLEdBQVcsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUM7UUFDaEYsSUFBSSxHQUFHLEdBQVcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUM7UUFDaEgsSUFBSSxHQUFHLEtBQUssQ0FBQyxFQUFFO1lBQ2IsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLENBQUM7U0FDZjtRQUNELEdBQUcsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUMsQ0FBQztRQUM5RyxHQUFHLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLENBQUM7UUFDOUcsR0FBRyxDQUFDLENBQUMsR0FBRyxHQUFH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QUNiLEdBQUcsR0FBRyxDQUFDLEdBQUcsR0FBRyxDQUFDO1NBQ2Y7UUFFRCxJQUFNLEdBQUcsR0FBVyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsRUFBRSxHQUFHLEdBQVcsSUFBSSxDQUFDLEVBQUUsQ0FBQyxDQUFDLEVBQUUsR0FBRyxHQUFXLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFDO1FBQ2hGLElBQU0sR0FBRyxHQUFXLElBQUksQ0FBQyxFQUFFLENBQUMsQ0FBQyxFQUFFLEdBQUcsR0FBVyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUN2RCxJQUFNLEdBQUcsR0FBVyxJQUFJLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUU5QixDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztRQUN2QyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztRQUN2QyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztRQUV2QyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNoQixDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztRQUN2QyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztRQUV2QyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNoQixDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQztRQUNoQixDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxHQUFHLEdBQUcsR0FBRyxHQUFHLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQztJQUN6QyxDQUFDO0lBRWEsZ0JBQVEsR0FBdEIsVUFBc0MsQ0FBVSxFQUFFLENBQU0sRUFBRSxHQUFNO1FBQzlELElBQU0sR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLEVBQUUsR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLEVBQUUsR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDOUQsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztRQUNuRCxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxDQUFDO1FBQ25ELEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLENBQUM7UUFDbkQsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBQ2EsaUJBQVMsR0FBdkIsVUFBdUMsQ0FBVSxFQUFFLENBQVMsRUFBRSxDQUFTLEVBQUUsQ0FBUyxFQUFFLEdBQU07UUFDeEYsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQztRQUM3QyxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQzdDLEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUM7UUFDN0MsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBQ2EsZ0JBQVEsR0FBdEIsVUFBcUMsQ0FBVSxFQUFFLENBQUssRUFBRSxHQUFNO1FBQzVELElBQU0sR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLEVBQUUsR0FBRyxHQUFXLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDM0MsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBRyxHQUFHLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxDQUFDO1FBQ3BDLEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUcsR0FBRyxHQUFHLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFHLEdBQUcsQ0FBQztRQUNwQyxPQUFPLEdBQUcsQ0FBQztJQUNiLENBQUM7SUFDYSxnQkFBUSxHQUF0QixVQUFxQyxDQUFVLEVBQUUsQ0FBUyxFQUFFLENBQVMsRUFBRSxHQUFNO1FBQzNFLEdBQUcsQ0FBQyxDQUFDLEdBQUcsQ0FB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LEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDaEMsSUFBTSxHQUFHLEdBQVcsQ0FBQyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsQ0FBQyxDQUFDLENBQUMsQ0FBQztRQUNoQyxHQUFHLENBQUMsQ0FBQyxHQUFHLENBQUUsS0FBSyxHQUFHLEdBQUcsR0FBRyxLQUFLLEdBQUcsR0FBRyxDQUFDLENBQUM7UUFDckMsR0FBRyxDQUFDLENBQUMsR0FBRyxDQUFDLENBQUMsS0FBSyxHQUFHLEdBQUcsR0FBRyxLQUFLLEdBQUcsR0FBRyxDQUFDLENBQUM7UUFDckMsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRWEsaUJBQUssR0FBbkIsVUFBb0IsQ0FBYyxFQUFFLENBQWMsRUFBRSxHQUFnQjtRQUNsRSxLQUFLLENBQUMsS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUMsRUFBRSxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDN0IsTUFBTSxDQUFDLEtBQUssQ0FBQyxLQUFLLENBQUMsS0FBSyxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUMsRUFBRSxHQUFHLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUMsRUFBRSxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDdkQsT0FBTyxHQUFHLENBQUM7SUFDYixDQUFDO0lBRWEsa0JBQU0sR0FBcEIsVUFBcUIsQ0FBYyxFQUFFLENBQWMsRUFBRSxHQUFnQjtRQUNuRSxLQUFLLENBQUMsTUFBTSxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUMsRUFBRSxHQUFHLENBQUMsQ0FBQyxDQUFDLENBQUM7UUFDOUIsS0FBSyxDQUFDLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQyxFQUFFLE1BQU0sQ0FBQyxLQUFLLENBQUMsQ0FBQyxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQyxFQUFFLEdBQUcsQ0FBQyxDQUFDLENBQUMsRUFBRSxHQUFHLENB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