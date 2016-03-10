#pragma once

#include <cmath>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#if 0
struct Vec4;

struct Vec3
{
    float x, y, z;

    Vec3() {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    float &operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }

    bool operator==(const Vec3 &o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const Vec3 &o) const { return x != o.x || y != o.y || z != o.z; }

    Vec3 operator+(const Vec3 &o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3 &o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator+(float k) const { return Vec3(x+k, y+k, z+k); }
    Vec3 operator-(float k) const { return Vec3(x-k, y-k, z-k); }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator*(const Vec3 &o) const { return Vec3(x*o.x, y*o.y, z*o.z); }
    Vec3 operator/(const Vec3 &o) const { return Vec3(x/o.x, y/o.y, z/o.z); }
    Vec3 operator*(float k) const { return Vec3(x*k, y*k, z*k); }
    Vec3 operator/(float k) const { return Vec3(x/k, y/k, z/k); }

    Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3 &operator+=(float k) { x += k; y += k; z += k; return *this; }
    Vec3 &operator-=(float k) { x -= k; y -= k; z -= k; return *this; }
    Vec3 &operator*=(const Vec3 &o) { x *= o.x; y *= o.y; z *= o.z; return *this; }
    Vec3 &operator/=(const Vec3 &o) { x /= o.x; y /= o.y; z /= o.z; return *this; }
    Vec3 &operator*=(float k) { x *= k; y *= k; z *= k; return *this; }
    Vec3 &operator/=(float k) { x /= k; y /= k; z /= k; return *this; }
};

struct Vec4
{ 
    float x, y, z, w; 

    Vec4() {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    explicit Vec4(const Vec3 &p, float w = 0) : x(p.x), y(p.y), z(p.z), w(w) {}

    float &operator[](int i)       { return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }

    bool operator==(const Vec4 &o) const { return x == o.x && y == o.y && z == o.z && w == o.w; }
    bool operator!=(const Vec4 &o) const { return x != o.x || y != o.y || z != o.z || w != o.w; }

    Vec4 operator+(const Vec4 &o) const { return Vec4(x+o.x, y+o.y, z+o.z, w+o.w); }
    Vec4 operator-(const Vec4 &o) const { return Vec4(x-o.x, y-o.y, z-o.z, w-o.w); }
    Vec4 operator+(float k) const { return Vec4(x+k, y+k, z+k, w+k); }
    Vec4 operator-(float k) const { return Vec4(x-k, y-k, z-k, w-k); }
    Vec4 operator-() const { return Vec4(-x, -y, -z, -w); }
    Vec4 operator*(float k) const { return Vec4(x*k, y*k, z*k, w*k); }
    Vec4 operator/(float k) const { return Vec4(x/k, y/k, z/k, w/k); }

    Vec4 &operator+=(const Vec4 &o) { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
    Vec4 &operator-=(const Vec4 &o) { x -= o.x; y -= o.y; z -= o.z; w -= o.w; return *this; }
    Vec4 &operator+=(float k) { x += k; y += k; z += k; w += k; return *this; }
    Vec4 &operator-=(float k) { x -= k; y -= k; z -= k; w -= k; return *this; }
    Vec4 &operator*=(float k) { x *= k; y *= k; z *= k; w *= k; return *this; }
    Vec4 &operator/=(float k) { x /= k; y /= k; z /= k; w /= k; return *this; }
};

namespace {

inline float dot(const Vec3& v1, const Vec3& v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline float dot(const Vec4& v1, const Vec4& v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

inline Vec3 cross(const Vec3& v1, const Vec3& v2) {
    return Vec3(v1.y*v2.z-v1.z*v2.y, v1.z*v2.x-v1.x*v2.z, v1.x*v2.y-v1.y*v2.x);
}

inline float length2(const Vec3& v) { return dot(v,v); }
inline float length( const Vec3& v) { return sqrtf(length2(v)); }
inline float length2(const Vec4& v) { return dot(v,v); }
inline float length(const Vec4& v)  { return sqrtf(length2(v)); }
inline Vec3 normalize(const Vec3& v) { return v * (1 / length(v)); }
inline Vec4 normalize(const Vec4& v) { return v * (1 / length(v)); }
// Vec3 reflect(const Vec3& v, const Vec3 &n) { return v - n*2.0f*dot(v, n); }
// Vec3 project(const Vec3& v, const Vec3 &n) { return v - n*dot(v, n); }

}

#endif

struct Matrix3x3;
struct Matrix3x4;

using Vec3 = glm::fvec3;
using Vec4 = glm::fvec4;

struct Quat
{
    float x,y,z,w;

    Quat() {}
    Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Quat(float angle, const Vec3 &axis)
    {
        float s = sinf(0.5f*angle);
        x = s*axis.x;
        y = s*axis.y;
        z = s*axis.z;
        w = cosf(0.5f*angle);
    }
    explicit Quat(const Vec3 &v) : x(v.x), y(v.y), z(v.z), 
            w(-sqrtf(std::max(1.0f - length2(v), 0.0f))) {}
    //explicit Quat(const float *v) : x(v[0]), y(v[1]), z(v[2]), w(v[3]) {}
    explicit Quat(const Matrix3x3 &m) { convertmatrix(m); }

    void restorew()
    {
        auto tmp = Vec3(x,y,z);
        w = -sqrtf(std::max(1.0f - length2(tmp), 0.0f));
    }

    Quat normalize() const { return *this * (1.0f / length(Vec4(x,y,z,w))); }

    Quat operator*(float k) const { return Quat(x*k, y*k, z*k, w*k); }
    Quat &operator*=(float k) { return (*this = *this * k); }

    Quat operator*(const Quat &o) const
    {
        return Quat(w*o.x + x*o.w + y*o.z - z*o.y,
                    w*o.y - x*o.z + y*o.w + z*o.x,
                    w*o.z + x*o.y - y*o.x + z*o.w,
                    w*o.w - x*o.x - y*o.y - z*o.z);
    }
    Quat &operator*=(const Quat &o) { return (*this = *this * o); }

    Quat operator+(const Vec4 &o) const { return Quat(x+o.x, y+o.y, z+o.z, w+o.w); }
    Quat &operator+=(const Vec4 &o) { return (*this = *this + o); }
    Quat operator-(const Vec4 &o) const { return Quat(x-o.x, y-o.y, z-o.z, w-o.w); }
    Quat &operator-=(const Vec4 &o) { return (*this = *this - o); }

    Vec3 transform(const Vec3 &p) const
    {
        Vec3 tmp = Vec3(x,y,z);
        return p + cross(tmp, cross(tmp,p) + p*w)*2.0f;
    }

    void calcangleaxis(float &angle, Vec3 &axis)
    {
        auto tmp = Vec3(x,y,z);
        float rr = length2(tmp);
        if(rr > 0)
        {
            angle = 2*acosf(w);
            axis = tmp * (1 / rr);
        }
        else { angle = 0; axis = Vec3(0, 0, 1); }
    }
     
    template<class M>
    void convertmatrix(const M &m)
    {
        float trace = m.a.x + m.b.y + m.c.z;
        if(trace>0)
        {
            float r = sqrtf(1 + trace), inv = 0.5f/r;
            w = 0.5f*r;
            x = (m.c.y - m.b.z)*inv;
            y = (m.a.z - m.c.x)*inv;
            z = (m.b.x - m.a.y)*inv;
        }
        else if(m.a.x > m.b.y && m.a.x > m.c.z)
        {
            float r = sqrtf(1 + m.a.x - m.b.y - m.c.z), inv = 0.5f/r;
            x = 0.5f*r;
            y = (m.b.x + m.a.y)*inv;
            z = (m.a.z + m.c.x)*inv;
            w = (m.c.y - m.b.z)*inv;
        }
        else if(m.b.y > m.c.z)
        {
            float r = sqrtf(1 + m.b.y - m.a.x - m.c.z), inv = 0.5f/r;
            x = (m.b.x + m.a.y)*inv;
            y = 0.5f*r;
            z = (m.c.y + m.b.z)*inv;
            w = (m.a.z - m.c.x)*inv;
        }
        else
        {
            float r = sqrtf(1 + m.c.z - m.a.x - m.b.y), inv = 0.5f/r;
            x = (m.a.z + m.c.x)*inv;
            y = (m.c.y + m.b.z)*inv;
            z = 0.5f*r;
            w = (m.b.x - m.a.y)*inv;
        }
    }
};

struct Matrix3x3
{
    Vec3 a, b, c;

    Matrix3x3() {}
    Matrix3x3(const Vec3 &a, const Vec3 &b, const Vec3 &c) : a(a), b(b), c(c) {}
    explicit Matrix3x3(const Quat &q) { convertquat(q); }
    explicit Matrix3x3(const Quat &q, const Vec3 &scale)
    {
        convertquat(q);
        a *= scale;
        b *= scale;
        c *= scale;
    }

    void convertquat(const Quat &q)
    {
        float x = q.x, y = q.y, z = q.z, w = q.w,
              tx = 2*x, ty = 2*y, tz = 2*z,
              txx = tx*x, tyy = ty*y, tzz = tz*z,
              txy = tx*y, txz = tx*z, tyz = ty*z,
              twx = w*tx, twy = w*ty, twz = w*tz;
        a = Vec3(1 - (tyy + tzz), txy - twz, txz + twy);
        b = Vec3(txy + twz, 1 - (txx + tzz), tyz - twx);
        c = Vec3(txz - twy, tyz + twx, 1 - (txx + tyy));
    }

    Matrix3x3 operator*(const Matrix3x3 &o) const
    {
        return Matrix3x3(
            o.a*a.x + o.b*a.y + o.c*a.z,
            o.a*b.x + o.b*b.y + o.c*b.z,
            o.a*c.x + o.b*c.y + o.c*c.z);
    }
    Matrix3x3 &operator*=(const Matrix3x3 &o) { return (*this = *this * o); }

    Vec3 transform(const Vec3 &o) const { return Vec3(dot(a,o), dot(b,o), dot(c,o)); }
    Vec3 transposedtransform(const Vec3 &o) const { return a*o.x + b*o.y + c*o.z; }
};

struct Matrix3x4
{
    Vec4 a, b, c;

    Matrix3x4() {}
    Matrix3x4(const Vec4 &a, const Vec4 &b, const Vec4 &c) : a(a), b(b), c(c) {}
    explicit Matrix3x4(const Matrix3x3 &rot, const Vec3 &trans)
        : a(Vec4(rot.a, trans.x)), b(Vec4(rot.b, trans.y)), c(Vec4(rot.c, trans.z))
    {
    }
    explicit Matrix3x4(const Quat &rot, const Vec3 &trans)
    {
        *this = Matrix3x4(Matrix3x3(rot), trans);
    }
    explicit Matrix3x4(const Quat &rot, const Vec3 &trans, const Vec3 &scale)
    {
        *this = Matrix3x4(Matrix3x3(rot, scale), trans);
    }

    Matrix3x4 operator*(float k) const { return Matrix3x4(*this) *= k; }
    Matrix3x4 &operator*=(float k)
    {
        a *= k;
        b *= k;
        c *= k;
        return *this;
    }

    Matrix3x4 operator+(const Matrix3x4 &o) const { return Matrix3x4(*this) += o; }
    Matrix3x4 &operator+=(const Matrix3x4 &o)
    {
        a += o.a;
        b += o.b;
        c += o.c;
        return *this;
    }

    void invert(const Matrix3x4 &o)
    {
        Matrix3x3 invrot(Vec3(o.a.x, o.b.x, o.c.x), Vec3(o.a.y, o.b.y, o.c.y), Vec3(o.a.z, o.b.z, o.c.z));
        invrot.a /= length2(invrot.a);
        invrot.b /= length2(invrot.b);
        invrot.c /= length2(invrot.c);
        Vec3 trans(o.a.w, o.b.w, o.c.w);
        a = Vec4(invrot.a, -dot(invrot.a, trans));
        b = Vec4(invrot.b, -dot(invrot.b, trans));
        c = Vec4(invrot.c, -dot(invrot.c, trans));
    }
    void invert() { invert(Matrix3x4(*this)); }

    Matrix3x4 operator*(const Matrix3x4 &o) const
    {
        return Matrix3x4(
          o.a*a.x + o.b*a.y + o.c*a.z + Vec4(0,0,0,a.w),
          o.a*b.x + o.b*b.y + o.c*b.z + Vec4(0,0,0,b.w),
          o.a*c.x + o.b*c.y + o.c*c.z + Vec4(0,0,0,c.w)
        );
    }

    Matrix3x4 &operator*=(const Matrix3x4 &o) { return (*this = *this * o); }

    Vec3 transform(const Vec3 &o) const { 
      return Vec3(
          dot(a, Vec4(o,1)),
          dot(b, Vec4(o,1)),
          dot(c, Vec4(o,1))
      );
    }

    Vec3 transformnormal(const Vec3 &o) const { 
      return Vec3(
          dot(a, Vec4(o,0)),
          dot(b, Vec4(o,0)),
          dot(c, Vec4(o,0))
      );
    }
};

