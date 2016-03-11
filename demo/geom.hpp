#pragma once

#include <cmath>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>

#if 1

using Vec3 = glm::fvec3;
using Vec4 = glm::fvec4;

#else

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


#if 1

using Quat = glm::fquat;

#else

struct Quat
{
    float x, y, z, w;
    Quat() : x(0), y(0), z(0), w(1) {}
    Quat(float w, float x, float y, float z) : x(x), y(y), z(z), w(w) {}
    float &operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }

    /*
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
    */
};

namespace {

float length2(const Quat& q) {
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

float length(const Quat& q) {
  return sqrtf(length2(q));
}

Quat operator*(Quat q, float k) {
  q.x *= k;
  q.y *= k;
  q.z *= k;
  q.w *= k;
  return q;
}

Quat normalize(Quat q) { 
  return q * (1.0f / length(q));
}

}

#endif



struct Matrix3x3
{
    Vec3 a, b, c;

    Matrix3x3() {}
    Matrix3x3(const Vec3 &a, const Vec3 &b, const Vec3 &c) : a(a), b(b), c(c) {}

    explicit Matrix3x3(const Quat &q) { 
        float x = q.x, y = q.y, z = q.z, w = q.w,
              tx = 2*x, ty = 2*y, tz = 2*z,
              txx = tx*x, tyy = ty*y, tzz = tz*z,
              txy = tx*y, txz = tx*z, tyz = ty*z,
              twx = w*tx, twy = w*ty, twz = w*tz;
        a = Vec3(1 - (tyy + tzz), txy - twz, txz + twy);
        b = Vec3(txy + twz, 1 - (txx + tzz), tyz - twx);
        c = Vec3(txz - twy, tyz + twx, 1 - (txx + tyy));
    }

    explicit Matrix3x3(const Vec3 &scale) :
      a(scale.x,0,0), b(0,scale.y,0), c(0,0,scale.z) {}

    Matrix3x3 operator+(const Matrix3x3 &o) const { return Matrix3x3(*this) += o; }
    Matrix3x3 &operator+=(const Matrix3x3 &o)
    {
        a += o.a;
        b += o.b;
        c += o.c;
        return *this;
    }

    Matrix3x3 operator-(const Matrix3x3 &o) const { return Matrix3x3(*this) -= o; }
    Matrix3x3 &operator-=(const Matrix3x3 &o)
    {
        a -= o.a;
        b -= o.b;
        c -= o.c;
        return *this;
    }

    Matrix3x3 operator*(const Matrix3x3 &o) const
    {
        return Matrix3x3(
            o.a*a.x + o.b*a.y + o.c*a.z,
            o.a*b.x + o.b*b.y + o.c*b.z,
            o.a*c.x + o.b*c.y + o.c*c.z);
    }
    Matrix3x3 &operator*=(const Matrix3x3 &o) { return (*this = *this * o); }
};


#if 0
inline void test() {
  Quat q1;
  q1.x = 1; q1.y = 2; q1.z = 3; q1.w = 4;
  auto q2 = *reinterpret_cast<glm::fquat*>(&q1);
  auto q3 = glm::fquat(1,2,3,4);

  printf("q1.x: %f %f %f %f\n", q1.x, q1.y, q1.z, q1.w);
  printf("q2.x: %f %f %f %f\n", q2.x, q2.y, q2.z, q2.w);
  printf("q3.x: %f %f %f %f\n", q3.x, q3.y, q3.z, q3.w);

  
  printf("length(q1): %f\n", length(q1));
  printf("length(q2): %f\n", length(q2));

  auto n1 = normalize(q1);
  auto n2 = normalize(q2);

  printf("normalized(q1) : %0.3f %0.3f %0.3f %0.3f \n", n1.x, n1.y, n1.z, n1.w);
  printf("normalized(q1) : %0.3f %0.3f %0.3f %0.3f \n", n2.x, n2.y, n2.z, n2.w);

  Quat default1;
  glm::fquat default2;

  printf("default1.x: %f %f %f %f\n", default1.x, default1.y, default1.z, default1.w);
  printf("default2.x: %f %f %f %f\n", default2.x, default2.y, default2.z, default2.w);

  auto m1 = Matrix3x3(q1);
  auto m2 = glm::fmat3(q2);

  printf("m1:\n");
  printf("  %f %f %f\n", m1.a.x, m1.a.y, m1.a.z);
  printf("  %f %f %f\n", m1.b.x, m1.b.y, m1.b.z);
  printf("  %f %f %f\n", m1.c.x, m1.c.y, m1.c.z);
  printf("m2:\n");
  printf("  %f %f %f\n", m2[0].x, m2[0].y, m2[0].z);
  printf("  %f %f %f\n", m2[1].x, m2[1].y, m2[1].z);
  printf("  %f %f %f\n", m2[2].x, m2[2].y, m2[2].z);
}
#endif

struct Matrix4x3
{
    Vec3 a, b, c, d;
    
    Matrix4x3(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d) : a(a), b(b), c(c), d(d) {}
    explicit Matrix4x3(const Matrix3x3 &rot, const Vec3 &trans)
        : a(rot.a), b(rot.b), c(rot.c), d(trans) {}
};

struct Matrix3x4
{
    Vec4 a, b, c;

    Matrix3x4() {}
    Matrix3x4(const Vec4 &a, const Vec4 &b, const Vec4 &c) : a(a), b(b), c(c) {}
    
    explicit Matrix3x4(const Matrix3x3 &rot, const Vec3 &trans)
        : a(Vec4(rot.a, trans.x)), b(Vec4(rot.b, trans.y)), c(Vec4(rot.c, trans.z)) {}

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

    Matrix3x4 operator-(const Matrix3x4 &o) const { return Matrix3x4(*this) -= o; }
    Matrix3x4 &operator-=(const Matrix3x4 &o)
    {
        a -= o.a;
        b -= o.b;
        c -= o.c;
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

namespace {
  
inline Matrix3x3 transpose(const Matrix3x3& mat) {
  return Matrix3x3(
    Vec3(mat.a.x, mat.b.x, mat.c.x), 
    Vec3(mat.a.y, mat.b.y, mat.c.y),
    Vec3(mat.a.z, mat.b.z, mat.c.z)
  );
}


inline Matrix4x3 transpose(const Matrix3x4& mat) {
  return Matrix4x3(
    Vec3(mat.a.x, mat.b.x, mat.c.x),
    Vec3(mat.a.y, mat.b.y, mat.c.y),
    Vec3(mat.a.z, mat.b.z, mat.c.z),
    Vec3(mat.a.w, mat.b.w, mat.c.w)
  );
}

inline Matrix3x4 transpose(const Matrix4x3& mat) {
  return Matrix3x4(
    Vec4(mat.a.x, mat.b.x, mat.c.x, mat.d.x),
    Vec4(mat.a.y, mat.b.y, mat.c.y, mat.d.y),
    Vec4(mat.a.z, mat.b.z, mat.c.z, mat.d.z)
  );
}

inline Vec3 operator*(const Matrix4x3& mat, const Vec4& v) {
  return Vec3( mat.a * v.x + mat.b * v.y + mat.c * v.z );
}

inline Vec3 operator*(const Matrix3x3& mat, const Vec3& v) {
  return Vec3( mat.a * v.x + mat.b * v.y + mat.c * v.z );
}

inline void print(const Matrix3x3& m) {
  printf("    %f %f %f\n", m.a.x, m.a.y, m.a.z);
  printf("    %f %f %f\n", m.b.x, m.b.y, m.b.z);
  printf("    %f %f %f\n", m.c.x, m.c.y, m.c.z);
}

inline void print(const Matrix3x4& m) {
  printf("    %f %f %f %f\n", m.a.x, m.a.y, m.a.z, m.a.w);
  printf("    %f %f %f %f\n", m.b.x, m.b.y, m.b.z, m.a.w);
  printf("    %f %f %f %f\n", m.c.x, m.c.y, m.c.z, m.a.w);
}

inline void print(Quat q) {
  printf("    %f %f %f %f\n", q[0], q[1], q[2], q[3]);
}

inline void print(Vec4 q) {
  printf("    %f %f %f %f\n", q[0], q[1], q[2], q[3]);
}

inline void print(Vec3 q) {
  printf("    %f %f %f\n", q[0], q[1], q[2]);
}

}
