#pragma once

#include <cmath>
#include <algorithm>


#if 1

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_operation.hpp>

using Vec2 = glm::fvec2;
using Vec3 = glm::fvec3;
using Vec4 = glm::fvec4;
using Vec4u8 = glm::u8vec4;
using Quat = glm::fquat;

using glm::clamp;

using Matrix3x3 = glm::fmat3x3;
using Matrix3x4 = glm::fmat3x4;
using Matrix4x3 = glm::fmat4x3;
using Matrix4x4 = glm::fmat4x4;

#else

struct Vec4u8 {
  uint8_t x, y, z, w;

  Vec4u8() : x(0), y(0), z(0), w(0) {}
  Vec4u8(uint8_t x, uint8_t y, uint8_t z, uint8_t w) : x(x), y(y), z(z), w(w) {}
};

struct Vec2
{
  float x,y;
  Vec2() : x(0), y(0) {}
  Vec2(float x, float y) : x(x), y(y) {}

  float &operator[](long i) { return (&x)[i]; }
};

struct Vec3
{
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    float &operator[](long i) { return (&x)[i]; }
    float operator[](long i) const { return (&x)[i]; }

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

    float &operator[](long i)       { return (&x)[i]; }
    float  operator[](long i) const { return (&x)[i]; }

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

inline float clamp(float x, float lower, float upper) {
  return std::min(std::max(x, lower), upper);
}

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

struct Quat
{
    float x, y, z, w;
    Quat() : x(0), y(0), z(0), w(1) {}
    Quat(float w, float x, float y, float z) : x(x), y(y), z(z), w(w) {}
    float &operator[](long i) { return (&x)[i]; }
    float operator[](long i) const { return (&x)[i]; }

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

struct Matrix3x3;
struct Matrix4x3;
struct Matrix3x4;
struct Matrix4x4;

void print(const Matrix3x3&);

struct Matrix3x3
{
    Vec3 a, b, c;

    Matrix3x3() {}
    Matrix3x3(const Vec3 &a, const Vec3 &b, const Vec3 &c) : a(a), b(b), c(c) {}

    Vec3& operator[](long i) { return (&a)[i]; }
    Vec3 operator[](long i) const { return (&a)[i]; }

    explicit Matrix3x3(const Matrix4x3& m);
    explicit Matrix3x3(const Matrix3x4& m);

    explicit Matrix3x3(const Quat &q) { 
        float x = q.x, y = q.y, z = q.z, w = q.w,
              tx = 2*x, ty = 2*y, tz = 2*z,
              txx = tx*x, tyy = ty*y, tzz = tz*z,
              txy = tx*y, txz = tx*z, tyz = ty*z,
              twx = w*tx, twy = w*ty, twz = w*tz;
        a = Vec3(1 - (tyy + tzz),      txy + twz ,      txz - twy);
        b = Vec3(     txy - twz , 1 - (txx + tzz),      tyz + twx);
        c = Vec3(     txz + twy ,      tyz - twx , 1 - (txx + tyy));
    }


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

};

struct Matrix4x4
{
    Vec4 a, b, c, d;

    Matrix4x4() {}
    Matrix4x4(const Vec4 &a, const Vec4 &b, const Vec4 &c, const Vec4 &d) : a(a), b(b), c(c), d(d) {}

    Vec4& operator[](long i) { return (&a)[i]; }
    Vec4 operator[](long i) const { return (&a)[i]; }

};

struct Matrix4x3
{
    Vec3 a, b, c, d;
    
    Matrix4x3(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d) : a(a), b(b), c(c), d(d) {}
    explicit Matrix4x3(const Matrix4x4& mat) : 
      a(mat[0].x, mat[0].y, mat[0].z),
      b(mat[1].x, mat[1].y, mat[1].z),
      c(mat[2].x, mat[2].y, mat[2].z),
      d(mat[3].x, mat[3].y, mat[3].z)
  {}

    //explicit Matrix4x3(const Matrix3x3 &rot, const Vec3 &trans)
    //    : a(rot.a), b(rot.b), c(rot.c), d(trans) {}

    Vec3& operator[](long i) { return (&a)[i]; }
    Vec3 operator[](long i) const { return (&a)[i]; }
};

struct Matrix3x4
{
    Vec4 a, b, c;

    Matrix3x4() {}
    Matrix3x4(const Vec4 &a, const Vec4 &b, const Vec4 &c) : a(a), b(b), c(c) {}
    explicit Matrix3x4(const Matrix4x4& mat) : a(mat[0]), b(mat[1]), c(mat[2]) {}
    
    Vec4& operator[](long i) { return (&a)[i]; }
    Vec4 operator[](long i) const { return (&a)[i]; }

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


    /*
    Matrix3x4 operator*(const Matrix3x4 &o) const
    {
        return Matrix3x4(
          o.a*a.x + o.b*a.y + o.c*a.z + Vec4(0,0,0,a.w),
          o.a*b.x + o.b*b.y + o.c*b.z + Vec4(0,0,0,b.w),
          o.a*c.x + o.b*c.y + o.c*c.z + Vec4(0,0,0,c.w)
        );
    }

    Matrix3x4 &operator*=(const Matrix3x4 &o) { return (*this = *this * o); }
    */
};

Matrix3x3::Matrix3x3(const Matrix4x3& m) : a(m.a), b(m.b), c(m.c) {}
Matrix3x3::Matrix3x3(const Matrix3x4& m) : a(m.a.x, m.a.y, m.a.z), b(m.b.x, m.b.y, m.b.z), c(m.c.x, m.c.y, m.c.z) {}

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

inline Matrix4x4 transpose(const Matrix4x4& mat) {
  return Matrix4x4(
    Vec4(mat.a.x, mat.b.x, mat.c.x, mat.d.x),
    Vec4(mat.a.y, mat.b.y, mat.c.y, mat.d.y),
    Vec4(mat.a.z, mat.b.z, mat.c.z, mat.d.z),
    Vec4(mat.a.w, mat.b.w, mat.c.w, mat.d.w)
  );
}

inline Matrix3x3 diagonal3x3(const Vec3& scale) {
   return Matrix3x3(Vec3(scale.x,0,0), Vec3(0,scale.y,0), Vec3(0,0,scale.z));
}

inline Vec4 operator*(const Matrix4x4& mat, const Vec4& v) {
  return mat.a * v.x + mat.b * v.y + mat.c * v.z + mat.d * v.w;
}

inline Vec4 operator*(float f, const Vec4& v) { return v*f; }

inline Vec3 operator*(const Matrix4x3& mat, const Vec4& v) {
  return mat.a * v.x + mat.b * v.y + mat.c * v.z + mat.d * v.w;
}

inline Vec3 operator*(float f, const Vec3& v) { return v*f; }

inline Vec3 operator*(const Matrix3x3& mat, const Vec3& v) {
  return mat.a * v.x + mat.b * v.y + mat.c * v.z;
}

Matrix3x3 operator*(const Matrix3x3& mat1, const Matrix3x3 &mat2) {
  return Matrix3x3(mat1 * mat2.a, mat1 * mat2.b, mat1 * mat2.c);
}

#if 0
Matrix4x4 operator*(const Matrix4x4& mat1, const Matrix4x4 &mat2) {
    return Matrix4x4(
        mat1.a.x * mat2.a + mat1.a.y * mat2.b + mat1.a.z * mat2.c + mat1.a.w * mat2.d,
        mat1.b.x * mat2.a + mat1.b.y * mat2.b + mat1.b.z * mat2.c + mat1.b.w * mat2.d,
        mat1.c.x * mat2.a + mat1.c.y * mat2.b + mat1.c.z * mat2.c + mat1.c.w * mat2.d,
        mat1.d.x * mat2.a + mat1.d.y * mat2.b + mat1.d.z * mat2.c + mat1.d.w * mat2.d
    );
}

#else

Matrix4x4 operator*(const Matrix4x4& mat1, const Matrix4x4 &mat2) {
    return Matrix4x4( mat1 * mat2.a, mat1 * mat2.b, mat1 * mat2.c, mat1 * mat2.d );
}
#endif

//Matrix3x3 &operator*=(Matrix3x3& mat1, const Matrix3x3& mat2) { return mat1 = mat1 * mat2; }

Matrix4x4 &operator*=(Matrix4x4& mat, const Matrix4x4 &o) {
  return (mat = mat * o);
}

Matrix4x4 operator*(const Matrix4x4& mat, float f) {
  return { mat.a * f, mat.b * f, mat.c * f, mat.d * f };
}

Matrix4x4 operator+(const Matrix4x4& mat1, const Matrix4x4& mat2) {
  return { mat1.a + mat2.a, mat1.b + mat2.b, mat1.c + mat2.c, mat1.d + mat2.d };
}

#endif

void print(const Matrix3x3& m) {
  printf("    %f %f %f\n", m[0].x, m[1].x, m[2].x);
  printf("    %f %f %f\n", m[0].y, m[1].y, m[2].y);
  printf("    %f %f %f\n", m[0].z, m[1].z, m[2].z);
}

inline void print(const Matrix3x4& m) {
  printf("    %f %f %f\n", m[0].x, m[1].x, m[2].x);
  printf("    %f %f %f\n", m[0].y, m[1].y, m[2].y);
  printf("    %f %f %f\n", m[0].z, m[1].z, m[2].z);
  printf("    %f %f %f\n", m[0].w, m[1].w, m[2].w);
}

inline void print(const Matrix4x3& m) {
  printf("    %f %f %f %f\n", m[0].x, m[1].x, m[2].x, m[3].x);
  printf("    %f %f %f %f\n", m[0].y, m[1].y, m[2].y, m[3].y);
  printf("    %f %f %f %f\n", m[0].z, m[1].z, m[2].z, m[3].z);
}

inline void print(const Matrix4x4& m) {
  printf("    %f %f %f %f\n", m[0].x, m[1].x, m[2].x, m[3].x);
  printf("    %f %f %f %f\n", m[0].y, m[1].y, m[2].y, m[3].y);
  printf("    %f %f %f %f\n", m[0].z, m[1].z, m[2].z, m[3].z);
  printf("    %f %f %f %f\n", m[0].w, m[1].w, m[2].w, m[3].w);
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

Matrix3x4 transform(const Matrix3x4& m1, const Matrix3x4 &m2)
{
  Matrix4x4 tmp1 = Matrix4x4(
    m1[0],
    m1[1],
    m1[2],
    Vec4(0,0,0,1)
  );

  Matrix4x4 tmp2 = Matrix4x4(
    m2[0],
    m2[1],
    m2[2],
    Vec4(0,0,0,1)
  );

  return Matrix3x4(tmp2 * tmp1);
}

Matrix3x4 transform(const Matrix3x4& m1, const Matrix3x4& m2, const Matrix3x4& m3) {
  return transform(transform(m1,m2), m3);
}

Matrix3x3 invertRotation(Matrix3x3 mat) {
    mat[0] /= length2(mat[0]);
    mat[1] /= length2(mat[1]);
    mat[2] /= length2(mat[2]);
    return transpose(mat);
}

/*
Matrix4x3 inverse(const Matrix4x3& mat) {
    Matrix3x3 invrot = invertRotation(Matrix3x3(mat));
    Vec3 trans = -(invrot * mat[3]);
    return Matrix4x3(invrot[0], invrot[1], invrot[2], trans);
}

Matrix3x4 inverse(const Matrix3x4 &o) {
    return transpose(inverse(transpose(o)));
}
*/

Matrix4x4 mix(Matrix4x4 m1, Matrix4x4 m2, float alpha) {
  return m1 * (1-alpha) + m2 * alpha;
}

Matrix4x4 inverse(const Matrix4x4& mat) {

  Matrix4x4 result;
  float* m = (float*)&mat;
  float* res = (float*)&result;

    res[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    res[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    res[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    res[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    res[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    res[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    res[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    res[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    res[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    res[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    res[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    res[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    res[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    res[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    res[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    res[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    float det = m[0] * res[0] + m[1] * res[4] + m[2] * res[8] + m[3] * res[12];

    det = 1.0 / det;

    return result * det;
}


#if 0

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_operation.hpp>

 void print(const glm::fmat3& m) {
  printf("    %f %f %f\n", m[0].x, m[1].x, m[2].x);
  printf("    %f %f %f\n", m[0].y, m[1].y, m[2].y);
  printf("    %f %f %f\n", m[0].z, m[1].z, m[2].z);
}

inline void print(const glm::fmat3x4& m) {
  printf("    %f %f %f\n", m[0].x, m[1].x, m[2].x);
  printf("    %f %f %f\n", m[0].y, m[1].y, m[2].y);
  printf("    %f %f %f\n", m[0].z, m[1].z, m[2].z);
  printf("    %f %f %f\n", m[0].w, m[1].w, m[2].w);
}

inline void print(const glm::fmat4x3& m) {
  printf("    %f %f %f %f\n", m[0].x, m[1].x, m[2].x, m[3].x);
  printf("    %f %f %f %f\n", m[0].y, m[1].y, m[2].y, m[3].y);
  printf("    %f %f %f %f\n", m[0].z, m[1].z, m[2].z, m[3].z);
}

inline void print(const glm::fmat4& m) {
  printf("    %f %f %f %f\n", m[0].x, m[1].x, m[2].x, m[3].x);
  printf("    %f %f %f %f\n", m[0].y, m[1].y, m[2].y, m[3].y);
  printf("    %f %f %f %f\n", m[0].z, m[1].z, m[2].z, m[3].z);
  printf("    %f %f %f %f\n", m[0].w, m[1].w, m[2].w, m[3].w);
}

glm::fmat3x4 transform(const glm::fmat3x4& m1, const glm::fmat3x4 &m2)
{
  auto tmp1 = glm::fmat4x4(
    m1[0],
    m1[1],
    m1[2],
    glm::fvec4(0,0,0,1)
  );

  auto tmp2 = glm::fmat4x4(
    m2[0],
    m2[1],
    m2[2],
    glm::fvec4(0,0,0,1)
  );

  printf("tmp1:\n");
  print(tmp1);
  printf("tmp2:\n");
  print(tmp2);

  return glm::fmat3x4(tmp1 * tmp2);
}

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


  /*
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
  */
  auto arg1 = transpose(Matrix4x3(
    Vec3(-0.000001, 1.000000, 0.000000),
    Vec3(-1.000000, -0.000001, 0.000000),
    Vec3(0.000000, 0.000000, 1.000000),
    Vec3(0.000000, 0.029336, 3.624440)
  ));

  auto arg2 = transpose(Matrix4x3(
    Vec3(-0.000001, -1.000000, 0.000000),
    Vec3(1.000000, -0.000001, 0.000000),
    Vec3(0.000000, 0.000000, 1.000000),
    Vec3(-0.000000, -0.000000, -3.772637)
  ));

  auto arg1_ = transpose(glm::fmat4x3(
    glm::fvec3(-0.000001, 1.000000, 0.000000),
    glm::fvec3(-1.000000, -0.000001, 0.000000),
    glm::fvec3(0.000000, 0.000000, 1.000000),
    glm::fvec3(0.000000, 0.029336, 3.624440)
  ));

  auto arg2_ = transpose(glm::fmat4x3(
    glm::fvec3(-0.000001, -1.000000, 0.000000),
    glm::fvec3(1.000000, -0.000001, 0.000000),
    glm::fvec3(0.000000, 0.000000, 1.000000),
    glm::fvec3(-0.000000, -0.000000, -3.772637)
  ));


  printf("transform1:\n");
  print(transform(arg1, arg2));
  printf("transform2:\n"); 
  print(transform(arg1_, arg2_));

  exit(0);

  /*
  auto r1 = transpose(glm::fmat4x3(
    glm::fvec3(1.000000, -0.000000, 0.000000),
    glm::fvec3(0.000000, 1.000000, 0.000000),
    glm::fvec3(0.000000, 0.000000, 1.000000),
    glm::fvec3(0.000000, 0.029336, -0.148197)
  ));

  auto r2 = transpose(Matrix4x3(
    Vec3(1.000000, -0.000000,  0.000000),
    Vec3(0.000000,  1.000000,  0.000000),
    Vec3(0.000000,  0.000000,  1.000000),
    Vec3(0.029336, -0.000000, -0.148197)
  ));
  */

  auto m1 = Matrix3x4( Vec4(1,2,3,4), Vec4(5,6,7,8), Vec4(9,10,11,12) );
  auto m2 = glm::fmat3x4( glm::fvec4(1,2,3,4), glm::fvec4(5,6,7,8), glm::fvec4(9,10,11,12));

  auto t1 = transform(m1,m1);
  auto t2 = transform(m2,m2);


  printf("t1:\n");
  printf("  %f %f %f %f \n", t1.a.x, t1.a.y, t1.a.z, t1.a.w);
  printf("  %f %f %f %f \n", t1.b.x, t1.b.y, t1.b.z, t1.b.w);
  printf("  %f %f %f %f \n", t1.c.x, t1.c.y, t1.c.z, t1.c.w);
  printf("t2:\n");
  printf("  %f %f %f %f \n", t2[0].x, t2[0].y, t2[0].z, t2[0].w);
  printf("  %f %f %f %f \n", t2[1].x, t2[1].y, t2[1].z, t2[1].w);
  printf("  %f %f %f %f \n", t2[2].x, t2[2].y, t2[2].z, t2[2].w);


}
#else

inline void test() {}

#endif

