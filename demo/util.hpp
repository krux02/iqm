#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>


static inline double radians(double x) { return (x * M_PI) / 180.0; }
static inline double degrees(double x) { return (x * 180.0) / M_PI; }
static inline float radians(float x) { return (x * M_PI) / 180.0; }
static inline float degrees(float x) { return (x * 180.0) / M_PI; }

static inline bool islittleendian() { union { int i; uint8_t b[sizeof(int)]; } conv; conv.i = 1; return conv.b[0] != 0; }

inline uint16_t endianswap16(uint16_t n) { return (n<<8) | (n>>8); }
inline uint32_t endianswap32(uint32_t n) { return (n<<24) | (n>>24) | ((n>>8)&0xFF00) | ((n<<8)&0xFF0000); }
inline uint64_t endianswap64(uint64_t n) { return endianswap32(n >> 32) | (uint64_t(endianswap32(n)) << 32); }

inline uint16_t endianswap(uint16_t n) { return endianswap16(n); }
inline int16_t endianswap(short n) { return endianswap16(n); }
inline uint32_t endianswap(uint32_t n) { return endianswap32(n); }
inline int32_t endianswap(int32_t n) { return endianswap32(n); }
inline uint64_t endianswap(uint64_t n) { return endianswap64(n); }
inline int64_t endianswap(int64_t n) { return endianswap64(n); }
inline float endianswap(float n) { return n; }
inline double endianswap(double n) { return n; }

template<class T> inline void endianswap(T *buf, int len) { for(T *end = &buf[len]; buf < end; buf++) *buf = endianswap(*buf); }
template<class T> inline T lilswap(T n) { return islittleendian() ? n : endianswap(n); }
template<class T> inline void lilswap(T *buf, int len) { if(!islittleendian()) endianswap(buf, len); }
template<class T> inline T bigswap(T n) { return islittleendian() ? endianswap(n) : n; }
template<class T> inline void bigswap(T *buf, int len) { if(islittleendian()) endianswap(buf, len); }

/*
template<class T> T getval(FILE *f) { T n; return fread(&n, 1, sizeof(n), f) == sizeof(n) ? n : 0; }
template<class T> T getlil(FILE *f) { return lilswap(getval<T>(f)); }
template<class T> T getbig(FILE *f) { return bigswap(getval<T>(f)); }
*/
