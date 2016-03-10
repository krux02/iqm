#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

template<class T> static inline T radians(T x) { return (x*M_PI)/180; }
template<class T> static inline T degrees(T x) { return (x*180)/M_PI; }

static inline bool islittleendian() { union { int i; uint8_t b[sizeof(int)]; } conv; conv.i = 1; return conv.b[0] != 0; }
inline uint16_t endianswap16(uint16_t n) { return (n<<8) | (n>>8); }
inline uint32_t endianswap32(uint32_t n) { return (n<<24) | (n>>24) | ((n>>8)&0xFF00) | ((n<<8)&0xFF0000); }
template<class T> inline T endianswap(T n) { union { T t; uint32_t i; } conv; conv.t = n; conv.i = endianswap32(conv.i); return conv.t; }
template<> inline uint16_t endianswap<uint16_t>(uint16_t n) { return endianswap16(n); }
template<> inline int16_t endianswap<short>(short n) { return endianswap16(n); }
template<> inline uint32_t endianswap<uint32_t>(uint32_t n) { return endianswap32(n); }
template<> inline int32_t endianswap<int>(int32_t n) { return endianswap32(n); }
template<class T> inline void endianswap(T *buf, int len) { for(T *end = &buf[len]; buf < end; buf++) *buf = endianswap(*buf); }
template<class T> inline T lilswap(T n) { return islittleendian() ? n : endianswap(n); }
template<class T> inline void lilswap(T *buf, int len) { if(!islittleendian()) endianswap(buf, len); }
template<class T> inline T bigswap(T n) { return islittleendian() ? endianswap(n) : n; }
template<class T> inline void bigswap(T *buf, int len) { if(islittleendian()) endianswap(buf, len); }

template<class T> T getval(FILE *f) { T n; return fread(&n, 1, sizeof(n), f) == sizeof(n) ? n : 0; }
template<class T> T getlil(FILE *f) { return lilswap(getval<T>(f)); }
template<class T> T getbig(FILE *f) { return bigswap(getval<T>(f)); }

