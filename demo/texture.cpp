#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <algorithm>

#include <GL/gl.h>
#include <GL/glext.h>

#include "util.hpp"
#include <glm/glm.hpp>

#define FUNCNAME(name) name##1
#define DEFPIXEL uint32_t OP(r, 0);
#define PIXELOP OP(r, 0);
#define BPP 1
#include "scale.h"

#define FUNCNAME(name) name##2
#define DEFPIXEL uint32_t OP(r, 0), OP(g, 1);
#define PIXELOP OP(r, 0); OP(g, 1);
#define BPP 2
#include "scale.h"

#define FUNCNAME(name) name##3
#define DEFPIXEL uint32_t OP(r, 0), OP(g, 1), OP(b, 2);
#define PIXELOP OP(r, 0); OP(g, 1); OP(b, 2);
#define BPP 3
#include "scale.h"

#define FUNCNAME(name) name##4
#define DEFPIXEL uint32_t OP(r, 0), OP(g, 1), OP(b, 2), OP(a, 3);
#define PIXELOP OP(r, 0); OP(g, 1); OP(b, 2); OP(a, 3);
#define BPP 4
#include "scale.h"

static void scaletexture(uint8_t *src, uint32_t sw, uint32_t sh, uint32_t bpp, uint32_t pitch, uint8_t *dst, uint32_t dw, uint32_t dh)
{
    if(sw == dw*2 && sh == dh*2)
    {
        switch(bpp)
        {
            case 1: return halvetexture1(src, sw, sh, pitch, dst);
            case 2: return halvetexture2(src, sw, sh, pitch, dst);
            case 3: return halvetexture3(src, sw, sh, pitch, dst);
            case 4: return halvetexture4(src, sw, sh, pitch, dst);
        }
    }
    else if(sw < dw || sh < dh || sw&(sw-1) || sh&(sh-1) || dw&(dw-1) || dh&(dh-1))
    {
        switch(bpp)
        {
            case 1: return scaletexture1(src, sw, sh, pitch, dst, dw, dh);
            case 2: return scaletexture2(src, sw, sh, pitch, dst, dw, dh);
            case 3: return scaletexture3(src, sw, sh, pitch, dst, dw, dh);
            case 4: return scaletexture4(src, sw, sh, pitch, dst, dw, dh);
        }
    }
    else
    {
        switch(bpp)
        {
            case 1: return shifttexture1(src, sw, sh, pitch, dst, dw, dh);
            case 2: return shifttexture2(src, sw, sh, pitch, dst, dw, dh);
            case 3: return shifttexture3(src, sw, sh, pitch, dst, dw, dh);
            case 4: return shifttexture4(src, sw, sh, pitch, dst, dw, dh);
        }
    }
}

static inline void bgr2rgb(uint8_t *data, int32_t len, int32_t bpp)
{
    for(uint8_t *end = &data[len]; data < end; data += bpp)
        std::swap(data[0], data[2]);
}

struct TGAHeader
{
    uint8_t  identsize;
    uint8_t  cmaptype;
    uint8_t  imagetype;
    uint8_t  cmaporigin[2]; 
    uint8_t  cmapsize[2];
    uint8_t  cmapentrysize;
    uint8_t  xorigin[2];
    uint8_t  yorigin[2];
    uint8_t  width[2];
    uint8_t  height[2];
    uint8_t  pixelsize;
    uint8_t  descbyte;
};

static uint8_t *loadtga(const char *fname, int32_t &w, int32_t &h, int32_t &bpp)
{
    FILE *f = fopen(fname, "rb");
    if(!f) return nullptr;

    uint8_t *data = nullptr, *cmap = nullptr;
    TGAHeader hdr;
    if(fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) goto error;
    if(fseek(f, hdr.identsize, SEEK_CUR) < 0) goto error;
    if(hdr.pixelsize != 8 && hdr.pixelsize != 24 && hdr.pixelsize != 32) goto error;

    bpp = hdr.pixelsize/8;
    w = hdr.width[0] + (hdr.width[1]<<8);
    h = hdr.height[0] + (hdr.height[1]<<8);

    if(hdr.imagetype==1)
    {
        int32_t cmapsize = hdr.cmapsize[0] + (hdr.cmapsize[1]<<8);
        if(hdr.cmapentrysize!=8 || hdr.cmapentrysize!=24 || hdr.cmapentrysize!=32) goto error;
        bpp = hdr.cmapentrysize/8;
        cmap = new uint8_t[bpp*cmapsize];
        if((int32_t)fread(cmap, 1, bpp*cmapsize, f) != bpp*cmapsize) goto error;
        if(bpp>=3) bgr2rgb(cmap, bpp*cmapsize, bpp);
        data = new uint8_t[bpp*w*h];
        uint8_t *idxs = &data[(bpp-1)*w*h];
        if((int32_t)fread(idxs, 1, w*h, f) != w*h) goto error;
        uint8_t *src = idxs, *dst = &data[bpp*w*h];    
        for(int32_t i = 0; i < h; i++)
        {
            dst -= bpp*w;
            uint8_t *row = dst;
            for(int32_t j = 0; j < w; j++)
            {
                memcpy(row, &cmap[*src++ * bpp], bpp);
                row += bpp;
            }
        }
    }
    else if(hdr.imagetype==2)
    { 
        data = new uint8_t[bpp*w*h];
        uint8_t *dst = &data[bpp*w*h];
        for(int32_t i = 0; i < h; i++)
        {
            dst -= bpp*w;
            if((int32_t)fread(dst, 1, bpp*w, f) != bpp*w) goto error;
        }
        if(bpp>=3) bgr2rgb(data, bpp*w*h, bpp);
    }
    else if(hdr.imagetype==9)
    {
        int32_t cmapsize = hdr.cmapsize[0] + (hdr.cmapsize[1]<<8);
        if(hdr.cmapentrysize!=8 || hdr.cmapentrysize!=24 || hdr.cmapentrysize!=32) goto error;
        bpp = hdr.cmapentrysize/8;
        cmap = new uint8_t[bpp*cmapsize];
        if((int32_t)fread(cmap, 1, bpp*cmapsize, f) != bpp*cmapsize) goto error;
        if(bpp>=3) bgr2rgb(cmap, bpp*cmapsize, bpp);
        data = new uint8_t[bpp*w*h];
        uint8_t buf[128];
        for(uint8_t *end = &data[bpp*w*h], *dst = end - bpp*w; dst >= data;)
        {
            int32_t c = fgetc(f);
            if(c==EOF) goto error;
            if(c&0x80)
            {
                int32_t idx = fgetc(f);
                if(idx==EOF) goto error;
                const uint8_t *col = &cmap[idx*bpp];
                c -= 0x7F;
                c *= bpp;
                while(c > 0 && dst >= data)
                {
                    int32_t n = std::min(c, int32_t(end-dst));
                    for(uint8_t *run = dst+n; dst < run; dst += bpp) memcpy(dst, col, bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; }
                }
            }
            else
            {
                c += 1;
                while(c > 0 && dst >= data)
                {
                    int32_t n = std::min(c, int32_t(end-dst)/bpp);
                    if((int32_t)fread(buf, 1, n, f) != n) goto error;
                    for(uint8_t *src = buf; src < &buf[n]; dst += bpp) memcpy(dst, &cmap[*src++ * bpp], bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; }
                }
            }
        }
    }
    else if(hdr.imagetype==10)
    {
        data = new uint8_t[bpp*w*h];
        uint8_t buf[4];
        for(uint8_t *end = &data[bpp*w*h], *dst = end - bpp*w; dst >= data;)
        {
            int32_t c = fgetc(f);
            if(c==EOF) goto error;
            if(c&0x80)
            {
                if((int32_t)fread(buf, 1, bpp, f) != bpp) goto error;
                c -= 0x7F;
                if(bpp>=3) std::swap(buf[0], buf[2]);
                c *= bpp;
                while(c > 0)
                {
                    int32_t n = std::min(c, int32_t(end-dst));
                    for(uint8_t *run = dst+n; dst < run; dst += bpp) memcpy(dst, buf, bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; if(dst < data) break; }
                }
            }
            else
            {
                c += 1;
                c *= bpp;
                while(c > 0)
                {
                    int32_t n = std::min(c, int32_t(end-dst));
                    if((int32_t)fread(dst, 1, n, f) != n) goto error;
                    if(bpp>=3) bgr2rgb(dst, n, bpp);
                    dst += n;
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; if(dst < data) break; }
                }
            }
        }
    }
    else goto error;

    if(cmap) delete[] cmap;
    fclose(f);
    return data;
    
error:
    if(data) delete[] data;
    if(cmap) delete[] cmap;
    fclose(f);
    return nullptr;
}

static GLenum texformat(int32_t bpp)
{
    switch(bpp)
    {
        case 8: return GL_LUMINANCE;
        case 16: return GL_LUMINANCE_ALPHA;
        case 24: return GL_RGB;
        case 32: return GL_RGBA;
        default: return 0;
    }
}

int32_t formatsize(GLenum format)
{
    switch(format)
    {
        case GL_LUMINANCE:
        case GL_ALPHA: return 1;
        case GL_LUMINANCE_ALPHA: return 2;
        case GL_RGB: return 3;
        case GL_RGBA: return 4;
        default: return 4;
    }
}

void resizetexture(int32_t w, int32_t h, bool mipmap, GLenum target, int32_t &tw, int32_t &th)
{
    GLint sizelimit = 4096;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &sizelimit);
    w = std::min(w, sizelimit);
    h = std::min(h, sizelimit);
    if(mipmap || w&(w-1) || h&(h-1))
    {
        tw = th = 1;
        while(tw < w) tw *= 2;
        while(th < h) th *= 2;
        if(w < tw - tw/4) tw /= 2;
        if(h < th - th/4) th /= 2;
    }
    else
    {
        tw = w;
        th = h;
    }
}

void uploadtexture(GLenum target, GLenum internal, int32_t tw, int32_t th, GLenum format, GLenum type, void *pixels, int32_t pw, int32_t ph, bool mipmap)
{
    int32_t bpp = formatsize(format);
    uint8_t *buf = nullptr;
    if(pw!=tw || ph!=th)
    {
        buf = new uint8_t[tw*th*bpp];
        scaletexture((uint8_t *)pixels, pw, ph, bpp, pw*bpp, buf, tw, th);
    }
    for(int32_t level = 0;; level++)
    {
        uint8_t *src = buf ? buf : (uint8_t *)pixels;
        if(target==GL_TEXTURE_1D) glTexImage1D(target, level, internal, tw, 0, format, type, src);
        else glTexImage2D(target, level, internal, tw, th, 0, format, type, src);
        if(!mipmap || std::max(tw, th) <= 1) break;
        int32_t srcw = tw, srch = th;
        if(tw > 1) tw /= 2;
        if(th > 1) th /= 2;
        if(!buf) buf = new uint8_t[tw*th*bpp];
        scaletexture(src, srcw, srch, bpp, srcw*bpp, buf, tw, th);
    }
    if(buf) delete[] buf;
}

void createtexture(int32_t tnum, int32_t w, int32_t h, void *pixels, int32_t clamp, int32_t filter, GLenum component = GL_RGB, GLenum target = GL_TEXTURE_2D, int32_t pw = 0, int32_t ph = 0)
{
    glBindTexture(target, tnum);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(target, GL_TEXTURE_WRAP_S, clamp&1 ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    if(target!=GL_TEXTURE_1D) glTexParameteri(target, GL_TEXTURE_WRAP_T, clamp&2 ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, filter ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, filter > 1 ? GL_LINEAR_MIPMAP_LINEAR : (filter ? GL_LINEAR : GL_NEAREST));

    GLenum format = component, type = GL_UNSIGNED_BYTE;
    switch(component)
    {
        case GL_RGB5:
        case GL_RGB8:
        case GL_RGB16:
            format = GL_RGB;
            break;

        case GL_RGBA8:
        case GL_RGBA16:
            format = GL_RGBA;
            break;
    }

    if(!pw) pw = w;
    if(!ph) ph = h;
    int32_t tw = w, th = h;
    bool mipmap = filter > 1;
    if(pixels) resizetexture(w, h, mipmap, target, tw, th);
    uploadtexture(target, component, tw, th, format, type, pixels, pw, ph, mipmap && pixels);
}

GLuint loadtexture(const char *name, int32_t clamp)
{
    int32_t w, h, b;
    uint8_t *data = loadtga(name, w, h, b); 
    if(!data) { printf("%s: failed loading\n", name); return 0; }
    GLenum format = texformat(b*8);
    if(!format) { printf("%s: failed loading\n", name); delete[] data; return 0; }

    GLuint tex;
    glGenTextures(1, &tex);
    createtexture(tex, w, h, data, clamp, 2, format);

    delete[] data;
    return tex;
}
 
