#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut.h>

#include "util.hpp"
#include "geom.hpp"
#include "iqm.h"

#include <algorithm>
#include <vector>
#include <memory>

#define FOR(TYPE, IDENT, BEGIN, END) for(TYPE IDENT = (TYPE)(BEGIN), IDENT##_end = (TYPE)(END); IDENT != IDENT##_end; ++IDENT)

#define EXTS(EXT) \
    EXT(PFNGLUSEPROGRAMPROC, glUseProgram, true) \
    EXT(PFNGLCREATEPROGRAMPROC, glCreateProgram, true) \
    EXT(PFNGLCREATESHADERPROC, glCreateShader, true) \
    EXT(PFNGLDELETEPROGRAMPROC, glDeleteProgram, true) \
    EXT(PFNGLDELETESHADERPROC, glDeleteShader, true) \
    EXT(PFNGLATTACHSHADERPROC, glAttachShader, true) \
    EXT(PFNGLBINDATTRIBLOCATIONPROC, glBindAttribLocation, true) \
    EXT(PFNGLCOMPILESHADERPROC, glCompileShader, true) \
    EXT(PFNGLLINKPROGRAMPROC, glLinkProgram, true) \
    EXT(PFNGLSHADERSOURCEPROC, glShaderSource, true) \
    EXT(PFNGLGETPROGRAMIVPROC, glGetProgramiv, true) \
    EXT(PFNGLGETSHADERIVPROC, glGetShaderiv, true) \
    EXT(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog, true) \
    EXT(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog, true) \
    EXT(PFNGLDISABLEVERTEXATTRIBARRAYPROC, glDisableVertexAttribArray, true) \
    EXT(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray, true) \
    EXT(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer, true) \
    EXT(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv, true) \
    EXT(PFNGLUNIFORM1IPROC, glUniform1i, true) \
    EXT(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation, true) \
    EXT(PFNGLBINDBUFFERPROC, glBindBuffer, true) \
    EXT(PFNGLDELETEBUFFERSPROC, glDeleteBuffers, true) \
    EXT(PFNGLGENBUFFERSPROC, glGenBuffers, true) \
    EXT(PFNGLBUFFERDATAPROC, glBufferData, true) \
    EXT(PFNGLBUFFERSUBDATAPROC, glBufferSubData, true) \
    EXT(PFNGLGETUNIFORMINDICESPROC, glGetUniformIndices, hasUBO) \
    EXT(PFNGLGETACTIVEUNIFORMSIVPROC, glGetActiveUniformsiv, hasUBO) \
    EXT(PFNGLGETUNIFORMBLOCKINDEXPROC, glGetUniformBlockIndex, hasUBO) \
    EXT(PFNGLGETACTIVEUNIFORMBLOCKIVPROC, glGetActiveUniformBlockiv, hasUBO) \
    EXT(PFNGLUNIFORMBLOCKBINDINGPROC, glUniformBlockBinding, hasUBO) \
    EXT(PFNGLBINDBUFFERBASEPROC, glBindBufferBase, hasUBO) \
    EXT(PFNGLBINDBUFFERRANGEPROC, glBindBufferRange, hasUBO)

#define DEFEXT(type, name, required) type name##_ = nullptr;
EXTS(DEFEXT)
 
void fatal(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(EXIT_FAILURE);
}

bool hasUBO = false;

void loadexts()
{
    const char *version = (const char *)glGetString(GL_VERSION);
    if(strcmp(version, "2.1") < 0) fatal("OpenGL version 2.1 required, found version: %s", version);

    hasUBO = glutExtensionSupported("GL_ARB_uniform_buffer_object") != 0;

    #define LOADEXT(type, name, required) if(required) { name##_ = (type)glutGetProcAddress(#name); if(!name##_) fatal("failed getting proc address: %s", #name); }
    EXTS(LOADEXT)
}

extern GLuint loadtexture(const char *name, int clamp);

// Note that while this demo stores pointer directly into mesh data in a buffer 
// of the entire IQM file's data, it is recommended that you copy the data and
// convert it into a more suitable internal representation for whichever 3D
// engine you use.

// uint8_t *meshdata = nullptr, *animdata = nullptr;
// bool meshdata = false, animdata = false;

std::vector<iqmmesh> meshes;
std::vector<GLuint> textures;
GLuint notexture = 0;

std::vector<iqmjoint> joints;
std::vector<iqmpose> poses;
std::vector<iqmanim> anims;

std::vector<Matrix4x4> baseframe, inversebaseframe, outframe, frames;

struct vertex
{
    Vec3 position;
    Vec3 normal;
    Vec4 tangent;
    Vec2 texcoord;
    Vec4u8 blendindex;
    Vec4u8 blendweight;
};

GLuint ebo = 0, vbo = 0, ubo = 0;
GLint ubosize = 0, bonematsoffset = 0;

void cleanupiqm()
{
    if(not textures.empty())
    {
        glDeleteTextures(textures.size(), textures.data());
        textures.clear();
    }
    if(notexture) glDeleteTextures(1, &notexture);
    if(ebo) glDeleteBuffers_(1, &ebo);
    if(vbo) glDeleteBuffers_(1, &vbo);
    if(ubo) glDeleteBuffers_(1, &ubo);
}

bool loadiqmmeshes(const char *filename, const iqmheader &hdr, uint8_t *buf)
{
    lilswap((uint32_t *)&buf[hdr.ofs_vertexarrays], hdr.num_vertexarrays*sizeof(iqmvertexarray)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_triangles], hdr.num_triangles*sizeof(iqmtriangle)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_meshes], hdr.num_meshes*sizeof(iqmmesh)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_joints], hdr.num_joints*sizeof(iqmjoint)/sizeof(uint32_t));

    outframe.resize(hdr.num_joints);
    textures.resize(hdr.num_meshes);
    fill(begin(textures), end(textures), 0);

    Vec3* inposition = nullptr;
    Vec3* innormal = nullptr;
    Vec4* intangent = nullptr;
    Vec2* intexcoord = nullptr;
    Vec4u8* inblendindex = nullptr;
    Vec4u8* inblendweight = nullptr;
    const char *str = hdr.ofs_text ? (char *)&buf[hdr.ofs_text] : "";
    iqmvertexarray *vas = (iqmvertexarray *)&buf[hdr.ofs_vertexarrays];
    for(int i = 0; i < (int)hdr.num_vertexarrays; i++)
    {
        iqmvertexarray &va = vas[i];
        switch(va.type)
        {
        case IQM_POSITION:
          if(va.format != IQM_FLOAT || va.size != 3) return false;
          inposition = (Vec3*)&buf[va.offset]; 
          break;
        case IQM_NORMAL:
          if(va.format != IQM_FLOAT || va.size != 3) return false;
          innormal = (Vec3*)&buf[va.offset];
          break;
        case IQM_TANGENT:
          if(va.format != IQM_FLOAT || va.size != 4) return false;
          intangent = (Vec4*)&buf[va.offset];
          break;
        case IQM_TEXCOORD:
          if(va.format != IQM_FLOAT || va.size != 2) return false;
          intexcoord = (Vec2*)&buf[va.offset];
          break;
        case IQM_BLENDINDEXES:
          if(va.format != IQM_UBYTE || va.size != 4) return false;
          inblendindex = (Vec4u8*)&buf[va.offset];
          break;
        case IQM_BLENDWEIGHTS:
          if(va.format != IQM_UBYTE || va.size != 4) return false;
          inblendweight = (Vec4u8*)&buf[va.offset];
          break;
        }
    }

    auto meshes_ptr = (iqmmesh*)&buf[hdr.ofs_meshes];
    meshes.clear();
    meshes.reserve(hdr.num_meshes);
    meshes.insert( begin(meshes), meshes_ptr, meshes_ptr + hdr.num_meshes);
    auto joints_ptr = (iqmjoint*)&buf[hdr.ofs_joints];
    joints.clear();
    joints.reserve(hdr.num_joints);
    joints.insert( begin(joints), joints_ptr, joints_ptr + hdr.num_joints);

    baseframe.resize(hdr.num_joints);
    inversebaseframe.resize(hdr.num_joints);
    for(int i = 0; i < (int)hdr.num_joints; i++)
    {
        iqmjoint &j = joints[i];
        auto translate = *(Vec3*)(j.translate);
        auto scale = *(Vec3*)(j.scale);
        auto rotate = normalize(*(Quat*)(j.rotate));
        auto scalerot_mat = Matrix3x3(rotate) * diagonal3x3(scale);
        baseframe[i] = Matrix4x4( Vec4(scalerot_mat[0],0), Vec4(scalerot_mat[1],0), Vec4(scalerot_mat[2],0), Vec4(translate,1));
        inversebaseframe[i] = inverse(baseframe[i]);
        if(j.parent >= 0)
        {
            baseframe[i] =        baseframe[j.parent] * baseframe[i];
            inversebaseframe[i] = inversebaseframe[i] * inversebaseframe[j.parent];
        }
    }

    for(int i = 0; i < (int)hdr.num_meshes; i++)
    {
        iqmmesh &m = meshes[i];
        printf("%s: loaded mesh: %s\n", filename, &str[m.name]);
        textures[i] = loadtexture(&str[m.material], 0);
        if(textures[i]) printf("%s: loaded material: %s\n", filename, &str[m.material]);
    }

    iqmtriangle *tris = (iqmtriangle *)&buf[hdr.ofs_triangles];

    if(!ebo) glGenBuffers_(1, &ebo);
    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData_(GL_ELEMENT_ARRAY_BUFFER, hdr.num_triangles*sizeof(iqmtriangle), tris, GL_STATIC_DRAW);
    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, 0);


    std::vector<vertex> verts;
    verts.resize(hdr.num_vertexes);
    memset(verts.data(), 0, hdr.num_vertexes*sizeof(vertex));

    FOR(int, i, 0, hdr.num_vertexes){
        vertex &v = verts[i];
        if(inposition) v.position = inposition[i];
        if(innormal) v.normal = innormal[i];
        if(intangent) v.tangent = intangent[i];
        if(intexcoord) v.texcoord = intexcoord[i];
        if(inblendindex) v.blendindex = inblendindex[i];
        if(inblendweight) v.blendweight = inblendweight[i];
    }

    if(!vbo) glGenBuffers_(1, &vbo);
    glBindBuffer_(GL_ARRAY_BUFFER, vbo);
    glBufferData_(GL_ARRAY_BUFFER, hdr.num_vertexes*sizeof(vertex), verts.data(), GL_STATIC_DRAW);
    glBindBuffer_(GL_ARRAY_BUFFER, 0);

    return true;
}

bool loadiqmanims(const char *filename, const iqmheader &hdr, uint8_t *buf)
{
    if(hdr.num_poses != hdr.num_joints) return false;

    lilswap((uint32_t *)&buf[hdr.ofs_poses], hdr.num_poses*sizeof(iqmpose)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_anims], hdr.num_anims*sizeof(iqmanim)/sizeof(uint32_t));
    lilswap((uint16_t *)&buf[hdr.ofs_frames], hdr.num_frames*hdr.num_framechannels);

    //numanims = hdr.num_anims;
    //numframes = hdr.num_frames;

    auto anims_ptr = (iqmanim *)&buf[hdr.ofs_anims];
    anims.clear();
    anims.reserve(hdr.num_anims);
    anims.insert(begin(anims), anims_ptr, anims_ptr + hdr.num_anims);
    auto poses_ptr = (iqmpose *)&buf[hdr.ofs_poses];
    poses.clear();
    poses.reserve(hdr.num_poses);
    poses.insert(begin(poses), poses_ptr, poses_ptr + hdr.num_poses);
    const char *str = hdr.ofs_text ? (char *)&buf[hdr.ofs_text] : "";
    frames.resize(hdr.num_frames * hdr.num_poses);
    uint16_t *framedata = (uint16_t *)&buf[hdr.ofs_frames];

    for(int i = 0; i < (int)hdr.num_frames; i++)
    {
        for(int j = 0; j < (int)hdr.num_poses; j++)
        {
            iqmpose &p = poses[j];
            Quat rotate;
            auto translate = Vec3(0,0,0);
            auto scale = Vec3(0,0,0);
            translate.x = p.channeloffset[0]; if(p.mask&0x01) translate.x += *framedata++ * p.channelscale[0];
            translate.y = p.channeloffset[1]; if(p.mask&0x02) translate.y += *framedata++ * p.channelscale[1];
            translate.z = p.channeloffset[2]; if(p.mask&0x04) translate.z += *framedata++ * p.channelscale[2];
            rotate.x = p.channeloffset[3]; if(p.mask&0x08) rotate.x += *framedata++ * p.channelscale[3];
            rotate.y = p.channeloffset[4]; if(p.mask&0x10) rotate.y += *framedata++ * p.channelscale[4];
            rotate.z = p.channeloffset[5]; if(p.mask&0x20) rotate.z += *framedata++ * p.channelscale[5];
            rotate.w = p.channeloffset[6]; if(p.mask&0x40) rotate.w += *framedata++ * p.channelscale[6];
            scale.x = p.channeloffset[7]; if(p.mask&0x80) scale.x += *framedata++ * p.channelscale[7];
            scale.y = p.channeloffset[8]; if(p.mask&0x100) scale.y += *framedata++ * p.channelscale[8];
            scale.z = p.channeloffset[9]; if(p.mask&0x200) scale.z += *framedata++ * p.channelscale[9];

            // Concatenate each pose with the inverse base pose to avoid doing this at animation time.
            // If the joint has a parent, then it needs to be pre-concatenated with its parent's base pose.
            // Thus it all negates at animation time like so:
            //   (parentPose * parentInverseBasePose) * (parentBasePose * childPose * childInverseBasePose) =>
            //   parentPose * (parentInverseBasePose * parentBasePose) * childPose * childInverseBasePose =>
            //   parentPose * childPose * childInverseBasePose

            auto scalerot_mat = Matrix3x3(normalize(rotate)) * diagonal3x3(scale);
            Matrix4x4 m = Matrix4x4( Vec4(scalerot_mat[0], 0), Vec4(scalerot_mat[1], 0), Vec4(scalerot_mat[2], 0), Vec4(translate,1));
            if(p.parent >= 0) {
              frames[i*hdr.num_poses + j] = baseframe[p.parent] * m * inversebaseframe[j];
            } else {
              frames[i*hdr.num_poses + j] = m * inversebaseframe[j];
            }
        }
    }

    for(int i = 0; i < (int)hdr.num_anims; i++)
    {
        iqmanim &a = anims[i];
        printf("%s: loaded anim: %s\n", filename, &str[a.name]);
    }
    
    return true;
}

bool loadiqm(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if(!f) return false;

    std::vector<uint8_t> buf;
    iqmheader hdr;
    if(fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) || memcmp(hdr.magic, IQM_MAGIC, sizeof(hdr.magic)))
        goto error;
    lilswap(&hdr.version, (sizeof(hdr) - sizeof(hdr.magic))/sizeof(uint32_t));
    if(hdr.version != IQM_VERSION)
        goto error;
    if(hdr.filesize > (16<<20)) 
        goto error; // sanity check... don't load files bigger than 16 MB

    buf.resize(hdr.filesize);

    if(fread(buf.data() + sizeof(hdr), 1, hdr.filesize - sizeof(hdr), f) != hdr.filesize - sizeof(hdr))
        goto error;

    if(hdr.num_meshes > 0 && !loadiqmmeshes(filename, hdr, buf.data())) goto error;
    if(hdr.num_anims > 0 && !loadiqmanims(filename, hdr, buf.data())) goto error;
 
    fclose(f);
    return true;

error:
    printf("%s: error while loading\n", filename);
    fclose(f);
    return false;
}

// Note that this animates all attributes (position, normal, tangent, bitangent)
// for expository purposes, even though this demo does not use all of them for rendering.
void animateiqm(float curframe)
{
    if(frames.empty()) return;

    int frame1 = (int)floor(curframe),
        frame2 = frame1 + 1;
    float frameoffset = curframe - frame1;
    frame1 %= frames.size() / joints.size();
    frame2 %= frames.size() / joints.size();
    Matrix4x4 *mat1 = &frames[frame1 * joints.size()],
              *mat2 = &frames[frame2 * joints.size()];
    // Interpolate matrixes between the two closest frames and concatenate with parent matrix if necessary.
    // Concatenate the result with the inverse of the base pose.
    // You would normally do animation blending and inter-frame blending here in a 3D engine.
    for(int i = 0; i < int(joints.size()); i++)
    {
        Matrix4x4 mat = mix(mat1[i], mat2[i], frameoffset);
        outframe[i] = joints[i].parent ? outframe[joints[i].parent] * mat : mat;
    }
}

struct binding
{
    const char *name;
    GLint index;
};

struct shader
{
    const char *name, *vsstr, *psstr;
    const binding *attribs, *texs;
    GLuint vs, ps, program, vsobj, psobj;

    shader(const char *name, const char *vsstr = nullptr, const char *psstr = nullptr, const binding *attribs = nullptr, const binding *texs = nullptr) : name(name), vsstr(vsstr), psstr(psstr), attribs(attribs), texs(texs), vs(0), ps(0), program(0), vsobj(0), psobj(0) {}

    static void showinfo(GLuint obj, const char *tname, const char *name)
    {
        GLint length = 0;
        if(!strcmp(tname, "PROG")) glGetProgramiv_(obj, GL_INFO_LOG_LENGTH, &length);
        else glGetShaderiv_(obj, GL_INFO_LOG_LENGTH, &length);
        if(length > 1)
        {
            GLchar *log = new GLchar[length];
            if(!strcmp(tname, "PROG")) glGetProgramInfoLog_(obj, length, &length, log);
            else glGetShaderInfoLog_(obj, length, &length, log);
            printf("GLSL ERROR (%s:%s)\n", tname, name);
            puts(log);
            delete[] log;
        }
    }

    static void compile(GLenum type, GLuint &obj, const char *def, const char *tname, const char *name, bool msg = true)
    {
        const GLchar *source = (const GLchar*)(def + strspn(def, " \t\r\n"));
        obj = glCreateShader_(type);
        glShaderSource_(obj, 1, &source, nullptr);
        glCompileShader_(obj);
        GLint success;
        glGetShaderiv_(obj, GL_COMPILE_STATUS, &success);
        if(!success)
        {
            if(msg) showinfo(obj, tname, name);
            glDeleteShader_(obj);
            obj = 0;
            fatal("error compiling shader");
        }
    }

    void link(const binding *attribs = nullptr, bool msg = true)
    {
        program = vsobj && psobj ? glCreateProgram_() : 0;
        GLint success = 0;
        if(program)
        {
            glAttachShader_(program, vsobj);
            glAttachShader_(program, psobj);

            if(attribs) for(const binding *a = attribs; a->name; a++)
                glBindAttribLocation_(program, a->index, a->name);

            glLinkProgram_(program);
            glGetProgramiv_(program, GL_LINK_STATUS, &success);
        }
        if(!success)
        {
            if(program)
            {
                if(msg) showinfo(program, "PROG", name);
                glDeleteProgram_(program);
                program = 0;
            }
            fatal("error linking shader");
        }
    }

    void compile(const char *vsdef, const char *psdef, const binding *attribs = nullptr)
    {
        compile(GL_VERTEX_SHADER,   vsobj, vsdef, "VS", name);
        compile(GL_FRAGMENT_SHADER, psobj, psdef, "PS", name);
        link(attribs, true);
    }

    void compile()
    {
        if(vsstr && psstr) compile(vsstr, psstr, attribs);
    }

    void set()
    {
        glUseProgram_(program);
        bindtexs();
    }

    GLint getparam(const char *pname)
    {
        return glGetUniformLocation_(program, pname);
    }

    void bindtex(const char *tname, GLint index)
    {
        GLint loc = getparam(tname);
        if(loc != -1) glUniform1i_(loc, index);
    }

    void bindtexs()
    {
        if(texs) for(const binding *t = texs; t->name; t++)
            bindtex(t->name, t->index);
    }
};

binding gpuskinattribs[] = { { "vtangent", 1 }, { "vweights", 6 }, { "vbones", 7 }, { nullptr, -1 } };
binding gpuskintexs[] = { { "tex", 0 }, { nullptr, -1 } };
shader gpuskin("gpu skin",
R"vertex(
#version 120
#ifdef GL_ARB_uniform_buffer_object
  #extension GL_ARB_uniform_buffer_object : enable
  layout(std140) uniform animdata
  {
     uniform mat4x4 bonemats[80];
  };
#else
  uniform mat4x4 bonemats[80];
#endif
attribute vec4 vweights;
attribute vec4 vbones;
attribute vec4 vtangent;
void main(void)
{
   mat4x4 m = bonemats[int(vbones.x)] * vweights.x;
   m += bonemats[int(vbones.y)] * vweights.y;
   m += bonemats[int(vbones.z)] * vweights.z;
   m += bonemats[int(vbones.w)] * vweights.w;
   vec4 mpos = m * gl_Vertex;
   gl_Position = gl_ModelViewProjectionMatrix * mpos;
   gl_TexCoord[0] = gl_MultiTexCoord0;
   mat3 madjtrans = mat3(cross(m[1].xyz, m[2].xyz), cross(m[2].xyz, m[0].xyz), cross(m[0].xyz, m[1].xyz));
   vec3 mnormal = gl_Normal * madjtrans;
   vec3 mtangent = vtangent.xyz * madjtrans; // tangent not used, just here as an example
   vec3 mbitangent = cross(mnormal, mtangent) * vtangent.w; // bitangent not used, just here as an example
   gl_FrontColor = gl_Color * (clamp(dot(normalize(gl_NormalMatrix * mnormal), gl_LightSource[0].position.xyz), 0.0, 1.0) * gl_LightSource[0].diffuse + gl_LightSource[0].ambient);
}
)vertex",

R"fragment(
uniform sampler2D tex;
void main(void)
{
   gl_FragColor = gl_Color * texture2D(tex, gl_TexCoord[0].xy);
}
)fragment",

gpuskinattribs, gpuskintexs);

binding noskinattribs[] = { { "vtangent", 1 }, { nullptr, -1 } };
binding noskintexs[] = { { "tex", 0 }, { nullptr, -1 } };
shader noskin("no skin",

R"vertex(
attribute vec4 vtangent;
void main(void)
{
   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
   gl_TexCoord[0] = gl_MultiTexCoord0;
   vec3 vbitangent = cross(gl_Normal, vtangent.xyz) * vtangent.w; // bitangent not used, just here as an example
   gl_FrontColor = gl_Color * (clamp(dot(normalize(gl_NormalMatrix * gl_Normal), gl_LightSource[0].position.xyz), 0.0, 1.0) * gl_LightSource[0].diffuse + gl_LightSource[0].ambient);
}
)vertex",

R"fragment(
uniform sampler2D tex;
void main(void)
{
   gl_FragColor = gl_Color * texture2D(tex, gl_TexCoord[0].xy);
}
)fragment",

noskinattribs, noskintexs);

float scale = 1, rotate = 0;

void renderiqm()
{
    static const GLfloat zero[4] = { 0, 0, 0, 0 }, 
                         one[4] = { 1, 1, 1, 1 },
                         ambientcol[4] = { 0.5f, 0.5f, 0.5f, 1 }, 
                         diffusecol[4] = { 0.5f, 0.5f, 0.5f, 1 },
                         lightdir[4] = { cosf(radians(-60.0f)), 0, sinf(radians(-60.0f)), 0 };

    glPushMatrix();
    glRotatef(rotate, 0, 0, -1);
    glScalef(scale, scale, scale);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, zero);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zero);
    glMaterialfv(GL_FRONT, GL_EMISSION, zero);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, one);
    glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientcol);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffusecol);
    glLightfv(GL_LIGHT0, GL_POSITION, lightdir);

    glColor3f(1, 1, 1);

    if(not frames.empty())
    {
        gpuskin.set();
    
        if(hasUBO)
        {
            glBindBuffer_(GL_UNIFORM_BUFFER, ubo);
            glBufferData_(GL_UNIFORM_BUFFER, ubosize, nullptr, GL_STREAM_DRAW);
            glBufferSubData_(GL_UNIFORM_BUFFER, bonematsoffset, joints.size()*sizeof(Matrix4x4), (float*)(outframe.data()));
            glBindBuffer_(GL_UNIFORM_BUFFER, 0);

            glBindBufferBase_(GL_UNIFORM_BUFFER, 0, ubo);
        }
        else 
        {
            glUniformMatrix4fv_(gpuskin.getparam("bonemats"), joints.size(), GL_FALSE, (float*)(outframe.data()));
        }
    }
    else 
    {
        noskin.set();
    }

    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBindBuffer_(GL_ARRAY_BUFFER, vbo);

    vertex *vert = nullptr;
    glVertexPointer(3, GL_FLOAT, sizeof(vertex), &vert->position);
    glNormalPointer(GL_FLOAT, sizeof(vertex), &vert->normal);
    glTexCoordPointer(2, GL_FLOAT, sizeof(vertex), &vert->texcoord);
    glVertexAttribPointer_(1, 4, GL_FLOAT, GL_FALSE, sizeof(vertex), &vert->tangent);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableVertexAttribArray_(1);
    if(not frames.empty())
    {
        glVertexAttribPointer_(6, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(vertex), &vert->blendweight);
        glVertexAttribPointer_(7, 4, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(vertex), &vert->blendindex);
        glEnableVertexAttribArray_(6);
        glEnableVertexAttribArray_(7);
    }
   
    iqmtriangle *tris = nullptr;
    for(int i = 0; i < int(meshes.size()); i++)
    {
        iqmmesh &m = meshes[i];
        glBindTexture(GL_TEXTURE_2D, textures[i] ? textures[i] : notexture);
        glDrawElements(GL_TRIANGLES, 3*m.num_triangles, GL_UNSIGNED_INT, &tris[m.first_triangle]);
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableVertexAttribArray_(1);
    if(not frames.empty())
    {
        glDisableVertexAttribArray_(6);
        glDisableVertexAttribArray_(7);
    }

    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer_(GL_ARRAY_BUFFER, 0);

    glPopMatrix();
}

void initgl()
{
    glClearColor(0, 0, 0, 0);
    glClearDepth(1);
    glDisable(GL_FOG);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    gpuskin.compile();
    
    if(hasUBO)
    {
        GLuint blockidx = glGetUniformBlockIndex_(gpuskin.program, "animdata"), bonematsidx;
        const GLchar *bonematsname = "bonemats";
        glGetUniformIndices_(gpuskin.program, 1, &bonematsname, &bonematsidx);
        glGetActiveUniformBlockiv_(gpuskin.program, blockidx, GL_UNIFORM_BLOCK_DATA_SIZE, &ubosize);
        glGetActiveUniformsiv_(gpuskin.program, 1, &bonematsidx, GL_UNIFORM_OFFSET, &bonematsoffset);
        glUniformBlockBinding_(gpuskin.program, blockidx, 0);
        if(!ubo) glGenBuffers_(1, &ubo);
    }

    noskin.compile();

    notexture = loadtexture("notexture.tga", 0);
}

int scrw = 0, scrh = 0;

void reshapefunc(int w, int h)
{
    scrw = w;
    scrh = h;
    glViewport(0, 0, w, h);
}

float camyaw = -90, campitch = 0, camroll = 0;
Vec3 campos(20, 0, 5);

void setupcamera()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLdouble aspect = double(scrw)/scrh,
             fov = radians(90.0f),
             fovy = 2*atan2(tan(fov/2), aspect),
             nearplane = 1e-2f, farplane = 1000,
             ydist = nearplane * tan(fovy/2), xdist = ydist * aspect;
    glFrustum(-xdist, xdist, -ydist, ydist, nearplane, farplane);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glRotatef(camroll, 0, 0, 1);
    glRotatef(campitch, -1, 0, 0);
    glRotatef(camyaw, 0, 1, 0);
    glRotatef(-90, 1, 0, 0);
    glScalef(1, -1, 1);
    glTranslatef(-campos.x, -campos.y, -campos.z);
}
 
float animate = 0;

void timerfunc(int val)
{
    animate += 10*val/1000.0f;
    glutPostRedisplay();
    glutTimerFunc(35, timerfunc, 35);
}

void displayfunc()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setupcamera();

    animateiqm(animate);
    renderiqm();

    glutSwapBuffers();
}

void keyboardfunc(uint8_t c, int x, int y)
{
    switch(c)   
    {
    case 27:
        exit(EXIT_SUCCESS);
        break;
    }
}

int main(int argc, char **argv)
{
    glutInitWindowSize(640, 480);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
    glutCreateWindow("IQM GPU Skinning Demo");

    loadexts();

    atexit(cleanupiqm);
    for(int i = 1; i < argc; i++)
    {
        if(argv[i][0] == '-') switch(argv[i][1])
        {
        case 's':
            if(i + 1 < argc) scale = clamp(atof(argv[++i]), 1e-8, 1e8);
            break;
        case 'r':
            if(i + 1 < argc) rotate = atof(argv[++i]);
            break;
        }
        else if(!loadiqm(argv[i])) return EXIT_FAILURE;
    }
    if(not loadiqm("mrfixit.iqm")) return EXIT_FAILURE;

    initgl();
   
    glutTimerFunc(35, timerfunc, 35);
    glutReshapeFunc(reshapefunc);
    glutDisplayFunc(displayfunc);
    glutKeyboardFunc(keyboardfunc);
    glutMainLoop();
     
    return EXIT_SUCCESS;
}

