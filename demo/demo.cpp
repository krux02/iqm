#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>

#include "util.hpp"
#include "geom.hpp"
#include "iqm.h"

extern GLuint loadtexture(const char *name, int32_t clamp);

// Note that while this demo stores pointer directly into mesh data in a buffer 
// of the entire IQM file's data, it is recommended that you copy the data and
// convert it into a more suitable internal representation for whichever 3D
// engine you use.
uint8_t *meshdata = nullptr, *animdata = nullptr;
float *inposition = nullptr, *innormal = nullptr, *intangent = nullptr, *intexcoord = nullptr;
uint8_t *inblendindex = nullptr, *inblendweight = nullptr, *incolor = nullptr;
float *outposition = nullptr, *outnormal = nullptr, *outtangent = nullptr, *outbitangent = nullptr;
int32_t nummeshes = 0, numtris = 0, numverts = 0, numjoints = 0, numframes = 0, numanims = 0;
iqmtriangle *tris = nullptr, *adjacency = nullptr;
iqmmesh *meshes = nullptr;
GLuint *textures = nullptr;
iqmjoint *joints = nullptr;
iqmpose *poses = nullptr;
iqmanim *anims = nullptr;
iqmbounds *bounds = nullptr;
Matrix4x4 *baseframe = nullptr, *inversebaseframe = nullptr, *outframe = nullptr, *frames = nullptr;

void cleanupiqm() {
    if(textures) {
        glDeleteTextures(nummeshes, textures);
        delete[] textures;
    }
    delete[] outposition;
    delete[] outnormal;
    delete[] outtangent;
    delete[] outbitangent;
    delete[] baseframe;
    delete[] inversebaseframe;
    delete[] outframe;
    delete[] frames;
}

bool loadiqmmeshes(const char *filename, const iqmheader &hdr, uint8_t *buf) {
    if(meshdata) return false;

    lilswap((uint32_t *)&buf[hdr.ofs_vertexarrays], hdr.num_vertexarrays*sizeof(iqmvertexarray)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_triangles], hdr.num_triangles*sizeof(iqmtriangle)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_meshes], hdr.num_meshes*sizeof(iqmmesh)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_joints], hdr.num_joints*sizeof(iqmjoint)/sizeof(uint32_t));
    if(hdr.ofs_adjacency) lilswap((uint32_t *)&buf[hdr.ofs_adjacency], hdr.num_triangles*sizeof(iqmtriangle)/sizeof(uint32_t));

    meshdata = buf;
    nummeshes = hdr.num_meshes;
    numtris = hdr.num_triangles;
    numverts = hdr.num_vertexes;
    numjoints = hdr.num_joints;
    outposition = new float[3*numverts];
    outnormal = new float[3*numverts];
    outtangent = new float[3*numverts];
    outbitangent = new float[3*numverts];
    outframe = new Matrix4x4[hdr.num_joints];
    textures = new GLuint[nummeshes];
    memset(textures, 0, nummeshes*sizeof(GLuint));

    const char *str = hdr.ofs_text ? (char *)&buf[hdr.ofs_text] : "";
    iqmvertexarray *vas = (iqmvertexarray *)&buf[hdr.ofs_vertexarrays];
    for(int32_t i = 0; i < (int32_t)hdr.num_vertexarrays; i++) {
        iqmvertexarray &va = vas[i];
        switch(va.type) {
        case IQM_POSITION: if(va.format != IQM_FLOAT || va.size != 3) return false; inposition = (float *)&buf[va.offset]; lilswap(inposition, 3*hdr.num_vertexes); break;
        case IQM_NORMAL: if(va.format != IQM_FLOAT || va.size != 3) return false; innormal = (float *)&buf[va.offset]; lilswap(innormal, 3*hdr.num_vertexes); break;
        case IQM_TANGENT: if(va.format != IQM_FLOAT || va.size != 4) return false; intangent = (float *)&buf[va.offset]; lilswap(intangent, 4*hdr.num_vertexes); break;
        case IQM_TEXCOORD: if(va.format != IQM_FLOAT || va.size != 2) return false; intexcoord = (float *)&buf[va.offset]; lilswap(intexcoord, 2*hdr.num_vertexes); break;
        case IQM_BLENDINDEXES: if(va.format != IQM_UBYTE || va.size != 4) return false; inblendindex = (uint8_t *)&buf[va.offset]; break;
        case IQM_BLENDWEIGHTS: if(va.format != IQM_UBYTE || va.size != 4) return false; inblendweight = (uint8_t *)&buf[va.offset]; break;
        case IQM_COLOR: if(va.format != IQM_UBYTE || va.size != 4) return false; incolor = (uint8_t *)&buf[va.offset]; break;
        }
    }
    tris = (iqmtriangle *)&buf[hdr.ofs_triangles];
    meshes = (iqmmesh *)&buf[hdr.ofs_meshes];
    joints = (iqmjoint *)&buf[hdr.ofs_joints];
    if(hdr.ofs_adjacency) adjacency = (iqmtriangle *)&buf[hdr.ofs_adjacency];

    baseframe = new Matrix4x4[hdr.num_joints];
    inversebaseframe = new Matrix4x4[hdr.num_joints];

    for(int32_t i = 0; i < (int32_t)hdr.num_joints; i++) {
        iqmjoint &j = joints[i];
        auto translate = Vec3(j.translate[0], j.translate[1], j.translate[2]);
        auto scale = Vec3(j.scale[0], j.scale[1], j.scale[2]);
        auto rotate_mat = Matrix3x3(normalize(*(Quat*)(j.rotate)));
        auto scalerot_mat = rotate_mat * diagonal3x3(scale);
        baseframe[i] = Matrix4x4(Vec4(scalerot_mat[0],0), Vec4(scalerot_mat[1],0), Vec4(scalerot_mat[2],0), Vec4(translate,1));
        inversebaseframe[i] = inverse(baseframe[i]);
        if(j.parent >= 0)  {
            baseframe[i]        = baseframe[j.parent] * baseframe[i];
            inversebaseframe[i] = inversebaseframe[i] * inversebaseframe[j.parent];
        }
    }

    for(int32_t i = 0; i < (int32_t)hdr.num_meshes; i++) {
        iqmmesh &m = meshes[i];
        printf("%s: loaded mesh: %s\n", filename, &str[m.name]);
        textures[i] = loadtexture(&str[m.material], 0);
        if(textures[i]) printf("%s: loaded material: %s\n", filename, &str[m.material]);
    }

    return true;
}

bool loadiqmanims(const char *filename, const iqmheader &hdr, uint8_t *buf) {
    if((int32_t)hdr.num_poses != numjoints) return false;

    if(animdata) {
        if(animdata != meshdata) delete[] animdata;
        delete[] frames;
        animdata = nullptr;
        anims = nullptr;
        frames = 0;
        numframes = 0;
        numanims = 0;
    }        

    lilswap((uint32_t *)&buf[hdr.ofs_poses], hdr.num_poses*sizeof(iqmpose)/sizeof(uint32_t));
    lilswap((uint32_t *)&buf[hdr.ofs_anims], hdr.num_anims*sizeof(iqmanim)/sizeof(uint32_t));
    lilswap((uint16_t *)&buf[hdr.ofs_frames], hdr.num_frames*hdr.num_framechannels);
    if(hdr.ofs_bounds) lilswap((uint32_t *)&buf[hdr.ofs_bounds], hdr.num_frames*sizeof(iqmbounds)/sizeof(uint32_t));

    animdata = buf;
    numanims = hdr.num_anims;
    numframes = hdr.num_frames;

    const char *str = hdr.ofs_text ? (char *)&buf[hdr.ofs_text] : "";
    anims = (iqmanim *)&buf[hdr.ofs_anims];
    poses = (iqmpose *)&buf[hdr.ofs_poses];
    frames = new Matrix4x4[hdr.num_frames * hdr.num_poses];
    uint16_t *framedata = (uint16_t *)&buf[hdr.ofs_frames];
    if(hdr.ofs_bounds) bounds = (iqmbounds *)&buf[hdr.ofs_bounds];

    for(int32_t i = 0; i < (int32_t)hdr.num_frames; i++) {
        for(int32_t j = 0; j < (int32_t)hdr.num_poses; j++) {
            iqmpose &p = poses[j];
            float rotate[4];
            Vec3 translate, scale;
            translate.x = p.channeloffset[0]; if(p.mask&0x01) translate.x += *framedata++ * p.channelscale[0];
            translate.y = p.channeloffset[1]; if(p.mask&0x02) translate.y += *framedata++ * p.channelscale[1];
            translate.z = p.channeloffset[2]; if(p.mask&0x04) translate.z += *framedata++ * p.channelscale[2];
            rotate[0] = p.channeloffset[3]; if(p.mask&0x08) rotate[0] += *framedata++ * p.channelscale[3];
            rotate[1] = p.channeloffset[4]; if(p.mask&0x10) rotate[1] += *framedata++ * p.channelscale[4];
            rotate[2] = p.channeloffset[5]; if(p.mask&0x20) rotate[2] += *framedata++ * p.channelscale[5];
            rotate[3] = p.channeloffset[6]; if(p.mask&0x40) rotate[3] += *framedata++ * p.channelscale[6];
            scale.x = p.channeloffset[7]; if(p.mask&0x80) scale.x += *framedata++ * p.channelscale[7];
            scale.y = p.channeloffset[8]; if(p.mask&0x100) scale.y += *framedata++ * p.channelscale[8];
            scale.z = p.channeloffset[9]; if(p.mask&0x200) scale.z += *framedata++ * p.channelscale[9];
            // Concatenate each pose with the inverse base pose to avoid doing this at animation time.
            // If the joint has a parent, then it needs to be pre-concatenated with its parent's base pose.
            // Thus it all negates at animation time like so: 
            //   (parentPose * parentInverseBasePose) * (parentBasePose * childPose * childInverseBasePose) =>
            //   parentPose * (parentInverseBasePose * parentBasePose) * childPose * childInverseBasePose =>
            //   parentPose * childPose * childInverseBasePose
            auto rotateQuat = normalize(*(Quat*)(rotate));
            auto scalerot_mat = Matrix3x3(rotateQuat) * diagonal3x3(scale);
            Matrix4x4 m(Vec4(scalerot_mat[0],0), Vec4(scalerot_mat[1],0), Vec4(scalerot_mat[2],0), Vec4(translate,1));
            if(p.parent >= 0) frames[i*hdr.num_poses + j] = transpose(baseframe[p.parent] * m * inversebaseframe[j]);
            else frames[i*hdr.num_poses + j] = transpose(m * inversebaseframe[j]);
        }
    }
 
    for(int32_t i = 0; i < (int32_t)hdr.num_anims; i++) {
        iqmanim &a = anims[i];
        printf("%s: loaded anim: %s\n", filename, &str[a.name]);
    }
    
    return true;
}

bool loadiqm(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if(!f) return false;

    uint8_t *buf = nullptr;
    iqmheader hdr;
    if(fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) || memcmp(hdr.magic, IQM_MAGIC, sizeof(hdr.magic)))
        goto error;
    lilswap(&hdr.version, (sizeof(hdr) - sizeof(hdr.magic))/sizeof(uint32_t));
    if(hdr.version != IQM_VERSION)
        goto error;
    if(hdr.filesize > (16<<20)) 
        goto error; // sanity check... don't load files bigger than 16 MB
    buf = new uint8_t[hdr.filesize];
    if(fread(buf + sizeof(hdr), 1, hdr.filesize - sizeof(hdr), f) != hdr.filesize - sizeof(hdr))
        goto error;

    if(hdr.num_meshes > 0 && !loadiqmmeshes(filename, hdr, buf)) goto error;
    if(hdr.num_anims > 0 && !loadiqmanims(filename, hdr, buf)) goto error;
 
    fclose(f);
    return true;

error:
    printf("%s: error while loading\n", filename);
    if(buf != meshdata && buf != animdata) delete[] buf;
    fclose(f);
    return false;
}

// Note that this animates all attributes (position, normal, tangent, bitangent)
// for expository purposes, even though this demo does not use all of them for rendering.
void animateiqm(float curframe) {
    if(!numframes) return;

    int32_t frame1 = (int32_t)floor(curframe),
        frame2 = frame1 + 1;
    float frameoffset = curframe - frame1;
    frame1 %= numframes;
    frame2 %= numframes;
    Matrix4x4 *mat1 = &frames[frame1 * numjoints],
              *mat2 = &frames[frame2 * numjoints];
    // Interpolate matrixes between the two closest frames and concatenate with parent matrix if necessary.
    // Concatenate the result with the inverse of the base pose.
    // You would normally do animation blending and inter-frame blending here in a 3D engine.
    for(int32_t i = 0; i < numjoints; i++) {
        Matrix4x4 mat = mat1[i]*(1 - frameoffset) + mat2[i]*frameoffset;
        if(joints[i].parent >= 0) outframe[i] = mat * outframe[joints[i].parent];
        else outframe[i] = mat;
    }
    // The actual vertex generation based on the matrixes follows...
    const Vec3 *srcpos = (const Vec3 *)inposition, *srcnorm = (const Vec3 *)innormal;
    const Vec4 *srctan = (const Vec4 *)intangent; 
    Vec3 *dstpos = (Vec3 *)outposition, *dstnorm = (Vec3 *)outnormal, *dsttan = (Vec3 *)outtangent, *dstbitan = (Vec3 *)outbitangent; 
    const uint8_t *index = inblendindex, *weight = inblendweight;
    for(int32_t i = 0; i < numverts; i++) {
        // Blend matrixes for this vertex according to its blend weights. 
        // the first index/weight is always present, and the weights are
        // guaranteed to add up to 255. So if only the first weight is
        // presented, you could optimize this case by skipping any weight
        // multiplies and intermediate storage of a blended matrix. 
        // There are only at most 4 weights per vertex, and they are in 
        // sorted order from highest weight to lowest weight. Weights with 
        // 0 values, which are always at the end, are unused.
        Matrix4x4 mat = outframe[index[0]] * (weight[0]/255.0f);
        for(int32_t j = 1; j < 4 && weight[j]; j++)
            mat += outframe[index[j]] * (weight[j]/255.0f);

        // Transform attributes by the blended matrix.
        // Position uses the full 3x4 transformation matrix.
        // Normals and tangents only use the 3x3 rotation part 
        // of the transformation matrix.
        auto tmp = transpose(mat) * Vec4(*srcpos,1);
        *dstpos = Vec3(tmp.x, tmp.y, tmp.z);

        // Note that if the matrix includes non-uniform scaling, normal vectors
        // must be transformed by the inverse-transpose of the matrix to have the
        // correct relative scale. Note that invert(mat) = adjoint(mat)/determinant(mat),
        // and since the absolute scale is not important for a vector that will later
        // be renormalized, the adjoint-transpose matrix will work fine, which can be
        // cheaply generated by 3 cross-products.
        //
        // If you don't need to use joint scaling in your models, you can simply use the
        // upper 3x3 part of the position matrix instead of the adjoint-transpose shown 
        // here.
       
        auto mat_a = Vec3(mat[0].x, mat[0].y, mat[0].z);
        auto mat_b = Vec3(mat[1].x, mat[1].y, mat[1].z);
        auto mat_c = Vec3(mat[2].x, mat[2].y, mat[2].z);
        Matrix3x3 matnorm(
            cross(mat_b, mat_c),
            cross(mat_c, mat_a),
            cross(mat_a, mat_b)
        );

        *dstnorm = transpose(matnorm) * *srcnorm;
        // Note that input tangent data has 4 coordinates, 
        // so only transform the first 3 as the tangent vector.
        *dsttan = transpose(matnorm) * Vec3(srctan->x, srctan->y, srctan->z);
        // Note that bitangent = cross(normal, tangent) * sign, 
        // where the sign is stored in the 4th coordinate of the input tangent data.
        *dstbitan = cross(*dstnorm, *dsttan) * srctan->w;

        srcpos++;
        srcnorm++;
        srctan++;
        dstpos++;
        dstnorm++;
        dsttan++;
        dstbitan++;

        index += 4;
        weight += 4;
    }
}

float scale = 1, rotate = 0;
Vec3 translate(0, 0, 0);

void renderiqm() {
    static const GLfloat zero[4] = { 0, 0, 0, 0 }, 
                         one[4] = { 1, 1, 1, 1 },
                         ambientcol[4] = { 0.5f, 0.5f, 0.5f, 1 }, 
                         diffusecol[4] = { 0.5f, 0.5f, 0.5f, 1 },
                         lightdir[4] = { cosf(radians(-60)), 0, sinf(radians(-60)), 0 };

    glPushMatrix();
    glTranslatef(translate.x*scale, translate.y*scale, translate.z*scale);
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

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    glColor3f(1, 1, 1);
    glVertexPointer(3, GL_FLOAT, 0, numframes > 0 ? outposition : inposition);
    glNormalPointer(GL_FLOAT, 0, numframes > 0 ? outnormal : innormal);
    glTexCoordPointer(2, GL_FLOAT, 0, intexcoord);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    if(incolor) {
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, incolor);

        glEnableClientState(GL_COLOR_ARRAY);
    }

    glEnable(GL_TEXTURE_2D);

    for(int32_t i = 0; i < nummeshes; i++) {
        iqmmesh &m = meshes[i];
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glDrawElements(GL_TRIANGLES, 3*m.num_triangles, GL_UNSIGNED_INT, &tris[m.first_triangle]);
    }

    glDisable(GL_TEXTURE_2D);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    if(incolor) glDisableClientState(GL_COLOR_ARRAY);

    glDisable(GL_NORMALIZE);    
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);

    glPopMatrix();
}

void initgl() {
    glClearColor(0, 0, 0, 0);
    glClearDepth(1);
    glDisable(GL_FOG);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
}

int32_t scrw = 0, scrh = 0;

void reshapefunc(int32_t w, int32_t h) {
    scrw = w;
    scrh = h;
    glViewport(0, 0, w, h);
}

float camyaw = -90, campitch = 0, camroll = 0;
Vec3 campos(20, 0, 5);

void setupcamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLdouble aspect = double(scrw)/scrh,
             fov = radians(90),
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

void timerfunc(int32_t val) {
    animate += 10*val/1000.0f;
    glutPostRedisplay();
    glutTimerFunc(35, timerfunc, 35);
}

void displayfunc() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setupcamera();

    animateiqm(animate);
    renderiqm();

    glutSwapBuffers();
}

void keyboardfunc(uint8_t c, int32_t x, int32_t y) {
    switch(c) {
    case 27:
        exit(EXIT_SUCCESS);
        break;
    }
}

int32_t main(int argc, char **argv) {
    test();
    glutInitWindowSize(640, 480);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
    glutCreateWindow("IQM Demo");

    atexit(cleanupiqm);
    for(int32_t i = 1; i < argc; i++) {
        if(argv[i][0] == '-') switch(argv[i][1]) {
        case 's':
            if(i + 1 < argc) scale = clamp(atof(argv[++i]), 1e-8, 1e8);
            break;
        case 'r':
            if(i + 1 < argc) rotate = atof(argv[++i]);
            break;
        case 't':
            if(i + 1 < argc) switch(sscanf(argv[++i], "%f , %f , %f", &translate.x, &translate.y, &translate.z)) {
                case 1: translate = Vec3(0, 0, translate.x); break;
            }
            break;
        }
        else if(!loadiqm(argv[i])) return EXIT_FAILURE;
    }
    if(!meshdata && !loadiqm("mrfixit.iqm")) return EXIT_FAILURE;

    initgl();
   
    glutTimerFunc(35, timerfunc, 35);
    glutReshapeFunc(reshapefunc);
    glutDisplayFunc(displayfunc);
    glutKeyboardFunc(keyboardfunc);
    glutMainLoop();
     
    return EXIT_SUCCESS;
}

