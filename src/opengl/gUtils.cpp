#include "gUtils.h"


GUtils::GUtils()
{

}

// Interpolates value i bounded by [imin, imax] to range [omin, omax]
//---------------------------------------------------------------------------
double GUtils::interpolate(double omin, double omax, double imin, double imax, double i) {
//---------------------------------------------------------------------------
    return omin + ((omax - omin) * (i - imin) / (imax - imin));
}

// Interpolates value i bounded by [imin, imax] to color value
// Colors are defined by GLfloat[4] pointers
// Return output in GLfloat[4] o
//---------------------------------------------------------------------------
void GUtils::interpolateColors(const GLfloat (& omin) [4], const GLfloat (& omax) [4], double imin, double imax, double i, GLfloat *o) {
//---------------------------------------------------------------------------

    for (int a = 0; a < 4; a++) {
        o[a] = interpolate(omin[a], omax[a], imin, imax, i);
    }

}

// Initializes a QOpenGLShaderProgram for drawing guides
//---------------------------------------------------------------------------
void GUtils::initOneColorShaderProgram(QOpenGLShaderProgram *p, int vaoVertexLoc) {
//---------------------------------------------------------------------------
    p->addShaderFromSourceCode(QOpenGLShader::Vertex, oneColorVertexShaderSource);
    p->addShaderFromSourceCode(QOpenGLShader::Fragment, oneColorFragmentShaderSource);
    p->bindAttributeLocation("vertexPos", vaoVertexLoc);
    p->link();
}

// Generic function for array buffer initialization
//---------------------------------------------------------------------------
void GUtils::initBuffer(GLuint &bufferId, long size, GLfloat *data) {
//---------------------------------------------------------------------------
    GLuint id = 0;
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glGenBuffers(1, &id);
    f->glBindBuffer(GL_ARRAY_BUFFER, id);
    f->glBufferData(GL_ARRAY_BUFFER, size * long(sizeof(GLfloat)), data, GL_STATIC_DRAW);
    bufferId = id;
}


// Initializes frame buffer object and texture buffer for shadow mapping
//---------------------------------------------------------------------------
void GUtils::initShadowBuffers(GLuint &framebuffId, GLuint &texbuffId, int frameHeight, int frameWidth) {
//---------------------------------------------------------------------------

    QOpenGLContext *context = QOpenGLContext::currentContext();
    QOpenGLFunctions_3_3_Core *f = context->versionFunctions<QOpenGLFunctions_3_3_Core>();

    GLuint depthMapFrameBuff;
    GLuint depthMap;
    f->glGenFramebuffers(1, &depthMapFrameBuff);
    f->glGenTextures(1, &depthMap);

    f->glBindTexture(GL_TEXTURE_2D, depthMap);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, frameHeight, frameHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    // Set texture parameters
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Attach to FBO
    f->glBindFramebuffer(GL_FRAMEBUFFER, depthMapFrameBuff);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    f->glDrawBuffer(GL_NONE);
    f->glReadBuffer(GL_NONE);
    if(f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        qDebug() << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!";
    f->glBindFramebuffer(GL_FRAMEBUFFER, 0);

    framebuffId = depthMapFrameBuff;
    texbuffId = depthMap;
}


// Fills vector of matrices with cube directions for point shadow
//---------------------------------------------------------------------------
void GUtils::fillShadowTransforms(QMatrix4x4 shadowProj, QVector3D lightPos,
                                  std::vector<QMatrix4x4> &shadowTransforms) {
//---------------------------------------------------------------------------

    QOpenGLContext *context = QOpenGLContext::currentContext();
    QOpenGLFunctions_3_3_Core *f = context->versionFunctions<QOpenGLFunctions_3_3_Core>();

    QMatrix4x4 face;
    face.setToIdentity();
    for (int i = 0; i < 6; i++) {
        shadowTransforms.push_back(face);
    }

    shadowTransforms[0].lookAt(lightPos, lightPos + QVector3D(1, 0, 0), lightPos + QVector3D(0, -1, 0));
    shadowTransforms[1].lookAt(lightPos, lightPos + QVector3D(-1, 0, 0), lightPos + QVector3D(0, -1, 0));
    shadowTransforms[2].lookAt(lightPos, lightPos + QVector3D(0, 1, 0), lightPos + QVector3D(0, 0, 1));
    shadowTransforms[3].lookAt(lightPos, lightPos + QVector3D(0, -1, 0), lightPos + QVector3D(0, 0, -1));
    shadowTransforms[4].lookAt(lightPos, lightPos + QVector3D(0, 0, 1), lightPos + QVector3D(0, -1, 0));
    shadowTransforms[5].lookAt(lightPos, lightPos + QVector3D(0, 0, -1), lightPos + QVector3D(0, -1, 0));

    for (int i = 0; i < 6; i++) {
        shadowTransforms[i] = shadowProj * shadowTransforms[i];
    }
}

// Create float data for a single axis (line)
void GUtils::createAxisData(vec3 origin, vec3 end, GLfloat *data) {

    // Allocate axis data
    GLfloat *p = data;
    *p++ = origin.x - end.x;
    *p++ = origin.y - end.y;
    *p++ = origin.z - end.z;
    *p++ = origin.x + end.x;
    *p++ = origin.y + end.y;
    *p++ = origin.z + end.z;
}


// Set ups main axes data and buffers
// Uses axes size
void GUtils::createMainAxes(GLuint &id, float size) {

    GLfloat *data = new GLfloat[6 * 3];

    createAxisData(vec3(0.0f, 0.0f, 0.0f), vec3(size, 0.0f, 0.0f), data);
    createAxisData(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, size, 0.0f), data + 6);
    createAxisData(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, size), data + 12);

    initBuffer(id, 6 * 3, data);
}

// Draw axis data
// Assumes already existing vertex array
// Takes vertex array attribute position as parameter
void GUtils::drawMainAxes(GLuint vaIndex, GLuint &bufferId) {
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glEnableVertexAttribArray(vaIndex);
    f->glBindBuffer(GL_ARRAY_BUFFER, bufferId);
    f->glVertexAttribPointer(vaIndex, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    f->glEnable(GL_LINE_SMOOTH);
    f->glDrawArrays(GL_LINES, 0, GLsizei(6));

    f->glDisableVertexAttribArray(vaIndex);
}


const char * GUtils::oneColorVertexShaderSource =
        "#version 330\n"
        "in vec3 vertexPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vec4(0.8, 0.8, 0.8, 0.8);\n"
        "}\n";

const char * GUtils::oneColorFragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";
