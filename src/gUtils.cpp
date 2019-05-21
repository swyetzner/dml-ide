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

// Create float data for a single axis (line)
void GUtils::createAxisData(vec3 origin, vec3 end, GLfloat *data) {

    // Allocate axis data
    GLfloat *p = data;
    *p++ = origin.x;
    *p++ = origin.y;
    *p++ = origin.z;
    *p++ = end.x;
    *p++ = end.y;
    *p++ = end.z;
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

    f->glDisable(GL_DEPTH_TEST);

    f->glEnableVertexAttribArray(vaIndex);
    f->glBindBuffer(GL_ARRAY_BUFFER, bufferId);
    f->glVertexAttribPointer(vaIndex, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

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
        "   vertexColor = vec4(1.0, 0.2, 0.2, 0.8);\n"
        "}\n";

const char * GUtils::oneColorFragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";
