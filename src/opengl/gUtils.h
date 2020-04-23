#ifndef GUTILS_H
#define GUTILS_H

#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#define GLM_FORCE_MESSAGES
#include <glm/glm.hpp>

typedef glm::vec3 vec3;
typedef glm::vec4 vec4;

class GUtils
{
public:
    GUtils();

    static double interpolate(double omin, double omax, double imin, double imax, double i);
    static void interpolateColors(const GLfloat (& omin) [4], const GLfloat (& omax) [4],
            double imin, double imax, double i, GLfloat *o);

    static void initOneColorShaderProgram(QOpenGLShaderProgram *p, int vaoVertexLoc);
    static void initBuffer(GLuint &id, long size, GLfloat *data);

    static void createAxisData(vec3 origin, vec3 end, GLfloat *data);
    static void createMainAxes(GLuint &bufferId, float size);
    static void drawMainAxes(GLuint vaPos, GLuint &bufferId);

    static void initShadowBuffers(GLuint &framebuffId, GLuint &texbuffId, int frameHeight, int frameWidth);
    static void fillShadowTransforms(QMatrix4x4 shadowProj, QVector3D lightPos, std::vector<QMatrix4x4> &shadowTransforms);
    static void initDepthShaderProgram(QOpenGLShaderProgram *p, int vaoVertexLoc);

    static const char *oneColorVertexShaderSource;
    static const char *oneColorFragmentShaderSource;
};

#endif // GUTILS_H
