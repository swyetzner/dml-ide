#ifndef GUTILS_H
#define GUTILS_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <glm/glm.hpp>
using namespace glm;

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

    static const char *oneColorVertexShaderSource;
    static const char *oneColorFragmentShaderSource;
};

#endif // GUTILS_H
