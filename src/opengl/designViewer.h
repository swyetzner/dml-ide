#ifndef DESIGNVIEWER_H
#define DESIGNVIEWER_H

#include <QObject>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QOpenGLVertexArrayObject>

#include "gUtils.h"
#include "model.h"

class DesignViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit DesignViewer(Design *design, QWidget *parent = nullptr);

    enum RenderMode { R_SOLID, R_ALPHA, R_WIRE };

public slots:
    void toggleModel(int volIndex);
    //void updateVertices();
    void updateColors();
    void renderSolid();
    void renderDesign();
    void renderWireframe();


protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent * event) override;
    void mouseMoveEvent(QMouseEvent * event) override;
    void wheelEvent(QWheelEvent * event) override;

private:

    Design *design;

    // --------------------------------------------------------------------
    // MODEL PROPERTIES
    // --------------------------------------------------------------------
    long n_models;
    long n_vertices;
    int *n_modelVertices;
    bool *activateModel;

    // --------------------------------------------------------------------
    // OPENGL FUNCTIONS
    // --------------------------------------------------------------------

    void initVertexArray();
    void initShader();
    void initBuffers();
    void updateShader();
    void updateBuffers();
    void drawVertexArray();
    void drawModels();

    void cleanUp();

    void initCamera();
    void getBoundingBox();
    void rotateBy(int x, int y, int z);

    // --------------------------------------------------------------------
    // OPENGL PROPERTIES
    // --------------------------------------------------------------------

    QOpenGLShaderProgram *shaderProgram;
    QOpenGLShaderProgram *guideShaderProgram;
    QOpenGLShaderProgram *depthShaderProgram;
    QOpenGLShaderProgram *depthQuadProgram;
    QOpenGLVertexArrayObject vertexArray;

    GLfloat *vertices = nullptr;
    GLfloat *colors = nullptr;

    GLuint *vertexBuff_ids;
    GLuint *normalBuff_ids;
    GLuint *colorBuff_ids;
    GLuint axesBuff_id;
    GLuint depthFrameBuff_id;
    GLuint depthTexBuff_id;
    GLuint quadBuff_id;

    float frameWidth;
    float frameHeight;
    float nearPlane;
    float farPlane;

    // Rotate + Zoom
    int m_xRot;
    int m_yRot;
    int m_zRot;
    float m_zoom;
    QVector3D m_center;
    QVector3D m_minCorner;
    QVector3D m_maxCorner;
    float m_dim;
    QPoint m_lastMousePos;
    RenderMode m_mode;

    // Rotation Matrices
    int projMatrixLoc;
    int mvMatrixLoc;
    int normalMatrixLoc;
    int lightPosLoc;
    int guideProjMatrixLoc;
    int guideMVProjMatrixLoc;
    QMatrix4x4 projection;
    QMatrix4x4 camera;
    QMatrix4x4 world;
    QMatrix3x3 normals;
    QVector3D eye;
    // --------------------------------------------------------------------
};

#endif // DESIGNVIEWER_H
