#ifndef MODELVIEWER_H
#define MODELVIEWER_H

#include <QObject>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>

#include "model.h"

class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit ModelViewer(model_data *models, QWidget *parent = nullptr);

public slots:
    void toggleModel(int modelIndex);
    void updateDisplay();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent * event) override;
    void mouseMoveEvent(QMouseEvent * event) override;
    void wheelEvent(QWheelEvent * event) override;

private:
    model_data *models;

    // --------------------------------------------------------------------
    // MODEL PROPERTIES
    // --------------------------------------------------------------------
    bool *activateModel;
    long *n_vertices;

    // --------------------------------------------------------------------
    // OPENGL FUNCTIONS
    // --------------------------------------------------------------------

    void updateColors();

    void initVertexArray();
    void initShader();
    void initBuffers();
    void updateShader();
    void updateBuffers();
    void drawVertexArray();

    void cleanUp();

    void initCamera();
    void rotateBy(int x, int y, int z);

    // --------------------------------------------------------------------
    // OPENGL PROPERTIES
    // --------------------------------------------------------------------

    GLfloat *colors;

    QOpenGLShaderProgram *shaderProgram;
    QOpenGLVertexArrayObject vertexArray;

    GLuint *vertexBuff_ids;
    GLuint *colorBuff_ids;

    // Rotate + Zoom
    int m_xRot;
    int m_yRot;
    int m_zRot;
    float m_zoom;
    QVector3D m_center;
    QPoint m_lastMousePos;

    // Rotation Matrices
    int projMatrixLoc;
    int mvMatrixLoc;
    int normalMatrixLoc;
    int lightPosLoc;
    QMatrix4x4 projection;
    QMatrix4x4 camera;
    QMatrix4x4 world;
    // --------------------------------------------------------------------

};

#endif // MODELVIEWER_H
