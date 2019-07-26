//
// Created by sw3390 on 7/25/19.
//

#ifndef DMLIDE_SIMVIEWER_H
#define DMLIDE_SIMVIEWER_H

#include <QMatrix4x4>
#include <QObject>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_4_Core>
#include <QOpenGLFramebufferObject>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QFutureWatcher>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QDir>
#include <QProcess>
#include <QFileDialog>

#include "simulator.h"
#include "gUtils.h"

class SimViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT
public:
    explicit SimViewer(Simulator *simulator, QWidget *parent = nullptr);

    Simulator *simulator;

signals:
    double getTimestep();
    double getRenderUpdate();
    void reloadSimulation();
    bool getShowStress();
    void stopCriteriaSat();
    void timeChange(double time);
    void log(const QString message);

public slots:
    void start();
    void step();
    void pause();
    void stop();
    void run();
    void recordVideo();
    void saveVideo();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent * event) override;
    void keyPressEvent(QKeyEvent * event) override;

private:
    // --------------------------------------------------------------------
    // VIDEO RENDERING
    // --------------------------------------------------------------------
    bool RECORDING;
    float framerate;
    int imageNumber;
    QString outputDir;
    void saveImage(const QImage &image, const QString &outputFile);

    // --------------------------------------------------------------------
    // OPENGL FUNCTIONS
    // --------------------------------------------------------------------
    void updatePairVertices();
    void updateColors();
    void updateDiameters();
    void updatePlaneVertices();
    void updateOverlays();

    void initVertexArray();
    void initShaders();
    void initBuffers();
    void updateShaders();
    void updateBuffers();
    void drawVertexArray();

    void cleanUp();

    void addColor(GLfloat *buffer, const GLfloat *color, int &count);
    void addMassColor(Mass *mass, GLfloat *buffer, int &count);
    void addSpringColor(Spring *spring, double totalStress, double totalForce, uint index, GLfloat *buffer, int &count);

    void initCamera();
    vector<QVector3D> getBoundingBox();
    void rotateBy(int x, int y, int z);
    void panCameraBy(int x, int y, int z);

    void renderText(const QString &text, int flags);
    void updateTextPanel();

    // --------------------------------------------------------------------
    // OPENGL PROPERTIES
    // --------------------------------------------------------------------
    bool resizeBuffers;
    int n_springs;
    int n_masses;
    int n_planes;

    GLfloat *pairVertices = nullptr;
    GLfloat *anchorVertices = nullptr;
    GLfloat *forceVertices = nullptr;
    GLfloat *colors = nullptr;
    GLfloat *diameters = nullptr;
    GLfloat *planeVertices = nullptr;

    QOpenGLShaderProgram *springShaderProgram;
    QOpenGLShaderProgram *planeShaderProgram;
    QOpenGLShaderProgram *axesShaderProgram;
    QOpenGLShaderProgram *anchorShaderProgram;
    QOpenGLShaderProgram *forceShaderProgram;
    QOpenGLVertexArrayObject vertexArray;
    QOpenGLFramebufferObject *frameBuffer;

    GLuint pairVertexBuff_id;
    GLuint massColorBuff_id;
    GLuint springColorBuff_id;
    GLuint diameterBuff_id;
    GLuint planeVertexBuff_id;
    GLuint axesVertexBuff_id;
    GLuint anchorVertexBuff_id;
    GLuint forceVertexBuff_id; // (pos[3], force[3], ...)

    bool showStress;
    bool showOverlays;

    vector<QVector3D> bounds;
    int verticesCount;
    vector<Vec> extForces;
    vector<Vec> anchors;

    // Animation
    int m_frame;
    QTimer *timer;

    // Rotate + Zoom + Pan
    int m_xRot;
    int m_yRot;
    int m_zRot;
    float m_xPan, m_yPan, m_zPan;
    float m_zoom;
    QVector3D m_center;
    QPoint m_lastMousePos;

    // Uniforms
    int lineColorLoc;
    int projMatrixLoc;
    int mvMatrixLoc;
    int normalMatrixLoc;
    int lightPosLoc;
    int fprojMatrixLoc;
    int fmvMatrixLoc;
    int arrowLenLoc;
    int scaleLoc;
    int centerLoc;
    float scale;
    QMatrix4x4 projection;
    QMatrix4x4 camera;
    QMatrix4x4 world;
    QMatrix3x3 normal;
    QMatrix4x4 ortho;
    QVector4D center;
};


#endif //DMLIDE_SIMVIEWER_H
