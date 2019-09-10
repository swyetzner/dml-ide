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

    struct SpringVisual {
        enum ColorScheme {
            NOTHING,
            MAX_STRESS,
            FORCES,
            ACTUATION
        } colorScheme;

        static QString colorSchemeName(ColorScheme cs) {
            switch (cs) {
                case NOTHING:
                    return "Nothing";
                case MAX_STRESS:
                    return "Max Stress";
                case FORCES:
                    return "Forces";
                case ACTUATION:
                    return "Actuation";
            }
        }
    };

signals:
    double getTimestep();
    double getRenderUpdate();
    void reloadSimulation();
    bool getShowText();
    int getVisualizeScheme();
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
    int simFrameInterval;
    int imageNumber, sampleNumber;
    int renderNumber;
    QString outputDir, sampleDir;
    QString imageMetricFile;

    void saveImage(const QImage &image);
    void getImageFileName(QString &outputFile);
    void getImageMetrics(QString &text);
    bool doRecord();
    void takeSample();

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
    void drawSolids();

    void cleanUp();

    void addColor(GLfloat *buffer, const GLfloat *color, int &count);
    void addMassColor(Mass *mass, GLfloat *buffer, int &count);
    void addSpringColor(Spring *spring, double totalStress, double totalForce, uint index, GLfloat *buffer, int &count);

    void initCamera();
    vector<QVector3D> getBoundingBox();
    void rotateBy(int x, int y, int z);
    void panCameraBy(int x, int y, int z);

    void renderText(const QString &text, int flags);
    void clearTextPanel();
    void updateTextPanel();

    // --------------------------------------------------------------------
    // OPENGL PROPERTIES
    // --------------------------------------------------------------------
    bool resizeBuffers;
    int n_springs;
    int n_masses;
    int n_planes;
    int frame_width;
    int frame_height;
    int near_plane;
    int far_plane;

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
    QOpenGLShaderProgram *barDepthShaderProgram;
    QOpenGLShaderProgram *defaultDepthShaderProgram;
    QOpenGLShaderProgram *depthQuadShaderProgram;
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
    GLuint depthMapFrameBuff_id;
    GLuint depthMapTexBuff_id;
    GLuint depthQuadBuff_id;

    bool showText;
    bool showOverlays;

    SpringVisual springVisual;

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
    QMatrix4x4 shadowProjection;
    QMatrix4x4 camera;
    QMatrix4x4 world;
    QMatrix3x3 normal;
    QMatrix4x4 ortho;
    QVector4D center;
    QVector3D eye;
    QVector3D light;
    QMatrix4x4 lightProjection, lightView;
    QMatrix4x4 lightSpace;
};


#endif //DMLIDE_SIMVIEWER_H
