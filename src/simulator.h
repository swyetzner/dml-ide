#ifndef TITANVIEWER_H
#define TITANVIEWER_H

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

#include "src/gUtils.h"
#include "model.h"
#include "optimizer.h"

#undef GRAPHICS
#include <Titan/sim.h>

using namespace std;

struct test_group {
    test_group() {};

    vector<Spring *> springs;
    double avgXForce;
    double avgYForce;
    double avgZForce;
};

class Simulator : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit Simulator(Simulation *sim, SimulationConfig *config,
            OptimizationConfig * optconfig = nullptr, QWidget *parent = nullptr);

    enum Status {
        STARTED,
        RECORDING,
        ENDED
    };

    Simulation *sim;
    SimulationConfig *config;
    Optimizer *optimizer;
    OptimizationConfig *optConfig;
    SpringInserter *springInserter;
    MassDisplacer *massDisplacer;

signals:
    void timeChange(double time);
    void setTimestep(double dt);
    double getTimestep();
    void setRenderUpdate(double dt);
    double getRenderUpdate();
    void setSpringConst(double k);
    double getSpringConst();
    bool getShowStress();
    void reloadSimulation();
    void log(const QString message);

public slots:
    // TODO separate out extensions of simulation functionality
    void start();
    void run();
    void step();
    void pause();
    void stop();
    void repeat();
    void recordVideo();
    void saveVideo();
    void prepare();

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
    // SIMULATION PROPERTIES + FUNCTIONS
    // --------------------------------------------------------------------
    long n_masses;
    long n_springs;
    long n_planes;
    double renderTimeStep;
    double springConstant;

    long n_masses_start;
    long n_springs_start;
    double totalMass_start;
    double totalLength;
    double totalEnergy;
    double totalLength_start;
    double totalEnergy_start;
    long steps;
    vector<double> avgXStresses;
    vector<double> avgYStresses;
    vector<double> avgZStresses;

    vector<uint> splitSprings;

    vector<Vec> bounds;

    double removalRate;

    test_group xLowSprings, xMidSprings, xHighSprings;
    test_group yLowSprings, yMidSprings, yHighSprings;
    test_group zLowSprings, zMidSprings, zHighSprings;
    QString calcAvgStresses(test_group t);

    int n_repeats;
    int optimizeAfter;
    double repeatTime;
    bool explicitRotation;
    Vec repeatRotation;
    bool equilibrium;
    int optimized;
    int closeToPrevious;
    double prevEnergy;
    int prevSteps;

    void iterateMassStructure(double ratio);
    void removePercentBars(double ratio);
    void adjustBarDiams();
    void addSecondOrderBars(double ratio);
    void moveStressedMasses(double ratio);

    // --------------------------------------------------------------------
    // VIDEO RENDERING
    // --------------------------------------------------------------------
    int viewerWidth;
    int viewerHeight;
    float framerate;
    int imageNumber;
    Status simStatus;
    QString outputDir;
    void saveImage(const QImage &image, const QString &outputFile);
    QVector<QFutureWatcher<void> *> futures;

    // --------------------------------------------------------------------
    // DATA COLLECTING
    // --------------------------------------------------------------------
    QString dataDir;
    void createDataDir();
    QString metricFile;
    QString customMetricFile;
    void writeMetricHeader(const QString &outputFile);
    void writeCustomMetricHeader(const QString &outputFile);
    void writeMetric(const QString &outputFile);
    void writeCustomMetric(const QString &outputFile);
    void writePos(const QString &outputFile);

    // --------------------------------------------------------------------
    // OPENGL FUNCTIONS
    // --------------------------------------------------------------------
    void updateVertices();
    void updateIndices();
    void updatePairVertices();
    void updateColors();
    void updateDiameters();
    void updatePlaneVertices();
    void updateOverlays();

    void initVertexArray();
    void initShader();
    void initBuffers();
    void updateShader();
    void updateBuffers();
    void drawVertexArray();

    void cleanUp();

    void addColor(GLfloat *buffer, const GLfloat *color, int &count);
    void addMassColor(Mass *mass, GLfloat *buffer, int &count);
    void addSpringColor(Spring *spring, double totalStress, double totalForce, uint index, GLfloat *buffer, int &count);

    void initCamera();
    vector<Vec> getBoundingBox();
    void rotateBy(int x, int y, int z);
    void panCameraBy(int x, int y, int z);

    void renderText(const QString &text, int flags);
    void updateTextPanel();

    // --------------------------------------------------------------------
    // OPENGL PROPERTIES
    // --------------------------------------------------------------------
    bool resizeBuffers;

    GLfloat *vertices;
    GLuint *indices;
    GLfloat *pairVertices = nullptr;
    GLfloat *boundVertices = nullptr;
    GLfloat *anchorVertices = nullptr;
    GLfloat *forceVertices = nullptr;
    GLfloat *colors = nullptr;
    GLfloat *diameters = nullptr;
    GLfloat *planeVertices = nullptr;

    QOpenGLShaderProgram *massShaderProgram;
    QOpenGLShaderProgram *springShaderProgram;
    QOpenGLShaderProgram *planeShaderProgram;
    QOpenGLShaderProgram *axesShaderProgram;
    QOpenGLShaderProgram *textShaderProgram;
    QOpenGLShaderProgram *boundShaderProgram;
    QOpenGLShaderProgram *anchorShaderProgram;
    QOpenGLShaderProgram *forceShaderProgram;
    QOpenGLVertexArrayObject vertexArray;
    QOpenGLFramebufferObject *frameBuffer;

    GLuint vertexBuff_id;
    GLuint indexBuff_id;
    GLuint pairVertexBuff_id;
    GLuint massColorBuff_id;
    GLuint springColorBuff_id;
    GLuint diameterBuff_id;
    GLuint planeVertexBuff_id;
    GLuint axesVertexBuff_id;
    GLuint textVertexBuff_id;
    GLuint boundVertexBuff_id;
    GLuint anchorVertexBuff_id;
    GLuint forceVertexBuff_id; // (pos[3], force[3], ...)

    bool showVertices;
    bool showStress;
    bool showCrossSection;
    bool showOverlays;

    int verticesCount;
    vector<Vec> extForces;
    vector<Vec> anchors;
    Vec cutPlane;
    double cutPlaneOffset;

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
    int textProjLoc;
    int fprojMatrixLoc;
    int fmvMatrixLoc;
    int arrowLenLoc;
    int scaleLoc;
    int cpXLoc, cpYLoc, cpZLoc;
    int centerLoc;
    float scale;
    QMatrix4x4 projection;
    QMatrix4x4 camera;
    QMatrix4x4 world;
    QMatrix3x3 normal;
    QMatrix4x4 ortho;
    QVector4D clipPlaneX;
    QVector4D clipPlaneY;
    QVector4D clipPlaneZ;
    QVector4D center;

    // --------------------------------------------------------------------
};

#endif // TITANVIEWER_H
