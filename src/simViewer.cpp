//
// Created by sw3390 on 7/25/19.
//

#include "simViewer.h"

SimViewer::SimViewer(Simulator *simulator, QWidget *parent)
        : QOpenGLWidget(parent),
          springShaderProgram(nullptr),
          planeShaderProgram(nullptr),
          anchorShaderProgram(nullptr),
          forceShaderProgram(nullptr),
          frameBuffer(nullptr),
          m_frame(0),
          m_xRot(0),
          m_yRot(0),
          m_zRot(0),
          m_xPan(0), m_yPan(0), m_zPan(0),
          m_zoom(0.1f) {

    this->simulator = simulator;

    resizeBuffers = true;
    showStress = false;
    showOverlays = true;

    RECORDING = false;
    outputDir = "output";
    sampleDir = "output/sample";
    framerate = 500;
    renderNumber = 0;
    simFrameInterval = 0;
    sampleNumber = 0;

    bounds = getBoundingBox();

    this->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
    this->setFocus();

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &SimViewer::run);
}

// --------------------------------------------------------------------
// SIMULATION CONTROLS
// --------------------------------------------------------------------

void SimViewer::start() {
    simulator->setSyncTimestep(getRenderUpdate());
    simulator->setSimTimestep(getTimestep());
    simFrameInterval = int(1.0 / getRenderUpdate());
    qInfo() << "VIDEO RECORDING";
    qInfo() << "Renders per sim second: " << simFrameInterval;
    qInfo() << "Video frames per sim second: " << int(framerate);
    qInfo() << "Renders per video frame: " << int(simFrameInterval / framerate);

    timer->start(1);
}

void SimViewer::step() {
    simulator->setSyncTimestep(getRenderUpdate());
    simulator->setSimTimestep(getTimestep());

    simulator->runStep();
}

void SimViewer::pause() {
    simulator->runSimulation(false);
    timer->stop();
}

void SimViewer::stop() {
    simulator->runSimulation(false);
    timer->stop();
    reloadSimulation();
}

void SimViewer::run() {
    simulator->runSimulation(true);
    update();

    // Record video
    if (RECORDING && doRecord()) saveImage(grabFramebuffer());
    renderNumber++;
}

void SimViewer::recordVideo() {
    RECORDING = true;
    imageNumber = 0;

    // Output directory
    assert(!outputDir.isEmpty());
    QString currentPath = QDir::currentPath();
    QDir output(currentPath + QDir::separator() + outputDir);
    if (!output.exists()) {
        log("Output folder does not exist. Creating...");
        QDir::current().mkdir(outputDir);
    } else {
        output.removeRecursively();
        QDir::current().mkdir(outputDir);
    }
    // Image metric file
    imageMetricFile = QString(QDir::currentPath() + QDir::separator() +
            outputDir + QDir::separator() + "imageCatalog.txt");

    // Record initial frame
    saveImage(grabFramebuffer());
}

void SimViewer::saveVideo() {
    if (RECORDING) {
        pause();
        RECORDING = false;
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save simulation video"), QDir::currentPath(),
                                                        tr("Video Files (*.mp4 *.mpeg4 *.avi);;All files (*)"));
        if (fileName.isEmpty())
            return;

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            log("Unable to open file: " + file.errorString());
            return;
        }

        QString outputPrefix = QDir::currentPath() + QDir::separator() + outputDir + QDir::separator();
        QString fps = QString::number(int(framerate));
        qDebug() << "Outputting video:" <<  fileName << " FPS" << fps;
        QString createVideoProgram = "ffmpeg -framerate " + fps + " -y -i " + outputPrefix +
                                     "dmlFrame_%d.png -vf \"eq=saturation=2.0, pad=ceil(iw/2)*2:ceil(ih/2)*2\" -vcodec libx264 -crf 25 -pix_fmt yuv420p ";

        createVideoProgram += fileName;
        QProcess *process = new QProcess(this);
        process->start(createVideoProgram);
    }
}


// --------------------------------------------------------------------
// OPENGL SHADERS
// --------------------------------------------------------------------

static const char *vertexShaderPlaneSource =
        "#version 330\n"
        "layout (location = 3) in vec3 vertexPos;\n"
        "out vec4 vertPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   vertPos = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   gl_Position = vertPos;\n"
        "   vertexColor = vec4(0.8, 0.8, 0.8, 1.0);\n"
        "}\n";

static const char *fragmentShaderPlaneSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertPos;\n"
        "in vec4 vertexColor;\n"
        "uniform mat3 normMatrix;\n"
        "uniform vec3 normal;\n"
        "uniform vec3 lightPos;\n"
        "void main() {\n"
        "    vec3 L = normalize(lightPos - vertPos.xyz);\n"
        "    float NL = max(dot(normalize(normMatrix * normal), L), 0.0);\n"
        "    vec4 clColor = clamp(vertexColor * 0.2 + vertexColor * 0.8 * NL, 0.0, 1.0);\n"
        "    fragColor = vec4(clColor.xyz, 1.0);\n"
        "}\n";


static const char *vertexShaderAnchorSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vec4(0.4, 0.3, 0.3, 1.0);\n"
        "}\n";

static const char *fragmentShaderAnchorSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";


//  ----- Color constants
static const GLfloat defaultMassColor[] = { 0.2f, 5.0f, 0.0f, 1.0f };
static const GLfloat fixedMassColor[] = { 1.0f, 0.0f, 0.0f, 1.0f };
static const GLfloat forceMassColor[] = { 0.0f, 0.0f, 1.0f, 1.0f };
static const GLfloat defaultSpringColor[] = { 0.5, 1.0, 0.0, 0.1 };
static const GLfloat expandSpringColor[] = { 0.5, 1.0, 0.0, 1.0 };
static const GLfloat contractSpringColor[] = { 0.7, 0.0, 1.0, 1.0 };
static const GLfloat brokenSpringColor[] = { 1.0f, 0.0f, 0.0f, 1.0f };

// --------------------------------------------------------------------
// OPENGL FUNCTIONS
// --------------------------------------------------------------------

//  ----- updatePairVertices()
//  Pushes simulation spring connections to vertices array
//
void SimViewer::updatePairVertices() {

    if (resizeBuffers) {
        n_springs = simulator->sim->springs.size();
        delete pairVertices;
        pairVertices = new GLfloat[2 * 3 * n_springs];
    }
    verticesCount = 0;

    double d;

    for (int i = 0; i < n_springs; i++) {
        GLfloat *p = pairVertices + verticesCount;
        Spring *s = simulator->sim->getSpringByIndex(i);

        Mass *m = s->_left;

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);

        m = s->_right;

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);

        verticesCount += 6;
    }
}


//  ----- updateOverlays()
//  Pushes bounds, anchor vertices, and force vertices
//  to vertex array
//
void SimViewer::updateOverlays() {

    // FORCE VERTICES
    n_masses = long(simulator->sim->masses.size());
    extForces = vector<Vec>();

    for (int i = 0; i < n_masses; i++) {
        Mass *m = simulator->sim->getMassByIndex(i);

        if (m->force.norm() > 1E-6) {
            extForces.push_back(m->pos);

            Vec av = m->force.normalized();
            extForces.push_back(av);
        }
    }

    if (resizeBuffers) {
        delete forceVertices;
        forceVertices = new GLfloat[3 * extForces.size()];
    }
    int forceCount = 0;
    for (int j = 0; j < extForces.size(); j++) {
        GLfloat *p = forceVertices + forceCount;

        *p++ = GLfloat(extForces[j][0]);
        *p++ = GLfloat(extForces[j][1]);
        *p++ = GLfloat(extForces[j][2]);
        forceCount += 3;
    }

    // ANCHORS
    anchors = vector<Vec>();
    for (int i = 0; i < n_masses; i++) {
        Mass *m = simulator->sim->getMassByIndex(i);

        if (m->constraints.fixed) {
            anchors.push_back(m->pos);
        }
    }
    if (resizeBuffers) {
        delete anchorVertices;
        anchorVertices = new GLfloat[3 * anchors.size()];
    }
    int anchorCount = 0;
    for (int j = 0; j < anchors.size(); j++) {
        GLfloat *p = anchorVertices + anchorCount;

        *p++ = GLfloat(anchors[j][0]);
        *p++ = GLfloat(anchors[j][1]);
        *p++ = GLfloat(anchors[j][2]);
        anchorCount += 3;
    }
}

//  ----- addColor()
//  Fills a color buffer with a 4-flot color starting from an offset
//
void SimViewer::addColor(GLfloat *buffer, const GLfloat *color, int &count) {

    GLfloat *p = buffer + count;

    *p++ = color[0];
    *p++ = color[1];
    *p++ = color[2];
    *p++ = color[3];

    count += 4;
}

//  ----- addMassColor()
//  Fills a color buffer with a 4-float color based on mass type
//
void SimViewer::addMassColor(Mass *mass, GLfloat *buffer, int &count) {

    if (mass->constraints.fixed) {

        addColor(buffer, fixedMassColor, count);

    } else if (fabs(mass->force[0]) > 1E-6 || fabs(mass->force[1]) > 1E-6 || fabs(mass->force[2]) > 1E-6) {

        addColor(buffer, forceMassColor, count);

    } else {

        addColor(buffer, defaultMassColor, count);
    }
}

//  ----- addSpringColor()
//  Fills a color buffer with two 4-float colors based on spring type
//
void SimViewer::addSpringColor(Spring *spring, double totalStress, double totalForce, uint index, GLfloat *buffer, int &count) {
    if (spring->_broken) {

        addColor(buffer, brokenSpringColor, count);
        addColor(buffer, brokenSpringColor, count);

    } else if (showStress) {


        GLfloat *stressColor = new GLfloat[4];
        //for (uint i = 0; i < 3; i++) {
        //    stressColor[i] = defaultSpringColor[i] * float(spring->_max_stress / totalStress);
        //}
        GUtils::interpolateColors(contractSpringColor, expandSpringColor, -totalForce, totalForce,
                                  spring->_curr_force, stressColor);

        // Set alpha to max stress value
        stressColor[3] = GUtils::interpolate(0.01, 1.0, 0.0, 1.0, fabs(spring->_curr_force) / totalForce);
        //stressColor[3] = float(abs(spring->_curr_force) / totalForce);

        addColor(buffer, stressColor, count);
        addColor(buffer, stressColor, count);

        delete[] stressColor;
    } else {

        addColor(buffer, defaultSpringColor, count);
        addColor(buffer, defaultSpringColor, count);

    }
}

//  ----- updateColors()
//  Fills color array with values according to mass properties
//
void SimViewer::updateColors() {
    int colorsCount = 0;
    int n_springs = simulator->sim->springs.size();
    if (resizeBuffers) {
        delete colors;
        colors = new GLfloat[2 * 4 * (2 * n_springs)];
    }

    for (int i = 0; i < n_springs; i++) {
        Spring *s = simulator->sim->getSpringByIndex(i);

        // MASS VERTEX COLORS
        Mass *m1 = s->_left;
        addMassColor(m1, colors, colorsCount);

        Mass *m2 = s->_right;
        addMassColor(m2, colors, colorsCount);
    }

    double totalStress = 0;
    double totalForce = 0;
    if (showStress) {
        for (Spring *s: simulator->sim->springs) {
            totalStress = fmax(totalStress, s->_max_stress);
            totalForce = fmax(totalForce, fabs(s->_curr_force));
        }
    }

    for (int i = 0; i < n_springs; i++) {
        Spring *s = simulator->sim->getSpringByIndex(i);

        // SPRING VERTEX COLORS
        addSpringColor(s, totalStress, totalForce, i, colors, colorsCount);
    }
}

//  ----- updateDiameters()
//  Pushes simulation bar diameters to spring arrays
//
void SimViewer::updateDiameters() {
    int diamCount = 0;
    int n_springs = simulator->sim->springs.size();

    if (resizeBuffers) {
        delete diameters;
        diameters = new GLfloat[2 * n_springs];
    }

    for (int i = 0; i < n_springs; i++) {
        GLfloat *p = diameters + diamCount;
        Spring *s = simulator->sim->getSpringByIndex(i);

        *p++ = s->_diam;
        *p++ = s->_diam;

        diamCount += 2;
    }
}

//  ----- updatePlaneVertices()
//  Pushes simulation plane positions to plane arrays
//
void SimViewer::updatePlaneVertices() {
    int pVerticesCount = 0;
    n_planes = simulator->sim->planes.size();

    if (!planeVertices) planeVertices = new GLfloat[12 * n_planes];

    for (ulong i =0; i < ulong(n_planes); i++) {
        GLfloat *p = planeVertices + pVerticesCount;
        ContactPlane *c = simulator->sim->planes[i];

        Vec temp = (dot(c->_normal, Vec(0, 1, 0)) < 0.8) ? Vec(0, 1, 0) : Vec(1, 0, 0);
        Vec u1 = cross(c->_normal, temp).normalized();
        Vec u2 = cross(c->_normal, u1).normalized();
        Vec v1 = c->_offset * c->_normal - 1*u1 - 1*u2;
        Vec v2 = c->_offset * c->_normal + 1*u1 - 1*u2;
        Vec v3 = c->_offset * c->_normal + 1*u1 + 1*u2;
        Vec v4 = c->_offset * c->_normal - 1*u1 + 1*u2;

        *p++ = GLfloat(v1[0]); *p++ = GLfloat(v1[1]); *p++ = GLfloat(v1[2]);
        *p++ = GLfloat(v2[0]); *p++ = GLfloat(v2[1]); *p++ = GLfloat(v2[2]);
        *p++ = GLfloat(v3[0]); *p++ = GLfloat(v3[1]); *p++ = GLfloat(v3[2]);
        *p++ = GLfloat(v4[0]); *p++ = GLfloat(v4[1]); *p++ = GLfloat(v4[2]);
        pVerticesCount += 12;
    }
}

//  ----- initVertexArray()
//  Create the vertex array object
//
void SimViewer::initVertexArray() {
    vertexArray.create();
    vertexArray.bind();
}


//  ----- initShader()
//  Creates an QOpenGLShaderProgram with source code shaders
//  Binds uniform locations
//
void SimViewer::initShaders() {

    springShaderProgram = new QOpenGLShaderProgram;
    springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/lineCylinders.vert");
    springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/resources/shaders/lineCylinders.geom");
    springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/lineCylinders.frag");
    springShaderProgram->bindAttributeLocation("vertexPos", 0);
    springShaderProgram->bindAttributeLocation("colorPos", 2);
    springShaderProgram->bindAttributeLocation("diameter", 3);
    springShaderProgram->link();

    planeShaderProgram = new QOpenGLShaderProgram;
    planeShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderPlaneSource);
    planeShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderPlaneSource);
    planeShaderProgram->bindAttributeLocation("vertexPos", 3);
    planeShaderProgram->link();

    axesShaderProgram = new QOpenGLShaderProgram;
    GUtils::initOneColorShaderProgram(axesShaderProgram, 4);
    axesShaderProgram->link();

    forceShaderProgram = new QOpenGLShaderProgram;
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/arrows.vert");
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/resources/shaders/arrows.geom");
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/arrows.frag");
    forceShaderProgram->bindAttributeLocation("vertexPos", 5);
    forceShaderProgram->bindAttributeLocation("forceVec", 6);
    forceShaderProgram->link();

    anchorShaderProgram = new QOpenGLShaderProgram;
    anchorShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderAnchorSource);
    anchorShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderAnchorSource);
    anchorShaderProgram->bindAttributeLocation("vertexPos", 0);
    anchorShaderProgram->link();

    // Bind uniform matrix locations

    springShaderProgram->bind();
    projMatrixLoc = springShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = springShaderProgram->uniformLocation("mvMatrix");
    normalMatrixLoc = springShaderProgram->uniformLocation("normMatrix");
    lightPosLoc = springShaderProgram->uniformLocation("lightPos");
    springShaderProgram->release();
    qDebug() << "locs " << projMatrixLoc << mvMatrixLoc << normalMatrixLoc << lightPosLoc;

    planeShaderProgram->bind();
    projMatrixLoc = planeShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = planeShaderProgram->uniformLocation("mvMatrix");
    planeShaderProgram->uniformLocation("normMatrix");
    planeShaderProgram->uniformLocation("normal");
    planeShaderProgram->uniformLocation("lightPos");
    planeShaderProgram->release();

    axesShaderProgram->bind();
    projMatrixLoc = axesShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = axesShaderProgram->uniformLocation("mvMatrix");
    axesShaderProgram->release();

    forceShaderProgram->bind();
    fprojMatrixLoc = forceShaderProgram->uniformLocation("projMatrix");
    fmvMatrixLoc = forceShaderProgram->uniformLocation("mvMatrix");
    arrowLenLoc = forceShaderProgram->uniformLocation("arrowLength");
    scaleLoc = forceShaderProgram->uniformLocation("scale");
    forceShaderProgram->release();

    anchorShaderProgram->bind();
    projMatrixLoc = anchorShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = anchorShaderProgram->uniformLocation("mvMatrix");
    anchorShaderProgram->release();

}


//  ----- initBuffers()
//  Creates buffers for vertex, index, and color data
//
void SimViewer::initBuffers() {

    // 1
    GLuint pairBuffer = 0;
    glGenBuffers(1, &pairBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, pairBuffer);
    glBufferData(GL_ARRAY_BUFFER, verticesCount * long(sizeof(GLfloat)), pairVertices, GL_DYNAMIC_DRAW);
    pairVertexBuff_id = pairBuffer;

    // 2
    GLuint colorBuffer = 0;
    glGenBuffers(1, &colorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors, GL_DYNAMIC_DRAW);
    massColorBuff_id = colorBuffer;

    // 3
    GLuint scolorBuffer = 0;
    glGenBuffers(1, &scolorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, scolorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors + (8 * n_springs), GL_DYNAMIC_DRAW);
    springColorBuff_id = scolorBuffer;

    GLuint diamBuffer = 0;
    glGenBuffers(1, &diamBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, diamBuffer);
    glBufferData(GL_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLfloat)), diameters, GL_DYNAMIC_DRAW);
    diameterBuff_id = diamBuffer;

    // 4
    GLuint planeBuffer = 0;
    glGenBuffers(1, &planeBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, planeBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * 4 * long(sizeof(GLfloat)), planeVertices, GL_DYNAMIC_DRAW);
    planeVertexBuff_id = planeBuffer;

    // 5
    GUtils::createMainAxes(axesVertexBuff_id, 10.0f);

    // 6
    GLuint forceBuffer = 0;
    glGenBuffers(1, &forceBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, forceBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * extForces.size() * long(sizeof(GLfloat)), forceVertices, GL_DYNAMIC_DRAW);
    forceVertexBuff_id = forceBuffer;

    // 7
    GLuint anchorBuffer = 0;
    glGenBuffers(1, &anchorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, anchorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * anchors.size() * long(sizeof(GLfloat)), anchorVertices, GL_DYNAMIC_DRAW);
    anchorVertexBuff_id = anchorBuffer;

    qDebug() << "Initialized buffers";
}

//  ----- updateShader()
//  Updates uniform matrices and set them in the shader
//
void SimViewer::updateShaders() {

    getBoundingBox();

    world.setToIdentity();
    world.rotate(m_xRot / 16.0f, 1, 0, 0);
    world.rotate(m_yRot / 16.0f, 0, 1, 0);
    world.rotate(m_zRot / 16.0f, 0, 0, 1);
    world.scale(m_zoom);
    scale = simulator->sim->springs[0]->_diam / 0.002;

    normal = world.normalMatrix();
    QVector3D pnq = QVector3D(0,0,0);
    if (n_planes == 1) {
        Vec pn = simulator->sim->planes[0]->_normal;
        pnq = QVector3D(pn[0], pn[1], pn[2]);
    }
    QVector3D light = QVector3D(0, 0, 70);

    springShaderProgram->bind();
    springShaderProgram->setUniformValue("projMatrix", projection);
    springShaderProgram->setUniformValue("mvMatrix", camera * world);
    springShaderProgram->setUniformValue("normMatrix", normal);
    springShaderProgram->setUniformValue("lightPos", light);
    springShaderProgram->release();

    planeShaderProgram->bind();
    planeShaderProgram->setUniformValue("projMatrix", projection);
    planeShaderProgram->setUniformValue("mvMatrix", camera * world);
    planeShaderProgram->setUniformValue("normMatrix", normal);
    planeShaderProgram->setUniformValue("normal", pnq);
    planeShaderProgram->setUniformValue("lightPos", light);
    planeShaderProgram->release();

    axesShaderProgram->bind();
    axesShaderProgram->setUniformValue(projMatrixLoc, projection);
    axesShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    axesShaderProgram->release();

    forceShaderProgram->bind();
    forceShaderProgram->setUniformValue(fprojMatrixLoc, projection);
    forceShaderProgram->setUniformValue(fmvMatrixLoc, camera * world);
    forceShaderProgram->setUniformValue(arrowLenLoc, 0.1f);
    forceShaderProgram->setUniformValue(scaleLoc, scale);
    forceShaderProgram->release();

    anchorShaderProgram->bind();
    anchorShaderProgram->setUniformValue(projMatrixLoc, projection);
    anchorShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    anchorShaderProgram->release();
}


//  ----- updateBuffers()
//  Binds buffers and resyncs underlying data arrays
//
void SimViewer::updateBuffers() {

    glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, verticesCount * long(sizeof(GLfloat)), pairVertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, massColorBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, springColorBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors + (8 * n_springs), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, diameterBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLfloat)), diameters, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, planeVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * 4 * long(sizeof(GLfloat)), planeVertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, forceVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * extForces.size() * long(sizeof(GLfloat)), forceVertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, anchorVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * anchors.size() * long(sizeof(GLfloat)), anchorVertices, GL_DYNAMIC_DRAW);

}


//  ----- drawVertexArray()
//  Bind buffers and draws the vertex array object
//
void SimViewer::drawVertexArray() {

    // DRAW AXES
    axesShaderProgram->bind();
    GUtils::drawMainAxes(4, axesVertexBuff_id);
    axesShaderProgram->release();

    //  DRAW PLANES
    planeShaderProgram->bind();
    glEnableVertexAttribArray(3);
    glBindBuffer(GL_ARRAY_BUFFER, planeVertexBuff_id);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4 * GLsizei(n_planes));

    glDisableVertexAttribArray(3);
    planeShaderProgram->release();


    //  DRAW SPRINGS
    springShaderProgram->bind();
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, springColorBuff_id);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, diameterBuff_id);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, nullptr);

    glEnable(GL_DEPTH_TEST);
    glDrawArrays(GL_LINES, 0, GLsizei(verticesCount / 3));

    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(0);
    springShaderProgram->release();

    //  DRAW OVERLAYS
    forceShaderProgram->bind();
    glEnableVertexAttribArray(5);
    glEnableVertexAttribArray(6);

    glBindBuffer(GL_ARRAY_BUFFER, forceVertexBuff_id);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), nullptr);
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));

    glPointSize(10.0f);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDrawArrays(GL_POINTS, 0, GLsizei(extForces.size()));

    glDisableVertexAttribArray(6);
    glDisableVertexAttribArray(5);
    forceShaderProgram->release();

    anchorShaderProgram->bind();
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, anchorVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glEnable(GL_DEPTH_TEST);
    glDrawArrays(GL_POINTS, 0, GLsizei(anchors.size()));

    glDisableVertexAttribArray(0);
    anchorShaderProgram->release();
}


//  ----- cleanUp()
//  Deletes objects and pointers
//
void SimViewer::cleanUp() {

    delete pairVertices;
    delete anchorVertices;
    delete forceVertices;
    delete colors;
    delete diameters;
    delete planeVertices;

    delete timer;

    makeCurrent();
    delete springShaderProgram;
    delete planeShaderProgram;
    delete forceShaderProgram;
    delete anchorShaderProgram;
    springShaderProgram = nullptr;
    planeShaderProgram = nullptr;
    forceShaderProgram = nullptr;
    anchorShaderProgram = nullptr;
    doneCurrent();
}




//  ----- initCamera()
//  Initializes camera matrix
//
void SimViewer::initCamera() {
    camera.setToIdentity();
    camera.translate(0, 0, -1);

    QVector3D up = QVector3D(-simulator->sim->global[0], -simulator->sim->global[1], -simulator->sim->global[2]);
    up.normalize();
    if (up == QVector3D(0, 0, 0)) { up = QVector3D(0, 0, 1); }

    float span = (bounds[0] - bounds[1]).length();
    m_zoom =  span > 1E-5? float(1/span) : 1.0f;

    camera.lookAt(QVector3D(4, 3, 3), QVector3D(0, 0, 0), up);
}


//  ----- getBoundingBox()
//  Finds bounds of the simulation masses
//
vector<QVector3D> SimViewer::getBoundingBox() {
    QVector3D minCorner;
    QVector3D maxCorner;

    double maxX = -numeric_limits<double>::max();
    double maxY = -numeric_limits<double>::max();
    double maxZ = -numeric_limits<double>::max();
    double minX = numeric_limits<double>::max();
    double minY = numeric_limits<double>::max();
    double minZ = numeric_limits<double>::max();

    for (Mass *m : simulator->sim->masses) {
        maxX = std::max(maxX, m->pos[0]);
        maxY = std::max(maxY, m->pos[1]);
        maxZ = std::max(maxZ, m->pos[2]);
        minX = std::min(minX, m->pos[0]);
        minY = std::min(minY, m->pos[1]);
        minZ = std::min(minZ, m->pos[2]);
    }
    minCorner = QVector3D(minX, minY, minZ);
    maxCorner = QVector3D(maxX, maxY, maxZ);
    vector<QVector3D> bounds = vector<QVector3D>();
    bounds.push_back(minCorner);
    bounds.push_back(maxCorner);

    m_center = 0.5 * (minCorner + maxCorner);
    return bounds;
}




// --------------------------------------------------------------------
// QT OPENGLWIDGET VIRUTAL FUNCTIONS
// --------------------------------------------------------------------


//  ----- initializeGL()
//  Virtual QOpenGLWidget function
//  Called once on widget construction
//
void SimViewer::initializeGL() {

    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &SimViewer::cleanUp);

    initializeOpenGLFunctions();
    qDebug() << "OpenGL Version: " << QLatin1String((const char *) glGetString(GL_VERSION));
    qDebug() << "GSLS version: " << QLatin1String((const char *) glGetString(GL_SHADING_LANGUAGE_VERSION));

    simulator->sim->initCudaParameters();
    qDebug() << "Initializing OpenGL";

    updateColors();
    qDebug() << "Initialized colors";
    updatePairVertices();
    qDebug() << "Initialized spring vertices";
    updateDiameters();
    qDebug() << "Initialized spring diameters";
    updatePlaneVertices();
    qDebug() << "Initialized plane vertices";
    updateOverlays();
    qDebug() << "Initialized overlays";

    qDebug() << "Filled graphics data";

    initShaders();
    initVertexArray();
    initBuffers();

    qDebug() << "Initialized QOpenGLWidget";
    qDebug() << "Format depth" << this->format();

    initCamera();
    ortho.setToIdentity();
    ortho.ortho(0.0f, 800.0f, 0.0f, 600.0, 0.1f, 100.0f);

    GLint i, count;
    GLint size;
    GLenum type;
    const GLsizei bufSize = 16;
    GLchar  name[bufSize];
    GLsizei  length;

    printf("GL_FLOAT_VEC3: %u\n", GL_FLOAT_VEC3);
    printf("GL_FLOAT_MAT4: %u\n", GL_FLOAT_MAT4);
    glGetProgramiv(springShaderProgram->programId(), GL_ACTIVE_UNIFORMS, &count);
    qDebug() << "Active uniforms: " << count;
    for (i = 0; i < count; i++) {
        glGetActiveUniform(springShaderProgram->programId(), GLuint(i), bufSize, &length, &size, &type, name);
        printf("Uniform #%d Type: %u Name: %s\n", i, type, name);
    }
}


//  ----- paintGL()
//  Virtual QOpenGLWidget function
//  Called every frame update
//
void SimViewer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_CLAMP);

    glClearColor(0, 0, 0, 0);

    if (n_springs != int(simulator->sim->springs.size())) {
        resizeBuffers = true;
    }
    if (showStress || getShowStress() != showStress || resizeBuffers) {
        showStress = getShowStress();
        updateColors();
    }
    if (resizeBuffers) {
        updatePairVertices();
    }
    updateDiameters();
    updatePlaneVertices();
    updateOverlays();

    vertexArray.bind();
    updateShaders();
    updateBuffers();
    drawVertexArray();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    vertexArray.release();

    updateTextPanel();
    m_frame++;

}



//  ----- resizeGL()
//  Virtual QOpenGLWidget function
//  Called for window resizing
//
void SimViewer::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, 0.1f, 20.0f);
}


//  ----- renderText()
//  Uses a QPainter to overlay text on OpenGL widget
//
void SimViewer::renderText(const QString &text, int flags) {
    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Helvetica [Cronyx]", 14));
    painter.drawText(rect(), flags, text);
}



//  ----- updateTextPanel()
//  Updates text panel with current stats about the simulation
//
void SimViewer::updateTextPanel() {

    sim_metrics metrics;
    simulator->getSimMetrics(metrics);

    QString upperPanel;
    QString simName = simulator->config->id;
    double latticeCutoff = simulator->config->lattices[0]->unit[0];
    QString material = simulator->config->lattices[0]->material->id;

    switch(metrics.optimize_rule.method) {
        case OptimizationRule::REMOVE_LOW_STRESS:
            upperPanel.sprintf("%s --- %s\n\n"
                               "Bars: %d\n"
                               "Time: %.2lf s\n"
                               "Weight remaining: %.2lf%%\n"
                               "Deflection: %.4lf m\n"
                               "Optimization iterations: %d\n"
                               "Optimization threshold: %.1lf%% bars per iteration",
                               simName.toUpper().toStdString().c_str(),
                               metrics.optimize_rule.methodName().replace(QChar('_'), QChar(' ')).toStdString().c_str(),
                               metrics.nbars, metrics.time,
                               100.0 * metrics.totalLength / metrics.totalLength_start,
                               metrics.deflection,
                               metrics.optimize_iterations,
                               metrics.optimize_rule.threshold * 100);
            break;

        case OptimizationRule::MASS_DISPLACE:
            upperPanel.sprintf("%s\n\n"
                               "Bars: %d\n"
                               "Time: %.2lf s\n"
                               "Weight remaining: %.2lf%%\n"
                               "Deflection: %.4lf m\n"
                               "Energy: %.2lf%%, %.4lf (current), %.4lf (start)\n"
                               "Optimization iterations: %d\n"
                               "Relaxation interval: %d order\n"
                               "Displacement: %.4lf meters",
                               simName.toUpper().toStdString().c_str(),
                               metrics.nbars, metrics.time,
                               100.0 * metrics.totalLength / metrics.totalLength_start,
                               metrics.deflection,
                               (100.0 * metrics.totalEnergy / ((metrics.totalEnergy_start > 0)? metrics.totalEnergy_start : metrics.totalEnergy)),
                               metrics.totalEnergy, metrics.totalEnergy_start,
                               metrics.optimize_iterations,
                               metrics.relaxation_interval,
                               metrics.displacement);
            break;

        case OptimizationRule::NONE:
            upperPanel.sprintf("%s\n\n"
                               "Bars: %d\n"
                               "Time: %.2lf s\n",
                               simName.toUpper().toStdString().c_str(),
                               metrics.nbars, metrics.time);
            break;
    }


    QString lowerPanel =
            tr("Random Lattice\n"
               "Spacing cutoff: %1 m\n"
               "Bounds: %2 x %3 x %4 m\n"
               "Material: %5")
                    .arg(latticeCutoff)
                    .arg(bounds[1][0] - bounds[0][0])
                    .arg(bounds[1][1] - bounds[0][1])
                    .arg(bounds[1][2] - bounds[0][2])
                    .arg(material.toUpper());

    renderText(upperPanel, Qt::AlignLeft | Qt::AlignTop);
    renderText(lowerPanel, Qt::AlignLeft | Qt::AlignBottom);

    emit timeChange(metrics.time);
}



//  ----- mousePressEvent()
//  Overrides listener for mouse presses
//
void SimViewer::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->pos();
}


//  ----- mouseMoveEvent()
//  Overrides listener for mouse moves to update rotation matrix
//
void SimViewer::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastMousePos.x();
    int dy = event->y() - m_lastMousePos.y();

    if (event->buttons() & Qt::LeftButton) {
        rotateBy(6 * dy, 6 * dx, 0);
    }
    m_lastMousePos = event->pos();
}


//  ----- wheelEvent()
//  Overrides listener for zooming to scale object
//
void SimViewer::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? m_zPan = 1 : m_zPan = -1;
    panCameraBy(0, 0, int(m_zPan) * 5);
    update();
}




//  ----- keyPressEvent()
//  Overrides listener for key presses
//
void SimViewer::keyPressEvent(QKeyEvent *event)
{
    switch(event->key()) {
        case Qt::Key_Left:
            panCameraBy(1, 0, 0);
            break;
        case Qt::Key_Right:
            panCameraBy(-1, 0, 0);
            break;
        case Qt::Key_Down:
            panCameraBy(0, 1, 0);
            break;
        case Qt::Key_Up:
            panCameraBy(0, -1, 0);
            break;
    }
}


//  ----- rotateBy()
//  Helper function to update rotation parameter and re-paint widget
//
void SimViewer::rotateBy(int xAngle, int yAngle, int zAngle)
{
    m_xRot += xAngle;
    m_yRot += yAngle;
    m_zRot += zAngle;
    update();
}


//  ----- panCameraBy()
//  Helper function to update camera parameter and re-paint widget
//
void SimViewer::panCameraBy(int x, int y, int z)
{
    m_xPan = 0.1f * x;
    m_yPan = 0.1f * y;
    m_zPan = 0.1f * z;

    QVector4D u = camera.row(0);
    QVector4D v = camera.row(1);
    QVector4D n = camera.row(2);

    camera.setColumn(3, QVector4D(u.w() + m_xPan,
                                  v.w() + m_yPan,
                                  n.w() + m_zPan,
                                  1.0f));
    update();
}


// --------------------------------------------------------------------
// VIDEO OUTPUT
// --------------------------------------------------------------------
void SimViewer::saveImage(const QImage &image) {

    QString outputFile;
    getImageFileName(outputFile);
    image.save(outputFile);

    QString imageMetrics;
    getImageMetrics(imageMetrics);
    QFile file(imageMetricFile);
    file.open(QIODevice::Append);
    file.write(imageMetrics.toUtf8());

    qDebug() << "Frame saved" << outputFile << " Render" << renderNumber;
    imageNumber++;
}

// Gets metrics for last image
void SimViewer::getImageMetrics(QString &text) {
    sim_metrics metrics;
    simulator->getSimMetrics(metrics);

    switch(metrics.optimize_rule.method) {
        case OptimizationRule::REMOVE_LOW_STRESS:
            text.sprintf("IMAGE --- dmlFrame_%d\n"
                           "Bars: %d\n"
                           "Time: %.2lf s\n"
                           "Weight (start): %.4lf\n"
                           "Weight (current): %.4lf\n"
                           "Weight remaining: %.2lf%%\n"
                           "Deflection: %.4lf m\n"
                           "Optimization iterations: %d\n"
                           "Optimization threshold: %.1lf%% bars per iteration\n\n",
                           imageNumber,
                           metrics.nbars, metrics.time,
                           metrics.totalLength_start, metrics.totalLength,
                           100.0 * metrics.totalLength / metrics.totalLength_start,
                           metrics.deflection,
                           metrics.optimize_iterations,
                           metrics.optimize_rule.threshold * 100);
            break;

        case OptimizationRule::MASS_DISPLACE:
            text.sprintf("IMAGE --- dmlFrame_%d\n"
                         "Bars: %d\n"
                         "Time: %.2lf s\n"
                         "Weight (start): %.4lf\n"
                         "Weight (current): %.4lf\n"
                         "Weight remaining: %.2lf%%\n"
                         "Deflection: %.4lf m\n"
                         "Energy (start): %.4lf\n"
                         "Energy (current): %.4lf\n"
                         "Energy remaining: %.2lf%%\n"
                         "Optimization iterations: %d\n"
                         "Relaxation interval: %d order\n"
                         "Displacement: %.4lf meters\n\n",
                         imageNumber,
                         metrics.nbars, metrics.time,
                         metrics.totalLength_start, metrics.totalLength,
                         100.0 * metrics.totalLength / metrics.totalLength_start,
                         metrics.deflection,
                         metrics.totalEnergy, metrics.totalEnergy_start,
                         (100.0 * metrics.totalEnergy / ((metrics.totalEnergy_start > 0)? metrics.totalEnergy_start : metrics.totalEnergy)),
                         metrics.optimize_iterations,
                         metrics.relaxation_interval,
                         metrics.displacement);
            break;

        case OptimizationRule::NONE:
            text.sprintf("IMAGE --- dmlFrame_%d\n"
                               "Bars: %d\n"
                               "Time: %.2lf s\n\n",
                               imageNumber,
                               metrics.nbars, metrics.time);
           break;
    }
}

// Gets output file name based on imageNumber
void SimViewer::getImageFileName(QString &outputFile) {
    outputFile = QString(QDir::currentPath() + QDir::separator() +
                                 outputDir + QDir::separator() +
                                 "dmlFrame_" + QString::number(imageNumber) + ".png");
}

// Returns true if renderNumber satisfies framerate
// simFrameInterval = renders per second (sim time)
// framerate = desired images per second (sim time)
bool SimViewer::doRecord() {
    return (renderNumber % int(simFrameInterval / framerate) == 0);
}

// Saves image into sample folder
void SimViewer::takeSample() {
    QString samplePrefix = QDir::currentPath() + QDir::separator() + sampleDir + QDir::separator();
    qInfo() << "Sampling video" << sampleNumber;

    QString outputFile = samplePrefix + "sample_" + QString::number(sampleNumber) + ".png";
    QImage image = grabFramebuffer();
    image.save(outputFile);
    sampleNumber++;
}