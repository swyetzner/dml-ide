#include "designViewer.h"

DesignViewer::DesignViewer(Design *design, QWidget *parent)
    : QOpenGLWidget (parent),
      shaderProgram(nullptr),
      guideShaderProgram(nullptr),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_zoom(1.0f),
      m_mode(R_SOLID)
{

    this->design = design;

    // Set model constants
    n_models = long(design->volumes.size());
    n_modelVertices = new long(n_models);
    for (uint i = 0; i < uint(n_models); i++)
        n_modelVertices[i] = design->volumes[i]->model->n_vertices;

    // Set activate models to true
    activateModel = new bool(n_models);
    for (uint i = 0; i < uint(n_models); i++)
        activateModel[i] = true;

    // Allocate buffer id for each model
    vertexBuff_ids = new GLuint(uint(n_models));
    normalBuff_ids = new GLuint(uint(n_models));
    colorBuff_ids = new GLuint(uint(n_models));

    // Set rotation + zoom constants
    m_center = QVector3D(0, 0, 0);
    for (uint i = 0; i < uint(n_models); i++)
        m_center += design->volumes[i]->model->center();
    m_center /= n_models; // Use average model center

    getBoundingBox();
    m_dim = (m_minCorner - m_maxCorner).length();
    if (m_dim != 0.0f) m_zoom = 0.4f / m_dim;
}

// --------------------------------------------------------------------
// OPENGL SHADERS
// --------------------------------------------------------------------

static const char *vertexShaderSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 1) in vec3 vertexNorm;\n"
        "layout (location = 2) in vec4 vertexCol;\n"
        "out vec3 vertexPosition;\n"
        "out vec3 vertexNormal;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "uniform mat3 normalMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexPosition = vertexPos;\n"
        "   vertexNormal = normalMatrix * vertexNorm;\n"
        "   vertexColor = vertexCol;\n"
        "}\n";

static const char *fragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec3 vertexPosition;\n"
        "in vec3 vertexNormal;\n"
        "in vec4 vertexColor;\n"
        "uniform vec3 lightPos;\n"
        "void main() {\n"
        "   vec3 L = normalize(lightPos - vertexPosition);\n"
        "   float NL = max(dot(normalize(vertexNormal), L), 0.0);\n"
        "   vec3 color = vertexColor.rgb;\n"
        "   vec3 clColor = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
        "   fragColor = vec4(clColor, vertexColor.a);\n"
        "}\n";


// --------------------------------------------------------------------
// MODEL SLOTS
// --------------------------------------------------------------------


//  ----- toggleModel()
//  Slot that toggles show/hide for a model
//
void DesignViewer::toggleModel(int modelIndex) {
    if (activateModel[modelIndex]) {
        activateModel[modelIndex] = false;
    } else {
        activateModel[modelIndex] = true;
    }
    update();
}

//  ----- updateColors()
//  Slot that updates pushes model colors to graphics arrays
//
void DesignViewer::updateColors() {
    for (uint i =0; i < n_models; i++) {
        design->volumes[i]->model->updateColors();
    }
    updateBuffers();
    update();
}

// ---- render/////()
//  Slots to change the RenderMode
void DesignViewer::renderSolid() {
    m_mode = R_SOLID;
    update();
}

void DesignViewer::renderDesign() {
    m_mode = R_ALPHA;
    update();
}

void DesignViewer::renderWireframe() {
    m_mode = R_WIRE;
    update();
}


// --------------------------------------------------------------------
// OPENGL FUNCTIONS
// --------------------------------------------------------------------


//  ----- initVertexArray()
//  Create the vertex array object
//
void DesignViewer::initVertexArray() {
    vertexArray.create();
    vertexArray.bind();
}


//  ----- initShader()
//  Creates an QOpenGLShaderProgram with source code shaders
//  Binds uniform locations
//
void DesignViewer::initShader() {
    shaderProgram = new QOpenGLShaderProgram;
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    shaderProgram->bindAttributeLocation("vertexPos", 0);
    shaderProgram->bindAttributeLocation("vertexNorm", 1);
    shaderProgram->bindAttributeLocation("vertexCol", 2);
    shaderProgram->link();

    // Guide shader
    guideShaderProgram = new QOpenGLShaderProgram;
    GUtils::initOneColorShaderProgram(guideShaderProgram, 3);
    guideShaderProgram->link();

    // Bind uniform value locations
    shaderProgram->bind();
    projMatrixLoc = shaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = shaderProgram->uniformLocation("mvMatrix");
    normalMatrixLoc = shaderProgram->uniformLocation("normalMatrix");
    lightPosLoc = shaderProgram->uniformLocation("lightPos");

    // Set light position
    shaderProgram->setUniformValue(lightPosLoc, 0, 0, 70);
    shaderProgram->release();


    guideShaderProgram->bind();
    guideProjMatrixLoc = shaderProgram->uniformLocation("projMatrix");
    guideMVProjMatrixLoc = shaderProgram->uniformLocation("mvMatrix");
    guideShaderProgram->release();
}


//  ----- initBuffers()
//  Creates buffers for vertex and color data for each model
//
void DesignViewer::initBuffers() {

    for (uint m = 0; m < uint(n_models); m++) {

        GLuint vertexBuffer = 0;
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     3 * n_modelVertices[m] * long(sizeof(GLfloat)),
                     &design->volumes[m]->model->vertices[0],
                     GL_STATIC_DRAW);
        vertexBuff_ids[m] = vertexBuffer;

        GLuint normalBuffer = 0;
        glGenBuffers(1, &normalBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     3 * n_modelVertices[m] * long(sizeof(GLfloat)),
                     &design->volumes[m]->model->normals[0],
                     GL_STATIC_DRAW);
        normalBuff_ids[m] = normalBuffer;

        GLuint colorBuffer = 0;
        glGenBuffers(1, &colorBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     4 * n_modelVertices[m] * long(sizeof(GLfloat)),
                     design->volumes[m]->model->constColor(),
                     GL_STATIC_DRAW
        );
        colorBuff_ids[m] = colorBuffer;
    }

    // Axes buffer
    GUtils::createMainAxes(axesBuff_id, 10.0f);
}


//  ----- updateShader()
//  Updates uniform matrices and set them in the shader
//
void DesignViewer::updateShader() {
    world.setToIdentity();
    world.rotate(m_xRot / 16.0f, 1, 0, 0);
    world.rotate(m_yRot / 16.0f, 0, 1, 0);
    world.rotate(m_zRot / 16.0f, 0, 0, 1);
    world.scale(m_zoom);
    world.translate(-m_center);

    normals = world.normalMatrix();

    shaderProgram->bind();
    shaderProgram->setUniformValue(projMatrixLoc, projection);
    shaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    shaderProgram->setUniformValue(normalMatrixLoc, normals);
    shaderProgram->release();

    guideShaderProgram->bind();
    guideShaderProgram->setUniformValue(guideProjMatrixLoc, projection);
    guideShaderProgram->setUniformValue(guideMVProjMatrixLoc, camera * world);
    guideShaderProgram->release();
}


//  ----- updateBuffers()
//  Binds buffers and resyncs underlying data arrays
//
void DesignViewer::updateBuffers() {

    for (uint m = 0; m < uint(n_models); m++) {

        if (activateModel[m]) {

            glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_ids[m]);
            glBufferData(
                GL_ARRAY_BUFFER,
                3 * n_modelVertices[m] * long(sizeof(GLfloat)),
                &design->volumes[m]->model->vertices[0],
                GL_STATIC_DRAW
            );

            glBindBuffer(GL_ARRAY_BUFFER, normalBuff_ids[m]);
            glBufferData(
                GL_ARRAY_BUFFER,
                3 * n_modelVertices[m] * long(sizeof(GLfloat)),
                &design->volumes[m]->model->normals[0],
                GL_STATIC_DRAW
            );

            glBindBuffer(GL_ARRAY_BUFFER, colorBuff_ids[m]);
            glBufferData(
                GL_ARRAY_BUFFER,
                4 * n_modelVertices[m] * long(sizeof(GLfloat)),
                design->volumes[m]->model->constColor(),
                GL_STATIC_DRAW
            );
        }
    }
}


//  ----- drawVertexArray()
//  Bind buffers and draws the vertex array object
//
void DesignViewer::drawVertexArray() {

    // DRAW MODELS
    shaderProgram->bind();
    for (uint m = 0; m < uint(n_models); m++) {

        if (activateModel[m]) {

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glEnableVertexAttribArray(2);

            glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_ids[m]);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                                  0, nullptr);

            glBindBuffer(GL_ARRAY_BUFFER, normalBuff_ids[m]);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                                  0, nullptr);

            glBindBuffer(GL_ARRAY_BUFFER, colorBuff_ids[m]);
            glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE,
                                  0, nullptr);

            // RenderMode settings
            switch(m_mode) {
                case R_SOLID:
                    glClearColor(0, 0, 0, 0);
                    glDisable(GL_BLEND);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                break;
                case R_ALPHA:
                    glClearColor(1, 1, 1, 0);
                    glEnable(GL_BLEND);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                break;
                case R_WIRE:
                    glClearColor(0, 0, 0, 0);
                    glDisable(GL_BLEND);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                break;
            }

            glDrawArrays(GL_TRIANGLES, 0, GLsizei(n_modelVertices[m]));

            glDisableVertexAttribArray(2);
            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(0);

        }
    }
    shaderProgram->release();

    // DRAW AXES
    guideShaderProgram->bind();
    GUtils::drawMainAxes(3, axesBuff_id);
    guideShaderProgram->release();
}


//  ----- cleanUp()
//  Deletes objects and pointers
//
void DesignViewer::cleanUp() {

    if (shaderProgram == nullptr || guideShaderProgram == nullptr)
        return;

    makeCurrent();

    for (uint m = 0; m < uint(n_models); m++) {
        glDeleteBuffers(1, &vertexBuff_ids[m]);
        glDeleteBuffers(1, &colorBuff_ids[m]);
    }
    glDeleteBuffers(1, &axesBuff_id);

    delete vertexBuff_ids;
    delete colorBuff_ids;
    delete activateModel;
    delete n_modelVertices;
    delete shaderProgram;
    delete guideShaderProgram;
    shaderProgram = nullptr;
    guideShaderProgram = nullptr;
    doneCurrent();
}


//  ----- initCamera()
//  Initializes camera matrix
//
void DesignViewer::initCamera() {
    camera.setToIdentity();
}


//  ----- getBoundingBox()
//  Finds bounds of resulting model
//
void DesignViewer::getBoundingBox() {
    m_minCorner = QVector3D(0, 0, 0);
    m_maxCorner = QVector3D(0, 0, 0);

    if (n_models > 0) {
        float m_minPointX = design->volumes[0]->model->bounds.minCorner.x;
        float m_minPointY = design->volumes[0]->model->bounds.minCorner.y;
        float m_minPointZ = design->volumes[0]->model->bounds.minCorner.z;
        float m_maxPointX = design->volumes[0]->model->bounds.maxCorner.x;
        float m_maxPointY = design->volumes[0]->model->bounds.maxCorner.y;
        float m_maxPointZ = design->volumes[0]->model->bounds.maxCorner.z;

        for (uint i = 0; i < uint(n_models); i++) {
            m_minPointX = std::min(m_minPointX, design->volumes[i]->model->bounds.minCorner.x);
            m_minPointY = std::min(m_minPointY, design->volumes[i]->model->bounds.minCorner.y);
            m_minPointZ = std::min(m_minPointZ, design->volumes[i]->model->bounds.minCorner.z);
            m_maxPointX = std::min(m_maxPointX, design->volumes[i]->model->bounds.maxCorner.x);
            m_maxPointY = std::min(m_maxPointY, design->volumes[i]->model->bounds.maxCorner.y);
            m_maxPointZ = std::min(m_maxPointZ, design->volumes[i]->model->bounds.maxCorner.z);
        }

        m_minCorner = QVector3D(m_minPointX, m_minPointY, m_minPointZ);
        m_maxCorner = QVector3D(m_maxPointX, m_maxPointY, m_maxPointZ);
    }
}


// --------------------------------------------------------------------
// QT OPENGLWIDGET VIRUTAL FUNCTIONS
// --------------------------------------------------------------------


//  ----- initializeGL()
//  Virtual QOpenGLWidget function
//  Called once on widget construction
//
void DesignViewer::initializeGL() {

    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &DesignViewer::cleanUp);

    initializeOpenGLFunctions();

    initShader();
    initVertexArray();
    initBuffers();

    camera.setToIdentity();
    camera.translate(0, 0, -1);
}


//  ----- paintGL()
//  Virtual QOpenGLWidget function
//  Called every frame update
//
void DesignViewer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glClearColor(1, 1, 1, 0);

    vertexArray.bind();
    updateShader();
    //updateBuffers();
    drawVertexArray();
}


//  ----- resizeGL()
//  Virtual QOpenGLWidget function
//  Called for window resizing
//
void DesignViewer::resizeGL(int width, int height) {
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, 0.01f, 100.0f);
}


//  ----- mousePressEvent()
//  Overrides listener for mouse presses
//
void DesignViewer::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->pos();
}


//  ----- mouseMoveEvent()
//  Overrides listener for mouse moves to update rotation matrix
//
void DesignViewer::mouseMoveEvent(QMouseEvent *event)
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
void DesignViewer::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? m_zoom += m_zoom*0.1f : m_zoom -= m_zoom*0.1f;
    update();
}

//  ----- rotateBy()
//  Helper function to update rotation parameter and re-paint widget
//
void DesignViewer::rotateBy(int xAngle, int yAngle, int zAngle)
{
    m_xRot += xAngle;
    m_yRot += yAngle;
    m_zRot += zAngle;
    update();
}

