#include "modelviewer.h"

ModelViewer::ModelViewer(model_data *models, QWidget *parent)
    : QOpenGLWidget (parent),
      shaderProgram(nullptr),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_zoom(0.1f)
{

    this->models = models;

    colors = new GLfloat[4 * models->n_vertices];

    // Set model sizes
    n_vertices = new long(models->n_models);
    for (uint m = 0; m < uint(models->n_models); m++) {
        int modelStartIndex = (m == 0)? 0 : models->model_indices[m - 1];
        int modelEndIndex = models->model_indices[m];
        n_vertices[m] = modelEndIndex - modelStartIndex;
    }

    // Set activate models to true
    activateModel = new bool(models->n_models);
    for (uint i = 0; i < uint(models->n_models); i++)
        activateModel[i] = true;

    // Allocate buffer id for each model
    vertexBuff_ids = new GLuint(uint(models->n_models));
    colorBuff_ids = new GLuint(uint(models->n_models));

    // Set rotation + zoom constants
    m_center = models->center();
    if (models->m_dim != 0.0f) m_zoom = 15.0f / models->m_dim;
}

// --------------------------------------------------------------------
// OPENGL SHADERS
// --------------------------------------------------------------------
static const char *vertexShaderSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 1) in vec4 vertexCol;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vertexCol;\n"
        "}\n";

static const char *fragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";


// --------------------------------------------------------------------
// MODEL SLOTS
// --------------------------------------------------------------------


//  ----- toggleModel()
//  Slot that toggles show/hide for a model
//
void ModelViewer::toggleModel(int modelIndex) {
    if (activateModel[modelIndex]) {
        activateModel[modelIndex] = false;
    } else {
        activateModel[modelIndex] = true;
    }
    update();
}

//  ----- updateDisplay()
//  Slot to update viewer on-demand
//
void ModelViewer::updateDisplay() {

    updateColors();
    updateBuffers();
    update();
}

// --------------------------------------------------------------------
// OPENGL FUNCTIONS
// --------------------------------------------------------------------


//  ----- updateColors()
//  Fills in model color for every vertex buffer
//
void ModelViewer::updateColors() {

    uint modelStart = 0;
    for (uint m = 0; m < uint(models->n_models); m++) {

        for (uint i = modelStart; i < uint(models->model_indices[m]); i++) {

            colors[i*4]   = GLfloat(models->colors[m].r);
            colors[i*4+1] = GLfloat(models->colors[m].g);
            colors[i*4+2] = GLfloat(models->colors[m].b);
            colors[i*4+3] = GLfloat(models->colors[m].a);
        }
        modelStart = uint(models->model_indices[m]);
    }
    //colors = models->constColor();
}


//  ----- initVertexArray()
//  Create the vertex array object
//
void ModelViewer::initVertexArray() {
    vertexArray.create();
    vertexArray.bind();
}


//  ----- initShader()
//  Creates an QOpenGLShaderProgram with source code shaders
//  Binds uniform locations
//
void ModelViewer::initShader() {
    shaderProgram = new QOpenGLShaderProgram;
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    shaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    shaderProgram->bindAttributeLocation("vertexPos", 0);
    shaderProgram->bindAttributeLocation("vertexCol", 1);
    shaderProgram->link();

    // Bind uniform matrix locations
    shaderProgram->bind();
    projMatrixLoc = shaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = shaderProgram->uniformLocation("mvMatrix");
    shaderProgram->release();
}


//  ----- initBuffers()
//  Creates buffers for vertex and color data for each model
//
void ModelViewer::initBuffers() {

    ulong i_m; // Index reference for the vertices arrays in models

    for (uint m = 0; m < uint(models->n_models); m++) {

        i_m = (m == 0)? 0 : ulong(models->model_indices[m - 1]);

        GLuint vertexBuffer = 0;
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, 3 * n_vertices[m] * long(sizeof(GLfloat)), &models->vertices[i_m], GL_STATIC_DRAW);
        vertexBuff_ids[m] = vertexBuffer;

        GLuint colorBuffer = 0;
        glGenBuffers(1, &colorBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
        glBufferData(GL_ARRAY_BUFFER, 4 * n_vertices[m] * long(sizeof(GLfloat)), &colors[i_m*4], GL_STATIC_DRAW);
        colorBuff_ids[m] = colorBuffer;

    }
}


//  ----- updateShader()
//  Updates uniform matrices and set them in the shader
//
void ModelViewer::updateShader() {
    world.setToIdentity();
    world.rotate(m_xRot / 16.0f, 1, 0, 0);
    world.rotate(m_yRot / 16.0f, 0, 1, 0);
    world.rotate(m_zRot / 16.0f, 0, 0, 1);
    world.scale(m_zoom);
    world.translate(-m_center);

    shaderProgram->bind();
    shaderProgram->setUniformValue(projMatrixLoc, projection);
    shaderProgram->setUniformValue(mvMatrixLoc, camera * world);
}


//  ----- updateBuffers()
//  Binds buffers and resyncs underlying data arrays
//
void ModelViewer::updateBuffers() {

    ulong i_m; // Index reference for the vertices arrays in models

    for (uint m = 0; m < uint(models->n_models); m++) {

        if (activateModel[m]) {

            i_m = (m == 0)? 0 : ulong(models->model_indices[m - 1]);

            glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_ids[m]);
            glBufferData(GL_ARRAY_BUFFER, 3 * n_vertices[m] * long(sizeof(GLfloat)), &models->vertices[i_m], GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, colorBuff_ids[m]);
            glBufferData(GL_ARRAY_BUFFER, 4 * n_vertices[m] * long(sizeof(GLfloat)), &colors[i_m*4], GL_STATIC_DRAW);
        }
    }
}


//  ----- drawVertexArray()
//  Bind buffers and draws the vertex array object
//
void ModelViewer::drawVertexArray() {

    for (uint m = 0; m < uint(models->n_models); m++) {

        if (activateModel[m]) {

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_ids[m]);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

            glBindBuffer(GL_ARRAY_BUFFER, colorBuff_ids[m]);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDrawArrays(GL_TRIANGLES, 0, GLsizei(n_vertices[m]));

            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(0);

        }
    }
}


//  ----- cleanUp()
//  Deletes objects and pointers
//
void ModelViewer::cleanUp() {

    if (shaderProgram == nullptr)
        return;

    makeCurrent();

    for (uint m = 0; m < uint(models->n_models); m++)
        glDeleteBuffers(1, &vertexBuff_ids[m]);

    delete vertexBuff_ids;
    delete colorBuff_ids;
    delete colors;
    delete activateModel;
    delete n_vertices;
    delete shaderProgram;
    shaderProgram = nullptr;
    doneCurrent();
}


//  ----- initCamera()
//  Initializes camera matrix
//
void ModelViewer::initCamera() {
    camera.setToIdentity();
}


// --------------------------------------------------------------------
// QT OPENGLWIDGET VIRUTAL FUNCTIONS
// --------------------------------------------------------------------


//  ----- initializeGL()
//  Virtual QOpenGLWidget function
//  Called once on widget construction
//
void ModelViewer::initializeGL() {

    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &ModelViewer::cleanUp);

    initializeOpenGLFunctions();

    updateColors();

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
void ModelViewer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 0);

    vertexArray.bind();
    updateShader();
    updateBuffers();
    drawVertexArray();
}


//  ----- resizeGL()
//  Virtual QOpenGLWidget function
//  Called for window resizing
//
void ModelViewer::resizeGL(int width, int height) {
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, 0.01f, 100.0f);
}


//  ----- mousePressEvent()
//  Overrides listener for mouse presses
//
void ModelViewer::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->pos();
}


//  ----- mouseMoveEvent()
//  Overrides listener for mouse moves to update rotation matrix
//
void ModelViewer::mouseMoveEvent(QMouseEvent *event)
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
void ModelViewer::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? m_zoom += m_zoom*0.1f : m_zoom -= m_zoom*0.1f;
    update();
}

//  ----- rotateBy()
//  Helper function to update rotation parameter and re-paint widget
//
void ModelViewer::rotateBy(int xAngle, int yAngle, int zAngle)
{
    m_xRot += xAngle;
    m_yRot += yAngle;
    m_zRot += zAngle;
    update();
}
