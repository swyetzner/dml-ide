#include "designViewer.h"

DesignViewer::DesignViewer(Design *design, QWidget *parent)
    : QOpenGLWidget (parent),
      shaderProgram(nullptr),
      guideShaderProgram(nullptr),
      depthShaderProgram(nullptr),
      depthQuadProgram(nullptr),
      frameWidth(2048),
      frameHeight(2048),
      nearPlane(0.1f),
      farPlane(7.5f),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_zoom(1.0f),
      m_mode(R_SOLID)
{

    this->design = design;
    qDebug() << "Design Assigned"; // For some reason, this prevents the IDE from crashing when loading a file
    // Set model constants
    n_models = long(design->volumes.size());
    n_modelVertices = new int[n_models];

    for (uint i = 0; i < uint(n_models); i++) {
        n_modelVertices[i] = design->volumes[i]->model->n_vertices;
    }

    // Set activate models to true
    activateModel = new bool[n_models];
    for (uint i = 0; i < n_models; i++)
        activateModel[i] = true;

    // Allocate buffer id for each model
    vertexBuff_ids = new GLuint[uint(n_models)];
    normalBuff_ids = new GLuint[uint(n_models)];
    colorBuff_ids = new GLuint[uint(n_models)];

    // Set rotation + zoom constants
    m_center = QVector3D(0, 0, 0);
    for (uint i = 0; i < uint(n_models); i++)
        m_center += design->volumes[i]->model->center();
    m_center /= n_models; // Use average model center

    getBoundingBox();
    m_dim = (m_minCorner - m_maxCorner).length();
    if (m_dim != 0.0f) m_zoom = 0.4f / m_dim;

    qDebug() << "Initialized DesignViewer";
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
        "out vec4 fragPosLightSpace;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "uniform mat3 normalMatrix;\n"
        "uniform mat4 lightSpaceMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexPosition = vertexPos;\n"
        "   vertexNormal = normalMatrix * vertexNorm;\n"
        "   vertexColor = vertexCol;\n"
        "   fragPosLightSpace = lightSpaceMatrix * vec4(vertexPos, 1.0);\n"
        "}\n";

static const char *fragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec3 vertexPosition;\n"
        "in vec3 vertexNormal;\n"
        "in vec4 vertexColor;\n"
        "in vec4 fragPosLightSpace;\n"
        "uniform sampler2D shadowMap;\n"
        "uniform vec3 lightPos;\n"
        "uniform vec3 viewPos;\n"
        "float shadowCalc(vec4 posLight, float normalLight) {\n"
        "   vec3 projCoords = posLight.xyz / posLight.w;\n"
        "   projCoords = projCoords * 0.5 + 0.5;\n"
        "   float closestDepth = texture(shadowMap, projCoords.xy).r;\n"
        "   float currentDepth = projCoords.z;\n"
        "   float bias = max(0.05 * (1.0 - normalLight), 0.005);\n"
        "   float shadow = 0.0;\n"
        "   vec2 texelSize = 1.0 / textureSize(shadowMap, 0);\n"
        "   for (int x = -1; x <= 1; ++x) {\n"
        "       for (int y = -1; y <= 1; ++y) {\n"
        "           float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;\n"
        "           shadow += currentDepth - bias > pcfDepth? 1.0 : 0.0;\n"
        "       }\n"
        "   }\n"
        "   return shadow / 9.0;\n"
        "}\n"
        "void main() {\n"
        "   vec3 L = normalize(lightPos - vertexPosition);\n"
        "   float NL = max(dot(normalize(vertexNormal), L), 0.0);\n"
        "   vec3 color = vertexColor.rgb;\n"
        "   float shadow = shadowCalc(fragPosLightSpace, NL);\n"
        "   vec3 clColor = clamp(color * 0.2 + color * 0.8 * NL * (1.0 - shadow), 0.0, 1.0);\n"
        "   fragColor = vec4(clColor, vertexColor.a);\n"
        "}\n";
static const char *depthQuadVertSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 1) in vec2 texCoords;\n"
        "out vec2 textureCoords;\n"
        ""
        "void main() {\n"
        "   textureCoords = texCoords;\n"
        "   gl_Position = vec4(vertexPos, 1.0);\n"
        "}\n";
static const char *depthQuadFragSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec2 textureCoords;\n"
        "uniform sampler2D depthMap;\n"
        "uniform float near_plane;\n"
        "uniform float far_plane;\n"
        ""
        "float LinearizeDepth(float depth) {\n"
        "   float z = depth * 2.0 - 1.0;\n"
        "   return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));\n"
        "}\n\n"
        "void main() {\n"
        "   float depthValue = texture(depthMap, textureCoords).r;\n"
        "   fragColor = vec4(vec3(depthValue), 1.0);\n"
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

    // Depth shader
    depthShaderProgram = new QOpenGLShaderProgram;
    depthShaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":resources/shaders/depth.vert");
    depthShaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":resources/shaders/depth.frag");
    depthShaderProgram->bindAttributeLocation("vertexPos", 0);
    depthShaderProgram->link();

    depthQuadProgram = new QOpenGLShaderProgram;
    depthQuadProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, depthQuadVertSource);
    depthQuadProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, depthQuadFragSource);
    depthQuadProgram->bindAttributeLocation("vertexPos", 0);
    depthQuadProgram->bindAttributeLocation("texCoords", 1);
    depthQuadProgram->link();

    // Bind uniform value locations
    shaderProgram->bind();
    projMatrixLoc = shaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = shaderProgram->uniformLocation("mvMatrix");
    normalMatrixLoc = shaderProgram->uniformLocation("normalMatrix");
    lightPosLoc = shaderProgram->uniformLocation("lightPos");

    // Set light position
    shaderProgram->setUniformValue(lightPosLoc, 2, 2, 2);
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

    GUtils::initShadowBuffers(depthFrameBuff_id, depthTexBuff_id, 2048, 2048);


    float quadVertices[] = {
            // positions        // texture Coords
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
            1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
    };
    GLuint quadBuffer;
    glGenBuffers(1, &quadBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, quadBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    quadBuff_id = quadBuffer;
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

    QVector3D lightPos = QVector3D(2,2,2);
    QMatrix4x4 lightProj;
    lightProj.ortho(-10, 10, -10, 10, nearPlane, farPlane);
    QMatrix4x4 lightView;
    lightView.lookAt(lightPos, QVector3D(0, 0, 0), QVector3D(0, 1, 0));
    QMatrix4x4 lightSpaceMatrix;
    lightSpaceMatrix = lightProj * lightView * world;
    QMatrix4x4 scaleBias;
    scaleBias = QMatrix4x4(0.5f, 0.0f, 0.0f, 0.0f,
                           0.0f, 0.5f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.5f, 0.0f,
                           0.5f, 0.5f, 0.5f, 1.0f);
    QMatrix4x4 shadowMatrix;
    shadowMatrix = scaleBias * lightSpaceMatrix;


    shaderProgram->bind();
    shaderProgram->setUniformValue(projMatrixLoc, projection);
    shaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    shaderProgram->setUniformValue(normalMatrixLoc, normals);
    shaderProgram->setUniformValue("shadowMap", 0);
    shaderProgram->setUniformValue("viewPos", eye);
    shaderProgram->setUniformValue("lightSpaceMatrix", lightSpaceMatrix);
    shaderProgram->setUniformValue("lightPos", lightPos);
    shaderProgram->release();

    guideShaderProgram->bind();
    guideShaderProgram->setUniformValue(guideProjMatrixLoc, projection);
    guideShaderProgram->setUniformValue(guideMVProjMatrixLoc, camera * world);
    guideShaderProgram->release();

    depthShaderProgram->bind();
    depthShaderProgram->setUniformValue("lightSpaceMatrix", lightSpaceMatrix);
    depthShaderProgram->release();

    depthQuadProgram->bind();
    depthQuadProgram->setUniformValue("depthMap", 0);
    depthQuadProgram->setUniformValue("near_plane", nearPlane);
    depthQuadProgram->setUniformValue("far_plane", farPlane);
    depthQuadProgram->release();
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

    // DRAW SHADOW MAP
    depthShaderProgram->bind();
    glViewport(0, 0, 2048, 2048);
    glBindFramebuffer(GL_FRAMEBUFFER, depthFrameBuff_id);
    glClear(GL_DEPTH_BUFFER_BIT);


    glEnableVertexAttribArray(0);
    glCullFace(GL_FRONT);
    drawModels();
    glCullFace(GL_BACK);
    glDisableVertexAttribArray(0);

    depthShaderProgram->release();
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());

    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);


    // DRAW MODELS
    glViewport(0, 0, frameWidth, frameHeight);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    shaderProgram->bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthTexBuff_id);


    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    drawModels();

    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    shaderProgram->release();

    // DRAW DEBUGGER
    depthQuadProgram->bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthTexBuff_id);
    glBindBuffer(GL_ARRAY_BUFFER, quadBuff_id);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    //glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);
    depthQuadProgram->release();

    // DRAW AXES
    guideShaderProgram->bind();
    GUtils::drawMainAxes(3, axesBuff_id);
    guideShaderProgram->release();
}

//  ----- drawModels
//  Calls draw commands after vertex attributes haves been bound
//
void DesignViewer::drawModels() {
    for (uint m = 0; m < n_models; m++) {

        if (activateModel[m]) {

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

        }
    }
}


//  ----- cleanUp()
//  Deletes objects and pointers
//
void DesignViewer::cleanUp() {

    qDebug() << "In cleanup";
    if (shaderProgram == nullptr || guideShaderProgram == nullptr)
        return;

    makeCurrent();

    for (uint m = 0; m < uint(n_models); m++) {
        glDeleteBuffers(1, &vertexBuff_ids[m]);
        glDeleteBuffers(1, &colorBuff_ids[m]);
    }
    glDeleteBuffers(1, &axesBuff_id);
    glDeleteFramebuffers(1, &depthFrameBuff_id);

    delete [] vertexBuff_ids;
    delete [] normalBuff_ids;
    delete [] colorBuff_ids;
    delete [] activateModel;
    delete [] n_modelVertices;
    delete shaderProgram;
    delete guideShaderProgram;
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

    qDebug() << "In initialize";
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &DesignViewer::cleanUp);

    initializeOpenGLFunctions();

    initShader();
    initVertexArray();
    initBuffers();

    camera.setToIdentity();
    camera.translate(0, 0, -1);
    eye = QVector3D(0, 0, 1);
    camera.lookAt(eye, QVector3D(0, 0, 0), QVector3D(0, 1, 0));

    qDebug() << "Initialized GL";
}


//  ----- paintGL()
//  Virtual QOpenGLWidget function
//  Called every frame update
//
void DesignViewer::paintGL() {
    qDebug() << "In paintGL";
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

    qDebug() << "Paint GL";
}


//  ----- resizeGL()
//  Virtual QOpenGLWidget function
//  Called for window resizing
//
void DesignViewer::resizeGL(int width, int height) {
    qDebug() << "In resizeGL";
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, nearPlane, farPlane);
    frameWidth = width;
    frameHeight = height;
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

