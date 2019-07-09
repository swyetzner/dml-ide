#ifndef MODEL_H
#define MODEL_H

#include "polygon.h"
#include "utils.h"

#include<QColor>
#include<QString>
#include<QUrl>
#include<QVector>
#include<QVector3D>
#include<QVector4D>
#include<glm/glm.hpp>
#include<QOpenGLWidget>
#include<QOpenGLFunctions>
#include<QOpenGLBuffer>
#include<QOpenGLVertexArrayObject>
#include<QOpenGLShaderProgram>
#include<float.h>
#include<QDebug>
using namespace std;

const double EPSILON = 0.0001;

struct Bar {
    Vec left;
    Vec right;
    double diameter;

    Bar(Vec left, Vec right, double diameter) {
        this->left = left;
        this->right = right;
        this->diameter = diameter;
    }
};

template <typename T>
struct boundingBox {
    T minCorner;
    T maxCorner;
    T center;

    boundingBox() {

    }

    boundingBox(vector<T> vertices) {
        minCorner = T(FLT_MAX, FLT_MAX, FLT_MAX);
        maxCorner = T(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (T vertex : vertices) {
            minCorner[0] = std::min(minCorner[0], vertex[0]);
            minCorner[1] = std::min(minCorner[1], vertex[1]);
            minCorner[2] = std::min(minCorner[2], vertex[2]);
            maxCorner[0] = std::max(maxCorner[0], vertex[0]);
            maxCorner[1] = std::max(maxCorner[1], vertex[1]);
            maxCorner[2] = std::max(maxCorner[2], vertex[2]);
        }

        center = (minCorner + maxCorner) * 0.5f;
        qDebug() << "Bounds Min" << minCorner[0] << minCorner[1] << minCorner[2];
        qDebug() << "Bounds Max" << maxCorner[0] << maxCorner[1] << maxCorner[2];
        qDebug() << "Center" << center[0] << center[1] << center[2];
    }

    bool isWithin(T pos) {
        if (pos[0] < minCorner[0] || pos[0] > maxCorner[0]) return false;
        if (pos[1] < minCorner[1] || pos[1] > maxCorner[1]) return  false;
        if (pos[2] < minCorner[2] || pos[2] > maxCorner[2]) return false;
        return true;
    }

    void combine(const boundingBox<T> &other) {
        minCorner[0] = std::min(minCorner[0], other.minCorner[0]);
        minCorner[1] = std::min(minCorner[1], other.minCorner[1]);
        minCorner[2] = std::min(minCorner[2], other.minCorner[2]);
        maxCorner[0] = std::max(maxCorner[0], other.maxCorner[0]);
        maxCorner[1] = std::max(maxCorner[1], other.maxCorner[1]);
        maxCorner[2] = std::max(maxCorner[2], other.maxCorner[2]);
    }
};


/**
 * @brief The model_data struct
 * Stores the data in model arrays where math functions
 * can be easily performed on them.
 */
struct model_data {
    char *id;

    int n_triangles = 0;
    int n_vertices = 0;
    int n_normals = 0;
    int n_indices = 0;
    int n_models = 0;
    int *model_indices;

    vector<glm::vec3> vertices = std::vector<glm::vec3>();
    vector<glm::vec3> normals = std::vector<glm::vec3>();
    vector<uint> indices = vector<uint>();
    vector<glm::vec4> colors = vector<glm::vec4>();

    QVector<float> m_data;
    QVector<float> m_colors;
    QVector3D m_center;
    float m_dim;
    int m_count = 0;
    int m_colCount = 0;

    boundingBox<glm::vec3> bounds;

    const GLfloat *constData() const { return m_data.constData(); }
    const GLuint *constIndicies() const { return indices.data(); }
    const GLfloat *constColor() { return m_colors.constData(); }
    int count() const { return m_count; }
    int vertexCount() const { return m_count / 6; }
    int indexCount() const { return indices.size(); }
    int colorCount() const { return m_colCount; }
    const QVector3D center() { return m_center; }

    // Loads graphics data from Polygon
    void createPolygonGraphicsData(const Polygon &polygon, glm::vec4 color) {
        n_triangles = polygon.triangles->size();
        n_vertices = 3 * n_triangles;
        m_data.resize(n_vertices * 6);

        for (ulong a = 0; a < n_triangles; a++) {
            shared_ptr<Tri> t = polygon.triangles->at(a);

            GLfloat *p = m_data.data() + m_count;
            *p++ = t->v1->p[0];
            *p++ = t->v1->p[1];
            *p++ = t->v1->p[2];
            *p++ = t->n[0];
            *p++ = t->n[1];
            *p++ = t->n[2];
            m_count += 6;

            *p++ = t->v2->p[0];
            *p++ = t->v2->p[1];
            *p++ = t->v2->p[2];
            *p++ = t->n[0];
            *p++ = t->n[1];
            *p++ = t->n[2];
            m_count += 6;

            *p++ = t->v3->p[0];
            *p++ = t->v3->p[1];
            *p++ = t->v3->p[2];
            *p++ = t->n[0];
            *p++ = t->n[1];
            *p++ = t->n[2];
            m_count += 6;
        }

        // Fill colors for each model
        m_colors.resize(n_vertices * 4);
        for (ulong a = 0; a < n_vertices; a++) {
            GLfloat *p = m_colors.data() + m_colCount;
            *p++ = color.r;
            *p++ = color.b;
            *p++ = color.g;
            *p++ = color.a;
            m_colCount += 4;
        }
    }


    /**
     * @brief updateVertices Copies vertex and normal data into graphics array
     */
    void updateVertices() {
        m_data.resize(n_vertices * 6);
        m_count = 0;

        for (ulong a = 0; a < n_vertices; a++) {
            GLfloat *p = m_data.data() + m_count;
            *p++ = vertices.at(a).x;
            *p++ = vertices.at(a).y;
            *p++ = vertices.at(a).z;
            *p++ = normals.at(a).x;
            *p++ = normals.at(a).y;
            *p++ = normals.at(a).z;
            m_count += 6;
        }

        bounds = boundingBox<glm::vec3>(vertices);
        m_center = QVector3D(bounds.center.x, bounds.center.y, bounds.center.z);
        m_dim = (bounds.minCorner - bounds.maxCorner).length();
    }

    /**
     * @brief updateColors Copies color data into graphics array
     */
    void updateColors() {
        // Fill colors for each model
        m_colors.resize(n_vertices * 4);
        m_colCount = 0;

        ulong modelStart = 0;
        for (ulong i = 0; i < n_models; i++) {
            for (ulong a = modelStart; a < model_indices[i]; a++) {
                GLfloat *p = m_colors.data() + m_colCount;
                *p++ = colors.at(i).r;
                *p++ = colors.at(i).b;
                *p++ = colors.at(i).g;
                *p++ = colors.at(i).a;
                m_colCount += 4;
            }
            modelStart = model_indices[i];
        }
    }

    /**
     * @brief createGraphicsData Moves data into QVector form
     */
    void createGraphicsData() {
        m_data.resize(n_vertices * 6);

        for (ulong a = 0; a < n_vertices; a++) {
            GLfloat *p = m_data.data() + m_count;
            *p++ = vertices.at(a).x;
            *p++ = vertices.at(a).y;
            *p++ = vertices.at(a).z;
            *p++ = normals.at(a).x;
            *p++ = normals.at(a).y;
            *p++ = normals.at(a).z;
            m_count += 6;
        }

        // Fill colors for each model
        m_colors.resize(n_vertices * 4);

        ulong modelStart = 0;
        for (ulong i = 0; i < n_models; i++) {
            for (ulong a = modelStart; a < model_indices[i]; a++) {
                GLfloat *p = m_colors.data() + m_colCount;
                *p++ = colors.at(i).r;
                *p++ = colors.at(i).b;
                *p++ = colors.at(i).g;
                *p++ = colors.at(i).a;
                m_colCount += 4;
            }
            modelStart = model_indices[i];
        }

        bounds = boundingBox<vec3>(vertices);
        m_center = QVector3D(bounds.center.x, bounds.center.y, bounds.center.z);
        m_dim = (bounds.maxCorner - bounds.maxCorner).length();
    }

    bool isInside(glm::vec3 point, int n_model) {

        uint modelStart = 0;
        uint modelEnd = 0;
        int intersections = 0;

        if (n_model != 0)
            modelStart = model_indices[n_model-1];
        modelEnd = model_indices[n_model];

        glm::vec3 p;
        float pu, pv;
        glm::vec3 dir = Utils::randDirection();

        for (uint i = modelStart; i < modelEnd; i+=3) {

            bool intersectPlane = Utils::intersectPlane(&vertices[i], point, dir, p, pu, pv);

            //qDebug() << "p " << p.x << p.y << p.z;

            if (intersectPlane && p.x > point.x && p.y > point.y && p.z > point.z) {

                if (Utils::insideTriangle(vertices[i], vertices[i+1], vertices[i+2], p)) {
                    intersections++;
                }
            }
        }

        //qDebug() << intersections << "intersections";

        if (intersections % 2 == 1)
            return true;
        else
            return false;
    }

    // Check if a point is too close to the hull of a model
    bool isCloseToEdge(glm::vec3 point, float cutoff, int n_model) {

        uint modelStart = 0;
        uint modelEnd = 0;
        if (n_model != 0)
            modelStart = model_indices[n_model-1];
        modelEnd = model_indices[0];

        for (uint i = modelStart; i < modelEnd; i ++) {
            if (length(vertices[i] - point) <= cutoff)
                return true;
        }

        return false;
    }

    void indexVertices(int n_model) {
        vector<glm::vec3> v_collapse = vector<glm::vec3>();
        vector<glm::vec3> n_collapse = vector<glm::vec3>();
        vector<uint> i_model = vector<uint>();
        map<uint, vector<uint>> vi_map = map<uint, vector<uint>>();
        map<uint, vector<glm::vec3>> n_map = map<uint, vector<glm::vec3>>();

        uint modelStart = 0;
        uint modelEnd = 0;

        if (n_model != 0)
            modelStart = model_indices[n_model-1];
        modelEnd = model_indices[n_model];

        for (uint i = modelStart; i < modelEnd; i++) {

            bool foundMatch = false;

            for (uint j = 0; j < v_collapse.size(); j++) {
                if (vertices[i] == v_collapse[j]) {
                    // Vertex already exists
                    foundMatch = true;

                    // Add index
                    i_model.push_back(j);
                    vi_map[j].push_back(i);

                    // Update normal map
                    if (n_map.find(j) == n_map.end()) {
                        // Doesn't exist in map; add new entry
                        n_map[j] = vector<glm::vec3>(1, normals[i]);

                        // Original vertex normal
                        uint oi = vi_map[j].front();
                        n_map[j].push_back(normals[oi]);
                    } else {
                        // Exists in map
                        n_map[j].push_back(normals[i]);
                    }
                    break;
                }
            }

            if (!foundMatch) {
                // Vertex does not already exist
                v_collapse.push_back(vertices[i]);

                // Add index
                i_model.push_back(v_collapse.size()-1);
                vi_map[v_collapse.size()-1].push_back(i);
            }
        }

        // Resolve normals
        n_collapse = vector<glm::vec3>(v_collapse.size());

        for (uint j = 0;  j < v_collapse.size(); j++) {

            // Average normals for single vertex
            if (n_map.find(j) != n_map.end()) {
                // Mutiple normals to average for vertex

                glm::vec3 n_average = glm::vec3(0.0f, 0.0f, 0.0f);

                for (uint k = 0; k < n_map[j].size(); k++) {
                    n_average += n_map[j][k];
                }

                n_collapse.at(j) = (1.0f / n_map[j].size()) * n_average;
            } else {
                // One normal, no average
                n_collapse.at(j) = normals.at(vi_map[j].front());
            }

        }

        vertices.erase(vertices.begin() + modelStart, vertices.begin() + modelEnd);
        vertices.insert(vertices.begin() + modelStart, v_collapse.begin(), v_collapse.end());
        model_indices[n_model] = modelStart + v_collapse.size();
        n_vertices -= ((modelEnd - modelStart) - v_collapse.size());

        normals.erase(normals.begin() + modelStart, normals.begin() + modelEnd);
        normals.insert(normals.begin() + modelStart, n_collapse.begin(), n_collapse.end());
        n_normals = ((modelEnd - modelStart) - n_collapse.size());

        indices.insert(indices.end(), i_model.begin(), i_model.end());
        qDebug() << "Collapsed vertices: " << v_collapse.size();
    }

    ~model_data() {
        std::vector<glm::vec3>().swap(vertices);
        std::vector<glm::vec3>().swap(normals);
    }
};

/**
 * @brief The simulation_data struct
 * Stores the data in a format that can
 * easily converted to a Titan simulation
 */
struct simulation_data {
    char *id;

    int n_vertices = 0;
    int n_indices = 0;

    vector<glm::vec3> vertices = std::vector<glm::vec3>();
    vector<uint> indices = vector<uint>();
    vector<glm::vec3> lattice = std::vector<glm::vec3>();
    vector<uint> hull = std::vector<uint>();
    // vec3 *d_vertices; //CUDA memory

    QVector<float> m_data;
    QVector<uint> m_index;
    QVector<float> m_colors;
    QVector3D m_center;
    float m_dim;
    int m_count = 0;
    int m_indexCount = 0;

    boundingBox<vec3> bounds;

    const GLfloat *constData() const { return m_data.constData(); }
    const GLuint *constIndices() const { return m_index.constData(); }
    int count() const { return m_count; }
    int vertexCount() const { return m_count / 3; }
    int indexCount() const { return m_indexCount; }

    void copyToGPU();

    void createGraphicsData() {
        m_data.resize(n_vertices * 3);

        for (ulong a = 0; a < n_vertices; a++) {
            GLfloat *p = m_data.data() + m_count;
            *p++ = vertices.at(a).x;
            *p++ = vertices.at(a).y;
            *p++ = vertices.at(a).z;
            m_count += 3;
        }

        m_index.resize(n_indices);

        for (ulong b = 0; b < n_indices; b++) {
            GLuint *p = m_index.data() + m_indexCount;
            *p++ = indices.at(b);
            m_indexCount++;
        }
    }

    void indexVertices(int n_model) {
        qDebug() << "Indexing vertices for model " << n_model;
        vector<glm::vec3> v_collapse = vector<glm::vec3>();
        vector<uint> i_model = vector<uint>();

        uint modelStart = 0;
        uint modelEnd = n_vertices;

        qDebug() << "Start index: " << modelStart << " End index: " << modelEnd;

        for (uint i = modelStart; i < modelEnd; i++) {

            bool foundMatch = false;

            for (uint j = 0; j < v_collapse.size(); j++) {
                if (vertices[i] == v_collapse[j]) {
                    // Vertex already exists
                    foundMatch = true;

                    // Add index
                    i_model.push_back(j);

                    break;
                }
            }

            if (!foundMatch) {
                // Vertex does not already exist
                v_collapse.push_back(vertices[i]);

                // Add index
                i_model.push_back(v_collapse.size()-1);
            }
        }


        vertices.erase(vertices.begin() + modelStart, vertices.begin() + modelEnd);
        vertices.insert(vertices.begin() + modelStart, v_collapse.begin(), v_collapse.end());

        ulong diffVertexCount = (modelEnd - modelStart) - v_collapse.size();

        n_vertices -= diffVertexCount;

        indices.insert(indices.end(), i_model.begin(), i_model.end());
        n_indices = indices.size();
        qDebug() << "Collapsed vertices: " << v_collapse.size();
        qDebug() << "New vertex array size: " << n_vertices;
    }


    // Cuda version of below function
    bool isInsideCuda(glm::vec3 point, int n_model);

    //
    // Returns true if point is inside model n
    //
    bool isInside(glm::vec3 point) {

        uint modelStart = 0;
        uint modelEnd = n_vertices;
        int intersections = 0;

        glm::vec3 p;
        float pu, pv;
        glm::vec3 dir = Utils::randDirection();

        for (uint i = modelStart; i < modelEnd; i+=3) {

            bool intersectPlane = Utils::intersectPlane(&vertices[i], point, dir, p, pu, pv);

            //qDebug() << "p " << p.x << p.y << p.z;

            if (intersectPlane && p.x > point.x && p.y > point.y && p.z > point.z) {

                if (Utils::insideTriangle(vertices[i], vertices[i+1], vertices[i+2], p)) {
                    intersections++;
                }
            }
        }

        //qDebug() << intersections << "intersections";

        if (intersections % 2 == 1)
            return true;
        else
            return false;
    }

    // Check if a point is too close to the hull of a model
    bool isCloseToEdge(glm::vec3 point, float cutoff) {

        uint modelStart = 0;
        uint modelEnd = n_vertices;

        for (uint i = modelStart; i < modelEnd; i ++) {
            if (length(vertices[i] - point) <= cutoff)
                return true;
        }

        return false;
    }


    simulation_data(model_data *arrays) {
        this->n_vertices = arrays->n_vertices;
        this->vertices = arrays->vertices;

        this->bounds = boundingBox<vec3>(vertices);

        // Copy vertices to CUDA memory
        /**cudaMallocManaged(&d_vertices, vertices.size()*sizeof(vec3));
        for (int i = 0; i < vertices.size(); i++) {
            d_vertices[i] = vertices[i];
        }

        qDebug() << "Simulation vertices: " << n_vertices;
        qDebug() << "Simulation indices: " << n_indices;
        qDebug() << "CUDA vertex:" << d_vertices[25].x;**/
    }

    simulation_data(Polygon *polygon) {
        this->n_vertices = polygon->nodeMap.size();
        qDebug() << "Set variables";

        for (const auto n : polygon->nodeMap) {
            vec3 v = vec3(n.first[0],n.first[1],n.first[2]);
            this->vertices.push_back(v);
        }
        qDebug() << "Copied vertices";

        this->bounds = boundingBox<vec3>(vertices);
    }

    ~simulation_data() {
        std::vector<glm::vec3>().swap(vertices);
        //cudaFree(d_vertices);
    }
};

struct bar_data {

    vector<Bar> bars;
    boundingBox<Vec> bounds;
    boundingBox<Vec> *anchorShell;
    Vec *forceSphere;
    double boxCutoff;

    bar_data() {
        bars = vector<Bar>();
        anchorShell = nullptr;
        forceSphere = nullptr;
        boxCutoff = 0;
    }

    void addBar(const Vec &left, const Vec &right, const double &diameter) {
        bars.emplace_back(Bar(left, right, diameter));
        qDebug() << "BAR" << left[0] << left[1] << left[2] << right[0] << right[1] << right[2];
    }

    void setRadii(const double &radius) {
        for (Bar &b : bars) {
            b.diameter = radius * 2;
        }
    }
};

/**
 * @brief The Volume class
 */
class Volume
{
public:
    Volume();
    Volume(QString id);
    Volume(QString id,
           QString primitive,
           QString url,
           QString units,
           QString rendering,
           QString alpha,
           QString color);

    QString id;
    QString primitive;
    QUrl url;
    QString units;
    QString rendering;
    QVector4D color;

    ulong index;

    model_data *model = nullptr;
    Polygon *geometry = nullptr;

    void loadModel();

    void updateColor() {
        if (model == nullptr) {
            model = new model_data();
        }
        vec4 cvec = vec4(color.x(), color.y(), color.z(), color.w());
        model->colors.insert(model->colors.begin(), cvec);
    }

signals:
    void log(const QString message);
};


class Material
{
public:
    Material() {}
    ~Material() {}

    QString id;
    QString name;
    double elasticity;
    QString eUnits;
    double yield;
    QString yUnits;
    double density;
    QString dUnits;

    ulong index;
};


class Anchor
{
public:
    Anchor() {}
    ~Anchor() {}

    Volume * volume;
};


class Force
{
public:
    Force() {}
    ~Force() {}

    Volume * volume;
    Vec magnitude;
    double duration;
};

class Loadcase
{
public:
    Loadcase() {}
    ~Loadcase() {}

    QString id;

    vector<Anchor> anchors;
    map<QString, Anchor *> anchorMap;

    vector<Force> forces;
    map<QString, Force *> forceMap;

    ulong index;
};


class LatticeConfig
{
public:
    LatticeConfig() {}
    ~LatticeConfig() {}

    enum LatticeFill { CUBIC_FILL, SPACE_FILL };

    LatticeFill fill;
    Vec unit;
    QString display;
    bool conform;
    Vec offset;
    Vec barDiameter;
    Material *material;
    Vec jiggle;
    bool hull;

    QString fillName() {
        switch(fill) {
        case CUBIC_FILL:
            return "cubic";
        case SPACE_FILL:
            return "space";
        }
    }

    vector<Vec> vertices;
};

class Damping
{
public:
    Damping() {}
    ~Damping() {}

    double velocity;
};

class Global
{
public:
    Global() = default;
    ~Global() = default;

    Vec acceleration;
};

class Stop
{
public:
    Stop()  = default;
    ~Stop() = default;

    enum StopCriterion { SC_TIME, SC_MOTION };

    StopCriterion criterion;
    double threshold;

    QString criterionName() {
        switch(criterion) {
        case SC_TIME:
            return "time";
        case SC_MOTION:
            return "motion";
        }
    }
};

class Repeat
{
public:

    double after = -1;
    bool rotationExplicit = true;
    Vec rotation = Vec(0, 0, 0);
};

class Plane
{
public:

    Plane() = default;
    ~Plane() = default;

    Vec normal = Vec(0, 1, 0);
    double offset = 0;

};

class SimulationConfig
{
public:
    SimulationConfig() {}
    ~SimulationConfig() {}

    QString id;
    Volume * volume;
    LatticeConfig lattice;
    Damping damping;
    Global global;
    Repeat repeat;
    Plane * plane = nullptr;
    Loadcase * load;
    vector<Stop> stops;

    ulong index;

    simulation_data *model = nullptr;
};

class OptimizationStop {
public:
    OptimizationStop() = default;
    ~OptimizationStop() = default;

    enum Metric { WEIGHT, ENERGY };

    Metric metric;
    double threshold;

    QString metricName() {
        switch(metric) {
            case WEIGHT:
                return "WEIGHT";
            case ENERGY:
                return "ENERGY";
        }
    }
};

class OptimizationRule {
public:
    OptimizationRule() = default;
    ~OptimizationRule() = default;

    enum Method { REMOVE_LOW_STRESS, MASS_DISPLACE, NONE };

    Method method;
    double threshold;
    int frequency;

    QString methodName() {
        switch (method) {
            case REMOVE_LOW_STRESS:
                return "REMOVE_LOW_STRESS";
            case MASS_DISPLACE:
                return "MASS_DISPLACE";
            case NONE:
                return "NONE";
        }
    }
};

class OptimizationConstraint {
public:
    OptimizationConstraint() = default;
    ~OptimizationConstraint() = default;

    enum Metric { VOLUME };
    Metric metric;
    QString id;

    QString metricName() {
        switch (metric) {
            case VOLUME:
                return "VOLUME";
        }
    }
};

class OptimizationConfig
{
public:
    OptimizationConfig() {}
    ~OptimizationConfig() {}

    SimulationConfig * simulationConfig;
    vector<OptimizationRule> rules;
    vector<OptimizationStop> stopCriteria;
};

// Hold output data
struct output_data {
    QString id;
    bar_data * barData; // Simulation output
    vector<Volume *> includes; // Volumes to include in final model
    vector<Volume *> excludes; // Volumes to exclude from final model

    SimulationConfig * sim;
};

/**
 * @brief The Design class holds the entire design model
 */
class Design
{
public:
    Design();
    ~Design() {
        delete optConfig;
    }

    vector<Volume *> volumes;
    map<QString, Volume *> volumeMap;

    vector<Material> materials;
    map<QString, Material *> materialMap;

    vector<Loadcase> loadcases;
    map<QString, Loadcase *> loadcaseMap;

    vector<SimulationConfig> simConfigs;
    map<QString, SimulationConfig *> simConfigMap;

    OptimizationConfig * optConfig = nullptr;

    vector<output_data *> outputs;
    map<QString, output_data *> outputMap;
};




#endif // MODEL_H
