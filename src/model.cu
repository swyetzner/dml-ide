#include "model.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_device_runtime_api.h>

// CUDA ERROR CHECK WRAPPER
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=false)
{
    if (code != cudaSuccess)
    {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);

        if (abort) {
            char buffer[200];
            snprintf(buffer, sizeof(buffer), "GPUassert error in CUDA kernel: %s %s %d\n", cudaGetErrorString(code), file, line);
            std::string buffer_string = buffer;
            throw std::runtime_error(buffer_string);
        }
    }
}

/**__device__ bool insideTriangle(vec3 a, vec3 b, vec3 c, vec3 &point) {
    vec3 ca = c - a;
    vec3 ba = b - a;
    vec3 pa = point - a;

    float dotCACA = dot(ca, ca);
    float dotCABA = dot(ca, ba);
    float dotCAPA = dot(ca, pa);
    float dotBABA = dot(ba, ba);
    float dotBAPA = dot(ba, pa);

    float invDenom = 1.0f / (dotCACA * dotBABA - dotCABA * dotCABA);
    float u = (dotBABA * dotCAPA - dotCABA * dotBAPA) * invDenom;
    float v = (dotCACA * dotBAPA - dotCABA * dotCAPA) * invDenom;

    return (u >= 0.0f) && (v >= 0.0f) && (u + v < 1.0f);
}

__global__ void intersectPlane(int n_triangles, vec3 *model, vec3 point, vec3 dir, bool *d_intersects) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n_triangles) {

        vec3 &v1 = model[3*idx];
        vec3 &v2 = model[3*idx + 1];
        vec3 &v3 = model[3*idx + 2];
        bool &ret = d_intersects[idx];

        vec3 n = cross((v2 - v1), (v3 - v2));
        n = normalize(n);

        float dn = dot(dir, n);

        if (fabs(dn) < EPSILON) {
            ret = false;
        }

        float dist = -dot((point-v1), n)/dn;
        vec3 p = point + dir*dist;

        ret = insideTriangle(v1, v2, v3, p);
    }
}


bool simulation_data::isInsideCuda(glm::vec3 point, int n_model) {

    uint modelStart = 0;
    uint modelEnd = 0;
    int intersections = 0;

    if (n_model != 0)
        modelStart = model_indices[n_model-1];
    modelEnd = model_indices[n_model];

    glm::vec3 dir = Utils::randDirection();

    int n_tri = (modelEnd - modelStart) / 3;
    vec3 *d_vertices;
    vec3 *p;
    bool *intersects;

    //qDebug() << "Allocating for N triangles:" << n_tri;
    //qDebug() << "Mode start end" << modelStart << modelEnd;

    // Allocate cuda memory for results
    cudaMallocManaged(&intersects, n_tri*sizeof(bool));

    int blockSize = 256;
    int numBlocks = (n_tri + blockSize - 1) / blockSize;
    intersectPlane<<<numBlocks, blockSize>>>(n_tri, this->d_vertices, point, dir, intersects);
    gpuErrchk(cudaPeekAtLastError());

    cudaDeviceSynchronize();
    // Add up intersections after CUDA is done
    for (int i = 0; i < n_tri; i++) {
        if (intersects[i]) intersections++;
    }

    // Free memory
    cudaFree(intersects);

    if (intersections % 2 == 1) {
        qDebug() << "FOUND INSIDE POINT";
        return true;
    } else
        return false;
}**/

void simulation_data::copyToGPU() {
    //float *h_vertices = new float[vertices.size() * 3];
    //int count = 0;
    //for (auto v : vertices) {
    //    h_vertices[count++] = v.x;
    //    h_vertices[count++] = v.y;
    //    h_vertices[count++] = v.z;
    //}

    float *CUDA_VERTICES;

    gpuErrchk(cudaMalloc(&CUDA_VERTICES, sizeof(float)));
    gpuErrchk(cudaMemcpy(CUDA_VERTICES, vertices.data(), sizeof(float), cudaMemcpyHostToDevice));

    gpuErrchk(cudaFree(CUDA_VERTICES));
}

/**
 * @brief Volume::Volume default constructor
 */
Volume::Volume() {

}

Volume::Volume(QString s_id) {
    id = s_id;
}

/**
 * @brief Volume::Volume full constructor
 * @param s_id ID
 * @param s_primitive Primitive type (e.g stl, obj, ...)
 * @param s_url File path for primitive mesh
 * @param s_units Units (e.g. mm)
 * @param s_rendering Rendering options
 * @param s_alpha Alpha value in [0.0,1.0]
 * @param s_color RGB string "r g b" where r,g,b in [0.0,1.0]
 */
Volume::Volume(QString s_id,
        QString s_primitive,
        QString s_url,
        QString s_units,
        QString s_rendering,
        QString s_alpha,
        QString s_color) {

    id = s_id;
    primitive = s_primitive;
    rendering = s_rendering;
    units = s_units;
    color = QVector4D(1.0f, 1.0f, 1.0f, 1.0f);

    url = QUrl(s_url);

    if (s_color != nullptr) {
        QList<QString> s_colors = s_color.split(" ");
        QList<float> f_colors;
        for (auto c : s_colors) {
            float f_c = c.toFloat();
            f_colors.append(f_c);
        }
        color.setX(f_colors.at(0));
        color.setY(f_colors.at(1));
        color.setZ(f_colors.at(2));
    }

    if (s_alpha != nullptr) {
        float f_alpha = s_alpha.toFloat();
        color.setW(f_alpha);
    }

    model = new model_data();
}

/**
 * @brief Design::Design default constructor
 */
Design::Design() {
    volumes = std::vector<Volume *>();
}
