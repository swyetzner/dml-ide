#include "simulatorParallel.h"


namespace cg = cooperative_groups;


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

// --------------------------------------------------------------------
//  PARELLEL FUNCTIONS
// --------------------------------------------------------------------

//  Shared memory utility struct to avoid extern linker errors
//  See https://docs.nvidia.com/cuda/cuda-samples/index.html#cuda-parallel-reduction
// --------------------------------------------------------------------
template<class T> struct SharedMemory {
// --------------------------------------------------------------------
    __device__ inline operator T *() {
        extern __shared__ int __smem[];
        return (T *)__smem;
    }

    __device__ inline operator const T *() const {
        extern __shared__ int __smem[];
        return (T *)__smem;
    }
};

// Binary operator to compare two springs by stress
//__host__ __device__ CUDA_SPRING * springPtr_minStressOperator(CUDA_SPRING * &lhs, CUDA_SPRING * &rhs) {
//    return lhs->_max_stress < rhs->_max_stress ? lhs : rhs;
//}

// Reduction kernel templates with float data type and max operator
// See https://docs.nvidia.com/cuda/cuda-samples/index.html#cuda-parallel-reduction
// and Wilt, Nicolas "The Cuda Handbook" Listing 12.3
/**template<unsigned int blockSize>
__device__ void maxF_reduceLogStep(float *out, volatile float *partials) {
    const int tid = threadIdx.x;
    if (blockSize >= 1024) {
        if (blockSize < 512) {
            partials[tid] = std::max(partials[tid], partials[tid + 512]);
        }
        __syncThreads();
    }
    if (blockSize >= 512) {
        if (blockSize < 256) {
            partials[tid] = std::max(partials[tid], partials[tid + 256]);
        }
        __syncThreads();
    }
    if (blockSize >= 256) {
        if (blockSize < 128) {
            partials[tid] = std::max(partials[tid], partials[tid + 128]);
        }
        __syncThreads();
    }
    if (blockSize >= 128) {
        if (blockSize < 64) {
            partials[tid] = std::max(partials[tid], partials[tid + 64]);
        }
        __syncThreads();
    }

    if (tid < 32) {
        if (blockSize >= 64) partials[tid] = std::max(partials[tid], partials[tid + 32]);
        if (blockSize >= 32) partials[tid] = std::max(partials[tid], partials[tid + 16]);
        if (blockSize >= 16) partials[tid] = std::max(partials[tid], partials[tid + 8]);
        if (blockSize >= 8 ) partials[tid] = std::max(partials[tid], partials[tid + 4]);
        if (blockSize >= 4 ) partials[tid] = std::max(partials[tid], partials[tid + 2]);
        if (blockSize >= 2 ) partials[tid] = std::max(partials[tid], partials[tid + 1]);
        if (tid == 0) {
            *out = partials[0];
        }
    }
}**/

/**template<unsigned int blockSize>
__device__ void maxF_reducePass(float *out, float *partial, const float *in, unsigned int n) {

    extern __shared__ float sdata[];
    unsigned int tid = threadIdx.x;
    float maxF = 0;

    for (size_t i = blockIdx.x*blockSize + tid; i < n; i += blockSize*gridDim.x) {
        maxF = std::max(maxF, in[i]);
    }
    sdata[tid] = maxF;
    __syncThreads();

    if (gridDim.x == 1) {
        maxF_reduceLogStep<blockSize>(&out[blockIdx.x], sdata);
        return;
    }
    maxF_reduceLogStep<blockSize>(&partial[blockIdx.x], sdata);

    __shared__ bool lastBlock;
    __threadFence();

    if (tid == 0) {
        unsigned int ticket = atomicAdd(&threadsDone, 1);
        lastBlock = (ticket == gridDim.x-1);
    }
    __syncThreads();

    if (lastBlock) {
        float lmaxF = 0;
        for (size_t i = tid; i < gridDim.x; i += blockSize) {
            lmaxF = std::max(lmaxF, partial[i];
        }
        sdata[threadIdx.x] = lmaxF;
        __syncThreads();
        maxF_reduceLogStep<blockSize>(out, partial);
        threadsDone = 0;
    }
}**/

// See https://docs.nvidia.com/cuda/cuda-samples/index.html#cuda-parallel-reduction
__global__ void warpReduce3MinSpring(CUDA_SPRING ** d_min_stress_out, CUDA_SPRING ** d_spring, int num_springs) {

    // Thread block group
    cg::thread_block cta = cg::this_thread_block();
    CUDA_SPRING ** s_spring = SharedMemory<CUDA_SPRING *>();

    // First level of reduction
    unsigned int tid = threadIdx.x;
    unsigned int i = blockIdx.x*(blockDim.x*2) + threadIdx.x;

    CUDA_SPRING * result = (i < num_springs) ? d_spring[i] : nullptr;

    if (i + blockDim.x < num_springs) {
        result = result->_max_stress < d_spring[i+blockDim.x]->_max_stress ? result : d_spring[i+blockDim.x];
    }
    s_spring[tid] = result;
    cg::sync(cta);

    // Shared Memory Reduction
    for (unsigned int s = blockDim.x/2; s > 0; s>>=1) {
        if (tid < s) {
            s_spring[tid] = result = result->_max_stress < s_spring[tid + s]->_max_stress ? result : s_spring[tid + s];
        }
        cg::sync(cta);
    }

    if (tid == 0) d_min_stress_out[blockIdx.x] = result;
}


// CONSTRUCTOR
// --------------------------------------------------------------------
SimulatorParallel::SimulatorParallel(Simulation *sim, SimulationConfig *config)
// --------------------------------------------------------------------
{
    this->sim = sim;
    this->config = config;

    status = NOT_RUNNING;

    updates = 0;
    //sdkCreateTimer(&timer);
    //gpuErrchk(cudaEventCreate(&startEvent));
    //gpuErrchk(cudaEventCreate(&stopEvent));
    //gpuErrchk(cudaEventCreate(&hostMemSyncEvent));
    removeLeastStressedBar();
}

SimulatorParallel::~SimulatorParallel()
{
    unregisterGraphicsResources();
}


// --------------------------------------------------------------------
// SIMULATION CONTROL
// --------------------------------------------------------------------

// TODO: add input
// Starts simulation timer and begins loop
void SimulatorParallel::start() {

    if (sim->running() || sim->masses.size() == 0) return;

    updates = 0;
    sdkStartTimer(&timer);
}

// Pauses simulation and syncs state back to host
// --------------------------------------------------------------------
void SimulatorParallel::pause() {
// --------------------------------------------------------------------

    if (!sim->running() || sim->masses.size() == 0) return;


}


// Steps simulation one timestep and syncs data back to host
// --------------------------------------------------------------------
void SimulatorParallel::step() {
// --------------------------------------------------------------------

    if (sim->running() || sim->masses.size() == 0) return;

    sim->step(sim->masses.front()->dt);

    cudaEventSynchronize(hostMemSyncEvent);
    exportGraphicsVertices();
    exportGraphicsIndices();

    float milli = 1;
    gpuErrchk(cudaEventRecord(stopEvent));
    gpuErrchk(cudaEventSynchronize(stopEvent));
    gpuErrchk(cudaEventElapsedTime(&milli, startEvent, stopEvent));
}


// Restarts simulation
// --------------------------------------------------------------------
void SimulatorParallel::stop() {
// --------------------------------------------------------------------

    sdkResetTimer(&timer);
}


// --------------------------------------------------------------------
// SIMULATION FUNCTIONS
// --------------------------------------------------------------------


// --------------------------------------------------------------------
void SimulatorParallel::removeLeastStressedBar() {
// --------------------------------------------------------------------

    cudaDeviceSynchronize();
    uint numBlocks = sim->springBlocksPerGrid;

    CUDA_SPRING * leastStress = nullptr;
    CUDA_SPRING ** h_out = (CUDA_SPRING **) malloc(numBlocks*sizeof(CUDA_SPRING *));
    CUDA_SPRING ** d_out = nullptr;

    gpuErrchk(cudaMalloc((void ***) &d_out, numBlocks*sizeof(CUDA_SPRING *)));
    gpuErrchk(cudaMemcpy(d_out, sim->d_spring,
                         numBlocks*sizeof(CUDA_SPRING *),
                         cudaMemcpyHostToDevice));

    reduceMinSpring(sim->springs.size(), THREADS_PER_BLOCK,
                    numBlocks, sim->d_spring, d_out);

    gpuErrchk(cudaPeekAtLastError());

    gpuErrchk(cudaMemcpy(h_out, d_out, numBlocks*sizeof(CUDA_SPRING *), cudaMemcpyDeviceToHost));

    leastStress = h_out[0];
    for (uint i = 1; i < numBlocks; i++) {
        leastStress = (leastStress->_max_stress < h_out[i]->_max_stress) ? leastStress : h_out[i];
    }

    qDebug() << "CUDA LEAST STRESS" << leastStress->_k;
}

// --------------------------------------------------------------------
void SimulatorParallel::removeNLeastStressedBars(int n) {
// --------------------------------------------------------------------
    //maxF_reducePass<THREADS_PER_BLOCK>(&structureMinStress, )

    // Test reduce
    //auto spring = thrust::reduce(sim->d_springs.begin(), sim->d_springs.end(), springPtr_minStressOperator);
    //qDebug() << "min spring" << spring;
    //thrust::device_vector<CUDA_SPRING *> d_springs_sorted(sim->d_springs);
    //thrust::sort(d_springs_sorted.begin(), d_springs_sorted.end());

    //qDebug() << d_springs_sorted.front();
}

// --------------------------------------------------------------------
void SimulatorParallel::reduceMinSpring(uint size, uint threads, uint blocks, CUDA_SPRING **d_spring, CUDA_SPRING **d_min_spring) {
// --------------------------------------------------------------------
    dim3 dimBlock(threads, 1, 1);
    dim3 dimGrid(blocks, 1, 1);

    int sharedMemSize = threads * sizeof(CUDA_SPRING *);

    warpReduce3MinSpring<<<dimGrid, dimBlock, sharedMemSize>>>(d_min_spring, d_spring, size);
}


// --------------------------------------------------------------------
//  GRAPHICS DATA OUTPUT
// --------------------------------------------------------------------


//  Registers CPU buffer ID to the GPU array for indices
//  NOTE: Assumes buffer input ID is bound
// --------------------------------------------------------------------
void SimulatorParallel::registerGraphicsVertexBuffer(uint cpuBufferId) {
// --------------------------------------------------------------------

    cudaGraphicsGLRegisterBuffer(&CUDA_VERTEX_BUFFER, cpuBufferId, cudaGraphicsMapFlagsWriteDiscard);

}


//  Registers CPU buffer ID to the GPU array for indices
//  NOTE: Assumes buffer input ID is bound
// --------------------------------------------------------------------
void SimulatorParallel::registerGraphicsIndexBuffer(uint cpuBufferId) {
// --------------------------------------------------------------------

    cudaGraphicsGLRegisterBuffer(&CUDA_INDEX_BUFFER, cpuBufferId, cudaGraphicsMapFlagsWriteDiscard);

}


//  Call GPU methods from the simulation to fill vertex buffer
// --------------------------------------------------------------------
void SimulatorParallel::exportGraphicsVertices() {
// --------------------------------------------------------------------

    float *vertices;
    size_t num_bytes;
    cudaGraphicsMapResources(1, &CUDA_VERTEX_BUFFER);
    cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void **>(&vertices), &num_bytes, CUDA_VERTEX_BUFFER);
    sim->updateMassVertices(vertices);
    cudaGraphicsUnmapResources(1, &CUDA_VERTEX_BUFFER);

}


//  Call GPU methods from the simulation to fill index buffer
// --------------------------------------------------------------------
void SimulatorParallel::exportGraphicsIndices() {
// --------------------------------------------------------------------

    unsigned int *indices;
    size_t num_bytes;
    cudaGraphicsMapResources(1, &CUDA_INDEX_BUFFER);
    cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void **>(&indices), &num_bytes, CUDA_INDEX_BUFFER);
    sim->updateSpringIndices(indices);
    cudaGraphicsUnmapResources(1, &CUDA_INDEX_BUFFER);

}

//  Unregisters GPU graphics resources
// --------------------------------------------------------------------
void SimulatorParallel::unregisterGraphicsResources() {
// --------------------------------------------------------------------

    cudaGraphicsUnregisterResource(CUDA_VERTEX_BUFFER);
    cudaGraphicsUnregisterResource(CUDA_INDEX_BUFFER);

}
