#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QObject>
#include <QTimer>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_device_runtime_api.h>
#include <cuda_gl_interop.h>

#include <cooperative_groups.h>
#include <helper_timer.h>

#include "src/loader.h"
#include "model.h"

#undef GRAPHICS
#include <Titan/sim.h>

class SimulatorParallel
{
public:
    SimulatorParallel(Simulation *sim, SimulationConfig *config);
    ~SimulatorParallel();

    Simulation *sim;

    enum Status {
        RUNNING,
        NOT_RUNNING
    };
    Status status;

    // --------------------------------------------------------------------
    // CUDA GRAPHICS INTEROP
    // --------------------------------------------------------------------
    void registerGraphicsVertexBuffer(uint id);
    void registerGraphicsIndexBuffer(uint id);
    void exportGraphicsVertices();
    void exportGraphicsIndices();
    void unregisterGraphicsResources();

public slots:
    // --------------------------------------------------------------------
    // SIMULATION CONTROL
    // --------------------------------------------------------------------
    void start();
    void step();
    void pause();
    void stop();

private:
    SimulationConfig *config;

    // --------------------------------------------------------------------
    // SIMULATION FUNCTIONS
    // --------------------------------------------------------------------
    void optimizeTopology();
    void removeLeastStressedBar();
    void removeNLeastStressedBars(int n);
    void removeDanglingMasses();
    void reduceMinSpring(uint size, uint threads, uint blocks, CUDA_SPRING ** d_spring, CUDA_SPRING ** d_min_spring);

    // --------------------------------------------------------------------
    // SIMULATION PROPERTIES
    // --------------------------------------------------------------------
    StopWatchInterface *timer;
    cudaEvent_t startEvent;
    cudaEvent_t stopEvent;
    cudaEvent_t hostMemSyncEvent;
    double syncTimestep;
    double renderTimestep;
    double optTimestep;

    long start_n_masses;
    long start_n_springs;
    long updates;

    // --------------------------------------------------------------------
    // CUDA GRAPHICS INTEROP
    // --------------------------------------------------------------------
    struct cudaGraphicsResource *CUDA_VERTEX_BUFFER;
    struct cudaGraphicsResource *CUDA_INDEX_BUFFER;
};



#endif // SIMULATOR_H
