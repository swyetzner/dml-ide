#ifndef TITANVIEWER_H
#define TITANVIEWER_H

#include "optimizer.h"
#include "loader.h"
#include "io/exportThread.h"

#undef GRAPHICS
#include <Titan/sim.h>

#include <ctime>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <QMatrix4x4>

struct sim_metrics {
    sim_metrics() = default;
    double clockTime;
    int nbars;
    double time;
    double totalLength;
    double totalEnergy;

    double totalLength_start;
    double totalEnergy_start;
    double deflection;
    int optimize_iterations;
    OptimizationRule optimize_rule;
    int relaxation_interval;
    double displacement;
};


class Simulator {
public:
    explicit Simulator(Simulation *sim, Loader *loader, SimulationConfig *config,
            OptimizationConfig * optconfig = nullptr, bool graphics = false, bool endExport = true);
    ~Simulator();

    enum Status {
        NOT_STARTED,
        STARTED,
        PAUSED,
        STOPPED
    };

    Simulation *sim;
    SimulationConfig *config;
    Optimizer *optimizer;
    OptimizationConfig *optConfig;
    Loader *loader;
    bar_data *barData;
    ExportThread exportThread;

    SpringInserter *springInserter;
    MassDisplacer *massDisplacer;
    SpringRemover *springRemover;

    Status simStatus;
    bool GRAPHICS;
    bool EXPORT;

    // --------------------------------------------------------------------
    // SIMULATION  FUNCTIONS
    // --------------------------------------------------------------------
    void setSyncTimestep(double st);
    void setSimTimestep(double dt);
    void setDataDir(std::string dp);
    void runSimulation(bool running);
    void runStep();
    void getSimMetrics(sim_metrics &metrics);
    void loadSimDump(std::string sp);
    void exportSimulation();
    void dumpSpringData();

private:

    // --------------------------------------------------------------------
    // SIMULATION PROPERTIES + FUNCTIONS
    // --------------------------------------------------------------------
    void run();
    void repeatLoad();

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
    double totalLength_prev;
    double totalEnergy_prev;
    double totalLength_start;
    double totalEnergy_start;
    double deflection_start;
    Vec deflectionPoint_start;
    long steps;
    std::chrono::time_point<std::chrono::system_clock> startWallClockTime;
    double prevWallClockTime;
    double wallClockTime;


    int n_repeats;
    int optimizeAfter;
    double repeatTime;
    bool explicitRotation;
    Vec repeatRotation;
    bool equilibrium;
    double relaxation;
    int optimized;
    int closeToPrevious;
    int stepsSinceEquil;
    double prevEnergy;
    int prevSteps;
    bool switched;
    Vec center;

    bool OPTIMIZER;

    void loadOptimizers();
    Vec getSimCenter();
    void equilibriate();
    bool stopCriteriaMet();
    bool dumpCriteriaMet();

    int currentLoad;
    float pastLoadTime;
    float optimizeTime;
    bool varyLoad;
    void clearLoads();
    void applyLoad(Loadcase * load);
    void varyLoadDirection();

    Vec getDeflectionPoint();
    double calcDeflection();

    void printStatus();

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
    void writeSimDump(const QString &outputFile);

    // --------------------------------------------------------------------
};

#endif // TITANVIEWER_H
