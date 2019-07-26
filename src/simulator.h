#ifndef TITANVIEWER_H
#define TITANVIEWER_H

#include "model.h"
#include "optimizer.h"
#include "loader.h"
#include "exportThread.h"

#undef GRAPHICS
#include <Titan/sim.h>

#include <ctime>
#include <iomanip>

struct sim_metrics {
    sim_metrics() = default;
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
            OptimizationConfig * optconfig = nullptr);
    ~Simulator();

    enum Status {
        STARTED,
        PAUSED,
        STOPPED
    };

    Simulation *sim;
    SimulationConfig *config;
    Optimizer *optimizer;
    OptimizationConfig *optConfig;
    Loader *loader;
    ExportThread exportThread;

    SpringInserter *springInserter;
    MassDisplacer *massDisplacer;
    SpringRemover *springRemover;

    Status simStatus;

    // --------------------------------------------------------------------
    // SIMULATION  FUNCTIONS
    // --------------------------------------------------------------------
    void setSyncTimestep(double st);
    void setSimTimestep(double dt);
    void runSimulation(bool running);
    void runStep();
    void getSimMetrics(sim_metrics &metrics);
    void exportSimulation();

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
    double totalLength_start;
    double totalEnergy_start;
    Vec deflectionPoint_start;
    long steps;


    int n_repeats;
    int optimizeAfter;
    double repeatTime;
    bool explicitRotation;
    Vec repeatRotation;
    bool equilibrium;
    double relaxation;
    int optimized;
    int closeToPrevious;
    double prevEnergy;
    int prevSteps;
    bool switched;
    Vec center;

    Vec getSimCenter();
    void equilibriate();
    bool stopCriteriaMet();

    int currentLoad;
    float pastLoadTime;
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
    void writePos(const QString &outputFile);

    // --------------------------------------------------------------------
};

#endif // TITANVIEWER_H
