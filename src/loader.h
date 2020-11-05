#ifndef LOADER_H
#define LOADER_H

#include <QFile>
#include <QObject>
#include <QProgressDialog>
#include <QTextStream>
#include <glm/glm.hpp>
#include <Eigen/SparseCore>
#include <Spectra/SymGEigsSolver.h>
#include <Spectra/MatOp/SparseCholesky.h>
#include "model.h"
#include "utils.h"
#include <ctime>


#undef GRAPHICS
#include <Titan/sim.h>

class Loader : public QObject
{
    Q_OBJECT

public:
    Loader(QObject *parent = nullptr);
    ~Loader();

    void createGridLattice(simulation_data *arrays, int dimX, int dimY, int dimZ);
    void createGridLattice(simulation_data *arrays, SimulationConfig *simConfig);
    void createSpaceLattice(simulation_data *arrays, SimulationConfig *simConfig);
    void createGridLattice(Polygon *geometryBound, LatticeConfig &lattice, float cutoff);
    void createSpaceLattice(Polygon *geometryBound, LatticeConfig &lattice, float cutoff, bool includeHull);

    void loadDesignModels(Design *design);
    void loadVolumeModel(Volume *volume);
    void loadVolumeGeometry(Volume *volume);
    void loadSimModel(SimulationConfig *simConfig);
    void loadVolumes(model_data *arrays, Design *design);
    void loadModel(model_data *arrays, Volume *volume, uint n_volume);
    void readSTLFromFile(model_data *arrays, QString filePath, uint n_model);
    void loadSimulation(Simulation *sim, SimulationConfig *simConfig);
    void loadSimulation(simulation_data *arrays, Simulation *sim, uint n_volume);
    void loadSimFromLattice(simulation_data *arrays, Simulation *sim, vector <LatticeConfig *> lattices);
    void loadSimFromLattice(LatticeConfig *lattice, Simulation *sim, double springCutoff);
    void loadBarsFromSim(Simulation *sim, bar_data *output, bool crossSection, bool markers);
    void applyLoadcase(Simulation *sim, Loadcase *load);

    double calculateNaturalPeriod(Simulation *sim);
    void suggestParams(Simulation *sim, SimulationConfig *simConfig);

	  int surfacePoints;

signals:
    void log(const QString &message);


};

#endif // LOADER_H
