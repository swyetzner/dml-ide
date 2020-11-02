//
// Created by sw3390 on 4/24/19.
//

#ifndef DMLIDE_OPTIMIZER_H
#define DMLIDE_OPTIMIZER_H

#include <QDebug>
#include <chrono>
#include <stdlib.h>

#include <Titan/sim.h>

#include "oUtils.h"
#include "utils.h"
#include "model.h"

/**
 * Optimizer base class
 */
class Optimizer {

public:

    explicit Optimizer(Simulation *sim) {
        this->sim = sim;
        n_masses = sim->masses.size();
        n_masses_start = n_masses;
        n_springs = sim->springs.size();
        n_springs_start = n_springs;
    }

    Simulation *sim;
    int n_masses;
    int n_masses_start;
    int n_springs;
    int n_springs_start;
    int iterations;

    virtual void optimize() = 0;

    uint minSpringByStress();
    void sortSprings_stress(vector<Spring *> &spring_list, vector<uint> &output_indices);
    void sortMasses_stress(vector<uint> &output_indices);
    int settleSim(double eps, bool use_cap=false, double cap=0);

    struct VolumeConstraint {
        double minX, minY, minZ;
        double maxX, maxY, maxZ;
    };

private:
    VolumeConstraint volumeConstraint;
};


/**
 * SpringRemover
 * Optimizer that removes springs from simulation based on stress
 */
class SpringRemover : public Optimizer {

public:
    SpringRemover(Simulation *sim, double removeRatio, double stopRatio = 0);

    bool regeneration;

    double stepRatio;
    double stopRatio;
    vector<Spring *> validSprings;
    map<Mass *, vector<Spring *>> massToSpringMap;
    vector<Spring *> removedSprings;
    vector<double> removedSprings_k;
    vector<Mass *> affectedMasses;
    vector<double> affectedWeights;
    double massFactor;
    double stressMemory;
    double regenRate;

    void deleteGhostSprings();
    void resetHalfLastRemoval();
    void regenerateLattice(SimulationConfig *config);
    void regenerateShift();

protected:
    void optimize() override;

private:
    void fillMassSpringMap();
    void removeSpringFromMap(Spring *d);
    void removeMassFromMap(Mass *d);
    void invalidateSpring(Spring *i);
    void removeHangingSprings(map<Spring *, bool> &hangingCandidates, map<Spring *, bool> &springsToDelete);
    void deleteSpring(Spring *d);
    void splitSprings();
};

/**
 * SpringResizer
 * Optimizer that resizes and removes ratio springs according to stress
 */
class SpringResizer : public Optimizer {

public:
    SpringResizer(Simulation *sim, double ratio, double removeCutoff, double maxCutoff);

    double ratio;
    double removeCutoff;
    double maxCutoff;
    double startDiam;

protected:
    void optimize() override;
};

/**
 * MassDisplacer
 * Optimizer that displaces masses randomly and checks resulting stress
 */
class MassDisplacer : public Optimizer {

public:
    MassDisplacer(Simulation *sim, double dx, double displaceRatio, double massFactor);
    ~MassDisplacer() {
        delete start;
        delete test;
    }

    Simulation *start;
    Simulation *test;

    double stepRatio;
    vector<uint> movedMasses;
    vector<Vec> movedVectors;

    double dx;
    bool equilibrium;
    vector<double> prevEnergy;
    double massFactor;

    double localEnergy;
    int attempts;
    int totalAttempts;
    double totalTrialTime;
    vector<int> prevAttemptNums;
    float maxAvgSuccessRate;
    int lastTune;

    double lastMetric;

    int order;
    double chunkSize;
    double maxLocalization;
    int relaxation;
    map<Mass *, vector<Spring *>> massConns;
    map<Spring *, vector<Spring *>> springConns;

    QString customMetricHeader;
    QString customMetric;


    // Struct holding the locality for a displaced mass
    // group -- surrounding masses within the locality
    // springs -- springs within the locality
    // outside -- masses that are connected within one order of the group,
    //      but are not within it
    struct MassGroup {
        Mass *displaced;
        vector<Mass *> group;
        vector<Mass *> candidates;
        vector<Spring *> springs;
        vector<Mass *> outside;
        vector<Mass *> edge;
        vector<Spring*> border;

        double origLength = 0;
        double origEnergy = 0;
        double testLength = 0;
        double testEnergy = 0;

        Vec dx;
        vector<Mass *> displacedList;
        vector<Vec> displacements;
        Vec displaceOrigPos;
        vector<Vec> startPos;
        vector<double> startMass;
        vector<double> startRest;
        vector<Mass *> fixed;
        vector<Spring> groupStart;

    } massGroup;

    vector<MassGroup *> massGroups;
    map<Mass *, MassGroup *> massGroupMap;
    vector<Spring *> trenchSprings;
    int gridSize[3];
    Vec gridOffset; // Current mass group offset
    Vec dimensions; // Dimensions of simulation
    double unit; // Unit for mass group cubes
    double springUnit; // Unit for mass separation

    // Struct holding separation grid information
    struct TrenchGrid {
        Vec startCorner;
        Vec endCorner;
        Vec dimension;
    } trenchGrid;



protected:
    void optimize() override;

private:

    bool STARTED;

    int pickRandomMass(Simulation *sim);
    int pickRandomMass(MassGroup &mg);
    int getMassCandidate(Simulation *sim, vector<int> existingMasses, double cutoff);
    double calcOrigDist(Mass *m1, Mass *m2);
    bool springExists(Simulation *sim, Mass *m1, Mass *m2);
    void mergeMasses(Simulation *sim, Mass *m1, Mass *m2, Spring *c);
    int shiftMassPos(Simulation *sim, int index, const Vec &dx, vector<Mass *> &merged);
    void shiftMassPos(Simulation *sim, Mass *m, const Vec &dx);
    int shiftRandomChunk(Simulation *sim, const Vec &dx, vector<int> indices, vector<Mass *> &merged);
    void createMassGroup(Simulation *sim, double cutoff, Mass *center, MassGroup &massGroup);
    void createMassGroup(Simulation *sim, Vec minc, Vec maxc, MassGroup &massGroup);
    void createMassTiles(Simulation *sim, double unit, Vec offset, vector<MassGroup *> &massGroups,
            map<Mass *, MassGroup *> &massGroupMap, vector<Spring *> &trenchSprings);
    int createTile(int n, int i, double width, double offset, double minPos, double &tileStart, double &tileEnd);
    void createMassGroupGrid(Simulation *sim, const TrenchGrid &trenchGrid, vector<MassGroup> &mgs);
    void splitMassTiles(Simulation *sim, vector<MassGroup *> &mgs, vector<Spring *> &tsSim, vector<Spring> &tsSave,
            vector<Mass *> &massSpans);
    void combineMassTiles(Simulation *sim, vector<MassGroup *> &massGroups, vector<Spring> &tsSave,
            vector<Mass *> massSpans);
    double calcTotalLength(Simulation *sim);
    double calcTotalEnergy(Simulation *sim);
    double calcOrderLength(Simulation *sim, vector<Spring *> group);
    double calcOrderEnergy(Simulation *sim, vector<Spring *> group);
    double calcMassGroupLength(MassGroup * massGroup);
    double calcMassGroupEnergy(MassGroup * massGroup);
    int settleSim(Simulation *sim, double eps, bool use_cap=false, double cap=0);
    void relaxSim(Simulation *sim, int steps, vector<Mass *> track=vector<Mass *>());
    void setMassState(const vector<Vec> &pos, const vector<double> &mm);

    int displaceSingleMass(double displacement, double chunkSize, int metricOrder);
    int displaceGroupMass(double displacement);
};


/**
 * SpringInserter
 * Optimizer that inserts 2nd degree springs in high stress areas
 */
class SpringInserter : public Optimizer {

public:
    SpringInserter(Simulation *sim, double addRatio);

    double stepRatio;
    double cutoff = 0;

    void optimize() override;

private:
    void findPlacesToAddSpring(Spring *stressedSpring, vector<Mass *> &locations);
    void braceSpring(Spring *stressedSpring, vector<Mass *> &locations);
    int combineParallelSprings();
    void bisectSpring(Spring *s, Mass *mid);
    void joinSprings(Spring *s1, Spring *s2);
};

class MassMigrator : public Optimizer {

};

class MassMigratorFreq : public Optimizer {
public:
    MassMigratorFreq(Simulation *sim, double dxMax, double upperFreq, double lowerFreq, bool shapeConstraint = false);

    const double pi = 4*atan(1);
    double dxMax;
    bool shapeConstraint;
    map<Mass *, vector<Spring *>> massToSpringMap;
    vector<Spring *> validSprings;

    void optimize() override;

private:
    void fillMassSpringMap();
    void findPeaks(Vec ** modeShapes, int massNum, int bands);
    Vec gradT(Vec * modeShapes, Mass* mass);
    Vec calcT(Vec * modeShapes, Simulation *sim);
    Vec gradV(Vec * modeShapes, Mass* mass);
    void shiftMassPos(Simulation *sim, Vec *disp);
};

#endif //DMLIDE_OPTIMIZER_H
