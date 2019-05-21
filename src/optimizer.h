//
// Created by sw3390 on 4/24/19.
//

#ifndef DMLIDE_OPTIMIZER_H
#define DMLIDE_OPTIMIZER_H

#include <QDebug>

#include <Titan/sim.h>

#include "utils.h"

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

    virtual void optimize() = 0;

    void sortSprings_stress(vector<uint> &output_indices);
    void sortMasses_stress(vector<uint> &output_indices);
};


/**
 * SpringRemover
 * Optimizer that removes springs from simulation based on stress
 */
class SpringRemover : public Optimizer {

public:
    SpringRemover(Simulation *sim, double removeRatio);

    double stepRatio;
    double stopRatio = 0;

protected:
    void optimize() override;
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
    MassDisplacer(Simulation *sim, double displaceRatio);
    ~MassDisplacer() {
        delete start;
        delete test;
    }

    Simulation *start;
    Simulation *test;

    double stepRatio;
    vector<uint> movedMasses;
    vector<Vec> movedVectors;
    bool equilibrium;
    vector<double> prevEnergy;

    double maxLocalization;
    map<Mass *, vector<Spring *>> massConns;
    map<Spring *, vector<Spring *>> springConns;

    void initializeClones(int n);
    void mutateClones(double dx);
    void incorporateClones();
    void resetClones();

    struct DisplacedMass {
        DisplacedMass() {
            index = -1;
            displacement = Vec(0, 0, 0);
        }

        int index;
        Vec displacement;
    };

    struct DisplacedSpring {
        DisplacedSpring() {
            index = -1;
            rest = 0;
            constant = 0;
        }

        DisplacedSpring(int i, double r, double k) {
            index = i;
            rest = r;
            constant = k;
        }

        int index;
        double rest;
        double constant;
    };

    struct Clone {
        Clone() {
            sim = nullptr;
            adjustedSprings = vector<DisplacedSpring>();
        }

        explicit Clone(Simulation *s) {
            sim = s;
            adjustedSprings = vector<DisplacedSpring>();
        }

        Simulation *sim;
        DisplacedMass displacedMass;
        vector<DisplacedSpring> adjustedSprings;
    };

protected:
    void optimize() override;

private:

    vector<Clone> clones;

    bool STARTED;

    int pickRandomMass(Simulation *sim);
    void shiftMassPos(Simulation *sim, int index, const Vec &dx);
    void shiftRandomChunk(Simulation *sim, const Vec &dx, vector<int> indices);
    void shiftCloneMass(Clone *clone, double dx);
    vector<DisplacedSpring> shiftOrigPos(Simulation *sim, Mass * m, const Vec &p);
    double calcTotalLength(Simulation *sim);
    double calcTotalEnergy(Simulation *sim);
    int settleSim(Simulation *sim, double eps, bool use_cap=false, double cap=0);

    int displaceParallelMasses(int copies, int n_copy);
    int displaceSingleMass();
    void displacePercentMass();
    void setMassState(const vector<Vec> &pos);
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

#endif //DMLIDE_OPTIMIZER_H
