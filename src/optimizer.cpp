//
// Created by sw3390 on 4/24/19.
//

#include "optimizer.h"

// Sorts springs by max stress
// Outputs sorted indices (indexing to sim->springs)
// into parameter output_indices
//---------------------------------------------------------------------------
void Optimizer::sortSprings_stress(vector<uint> &output_indices) {
//---------------------------------------------------------------------------

    vector<double> springStress = vector<double>();
    output_indices = vector<uint>();

    for (uint s = 0; s < sim->springs.size(); s++) {
        bool underExternalForce = sim->springs[s]->_left->extforce.norm() > 1E-6
                                  && sim->springs[s]->_right->extforce.norm() > 1E-6;
        bool fixed = sim->springs[s]->_left->constraints.fixed && sim->springs[s]->_right->constraints.fixed;
        if (!underExternalForce && !fixed) {
            output_indices.push_back(s);
        }

        double force = sim->springs[s]->_max_stress;
        springStress.push_back(force);
    }

    // Sort in increasing order by max stress
    sort(output_indices.begin(), output_indices.end(),
         [springStress](uint s1, uint s2) -> bool {
             return springStress[s1] < springStress[s2];
         });
    qDebug() << "Sorted springs by stress" << output_indices.size();
}


// Sorts masses by max stress
// Outputs sorted indices (indexing to sim->masses)
// into parameter output_indices
//---------------------------------------------------------------------------
void Optimizer::sortMasses_stress(vector<uint> &output_indices){
//---------------------------------------------------------------------------

    vector<double> massStresses = vector<double>(sim->masses.size());
    output_indices = vector<uint>();

    for (uint m = 0; m < sim->masses.size(); m++) {
        bool underExternalForce = sim->masses[m]->extforce.norm() > 1E-6;
        bool fixed = sim->masses[m]->constraints.fixed;

        if (!underExternalForce && !fixed) {
            output_indices.push_back(m);
        }

        massStresses[m] = sim->masses[m]->maxforce.norm();
    }

    sort(output_indices.begin(), output_indices.end(),
         [massStresses](uint m1, uint m2) -> bool {
             return massStresses[m1] < massStresses[m2];
         });
    qDebug() << "Sorted masses by stress";
}


//---------------------------------------------------------------------------
//  SPRING REMOVER
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
SpringRemover::SpringRemover(Simulation *sim, double removeRatio, double stopRatio)
    : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->stepRatio = removeRatio;
    this->stopRatio = stopRatio;
    qDebug() << "Set spring remover ratios" << this->stepRatio << this->stopRatio;
}


// Removes stepRatio percent least stressed springs
//---------------------------------------------------------------------------
void SpringRemover::optimize() {
//---------------------------------------------------------------------------

    sim->getAll();
    n_springs = sim->springs.size();

    if (n_springs > n_springs_start * stopRatio) {
        vector<uint> springIndicesToSort;
        sortSprings_stress(springIndicesToSort);

        uint toRemove = uint(stepRatio * sim->springs.size());
        map<Spring *, bool> springsToDelete = map<Spring *, bool>();
        for (Spring *s : sim->springs) {
            springsToDelete[s] = false;
        }
        for (uint j = 0; j < toRemove; j++) {
            if (j < springIndicesToSort.size())
                springsToDelete[sim->springs[springIndicesToSort[j]]] = true;
        }
        qDebug() << "Removing" << toRemove << "Springs";

        // Remove hanging springs (attached to masses with only one attached spring)
        for (Spring *s : sim->springs) {
            if (s->_left->spring_count < 2 || s->_right->spring_count < 2) {
                springsToDelete[s] = true;
            }
            // For 2 attached springs, determine angle between them
            if (s->_left->spring_count < 3) {
                for (Spring *s1 : sim->springs) {
                    if (s1 != s) {
                        if (s1->_left == s->_left) {
                            Vec bar1 = s->_right->pos - s->_left->pos;
                            Vec bar2 = s1->_right->pos - s1->_left->pos;
                            if (Utils::isAcute(bar1, bar2)) {
                                springsToDelete[s] = true;
                                springsToDelete[s1] = true;
                            }
                        }
                        if (s1->_right == s->_left) {
                            Vec bar1 = s->_right->pos - s->_left->pos;
                            Vec bar2 = s1->_left->pos - s1->_right->pos;
                            if (Utils::isAcute(bar1, bar2)) {
                                springsToDelete[s] = true;
                                springsToDelete[s1] = true;
                            }
                        }
                    }
                }
            }
            if (s->_right->spring_count < 3) {
                for (Spring *s1 : sim->springs) {
                    if (s1 != s) {
                        if (s1->_left == s->_right) {
                            Vec bar1 = s->_left->pos - s->_right->pos;
                            Vec bar2 = s1->_right->pos - s1->_left->pos;
                            if (Utils::isAcute(bar1, bar2)) {
                                springsToDelete[s] = true;
                                springsToDelete[s1] = true;
                            }
                        }
                        if (s1->_right == s->_right) {
                            Vec bar1 = s->_left->pos - s->_right->pos;
                            Vec bar2 = s1->_left->pos - s1->_right->pos;
                            if (Utils::isAcute(bar1, bar2)) {
                                springsToDelete[s] = true;
                                springsToDelete[s1] = true;
                            }
                        }
                    }
                }
            }
        }


        // Remove springs
        qDebug() << "About to deleted";
        uint i = 0;
        while (i < sim->springs.size()) {
            if (sim->springs[i] != nullptr && springsToDelete[sim->springs[i]]) {
                sim->deleteSpring(sim->springs[i]);
                i--;
            }
            i++;
        }
        qDebug() << "Deleted springs";

        int hangingSprings = 0;
        while(hangingSprings > -1) {

            hangingSprings = 0;
            int springSize = sim->springs.size();
            for (int s = 0; s < springSize; s++) {
                Mass *m1 = sim->springs[s]->_left;
                Mass *m2 = sim->springs[s]->_right;
                int leftCount = 0, rightCount = 0;

                for (int t = 0; t < springSize; t++) {
                    if (t != s) {

                        if (sim->springs[t]->_left == m1 || sim->springs[t]->_right == m1) {
                            leftCount++;
                        }
                        if (sim->springs[t]->_left == m2 || sim->springs[t]->_right == m2) {
                            rightCount++;
                        }
                    }
                }

                if (leftCount == 0 || rightCount == 0) {
                    hangingSprings++;
                    sim->deleteSpring(sim->springs[s]);
                    s--;
                    springSize--;
                    goto ENDLOOP;
                }

                // For 2 attached springs, determine angle between them
                // Delete one and leave one for cleanup
                if (leftCount == 1) {
                    for (int s1 = 0; s1 < springSize; s1++) {
                        if (s1 != s) {
                            if (sim->springs[s]->_left == sim->springs[s1]->_left) {
                                Vec bar1 = sim->springs[s]->_right->pos - sim->springs[s]->_left->pos;
                                Vec bar2 = sim->springs[s1]->_right->pos - sim->springs[s1]->_left->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    hangingSprings ++;
                                    sim->deleteSpring(sim->springs[s]);
                                    s --;
                                    springSize --;
                                    goto ENDLOOP;
                                }
                            }
                            if (sim->springs[s]->_right == sim->springs[s1]->_left) {
                                Vec bar1 = sim->springs[s]->_right->pos - sim->springs[s]->_left->pos;
                                Vec bar2 = sim->springs[s1]->_left->pos - sim->springs[s1]->_right->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    hangingSprings ++;
                                    sim->deleteSpring(sim->springs[s]);
                                    s --;
                                    springSize --;
                                    goto ENDLOOP;
                                }
                            }
                        }
                    }
                }
                if (rightCount == 1) {
                    for (int s1 = 0; s1 < springSize; s1++) {
                        if (s1 != s) {
                            if (sim->springs[s]->_left == sim->springs[s1]->_right) {
                                Vec bar1 = sim->springs[s]->_left->pos - sim->springs[s]->_right->pos;
                                Vec bar2 = sim->springs[s1]->_right->pos - sim->springs[s1]->_left->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    hangingSprings ++;
                                    sim->deleteSpring(sim->springs[s]);
                                    s --;
                                    springSize --;
                                    goto ENDLOOP;
                                }
                            }
                            if (sim->springs[s]->_right == sim->springs[s1]->_right) {
                                Vec bar1 = sim->springs[s]->_left->pos - sim->springs[s]->_right->pos;
                                Vec bar2 = sim->springs[s1]->_left->pos - sim->springs[s1]->_right->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    hangingSprings ++;
                                    sim->deleteSpring(sim->springs[s]);
                                    s --;
                                    springSize --;
                                    goto ENDLOOP;
                                }
                            }
                        }
                    }
                }
                ENDLOOP: ;
            }

            qDebug() << "Corrected" << hangingSprings << "hanging springs";
            if (hangingSprings == 0) hangingSprings = -1;
        }

        sim->setAll(); // Set spring stresses and mass value updates on GPU

        n_springs = int(sim->springs.size());
        qDebug() << "Springs" << n_springs << "Percent springs left" << 100 * n_springs / n_springs_start;
    } else {
        qDebug() << "Optimization ended";
    }
}


//---------------------------------------------------------------------------
//  SPRING RESIZER
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
SpringResizer::SpringResizer(Simulation *sim, double ratio, double removeCutoff, double maxCutoff)
        : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->ratio = ratio;
    this->removeCutoff = removeCutoff;
    this->maxCutoff = maxCutoff;
    this->startDiam = sim->springs.front()->_diam;
}

// Resizes all spring diameters according to stress
// Removes springs with diameters under removeCutoff
//---------------------------------------------------------------------------
void SpringResizer::optimize() {
//---------------------------------------------------------------------------

    double maxObjectStress = 0;
    double avgObjectStress = 0;
    double avgObjectDiam = 0;
    map<Spring *, bool> springsToDelete = map<Spring *, bool>();

    sim->getAll();
    n_springs = sim->springs.size();
    int toResize = int(ratio * n_springs);

    for (Spring *s : sim->springs) {
        double stress = s->_max_stress;

        if (stress > maxObjectStress) {
            maxObjectStress = stress;
        }

        avgObjectStress += stress;
        avgObjectDiam += s->_diam;
        springsToDelete[s] = false;
    }
    avgObjectStress /= n_springs;
    avgObjectDiam /= n_springs;
    qDebug() << "Average stress" << avgObjectStress;
    qDebug() << "Max stress" << maxObjectStress;

    vector<uint> springIndicesToSort = vector<uint>();
    sortSprings_stress(springIndicesToSort);

    /**for (Spring *t : sim->springs) {
        double stress = t->_max_stress;

        bool underExternalForce = t->_left->extforce.norm() > 1E-6
                                  && t->_right->extforce.norm() > 1E-6;
        bool fixed = t->_left->constraints.fixed && t->_right->constraints.fixed;

        double e = (stress / avgObjectStress - 1) * avgObjectDiam / 10;

        t->_k *= 1 / (t->_diam * t->_diam);
        t->_diam += e;
        t->_k *= t->_diam * t->_diam;

        if (!underExternalForce && !fixed) {
            if (t->_diam < this->removeCutoff) {
                springsToDelete[t] = true;
            }
        }

        // Rest max stress
        t->_max_stress = 0;
    **/
    for (int i = 0; i < toResize; i++) {
        if (i < springIndicesToSort.size()) {
            Spring *t = sim->springs[springIndicesToSort[i]];
            double e = startDiam / 2;

            t->_k *= 1 / (t->_diam * t->_diam);
            t->_diam -= e;
            t->_k *= t->_diam * t->_diam;
            sim->setAll();

            if (t->_diam < this->removeCutoff) {
                springsToDelete[t] = true;
            }
        }
    }
    qDebug() << "Decreased" << toResize << "spring diameters";

    for (int j = springIndicesToSort.size() - 1; j > springIndicesToSort.size() - toResize - 1; j--) {
        if (j >= 0) {
            Spring *t = sim->springs[springIndicesToSort[j]];
            double e = startDiam / 2;

            if (t->_diam < this->maxCutoff) {
                t->_k *= 1 / (t->_diam * t->_diam);
                t->_diam += e;
                t->_k *= t->_diam * t->_diam;
                t->_max_stress = 0;
                sim->setAll();
            }
        }
    }
    qDebug() << "Increased" << toResize << "spring diameters";


    // Remove springs
    uint i = 0;
    while (i < sim->springs.size()) {
        if (sim->springs[i] != nullptr && springsToDelete[sim->springs[i]]) {
            sim->deleteSpring(sim->springs[i]);
            i--;
        }
        i++;
    }

    sim->setAll(); // Set spring stresses and mass value updates on GPU

    n_springs = int(sim->springs.size());
    qDebug() << "Springs" << n_springs << "Percent springs left" << 100 * n_springs / n_springs_start;
}


//---------------------------------------------------------------------------
//  MASS DISPLACER
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
MassDisplacer::MassDisplacer(Simulation *sim, double dx, double displaceRatio)
    : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->stepRatio = displaceRatio;
    this->dx = dx;
    this->order = 0;
    this->chunkSize = -1;
    this->relaxation = 0;
    this->maxLocalization = 0;
    this->iterations = 0;
    this->attempts = 0;
    this->prevAttemptNums = vector<int>();
    this->maxAvgSuccessRate = 0;
    this->lastTune = 0;
    test = new Simulation();
    start = new Simulation();

    this->maxLocalization = 0;

    customMetricHeader = "Time,Position(x),Position(y),Position(z),Force(x),Force(y),Force(z),Index\n";

    // Initialize connections
    /**springConns = map<Spring *, vector<Spring *>>();
    massConns = map<Mass *, vector<Spring *>>();
    for (Mass *m : sim->masses) {

        massConns[m] = vector<Spring *>();
        for (Spring *s : sim->springs) {
            if (s->_left == m || s->_right == m) {
                massConns[m].push_back(s);
            }
        }
    }
    for (Spring *s : sim->springs) {

        springConns[s] = vector<Spring *>();
        for (Mass *m : sim->masses) {
            if (s->_left == m || s->_right == m) {
                for (Spring *t : massConns[m]) {
                    if (t != s) {
                        springConns[s].push_back(t);
                    }
                }
            }
        }
    }**/

    STARTED = false;
}

// Moves stepRatio percent masses
// Creates parellel Simulations and compares output
//---------------------------------------------------------------------------
void MassDisplacer::optimize() {
//---------------------------------------------------------------------------

    int displaced = 0;
    attempts = 0;

    while (displaced == 0) {
        attempts ++;
        displaced = displaceSingleMass(dx, chunkSize, order);
        //displaced = displaceManyMasses(dx, order, 2);
    }

    /**prevAttemptNums.push_back(attempts);
    while (prevAttemptNums.size() > 100) {
        prevAttemptNums.erase(prevAttemptNums.begin());
    }
    if (prevAttemptNums.size() == 100) {
        float avgAttempts = 0;
        for (int a : prevAttemptNums) {
            avgAttempts += a;
        }
        avgAttempts /= prevAttemptNums.size();

        float successRate = 1 / avgAttempts;
        if (successRate > maxAvgSuccessRate) {
            maxAvgSuccessRate = successRate;
        }

        qDebug() << "Success Rate:" << successRate << " Max Success Rate:" << maxAvgSuccessRate;
        if (successRate < maxAvgSuccessRate * 0.5 && (iterations - lastTune) > 100) {
            // If success rate is less than half of max, decrease displacement to fine-tune
            dx /= 2;
            lastTune = iterations;
        }
    }**/

    iterations++;

    /**if (!STARTED) {
        initializeClones(100);
        STARTED = true;
    }

    mutateClones(0.002);
    incorporateClones();
    resetClones();**/

    //displaceParallelMasses(50, 1);
}


int MassDisplacer::displaceParallelMasses(int copies, int n_copy) {

    test->copy(*sim);
    qDebug() << "Copied sim" << test->masses.size() << test << sim;

    vector<int> moved = vector<int>();
    double totalEnergySim = 0;
    double totalLengthSim = 0;
    vector<double> totalEnergyTests = vector<double>();
    vector<double> totalLengthTests = vector<double>();

    // Calculate sim length
    for (Spring *s : test->springs) {
        totalLengthSim += s->_rest;
    }

    for (int n = 1; n < copies; n++) {

        test->append(*sim);

        // Pick a random mass
        int i = rand() % n_masses;
        bool underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
        bool fixed = sim->masses[i]->constraints.fixed;
        bool used = find(moved.begin(), moved.end(), i) != moved.end();

        while (used || underExternalForce || fixed) {
            i = rand() % n_masses;
            underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
            fixed = sim->masses[i]->constraints.fixed;
            used = find(moved.begin(), moved.end(), i) == moved.end();
        }
        moved.push_back(i);
        qDebug() << "Picked mass" << i << n_masses << test->masses.size();

        Mass *mt = test->masses[i + test->masses.size() - n_masses];

        // Pick a random direction
        Vec dir = Utils::randDirectionVec();
        dir = dir.normalized();
        qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
        Vec dx = 0.002 * dir;

        // Move mass
        totalLengthTests.push_back(0);
        for (int j = test->springs.size() - n_springs; j < test->springs.size(); j++) {
            Spring *s = test->springs[j];
            Vec orig = mt->origpos + dx;
            if (s->_left == mt) {
                double origLen = s->_rest;
                s->_rest = (s->_right->origpos - orig).norm();
                s->_k *= origLen / s->_rest;
            }
            if (s->_right == mt) {
                double origLen = s->_rest;
                s->_rest = (s->_left->origpos - orig).norm();
                s->_k *= origLen / s->_rest;
            }

            // Calculate test length
            totalLengthTests.back() += s->_rest;
        }
        mt->origpos += dx;
        mt->pos += dx;
    }

    test->initCudaParameters();

    qDebug() << "Parallel Masses" << test->masses.size() << "Springs" << test->springs.size();
    qDebug() << "Time" << test->time();


    // Run simulation
    double totalEnergy = 0;
    int steps = 0;
    equilibrium = false;
    prevEnergy.clear();
    while (!equilibrium) {
        totalEnergy = 0;
        for (Spring *s : test->springs) {
            totalEnergy += s->_curr_force * s->_curr_force / s->_k;
        }
        qDebug() << "ENERGY" << totalEnergy << prevEnergy.size();
        if (prevEnergy.size() > 4) {
            bool close = true;
            for (int p = 0; p < 6; p++) {
                int i = prevEnergy.size() - p - 1;
                if (fabs(prevEnergy[i] - totalEnergy) > totalEnergy * 1E-4)
                    close = false;
            }
            if (close) {
                equilibrium = true;
            }
        }
        prevEnergy.push_back(totalEnergy);
        if (prevEnergy.size() > 100) {
            prevEnergy.erase(prevEnergy.begin());
        }

        // Step simulation
        test->step(sim->masses.front()->dt * 100);
        test->getAll();
        steps++;
    }

    // Record metrics
    for (int j = 0; j < n_springs; j++) {
        Spring *s = test->springs[j];
        totalEnergySim += s->_curr_force * s->_curr_force / s->_k;
    }
    for (int n = 1; n < copies; n++) {
        totalEnergyTests.push_back(0);
        for (int j = n * n_springs; j < (n + 1) * n_springs; j++) {
            Spring *s = test->springs[j];
            totalEnergyTests.back() += s->_curr_force * s->_curr_force / s->_k;
        }
    }

    // Add optimizations
    int op = 0;
    for (int n = 1; n < copies; n++) {
        double simMetric = (totalEnergySim * totalLengthSim);
        double testMetric = (totalEnergyTests[n-1] * totalLengthTests[n-1]);
        double diff = simMetric - testMetric;
        qDebug() << "Total" << simMetric << testMetric << n << "Energy" << totalEnergySim << totalEnergyTests[n-1] << "Length" << totalLengthSim << totalLengthTests[n-1];
        if (diff > 0) {
            Mass *m = sim->masses[moved[n-1]];
            Vec dx = test->masses[moved[n-1] + n*n_masses]->origpos - m->origpos;
            m->origpos += dx;
            m->pos += dx;
            for (Spring *s : sim->springs) {
                if (s->_left == m || s->_right == m) {
                    double origLen = s->_rest;
                    s->_rest = (s->_left->origpos - s->_right->origpos).norm();
                    s->_k *= origLen / s->_rest;
                }
            }
            op++;
        }
    }
    qDebug() << "Moved" << op << "Masses";
    sim->setAll();
}

// Picks a random mass from a sim
//---------------------------------------------------------------------------
int MassDisplacer::pickRandomMass(Simulation *sim) {
//---------------------------------------------------------------------------

    int nm = sim->masses.size();

    // Pick a random mass
    int i = rand() % nm;
    bool underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
    bool fixed = sim->masses[i]->constraints.fixed;

    while (underExternalForce || fixed) {
        i = rand() % nm;
        underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
        fixed = sim->masses[i]->constraints.fixed;
    }

    return i;
}

// Picks a random mass from a sim that is over cutoff distance away
// from existing masses
//---------------------------------------------------------------------------
int MassDisplacer::getMassCandidate(Simulation *sim, vector<int> existingMasses, double cutoff) {
//---------------------------------------------------------------------------

    int i = -1;
    bool found = false;

    while(!found) {
        i = pickRandomMass(sim);
        Mass *m = sim->getMassByIndex(i);

        bool tooFar = false;
        for (int j : existingMasses) {
            Mass *e = sim->getMassByIndex(j);
            if (calcOrigDist(m,e) < cutoff) {
                tooFar = true;
                break;
            }
        }
        found = !tooFar;
    }

    return i;
}


// Shift mass at index by dx
//---------------------------------------------------------------------------
void MassDisplacer::shiftMassPos(Simulation *sim, int index, const Vec &dx) {
//---------------------------------------------------------------------------

    Mass *mt = sim->masses[index];
    for (Spring *s : sim->springs) {
        Vec orig = mt->origpos + dx;
        if (s->_left == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_right->origpos - orig).norm();
            s->_k *= origLen / s->_rest;
        }
        if (s->_right == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_left->origpos - orig).norm();
            s->_k *= origLen / s->_rest;
        }
    }

    mt->origpos += dx;
    mt->pos += dx;
    mt->vel = Vec(0, 0, 0);
    sim->setAll();
}


// Moves masses in the chunk by dx
//---------------------------------------------------------------------------
void MassDisplacer::shiftRandomChunk(Simulation *sim, const Vec &dx, vector<int> indices) {
//---------------------------------------------------------------------------

    for (int i : indices) {
        shiftMassPos(sim, i, dx);
    }
}

// Returns distance between original positions of two masses
//---------------------------------------------------------------------------
double MassDisplacer::calcOrigDist(Mass *m1, Mass *m2) {
//---------------------------------------------------------------------------

    return (m1->origpos - m2->origpos).norm();

}

// Moves a single mass in a random direction
// Creates serial Simulations and compares output
//---------------------------------------------------------------------------
int MassDisplacer::displaceSingleMass(double displacement, double chunkCutoff, int metricOrder) {
//---------------------------------------------------------------------------
    qDebug() << "Displacing mass";
    sim->getAll();

    n_springs = sim->springs.size();
    n_masses = sim->masses.size();

    // Pick a random mass
    int i = pickRandomMass(sim);
    Mass *mt = sim->masses[i];
    qDebug() << "Chose mass" << i;

    // Define chunk
    vector<int> chunk = vector<int>();
    chunk.push_back(i);
    if (chunkCutoff > 0) {
        for (int a = 0; a < sim->masses.size(); a++) {
            if (a != i) {
                Mass *m = sim->masses[a];
                if ((m->origpos - mt->origpos).norm() < chunkCutoff) {
                    chunk.push_back(a);
                }
            }
        }
        qDebug() << "Using chunk of size" << chunk.size();
    }
    // Define order group
    customMetric = QString();
    vector<Spring *> orderGroup = vector<Spring *>();
    vector<Mass *> orderMasses = vector<Mass *>();
    vector<Mass *> outsideGroup = vector<Mass *>();
    vector<Mass *> edgeGroup = vector<Mass *>();
    vector<Vec> addedForces = vector<Vec>();
    if (metricOrder > 0) {
        double metricCutoff = maxLocalization * metricOrder;

        for (Spring *s : sim->springs) {
            double ldist = calcOrigDist(s->_left, mt);
            double rdist = calcOrigDist(s->_right, mt);
            if (ldist <= metricCutoff && rdist <= metricCutoff) {
                orderGroup.push_back(s);
                orderMasses.push_back(s->_left);
                orderMasses.push_back(s->_right);
            } else if (ldist <= metricCutoff) {
                // Border spring: Left mass is within the order group, right mass is not
                edgeGroup.push_back(s->_left);
                outsideGroup.push_back(s->_right);
            } else if (rdist <= metricCutoff) {
                // Border spring: Right mass is within the order group, left mass is not
                edgeGroup.push_back(s->_right);
                outsideGroup.push_back(s->_left);
            } else {
                outsideGroup.push_back(s->_left);
                outsideGroup.push_back(s->_right);
            }
        }

        qDebug() << "Edge nodes" << edgeGroup.size();
        qDebug() << "Outside nodes" << outsideGroup.size();

        vector<Mass *> culledEdgeGroup = vector<Mass *>();
        vector<Mass *> culledOutsideGroup = vector<Mass *>();
        for (Mass *m : sim->masses) {
            if (find(edgeGroup.begin(), edgeGroup.end(), m) != edgeGroup.end()) {
                if (m->extforce == Vec(0,0,0)) {
                    culledEdgeGroup.push_back(m);
                }
            }
            if (find(outsideGroup.begin(), outsideGroup.end(), m) != outsideGroup.end()) {
                    if (!m->constraints.fixed) {
                        culledOutsideGroup.push_back(m);
                    }
            }
        }
        edgeGroup = culledEdgeGroup;
        outsideGroup = culledOutsideGroup;
        qDebug() << "Edge nodes" << edgeGroup.size();
        qDebug() << "Outside nodes" << outsideGroup.size();
    }

    // Record start positions
    vector<Vec> startPos = vector<Vec>();
    vector<Vec> origPos = vector<Vec>();
    vector<double> startRest = vector<double>();
    for (Mass *m : sim->masses) {
        startPos.push_back(m->pos);
        origPos.push_back(m->origpos);
    }
    for (Spring *s : sim->springs) {
        startRest.push_back(s->_rest);
    }
    for (Spring *t : sim->springs) {
        t->_broken = false;
    }

    for (Mass *m : edgeGroup) {
        double metricCutoff = maxLocalization * metricOrder;

        //if (m->pos[0] > mt->pos[0]) {
            Vec extForceApply = Vec(0, 0, 0);
            // Iterate over springs to find border connections
            for (Spring *t : sim->springs) {
                // Check for border springs
                if (t->_left == m && calcOrigDist(t->_right, mt) > metricCutoff) {
                //if (t->_left == m) {
                    // Left is an edge mass and right is an outside mass
                    extForceApply += t->_curr_force * (t->_left->pos - t->_right->pos).normalized();  // Add external forces
                }
                if (t->_right == m && calcOrigDist(t->_left, mt) > metricCutoff) {
                //if (t->_right == m) {
                    // Right is an edge mass and left is an outside mass
                    extForceApply += t->_curr_force * (t->_left->pos - t->_right->pos).normalized(); // Add external forces
                }
            }
            //addedForces.push_back(extForceApply);
            //m->extforce += extForceApply;
            //m->force = m->extforce;
            //m->extduration = DBL_MAX;

            qDebug() << QString("Pos: (%1,%2,%3), Force: (%4,%5,%6)")
                    .arg(m->pos[0])
                    .arg(m->pos[1])
                    .arg(m->pos[2])
                    .arg(m->force[0])
                    .arg(m->force[1])
                    .arg(m->force[2]);
        //}
        addedForces.push_back(m->extforce);
    }
    for (Mass *m : outsideGroup) {
        if (!m->constraints.fixed) {
            m->fix();
        }
    }
    sim->setAll();

    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation);
    }

    // Record start metrics
    double totalMetricSim = 0;
    double totalLengthSim = 0;
    double totalEnergySim = 0;

    if (metricOrder > 0)  {
        totalLengthSim = calcOrderLength(sim, orderGroup);
        totalEnergySim = calcOrderEnergy(sim, orderGroup);
    }
    else {
        totalLengthSim = calcTotalLength(sim);
        totalEnergySim = calcTotalEnergy(sim);
    }

    double totalMetricTest = 0;
    double totalLengthTest = 0;
    double totalEnergyTest = 0;




    // Pick a random direction
    Vec dir = Utils::randDirectionVec();
    qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
    Vec dx = displacement * dir;

    // Move mass
    shiftRandomChunk(sim, dx, chunk);

    // Run simulation
    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation, outsideGroup);
    }


    // Calculate test metrics
    if (metricOrder > 0) {
        totalLengthTest = calcOrderLength(sim, orderGroup);
        totalEnergyTest = calcOrderEnergy(sim, orderGroup);
    } else {
        totalLengthTest = calcTotalLength(sim);
        totalEnergyTest = calcTotalEnergy(sim);
    }

    totalMetricSim = totalEnergySim * totalLengthSim ;
    totalMetricTest = totalEnergyTest * totalLengthTest ;

    qDebug() << "Total lengths Sim" << totalLengthSim << " Test" << totalLengthTest;
    qDebug() << "Total energies Sim" << totalEnergySim << " Test" << totalEnergyTest;
    qDebug() << "Total metrics Sim" << totalMetricSim << " Test" << totalMetricTest;

    for (int e = 0; e < edgeGroup.size(); e++) {
        Mass *m = edgeGroup[e];
        //m->extforce -= addedForces[e];
        //m->force = Vec(0, 0, 0);
    }
    for (Mass *m : outsideGroup) {

        m->unfix();
    }

    if (totalMetricTest >= totalMetricSim) {
        setMassState(startPos);
        for (int m = 0; m < sim->masses.size(); m++) {
            sim->masses[m]->origpos = origPos[m];
        }
        for (int j = 0; j < n_springs; j++) {
            Spring *s = sim->springs[j];

            //if (s->_left == mt || s->_right == mt) {
                s->_k *= s->_rest / startRest[j];
                s->_rest = startRest[j];
                s->_max_stress = 0;
            //}
        }

        localEnergy = totalEnergySim;
        sim->setAll();
    } else {
        sim->setAll();
        qDebug() << "Moved" << i;
        localEnergy = totalEnergyTest;
        return 1;
    }

    return 0;
}

// Creates arrays of surrounding masses around a center mass
//---------------------------------------------------------------------------
void MassDisplacer::createMassGroup(Simulation *sim, double cutoff, Mass *center,
                                    MassDisplacer::MassGroup &massGroup) {
//---------------------------------------------------------------------------

        massGroup.displaced = center;
        massGroup.group = vector<Mass *>();
        massGroup.springs = vector<Spring *>();
        massGroup.outside = vector<Mass *>();

        for (Spring *s : sim->springs) {
            double ldist = calcOrigDist(s->_left, center);
            double rdist = calcOrigDist(s->_right, center);
            if (ldist <= cutoff && rdist <= cutoff) {
                massGroup.springs.push_back(s);
                massGroup.group.push_back(s->_left);
                massGroup.group.push_back(s->_right);
            } else if (ldist <= cutoff) {
                // Border spring: Left mass is within the order group, right mass is not
                massGroup.outside.push_back(s->_right);
            } else if (rdist <= cutoff) {
                // Border spring: Right mass is within the order group, left mass is not
                massGroup.outside.push_back(s->_left);
            }
        }

        vector<Mass *> culledOrderGroup = vector<Mass *>();
        vector<Mass *> culledOutsideGroup = vector<Mass *>();
        for (Mass *m : sim->masses) {
            if (find(massGroup.group.begin(), massGroup.group.end(), m) != massGroup.group.end()) {
                culledOrderGroup.push_back(m);
            }
            if (find(massGroup.outside.begin(), massGroup.outside.end(), m) != massGroup.outside.end()) {
                culledOutsideGroup.push_back(m);
            }
        }
        massGroup.group = culledOrderGroup;
        massGroup.outside = culledOutsideGroup;

}

// Moves num masses in random directions by displacement
// Creates serial Simulations and compares output
//---------------------------------------------------------------------------
int MassDisplacer::displaceManyMasses(double displacement, int metricOrder, int num) {
//---------------------------------------------------------------------------
    qDebug() << "Displacing masses";
    int displaced = 0;

    if (num < 1) {
        qDebug() << "Must run displaceManyMasses with more than one mass";
        return -1;
    }
    sim->getAll();

    n_springs = sim->springs.size();
    n_masses = sim->masses.size();
    for (Spring *t : sim->springs) {
        t->_broken = false;
    }
    sim->setAll();

    // Vectors with trial data
    vector<int> chosenMasses = vector<int>();
    vector<MassGroup> chosenLocalities = vector<MassGroup>();

    // Pick an initial random mass
    int i = pickRandomMass(sim);
    chosenMasses.push_back(i);
    qDebug() << "Chose mass" << i;

    // Pick remaining Masses
    for (int n = 1; n < num; n++) {
        int j = getMassCandidate(sim, chosenMasses, maxLocalization * metricOrder * 2);
        chosenMasses.push_back(j);
    }

    // Create mass groups
    for (int m : chosenMasses) {
        MassGroup mg;
        createMassGroup(sim, maxLocalization * metricOrder, sim->getMassByIndex(m), mg);
        chosenLocalities.push_back(mg);
    }

    // Record start values
    vector<Vec> startPos = vector<Vec>();
    vector<Vec> origPos = vector<Vec>();
    vector<double> startRest = vector<double>();
    for (Mass *m : sim->masses) {
        startPos.push_back(m->pos);
        origPos.push_back(m->origpos);
    }
    for (Spring *s : sim->springs) {
        startRest.push_back(s->_rest);
    }

    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation);
    }

    for (MassGroup &mg : chosenLocalities) {
        // Record start metrics
        mg.origLength = calcOrderLength(sim, mg.springs);
        mg.origEnergy = calcOrderEnergy(sim, mg.springs);
    }

    // Displace masses
    for (int m : chosenMasses) {
        // Pick random directions
        Vec dir = Utils::randDirectionVec();
        qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
        Vec dvec = displacement * dir;

        shiftMassPos(sim, m, dvec);
    }

    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation);
    }

    for (MassGroup &mg : chosenLocalities) {
        // Calculate test metrics
        mg.testLength = calcOrderLength(sim, mg.springs);
        mg.testEnergy = calcOrderEnergy(sim, mg.springs);

        double totalMetricOrig = mg.origEnergy * mg.origLength;
        double totalMetricTest = mg.testEnergy * mg.testLength;

        qDebug() << "---------------- Mass" << mg.displaced->index << "-------------";
        qDebug() << "Total lengths Orig" << mg.origLength << " Test" << mg.testLength;
        qDebug() << "Total energies Orig" << mg.origEnergy << " Test" << mg.testEnergy;
        qDebug() << "Total metrics Orig" << totalMetricOrig << " Test" << totalMetricTest;


        if (totalMetricTest >= totalMetricOrig) {
            setMassState(startPos);
            for (int m = 0; m < sim->masses.size(); m++) {
                sim->masses[m]->origpos = origPos[m];
            }
            for (int j = 0; j < n_springs; j++) {
                Spring *s = sim->springs[j];

                s->_k *= s->_rest / startRest[j];
                s->_rest = startRest[j];
                s->_max_stress = 0;
            }
            sim->setAll();

        } else {
            for (Spring *s : mg.springs) {
                s->_broken = true;
            }
            sim->setAll();
            qDebug() << "Moved" << mg.displaced->index;
            displaced++;
        }

    }
    return displaced;
}


// Initialize array of concurrent simulation clones
//---------------------------------------------------------------------------
void MassDisplacer::initializeClones(int n) {
//---------------------------------------------------------------------------

    clones.resize(n);
    for (int i = 0; i < n; i++) {
        clones[i] = Clone(new Simulation);
        clones[i].sim->copy(*sim);
        clones[i].sim->initCudaParameters();
    }
    qDebug() << "Initialized" << clones.size() << "Clones";

}


// Concurrently displace a single mass in each clone by dx
//---------------------------------------------------------------------------
void MassDisplacer::mutateClones(double dx) {
//---------------------------------------------------------------------------

#pragma omp parallel for
    for (int i = 0; i < clones.size(); i++) {
        qDebug() << "Mutating clone" << i;
        shiftCloneMass(&clones[i], dx);
        int steps = settleSim(clones[i].sim, 1E-4);
        qDebug() << "Settled" << i << "in" << steps << "steps";
    }

}


// Compare energy of clones to energy of sim
// Add displacements from clones with lower energy
//---------------------------------------------------------------------------
void MassDisplacer::incorporateClones() {
//---------------------------------------------------------------------------

    double controlEnergy = calcTotalEnergy(sim);
    for (Clone c : clones) {
        if (calcTotalEnergy(c.sim) < controlEnergy) {
            // Good displacement

            // Incorporate displacement structs
            for (auto ds : c.adjustedSprings) {
                Spring *s = sim->springs[ds.index];
                s->_rest = ds.rest;
                s->_k = ds.constant;
            }

            Mass *m = sim->masses[c.displacedMass.index];
            m->origpos += c.displacedMass.displacement;
        }
    }
    sim->setAll();

}



// Resets clones to mirror sim
//---------------------------------------------------------------------------
void MassDisplacer::resetClones() {
//---------------------------------------------------------------------------

#pragma omp parallel for
    for (int i = 0; i < clones.size(); i++) {
        // Reset mass
        int im = clones[i].displacedMass.index;
        Mass *cm = clones[i].sim->masses[im];
        Mass *sm = sim->masses[im];
        cm->origpos = sm->origpos;

        // Reset springs
        for (auto ds : clones[i].adjustedSprings) {
            int is = ds.index;
            Spring *cs = clones[i].sim->springs[is];
            Spring *ss = sim->springs[is];
            cs->_rest = ss->_rest;
            cs->_k = ss->_k;
        }
        clones[i].sim->setAll();
    }
}


// Shifts a mass's original position and updates connected springs
// Return structs with data of adjusted springs
//---------------------------------------------------------------------------
vector<MassDisplacer::DisplacedSpring> MassDisplacer::shiftOrigPos(Simulation *sim, Mass *m, const Vec &p) {
//---------------------------------------------------------------------------

    vector<DisplacedSpring> adjusted = vector<DisplacedSpring>();
    m->origpos = p;
    for (int i = 0; i < sim->springs.size(); i++)  {
        Spring *s = sim->springs[i];
        if (s->_right == m || s->_left == m) {

            double oldRest = s->_rest;
            s->_rest = (s->_left->origpos - s->_right->origpos).norm();
            s->_k *= oldRest / s->_rest;

            DisplacedSpring ds = DisplacedSpring(i, s->_rest, s->_k);
            adjusted.push_back(ds);
        }
    }
    return adjusted;

}

// Calculate total rest length of all springs in a simulation
//---------------------------------------------------------------------------
double MassDisplacer::calcTotalLength(Simulation *sim){
//---------------------------------------------------------------------------

    double length = 0;
    for (Spring *s : sim->springs) {
        length += s->_rest;
    }
    return length;

}

// Calculate rest length of simulation of order n springs around mass m
//---------------------------------------------------------------------------
double MassDisplacer::calcOrderLength(Simulation *sim, vector<Spring *> group) {
//---------------------------------------------------------------------------

    double length = 0;
    for (Spring *s : group) {
        length += s->_rest;
    }
    return length;
}


// Calculate energy of a simulation
//---------------------------------------------------------------------------
double MassDisplacer::calcTotalEnergy(Simulation *sim) {
//---------------------------------------------------------------------------

    double energy = 0;
    for (Spring *s : sim->springs) {
        energy += s->_curr_force * s->_curr_force / s->_k;
    }
    return energy;

}

// Caculate energy of simulation of order n springs around mass m
//---------------------------------------------------------------------------
double MassDisplacer::calcOrderEnergy(Simulation *sim, vector<Spring *> group) {
//---------------------------------------------------------------------------

    double energy = 0;

    int measured = 0;

    for (Spring *s : group) {
        s->_broken = true;
        energy += s->_curr_force * s->_curr_force / s->_k;
        measured++;
    }

    qDebug() << "Energy from surrounding" << measured << "springs is" << energy;
    return energy;
}


// Run simulation until it reaches mechanical equilibrium within eps
//---------------------------------------------------------------------------
int MassDisplacer::settleSim(Simulation *sim, double eps, bool use_cap, double cap) {
//---------------------------------------------------------------------------

    equilibrium = false;
    double totalEnergy = 0;
    double prevTotalEnergy = 0;
    int closeToPrevious = 0;
    int steps = 0;
    while (!equilibrium) {
        totalEnergy = 0;
        for (Spring *s : sim->springs) {
            totalEnergy += s->_curr_force * s->_curr_force / s->_k;
        }
        qDebug() << "ENERGY" << totalEnergy << prevTotalEnergy << closeToPrevious;

        if (prevTotalEnergy > 0 && fabs(prevTotalEnergy - totalEnergy) < totalEnergy * eps) {
            closeToPrevious++;
        } else {
            closeToPrevious = 0;
        }
        if (closeToPrevious > 10) {
            equilibrium = true;
        }
        if (use_cap) {
            if (totalEnergy > cap && steps > 50) {
                equilibrium = true;
            }
        }
        prevTotalEnergy = totalEnergy;

        // Step simulation
        sim->step(sim->masses.front()->dt * 100);
        sim->getAll();
        steps++;
    }

    return steps;

}

// Relax simulation for steps amount of time
//---------------------------------------------------------------------------
void MassDisplacer::relaxSim(Simulation *sim, int steps, vector<Mass *> track) {
//---------------------------------------------------------------------------

        if (track.empty()) {
            // Step simulation
            sim->step(sim->masses.front()->dt * steps);
            sim->getAll();
        } else {
            sim->getAll();
            QTextStream metricStream(&customMetric);
            int n = 0;
            for (Mass *m : track) {
                Vec force = m->acc * m->m;
                metricStream << 0 << ',';
                metricStream << m->pos[0] << ',' << m->pos[1] << ',' << m->pos[2] << ',';
                metricStream << force[0] << ',' << force[1] << ',' << force[2] << ',';
                metricStream << n << '\n';
                n++;
            }
            for (int i = 0; i < steps; i++) {
                sim->step(sim->masses.front()->dt);
                sim->getAll();
                QTextStream metricStream(&customMetric);
                n = 0;
                for (Mass *m : track) {
                    Vec force = m->acc * m->m;
                    metricStream << i+1 << ',';
                    metricStream << m->pos[0] << ',' << m->pos[1] << ',' << m->pos[2] << ',';
                    metricStream << force[0] << ',' << force[1] << ',' << force[2] << ',';
                    metricStream << n << '\n';
                    n++;
                }
            }
        }
}

// Pick a random mass and shift in a random direction by dx
// Record shift in clone struct
//---------------------------------------------------------------------------
void MassDisplacer::shiftCloneMass(MassDisplacer::Clone *clone, double dx){
//---------------------------------------------------------------------------

    int nm = clone->sim->masses.size();

    // Pick random mass
    int r = rand() % nm;
    Mass *m = clone->sim->masses[r];

    // Pick random direction and scale by dx
    Vec d = dx * Utils::randDirectionVec();

    // Shift mass
    m->pos += d;
    vector<DisplacedSpring> adjustedSprings = shiftOrigPos(clone->sim, m, m->origpos + d);

    // Record shift
    clone->displacedMass.index = r;
    clone->displacedMass.displacement = d;
    clone->adjustedSprings = adjustedSprings;

    clone->sim->setAll();
}


// Sets mass positions via a vector input
//---------------------------------------------------------------------------
void MassDisplacer::setMassState(const vector<Vec> &pos){
//---------------------------------------------------------------------------

    // Make sure vector maps one-to-one to masses
    assert(pos.size() == sim->masses.size());

    for (int i = 0; i < pos.size(); i++) {
        sim->masses[i]->pos = pos[i];
    }

}


//---------------------------------------------------------------------------
//  SPRING INSERTER
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
SpringInserter::SpringInserter(Simulation *sim, double addRatio)
    : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->stepRatio = addRatio;
}


// Inserts stepRatio percent springs
//---------------------------------------------------------------------------
void SpringInserter::optimize() {
//---------------------------------------------------------------------------

    sim->getAll();
    n_springs = sim->springs.size();

    vector<uint> springIndicesToSort;
    sortSprings_stress(springIndicesToSort);

    int added = 0;
    double kFactor = sim->springs.front()->_k * sim->springs.front()->_rest;

    uint toAdd = uint(stepRatio * sim->springs.size()) + 1;
    qDebug() << "Adding around" << toAdd << "springs";
    vector<Spring *> springsToAdd = vector<Spring *>();
    for (uint j = springIndicesToSort.size() - 1; j >= springIndicesToSort.size() - toAdd; j--) {
        if (j > 0) {
            vector<Mass *> massLocs = vector<Mass *>();
            braceSpring(sim->springs[springIndicesToSort[j]], massLocs);

            qDebug() << "Found" << massLocs.size() / 2 << "potential insertion points";

            for (int i = 0; i < massLocs.size(); i += 2) {
                Spring *s = new Spring(*sim->springs.front()); // Get all of the constants from existing spring
                s->setMasses(massLocs[i], massLocs[i+1]);
                springsToAdd.push_back(s);
                added++;
            }
        }
        // Reset max stress
        //sim->springs[springIndicesToSort[j]]->_max_stress = 0;
    }

    /**for (Spring *s : springsToAdd) {
        sim->springs.push_back(s);
        sim->createSpring(s);
        sim->springs.back()->_k = kFactor / sim->springs.back()->_rest;
        sim->springs.back()->_max_stress = 0;
        sim->setAll();
    }**/
    qDebug() << "Inserted" << added << "Springs";

    sim->setAll();
    n_springs = int(sim->springs.size());
}


// Finds second degree locations to add a spring
// Searches around stressedSpring
// Appends resulting Mass pairs to locations vector
//---------------------------------------------------------------------------
void SpringInserter::findPlacesToAddSpring(Spring *stressedSpring, vector<Mass *> &locations) {
//---------------------------------------------------------------------------

    Mass *m1 = stressedSpring->_left;
    Mass *m2 = stressedSpring->_right;
    vector<Mass *> left_so = vector<Mass *>();
    vector<Mass *> right_so = vector<Mass *>();

    Mass *option = nullptr;
    for (Spring *s : sim->springs) {

        if (s != stressedSpring) {
            if (s->_left == m1|| s->_left == m2) {
                option = s->_right;
                if (find(left_so.begin(), left_so.end(), option) == left_so.end()) {
                    left_so.push_back(option);
                }
            }
            if (s->_right == m1 || s->_right == m2) {
                option = s->_left;
                if (find(right_so.begin(), right_so.end(), option) == right_so.end()) {
                    right_so.push_back(option);
                }
            }
        }
    }
    //qDebug() << "Found" << secondOrderMasses.size() << "second order masses";

    // Add connections
    if (!left_so.empty() && !right_so.empty()) {
        for (int i = 0; i < left_so.size(); i++) {
            for (int j = 0; j < right_so.size(); j++) {

                if ((left_so[i]->pos - right_so[j]->pos).norm() > this->cutoff) {
                    continue;
                }

                bool connected = false;
                for (Spring *t : sim->springs) {
                    if (t->_left == left_so[i] && t->_right == right_so[j]) {
                        connected = true;
                    }
                    if (t->_left == right_so[j] && t->_right == left_so[i]) {
                        connected = true;
                    }
                }

                // If a spring does not already exist there, add to vector
                if (!connected) {
                    locations.push_back(left_so[i]);
                    locations.push_back(right_so[j]);
                }
            }
        }
    }
    // TODO: Fix bug with removal
    // TODO: Sweep springs and join lone bisected pairs -.- = ---

    //qDebug() << "Done with finding second order masses";
}

// Finds second degree locations to add a spring
// Bisects springs around stressedSpring and checks orientation of new masses
// Appends resulting Mass pairs to locations vector
//---------------------------------------------------------------------------
void SpringInserter::braceSpring(Spring *stressedSpring, vector<Mass *> &locations) {
//---------------------------------------------------------------------------

    // Find second order springs
    Mass *m1 = stressedSpring->_left;
    Mass *m2 = stressedSpring->_right;
    Vec svec = m1->pos - m2->pos;
    vector<Spring *> springs_so = vector<Spring *>();
    vector<Mass *> masses_so = vector<Mass *>();

    for (Spring *s : sim->springs) {

        bool underExternalForce = s->_left->extforce.norm() > 1E-6
                                  && s->_right->extforce.norm() > 1E-6;
        bool fixed = s->_left->constraints.fixed && s->_right->constraints.fixed;

        if (s != stressedSpring && !underExternalForce && !fixed) {
            if (s->_left == m1 || s->_left == m2 || s->_right == m1 || s->_right == m2) {
                springs_so.push_back(s);
            }
        }

        if (s != stressedSpring) {
            if (s->_right == m1 || s->_right == m2) {
                masses_so.push_back(s->_left);
            }
            if (s->_left == m1 || s->_left == m2) {
                masses_so.push_back(s->_right);
            }
        }
    }
    qDebug() << springs_so.size() << "second order springs";

    // Bisect springs
    vector<Vec> mids = vector<Vec>();
    vector<Vec> omids = vector<Vec>(); // Original positions for mids
    vector<Mass *> midUsed = vector<Mass *>();
    for (Spring *so : springs_so) {
        mids.push_back(Utils::bisect(so->_left->pos, so->_right->pos));
        omids.push_back(Utils::bisect(so->_left->origpos, so->_right->origpos));
    }

    sim->pause(sim->time());

    // Create connections
    double halfcutoff = stressedSpring->_rest / 2;
    double pi = atan(1.0)*4;
    int added = 0;
    for (int i = 0; i < mids.size() - 1; i++) {
        for (int j = i + 1; j < mids.size(); j++) {

            Vec mvec = mids[i] - mids[j];
            double angle = Utils::getAngle(mvec, svec);

            if (mvec.norm() <= halfcutoff * 2 && angle <= pi/4) {

                Mass *n = nullptr;
                Mass *o = nullptr;
                sim->getAll();
                for (Mass *m : sim->masses) {
                    if (m->pos == mids[i]) {
                        n = m;
                    }
                    if (m->pos == mids[j]) {
                        o = m;
                    }
                }

                if (n == nullptr) {
                    n = sim->createMass(mids[i]);
                    n->origpos = omids[i];

                    /**sim->springs.push_back(sim->createSpring(springs_so[i]->_left, n));
                    sim->springs.back()->_rest = springs_so[i]->_rest / 2;
                    sim->springs.back()->_k = springs_so[i]->_k * 2;
                    sim->springs.back()->_diam = springs_so[i]->_diam;
                    sim->springs.back()->_break_force = springs_so[i]->_break_force;**/

                    bisectSpring(springs_so[i], n);
                    assert(n->spring_count == 2);

                    midUsed.push_back(n);
                }
                if (o == nullptr) {
                    o = sim->createMass(mids[j]);
                    o->origpos = omids[j];

                    bisectSpring(springs_so[j], o);
                    assert(o->spring_count == 2);

                    midUsed.push_back(o);
                }

                // Create connection between midpoints
                Spring *tmp = sim->springs.front();
                Spring *b = new Spring(*tmp);
                b->setMasses(n, o);
                b->_rest = (n->origpos - o->origpos).norm();
                b->_k *= tmp->_rest / b->_rest;
                sim->createSpring(b);

                sim->setAll();
                added++;

            }
        }
    }

    // Add periphary springs
    for (Mass *p : midUsed) {
        for (Mass *so : masses_so) {
            if (so != p) {
                Vec v = so->origpos - p->origpos;
                double angle = Utils::getAngle(v, svec);

                if (v.norm() <= halfcutoff) {
                    Spring *tmp = sim->springs.front();

                    Spring *s = new Spring(*tmp);
                    s->setMasses(so, p);
                    s->_rest = v.norm();
                    s->_k *= tmp->_rest / s->_rest;
                    sim->createSpring(s);
                    sim->setAll();
                    added++;
                }
            }
        }
    }
    stressedSpring->_max_stress = 0;
    qDebug() << "Added" << added << "springs";

    // Combine springs that have been optimized out
    int combined = combineParallelSprings();
    qDebug() << "Combined springs" << combined;
}

// Combines parallel springs joined by a mass with no other springs attached
// Returns number of springs combined
//---------------------------------------------------------------------------
int SpringInserter::combineParallelSprings() {
//---------------------------------------------------------------------------

    int combined = 0;
    double pi = atan(1.0) * 4;
    for (int s = 0; s < n_springs - 1; s++) {
        for (int t = s + 1; t < n_springs; t++) {
            Spring *a = sim->springs[s];
            Spring *b = sim->springs[t];

            Mass *com = nullptr;
            if (a->_left == b->_left || a->_left == b->_right) {
                com = a->_left;
            }
            if (a->_right == b->_right || a->_right == b->_left) {
                com = a->_right;
            }
            if (com == nullptr || com->spring_count != 2) {
                // Springs do not share a mass
                // Or mass does not have exactly 2 springs attached
                continue;
            }

            Vec av = a->_left->pos - a->_right->pos;
            Vec bv = b->_left->pos - b->_right->pos;

            double angle = Utils::getAngle(av, bv);
            if (angle >= pi - 1E-4 || angle <= 1E-4) {
                // Close to parallel
                joinSprings(a,b);
                combined++;
            }
        }
    }

    return combined;
}

// Bisects a spring into two springs
// New middle mass must be provided
//---------------------------------------------------------------------------
void SpringInserter::bisectSpring(Spring *s, Mass *mid) {
//---------------------------------------------------------------------------

qDebug() << "Bisecting spring";
    Mass *l = s->_left;
    Mass *r = s->_right;

    assert(l != mid && r != mid);

    // Set mass parameters
    mid->m = l->m / l->spring_count + r->m / r->spring_count;
    mid->dt = l->dt;

    // Form left spring by shortening existing spring
    s->setMasses(l, mid);
    s->_rest *= 0.5;
    s->_k *= 2;
    r->spring_count--;
    mid->spring_count++;
    qDebug() << "Created spring 1";

    // Create a new springs for right spring
    Spring *rs = new Spring(*s);
    rs->setMasses(mid, r);
    qDebug() << "About to create spring";
    sim->createSpring(rs);
    qDebug() << "Created spring 2";

    sim->setAll();
}

// Joins two springs sharing a mass
//---------------------------------------------------------------------------
void SpringInserter::joinSprings(Spring *s1, Spring *s2) {
//---------------------------------------------------------------------------

    Mass *com = nullptr, *sep1 = nullptr, *sep2 = nullptr;

    if (s1->_left == s2->_left) {
        com = s1->_left;
        sep1 = s1->_right;
        sep2 = s2->_right;
    }
    if (s1->_left == s2->_right) {
        com = s1->_left;
        sep1 = s1->_right;
        sep2 = s2->_left;
    }
    if (s1->_right == s2->_right) {
        com = s1->_right;
        sep1 = s1->_left;
        sep2 = s2->_left;
    }
    if (s1->_right == s2->_left) {
        com = s1->_right;
        sep1 = s1->_left;
        sep2 = s2->_right;
    }

    assert(com != nullptr);
    assert(!(sep1->pos == sep2->pos));
    assert(com->spring_count == 2);

    int sc1 = sep1->spring_count;
    int sc2 = sep2->spring_count;

    // Form long spring by extending one existing spring
    Vec v = sep1->pos - sep2->pos;
    s1->setMasses(sep1, sep2);
    s1->_k *= s1->_rest / v.norm();
    s1->_rest = v.norm();
    sep2->spring_count++;

    // Delete remaining spring
    sim->deleteSpring(s2);

    // Verify mass has been deleted
    assert(!com->valid);
    assert(sep1->spring_count == sc1);
    assert(sep2->spring_count == sc2);

    sim->setAll();
}