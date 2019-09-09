//
// Created by sw3390 on 4/24/19.
//

#include "optimizer.h"

// Returns index of the spring with the minimum max stress
//---------------------------------------------------------------------------
uint Optimizer::minSpringByStress() {
//---------------------------------------------------------------------------

    uint msi = -1;
    double minStress = FLT_MAX;
    for (uint s = 0; s < sim->springs.size(); s++) {
        bool underExternalForce = sim->springs[s]->_left->extforce.norm() > 1E-6
                                  && sim->springs[s]->_right->extforce.norm() > 1E-6;
        bool fixed = sim->springs[s]->_left->constraints.fixed && sim->springs[s]->_right->constraints.fixed;
        double force = sim->springs[s]->_max_stress;

        if (!underExternalForce && !fixed) {
            if (force < minStress) {
                minStress = force;
                msi = s;
            }
        }
    }

    return msi;
}

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

    // Fill mass to spring map
    for (Mass *m : sim->masses) {

        massToSpringMap[m] = vector<Spring *> ();

        for (Spring *s : sim->springs) {
            if (m == s->_left || m == s->_right) {
                massToSpringMap[m].push_back(s);
            }
        }
    }
}


//---------------------------------------------------------------------------
void SpringRemover::removeSpringFromMap(Spring *d) {
//---------------------------------------------------------------------------

    auto &m1 = massToSpringMap[d->_left];
    m1.erase(remove(m1.begin(), m1.end(), d), m1.end());

    auto &m2 = massToSpringMap[d->_right];
    m2.erase(remove(m2.begin(), m2.end(), d), m2.end());

}


// Removes stepRatio percent least stressed springs
//---------------------------------------------------------------------------
void SpringRemover::optimize() {
//---------------------------------------------------------------------------

    sim->getAll();
    n_springs = sim->springs.size();

    if (n_springs > n_springs_start * stopRatio) {
        map<Spring *, bool> springsToDelete = map<Spring *, bool>();
        map<Spring *, bool> hangingCandidates = map<Spring *, bool>();

        uint toRemove = stepRatio > 0 ?  uint(stepRatio * sim->springs.size()): 1;

        if (toRemove > 1) {
            vector<uint> springIndicesToSort;
            sortSprings_stress(springIndicesToSort);

            for (Spring *s : sim->springs) {
                springsToDelete[s] = false;
            }
            for (uint j = 0; j < toRemove; j++) {
                if (j < springIndicesToSort.size()) {
                    Spring *d = sim->springs[springIndicesToSort[j]];
                    springsToDelete[d] = true;
                    removeSpringFromMap(d);

                    for (Spring *c : massToSpringMap[d->_left]) {
                        hangingCandidates[c] = true;
                    }
                    for (Spring *c : massToSpringMap[d->_right]) {
                        hangingCandidates[c] = true;
                    }
                }
            }
        } else {
            uint ms = minSpringByStress();
            springsToDelete[sim->springs[ms]] = true;
            removeSpringFromMap(sim->springs[ms]);

            for (Spring *c : massToSpringMap[sim->springs[ms]->_left]) {
                hangingCandidates[c] = true;
            }
            for (Spring *c : massToSpringMap[sim->springs[ms]->_right]) {
                hangingCandidates[c] = true;
            }
        }
        qDebug() << "Removing" << toRemove << "Springs";


        // Remove hanging springs (attached to masses with only one attached spring
        int hangingSprings = 0;
        while (!hangingCandidates.empty()) {
            qDebug() << "Hanging spring candidates" << hangingCandidates.size();
            map<Spring *, bool> newCandidates = map<Spring *, bool>();
            for (auto hc : hangingCandidates) {
                Spring *s = hc.first;
                if (!springsToDelete[s] && s != nullptr) {
                    if (massToSpringMap[s->_left].size()  == 1) {
                        if (!springsToDelete[s]) hangingSprings++;
                        springsToDelete[s] = true;
                        removeSpringFromMap(s);

                        // Add connected springs
                        for (Spring *c : massToSpringMap[s->_right]) {
                            if (c != s) newCandidates[c] = true;
                        }
                    }
                    if (massToSpringMap[s->_right].size() == 1) {
                        if (!springsToDelete[s]) hangingSprings++;
                        springsToDelete[s] = true;
                        removeSpringFromMap(s);

                        // Add connected springs
                        for (Spring *c : massToSpringMap[s->_left]) {
                            if (c != s) newCandidates[c] = true;
                        }
                    }

                    // For 2 attached springs, determine angle between them
                    if (massToSpringMap[s->_left].size() == 2) {
                        for (Spring *h : massToSpringMap[s->_left]) {
                            if (h != s) {
                                // h and s might be part of a hanging pair
                                Vec bar1 = s->_right->pos - s->_left->pos;
                                Vec bar2 = h->_right->pos - h->_left->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    if (!springsToDelete[s]) hangingSprings++;
                                    if (!springsToDelete[h]) hangingSprings++;
                                    springsToDelete[s] = true;
                                    springsToDelete[h] = true;
                                    removeSpringFromMap(s);
                                    removeSpringFromMap(h);

                                    // Add connected springs
                                    if (s->_left == h->_left) {
                                        for (Spring *c : massToSpringMap[h->_right]) {
                                            if (c != h) newCandidates[c] = true;
                                        }
                                    }
                                    if (s->_left == h->_right) {
                                        for (Spring *c : massToSpringMap[h->_left]) {
                                            if (c != h) newCandidates[c] = true;
                                        }
                                    }
                                    for (Spring *c : massToSpringMap[s->_right]) {
                                        if (c != s) newCandidates[c] = true;
                                    }
                                }
                            }
                        }
                    }
                    if (massToSpringMap[s->_right].size() == 2) {
                        for (Spring *h : massToSpringMap[s->_right]) {
                            if (h != s) {
                                // h and s might be part of a hanging pair
                                Vec bar1 = s->_right->pos - s->_left->pos;
                                Vec bar2 = h->_right->pos - h->_left->pos;
                                if (Utils::isAcute(bar1, bar2)) {
                                    if (!springsToDelete[s]) hangingSprings++;
                                    if (!springsToDelete[h]) hangingSprings++;
                                    springsToDelete[s] = true;
                                    springsToDelete[h] = true;
                                    removeSpringFromMap(s);
                                    removeSpringFromMap(h);

                                    // Add connected springs
                                    if (s->_right == h->_left) {
                                        for (Spring *c : massToSpringMap[h->_left]) {
                                            if (c != h) newCandidates[c] = true;
                                        }
                                    }
                                    if (s->_right == h->_right) {
                                        for (Spring *c : massToSpringMap[h->_right]) {
                                            if (c != h) newCandidates[c] = true;
                                        }
                                    }
                                    for (Spring *c : massToSpringMap[s->_left]) {
                                        if (c != s) newCandidates[c] = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            qDebug() << "Hanging springs" << hangingSprings;
            qDebug() << "New candidates" << newCandidates.size();
            hangingCandidates.clear();
            hangingCandidates = newCandidates;
        }

        // Remove springs
        uint i = 0;
        while (i < sim->springs.size()) {
            if (sim->springs[i] != nullptr && springsToDelete[sim->springs[i]]) {
                sim->deleteSpring(sim->springs[i]);
                i--;
            }
            i++;
        }
        for (auto ms : massToSpringMap) {
            ms.second.erase(remove(ms.second.begin(), ms.second.end(), nullptr), ms.second.end());
        }
        qDebug() << "Deleted springs";


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
MassDisplacer::MassDisplacer(Simulation *sim, double dx, double displaceRatio, double massFactor)
    : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->stepRatio = displaceRatio;
    this->dx = dx;
    this->massFactor = massFactor;
    this->order = 0;
    this->chunkSize = -1;
    this->relaxation = 0;
    this->maxLocalization = 0;
    this->iterations = 0;
    this->attempts = 0;
    this->prevAttemptNums = vector<int>();
    this->maxAvgSuccessRate = 0;
    this->lastTune = 0;
    this->lastMetric = 0;
    test = new Simulation();
    start = new Simulation();

    this->maxLocalization = 0;

    customMetricHeader = "Time,Position(x),Position(y),Position(z),Force(x),Force(y),Force(z),Index\n";

    trenchGrid.startCorner = Vec(-0.3, -0.05, -0.05);
    trenchGrid.endCorner = Vec(0.3, 0.05, 0.05);
    trenchGrid.dimension = Vec(0.15, 0.1, 0.1);
    massGroups = vector<MassGroup>();

    createMassGroup(this->sim, Vec(-0.4, -0.05, -0.05), Vec(-0.05, 0.05, 0.05), this->massGroup);
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
        //displaced = displaceGroupMass(dx);
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

    if (!STARTED) STARTED = true;
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

// Pick a random mass form a mass group
//---------------------------------------------------------------------------
int MassDisplacer::pickRandomMass(MassDisplacer::MassGroup &group) {
//---------------------------------------------------------------------------

    int nm = massGroup.group.size();

    // Pick a random mass
    int i = rand() % nm;
    bool underExternalForce = massGroup.group[i]->extforce.norm() > 1E-6;
    bool fixed = massGroup.group[i]->constraints.fixed;

    while (underExternalForce || fixed) {
        i = rand() % nm;
        underExternalForce = massGroup.group[i]->extforce.norm() > 1E-6;
        fixed = massGroup.group[i]->constraints.fixed;
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

// Returns true if a spring exists in sim that connects m1 and m2
//---------------------------------------------------------------------------
bool MassDisplacer::springExists(Simulation *sim, Mass *m1, Mass *m2) {
//---------------------------------------------------------------------------

    for (Spring *s : sim->springs) {
        if ((s->_left == m1 && s->_right == m2) || (s->_left == m2 && s->_right == m1)) {
            return true;
        }
    }
    return false;

}


// Merge Mass m2 onto Mass m1 with Spring c as the connecting spring of length close to 0 to be deleted
//---------------------------------------------------------------------------
void MassDisplacer::mergeMasses(Simulation *sim, Mass *m1, Mass *m2, Spring *c) {
//---------------------------------------------------------------------------

    assert((c->_left == m1 && c->_right == m2) || (c->_left == m2 && c->_right == m1));
    // Delete c
    sim->deleteSpring(c);

    // Find springs connected to m2
    for (int i = 0; i < sim->springs.size(); i++) {
        Spring *s = sim->springs[i];

        if (s->_left == m2) {
            if (springExists(sim, s->_right, m1)) {
                sim->deleteSpring(s);
                i--;
                continue;
            } else {
                double origLen = s->_rest;
                s->setLeft(m1);
                s->_rest = (s->_right->origpos - m2->origpos).norm();
                s->_k *= origLen / s->_rest;
            }
        }
        if (s->_right == m2) {
            if (springExists(sim, s->_left, m1)) {
                sim->deleteSpring(s);
                i--;
                continue;
            } else {
                double origLen = s->_rest;
                s->setRight(m1);
                s->_rest = (s->_left->origpos - m2->origpos).norm();
                s->_k *= origLen / s->_rest;
            }
        }
    }

    // Set mass
    m1->m += m2->m;

    sim->setAll();
}


// Shift mass at index by dx
// Fills merged with masses with springs that have been deleted through merge
//---------------------------------------------------------------------------
int MassDisplacer::shiftMassPos(Simulation *sim, int index, const Vec &dx, vector<Mass *> &merged) {
//---------------------------------------------------------------------------

    Mass *mt = sim->masses[index];
    for (Spring *s : sim->springs) {
        Vec orig = mt->origpos + dx;
        if (s->_left == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_right->origpos - orig).norm();
            // Check for merge
            /**if (s->_rest < maxLocalization / 10) {
                mergeMasses(sim, s->_right, mt, s);
                merged.push_back(s->_right);
                return 2;
            }**/
            if (s->_rest < 0.001) {
                s->_rest = origLen;
                return 0;
            }
            s->_k *= origLen / s->_rest;

            double massDiff = massFactor * (s->_rest - origLen);
            s->_left->m += massDiff / 2;
            s->_right->m += massDiff / 2;
            qDebug() << massDiff / 2 << s->_left->m;
        }
        if (s->_right == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_left->origpos - orig).norm();
            // Check for merge
            /**if (s->_rest < maxLocalization / 10) {
                mergeMasses(sim, s->_left, mt, s);
                merged.push_back(s->_left);
                return 2;
            }**/
            if (s->_rest < 0.001) {
                s->_rest = origLen;
                return 0;
            }
            s->_k *= origLen / s->_rest;

            double massDiff = massFactor * (s->_rest - origLen);
            s->_left->m += massDiff / 2;
            s->_right->m += massDiff / 2;
        }
    }

    mt->origpos += dx;
    mt->pos += dx;
    mt->vel = Vec(0, 0, 0);
    sim->setAll();
    return 1;
}

// Shift mass at pointer by dx
//---------------------------------------------------------------------------
void MassDisplacer::shiftMassPos(Simulation *sim, Mass *mt, const Vec &dx) {
//---------------------------------------------------------------------------

    for (Spring *s : sim->springs) {
        Vec orig = mt->origpos + dx;
        if (s->_left == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_right->origpos - orig).norm();
            // Check for merge
            if (s->_rest < 0.001) {
                mergeMasses(sim, s->_right, mt, s);
                return;
            }
            s->_k *= origLen / s->_rest;
            double massDiff = s->_rest / origLen;
            s->_left->m -= massDiff / 2;
            s->_right->m -= massDiff / 2;
        }
        if (s->_right == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_left->origpos - orig).norm();
            // Check for merge
            if (s->_rest < 0.001) {
                mergeMasses(sim, s->_left, mt, s);
                return;
            }
            s->_k *= origLen / s->_rest;
            double massDiff = s->_rest / origLen;
            s->_left->m -= massDiff / 2;
            s->_right->m -= massDiff / 2;
        }
    }

    mt->origpos += dx;
    mt->pos += dx;
    mt->vel = Vec(0, 0, 0);
    sim->setAll();
}



// Moves masses in the chunk by dx
//---------------------------------------------------------------------------
int MassDisplacer::shiftRandomChunk(Simulation *sim, const Vec &dx, vector<int> indices, vector<Mass *> &merged) {
//---------------------------------------------------------------------------

    int ret = 1;
    for (int i : indices) {
        ret *= shiftMassPos(sim, i, dx, merged);
    }
    return ret;
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

    vector<Mass *> merged = vector<Mass *>();

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


    // Record start positions
    vector<Vec> startPos = vector<Vec>();
    vector<Vec> origPos = vector<Vec>();
    vector<Mass *> startSprings = vector<Mass *>();
    vector<double> startMass = vector<double>();
    vector<double> startRest = vector<double>();
    for (Mass *m : sim->masses) {
        startPos.push_back(m->pos);
        origPos.push_back(m->origpos);
        startMass.push_back(m->m);
    }
    for (Spring *s : sim->springs) {
        startRest.push_back(s->_rest);
        startSprings.push_back(s->_left);
        startSprings.push_back(s->_right);
    }
    for (Spring *t : sim->springs) {
        t->_broken = false;
    }

    //sim->setAll();

    // Equilibrate simulation
    //if (relaxation == 0) {
    //    settleSim(sim, 1E-6);
    //} else {
    //    relaxSim(sim, relaxation);
    //}

    // Record start metrics
    //double totalMetricSim = 0;
    //double totalLengthSim = 0;
    //double totalEnergySim = 0;

    /**if (metricOrder > 0)  {
        totalLengthSim = calcOrderLength(sim, orderGroup);
        totalEnergySim = calcOrderEnergy(sim, orderGroup);
    }
    else {
        totalLengthSim = calcTotalLength(sim);
        totalEnergySim = calcTotalEnergy(sim);
    }**/

    if (isnan(lastMetric)) {
        for (Mass *m : sim->masses) {
            std::cout << "Mass " << m->index << " m " << m->m << " pos " << m->pos[0] << "," << m->pos[1] << "," << m->pos[2] << std::endl;
        }
        for (Spring *s : sim->springs) {
            std::cout << "Spring " << s->_left->index << "," << s->_right->index << " rest " << s->_rest << " k " << s->_k << std::endl;
        }
        exit(1);
    }

    double totalMetricTest = 0;
    double totalLengthTest = 0;
    double totalEnergyTest = 0;




    // Pick a random direction
    Vec dir = Utils::randDirectionVec();
    qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
    Vec dx = displacement * dir;

    // Move mass
    int successMove = shiftRandomChunk(sim, dx, chunk, merged);
    if (!successMove) {
        qDebug() << "Overlapped mass";
        return 0;
    }
    if (!merged.empty()) {
        qDebug() << "Merged masses";
    }

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

    if (isnan(totalEnergyTest)) {
        for (Mass *m : sim->masses) {
            std::cout << "Mass " << m->index << " m " << m->m << " pos " << m->pos[0] << "," << m->pos[1] << "," << m->pos[2] << std::endl;
        }
        for (Spring *s : sim->springs) {
            std::cout << "Spring " << s->_left->index << "," << s->_right->index << " rest " << s->_rest << " k " << s->_k << std::endl;
        }
        exit(1);
    }

    totalMetricTest = totalEnergyTest * totalLengthTest ;

    qDebug() << "Total lengths Test" << totalLengthTest;
    qDebug() << "Total energies Test" << totalEnergyTest;
    qDebug() << "Total metrics Sim" << lastMetric << " Test" << totalMetricTest;

    for (int e = 0; e < edgeGroup.size(); e++) {
        Mass *m = edgeGroup[e];
        //m->extforce -= addedForces[e];
        //m->force = Vec(0, 0, 0);
    }
    for (Mass *m : outsideGroup) {

        //m->unfix();
    }

    if (isnan(totalMetricTest) || totalMetricTest >= lastMetric) {
        setMassState(startPos, startMass);
        for (int m = 0; m < sim->masses.size(); m++) {
            sim->masses[m]->origpos = origPos[m];
            sim->masses[m]->vel = Vec(0,0,0);
        }
        // Reverse merges
        for (int m = 0; m < startSprings.size(); m+=2) {
            Mass *m1 = startSprings[m];
            Mass *m2 = startSprings[m + 1];
            if (m/2 < sim->springs.size()) {
                Spring *s = sim->springs[m/2];
                if (s->_left != m1) {
                    s->setLeft(m1);
                }
                if (s->_right != m2) {
                    s->setRight(m2);
                }
            } else {
                Spring *s = new Spring(*sim->springs.front());
                s->setMasses(m1, m2);
                sim->createSpring(s);
                qDebug() << "Rest" << s->_rest;
            }
        }
        for (int j = 0; j < sim->springs.size(); j++) {
            Spring *s = sim->springs[j];
            s->_k *= s->_rest / startRest[j];
            s->_rest = startRest[j];
            s->_max_stress = 0;
        }

        sim->setAll();
    } else {
        sim->setAll();
        qDebug() << "Moved" << i;
        lastMetric = totalMetricTest;
        return 1;
    }

    return 0;
}

// Displaces mass within a mass group
//---------------------------------------------------------------------------
int MassDisplacer::displaceGroupMass(double displacement) {
//---------------------------------------------------------------------------

    qDebug() << "Displacing mass";
    sim->getAll();

    n_springs = sim->springs.size();
    n_masses = sim->masses.size();

    // Pick a random mass
    int i = pickRandomMass(massGroup);
    Mass *mt = massGroup.group[i];
    qDebug() << "Chose mass" << i;

    // Record start positions
    vector<Vec> startPos = vector<Vec>();
    vector<Vec> origPos = vector<Vec>();
    vector<double> startMass = vector<double>();
    vector<double> startRest = vector<double>();
    vector<Spring> startBorder = vector<Spring>();
    vector<Mass *> startMassSpan = vector<Mass *>();
    for (Mass *m : sim->masses) {
        startPos.push_back(m->pos);
        origPos.push_back(m->origpos);
        startMass.push_back(m->m);
    }
    for (Spring *s : sim->springs) {
        startRest.push_back(s->_rest);
    }
    if (iterations == 0) {
        for (Mass *m : massGroup.group) {
            //m->vel = Vec(0, 0, 0);
        }
        for (Mass *m : massGroup.edge) {

            for (Spring *s : sim->springs) {
                if (s->_right == m) {
                    m->extforce += s->getForce();
                    m->extduration = DBL_MAX;
                }
                if (s->_left == m) {
                    m->extforce -= s->getForce();
                    m->extduration = DBL_MAX;
                }
            }
            for (Spring *s : massGroup.springs) {
                if (s->_right == m) {
                    m->extforce -= s->getForce();
                }
                if (s->_left == m) {
                    m->extforce += s->getForce();
                }
            }
            m->force = m->extforce;
            qDebug() << "Mass force" << m->extforce[0] << m->extforce[1] << m->extforce[2];
        }
        for (Spring *s : massGroup.border) {
            Spring t = Spring(*s);
            startBorder.push_back(t);
            startMassSpan.push_back(s->_left);
            startMassSpan.push_back(s->_right);
            sim->deleteSpring(s);
        }
        /**for (int s = 0; s < sim->springs.size(); s++) {
            Spring *s1 = sim->springs[s];
            bool found = false;
            for (Spring *s2: massGroup.springs) {
                if (s1 == s2) {
                    found = true;
                }
            }
            if (!found) {
                sim->deleteSpring(s1);
                s--;
            }
        }**/


        n_springs = sim->springs.size();
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

    totalLengthSim = calcTotalLength(sim);
    totalEnergySim = calcTotalEnergy(sim);

    double totalMetricTest = 0;
    double totalLengthTest = 0;
    double totalEnergyTest = 0;

    // Pick a random direction
    Vec dir = Utils::randDirectionVec();
    qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
    Vec dx = displacement * dir;

    // Move mass
    shiftMassPos(sim, mt, dx);

    // Run simulation
    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation);
    }

    // Calculate test metrics
    totalLengthTest = calcTotalLength(sim);
    totalEnergyTest = calcTotalEnergy(sim);

    totalMetricSim = totalEnergySim * totalLengthSim ;
    totalMetricTest = totalEnergyTest * totalLengthTest ;

    qDebug() << "Total lengths Sim" << totalLengthSim << " Test" << totalLengthTest;
    qDebug() << "Total energies Sim" << totalEnergySim << " Test" << totalEnergyTest;
    qDebug() << "Total metrics Sim" << totalMetricSim << " Test" << totalMetricTest;

    for (Mass *m : massGroup.edge) {
        m->extforce = Vec(0,0,0);
        m->force = Vec(0,0,0);
    }

    //Vec mtPos = mt->pos;
    //setMassState(startPos, startMass);
    //mt->pos = mtPos;

    for (Mass *m : sim->masses) {
        //m->pos = m->origpos;
        m->vel = Vec(0,0,0);
    }
    for (int s = 0; s < startBorder.size(); s++) {
        Spring *n = new Spring(startBorder[s]);
        n->setMasses(startMassSpan[s*2], startMassSpan[s*2+1]);
        sim->createSpring(n);
    }

    if (totalMetricTest >= totalMetricSim) {
        setMassState(startPos, startMass);
        for (int m = 0; m < sim->masses.size(); m++) {
            sim->masses[m]->origpos = origPos[m];
        }
        for (int j = 0; j < n_springs; j++) {
            Spring *s = sim->springs[j];

            s->_k *= s->_rest / startRest[j];
            s->_rest = startRest[j];
            s->_max_stress = 0;
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
        massGroup.edge = vector<Mass *>();
        massGroup.border = vector<Spring *>();

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
                massGroup.edge.push_back(s->_left);
                massGroup.springs.push_back(s);
            } else if (rdist <= cutoff) {
                // Border spring: Right mass is within the order group, left mass is not
                massGroup.outside.push_back(s->_left);
                massGroup.edge.push_back(s->_right);
                massGroup.springs.push_back(s);
            }
        }

        vector<Mass *> culledOrderGroup = vector<Mass *>();
        vector<Mass *> culledOutsideGroup = vector<Mass *>();
        vector<Mass *> culledEdgeGroup = vector<Mass *>();
        for (Mass *m : sim->masses) {
            if (find(massGroup.group.begin(), massGroup.group.end(), m) != massGroup.group.end()) {
                culledOrderGroup.push_back(m);
            }
            if (find(massGroup.outside.begin(), massGroup.outside.end(), m) != massGroup.outside.end()) {
                culledOutsideGroup.push_back(m);
            }
            if (find(massGroup.edge.begin(), massGroup.edge.end(), m) != massGroup.edge.end()) {
                culledEdgeGroup.push_back(m);
            }
        }
        massGroup.group = culledOrderGroup;
        massGroup.outside = culledOutsideGroup;
        massGroup.edge = culledEdgeGroup;

}

// Creates arrays of surrounding masses within a bounding box defined by min and max corner
//---------------------------------------------------------------------------
void MassDisplacer::createMassGroup(Simulation *sim, Vec minc, Vec maxc, MassGroup &massGroup) {
//---------------------------------------------------------------------------

    massGroup.group = vector<Mass *>();
    massGroup.springs = vector<Spring *>();
    massGroup.outside = vector<Mass *>();
    massGroup.edge = vector<Mass *>();
    massGroup.border = vector<Spring *>();

    for (Spring *s : sim->springs) {
        bool leftInBounds = Utils::inBounds(s->_left->pos, minc, maxc);
        bool rightInBounds = Utils::inBounds(s->_right->pos, minc, maxc);
        if (leftInBounds && rightInBounds) {
            massGroup.springs.push_back(s);
            massGroup.group.push_back(s->_left);
            massGroup.group.push_back(s->_right);
        } else if (leftInBounds) {
            // Border spring: Left mass is within the order group, right mass is not
            massGroup.outside.push_back(s->_right);
            massGroup.edge.push_back(s->_left);
            massGroup.border.push_back(s);
        } else if (rightInBounds) {
            // Border spring: Right mass is within the order group, left mass is not
            massGroup.outside.push_back(s->_left);
            massGroup.edge.push_back(s->_right);
            massGroup.border.push_back(s);
        }
    }

    vector<Mass *> culledOrderGroup = vector<Mass *>();
    vector<Mass *> culledOutsideGroup = vector<Mass *>();
    vector<Mass *> culledEdgeGroup = vector<Mass *>();
    for (Mass *m : sim->masses) {
        if (find(massGroup.group.begin(), massGroup.group.end(), m) != massGroup.group.end()) {
            culledOrderGroup.push_back(m);
        }
        if (find(massGroup.outside.begin(), massGroup.outside.end(), m) != massGroup.outside.end()) {
            culledOutsideGroup.push_back(m);
        }
        if (find(massGroup.edge.begin(), massGroup.edge.end(), m) != massGroup.edge.end()) {
            culledEdgeGroup.push_back(m);
        }
    }
    massGroup.group = culledOrderGroup;
    massGroup.outside = culledOutsideGroup;
    massGroup.edge = culledEdgeGroup;

}

// Creates Mass Groups from global Trench Grid
// Trenches separate mass groups
//---------------------------------------------------------------------------
void MassDisplacer::createMassGroupGrid(Simulation *sim, const TrenchGrid &grid, vector<MassGroup> &groups) {
//---------------------------------------------------------------------------

    int nx, ny, nz;
    assert(grid.startCorner[0] <= grid.endCorner[0]);
    assert(grid.startCorner[1] <= grid.endCorner[1]);
    assert(grid.startCorner[2] <= grid.endCorner[2]);

    nx = ceil((grid.endCorner[0] - grid.startCorner[0]) / grid.dimension[0]);
    ny = ceil((grid.endCorner[1] - grid.startCorner[1]) / grid.dimension[1]);
    nz = ceil((grid.endCorner[2] - grid.startCorner[2]) / grid.dimension[2]);

    for (int x = 0; x < nx; x++) {
        for (int y = 0; y < ny; y++) {
            for (int z = 0; z < nz; z++) {
                Vec minc = grid.startCorner + Vec(x*grid.dimension[0], y*grid.dimension[1], z*grid.dimension[2]);
                Vec maxc = minc + grid.dimension;
                MassGroup mg;
                createMassGroup(sim, minc, maxc, mg);
            }
        }
    }
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

        //shiftMassPos(sim, m, dvec);
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
            //setMassState(startPos);
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
void MassDisplacer::setMassState(const vector<Vec> &pos, const vector<double> &mm){
//---------------------------------------------------------------------------

    // Make sure vectors maps one-to-one to masses
    assert(pos.size() == sim->masses.size());
    assert(mm.size() == sim->masses.size());

    for (int i = 0; i < pos.size(); i++) {
        sim->masses[i]->pos = pos[i];
        sim->masses[i]->m = mm[i];
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
