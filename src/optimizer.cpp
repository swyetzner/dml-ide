//
// Created by sw3390 on 4/24/19.
//

#include "optimizer.h"
#include "../lib/Titan/include/Titan/sim.h"

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
void Optimizer::sortSprings_stress(vector<Spring *> &spring_list, vector<uint> &output_indices) {
//---------------------------------------------------------------------------

    vector<double> springStress = vector<double>();
    output_indices = vector<uint>();

    for (uint s = 0; s < spring_list.size(); s++) {
        bool underExternalForce = spring_list[s]->_left->extforce.norm() > 1E-6
                                  && spring_list[s]->_right->extforce.norm() > 1E-6;
        bool fixed = spring_list[s]->_left->constraints.fixed && spring_list[s]->_right->constraints.fixed;
        if (!underExternalForce && !fixed) {
            output_indices.push_back(s);
        }

        double force = spring_list[s]->_max_stress;
        springStress.push_back(force);
    }

    // Sort in increasing order by max stress
    sort(output_indices.begin(), output_indices.end(),
         [springStress](uint s1, uint s2) -> bool {
             return springStress[s1] < springStress[s2];
         });
    qDebug() << "Sorted springs by stress" << output_indices.size() << output_indices.front() << spring_list[output_indices.front()]->_max_stress;
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

// Run simulation until it reaches mechanical equilibrium within eps
//---------------------------------------------------------------------------
int Optimizer::settleSim(double eps, bool use_cap, double cap) {
//---------------------------------------------------------------------------

    bool equilibrium = false;
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


//---------------------------------------------------------------------------
//  SPRING REMOVER
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
SpringRemover::SpringRemover(Simulation *sim, double removeRatio, double stopRatio)
    : Optimizer(sim) {
//---------------------------------------------------------------------------

    this->regeneration = false;
    this->stepRatio = removeRatio;
    this->stopRatio = stopRatio;
    this->stressMemory = 1;
    this->regenRate = 0;
    qDebug() << "Set spring remover ratios" << this->stepRatio << this->stopRatio;

    // Fill mass to spring map
    validSprings = sim->springs;
    fillMassSpringMap();
}

//---------------------------------------------------------------------------
void SpringRemover::fillMassSpringMap() {
//---------------------------------------------------------------------------
        massToSpringMap = map<Mass *, vector<Spring *>>();
    validSprings = vector<Spring *>();
    assert(massToSpringMap.empty());
    for (Spring *s : sim->springs) {
        if (s->_k > 0) {
            massToSpringMap[s->_left].push_back(s);
            massToSpringMap[s->_right].push_back(s);
            validSprings.push_back(s);
        }
    }
    qDebug() << "fillMassSpringMap():  map size:" << massToSpringMap.size() << " valid size:" << validSprings.size();
    assert(massToSpringMap.size() <= sim->masses.size());

}

//---------------------------------------------------------------------------
void SpringRemover::removeSpringFromMap(Spring *d) {
//---------------------------------------------------------------------------

    auto &m1 = massToSpringMap[d->_left];
    m1.erase(remove(m1.begin(), m1.end(), d), m1.end());

    auto &m2 = massToSpringMap[d->_right];
    m2.erase(remove(m2.begin(), m2.end(), d), m2.end());

}

//---------------------------------------------------------------------------
void SpringRemover::invalidateSpring(Spring *i) {
//---------------------------------------------------------------------------

    double d = 0.5 * (i->_left->density + i->_right->density);
    double v = M_PI * i->_diam/2 * i->_diam/2 * i->_rest;
    double m = d * v;

    i->_k = 0;
    validSprings.erase(remove(validSprings.begin(), validSprings.end(), i), validSprings.end());

    assert(i->_left->density > 0 && i->_right->density > 0);

    i->_left->m -= m/2;
    i->_right->m -= m/2;
}

//---------------------------------------------------------------------------
void SpringRemover::removeMassFromMap(Mass *d) {
//---------------------------------------------------------------------------

    massToSpringMap.erase(d);

}

//---------------------------------------------------------------------------
void SpringRemover::removeHangingSprings(map<Spring *, bool> &hangingCandidates,
                                         map<Spring *, bool> &springsToDelete) {
//---------------------------------------------------------------------------

    // Remove hanging springs (attached to masses with only one attached spring
    int hangingSprings = 0;
    while (!hangingCandidates.empty()) {
        qDebug() << "Hanging spring candidates" << hangingCandidates.size();
        map<Spring *, bool> newCandidates = map<Spring *, bool>();
        for (auto hc : hangingCandidates) {
            if (hc.first == nullptr) continue;
            Spring *s = hc.first;
            if (!springsToDelete[s] && s != nullptr) {
                float EPSILON = 1E-6;

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
                            bool colinear = false;
                            if (s->_left->pos == h->_right->pos) {
                                if (Utils::areCloseToColinear(s->_right->pos, s->_left->pos, h->_left->pos, EPSILON)) {
                                    colinear = true;
                                }
                            } else {
                                if (Utils::areCloseToColinear(s->_right->pos, s->_left->pos, h->_right->pos, EPSILON)) {
                                    colinear = true;
                                }
                            }
                            if (!colinear) {
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
                            bool colinear = false;
                            if (s->_right->pos == h->_right->pos) {
                                if (Utils::areCloseToColinear(s->_left->pos, s->_right->pos, h->_left->pos, EPSILON)) {
                                    colinear = true;
                                }
                            } else {
                                if (Utils::areCloseToColinear(s->_left->pos, s->_right->pos, h->_right->pos, EPSILON)) {
                                    colinear = true;
                                }
                            }
                            if (!colinear) {
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
                if (massToSpringMap[s->_left].size() == 3) {
                    Vec commonVertex = s->_left->pos;
                    std::vector<Vec> points = std::vector<Vec>();
                    points.push_back(commonVertex);
                    for (Spring *h : massToSpringMap[s->_left]) {
                        if (h->_left->pos == commonVertex){
                            points.push_back(h->_right->pos);
                        }
                        else {
                            points.push_back(h->_left->pos);
                        }
                    }
                    assert(points.size() == 4);
                    if (Utils::areCloseToCoplanar(points[0], points[1], points[2], points[3], EPSILON)) {
                        for (Spring *h : massToSpringMap[s->_left]) {
                            if (!springsToDelete[h]) hangingSprings++;
                            springsToDelete[h] = true;
                            removeSpringFromMap(h);

                            // Add connected springs
                            if (h->_left->pos == commonVertex) {
                                for (Spring *c : massToSpringMap[h->_right]) {
                                    if (c != h) newCandidates[c] = true;
                                }
                            }
                            if (h->_right->pos == commonVertex) {
                                for (Spring *c : massToSpringMap[h->_left]) {
                                    if (c != h) newCandidates[c] = true;
                                }
                            }
                        }
                    }
                }
                if (massToSpringMap[s->_right].size() == 3) {
                    Vec commonVertex = s->_right->pos;
                    std::vector<Vec> points = std::vector<Vec>();
                    points.push_back(commonVertex);
                    for (Spring *h : massToSpringMap[s->_right]) {
                        if (h->_left->pos == commonVertex){
                            points.push_back(h->_right->pos);
                        }
                        else {
                            points.push_back(h->_left->pos);
                        }
                    }
                    assert(points.size() == 4);
                    if (Utils::areCloseToCoplanar(points[0], points[1], points[2], points[3], EPSILON)) {
                        for (Spring *h : massToSpringMap[s->_right]) {
                            if (!springsToDelete[h]) hangingSprings++;
                            springsToDelete[h] = true;
                            removeSpringFromMap(h);

                            // Add connected springs
                            if (h->_left->pos == commonVertex) {
                                for (Spring *c : massToSpringMap[h->_right]) {
                                    if (c != h) newCandidates[c] = true;
                                }
                            }
                            if (h->_right->pos == commonVertex) {
                                for (Spring *c : massToSpringMap[h->_left]) {
                                    if (c != h) newCandidates[c] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        qDebug() << "Hanging springs" << hangingSprings;
        qDebug() << "New candidates" << newCandidates.size();
        hangingCandidates = newCandidates;
    }
    qDebug() << "massToSpring map" << massToSpringMap.size();
}


//---------------------------------------------------------------------------
void SpringRemover::deleteGhostSprings() {
//---------------------------------------------------------------------------

    if (validSprings.size() == sim->springs.size()) return;

    for (int i = 0; i < sim->springs.size(); i++) {
        Spring *s = sim->springs[i];
        if (s->_k == 0) {
            sim->deleteSpring(s);
            i--;
        }
    }

}



//---------------------------------------------------------------------------
void SpringRemover::resetHalfLastRemoval() {
//---------------------------------------------------------------------------

    qDebug() << "Resetting" << removedSprings.size() << "Springs";
    sim->getAll();

    if (removedSprings.empty()) return;

    int start = 0;
    int end = removedSprings.size();
    if (removedSprings.front()->_k == 0) {
        end = end/2;
    } else {
        start = end/2;
        for (int i = 0; i < start; i++) {
            Spring *s = removedSprings[i];

            invalidateSpring(s);
        }
    }

    for (int i = start; i < end; i++) {
        Spring *s = removedSprings[i];

        s->_k = removedSprings_k[i];

        s->_left->m += s->_left->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;
        s->_right->m += s->_right->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;

        validSprings.push_back(s);
    }

    for (Mass *m : sim->masses) {
        m->pos = m->origpos;
    }

    fillMassSpringMap();
    sim->setAll();
}


void SpringRemover::regenerateShift() {

    qDebug() << "REGENERATING LATTICE";

    sim->getAll();

    qDebug() << massToSpringMap.size();
    fillMassSpringMap();

    for (auto &ms : massToSpringMap) {

        if (ms.first == nullptr || ms.second.empty()) continue;
        qDebug() << ms.first->index;

        Vec dir = Utils::randDirectionVec();
        double unit = sim->springs.front()->_rest / 4;
        dir = dir * unit;

        Mass *n = new Mass(*sim->masses.front());
        Mass *m = sim->createMass(n);
        m->pos = ms.first->origpos;
        m->origpos = ms.first->origpos;
        m->constraints.fixed = ms.first->constraints.fixed;
        m->m = 0.0;
        m->ref_count = 1;


        for (Spring *t : ms.second) {
            if (t == nullptr) continue;

            Spring f = Spring(*sim->springs.front());
            Spring *s = new Spring(f);
            if (t->_left == ms.first) s->setMasses(m, t->_right);
            if (t->_right == ms.first) s->setMasses(t->_left, m);

            double origLen = s->_rest;
            s->_rest = (s->_left->origpos - s->_right->origpos).norm();
            s->_k *= origLen / s->_rest;
            s->_max_stress = 0;

            s->_left->m += s->_left->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;
            s->_right->m += s->_right->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;

            sim->createSpring(s);
            //hangingCandidates[s] = true;
        }
    }
    qDebug() << "Created new masses";

    // Reindex masses
    for (int i = 0; i < sim->masses.size(); i++) {
        sim->masses[i]->index = i;
    }
    fillMassSpringMap();

    sim->setAll();
}


void SpringRemover::regenerateLattice(SimulationConfig *config) {

    qDebug() << "REGENERATING LATTICE";
    sim->getAll();

    double minCut = 2 * config->lattices.front()->unit[0];

    vector<Vec> lattice;
    fillMassSpringMap();

    deleteGhostSprings();
    oUtils::generateMassesBounded(minCut, massToSpringMap, lattice, this->n_masses_start * this->regenRate);
    //oUtils::generateMassesPoisson(minCut, massToSpringMap, lattice);

    fillMassSpringMap();

    // Add masses to simulation
    int nm = sim->masses.size();
    for (Vec l : lattice) {
        if (config->plane) {
            Vec proj;
            double d = Utils::distPointPlane(l, config->plane->normal, config->plane->offset, proj);
            Vec projV = (l - proj).normalized();
            qDebug() << "Projection and dist" << projV[0]  << projV[1] << projV[2] << d;
            if (d == 0 || !((l - proj).normalized() == config->plane->normal)) {
                qDebug() << "CONTINUE";
                continue;
            }
        }
        Mass *n = new Mass(*sim->masses.front());
        Mass *m = sim->createMass(n);
        m->pos = l;
        m->origpos = l;
        m->vel = Vec(0,0,0);
        m->constraints.fixed = false;
        m->m = 0.0;
        m->ref_count = 1;
        m->density = sim->masses.front()->density;
    }

  // Reindex masses
    for (int i = 0; i < sim->masses.size(); i++) {
        sim->masses[i]->index = i;
    }

    map<Spring *, bool> hangingCandidates;
    for (Spring *s : validSprings) {
        hangingCandidates[s] = false;
    }
    double maxS = 0.5 * 2.9 * minCut;
    int newSprings = 0;

    for (int j = 0; j < sim->masses.size() - 1; j++) {
        for (int k = nm; k < sim->masses.size(); k++) {

            if (j == k) continue;

            Mass *m1 = sim->masses[j];
            Mass *m2 = sim->masses[k];

            if (massToSpringMap.count(m1) == 0) continue;

            assert(m1 != nullptr);
            assert(m2 != nullptr);

            if ((m1->pos - m2->pos).norm() <= maxS && (m1->pos - m2->pos).norm() > maxS / 4) {
                Spring t = Spring(*validSprings.front());
                Spring *s = new Spring(t);
                s->setMasses(m1, m2);
                double origLen = s->_rest;
                s->_rest = (m1->origpos - m2->pos).norm();
                s->_k *= origLen / s->_rest;
                // qDebug() << "K" << s->_k;
                s->_max_stress = 0;

                // Mass values
                m1->m += m1->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;
                m2->m += m2->density * s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;

                //qDebug() << "Masses" << m1->m << m2->m << m1->index << m2->index;
                //qDebug() << "Spring" << s->_rest << s->_k;

                sim->createSpring(s);
                validSprings.push_back(s);
                hangingCandidates[s] = true;
                newSprings++;
            }
        }
    }
    qDebug() << "New springs created" << newSprings;

    for (Mass *m : sim->masses) {
        if (m->spring_count < 2) qDebug() << "SPRING COUNT" << m->index << m->spring_count;
    }

    fillMassSpringMap();

    map<Spring *, bool> springsToDelete;
    for (Spring *s : validSprings) {
        springsToDelete[s] = false;
    }
    removeHangingSprings(hangingCandidates, springsToDelete);

    // Remove springs
    uint i = 0;
    while (i < validSprings.size()) {
        if (validSprings[i] != nullptr && springsToDelete[sim->springs[i]]) {
            //sim->deleteSpring(sim->springs[i]);
            removedSprings.push_back(validSprings[i]);
            removedSprings_k.push_back(validSprings[i]->_k);
            //validSprings[i]->_k = 0;
            //validSprings.erase(remove(validSprings.begin(), validSprings.end(), validSprings[i]), validSprings.end());
            invalidateSpring(validSprings[i]);
            i--;
        }
        i++;
    }
    qDebug() << "Deleted springs";
    fillMassSpringMap();

    //deleteGhostSprings();

    sim->setAll();
    sim->step(sim->masses.front()->dt * 40000);
    sim->getAll();

    for (Spring *s : sim->springs) {
        s->_max_stress = 0;
    }
    sim->setAll();

}

void SpringRemover::splitSprings() {
  
  int n = sim->springs.size();
  
  qDebug() << "Springs" << n;
  for (int i = 0; i < n; i++) {
  
    Spring *s = sim->springs[i];
    
    // Find midpoint
    Vec mid = 0.5 * (s->_left->pos + s->_right->pos);
    
    // Add mass at midpoint
    Mass *m = sim->createMass(mid);
    qDebug() << "Added mass" << m->pos[0] << m->pos[1] << m->pos[2];
  }

    double maxCut = 0;
    for (Spring *s : sim->springs) {
      maxCut = std::max(maxCut, s->_rest);
    }


    for (int j = 0; j < sim->masses.size() - 1; j++) {
      for (int k = j+1; k < sim->masses.size(); k++) {
	Mass *m1 = sim->masses[j];
	Mass *m2 = sim->masses[k];

	if ((m2->pos - m1->pos).norm() <= maxCut) {
	// Make spring
	  sim->createSpring(m1, m2);
	}
      }
    }
    sim->setAll();
}


// Removes stepRatio percent least stressed springs
//---------------------------------------------------------------------------
void SpringRemover::optimize() {
//---------------------------------------------------------------------------

    sim->getAll();
    fillMassSpringMap();
    n_springs = validSprings.size();
    qDebug() << "n_springs" << n_springs;

    if (n_springs > n_springs_start * stopRatio) {
        map<Spring *, bool> springsToDelete = map<Spring *, bool>();
        map<Mass *, bool> massesToDelete = map<Mass *, bool>();
        map<Spring *, bool> hangingCandidates = map<Spring *, bool>();

        removedSprings = vector<Spring *>();

        uint toRemove = stepRatio > 0 ?  uint(stepRatio * n_springs): 1;
        qDebug() << "toRemove" << toRemove;

	
        if (toRemove > 1) {
            vector<uint> springIndicesToSort;
            sortSprings_stress(validSprings, springIndicesToSort);

            for (Spring *s : validSprings) {
                springsToDelete[s] = false;
            }
            for (uint j = 0; j < toRemove; j++) {
                if (j < springIndicesToSort.size()) {
                    Spring *d = validSprings[springIndicesToSort[j]];
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
            springsToDelete[validSprings[ms]] = true;
            removeSpringFromMap(validSprings[ms]);

            for (Spring *c : massToSpringMap[validSprings[ms]->_left]) {
                hangingCandidates[c] = true;
            }
            for (Spring *c : massToSpringMap[validSprings[ms]->_right]) {
                hangingCandidates[c] = true;
            }
        }
        qDebug() << "Removing" << toRemove << "Springs";


        // Remove hanging springs (attached to masses with only one attached spring
        removeHangingSprings(hangingCandidates, springsToDelete);

        // Remove springs
        uint i = 0;

        while (i < validSprings.size()) {
            if (validSprings[i] != nullptr && springsToDelete[validSprings[i]]) {
                removedSprings.push_back(validSprings[i]);
                removedSprings_k.push_back(validSprings[i]->_k);
                //validSprings[i]->_k = 0;
                //validSprings.erase(remove(validSprings.begin(), validSprings.end(), validSprings[i]), validSprings.end());
                invalidateSpring(validSprings[i]);
                //sim->deleteSpring(sim->springs[i]);
                i--;
            }
            i++;
        }
        qDebug() << "Deleted springs" << validSprings.size();

        for (auto ms : massToSpringMap) {
            ms.second.erase(remove(ms.second.begin(), ms.second.end(), nullptr), ms.second.end());
        }
        qDebug() << "Culled map";
        for (Spring *s : sim->springs) {
            s->_max_stress *= stressMemory;
        }

        qDebug() << "Applied stress memory";
        // Remove masses
        for (Mass *m : sim->masses) {
            if (massToSpringMap.count(m) && massToSpringMap[m].empty()) {
                removeMassFromMap(m);
            }
        }

        qDebug() << "Deleted masses";
        for (i = 0; i < sim->masses.size(); i++) {
            sim->masses[i]->index = i;
        }
        qDebug() << "Reindexed masses";

        sim->setAll(); // Set spring stresses and mass value updates on GPU

        n_springs = int(validSprings.size());
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
    sortSprings_stress(sim->springs, springIndicesToSort);

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
    this->totalAttempts = 0;
    this->totalTrialTime = 0;
    this->prevAttemptNums = vector<int>();
    this->maxAvgSuccessRate = 0;
    this->lastTune = 0;
    this->lastMetric = 0;
    this->gridOffset = Vec(0,0,0);
    this->dimensions = Vec(0,0,0);
    this->unit = 0.1;
    test = new Simulation();
    start = new Simulation();

    this->maxLocalization = 0;

    customMetricHeader = "Time,Position(x),Position(y),Position(z),Force(x),Force(y),Force(z),Index\n";

    trenchGrid.startCorner = Vec(-0.3, -0.05, -0.05);
    trenchGrid.endCorner = Vec(0.3, 0.05, 0.05);
    trenchGrid.dimension = Vec(0.15, 0.1, 0.1);
    massGroups = vector<MassGroup *>();

    createMassGroup(this->sim, Vec(-0.05, -0.05, -0.05), Vec(0.05, 0.05, 0.05), this->massGroup);

    createMassTiles(sim, 0.1, Vec(0,0,0), massGroups, massGroupMap, trenchSprings);


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

START_OPTIMIZE:
    int displaced = 0;
    attempts = 0;
    double trialTime = 0;

    int nx = ceil(dimensions[0] / unit);
    int ny = ceil(dimensions[1] / unit);
    int nz = ceil(dimensions[2] / unit);

    bool shiftx = nx > 3;
    bool shifty = ny > 3;
    bool shiftz = nz > 3;
    bool dim [] = {shiftx, shifty, shiftz};
    bool carry [] = {false, false, false};

    int rightmost = -1;
    for (int d = 2; d >= 0; d--) {
        if (dim[d]) {
            rightmost = d;
        }
        if (rightmost >= 0) continue;
    }

    for (int d = 2; d >= 0; d--) {
        if (rightmost == d) carry[d] = true;
        if (dim[d]) {
            if (carry[d]) {
                gridOffset[d] += springUnit;
                if (gridOffset[d] > unit) {
                    for (int i = d; i < 3; i++) {
                        gridOffset[i] = 0;
                    }
                    if (d > 0)
                        carry[d - 1] = true;
                }
                carry[d] = false;
            }
        } else {
            if (carry[d]) {
                if (d > 0)
                    carry[d-1] = true;
            }
            carry[d] = false;
        }
    }

    qDebug() << "Grid Offset" << gridOffset[0] << gridOffset[1] << gridOffset[2];

    while (displaced == 0) {
        attempts ++;
        auto pstart = std::chrono::system_clock::now();
        //displaced = displaceSingleMass(dx, chunkSize, order);
        createMassTiles(sim, unit, gridOffset, massGroups, massGroupMap, trenchSprings);
        if (massGroups.empty()) goto START_OPTIMIZE;
        displaced = displaceGroupMass(dx);
        //displaced = displaceManyMasses(dx, order, 2);
        auto pend = std::chrono::system_clock::now();
        std::chrono::duration<double> pduration = pend - pstart;
        std::cout << "Trial " << attempts << " duration: " << pduration.count() << '\n';
        trialTime += pduration.count();
    }
    totalTrialTime += trialTime;
    totalAttempts += attempts;

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

    cout << "Iteration " << iterations << "\tAttempts: " << attempts << "\tAverage trial time: " << trialTime / attempts << "s \n";

    /**if (!STARTED) {
        initializeClones(100);
        STARTED = true;
    }

    mutateClones(0.002);
    incorporateClones();
    resetClones();**/

    //displaceParallelMasses(50, 1);
}


// Picks a random mass from a sim
//---------------------------------------------------------------------------
int MassDisplacer::pickRandomMass(Simulation *sim) {
//---------------------------------------------------------------------------

    int nm = sim->masses.size();

    // Pick a random mass
    int i = round(Utils::randUnit() * nm);
    bool underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
    bool fixed = sim->masses[i]->constraints.fixed;

    while (underExternalForce || fixed) {
        i = round(Utils::randUnit() * nm);
        underExternalForce = sim->masses[i]->extforce.norm() > 1E-6;
        fixed = sim->masses[i]->constraints.fixed;
    }

    return i;
}

// Pick a random mass form a mass group
//---------------------------------------------------------------------------
int MassDisplacer::pickRandomMass(MassDisplacer::MassGroup &mg) {
//---------------------------------------------------------------------------

    int nm = mg.candidates.size() - 1;

    // Pick a random mass
    int i = round(Utils::randUnit() * nm);

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
                qDebug() << "SMALL REST";
                return 0;
            }
            s->_k *= origLen / s->_rest;

            //double massDiff = massFactor * (s->_rest - origLen);
            //s->_left->m += massDiff / 2;
            //s->_right->m += massDiff / 2;
            //qDebug() << massDiff / 2 << s->_left->m;
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
                qDebug() << "SMALL REST";
                return 0;
            }
            s->_k *= origLen / s->_rest;

            //double massDiff = massFactor * (s->_rest - origLen);
            //s->_left->m += massDiff / 2;
            //s->_right->m += massDiff / 2;
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

    //for (Mass *m : sim->masses) {
    //    m->m = 0; // Reset masses
    //}
    for (Spring *s : sim->springs) {
        Vec orig = mt->origpos + dx;
        if (s->_left == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_right->origpos - orig).norm();
            // Check for merge
            if (s->_rest < 0.001) {
                //mergeMasses(sim, s->_right, mt, s);
                s->_rest = origLen;
                return;
            }
            s->_k *= origLen / s->_rest;
            //s->_mass *= s->_rest / origLen;
        }
        if (s->_right == mt) {
            double origLen = s->_rest;
            s->_rest = (s->_left->origpos - orig).norm();
            // Check for merge
            if (s->_rest < 0.001) {
                //mergeMasses(sim, s->_left, mt, s);
                s->_rest = origLen;
                return;
            }
            s->_k *= origLen / s->_rest;
            //s->_mass *= s->_rest / origLen;
        }
        //s->_left->m += s->_mass / 2;
        //s->_right->m += s->_mass / 2;
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

    if (isnan(totalEnergySim)) {
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

    totalMetricSim = totalEnergySim * totalLengthSim ;
    totalMetricTest = totalEnergyTest * totalLengthTest ;

    qDebug() << "Total lengths Test" << totalLengthTest;
    qDebug() << "Total energies Test" << totalEnergyTest;
    qDebug() << "Total metrics Sim" << totalMetricSim << " Test" << totalMetricTest;

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

    int result = 0;
    int attempts = 0;
    qDebug() << "Displacing mass";
    sim->getAll();

    n_springs = sim->springs.size();
    n_masses = sim->masses.size();

    double totalMass = 0;
    for (Mass *m : sim->masses) {
        totalMass += m->m;
    }
    qDebug() << "Total Mass" << totalMass;
    // Pick a random mass

    // Record start positions
    vector<Vec> startPos = vector<Vec>();
    vector<Vec> origPos = vector<Vec>();
    vector<double> startMass = vector<double>();
    vector<double> startRest = vector<double>();
    vector<Spring> startBorder = vector<Spring>();
    vector<Vec> startForces = vector<Vec>();
    vector<Mass *> startMassSpan = vector<Mass *>();
    for (Mass *m : sim->masses) {
        startPos.push_back(m->pos);
        origPos.push_back(m->origpos);
        startMass.push_back(m->m);
        startForces.push_back(m->extforce);
    }
    for (Spring *s : sim->springs) {
        startRest.push_back(s->_rest);
    }
    vector<Vec> disPos = vector<Vec>();

    splitMassTiles(sim, massGroups, trenchSprings, startBorder, startMassSpan);

    qDebug() << "Mass groups" << massGroups.size();

    n_springs = sim->springs.size();

    sim->setAll();

    // Equilibrate simulation
    if (relaxation == 0) {
        settleSim(sim, 1E-6);
    } else {
        relaxSim(sim, relaxation);
    }

    for (MassGroup *mg : massGroups) {
        // Record start metrics

        mg->origEnergy = calcMassGroupEnergy(mg);
        mg->origLength = calcMassGroupLength(mg);

        for (Mass *m : mg->group) {
            mg->startPos.push_back(m->pos);
            mg->startMass.push_back(m->m);
        }
        for (Spring *s : mg->springs) {
            mg->startRest.push_back(s->_rest);
        }
    }

    while (result <= 10) {
        if (attempts > 50){
            result++;
            break;
        }

        for (MassGroup *mg : massGroups) {

            int i = pickRandomMass(*mg);
            qDebug() << "Picked mass";
            Mass *mt = mg->candidates[i];

            mg->displaced = mt;
            mg->displaceOrigPos = mt->origpos;
            qDebug() << "Chose mass" << i;

            // Pick a random direction
            Vec dir = Utils::randDirectionVec();
            qDebug() << "Direction" << dir[0] << dir[1] << dir[2];
            Vec dx = displacement * dir;
            mg->dx = dx;

            // Move mass
            qDebug() << "Shifting mass" << mg->displaced->index << dx[0] << dx[1] << dx[2];
            shiftMassPos(sim, mg->displaced, dx);
        }

        // Run simulation
        // Equilibrate simulation
        if (relaxation == 0) {
            settleSim(sim, 1E-6);
        } else {
            relaxSim(sim, relaxation);
        }


        for (MassGroup *mg : massGroups) {
            // Calculate test metrics
            mg->testEnergy = calcMassGroupEnergy(mg);
            mg->testLength = calcMassGroupLength(mg);
            disPos.push_back(mg->displaced->pos);
        }



        int mgi = 0;
        for (MassGroup *mg : massGroups) {

            double origMetric = mg->origLength * mg->origEnergy;
            double testMetric = mg->testLength * mg->testEnergy;

            qDebug() << "MG length Sim" << mg->origLength << " Test" << mg->testLength;
            qDebug() << "MG energy Sim" << mg->origEnergy << " Test" << mg->testEnergy;
            qDebug() << "MG metric Sim" << origMetric << " Test" << testMetric;

            for (int i = 0; i < mg->group.size(); i++) {
                mg->group[i]->pos = mg->startPos[i];
                mg->group[i]->vel = Vec(0,0,0);
            }
            mg->displaced->pos += mg->dx; // Set off following call
            shiftMassPos(sim, mg->displaced, -mg->dx);

            if (testMetric < origMetric) {
                mg->displacements.push_back(mg->dx);
                mg->displacedList.push_back(mg->displaced);
                qDebug() << "Moved " << mg->displaced->index;
                result++;
            }
            mgi++;
        }
        attempts++;
    }
    for (Mass *m : sim->masses) {
        for (MassGroup *mg : massGroups) {
            auto it = find(mg->displacedList.begin(), mg->displacedList.end(), m);
            if (it != mg->displacedList.end()) {

                int d = distance(mg->displacedList.begin(), it);
                shiftMassPos(sim, mg->displacedList[d], mg->displacements[d]);
            }
        }
    }

    combineMassTiles(sim, massGroups, startBorder, startMassSpan);
    for (int j = 0; j < sim->masses.size(); j++) {
        sim->masses[j]->extforce = startForces[j];
        sim->masses[j]->pos = startPos[j];
        sim->masses[j]->m = startMass[j];
    }
    sim->setAll();

    return result;

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
        massGroup.groupStart = vector<Spring>();

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
    massGroup.candidates = vector<Mass *>();
    massGroup.springs = vector<Spring *>();
    massGroup.outside = vector<Mass *>();
    massGroup.edge = vector<Mass *>();
    massGroup.border = vector<Spring *>();
    massGroup.displacedList = vector<Mass *>();

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
    vector<Mass *> culledCandidateGroup = vector<Mass *>();
    vector<Mass *> culledOutsideGroup = vector<Mass *>();
    vector<Mass *> culledEdgeGroup = vector<Mass *>();
    for (Mass *m : sim->masses) {
        bool underExternalForce = m->extforce.norm() > 1E-6;
        bool fixed = m->constraints.fixed;
        bool edge = find(massGroup.edge.begin(), massGroup.edge.end(), m) != massGroup.edge.end();

        if (find(massGroup.group.begin(), massGroup.group.end(), m) != massGroup.group.end()) {
            culledOrderGroup.push_back(m);
        }
        if (find(massGroup.outside.begin(), massGroup.outside.end(), m) != massGroup.outside.end()) {
            culledOutsideGroup.push_back(m);
        }
        if (edge) {
            culledEdgeGroup.push_back(m);
        }
        if (!underExternalForce && !fixed &&!edge) {
            if (find(massGroup.group.begin(), massGroup.group.end(), m) != massGroup.group.end()) {
                culledCandidateGroup.push_back(m);
            }
        }
    }
    massGroup.group = culledOrderGroup;
    massGroup.candidates = culledCandidateGroup;
    massGroup.outside = culledOutsideGroup;
    massGroup.edge = culledEdgeGroup;

}


// Create 1D tile from a span between minPos and maxPos
// Minimum tile width is unit unless span < unit
// Middle tiles will be offset to enable grid shifting
// Outputs start and end values for tile
// Returns 1 if tile is created, 0 if not
//---------------------------------------------------------------------------
int MassDisplacer::createTile(int n, int i, double width, double offset, double minPos, double &tileStart,
                              double &tileEnd) {
//---------------------------------------------------------------------------

    if (n < 3) { // Small tile doesn't get split
        if (i == 0) {
            tileStart = minPos;
            tileEnd = minPos + 3 * unit;
        } else
            return 0;

    } else {
        if (i == 0) { // Start tile begins at 2x length
            tileStart = minPos;
            tileEnd = minPos + 2 * unit - offset;
        } else if (i == n - 1) // Throw out end tile
            return 0;
        else if (i == n - 2) { // Next end tile begins with remainder from split
            tileStart = minPos + (i + 1) * unit - offset;
            tileEnd = minPos + (i + 3) * unit;
        } else { // Middle tiles are 1 unit length
            tileStart = minPos + (i + 1) * unit - offset;
            tileEnd = minPos + (i + 2) * unit - offset;
        }
    }
    return 1;
}


// Create cubic tiles of a lattice with springs in between
//---------------------------------------------------------------------------
void MassDisplacer::createMassTiles(Simulation *sim, double unit, Vec offset, vector<MassGroup *> &mgs,
                                    map<Mass *, MassGroup *> &mgm, vector<Spring *> &ts) {
//---------------------------------------------------------------------------

    int nx, ny, nz;
    mgs = vector<MassGroup *>();
    mgm = map<Mass *, MassGroup *>();
    ts = vector<Spring *>();

    Vec minPos = Vec(FLT_MAX, FLT_MAX, FLT_MAX);
    Vec maxPos = Vec(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (Mass *m : sim->masses) {
        minPos[0] = std::min(m->pos[0], minPos[0]);
        minPos[1] = std::min(m->pos[1], minPos[1]);
        minPos[2] = std::min(m->pos[2], minPos[2]);
        maxPos[0] = std::max(m->pos[0], maxPos[0]);
        maxPos[1] = std::max(m->pos[1], maxPos[1]);
        maxPos[2] = std::max(m->pos[2], maxPos[2]);
    }

    this->dimensions = maxPos - minPos;
    nx = ceil(dimensions[0] / unit);
    ny = ceil(dimensions[1] / unit);
    nz = ceil(dimensions[2] / unit);

    if (nx > 1) nx--;
    if (ny > 1) ny--;
    if (nz > 1) nz--;
    qDebug() << "Grid" << nx << ny << nz;

    double xst, yst, zst;
    double xen, yen, zen;
    int tx, ty, tz;

    for (int x = 0; x < nx; x++) {

        tx = createTile(nx, x, unit, offset[0], minPos[0], xst, xen);

        for (int y = 0; y < ny; y++) {

            ty = createTile(ny, y, unit, offset[1], minPos[1], yst, yen);

            for (int z = 0; z < nz; z++) {

                tz = createTile(nz, z, unit, offset[2], minPos[2], zst, zen);

                qDebug() << "Ts" << tx << ty << tz;
                if (tx && ty && tz) {
                    auto *mg = new MassGroup();
                    createMassGroup(sim, Vec(xst, yst, zst), Vec(xen, yen, zen), *mg);
                    qDebug() << "Created mass group" << mg->group.size();

                    if (!mg->candidates.empty()) {
                        mgs.push_back(mg);
                    }
                    for (Mass *m : mg->group) {
                        mgm[m] = mg;
                    }
                    for (Spring *s : mg->border) {
                        ts.push_back(s);
                    }
                }

            }
        }
    }

    // Cull trench springs
    vector<Spring *> culledTrenchSprings = vector<Spring *>();
    for (Spring *s1 : ts) {
        bool duplicate = false;
        for (Spring *s2 : culledTrenchSprings) {
            if (s1 == s2) duplicate = true;
        }
        if (!duplicate) culledTrenchSprings.push_back(s1);
    }
    ts = culledTrenchSprings;

    qDebug() << "Created mass tiles" << mgs.size();
    qDebug() << "Trench springs" << ts.size();
    for (MassGroup *mg : mgs) {
        qDebug() << "Mass Group" << mg->group.size() << mg->springs.size();
    }
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


// Split simulation into tiled chunks by deleting inbetween springs
//---------------------------------------------------------------------------
void MassDisplacer::splitMassTiles(Simulation *sim, vector<MassGroup *> &mgs, vector<Spring *> &tsSim,
                                   vector<Spring> &tsSave, vector<Mass *> &massSpans) {
//---------------------------------------------------------------------------

    for (MassGroup *mg : mgs) {
        for (Mass *m : mg->edge) {
            if (!m->constraints.fixed) {
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
                for (Spring *s : mg->springs) {
                    if (s->_right == m) {
                        m->extforce -= s->getForce();
                    }
                    if (s->_left == m) {
                        m->extforce += s->getForce();
                    }
                }
                m->force = m->extforce;
            }
        }

    }
    for (Spring *s : tsSim) {
        Spring t = Spring(*s);
        tsSave.push_back(t);
        massSpans.push_back(s->_left);
        massSpans.push_back(s->_right);
        //s->_left->m -= s->_left->m / s->_left->ref_count;
        //s->_right->m -= s->_right->m / s->_right->ref_count;
        sim->deleteSpring(s);
    }
}

// Combine simulation from tiled chunks by recreating inbetween springs
//---------------------------------------------------------------------------
void MassDisplacer::combineMassTiles(Simulation *sim, vector<MassGroup *> &massGroups, vector<Spring> &tsSave,
                                     vector<Mass *> massSpans) {
//---------------------------------------------------------------------------

    for (int s = 0; s < tsSave.size(); s++) {
        Spring *n = new Spring(tsSave[s]);
        n->setMasses(massSpans[s*2], massSpans[s*2+1]);
        for (MassGroup *mg : massGroups) {
            if (n->_left == mg->displaced) {
                qDebug() << "Connected spring" << n->_rest;
                double origLen = n->_rest;
                n->_rest = (n->_right->origpos - mg->displaced->origpos).norm();
                if (n->_rest < 0.001) {
                    n->_rest = origLen;
                    mg->testEnergy = FLT_MAX; // Automatically reject
                }
                n->_k *= origLen / n->_rest;
                qDebug() << "Set" << n->_k << n->_rest;
            }
            if (n->_right == mg->displaced) {
                qDebug() << "Connected spring" << n->_rest;
                double origLen = n->_rest;
                n->_rest = (n->_left->origpos - mg->displaced->origpos).norm();
                if (n->_rest < 0.001) {
                    n->_rest = origLen;
                    mg->testEnergy = FLT_MAX; // Automatically reject
                }
                n->_k *= origLen / n->_rest;
            }
        }
        //n->_left->m = n->_left->m / (1 - 1.0/(n->_left->ref_count + 1));
        //n->_right->m = n->_right->m / (1 - 1.0/(n->_right->ref_count + 1));

        sim->createSpring(n);
    }
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


// Calculate length of the springs in a mass group
//---------------------------------------------------------------------------
double MassDisplacer::calcMassGroupLength(MassDisplacer::MassGroup *massGroup) {
//---------------------------------------------------------------------------

    double length = 0;
    for (Spring *s : massGroup->springs) {
        length += s->_rest;
    }
    return length;
}


// Calculate length of the springs in a mass group
//---------------------------------------------------------------------------
double MassDisplacer::calcMassGroupEnergy(MassDisplacer::MassGroup *massGroup) {
//---------------------------------------------------------------------------

    double energy = 0;
    for (Spring *s : massGroup->springs) {
        energy += s->_curr_force * s->_curr_force / s->_k;
    }
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
    sortSprings_stress(sim->springs, springIndicesToSort);

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
