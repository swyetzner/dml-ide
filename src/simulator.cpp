#include "simulator.h"
#include <QDir>

Simulator::Simulator(Simulation *sim, Loader *loader, SimulationConfig *config, OptimizationConfig *optConfig,
        bool graphics, bool endExport) {
    this->sim = sim;
    this->config = config;
    this->optConfig = optConfig;
    this->loader = loader;
    this->optimizer = nullptr;

    n_masses = long(sim->masses.size());
    n_springs = long(sim->springs.size());
    n_planes = long(sim->planes.size());
    n_masses_start = long(sim->masses.size());
    n_springs_start = long(sim->springs.size());
    totalMass_start = sim->getTotalMass();
    deflectionPoint_start = getDeflectionPoint();
    totalEnergy = 0;
    totalLength = 0;
    totalEnergy_prev = 0;
    totalLength_prev = 0;
    totalLength_start = 0;
    totalEnergy_start = 0;
    springInserter = nullptr;
    springRemover = nullptr;
    massDisplacer = nullptr;
    OPTIMIZER = optConfig != nullptr;

    double pi = atan(1.0)*4;
    for (Spring *s : sim->springs) {
        if (s->_k != 0) {
            totalLength_start += s->_rest;
        }
    }

    steps = 0;

    simStatus = NOT_STARTED;
    dataDir = QDir::currentPath() + QDir::separator() + "data";

    n_repeats = 0;
    repeatTime = config->repeat.after;
    explicitRotation = config->repeat.rotationExplicit;
    repeatRotation = config->repeat.rotation;
    optimizeAfter = (repeatTime > 0)? 10 : 0;

    barData = nullptr;
    GRAPHICS = graphics;
    EXPORT = endExport;

    relaxation = 3000;

    if (OPTIMIZER) loadOptimizers();
    //optimizer = new MassDisplacer(sim, 0.2);
    //springInserter = new SpringInserter(sim, 0.001);
    //springInserter->cutoff = 3.5 * config->lattice.unit[0];

    // LOAD QUEUE
    currentLoad = 0;
    pastLoadTime = 0;
    optimizeTime = 0;
    varyLoad = false;
    if (config->load != nullptr) {
        for (Force *f : config->load->forces) {
            if (!(f->vary == Vec(0,0,0))) varyLoad = true;
        }
    }
    for (Loadcase *l : config->loadQueue) {
        qDebug() << "Force masses" << l->forces.front()->masses.size();
        for (Force *f : l->forces) {
            if (!(f->vary == Vec(0,0,0))) varyLoad = true;
        }
    }

    center = getSimCenter();

    equilibrium = false;
    optimized = 0;
    closeToPrevious = 0;
    stepsSinceEquil = 0;
    prevEnergy = -1;
    prevSteps = 0;
    switched = false;

    if (!GRAPHICS) {
        for (int i = 0; i < 30; i++) {
            printf("\n");
        }
        printf("\033[10;1f");
    }

    wallClockTime = 0;
    prevWallClockTime = 0;

    qDebug() << "Initialized Simulator";
}

Simulator::~Simulator() {
    delete barData;
    delete springInserter;
    delete springRemover;
    delete massDisplacer;
}

// --------------------------------------------------------------------
// SIMULATION CONTROLS
// --------------------------------------------------------------------

void Simulator::setSyncTimestep(double st) {
    renderTimeStep = st;
}

void Simulator::setSimTimestep(double dt) {
    sim->setAllDeltaTValues(dt);
}

void Simulator::setDataDir(std::string dp) {
    if (!dp.empty()) {
        dataDir = QString::fromStdString(dp);
    }
}

void Simulator::runSimulation(bool running) {
    if (running) {
        if (simStatus == NOT_STARTED) {
            createDataDir();
            sim->initCudaParameters();
            dumpSpringData();
            startWallClockTime = std::chrono::system_clock::now();
        }
        //if (simStatus == PAUSED) dumpSpringData();
        simStatus = STARTED;
        run();
        if (!GRAPHICS) printStatus();
    } else {
        simStatus = PAUSED;
    }
}

void Simulator::runStep() {
    simStatus = PAUSED;
    sim->step(sim->masses.front()->dt);
    sim->getAll();
}

void Simulator::getSimMetrics(sim_metrics &metrics) {
    metrics.clockTime = wallClockTime;
    metrics.time = sim->time();
    metrics.nbars = sim->springs.size();
    metrics.totalLength = totalLength;
    metrics.totalEnergy = totalEnergy;
    metrics.totalLength_start = totalLength_start;
    metrics.totalEnergy_start = totalEnergy_start;
    metrics.deflection = calcDeflection();
    metrics.relaxation_interval = relaxation;
    metrics.optimize_iterations = optimized;
    metrics.optimize_rule = (OPTIMIZER && !optConfig->rules.empty())? optConfig->rules.front() : OptimizationRule();
    metrics.displacement = massDisplacer? massDisplacer->dx : 0;
}

void Simulator::dumpSpringData() {
    cout << "DUMPING SPRING DATA\n";
    QString dumpFile = QString(dataDir + QDir::separator() +
                               "simDump_%1.txt").arg(optimized);
    writeSimDump(dumpFile);
}

void Simulator::loadSimDump(std::string sp) {
    int nm = 0, ns = 0;
    double time = 0;
    double slen, clen;
    double sener, cener;

    ifstream dumpFile(sp, ios::in);
    if (dumpFile.is_open()) {
        std::string line;
        // First line
        if (!(getline(dumpFile, line) && Utils::startsWith(line, "SIMULATION DUMP"))) {
            cout << "Sim dump file is in an unrecognizable format." << std::endl;
            exit(1);
        }
        sscanf(line.c_str(), "SIMULATION DUMP; Time: %lf, Springs: %d, Masses: %d", &time, &ns, &nm);
        // Stat lines
        getline(dumpFile, line);
        sscanf(line.c_str(), "Start Length: %lf, Start Energy %lf", &slen, &sener);
        totalLength_start = slen;
        totalEnergy_start = sener;
        getline(dumpFile, line);
        sscanf(line.c_str(), "Current Length: %lf, Current Energy %lf", &clen, &cener);
        totalLength = clen;
        totalEnergy = cener;
        totalLength_prev = totalLength;
        totalEnergy_prev = totalEnergy;

        int ss = sim->springs.size();
        int sm = sim->masses.size();
        vector<Spring *> delSpring = vector<Spring *>();
        for (int i = ns; i < ss; i++) {
            delSpring.push_back(sim->getSpringByIndex(i));
        }
        vector<Mass *> delMass = vector<Mass *>();
        for (int i = nm; i < sm; i++) {
            delMass.push_back(sim->getMassByIndex(i));
        }
        for (auto s : delSpring) {
            sim->deleteSpring(s);
        }
        for (auto m : delMass) {
            sim->deleteMass(m);
        }
        while (getline(dumpFile, line)) {
            if (Utils::startsWith(line, "Mass")) {
                Vec p, f;
                double m;
                int i;
                sscanf(line.c_str(), "Mass %d (%lf, %lf, %lf) (%lf, %lf, %lf) %lf", &i, &p[0], &p[1], &p[2],
                        &f[0], &f[1], &f[2], &m);

                if (i < sim->masses.size()) {
                    sim->masses[i]->origpos = p;
                    sim->masses[i]->pos = p;
                    sim->masses[i]->m = m;
                    sim->masses[i]->extforce = f;
                    sim->masses[i]->force = f;
                    sim->masses[i]->extduration = FLT_MAX;
                    sim->masses[i]->vel = Vec(0,0,0);
                    (Utils::endsWith(line, "F"))? sim->masses[i]->fix() : sim->masses[i]->unfix();
                } else {
                    sim->createMass(p);
                    sim->masses[i]->m = m;
                    sim->masses[i]->extforce = f;
                    sim->masses[i]->extduration = FLT_MAX;
                    if (Utils::endsWith(line, "F")) sim->masses[i]->fix();
                    if (f.norm() > 0) {
                        qDebug() << "Force on" << sim->masses[i]->pos[0];
                    }
                }
            }
            if (Utils::startsWith(line, "Spring")) {
                int l, r, i;
                double k, d;
                sscanf(line.c_str(), "Spring %d %d %d %lf %lf", &i, &l, &r, &k, &d);

                assert(l < sim->masses.size() && r < sim->masses.size());
                Mass *m1 = sim->masses[l];
                Mass *m2 = sim->masses[r];
                double rest = (m1->origpos - m2->origpos).norm();

                if (i < sim->springs.size()) {
                    sim->springs[i]->setMasses(m1, m2);
                    double a = 3.14159 * (d / 2) * (d / 2);
                    double unit = 1;
                    sim->springs[i]->_k = k;
                    sim->springs[i]->_diam = d;
                    sim->springs[i]->_rest = rest;

                } else {
                    Spring *spring = new Spring(m1, m2, k, rest, d);
                    int unit = 1;
                    if (config->lattices[0]->material->yUnits == "GPa") { unit *= 1000 * 1000 * 1000; }
                    if (config->lattices[0]->material->yUnits == "MPa") { unit *= 1000 * 1000; }
                    spring->_break_force = 5 * config->lattices[0]->material->yield * unit;
                    sim->createSpring(spring);
                }
            }
        }
        n_springs = sim->springs.size();
        n_masses = sim->masses.size();
        //sim->setAll();
        // DAMPING
        for (Mass *m : sim->masses) {
            m->damping = 1.0 - config->damping.velocity;
        }
        qDebug() << "Damping" << config->damping.velocity;

        // GLOBAL
        qDebug() << "Global" << config->global.acceleration[0];
        sim->global = config->global.acceleration;
        loadOptimizers();
        optimized = 0;
    } else {
        cout << "Cannot open file for loading: " << sp << std::endl;
        exit(1);
    }
}

void Simulator::exportSimulation() {
    int NUM_THREADS = 32;

    qDebug() << "In exportSimulation";
    delete barData;
    barData = new bar_data();
    loader->loadBarsFromSim(sim, barData, false, false);

    if (!config->output) {
        config->output = new output_data();
    }
    qDebug() << config->output->id;
    config->output->barData = barData;
    qDebug() << "Saved" << barData->bars.size() << "bars from simulation";

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    ostringstream oss;
    oss << put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    string tmString = oss.str();

    cout << "Starting export...\n";
    Polygonizer *polygonizer = new Polygonizer(config->output,
                                               sim->springs[0]->_diam * 0.5,
                                               sim->springs[0]->_diam,
                                               NUM_THREADS);
    polygonizer->initBaseSegments();
    polygonizer->calculatePolygon();
    polygonizer->writePolygonToSTL(tmString + ".stl");
    delete polygonizer;
}

void Simulator::run() {

    if (!sim->running()) {
        qDebug() << "Next Load" << currentLoad << "Queue size" << config->loadQueue.size() << "Switch at time" << pastLoadTime;
        bool loadQueueDone = false;

        // Set repeats
        if (repeatTime > 0 && repeatTime < sim->time()) {
            repeatLoad();
        }

        bool currLoadDone = sim->time() >= pastLoadTime;
        if (currLoadDone && !config->loadQueue.empty()) {
            // Load queue
            if (currentLoad >= config->loadQueue.size()) {
                loadQueueDone = true;
            } else {
                Loadcase *load = config->loadQueue[currentLoad];
                clearLoads();
                applyLoad(load);
                currentLoad++;
                pastLoadTime += load->totalDuration;
            }
        }

        sim->step(renderTimeStep);
        qDebug() << "Stepped" << steps << "Repeats" << n_repeats;
        sim->getAll();
        totalLength_prev = totalLength;
        totalLength = 0;
        double maxForce = 0;
        Spring *maxForceSpring = nullptr;
        int i = 0, n = 0;
        for (Spring *s: sim->springs) {
            if (s == nullptr) continue;

            if (s->_k == 0) continue;
            totalLength += s->_rest;
            if (maxForce < fabs(s->_curr_force)) {
                maxForce = fabs(s->_curr_force);
                maxForceSpring = s;
                n = i;
            }
            i++;
        }
        if (maxForceSpring != nullptr) qDebug() << "MAX FORCE SPRING" << n << maxForce << maxForceSpring->_rest << (maxForceSpring->_left->pos - maxForceSpring->_right->pos).norm();

        bool stopReached = stopCriteriaMet();
        qDebug() << "Removed springs" << springRemover->removedSprings.size();

        if (!optimized) {
            if (varyLoad) {
                varyLoadDirection();
            }
        }

        bool equilibriumMetric = optConfig != nullptr && optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE;
        /**if (equilibriumMetric && totalEnergy / totalEnergy_start < 0.1) {
            equilibriumMetric = false;
            optimizer = springRemover;
            switched = true;
            //emit stopCriteriaSat();
        }**/
        if (equilibriumMetric) {

            equilibriate();
            if (stepsSinceEquil > 1000) {
                for (Mass *m : sim->masses) {
                    m->pos = m->origpos;
                    m->vel = Vec(0,0,0);
                    m->acc = Vec(0,0,0);
                    m->force = Vec(0,0,0);
                }
                for (Spring *s : sim->springs) {
                    double l = (s->_left->pos - s->_right->pos).norm();
                    assert(s->_rest <= l + 1E-6 && s->_rest >= l - 1E-6);
                    s->_curr_force = 0;
                }
                //equilibrium = true;
                sim->setAll();
                stepsSinceEquil = 0;
                equilibrium = true;
            }

            if (optimizeAfter <= n_repeats && equilibrium && !stopReached) {

                if (optimizer != nullptr) {
                    if (!optimized) {
                        massDisplacer->lastMetric = totalLength * totalEnergy;
                        writeMetricHeader(metricFile);
                        writeCustomMetricHeader(customMetricFile);
                    }

                    qDebug() << "About to optimize";
                    optimizer->optimize();
                    equilibrium = false;
                    closeToPrevious = 0;
                    cout << "Average trial time (simulation): " << massDisplacer->totalTrialTime / massDisplacer->totalAttempts << "s \n";

                    if (varyLoad) {
                        varyLoadDirection();
                    }

                    writeMetric(metricFile);
                    if (optimized == 0)
                        writeCustomMetric(customMetricFile);
                    optimized++;

                    cout << "Average iteration time (simulation): " << massDisplacer->totalTrialTime / optimized << "s \n";
                }

                prevSteps = 0;
            }
            prevEnergy = totalEnergy;
            if (optimized) stepsSinceEquil++;

        } else {

            if (optConfig != nullptr) {
                if (switched) {
                    optimizer->optimize();
                    //writeMetric(metricFile);

                    optimized++;

                    n_masses = int(sim->masses.size());
                    n_springs = int(sim->springs.size());
                    prevSteps = 0;


                    currentLoad = 0;
                } else {
                    for (OptimizationRule r : optConfig->rules) {
                        if ((loadQueueDone || config->repeat.afterExplicit) && optimizeAfter <= n_repeats &&
                            prevSteps >= r.frequency && !stopReached) {

                            if (!optimized && n_repeats == 0) {
                                writeMetricHeader(metricFile);
                                deflection_start = calcDeflection();
                            }

                            qDebug() << "OPTIMIZING";
                            writeMetric(metricFile);
                            double simTimeBeforeOpt = sim->time();

                            if (calcDeflection() > deflection_start * 10) {
                                qDebug() << "Deflection" << calcDeflection() << deflection_start;
                                springRemover->resetHalfLastRemoval();
                            } else {
                                optimizer->optimize();
                                if (!springRemover->regeneration) optimized++;
                                qDebug() << "Removed spring post opt" << springRemover->removedSprings.size();
                                n_repeats = optimizeAfter > 0 ? optimizeAfter - 1 : 0;
                            }


                            n_masses = int(sim->masses.size());
                            n_springs = int(sim->springs.size());
                            prevSteps = 0;

                            currentLoad = 0;
                                if (springRemover->regeneration && !optConfig->rules.empty()) {
                                    if (totalLength <= totalLength_start * optConfig->rules.front().regenThreshold) {
                                        springRemover->regenerateLattice(config);
                                        optimized++;
                                        //springRemover->regenerateShift();
                                        n_masses = int(sim->masses.size());
                                        n_springs = int(sim->springs.size());
                                    }

                                }
                            // Account for time shift
                            optimizeTime = sim->time() - simTimeBeforeOpt;
                            qDebug() << "OPTIMIZE TIME" << optimizeTime;
                            pastLoadTime += optimizeTime;

                        }
                    }
                }
            }
        }


        steps += long(renderTimeStep / sim->masses.front()->dt);
        prevSteps += long(renderTimeStep / sim->masses.front()->dt);
        qDebug() << steps;


        if (stopReached) {
            simStatus = STOPPED;
            //dumpSpringData();
            //if (EXPORT) exportSimulation();
            exit(0);
        }
    }

    auto currentWallClockTime = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = (currentWallClockTime - startWallClockTime);
    prevWallClockTime = wallClockTime;
    wallClockTime = diff.count();

    if (dumpCriteriaMet()) dumpSpringData();

    qDebug() << "WALL CLOCK TIME" << wallClockTime;
}


void Simulator::repeatLoad() {

    if (!sim->running()) {

        if (n_repeats == 0) {
            writeMetricHeader(metricFile);
        }
        writeMetric(metricFile);

        // Get rotation
        Vec rotation;
        if (explicitRotation) {
            rotation = repeatRotation;
        } else {
            // Random
            rotation = Utils::randDirectionVec();
        }

        // Set masses back to original positions
        for (Mass *m : sim->masses) {
            m->pos = m->origpos;

            // rotate positions
            QVector4D p = QVector4D(m->pos[0], m->pos[1], m->pos[2], 0);
            p -= QVector4D(center[0], center[1], center[2], 1);

            QMatrix4x4 rot;
            rot.setToIdentity();
            rot.rotate(rotation[0] * 360, 1.0f, 0.0f, 0.0f);
            rot.rotate(rotation[1] * 360, 0.0f, 1.0f, 0.0f);
            rot.rotate(rotation[2] * 360, 0.0f, 0.0f, 1.0f);
            p = rot * p;

            p += QVector4D(center[0], center[1], center[2], 1);
            m->pos = Vec(p.x(), p.y(), p.z());
            m->vel = Vec(0, 0, 0);
            m->acc = Vec(0, 0, 0);
        }
        qDebug() << "Center" << center[0] << center[1] << center[2];

        // Increase repeat time
        repeatTime += config->repeat.after;
        sim->setAll();

        n_repeats++;
    }
}

void Simulator::loadOptimizers() {
    if (optConfig != nullptr) {
        for (OptimizationRule r : optConfig->rules) {
            switch(r.method) {
                case OptimizationRule::REMOVE_LOW_STRESS:
                    springRemover = new SpringRemover(sim, r.threshold);
                    springRemover->massFactor = M_PI * (sim->springs.front()->_diam / 2) * (sim->springs.front()->_diam / 2) *
                                                config->lattices[0]->material->density * ((config->lattices[0]->material->dUnits == "gcc")? 1000 : 1);
                    springRemover->stressMemory = r.memory;
                    if (r.regenThreshold > 0) {
                        springRemover->regeneration = true;
                        springRemover->regenRate = r.regenRate;
                    }
                    this->optimizer = springRemover;
                    qDebug() << "Created SpringRemover" << r.threshold;
                    break;

                case OptimizationRule::MASS_DISPLACE: {
                    double minUnitDist = DBL_MAX;
                    for (Spring *t : sim->springs) {
                        minUnitDist = fmin(minUnitDist, t->_rest);
                    }

                    double mf = M_PI * (sim->springs.front()->_diam / 2) * (sim->springs.front()->_diam / 2) *
                            config->lattices[0]->material->density * ((config->lattices[0]->material->dUnits == "gcc")? 1000 : 1);
                    massDisplacer = new MassDisplacer(sim, config->lattices[0]->unit[0] * 0.2, r.threshold, mf);
                    massDisplacer->maxLocalization = minUnitDist + 1E-4;
                    massDisplacer->order = 0;
                    massDisplacer->chunkSize = 0;
                    massDisplacer->relaxation = relaxation;
                    massDisplacer->springUnit = config->lattices.front()->unit[0];
                    massDisplacer->unit = massDisplacer->springUnit * 6;
                    this->optimizer = massDisplacer;
                    qDebug() << "Created MassDisplacer" << r.threshold;
                    break;
                }

                case OptimizationRule::NONE:
                    optimizer = nullptr;
                    optimizer = nullptr;
                    break;
            }
        }
    }
    qDebug() << "Set optimizations";
}

Vec Simulator::getSimCenter() {
    Vec minCorner;
    Vec maxCorner;

    double maxX = -numeric_limits<double>::max();
    double maxY = -numeric_limits<double>::max();
    double maxZ = -numeric_limits<double>::max();
    double minX = numeric_limits<double>::max();
    double minY = numeric_limits<double>::max();
    double minZ = numeric_limits<double>::max();

    for (Mass *m : sim->masses) {
        maxX = std::max(maxX, m->pos[0]);
        maxY = std::max(maxY, m->pos[1]);
        maxZ = std::max(maxZ, m->pos[2]);
        minX = std::min(minX, m->pos[0]);
        minY = std::min(minY, m->pos[1]);
        minZ = std::min(minZ, m->pos[2]);
    }
    minCorner = Vec(minX, minY, minZ);
    maxCorner = Vec(maxX, maxY, maxZ);

    qDebug() << "Max Corner" << maxCorner[0] << maxCorner[1] << maxCorner[2];
    qDebug() << "Min Corner" << minCorner[0] << minCorner[1] << minCorner[2];
    return 0.5 * (minCorner + maxCorner);
}

void Simulator::equilibriate() {
    totalEnergy_prev = totalEnergy;
    totalEnergy = 0;
    for (Spring *s : sim->springs) {
        totalEnergy += s->_curr_force * s->_curr_force / s->_k;
    }
    qDebug() << "ENERGY" << totalEnergy << prevEnergy << closeToPrevious << stepsSinceEquil;
    if (prevEnergy > 0 && fabs(prevEnergy - totalEnergy) < totalEnergy * 1E-6) {
        closeToPrevious++;
    } else {
        closeToPrevious = 0;
    }
    if (closeToPrevious > 10) {
        equilibrium = true;
        stepsSinceEquil = 0;
        if (!optimized) {
            totalEnergy_start = totalEnergy;
            writeMetricHeader(metricFile);
            writeCustomMetricHeader(customMetricFile);
        }
    }
}

bool Simulator::stopCriteriaMet() {
    bool stopReached = false;
    if (OPTIMIZER) {
        for (auto s : optConfig->stopCriteria) {
            switch (s.metric) {
                case OptimizationStop::ENERGY:
                    stopReached = totalEnergy / totalEnergy_start <= s.threshold;
                    break;
                case OptimizationStop::WEIGHT:
                    stopReached = totalLength / totalLength_start <= s.threshold;
                    break;
                case OptimizationStop::DEFLECTION:
                    stopReached = calcDeflection() >= s.threshold;
                    break;
                case OptimizationStop::ITERATIONS:
                    stopReached = optimized >= s.threshold;
                    break;
                case OptimizationStop::NONE:
                    stopReached = false;
                    break;
            }
        }
    }
    return stopReached;
}

bool Simulator::dumpCriteriaMet() {
    bool dump = false;
    qDebug() << "Dump criteria" << int(floor(wallClockTime)) % (60 * 5) <<  int(floor(prevWallClockTime)) % (60 * 5);
    if (int(floor(wallClockTime)) % 60 < int(floor(prevWallClockTime)) % 60) {
        return true;
    } return false;

    if (OPTIMIZER) {
        for (auto s : optConfig->stopCriteria) {
            double interval = (1 - s.threshold) / 10;
            switch (s.metric) {
                case OptimizationStop::ENERGY:
                    for (int i = 0; i < 10; i++) {
                        double marker = 1 - (i * interval);
                        if ((totalEnergy / totalEnergy_start) <= marker
                            && (totalEnergy_prev / totalEnergy_start) > marker) {
                            return true;
                        }
                    }
                    break;
                case OptimizationStop::WEIGHT:
                    for (int i = 0; i < 10; i++) {
                        double marker = 1 - (i * interval);
                        if ((totalLength / totalLength_start) <= marker
                            && (totalLength_prev / totalLength_start) > marker) {
                            return true;
                        }
                    }
                    break;
                case OptimizationStop::DEFLECTION:
                    dump = false;
                    break;
                case OptimizationStop::ITERATIONS:
                    interval = s.threshold / 10;
                    for (int i = 0; i < 10; i++) {
                        double marker = i * interval;
                        if ((optimized) <= marker && (optimized + 1) > marker) {
                            return true;
                        }
                    }
                case OptimizationStop::NONE:
                    dump = false;
                    break;
            }
        }
    }
    return dump;
}

double Simulator::calcDeflection() {
    double deflection = 0;
    vector<Mass *> points = vector<Mass *>();
    if (config->load && !config->load->forces.empty()) {
        for (Force *f : config->load->forces) {
            for (Mass *m : f->masses) {
                points.push_back(m);
                deflection += (m->origpos - m->pos).norm();
            }
        }
        deflection /= points.size();
    } else {
        assert(!sim->masses.empty());
        Mass *refMass = sim->masses.front();
        for (Mass *m : sim->masses) {
            double origDist = (m->origpos - refMass->origpos).norm();
            double newDist = (m->pos - refMass->pos).norm();
            deflection += fabs(newDist - origDist);
        }
    }
    return deflection;
}

Vec Simulator::getDeflectionPoint() {
    Vec deflectionPoint = Vec(0,0,0);
    vector<Mass *> points = vector<Mass *>();
    if (config->load && !config->load->forces.empty()) {
        for (Force *f : config->load->forces) {
            for (Mass *m : f->masses) {
                points.push_back(m);
            }
        }
        for (Mass *p : points) {
            deflectionPoint += p->origpos;
        }
        deflectionPoint[0] /= points.size();
        deflectionPoint[1] /= points.size();
        deflectionPoint[2] /= points.size();
    }

    return deflectionPoint;
}

void Simulator::createDataDir() {

    qDebug() << "CREATE DATA DIR";
    QDir data(dataDir);
    if (!data.exists()) {
        qDebug() << "Data folder does not exist. Creating...";
        QDir::home().mkdir(dataDir);
    } else {
        data.removeRecursively();
        qDebug() << QDir::current().path();
        QDir::home().mkdir(dataDir);
    }

    // Create metric file
    metricFile = QString(dataDir + QDir::separator() +
                        "optMetrics.csv");
    customMetricFile = QString(dataDir + QDir::separator() +
                               "outsideForces.csv");
}

void Simulator::writeMetricHeader(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {
        return;
    }

    if (!optConfig->rules.empty()) {
        if (optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE) {
            file.write("Wall Clock,Time,Iteration,Deflection,Displacement,Attempts,Total Energy,Total Weight\n");
        } else {
            file.write("Wall Clock,Time,Iteration,Deflection,Total Weight,Bar Number\n");
        }
    }
}

void Simulator::writeCustomMetricHeader(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {
        return;
    }

    if (!optConfig->rules.empty()) {
        if (optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE) {
            file.write(massDisplacer->customMetricHeader.toUtf8());
        }
    }
}

void Simulator::writeMetric(const QString &outputFile) {

    QFile file(outputFile);
    qDebug() << "WRITE METRIC";
    if (!file.open(QFile::WriteOnly | QIODevice::Append | QFile::Text)) {
        return;
    }

    if (!optConfig->rules.empty()) {
        if (optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE) {
            QString mLine = QString("%1,%2,%3,%4,%5,%6,%7,%8\n")
                    .arg(wallClockTime)
                    .arg(optimized? massDisplacer->totalTrialTime / optimized : 0)
                    .arg(optimized)
                    .arg(calcDeflection())
                    .arg(massDisplacer->dx)
                    .arg(massDisplacer->attempts)
                    .arg(totalEnergy)
                    .arg(totalLength);
            file.write(mLine.toUtf8());
        } else if (optConfig->rules.front().method == OptimizationRule::REMOVE_LOW_STRESS) {
            QString mLine = QString("%1,%2,%3,%4,%5,%6\n")
                    .arg(wallClockTime)
                    .arg(sim->time())
                    .arg(optimized)
                    .arg(calcDeflection())
                    .arg(totalLength)
                    .arg(n_springs);
            file.write(mLine.toUtf8());
        }
    }
}

void Simulator::writeCustomMetric(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QIODevice::Append | QFile::Text)) {

        return;
    }

    if (!optConfig->rules.empty()) {
        if (optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE) {
            file.write(massDisplacer->customMetric.toUtf8());
        }
    }
}

void Simulator::writeSimDump(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {
        return;
    }

    // HEADER
    QString h = QString("SIMULATION DUMP; Time: %1, Springs: %2, Masses: %3\n")
            .arg(sim->time())
            .arg(sim->springs.size())
            .arg(sim->masses.size());
    file.write(h.toUtf8());

    // STATS
    QString s = QString("Start Length: %1, Start Energy: %2\n"
                        "Current Length: %3, Current Energy: %4\n")
            .arg(totalLength_start)
            .arg(totalEnergy_start)
            .arg(totalLength)
            .arg(totalEnergy);
    file.write(s.toUtf8());

    for (int m = 0; m < sim->masses.size(); m++) {
        QString mline = QString("Mass %1 (%2, %3, %4) (%5, %6, %7) %8 %9\n")
                .arg(m)
                .arg(sim->masses[m]->origpos[0])
                .arg(sim->masses[m]->origpos[1])
                .arg(sim->masses[m]->origpos[2])
                .arg(sim->masses[m]->extforce[0])
                .arg(sim->masses[m]->extforce[1])
                .arg(sim->masses[m]->extforce[2])
                .arg(sim->masses[m]->m)
                .arg(sim->masses[m]->constraints.fixed? "F": "");
        file.write(mline.toUtf8());
    }
    for (int s = 0; s < sim->springs.size(); s++) {
        QString sline = QString("Spring %1 %2 %3 %4 %5\n")
                .arg(s)
                .arg(sim->springs[s]->_left->index)
                .arg(sim->springs[s]->_right->index)
                .arg(sim->springs[s]->_k)
                .arg(sim->springs[s]->_diam);
        file.write(sline.toUtf8());
    }
}

void Simulator::printStatus() {
    sim_metrics metrics;
    getSimMetrics(metrics);

    //printf("\033[2J");
    //printf("\033[10;1f");
    printf("\033[J");
    cout << "\033[0K" << "\n\n========================================" << std::endl;
    cout << "\033[0K" << "\033[92m" << "SIMULATING" << "\033[97m" << std::endl;
    cout << "\033[0K" << "========================================\n" << std::endl;

    cout << "\033[0K" << "Wall Clock: " << wallClockTime << " s" << std::endl;
    cout << "\033[0K" << "Optimization Iterations: " << metrics.optimize_iterations << std::endl;
    cout << "\033[0K" << "Bars: " << metrics.nbars << std::endl;
    cout << "\033[0K" << "Time: " << setw(5) << std::left << std::setfill('0') << metrics.time << " s"  << std::endl;
    cout << "\033[0K" << "Weight: " << "\033[94m"  << std::setprecision(6) << metrics.totalLength_start << " (start), ";
    cout << "\033[95m" << metrics.totalLength << " (current), " << "\033[97m";
    cout << std::setprecision(4) << 100 * (metrics.totalLength / metrics.totalLength_start) << "%" << std::endl;

    cout << "\033[0K" << "Energy: " << "\033[94m" << metrics.totalEnergy_start << " (start), ";
    cout << "\033[95m" << metrics.totalEnergy << " (current), " << "\033[97m";
    cout << std::setprecision(4) << ((metrics.totalEnergy_start > 0.0)? (100 * (metrics.totalEnergy / metrics.totalEnergy_start)) : 100.0) << "%" << std::endl;
    cout << "\033[0K" << "Deflection: " << metrics.deflection << std::endl;
    cout << "\n";
}


// --------------------------------------------------------------------
// LOAD CONTROLS
// --------------------------------------------------------------------

void Simulator::clearLoads() {
    for (Mass *m : sim->masses) {
        m->extforce = Vec(0,0,0);
        m->extduration = 0;
        m->unfix();
    }
}

void Simulator::applyLoad(Loadcase *load) {
    sim->getAll();

    qDebug() << "Applying" << load->anchors[0]->masses.size() << "anchors and" << load->forces[0]->masses.size() << "forces";
    for (Mass *m : sim->masses) {
        for (Anchor *a : load->anchors) {
            for (Mass *am : a->masses) {
                if (m == am) {
                    m->fix();
                }
            }
        }
    }
    for (Force *f : load->forces) {
        int forceMasses = 0;

        for (Mass *fm : f->masses) {
            bool valid = false;
            for (Mass *m : sim->masses) {
                    if (m == fm) {
                    m->extduration = f->duration + pastLoadTime;
                    qDebug() << "DURATION" << m->extduration;
                    if (m->extduration < 0) {
                        m->extduration = DBL_MAX;
                    }
                    forceMasses ++;
                    valid = true;
                }
            }
            if (!valid) {
                f->masses.erase(remove(f->masses.begin(), f->masses.end(), fm), f->masses.end());
            }
        }
        if (forceMasses > 0) {
            Vec distributedForce = f->magnitude / forceMasses;
            for (Mass *fm : f->masses) {
                fm->extforce += distributedForce;
                fm->force += distributedForce;
            }
        }
    }
    sim->setAll();
}

void Simulator::varyLoadDirection() {

    Loadcase *l = nullptr;
    if (!config->loadQueue.empty()) {
        l = currentLoad > 0? config->loadQueue[currentLoad - 1] : config->loadQueue.back();
    } else if (config->load != nullptr) {
        l = config->load;
    }

    if (l != nullptr) {
        for (Mass *m : sim->masses) {
            m->extforce = Vec(0,0,0);
        }
        for (Force *f : l->forces) {

            qDebug() << f->vary[0] << f->vary[1] << f->vary[2];
            if (!(f->vary == Vec(0, 0, 0))) {
                double distributedMag = (f->magnitude / f->masses.size()).norm();
                Vec forceDir = f->magnitude.normalized();

                Vec dForceDir = Vec(Utils::randFloat(-f->vary[0],f->vary[0]),
                                Utils::randFloat(-f->vary[1],f->vary[1]),
                                Utils::randFloat(-f->vary[2],f->vary[2]));
                forceDir = (forceDir + dForceDir).normalized();
                qDebug() << "Varying Load" << forceDir[0] << forceDir[1] << forceDir[2];

                for (Mass *fm : f->masses) {
                    assert(fm != nullptr);

                    fm->extforce += distributedMag * forceDir;
                }
            }
        }
        sim->setAll();
    }
}
