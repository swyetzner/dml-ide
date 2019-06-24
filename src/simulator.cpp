#include "simulator.h"
#include <QDir>
#include <QProcess>
#include <QFileDialog>
#include <unistd.h>


Simulator::Simulator(Simulation *sim, SimulationConfig *config, OptimizationConfig *optConfig, QWidget *parent)
    : QOpenGLWidget(parent),
      massShaderProgram(nullptr),
      springShaderProgram(nullptr),
      planeShaderProgram(nullptr),
      textShaderProgram(nullptr),
      boundShaderProgram(nullptr),
      anchorShaderProgram(nullptr),
      forceShaderProgram(nullptr),
      frameBuffer(nullptr),
      m_frame(0),
      m_xRot(0),
      m_yRot(0),
      m_zRot(0),
      m_xPan(0), m_yPan(0), m_zPan(0),
      m_zoom(0.1f)
{
    this->sim = sim;
    this->config = config;
    this->optConfig = optConfig;
    this->optimizer = nullptr;

    n_masses = long(sim->masses.size());
    n_springs = long(sim->springs.size());
    n_planes = long(sim->planes.size());
    n_masses_start = long(sim->masses.size());
    n_springs_start = long(sim->springs.size());
    totalMass_start = sim->getTotalMass();
    totalEnergy = 0;
    totalLength = 0;
    totalLength_start = 0;
    totalEnergy_start = 0;
    double pi = atan(1.0)*4;
    for (Spring *s : sim->springs) {
        totalLength_start += s->_rest;
    }

    removalRate = 0.1;

    vertices = new GLfloat[3 * n_masses];
    colors = new GLfloat[2 * 8 * n_springs];
    planeVertices = new GLfloat[3 * 4];

    resizeBuffers = true;

    showVertices = false;
    showStress = false;
    showCrossSection = false;
    showOverlays = true;

    extForces = vector<Vec>();
    anchors = vector<Vec>();

    for (int i = 0; i < n_masses; i++) {
        Mass *m = sim->getMassByIndex(i);

        if (m->constraints.fixed) {
            anchors.push_back(m->pos);
        }

        if (m->force.norm() > 1E-6) {
            extForces.push_back(m->pos);
            extForces.push_back(m->force);
        }
    }

    renderTimeStep = 0.001;
    setRenderUpdate(renderTimeStep);

    springConstant = 10000;
    setSpringConst(springConstant);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Simulator::run);
    steps = 0;

    this->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
    this->setFocus();

    simStatus = ENDED;
    outputDir = "output";
    viewerWidth = (size() * devicePixelRatio()).width();
    viewerHeight = (size() * devicePixelRatio()).height();
    imageNumber = 0;
    framerate = 0.01; // 100 FPS

    dataDir = "data";
    createDataDir();

    bounds = getBoundingBox();
    double xDim = bounds[1][0] - bounds[0][0];
    double yDim = bounds[1][1] - bounds[0][1];
    double zDim = bounds[1][2] - bounds[0][2];

    for (Spring *s : sim->springs) {

        Mass *l = s->_left;
        Mass *r = s->_right;

        if (l->pos[0] < xDim/3 + bounds[0][0] || r->pos[0] < xDim/3 + bounds[0][0]) xLowSprings.springs.push_back(s);
        if (l->pos[0] >= xDim/3  + bounds[0][0] && l->pos[0] <= 2*xDim/3  + bounds[0][0] &&
                r->pos[0] >= xDim/3  + bounds[0][0] && r->pos[0] <= 2*xDim/3 + bounds[0][0])
            xMidSprings.springs.push_back(s);
        if (l->pos[0] > 2*xDim/3 + bounds[0][0] || r->pos[0] > 2*xDim/3 + bounds[0][0]) xHighSprings.springs.push_back(s);

        if (l->pos[1] < yDim/3 + bounds[0][1] || r->pos[1] < yDim/3 + bounds[0][1]) yLowSprings.springs.push_back(s);
        if (l->pos[1] >= yDim/3  + bounds[0][1] && l->pos[1] <= 2*yDim/3  + bounds[0][1] &&
                r->pos[1] >= yDim/3  + bounds[0][1] && r->pos[1] <= 2*yDim/3 + bounds[0][1])
            yMidSprings.springs.push_back(s);
        if (l->pos[1] > 2*yDim/3 + bounds[0][1] || r->pos[1] > 2*yDim/3 + bounds[0][1]) yHighSprings.springs.push_back(s);

        if (l->pos[2] < zDim/3 + bounds[0][2] || r->pos[2] < zDim/3 + bounds[0][2]) zLowSprings.springs.push_back(s);
        if (l->pos[2] >= zDim/3  + bounds[0][2] && l->pos[2] <= 2*zDim/3  + bounds[0][2] &&
                r->pos[2] >= zDim/3  + bounds[0][2] && r->pos[2] <= 2*zDim/3 + bounds[0][2])
            zMidSprings.springs.push_back(s);
        if (l->pos[2] > 2*zDim/3 + bounds[0][2] || r->pos[2] > 2*zDim/3 + bounds[0][2]) zHighSprings.springs.push_back(s);
    }

    qDebug() << "x group" << xLowSprings.springs.size() << xMidSprings.springs.size() << xHighSprings.springs.size();
    qDebug() << "y group" << yLowSprings.springs.size() << yMidSprings.springs.size() << yHighSprings.springs.size();
    qDebug() << "z group" << zLowSprings.springs.size() << zMidSprings.springs.size() << zHighSprings.springs.size();

    n_repeats = 0;
    repeatTime = config->repeat.after;
    explicitRotation = config->repeat.rotationExplicit;
    repeatRotation = config->repeat.rotation;
    optimizeAfter = (repeatTime > 0)? 20 : 0;

    double minUnitDist = DBL_MAX;
    for (Spring *t : sim->springs) {
        minUnitDist = fmin(minUnitDist, t->_rest);
    }
    qDebug() << "Min unit distance" << minUnitDist;

    // TODO: add stop criteria
    if (optConfig != nullptr) {
        for (OptimizationRule r : optConfig->rules) {
            switch(r.method) {
                case OptimizationRule::REMOVE_LOW_STRESS:
                    this->optimizer = new SpringRemover(sim, r.threshold);
                    qDebug() << "Created SpringRemover" << r.threshold;
                    break;

                case OptimizationRule::MASS_DISPLACE: {
                    massDisplacer = new MassDisplacer(sim, config->lattice.unit[0] * 0.2, r.threshold);
                    massDisplacer->maxLocalization = minUnitDist + 1E-4;
                    massDisplacer->order = 0;
                    massDisplacer->chunkSize = 0;
                    massDisplacer->relaxation = 2000;
                    this->optimizer = massDisplacer;
                    qDebug() << "Created MassDisplacer" << r.threshold;
                    break;
                }

                case OptimizationRule::NONE:
                    optimizer = nullptr;
                    break;
            }
        }
    }

    originalModel = config->model;
    //optimizer = new MassDisplacer(sim, 0.2);
    //springInserter = new SpringInserter(sim, 0.001);
    //springInserter->cutoff = 3.5 * config->lattice.unit[0];

    equilibrium = false;
    optimized = 0;
    closeToPrevious = 0;
    prevEnergy = -1;
    prevSteps = 0;
}

// --------------------------------------------------------------------
// SIMULATION CONTROLS
// --------------------------------------------------------------------


void Simulator::start() {
    renderTimeStep = getRenderUpdate();

    //springConstant = getSpringConst();
    //sim->setAllSpringConstantValues(springConstant);
    qDebug() << sim->masses.front()->dt << "b";
    sim->setAllDeltaTValues(getTimestep());
    qDebug() << sim->masses.front()->dt << "p";

    sim->step(renderTimeStep);
    steps++;
    timer->start(int(renderTimeStep * 100));
}

void Simulator::run() {
    if (getShowStress() != showStress) {
        showStress = getShowStress();
        updateColors();
    }

    if (!sim->running()) {

        // Set repeats
        if (repeatTime > 0 && repeatTime < sim->time()) {
            repeat();
        }

        if (steps % int((framerate / renderTimeStep)) == 0) {
            if (simStatus == RECORDING) {
                QString outputFile = QString(QDir::currentPath() + QDir::separator() +
                                             outputDir + QDir::separator() +
                                             "dmlFrame_" + QString::number(imageNumber) + ".png");

                saveImage(grabFramebuffer(), outputFile);
                imageNumber++;
            }
        }

        qDebug() << "About to step" << sim;
        sim->step(renderTimeStep);
        qDebug() << "Stepped" << steps << "Repeats" << n_repeats;
        sim->getAll();
        qDebug() << "Synced to CPU";
        totalLength = 0;
        for (Spring *s: sim->springs) {
            totalLength += s->_rest;
        }
        totalEnergy = 0;


        bool equilibriumMetric = optConfig != nullptr && optConfig->rules.front().method == OptimizationRule::MASS_DISPLACE;
        if (equilibriumMetric) {
            for (Spring *s : sim->springs) {
                totalEnergy += s->_curr_force * s->_curr_force / s->_k;
            }
            qDebug() << "ENERGY" << totalEnergy << prevEnergy << closeToPrevious;
            if (prevEnergy > 0 && fabs(prevEnergy - totalEnergy) < totalEnergy * 1E-6) {
                closeToPrevious++;
            } else {
                closeToPrevious = 0;
            }
            if (closeToPrevious > 10) {
                equilibrium = true;
                if (!optimized) {
                    totalEnergy_start = totalEnergy;
                    writeMetricHeader(metricFile);
                    writeCustomMetricHeader(customMetricFile);
                }
            }

            bool stopReached = false;
            for (auto s : optConfig->stopCriteria) {
                switch (s.metric) {
                    case OptimizationStop::ENERGY:
                        stopReached = totalEnergy / totalEnergy_start <= s.threshold;
                    break;
                    case OptimizationStop::WEIGHT:
                        stopReached = totalLength / totalLength_start <= s.threshold;
                    break;
                }
            }

            if (optimizeAfter <= n_repeats && equilibrium && !stopReached) {

                if (optimizer != nullptr) {
                    qDebug() << "About to optimize";
                    optimizer->optimize();
                    equilibrium = false;
                    closeToPrevious = 0;

                    writeMetric(metricFile);
                    if (optimized == 0)
                        writeCustomMetric(customMetricFile);
                    optimized++;
                }
                //if (steps > 1 && steps % 500 == 0) {
                //    springInserter->optimize();
                //}
                if (n_masses != int(sim->masses.size())) resizeBuffers = true;
                if (n_springs != int(sim->springs.size())) resizeBuffers = true;
                n_masses = int(sim->masses.size());
                n_springs = int(sim->springs.size());
                updateColors();
                updateDiameters();
            }
            prevEnergy = totalEnergy;
        } else {
            if (optConfig != nullptr) {

                bool stopReached = false;
                for (auto s : optConfig->stopCriteria) {
                    switch (s.metric) {
                        case OptimizationStop::ENERGY:
                            stopReached = totalEnergy / totalEnergy_start <= s.threshold;
                            break;
                        case OptimizationStop::WEIGHT:
                            qDebug() << "Stop Reached" << totalLength_start << 100 * totalLength / totalLength_start << s.threshold;
                            stopReached = totalLength / totalLength_start <= s.threshold;
                            break;
                    }
                }

                for (OptimizationRule r : optConfig->rules) {
                    if (optimizeAfter <= n_repeats && prevSteps >= r.frequency && !stopReached) {

                        optimizer->optimize();
                        optimized++;

                        if (n_masses != int(sim->masses.size())) resizeBuffers = true;
                        if (n_springs != int(sim->springs.size())) resizeBuffers = true;
                        n_masses = int(sim->masses.size());
                        n_springs = int(sim->springs.size());
                        prevSteps = 0;

                        resizeBuffers = true;
                        updateColors();
                        updateDiameters();
                    }
                }
            }
        }

        /**if (steps > 100 && (steps + 50) % 100 == 0) {
            optimizer->optimize();
            updateColors();
            updateDiameters();
        }
        if (steps > 1 && steps % 100 == 0) {
            springInserter->optimize();
            updateColors();
            updateDiameters();
        }**/

        updatePairVertices();
        updateOverlays();
        qDebug() << "Updated vertices";
        //if (resizeBuffers) {
        //    updateBuffers();
        //}
        qDebug() << "Updated buffers";
        //drawVertexArray();
        qDebug() << "drawn";
        update();
        timeChange(sim->time());

        if (showStress) {
            updateColors();
        }

        steps += long(renderTimeStep / sim->masses.front()->dt);
        prevSteps += long(renderTimeStep / sim->masses.front()->dt);
        qDebug() << steps;
    }
}

void Simulator::step() {
    //springConstant = getSpringConst();
    if (getShowStress() != showStress) {
        showStress = getShowStress();
        updateColors();
    }

    if (!sim->running() && sim->masses.size() > 0) {
        if (steps > 100) {
            //for (int i =0; i < 1000; i++)
                //removePercentBars(0.2);
        }

       // sim->setAllSpringConstantValues(springConstant);
        sim->step(getTimestep());
        sim->getAll();

        //updateVertices();
        //updateIndices();
        updatePairVertices();
        updateOverlays();
        updateColors();
        update();
        timeChange(sim->time());
        steps++;
    }
}

void Simulator::pause() {
    timer->stop();
}

void Simulator::stop() {
    timer->stop();
    reloadSimulation();
}

void Simulator::repeat() {

    if (!sim->running()) {

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
            p -= center;

            QMatrix4x4 rot;
            rot.setToIdentity();
            rot.rotate(rotation[0] * 360, 1.0f, 0.0f, 0.0f);
            rot.rotate(rotation[1] * 360, 0.0f, 1.0f, 0.0f);
            rot.rotate(rotation[2] * 360, 0.0f, 0.0f, 1.0f);
            p = rot * p;

            p += center;
            m->pos = Vec(p.x(), p.y(), p.z());
            m->vel = Vec(0, 0, 0);
            m->acc = Vec(0, 0, 0);
        }

        // Increase repeat time
        repeatTime += config->repeat.after;

        sim->setAll();
        n_repeats++;
    }
}

void Simulator::createDataDir() {
    QString currentPath = QDir::currentPath();
    qDebug() << currentPath;
    QDir data(currentPath + QDir::separator() + dataDir);
    if (!data.exists()) {
        qDebug() << "Data folder does not exist. Creating...";
        QDir::current().mkdir(dataDir);
    } else {
        data.removeRecursively();
        qDebug() << QDir::current().path();
        QDir::current().mkdir(dataDir);
    }

    // Create metric file
    metricFile = QString(QDir::currentPath() + QDir::separator() +
                        dataDir + QDir::separator() +
                        "optMetrics.csv");
    customMetricFile = QString(QDir::currentPath() + QDir::separator() +
                               dataDir + QDir::separator() +
                               "outsideForces.csv");
}

void Simulator::writeMetricHeader(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {

        log(tr("Cannot read file %1:\n%2.")
                    .arg(QDir::toNativeSeparators(outputFile),
                         file.errorString()));
        return;
    }

    if (optConfig->stopCriteria.front().metric == OptimizationStop::ENERGY) {
        file.write("Time,Iteration,Displacement,Attempts,Total Energy,Total Weight\n");
    } else {
        file.write("Time,Iteration,Total Weight,Bar Number\n");
    }

    // Write starting line
    writeMetric(outputFile);
}

void Simulator::writeCustomMetricHeader(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QFile::Text)) {

        log(tr("Cannot read file %1:\n%2.")
                    .arg(QDir::toNativeSeparators(outputFile),
                         file.errorString()));
        return;
    }

    if (optConfig->stopCriteria.front().metric == OptimizationStop::ENERGY) {
        file.write(massDisplacer->customMetricHeader.toUtf8());
    }
}

void Simulator::writeMetric(const QString &outputFile) {

    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QIODevice::Append | QFile::Text)) {

        log(tr("Cannot read file %1:\n%2.")
                    .arg(QDir::toNativeSeparators(outputFile),
                         file.errorString()));
        return;
    }

    if (optConfig->stopCriteria.front().metric == OptimizationStop::ENERGY) {
        QString mLine = QString("%1,%2,%3,%4,%5,%6\n")
                .arg(sim->time())
                .arg(optimized)
                .arg(massDisplacer->dx)
                .arg(massDisplacer->attempts)
                .arg(totalEnergy)
                .arg(totalLength);
        file.write(mLine.toUtf8());
    }
}

void Simulator::writeCustomMetric(const QString &outputFile) {
    QFile file(outputFile);
    if (!file.open(QFile::WriteOnly | QIODevice::Append | QFile::Text)) {

        log(tr("Cannot read file %1:\n%2.")
                    .arg(QDir::toNativeSeparators(outputFile),
                         file.errorString()));
        return;
    }

    if (optConfig->stopCriteria.front().metric == OptimizationStop::ENERGY) {
        file.write(massDisplacer->customMetric.toUtf8());
    }
}

void Simulator::recordVideo() {
    simStatus = RECORDING;
    imageNumber = 0;
    QString currentPath = QDir::currentPath();
    qDebug() << currentPath;
    QDir output(currentPath + QDir::separator() + outputDir);
    if (!output.exists()) {
        log("Output folder does not exist. Creating...");
        QDir::current().mkdir(outputDir);
    } else {
        output.removeRecursively();
        qDebug() << QDir::current().path();
        QDir::current().mkdir(outputDir);
    }

    // Record initial frame
    QString outputFile = QString(QDir::currentPath() + QDir::separator() +
                                 outputDir + QDir::separator() +
                                 "dmlFrame_" + QString::number(imageNumber) + ".png");

    saveImage(grabFramebuffer(), outputFile);
    imageNumber++;
}

void Simulator::saveVideo() {
    if (simStatus == RECORDING) {
        pause();
        simStatus = STARTED;
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save simulation video"), QDir::currentPath(),
                                                        tr("Video Files (*.mp4 *.mpeg4 *.avi);;All files (*)"));
        if (fileName.isEmpty())
            return;

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            log("Unable to open file: " + file.errorString());
            return;
        }

        QString outputPrefix = QDir::currentPath() + QDir::separator() + outputDir + QDir::separator();
        QString fps = QString::number(int(1/framerate));
        qDebug() << "Outputting video:" <<  fileName << " FPS" << fps;
        QString createVideoProgram = "ffmpeg -framerate " + fps + " -y -i " + outputPrefix +
                "dmlFrame_%d.png -vf \"eq=saturation=2.0, pad=ceil(iw/2)*2:ceil(ih/2)*2\" -vcodec libx264 -crf 25 -pix_fmt yuv420p ";

        createVideoProgram += fileName;
        QProcess *process = new QProcess(this);
        process->start(createVideoProgram);
    }
}

void Simulator::prepare() {

    int hangingSprings = 0;
    int totalCorrected = 0;

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
            }
        }

        qDebug() << "Corrected" << hangingSprings << "hanging springs";
        if (hangingSprings == 0) hangingSprings = -1;

        totalCorrected += hangingSprings;
    }
    log(tr("Corrected %1 hanging springs.").arg(totalCorrected));
}


QString Simulator::calcAvgStresses(test_group t) {
    t.avgXForce = 0;
    t.avgYForce = 0;
    t.avgZForce = 0;
    for (Spring *p : t.springs) {
                t.avgXForce += p->_max_stress;
                t.avgYForce += p->getForce()[1];
                t.avgZForce += p->getForce()[2];
    }
    t.avgXForce /= t.springs.size();
    t.avgYForce /= t.springs.size();
    t.avgZForce /= t.springs.size();
    char buffer [50];
    sprintf(buffer, "Average Force: %lf", t.avgXForce);
    return QString::fromLatin1(buffer);
}

void Simulator::iterateMassStructure(double ratio) {
    if (sim->masses.size() > n_masses_start * ratio) {

        sim->getAll();

        double minForce = DBL_MAX;
        int f = -1;

        // Find mass with the minimum force
        for (uint i = 0; i < sim->masses.size(); i++) {
            Vec force = sim->masses.at(i)->m * sim->masses.at(i)->acc;
            if (force.norm() < minForce) {
                minForce = force.norm();
                f = int(i);
            }
        }

        // Delete springs attached to mass
        for (uint j = 0; j < sim->springs.size(); j++) {
            Spring *s = sim->springs.at(j);
            if (s->getLeft() == f || s->getRight() == f) {
                sim->deleteSpring(s);
                //sim->springs.erase(sim->springs.begin() + j);
                n_springs--;
            }
        }

        // Delete mass
        //sim->masses.erase(sim->masses.begin() + f);
        sim->deleteMass(sim->masses.at(uint(f)));
        n_masses--;


        //for (Spring *s : sim->springs) {
        //    s->_left->index = sim->getMassByIndex(s->getLeft())->index;
        //    s->_right->index = sim->getMassByIndex(s->getRight())->index;
        //}

        //sim->setAll();

    }
}

void Simulator::removePercentBars(double ratio) {
    showVertices = false;
    if (n_springs > n_springs_start * ratio && steps % 10 == 0) {
        sim->getAll();

        //sim->rebalanceMasses(0.1);



        qDebug() << "Iterating";
        vector<uint> springIndicesToSort = vector<uint>();
        vector<double> springStress = vector<double>();

        for (uint s = 0; s < sim->springs.size(); s++) {

            bool underExternalForce = sim->springs[s]->_left->extforce.norm() > 1E-6
                    && sim->springs[s]->_right->extforce.norm() > 1E-6;
            bool fixed = sim->springs[s]->_left->constraints.fixed && sim->springs[s]->_right->constraints.fixed;

            if (!underExternalForce && !fixed) {
                springIndicesToSort.push_back(s);
            }

            double force = sim->springs[s]->_max_stress;
            /**double newl = (sim->springs[s]->_left->pos - sim->springs[s]->_right->pos).norm();
            double l = newl - sim->springs[s]->_rest;
            l /= sim->springs[s]->_rest;
            double d = -0.5 * springDiams[sim->springs[s]] * l;
            double stress = force / ((d+springDiams[sim->springs[s]]) * newl);**/

            springStress.push_back(force);
        }


        qDebug() << "Filled sorting array" << springIndicesToSort.size() << springStress.size();

        sort(springIndicesToSort.begin(), springIndicesToSort.end(),
             [springStress](uint s1, uint s2) -> bool {
                 return springStress[s1] < springStress[s2];
             });
        qDebug() << "Sorted springs by stress";


        uint toRemove = uint(ratio * sim->springs.size());
        map<Spring *, bool> springsToDelete = map<Spring *, bool>();
        for (Spring *s : sim->springs) {
            springsToDelete[s] = false;
        }
        for (uint j = 0; j < toRemove; j++) {
            if (j < springIndicesToSort.size())
                springsToDelete[sim->springs[springIndicesToSort[j]]] = true;
        }
        qDebug() << "Removing" << toRemove << "Springs";

        /**structureMaxStress = 0;
         double minForce = DBL_MAX;
        uint i_minForce = 0;
        uint i_maxForce = 0;
        for (uint i = 0; i < sim->springs.size(); i++) {
            double force = sim->springs[i]->_max_stress;
            double newl = (sim->springs[i]->_left->pos - sim->springs[i]->_right->pos).norm();
            double l = newl - sim->springs[i]->_rest;
            l /= sim->springs[i]->_rest;
            double d = -0.5 * springDiams[sim->springs[i]] * l;
            double stress = force / ((d+springDiams[sim->springs[i]]) * newl);


            if (steps < retainStresses) {
                leakySpringStresses[i].push_back(stress);
                maxLeakyStresses[i] = std::max(maxLeakyStresses[i], stress);
            } else {
                double oldStress = leakySpringStresses[i].front();
                leakySpringStresses[i].erase(leakySpringStresses[i].begin());
                leakySpringStresses[i].push_back(stress);
                if (fabs(maxLeakyStresses[i] - oldStress) < 1E-6) {
                    // Recalculate max
                    maxLeakyStresses[i] = 0;
                    for (double st : leakySpringStresses[i]) {
                        maxLeakyStresses[i] = std::max(maxLeakyStresses[i], st);
                    }
                }
            }
            //sim->springs[i]->_max_stress = 0;
        }

        totalMaxStress = 0;
        for (uint i = 0; i < sim->springs.size(); i++) {
            bool underExternalForce = sim->springs[i]->_left->extforce.norm() > 1E-6
                    || sim->springs[i]->_right->extforce.norm() > 1E-6;

            if (maxLeakyStresses[i] < minForce
                && (!sim->springs[i]->_left->constraints.fixed
                || !sim->springs[i]->_right->constraints.fixed)
                && !underExternalForce) {
                minForce = maxLeakyStresses[i];
                i_minForce = i;
            }

            totalMaxStress = std::max(totalMaxStress, maxLeakyStresses[i]);
        }

        qDebug() << "Min force" << minForce << "index" << i_minForce;

        Mass *m1 = sim->springs[i_minForce]->_left;
        Mass *m2 = sim->springs[i_minForce]->_right;
        Spring *s0 = sim->springs[i_minForce];

        // Remove hanging masses (less than 3 attached springs)
        // This makes hanging masses into lone masses for later removal
        map<Spring *, bool> springsToDelete = map<Spring *, bool>();
        for (Spring *s : sim->springs) {
            springsToDelete[s] = false;
            if (s->_left->ref_count < 4) {
                springsToDelete[s] = true;
            }
            if (s->_right->ref_count < 4) {
                springsToDelete[s] = true;
            }
        }

        qDebug() << "mass weights" << m1->m << m2->m << "ref counts" << m1->ref_count << m2->ref_count;
        // Remove lone masses
        if (m1->ref_count < 3) sim->deleteMass(m1);
        if (m2->ref_count < 3) sim->deleteMass(m2);**/

        /**for (int i = 0; i < n_masses; i++) {
            if (sim->masses[i] == m1 && m1->ref_count < 10)
                *(colors + i*4) = 1.0f;
            if (sim->masses[i] == m2 && m2->ref_count < 10)
                *(colors + i*4) = 1.0f;
        }**/

        // Remove hanging springs (attached to masses with only one attached spring)
        for (Spring *s : sim->springs) {
            if (s->_left->ref_count < 3 || s->_right->ref_count < 3) {
                springsToDelete[s] = true;
            }
            // For 2 attached springs, determine angle between them
            if (s->_left->ref_count < 5) {
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
            if (s->_right->ref_count < 5) {
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
        //springsToDelete[s0] = true;
        //if (steps > retainStresses) {
            uint i = 0;
            while(i < sim->springs.size()) {
                if (sim->springs[i] != nullptr && springsToDelete[sim->springs[i]]) {
                    sim->deleteSpring(sim->springs[i]);
                    //maxLeakyStresses.erase(maxLeakyStresses.begin() + i);
                    //leakySpringStresses.erase(leakySpringStresses.begin() + i);
                    i--;
                }
                i++;
            }

            sim->setAll(); // Set spring stresses and mass value updates on GPU
            updateColors();
        //}


        // Find place to add springs
        /**if (steps > 1000) {
            m1 = sim->springs[i_maxForce]->_left;
            m2 = sim->springs[i_maxForce]->_right;

            // Collect surrounding masses
            vector<Mass *> leftCandidates = vector<Mass *>();
            vector<Mass *> rightCanditates = vector<Mass *>();
            for (Spring *s : sim->springs) {
                if (s != sim->springs[i_maxForce]) {
                    if (s->_left == m1) leftCandidates.push_back(s->_right);
                    if (s->_right == m1) leftCandidates.push_back(s->_left);
                    if (s->_left == m2) rightCanditates.push_back(s->_right);
                    if (s->_right == m2) rightCanditates.push_back(s->_left);
                }
            }

            qDebug() << "Testing" << leftCandidates.size() << "left" << rightCanditates.size() << "right";

            // Attempt to add a connection
            bool madeConnection = false;
            for (Mass *l : leftCandidates) {
                for (Mass *r : rightCanditates) {
                    if (l != r) {
                        sim->createSpring(l, r);
                        madeConnection = true;  break;
                    }
                }
                if (madeConnection) break;
            }
            updateColors();
        }**/

        n_springs = int(sim->springs.size());
        n_masses = int(sim->masses.size());

        qDebug() << "Masses" << n_masses <<  "Springs" << n_springs;
        qDebug() << "Percent masses left" << 100 * n_masses / n_masses_start << "Percent springs left" << 100 * n_springs / n_springs_start;
    }
}

// Optimizer that adjusts bar diameters based on stress
void Simulator::adjustBarDiams() {

    double maxStress = 0;
    map<Spring *, bool> springsToDelete = map<Spring *, bool>();

    for (Spring * s : sim->springs) {
        maxStress = std::max(maxStress, s->_max_stress);
        springsToDelete[s] = false;
    }

    for (Spring * s : sim->springs) {
        double ratio = s->_max_stress / maxStress;
        s->_diam *= ratio * 2;

        if (s->_diam < 1E-6) {
            springsToDelete[s] = true;
        }
    }

    // Delete springs
    uint i = 0;
    while(i < sim->springs.size()) {
        if (sim->springs[i] != nullptr && springsToDelete[sim->springs[i]]) {
            sim->deleteSpring(sim->springs[i]);
            i--;
        }
        i++;
    }

    sim->setAll(); // Set spring stresses and mass value updates on GPU
    updateColors();
}


// Optimizer that finds unconnected masses near stressed springs
// and adds new springs between them
void Simulator::addSecondOrderBars(double ratio) {

    uint toAdd = uint(ratio * sim->springs.size());

    vector<uint> springIndicesToSort = vector<uint>();
    vector<double> springStress = vector<double>();

    for (uint s = 0; s < sim->springs.size(); s++) {

        bool underExternalForce = sim->springs[s]->_left->extforce.norm() > 1E-6
                                  && sim->springs[s]->_right->extforce.norm() > 1E-6;
        bool fixed = sim->springs[s]->_left->constraints.fixed && sim->springs[s]->_right->constraints.fixed;

        if (!underExternalForce && !fixed) {
            springIndicesToSort.push_back(s);
        }

        double force = sim->springs[s]->_max_stress;
        springStress.push_back(force);
    }


    qDebug() << "Filled sorting array" << springIndicesToSort.size() << springStress.size();
    sort(springIndicesToSort.begin(), springIndicesToSort.end(),
         [springStress](uint s1, uint s2) -> bool {
             return springStress[s1] > springStress[s2];
         });
    qDebug() << "Sorted springs by stress (reverse)";



    for (uint j = 0; j < toAdd; j++) {
        if (j < springIndicesToSort.size()) {

            Spring *s = sim->springs[springIndicesToSort[j]];
            Mass *m1 = s->_left;
            Mass *m2 = s->_right;

            // Start with a dumb solution
            for (Spring *t : sim->springs) {
                Mass *secondDegree = nullptr;
                if (t != s) {
                    if (t->_left == m1 || t->_left == m2) {
                        secondDegree = t->_right;
                    }
                    if (t->_right == m1 || t->_right == m2) {
                        secondDegree = t->_left;
                    }
                }
                if (secondDegree != nullptr) {

                    sim->createSpring(m1, secondDegree);
                }
            }
        }
    }
}

void Simulator::moveStressedMasses(double ratio) {

    uint toMove = uint(ratio * sim->masses.size());

    vector<uint> massIndicesToSort = vector<uint>();
    vector<double> massStresses = vector<double>(sim->masses.size());

    for (uint m = 0; m < sim->masses.size(); m++) {
        bool underExternalForce = sim->masses[m]->extforce.norm() > 1E-6;
        bool fixed = sim->masses[m]->constraints.fixed;

        if (!underExternalForce && !fixed) {
            massIndicesToSort.push_back(m);
        }

        massStresses[m] = sim->masses[m]->maxforce.norm();
    }

    qDebug() << "Filled sorting array" << massIndicesToSort.size() << massIndicesToSort.size();
    sort(massIndicesToSort.begin(), massIndicesToSort.end(),
         [massStresses](uint m1, uint m2) -> bool {
             return massStresses[m1] < massStresses[m2];
         });
    qDebug() << "Sorted masses by stress";

    for (uint n = 0; n < toMove; n++) {
        if (n < massIndicesToSort.size()) {

            Mass *m = sim->masses[massIndicesToSort[n]];

            // Get movement direction
            Vec dir = m->maxforce.normalized();
            Vec pdiff = 0.001 * dir;

            // Move mass
            for (Spring *s : sim->springs) {
                if (s->_left == m) {
                    s->_rest = (s->_right->origpos - (s->_left->origpos + pdiff)).norm();
                }
                if (s->_right == m) {
                    s->_rest = (s->_left->origpos - (s->_right->origpos + pdiff)).norm();
                }
            }
            qDebug() << "Moving" << n << m->pos[0] << m->pos[1] << m->pos[2] << " " << pdiff[0] << pdiff[1] << pdiff[2];
            m->pos += pdiff;
            m->origpos += pdiff;
            qDebug() << m->pos[0] << m->pos[1] << m->pos[2];
        }
    }

    sim->setAll();
}


// --------------------------------------------------------------------
// OPENGL SHADERS
// --------------------------------------------------------------------
static const char *vertexShaderSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 1) in vec4 colorPos;\n"
        "out vec4 vertexColor;\n"
        "out vec4 position;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = colorPos;\n"
        "   position = vec4(vertexPos.xyz, 0.0);\n"
        "}\n";

static const char *clipVertexShaderSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 1) in vec4 colorPos;\n"
        "out vec4 vertexColor;\n"
        "out vec4 position;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "uniform vec4 center;\n"
        "uniform vec4 clipPlaneX;\n"
        "uniform vec4 clipPlaneY;\n"
        "uniform vec4 clipPlaneZ;\n"
        ""
        "out gl_PerVertex\n"
        "{\n"
        "  vec4 gl_Position;\n"
        "  float gl_ClipDistance[3];\n"
        "};\n"
        "void main() {\n"
        "   gl_ClipDistance[0] = dot(position, vec4(clipPlaneX.xyz, center.x));\n"
        "   gl_ClipDistance[1] = dot(position, vec4(clipPlaneY.xyz, center.y));\n"
        "   gl_ClipDistance[2] = dot(position, vec4(clipPlaneY.xyz, center.z));\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = colorPos;\n"
        "   position = vec4(vertexPos.xyz, 0.0);\n"
        "}\n";

static const char *fragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";

static const char *clipFragmentShaderSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "in vec4 position;\n"
        "uniform vec4 center;\n"
        "uniform vec4 clipPlaneX;\n"
        "uniform vec4 clipPlaneY;\n"
        "uniform vec4 clipPlaneZ;\n"
        "void main() {\n"
        "   vec4 d = position - center;\n"
        "   if ((clipPlaneX.x * d.x + clipPlaneX.y * d.y + clipPlaneX.z * d.z) > 0.0f) discard;\n"
        "   if ((clipPlaneY.x * d.x + clipPlaneY.y * d.y + clipPlaneY.z * d.z) > 0.0f) discard;\n"
        "   if ((clipPlaneZ.x * d.x + clipPlaneZ.y * d.y + clipPlaneZ.z * d.z) > 0.0f) discard;\n"
        "   fragColor = vertexColor;\n"
        "}\n";

static const char *vertexShaderSpringSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "layout (location = 2) in vec4 colorPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = colorPos;\n"
        "}\n";

static const char *fragmentShaderSpringSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";

static const char *vertexShaderPlaneSource =
        "#version 330\n"
        "layout (location = 3) in vec3 vertexPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vec4(0.2, 0.2, 0.2, 1.0);\n"
        "}\n";

static const char *fragmentShaderPlaneSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";

static const char *vertexShaderBoundSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "out vec4 vertexColor;\n"
        //"out float gl_ClipDistance[1]\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        //"uniform vec4 clipPlane;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vec4(0.3, 0.6, 0.0, 0.2);\n"
        //"   gl_ClipDistance[0] = dot(mvMatrix * vec4(vertexPos, 1.0, clipPlane);\n"
        "}\n";

static const char *fragmentShaderBoundSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";

static const char *vertexShaderAnchorSource =
        "#version 330\n"
        "layout (location = 0) in vec3 vertexPos;\n"
        "out vec4 vertexColor;\n"
        "uniform mat4 projMatrix;\n"
        "uniform mat4 mvMatrix;\n"
        "void main() {\n"
        "   gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);\n"
        "   vertexColor = vec4(0.4, 0.3, 0.3, 1.0);\n"
        "}\n";

static const char *fragmentShaderAnchorSource =
        "#version 330\n"
        "out vec4 fragColor;\n"
        "in vec4 vertexColor;\n"
        "void main() {\n"
        "   fragColor = vertexColor;\n"
        "}\n";


//  ----- Color constants
static const GLfloat defaultMassColor[] = { 0.2f, 5.0f, 0.0f, 1.0f };
static const GLfloat fixedMassColor[] = { 1.0f, 0.0f, 0.0f, 1.0f };
static const GLfloat forceMassColor[] = { 0.0f, 0.0f, 1.0f, 1.0f };
static const GLfloat defaultSpringColor[] = { 0.5, 1.0, 0.0, 0.1 };
static const GLfloat expandSpringColor[] = { 0.5, 1.0, 0.0, 1.0 };
static const GLfloat contractSpringColor[] = { 0.7, 0.0, 1.0, 1.0 };
static const GLfloat brokenSpringColor[] = { 1.0f, 0.0f, 0.0f, 1.0f };


// --------------------------------------------------------------------
// OPENGL FUNCTIONS
// --------------------------------------------------------------------


//  ----- updateVertices()
//  Pushes simulation mass positions to vertices array
//
void Simulator::updateVertices() {
    int verticesCount = 0;
    n_masses = long(sim->masses.size());
    vertices = new GLfloat[3 * n_masses];

    //glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_id);
    //sim->exportMassVertices(vertexBuff_id);

    for (int i = 0; i < n_masses; i++) {
        GLfloat *p = vertices + verticesCount;
        Mass *m = sim->getMassByIndex(i);

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);
        verticesCount += 3;
    }
}

//  ----- updateIndices()
//  Pushes simulation spring connections to indices array
//
void Simulator::updateIndices() {

    glBindBuffer(GL_ARRAY_BUFFER, indexBuff_id);
    sim->exportSpringIndices(indexBuff_id);

    //int indicesCount = 0;
    //int indicesFound = 0;
    //n_springs = long(sim->springs.size());
    //n_masses = long(sim->masses.size());
    //indices = new GLuint[2 * n_springs];

        /***p++ = GLuint(s->getLeft());
        *p++ = GLuint(s->getRight());
        indicesCount += 2;

        if (s->getLeft() >= n_masses || s->getRight() >= n_masses)
            qDebug() << "INDEX ERROR" << s->_left->index << s->_right->index << n_masses;
    }**/
}


//  ----- updatePairVertices()
//  Pushes simulation spring connections to vertices array
//
void Simulator::updatePairVertices() {

    //glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    //sim->exportSpringVertices(pairVertexBuff_id);

    //n_masses = long(sim->masses.size());
    //n_springs = long(sim->springs.size());

    if (resizeBuffers) {
        delete pairVertices;
        pairVertices = new GLfloat[2 * 3 * n_springs];
    }
    //vector<Vec> vertices = vector<Vec>();
    verticesCount = 0;

    double d;
    //Vec cpX = Vec(clipPlaneX.x(), clipPlaneX.y(), clipPlaneX.z());
    //Vec cen = Vec(center.x(), center.y(), center.z());
    //Utils::getPlaneEquation(cpX, cen, d);

    /**for (int i = 0; i < n_springs; i++) {
        Spring *s = sim->getSpringByIndex(i);

        Mass *m = s->_left;
        Vec pt1 = m->pos;
        m = s->_right;
        Vec pt2 = m->pos;

        //if ((pt1[0] * cpX[0] + pt1[1] * cpX[1] + pt1[2] * cpX[2] + d) < 0.0
        //    && (pt2[0] * cpX[0] + pt2[1] * cpX[1] + pt2[2] * cpX[2] + d) < 0.0) {
            vertices.push_back(pt1);
            vertices.push_back(pt2);
        //}
    }**/

    for (int i = 0; i < n_springs; i++) {
        GLfloat *p = pairVertices + verticesCount;
        Spring *s = sim->getSpringByIndex(i);

        Mass *m = s->_left;

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);

        m = s->_right;

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);

        verticesCount += 6;
    }
}

//  ----- updateOverlays()
//  Pushes bounds, anchor vertices, and force vertices
//  to vertex array
//
void Simulator::updateOverlays() {

    // FORCE VERTICES
    //n_masses = long(sim->masses.size());
    extForces = vector<Vec>();

    for (int i = 0; i < n_masses; i++) {
        Mass *m = sim->getMassByIndex(i);

        if (m->force.norm() > 1E-6) {
            extForces.push_back(m->pos);

            Vec av = m->force.normalized();
            /**if (originalModel->isInside(vec3(ev[0], ev[1], ev[2]), 0)) {
                av = -av;
            }**/
            extForces.push_back(av);
        }
    }

    if (resizeBuffers) {
        delete forceVertices;
        forceVertices = new GLfloat[3 * extForces.size()];
    }
    int forceCount = 0;
    for (int j = 0; j < extForces.size(); j++) {
        GLfloat *p = forceVertices + forceCount;

        *p++ = GLfloat(extForces[j][0]);
        *p++ = GLfloat(extForces[j][1]);
        *p++ = GLfloat(extForces[j][2]);
        forceCount += 3;
    }

    // ANCHORS
    anchors = vector<Vec>();
    for (int i = 0; i < n_masses; i++) {
        Mass *m = sim->getMassByIndex(i);

        if (m->constraints.fixed) {
            anchors.push_back(m->pos);
        }
    }
    if (resizeBuffers) {
        delete anchorVertices;
        anchorVertices = new GLfloat[3 * anchors.size()];
    }
    int anchorCount = 0;
    for (int j = 0; j < anchors.size(); j++) {
        GLfloat *p = anchorVertices + anchorCount;

        *p++ = GLfloat(anchors[j][0]);
        *p++ = GLfloat(anchors[j][1]);
        *p++ = GLfloat(anchors[j][2]);
        anchorCount += 3;
    }


    //  BOUNDS
    if (resizeBuffers) {
        delete boundVertices;
        boundVertices = new GLfloat[3 * originalModel->hull.size()];
    }
    int boundCount = 0;
    for (int i = 0; i < originalModel->hull.size(); i++) {
        GLfloat *p = boundVertices + boundCount;
        Mass *m = sim->getMassByIndex(originalModel->hull[i]);

        *p++ = GLfloat(m->pos[0]);
        *p++ = GLfloat(m->pos[1]);
        *p++ = GLfloat(m->pos[2]);
        boundCount += 3;
    }

}

//  ----- addColor()
//  Fills a color buffer with a 4-flot color starting from an offset
//
void Simulator::addColor(GLfloat *buffer, const GLfloat *color, int &count) {

    GLfloat *p = buffer + count;

    *p++ = color[0];
    *p++ = color[1];
    *p++ = color[2];
    *p++ = color[3];

    count += 4;
}

//  ----- addMassColor()
//  Fills a color buffer with a 4-float color based on mass type
//
void Simulator::addMassColor(Mass *mass, GLfloat *buffer, int &count) {

    if (mass->constraints.fixed) {

        addColor(buffer, fixedMassColor, count);

    } else if (fabs(mass->force[0]) > 1E-6 || fabs(mass->force[1]) > 1E-6 || fabs(mass->force[2]) > 1E-6) {

        addColor(buffer, forceMassColor, count);

    } else {

        addColor(buffer, defaultMassColor, count);
    }

}

//  ----- addSpringColor()
//  Fills a color buffer with two 4-float colors based on spring type
//
void Simulator::addSpringColor(Spring *spring, double totalStress, double totalForce, uint index, GLfloat *buffer, int &count) {
    if (spring->_broken) {

        addColor(buffer, brokenSpringColor, count);
        addColor(buffer, brokenSpringColor, count);

    } else if (showStress) {


        GLfloat *stressColor = new GLfloat[4];
        //for (uint i = 0; i < 3; i++) {
        //    stressColor[i] = defaultSpringColor[i] * float(spring->_max_stress / totalStress);
        //}
        GUtils::interpolateColors(contractSpringColor, expandSpringColor, -totalForce, totalForce,
                spring->_curr_force, stressColor);

        // Set alpha to max stress value
        stressColor[3] = float(abs(spring->_curr_force) / totalForce);

        addColor(buffer, stressColor, count);
        addColor(buffer, stressColor, count);

        delete[] stressColor;
    } else {

        addColor(buffer, defaultSpringColor, count);
        addColor(buffer, defaultSpringColor, count);

    }
}

//  ----- updateColors()
//  Fills color array with values according to mass properties
//
void Simulator::updateColors() {
    int colorsCount = 0;
    //n_masses = long(sim->masses.size());
    //n_springs = long(sim->springs.size());
    if (resizeBuffers) {
        delete colors;
        colors = new GLfloat[2 * 4 * (2 * n_springs)];
    }

    for (int i = 0; i < n_springs; i++) {

        Spring *s = sim->getSpringByIndex(i);

        // MASS VERTEX COLORS
        Mass *m1 = s->_left;
        addMassColor(m1, colors, colorsCount);

        Mass *m2 = s->_right;
        addMassColor(m2, colors, colorsCount);
    }

    double totalStress = 0;
    double totalForce = 0;
    if (showStress) {
        for (Spring *s: sim->springs) {
            totalStress = fmax(totalStress, s->_max_stress);
            totalForce = fmax(totalForce, fabs(s->_curr_force));
        }
    }

    for (int i = 0; i < n_springs; i++) {
        Spring *s = sim->getSpringByIndex(i);

        // SPRING VERTEX COLORS
        addSpringColor(s, totalStress, totalForce, i, colors, colorsCount);
    }

}

void Simulator::updateDiameters() {
    int diamCount = 0;
    n_springs = long(sim->springs.size());

    if (resizeBuffers) {
        delete diameters;
        diameters = new GLfloat[2 * n_springs];
    }

    for (int i = 0; i < n_springs; i++) {
        GLfloat *p = diameters + diamCount;
        Spring *s = sim->getSpringByIndex(i);

        *p++ = s->_diam;
        *p++ = s->_diam;

        diamCount += 2;
    }
}

//  ----- updatePlaneVertices()
//  Pushes simulation plane positions to plane arrays
//
void Simulator::updatePlaneVertices() {
    int pVerticesCount = 0;

    for (ulong i =0; i < ulong(n_planes); i++) {
        GLfloat *p = planeVertices + pVerticesCount;
        ContactPlane *c = sim->planes[i];

        Vec temp = (dot(c->_normal, Vec(0, 1, 0)) < 0.8) ? Vec(0, 1, 0) : Vec(1, 0, 0);
        Vec u1 = cross(c->_normal, temp).normalized();
        Vec u2 = cross(c->_normal, u1).normalized();
        Vec v1 = c->_offset * c->_normal - 1*u1 - 1*u2;
        Vec v2 = c->_offset * c->_normal + 1*u1 - 1*u2;
        Vec v3 = c->_offset * c->_normal + 1*u1 + 1*u2;
        Vec v4 = c->_offset * c->_normal - 1*u1 + 1*u2;
        qDebug() << v1[0] << v1[1] << v1[2];
        qDebug() << v2[0] << v2[1] << v2[2];
        qDebug() << v3[0] << v3[1] << v3[2];
        qDebug() << v4[0] << v4[1] << v4[2];

        *p++ = GLfloat(v1[0]); *p++ = GLfloat(v1[1]); *p++ = GLfloat(v1[2]);
        *p++ = GLfloat(v2[0]); *p++ = GLfloat(v2[1]); *p++ = GLfloat(v2[2]);
        *p++ = GLfloat(v3[0]); *p++ = GLfloat(v3[1]); *p++ = GLfloat(v3[2]);
        *p++ = GLfloat(v4[0]); *p++ = GLfloat(v4[1]); *p++ = GLfloat(v4[2]);
        pVerticesCount += 12;
    }
}


//  ----- initVertexArray()
//  Create the vertex array object
//
void Simulator::initVertexArray() {
    vertexArray.create();
    vertexArray.bind();
}


//  ----- initShader()
//  Creates an QOpenGLShaderProgram with source code shaders
//  Binds uniform locations
//
void Simulator::initShader() {
    massShaderProgram = new QOpenGLShaderProgram;
    massShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    massShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    massShaderProgram->bindAttributeLocation("vertexPos", 0);
    massShaderProgram->bindAttributeLocation("colorPos", 1);
    massShaderProgram->link();

    springShaderProgram = new QOpenGLShaderProgram;
    if (showCrossSection) {
        springShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, clipVertexShaderSource);
    }
    else {
        //springShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSpringSource);
        springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/bars.vert");
    }
    //springShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSpringSource);
    springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/resources/shaders/lineCylinders.geom");
    springShaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/bars.frag");
    springShaderProgram->bindAttributeLocation("vertexPos", 0);
    springShaderProgram->bindAttributeLocation("colorPos", 2);
    springShaderProgram->bindAttributeLocation("diameter", 3);
    springShaderProgram->link();

    planeShaderProgram = new QOpenGLShaderProgram;
    planeShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderPlaneSource);
    planeShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderPlaneSource);
    planeShaderProgram->bindAttributeLocation("vertexPos", 3);
    planeShaderProgram->link();

    axesShaderProgram = new QOpenGLShaderProgram;
    GUtils::initOneColorShaderProgram(axesShaderProgram, 4);
    axesShaderProgram->link();

    forceShaderProgram = new QOpenGLShaderProgram;
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/arrows.vert");
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/resources/shaders/arrows.geom");
    forceShaderProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/arrows.frag");
    forceShaderProgram->bindAttributeLocation("vertexPos", 5);
    forceShaderProgram->bindAttributeLocation("forceVec", 6);
    forceShaderProgram->link();

    anchorShaderProgram = new QOpenGLShaderProgram;
    anchorShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderAnchorSource);
    anchorShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderAnchorSource);
    anchorShaderProgram->bindAttributeLocation("vertexPos", 0);
    anchorShaderProgram->link();

    boundShaderProgram = new QOpenGLShaderProgram;
    boundShaderProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderBoundSource);
    boundShaderProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderBoundSource);
    boundShaderProgram->bindAttributeLocation("vertexPos", 0);
    boundShaderProgram->link();

    // Bind uniform matrix locations
    massShaderProgram->bind();
    projMatrixLoc = massShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = massShaderProgram->uniformLocation("mvMatrix");
    massShaderProgram->release();

    springShaderProgram->bind();
    projMatrixLoc = springShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = springShaderProgram->uniformLocation("mvMatrix");
    normalMatrixLoc = springShaderProgram->uniformLocation("normalMatrix");
    lightPosLoc = springShaderProgram->uniformLocation("lightPos");
    if (showCrossSection) {
        centerLoc = springShaderProgram->uniformLocation("center");
        cpXLoc = springShaderProgram->uniformLocation("clipPlaneX");
        cpYLoc = springShaderProgram->uniformLocation("clipPlaneY");
        cpZLoc = springShaderProgram->uniformLocation("clipPlaneZ");
    }
    springShaderProgram->setUniformValue(lightPosLoc, 0, 0, 70);
    springShaderProgram->release();

    planeShaderProgram->bind();
    projMatrixLoc = planeShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = planeShaderProgram->uniformLocation("mvMatrix");
    planeShaderProgram->release();

    axesShaderProgram->bind();
    projMatrixLoc = axesShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = axesShaderProgram->uniformLocation("mvMatrix");
    axesShaderProgram->release();

    forceShaderProgram->bind();
    fprojMatrixLoc = forceShaderProgram->uniformLocation("projMatrix");
    fmvMatrixLoc = forceShaderProgram->uniformLocation("mvMatrix");
    arrowLenLoc = forceShaderProgram->uniformLocation("arrowLength");
    scaleLoc = forceShaderProgram->uniformLocation("scale");
    forceShaderProgram->release();

    anchorShaderProgram->bind();
    projMatrixLoc = anchorShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = anchorShaderProgram->uniformLocation("mvMatrix");
    anchorShaderProgram->release();

    boundShaderProgram->bind();
    projMatrixLoc = boundShaderProgram->uniformLocation("projMatrix");
    mvMatrixLoc = boundShaderProgram->uniformLocation("mvMatrix");
    boundShaderProgram->release();

}


//  ----- initBuffers()
//  Creates buffers for vertex, index, and color data
//
void Simulator::initBuffers() {
    // 0
    /**GLuint vertexBuffer = 0;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * n_masses * long(sizeof(GLfloat)), vertices, GL_DYNAMIC_DRAW);
    //cudaGLRegisterBufferObject(vertexBuffer);
    vertexBuff_id = vertexBuffer;**/

    /**GLuint indexBuffer = 0;
    glGenBuffers(1, &indexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, indexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLuint)), nullptr, GL_DYNAMIC_DRAW);
    cudaGLRegisterBufferObject(indexBuffer);
    indexBuff_id = indexBuffer;**/

    // 1
    GLuint pairBuffer = 0;
    glGenBuffers(1, &pairBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, pairBuffer);
    glBufferData(GL_ARRAY_BUFFER, verticesCount * long(sizeof(GLfloat)), pairVertices, GL_DYNAMIC_DRAW);
    //cudaGLRegisterBufferObject(pairBuffer);
    pairVertexBuff_id = pairBuffer;

    // 2
    GLuint colorBuffer = 0;
    glGenBuffers(1, &colorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors, GL_DYNAMIC_DRAW);
    massColorBuff_id = colorBuffer;

    // 3
    GLuint scolorBuffer = 0;
    glGenBuffers(1, &scolorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, scolorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors + (8 * n_springs), GL_DYNAMIC_DRAW);
    springColorBuff_id = scolorBuffer;

    GLuint diamBuffer = 0;
    glGenBuffers(1, &diamBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, diamBuffer);
    glBufferData(GL_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLfloat)), diameters, GL_DYNAMIC_DRAW);
    diameterBuff_id = diamBuffer;

    // 4
    GLuint planeBuffer = 0;
    glGenBuffers(1, &planeBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, planeBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * 4 * long(sizeof(GLfloat)), planeVertices, GL_DYNAMIC_DRAW);
    planeVertexBuff_id = planeBuffer;

    // 5
    GUtils::createMainAxes(axesVertexBuff_id, 10.0f);

    /*GLuint textbuffer = 0;
    glGenBuffers(1, &textbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, textbuffer);
    glBufferData(GL_ARRAY_BUFFER, 6 * 4 * long(sizeof(GLfloat)), nullptr, GL_DYNAMIC_DRAW);
    textVertexBuff_id = textbuffer;*/

    GLuint forceBuffer = 0;
    glGenBuffers(1, &forceBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, forceBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * extForces.size() * long(sizeof(GLfloat)), forceVertices, GL_DYNAMIC_DRAW);
    forceVertexBuff_id = forceBuffer;

    GLuint anchorBuffer = 0;
    glGenBuffers(1, &anchorBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, anchorBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * anchors.size() * long(sizeof(GLfloat)), anchorVertices, GL_DYNAMIC_DRAW);
    anchorVertexBuff_id = anchorBuffer;

    GLuint boundBuffer = 0;
    glGenBuffers(1, &boundBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, boundBuffer);
    glBufferData(GL_ARRAY_BUFFER, 3 * originalModel->hull.size() * long(sizeof(GLfloat)), boundVertices, GL_DYNAMIC_DRAW);
    boundVertexBuff_id = boundBuffer;

    qDebug() << "Initialized buffers";
}


//  ----- updateShader()
//  Updates uniform matrices and set them in the shader
//
void Simulator::updateShader() {

    getBoundingBox();

    world.setToIdentity();
    world.rotate(m_xRot / 16.0f, 1, 0, 0);
    world.rotate(m_yRot / 16.0f, 0, 1, 0);
    world.rotate(m_zRot / 16.0f, 0, 0, 1);
    world.scale(m_zoom);
    scale = sim->springs[0]->_diam / 0.002;

    normal = world.normalMatrix();

    massShaderProgram->bind();
    massShaderProgram->setUniformValue(projMatrixLoc, projection);
    massShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    massShaderProgram->release();

    springShaderProgram->bind();
    springShaderProgram->setUniformValue(projMatrixLoc, projection);
    springShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    springShaderProgram->setUniformValue(normalMatrixLoc, normal);
    springShaderProgram->setUniformValue(lightPosLoc, 0, 0, 70);
    if (showCrossSection) {
        springShaderProgram->setUniformValue(centerLoc, center);
        springShaderProgram->setUniformValue(cpXLoc, clipPlaneX);
        springShaderProgram->setUniformValue(cpYLoc, clipPlaneY);
        springShaderProgram->setUniformValue(cpZLoc, clipPlaneZ);
    }
    springShaderProgram->release();

    planeShaderProgram->bind();
    planeShaderProgram->setUniformValue(projMatrixLoc, projection);
    planeShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    planeShaderProgram->release();

    axesShaderProgram->bind();
    axesShaderProgram->setUniformValue(projMatrixLoc, projection);
    axesShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    axesShaderProgram->release();

    /*textShaderProgram->bind();
    textShaderProgram->setUniformValue(textProjLoc, ortho);
    textShaderProgram->release();*/

    forceShaderProgram->bind();
    forceShaderProgram->setUniformValue(fprojMatrixLoc, projection);
    forceShaderProgram->setUniformValue(fmvMatrixLoc, camera * world);
    forceShaderProgram->setUniformValue(arrowLenLoc, 0.1f);
    forceShaderProgram->setUniformValue(scaleLoc, scale);
    forceShaderProgram->release();

    anchorShaderProgram->bind();
    anchorShaderProgram->setUniformValue(projMatrixLoc, projection);
    anchorShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    anchorShaderProgram->release();

    boundShaderProgram->bind();
    boundShaderProgram->setUniformValue(projMatrixLoc, projection);
    boundShaderProgram->setUniformValue(mvMatrixLoc, camera * world);
    boundShaderProgram->release();
}


//  ----- updateBuffers()
//  Binds buffers and resyncs underlying data arrays
//
void Simulator::updateBuffers() {

    n_masses = int(sim->masses.size());
    n_springs = int(sim->springs.size());

    //cudaGLUnregisterBufferObject(vertexBuff_id);
    //glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_id);
    //glBufferData(GL_ARRAY_BUFFER, 3 * n_masses * long(sizeof(GLfloat)), vertices, GL_DYNAMIC_DRAW);
    //cudaGLRegisterBufferObject(vertexBuff_id);
    //updateVertices();

    /**cudaGLUnregisterBufferObject(indexBuff_id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuff_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLuint)), nullptr, GL_DYNAMIC_DRAW);
    cudaGLRegisterBufferObject(indexBuff_id);
    updateIndices();**/

    //cudaGLUnregisterBufferObject(pairVertexBuff_id);
    glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, verticesCount * long(sizeof(GLfloat)), pairVertices, GL_DYNAMIC_DRAW);
    //cudaGLUnregisterBufferObject(pairVertexBuff_id);
    //updatePairVertices();

    glBindBuffer(GL_ARRAY_BUFFER, massColorBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, springColorBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 8 * n_springs * long(sizeof(GLfloat)), colors + (8 * n_springs), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, diameterBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 2 * n_springs * long(sizeof(GLfloat)), diameters, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, planeVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * 4 * long(sizeof(GLfloat)), planeVertices, GL_DYNAMIC_DRAW);

    //glBindBuffer(GL_ARRAY_BUFFER, textVertexBuff_id);
    //glBufferData(GL_ARRAY_BUFFER, 6 * 4 * long(sizeof(GLfloat)), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, forceVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * extForces.size() * long(sizeof(GLfloat)), forceVertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, anchorVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * anchors.size() * long(sizeof(GLfloat)), anchorVertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, boundVertexBuff_id);
    glBufferData(GL_ARRAY_BUFFER, 3 * originalModel->hull.size() * long(sizeof(GLfloat)), boundVertices, GL_DYNAMIC_DRAW);

}


//  ----- drawVertexArray()
//  Bind buffers and draws the vertex array object
//
void Simulator::drawVertexArray() {

    // DRAW AXES
    axesShaderProgram->bind();
    GUtils::drawMainAxes(4, axesVertexBuff_id);
    axesShaderProgram->release();


    //  DRAW PLANES
    planeShaderProgram->bind();

    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, planeVertexBuff_id);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4 * GLsizei(n_planes));

    glDisableVertexAttribArray(3);

    planeShaderProgram->release();

    // BOUNDS
    boundShaderProgram->bind();

    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, boundVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(originalModel->hull.size()));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDisableVertexAttribArray(0);

    boundShaderProgram->release();

    // DRAW MASSES
    massShaderProgram->bind();

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    //glBindBuffer(GL_ARRAY_BUFFER, vertexBuff_id);
    glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, massColorBuff_id);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glPointSize(10.0f);
    if (showVertices && !showCrossSection)
        glDrawArrays(GL_POINTS, 0, GLsizei(n_springs * 2));

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    massShaderProgram->release();


    //  DRAW SPRINGS
    springShaderProgram->bind();

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, pairVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, springColorBuff_id);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, diameterBuff_id);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, nullptr);

    //glDrawElements(GL_LINES, 2 * GLsizei(n_springs), GL_UNSIGNED_INT, nullptr);
    glDrawArrays(GL_LINES, 0, GLsizei(verticesCount / 3));

    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(0);

    springShaderProgram->release();

    //  DRAW OVERLAYS
    forceShaderProgram->bind();

    glEnableVertexAttribArray(5);
    glEnableVertexAttribArray(6);

    glBindBuffer(GL_ARRAY_BUFFER, forceVertexBuff_id);
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), nullptr);
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));

    glDrawArrays(GL_POINTS, 0, GLsizei(extForces.size()));

    glDisableVertexAttribArray(6);
    glDisableVertexAttribArray(5);

    forceShaderProgram->release();

    anchorShaderProgram->bind();

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, anchorVertexBuff_id);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glDrawArrays(GL_POINTS, 0, GLsizei(anchors.size()));

    glDisableVertexAttribArray(0);
    anchorShaderProgram->release();
}


//  ----- cleanUp()
//  Deletes objects and pointers
//
void Simulator::cleanUp() {

    if (massShaderProgram == nullptr)
        return;

    delete vertices;
    delete indices;
    delete pairVertices;
    delete boundVertices;
    delete anchorVertices;
    delete forceVertices;
    delete colors;
    delete diameters;
    delete planeVertices;

    delete timer;

    makeCurrent();
    glDeleteBuffers(1, &vertexBuff_id);
    glDeleteTextures(1, &textVertexBuff_id);
    delete massShaderProgram;
    delete springShaderProgram;
    delete planeShaderProgram;
    delete textShaderProgram;
    delete boundShaderProgram;
    delete forceShaderProgram;
    delete anchorShaderProgram;
    massShaderProgram = nullptr;
    springShaderProgram = nullptr;
    planeShaderProgram = nullptr;
    textShaderProgram = nullptr;
    doneCurrent();
}


//  ----- initCamera()
//  Initializes camera matrix
//
void Simulator::initCamera() {
    camera.setToIdentity();
}


//  ----- getBoundingBox()
//  Finds bounds of the simulation masses
//
vector<Vec> Simulator::getBoundingBox() {
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
    vector<Vec> bounds = vector<Vec>();
    bounds.push_back(minCorner);
    bounds.push_back(maxCorner);

    Vec centerVec = 0.5 * (minCorner + maxCorner);
    m_center = QVector3D(float(centerVec[0]), float(centerVec[1]), float(centerVec[2]));
    return bounds;
}


// --------------------------------------------------------------------
// QT OPENGLWIDGET VIRUTAL FUNCTIONS
// --------------------------------------------------------------------


//  ----- initializeGL()
//  Virtual QOpenGLWidget function
//  Called once on widget construction
//
void Simulator::initializeGL() {

    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &Simulator::cleanUp);

    initializeOpenGLFunctions();
    qDebug() << "OpenGL Version: " << QLatin1String((const char *) glGetString(GL_VERSION));
    qDebug() << "GSLS version: " << QLatin1String((const char *) glGetString(GL_SHADING_LANGUAGE_VERSION));

    sim->initCudaParameters();

    if (showCrossSection) {
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CLIP_DISTANCE0);
        glEnable(GL_CLIP_DISTANCE1);
        glEnable(GL_CLIP_DISTANCE2);
    }

    qDebug() << "Initializing OpenGL";

    // CLIP PLANES
    center = QVector4D(0, 0, 0, 0);
    map<uint, bool> seenMasses = map<uint, bool>();
    qDebug() << originalModel->hull.size();

    if (originalModel->hull.size() > 0) {
        for (int i = 0; i < originalModel->hull.size(); i++) {

            if (seenMasses.find(originalModel->hull[i]) == seenMasses.end()) {
                Vec p = sim->masses[originalModel->hull[i]]->pos;
                center += QVector4D(p[0], p[1], p[2], 0);
                seenMasses[originalModel->hull[i]] = true;
            }
        }
        center /= seenMasses.size();
    } else {
        vec3 c = originalModel->bounds.center;
        center = QVector4D(c[0], c[1], c[2], 0.0);
    }


    qDebug() << "Center:" << center << seenMasses.size();

    Vec dx = sim->masses[2]->pos - sim->masses[0]->pos;
    Vec dy = sim->masses[7]->pos - sim->masses[4]->pos;
    Vec dz = sim->masses[0]->pos - sim->masses[7]->pos;
    clipPlaneX = QVector4D(dx[0], dx[1], dx[2], 0.0);
    clipPlaneY = QVector4D(dy[0], dy[1], dy[2], 0.0);
    clipPlaneZ = QVector4D(dz[0], dz[1], dz[2], 0.0);

    qDebug() << "Clipping plane X:" << clipPlaneX;
    qDebug() << "Clipping plane Y:" << clipPlaneY;
    qDebug() << "Clipping plane Z:" << clipPlaneZ;

    //updateVertices();
    //updateIndices();
    updateColors();
    updatePairVertices();
    updateDiameters();
    updatePlaneVertices();
    updateOverlays();

    qDebug() << "Updated vertices";

    initShader();
    initVertexArray();
    initBuffers();

    qDebug() << "Initialized QOpenGLWidget";
    //updateVertices();
    //updateIndices();
    //updatePairVertices();

    //initTextLibrary();


    camera.setToIdentity();
    camera.translate(0, 0, -1);
    Vec up = -sim->global.normalized();
    if (up == Vec(0, 0, 0)) { up = Vec(0, 0, 1); }

    vector<Vec> bounds = getBoundingBox();
    double span = (bounds[0] - bounds[1]).norm();
    m_zoom =  span > 1E-5? float(1/span) : 1.0f;

    camera.lookAt(QVector3D(4, 3, 3), QVector3D(0, 0, 0), QVector3D(float(up[0]), float(up[1]), float(up[2])));

    ortho.setToIdentity();
    ortho.ortho(0.0f, 800.0f, 0.0f, 600.0, 0.1f, 100.0f);

}


//  ----- paintGL()
//  Virtual QOpenGLWidget function
//  Called every frame update
//
void Simulator::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glClearColor(0, 0, 0, 0);

    if (showCrossSection) {
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CLIP_DISTANCE0);
        glEnable(GL_CLIP_DISTANCE1);
        glEnable(GL_CLIP_DISTANCE2);
    }

    vertexArray.bind();
    updateShader();
    updateBuffers();
    drawVertexArray();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    vertexArray.release();

    updateTextPanel();
    m_frame++;
}


//  ----- resizeGL()
//  Virtual QOpenGLWidget function
//  Called for window resizing
//
void Simulator::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, 0.01f, 100.0f);
}

//  ----- renderText()
//  Uses a QPainter to overlay text on OpenGL widget
//
void Simulator::renderText(const QString &text, int flags) {
    QPainter painter(this);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Helvetica [Cronyx]", 14));
    painter.drawText(rect(), flags, text);
}

//  ----- updateTextPanel()
//  Updates text panel with current stats about the simulation
//
void Simulator::updateTextPanel() {

    //double totalMass = sim->getTotalMass();
    totalLength = 0;
    totalEnergy = 0;
    double pi = atan(1.0)*4;
    for (Spring * s: sim->springs) {
        totalLength += s->_rest;
        totalEnergy += s->_curr_force * s->_curr_force / s->_k;
    }

    QString upperPanel;
    QString simName = config->id;
    double latticeCutoff = config->lattice.unit[0];
    QString material = config->lattice.material->id;

    if (optConfig != nullptr) {
        switch(optConfig->rules.front().method) {
            case OptimizationRule::REMOVE_LOW_STRESS:
                upperPanel.sprintf("%s --- %s\n\n"
                                   "Bars: %d\n"
                                   "Time: %.2lf s\n"
                                   "Weight remaining: %.2lf%\n"
                                   "Optimization iterations: %d\n"
                                   "Optimization threshold: %.1lf% bars per iteration",
                                   simName.toUpper().toStdString().c_str(),
                                   optConfig->rules.front().methodName().replace(QChar('_'), QChar(' ')).toStdString().c_str(),
                                   sim->springs.size(),
                                   sim->time(), 100.0 * totalLength / totalLength_start,
                                   optimized,
                                   optConfig->rules.front().threshold * 100);
                break;

            case OptimizationRule::MASS_DISPLACE:
                upperPanel.sprintf("%s\n\n"
                                   "Bars: %d\n"
                                   "Time: %.2lf s\n"
                                    "Weight remaining: %.2lf%\n"
                                    "Energy: %.2lf%, %.4lf (current), %.4lf (start)\n"
                                    "Optimization iterations: %d\n"
                                    "Relaxation interval: %d order\n"
                                    "Displacement: %.4lf meters",
                                    simName.toUpper().toStdString().c_str(), sim->springs.size(),
                                    sim->time(), 100.0 * totalLength / totalLength_start,
                                    (100.0 * totalEnergy / ((totalEnergy_start > 0)? totalEnergy_start : totalEnergy)),
                                    totalEnergy,
                                    totalEnergy_start,
                                    optimized,
                                    massDisplacer->relaxation,
                                    massDisplacer->dx);
                break;

            case OptimizationRule::NONE:
                upperPanel.sprintf("%s\n\n"
                                   "Bars: %d\n"
                                   "Time: %.2lf s\n",
                                   simName.toUpper().toStdString().c_str(), sim->springs.size(),
                                   sim->time(), 100.0 * totalLength / totalLength_start);
                break;
        }
    }


    QString lowerPanel =
            tr("Random Lattice\n"
               "Spacing cutoff: %1 m\n"
               "Bounds: %2 x %3 x %4 m\n"
               "Material: %5")
               .arg(latticeCutoff)
               .arg(bounds[1][0] - bounds[0][0])
               .arg(bounds[1][1] - bounds[0][1])
               .arg(bounds[1][2] - bounds[0][2])
               .arg(material.toUpper());

    renderText(upperPanel, Qt::AlignLeft | Qt::AlignTop);
    renderText(lowerPanel, Qt::AlignLeft | Qt::AlignBottom);
}

//  ----- mousePressEvent()
//  Overrides listener for mouse presses
//
void Simulator::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->pos();
}


//  ----- mouseMoveEvent()
//  Overrides listener for mouse moves to update rotation matrix
//
void Simulator::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastMousePos.x();
    int dy = event->y() - m_lastMousePos.y();

    if (event->buttons() & Qt::LeftButton) {
        rotateBy(6 * dy, 6 * dx, 0);
    }
    m_lastMousePos = event->pos();
}


//  ----- wheelEvent()
//  Overrides listener for zooming to scale object
//
void Simulator::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? m_zPan = 1 : m_zPan = -1;
    panCameraBy(0, 0, int(m_zPan) * 5);
    update();
}


//  ----- keyPressEvent()
//  Overrides listener for key presses
//
void Simulator::keyPressEvent(QKeyEvent *event)
{
    switch(event->key()) {
        case Qt::Key_Left:
            panCameraBy(1, 0, 0);
            break;
        case Qt::Key_Right:
            panCameraBy(-1, 0, 0);
            break;
        case Qt::Key_Down:
            panCameraBy(0, 1, 0);
            break;
        case Qt::Key_Up:
            panCameraBy(0, -1, 0);
        break;
    }
}


//  ----- rotateBy()
//  Helper function to update rotation parameter and re-paint widget
//
void Simulator::rotateBy(int xAngle, int yAngle, int zAngle)
{
    m_xRot += xAngle;
    m_yRot += yAngle;
    m_zRot += zAngle;
    update();
}

//  ----- panCameraBy()
//  Helper function to update camera parameter and re-paint widget
//
void Simulator::panCameraBy(int x, int y, int z)
{
    m_xPan = 0.1f * x;
    m_yPan = 0.1f * y;
    m_zPan = 0.1f * z;

    QVector4D u = camera.row(0);
    QVector4D v = camera.row(1);
    QVector4D n = camera.row(2);

    camera.setColumn(3, QVector4D(u.w() + m_xPan,
                                  v.w() + m_yPan,
                                  n.w() + m_zPan,
                                  1.0f));
    update();
}


// --------------------------------------------------------------------
// VIDEO OUTPUT
// --------------------------------------------------------------------
void Simulator::saveImage(const QImage &image, const QString &outputFile) {
    image.save(outputFile);
    qDebug() << "Saved" << outputFile;
}
