#include "loader.h"

#define EPSILON 1E-5
#define NUM_RAYS 1

Loader::Loader(QObject *parent) : QObject(parent)
{

}

Loader::~Loader() {
}

/**
 * @brief Loader::loadDesignModels
 * Loads model_data into each design volume
 * Creates OpenGL data for rendering
 */
void Loader::loadDesignModels(Design *design) {
    for (uint v = 0; v < design->volumes.size(); v++) {
        loadVolumeModel(design->volumes[v]);
        loadVolumeGeometry(design->volumes[v]);
	#ifdef USE_OpenGL
        design->volumes[v]->model->createGraphicsData();
	#endif
    }
    for (uint s = 0; s < design->simConfigs.size(); s++) {
        design->simConfigs[s] = *design->simConfigMap[design->simConfigs[s].id];
        loadSimModel(&design->simConfigs[s]);
        design->simConfigMap[design->simConfigs[s].id] = &design->simConfigs[s];
    }
}

/**
 * @brief Loader::loadVolumeModel
 * Populates model_data in a given Volume
 * Note that model_data here will have only one mesh per volume
 */
void Loader::loadVolumeModel(Volume *volume) {

    volume->model = new model_data();
    volume->model->model_indices = new int[1];
    volume->model->colors = vector<vec4>();
    volume->model->n_models = 1;

    vec4 color = vec4(volume->color.x(),
                      volume->color.y(),
                      volume->color.z(),
                      volume->color.w());
    volume->model->colors.push_back(color);
    if (volume->primitive == "stl") {
        cout << "\nAttempting to load " << volume->url.path().toStdString() << "\n";
        if (volume->url.isEmpty())
            log("Empty URL for volume " + volume->id + ". Cannot load model");

        if (volume->url.isValid()) {

            float scale = 1;
            if (volume->units != nullptr) {
                if (volume->units == "meters" || volume->units == "m") {
                    scale = 1;
                } else if (volume->units == "centimeters" || volume->units == "cm") {
                    scale = 0.01;
                } else if (volume->units == "millimeters" || volume->units == "mm") {
                    scale = 0.001;
                }
            }

            qDebug() << "Scale" << scale;

            Utils::createModelFromFile(volume->url.path().toStdString(), scale, volume->model->vertices, volume->model->normals);
            cout << "Created model from" << volume->url.path().toStdString() << "\n";
        } 
    } else if (volume->primitive == "cube") {
        Utils::createCube(vec3(0.0, 0.03, 0.0), 0.1f, volume->model->vertices, volume->model->normals);
    } else if (volume->primitive == "3mf") {
        loadGeometry3MF(volume->url.path().toStdString(), volume->model->vertices, volume->model->normals);
        cout << "Created model from" << volume->url.path().toStdString() << "\n";
    } else {
            log("Invalid URL: \'" + volume->url.url() + "\' for volume " + volume->id + ". cannot load model");
    }



    volume->model->n_vertices = int(volume->model->vertices.size());
    volume->model->n_normals = int(volume->model->normals.size());
    volume->model->model_indices[0] = volume->model->n_vertices;
    volume->model->bounds = boundingBox<vec3>(volume->model->vertices);

    log(QString("Loaded %1 vertices").arg(volume->model->n_vertices));
}

/**
 * @brief Loader::loadVolumeGeometry
 * Populates Polygon in a given Volume
 */
void Loader::loadVolumeGeometry(Volume *volume){

    volume->geometry = new Polygon();

    if (volume->primitive == "stl") {
        if (volume->url.isEmpty())
            log("Empty URL for volume " + volume->id + ". Cannot load model");

        if (volume->url.isValid()) {

            // SCALE
            float scale = 1;
            if (volume->units != nullptr) {
                if (volume->units == "meters" || volume->units == "m") {
                    scale = 1;
                } else if (volume->units == "centimeters" || volume->units == "cm") {
                    scale = 0.01;
                } else if (volume->units == "millimeters" || volume->units == "mm") {
                    scale = 0.001;
                }
            }

            // LOAD FROM INPUT FILE
            volume->geometry->createPolygonFromFile(volume->url.path().toStdString(), scale);

            /**iUtils::iglMesh *mesh = new iUtils::iglMesh();
            iUtils::iglFromGeometry(*volume->geometry, *mesh);
            qDebug() << "Matrix";
            qDebug() << "\tVertices" << mesh->V.cols() << mesh->V.rows();
            qDebug() << "\tFaces" << mesh->F.cols() << mesh->F.rows();
            volume->mesh = mesh;**/

        } else if (volume->primitive == "3mf") {
            volume->geometry->createPolygonFrom3MF(volume->url.path().toStdString());
            cout << "Created polygon from" << volume->url.path().toStdString() << "\n";
        } else {
            log("Invalid URL: \'" + volume->url.url() + "\' for volume " + volume->id + ". cannot load model");
        }
    }

    log(QString("Loaded %1 vertices").arg(volume->geometry->nodeMap.size()));
}

void Loader::loadSimModel(SimulationConfig *simConfig) {
    Volume *simVolume = simConfig->volume;

    simConfig->model = new simulation_data(simVolume->model);
    //simConfig->model = new simulation_data(simVolume->geometry);
    qDebug() << "Loaded simulation data structure";
}


void Loader::loadVolumes(model_data *arrays, Design *design) {
    arrays->model_indices = new int[design->volumes.size()];
    arrays->colors = vector<glm::vec4>(design->volumes.size());
    arrays->n_models = design->volumes.size();
    for (uint v = 0; v < design->volumes.size(); v++) {
       // loadModel(arrays, &design->volumes.at(v), v);
    }
    arrays->n_models = 1;
    loadModel(arrays, design->volumes.at(0), 0);

    qDebug() << "Total vertices: " << arrays->vertices.size();
    qDebug() << "Total normals: " << arrays->normals.size();
    qDebug() << "Total indices: " << arrays->indices.size();
}

/**
 * @brief Volume::loadModel Reads primitive mesh and loads
 * vertices/colors/normals into model_data
 */
void Loader::loadModel(model_data *arrays, Volume *volume, uint n_volume) {

    arrays->colors[n_volume].r = volume->color.x();
    arrays->colors[n_volume].g = volume->color.y();
    arrays->colors[n_volume].b = volume->color.z();
    arrays->colors[n_volume].a = volume->color.w();

    if (volume->url.isEmpty())
        log("Empty URL for volume " + volume->id + ". Cannot load model");

    if (volume->url.isValid()) {


        /**if (n_volume == 0) {

            Utils::createCube(vec3(0.0, 0.0, 0.3f), 0.1f, arrays->vertices, arrays->normals);

            arrays->n_vertices = int(arrays->vertices.size());
            arrays->n_normals = int(arrays->normals.size());
            arrays->model_indices[0] = arrays->n_vertices;
            arrays->bounds = boundingBox(volume->model->vertices);

        } else {**/
            readSTLFromFile(arrays, volume->url.path(), n_volume);
        //}

    } else {
        log("Invalid URL: \'" + volume->url.url() + "\' for volume " + volume->id + ". cannot load model");
    }
}


void Loader::readSTLFromFile(model_data *arrays, QString filePath, uint n_model) {
    int n_vert = 0;
    int n_norm = 0;
    int n_tria = 0;
    log("Reading STL format from " + filePath);
    if(filePath.isEmpty()) {
        return;
    } else {
        QFile file(filePath);

        if(!file.open(QIODevice::ReadOnly)) {
            return;
        }

        QTextStream in(&file);
        int vertex = -1;

        while (!in.atEnd()) {
            QString line = in.readLine();
            if (line.trimmed().startsWith("facet")) {

                QStringList nstring = line.trimmed().split(" ");
                glm::vec3 nvector = glm::vec3(nstring.at(2).toDouble(),
                                              nstring.at(3).toDouble(),
                                              nstring.at(4).toDouble());
                arrays->normals.push_back(nvector);
                arrays->normals.push_back(nvector);
                arrays->normals.push_back(nvector);
                n_norm++;
            }
            if (line.trimmed() == "outer loop") {
                vertex = 0;
            }
            if (line.trimmed().startsWith("vertex")) {

                QStringList vstring = line.trimmed().split(" ");
                glm::vec3 vvector = glm::vec3(vstring.at(1).toDouble(),
                                              vstring.at(2).toDouble(),
                                              vstring.at(3).toDouble());
                arrays->vertices.push_back(vvector);
                n_vert++;
            }
            if (line.trimmed() == "endloop") {
                vertex = -1;
            }
        }

        file.close();
        n_tria = n_vert/3;
        log(QString("Read %1 triangles: %2 vertices and %3 normals.").arg(
                n_tria).arg(
                n_vert).arg(
                n_norm));
        arrays->n_vertices = arrays->vertices.size();
        arrays->n_normals = arrays->normals.size();
        arrays->model_indices[n_model] = arrays->n_vertices;
        qDebug() << "End bound " << arrays->model_indices[n_model];

        arrays->bounds = boundingBox<vec3>(arrays->vertices);
    }
}

void Loader::loadSimulation(Simulation *sim, SimulationConfig *simConfig) {

    int NUM_Y = 10, NUM_X = 10;
    float SIZE = 0.05, SPACE = 0.01;
    int DENSITY = 5;
    Mass * m1, *m2, *m3;
    Spring * s1, *s2;
    Lattice * grid[NUM_Y][NUM_X];
    int vs = simConfig->model->lattice.size();
    int count = 20;

    // Reload sim volume
    switch(simConfig->lattices[0]->fill) {
        case LatticeConfig::CUBIC_FILL:
            //createGridLattice(simConfig->volume->geometry, simConfig->lattice, float(simConfig->lattice.unit[0]));
            createGridLattice(simConfig->model, simConfig);

            //vs = simConfig->model->lattice.size();

            /**for (int i = 0; i < NUM_Y; i++) {
                for (int j = 0; j < NUM_X; j++) {
                    grid[i][j] = sim->createLattice(Vec((SIZE + SPACE) * j + SIZE / 2, SIZE / 2, (SIZE + SPACE) * i + SIZE / 2), Vec(SIZE, SIZE, SIZE), DENSITY, DENSITY, DENSITY);
                }
            }

            for (int i = 0; i < NUM_Y - 1; i++) {
                for (int j = 0; j < NUM_X - 1; j++) {

                    for (Mass *m1 : grid[i][j]->masses) {
                        if (m1->origpos[1] == SIZE || m1->origpos[1] == 0) {
                            for (Mass *m2 : grid[i + 1][j]->masses) {
                                if ((m1->origpos - m2->origpos).norm() < SPACE + 1E-4) {
                                    s1 = sim->createSpring(m1, m2);
                                    s1->_diam = 0.0004;
                                }
                            }
                        }
                    }

                    for (Mass *m1 : grid[i][j]->masses) {
                        if (m1->origpos[1] == SIZE || m1->origpos[1] == 0) {
                            for (Mass *m2 : grid[i][j + 1]->masses) {
                                if ((m1->origpos - m2->origpos).norm() < SPACE + 1E-4) {
                                    s1 = sim->createSpring(m1, m2);
                                    s1->_diam = 0.0004;
                                }
                            }
                        }
                    }
                }
            }
            for (int i = 0; i < NUM_Y-1; i++) {
                for (Mass *m1 : grid[i][NUM_X-1]->masses) {
                    if (m1->origpos[1] == SIZE || m1->origpos[1] == 0) {
                        for (Mass *m2 : grid[i + 1][NUM_X - 1]->masses) {
                            if ((m1->origpos - m2->origpos).norm() < SPACE + 1E-4) {
                                s1 = sim->createSpring(m1, m2);
                                s1->_diam = 0.0004;
                            }
                        }
                    }
                }
            }
            for (int j = 0; j < NUM_X-1; j++) {
                for (Mass *m1 : grid[NUM_Y-1][j]->masses) {
                    if (m1->origpos[1] == SIZE || m1->origpos[1] == 0) {
                        for (Mass *m2 : grid[NUM_Y-1][j+1]->masses) {
                            if ((m1->origpos - m2->origpos).norm() < SPACE + 1E-4) {
                                s1 = sim->createSpring(m1, m2);
                                s1->_diam = 0.0004;
                            }
                        }
                    }
                }
            }

            sim->defaultRestLength();**/


            /*for (int i = 0; i < count; i++) {
                for (int j = 0; j < count; j++) {
                    for (int v = 0; v < vs; v++) {
                        if (i == 0 && j == 0) continue;
                        simConfig->model->lattice.push_back(simConfig->model->lattice[v] + vec3(0.4 * i, 0.0, 0.2 * j));
                        simConfig->model->pointOrigins.push_back(simConfig->model->pointOrigins[v]);
                    }
                }
            }
            qDebug() << vs << simConfig->model->lattice.size() << simConfig->model->pointOrigins.size();*/

            break;
        case LatticeConfig::SPACE_FILL:
            // TODO: add conform bool as includeHull
            //createSpaceLattice(simConfig->volume->geometry, simConfig->lattice, float(simConfig->lattice.unit[0]), true);
            createSpaceLattice(simConfig->model, simConfig);
            break;
    }

    //loadSimFromLattice(&simConfig->lattice, sim, simConfig->lattice.unit[0]*springMult);
    loadSimFromLattice(simConfig->model, sim, simConfig->lattices);

    // PLANE
    if (simConfig->plane != nullptr) {
        sim->createPlane(simConfig->plane->normal, simConfig->plane->offset, 0.8, 1.0);
    }

    // DAMPING
    for (Mass *m : sim->masses) {
        m->damping = 1.0 - simConfig->damping.velocity;
    }
    qDebug() << "Damping" << simConfig->damping.velocity;

    // GLOBAL
    qDebug() << "Global" << simConfig->global.acceleration[0];
    sim->global = simConfig->global.acceleration;

    // DIAMETER
    for (Spring *s : sim->springs) {
        s->_diam = simConfig->lattices[0]->barDiameter[1];
    }
    qDebug() << "Set spring diameters";

    // LOADCASES
    if (simConfig->load != nullptr && simConfig->loadQueue.empty()) {
        applyLoadcase(sim, simConfig->load);
    } else {
        for (Loadcase *l : simConfig->loadQueue) {
            applyLoadcase(sim, l);
        }
    }


    qDebug() << "Applied loadcase";


    // MATERIAL
    for (LatticeConfig *lat : simConfig->lattices) {
        if (simConfig->lattices[0]->material != nullptr) {
            Material *mat = lat->material;

            // TODO: get correct bardiam dimension
            double maxK = 0;
            double minMassXVal = FLT_MAX;
            double maxMassXVal = -FLT_MAX;
            for (Mass *m : sim->masses) {
                minMassXVal = std::min(minMassXVal, m->origpos[2]);
                maxMassXVal = std::max(maxMassXVal, m->origpos[2]);
            }
            qDebug() << "Max X val" << maxMassXVal << minMassXVal;
            for (Spring *s : sim->springs) {

                // ELASTICITY -- SPRING CONSTANT
                if (mat->eUnits != nullptr) {
                    double pi = atan(1.0) * 4;
                    double a = pi * (simConfig->lattices[0]->barDiameter[1] / 2) *
                               (simConfig->lattices[0]->barDiameter[1] / 2);
                    double unit = 1;

                    if (mat->eUnits == "GPa") { unit *= 1000 * 1000 * 1000; }
                    if (mat->eUnits == "MPa") { unit *= 1000 * 1000; }


                    double k = mat->elasticity * unit * a / s->_rest;
                    //s->_k = Utils::interpolate(500, 2E5, minMassXVal, maxMassXVal, (s->_left->pos[2] + s->_right->pos[2])/2);
                    s->_k = k;
                    maxK = std::max(k, maxK);
                }

            }

            // YIELD
            double unit = 1;
            if (mat->yUnits == "GPa") { unit *= 1000 * 1000 * 1000; }
            if (mat->yUnits == "MPa") { unit *= 1000 * 1000; }
            for (Spring *s : sim->springs) {
                s->_break_force = 5 * mat->yield * unit;
            }

            // DENSITY -- MASS VALUES
            if (mat->dUnits != nullptr) {
                // Note that this is an approximation for volume governed up by a single mass
                double v = pow(simConfig->lattices[0]->unit[0], 3);
                double d = mat->density;
                double unit = 1;
                if (mat->dUnits == "gcc") { unit *= 1000; }

                switch (simConfig->lattices[0]->structure) {
                    case LatticeConfig::FULL:
                        qDebug() << v * d * unit;
                        for (Spring *s : sim->springs) {
                            double m = v * d * unit / 8;
                            //s->_mass = m;
                            s->_left->m += m / 2;
                            s->_right->m += m / 2;
                        }
                        qDebug() << sim->masses.front()->m;
                        break;
                    case LatticeConfig::BARS:
                        for (Mass *m : sim->masses) {
                            m->m = 0;
                            m->density = d * unit;
                        }
                        double totalM = 0;
                        for (Spring *s : sim->springs) {
                            // get volume for half
                            double vol = s->_rest / 2 * M_PI * s->_diam / 2 * s->_diam / 2;
                            qDebug() << vol << s->_rest / 2 << s->_diam / 2;
                            double m = vol * d * unit;
                            //s->_mass = 2 * m;
                            s->_left->m += m;
                            s->_right->m += m;
                            totalM += 2 * m;
                        }
                        qDebug() << "Total mass" << totalM << "kg";
                        break;
                }
            }
    }

        // TIMESTEP
        double maxNatFreq = 0;
        for (Spring * s: sim->springs) {
            double minM = std::min(s->_left->m, s->_right->m);
            maxNatFreq = std::max(sqrt(s->_k / minM), maxNatFreq);
        }
        double timestepLimit = 1 / (2 * 3.14159 * maxNatFreq);
        qDebug() << maxNatFreq << timestepLimit;
        //sim->masses.front()->extforce = Vec(-100, 0, 0);
        //sim->masses.front()->extduration = 0.1;

        // ACTUATION
        /*double minMassXVal = FLT_MAX;
        for (Mass *m : sim->masses) {
            minMassXVal = std::min(minMassXVal, m->origpos[0]);
        }
        for (Spring *s : sim->springs) {
                s->_type = 4;
                s->_omega = 20;
                s->_period = 0.5;
                s->_offset = (std::min(s->_left->origpos[0], s->_right->origpos[0]) - minMassXVal);
        }*/

    }

    // TIMESTEP
    //double timestep = std::min(1/pow(10, to_string(int(maxK)).length()-2), 0.0001);
    sim->setAllDeltaTValues(1e-4);

    //suggestParams(sim, simConfig);

    qDebug() << "Loaded simulation configuration";
}

void Loader::loadSimulation(simulation_data *arrays, Simulation *sim, uint n_volume) {

    // Create graphics data
    #ifdef USE_OpenGL
    arrays->createGraphicsData();
    #endif

    // Already existing triangle edges/springs
    // Stored in form key=(mass m), value=(masses connected to mass m)
    vector<vector<uint>> edges = vector<vector<uint>>(arrays->n_vertices, vector<uint>());

    // Create masses at vertices
    int modelStart = 0;
    int modelEnd = arrays->n_vertices;

    for (uint i = modelStart; i < modelEnd; i++) {
        sim->createMass(Vec(arrays->vertices.at(i).x * 100,
                            arrays->vertices.at(i).y * 100,
                            arrays->vertices.at(i).z * 100));
    }

    // Create springs at triangle edges
    for (uint j = 0; j < arrays->n_indices; j+=3) {
        // Get vertex indices
        uint v1 = arrays->indices[j];
        uint v2 = arrays->indices[j+1];
        uint v3 = arrays->indices[j+2];

        // Check if spring already exists
        vector<uint>::iterator it = find(edges[v1].begin(), edges[v1].end(), v2);

        if (it == edges[v1].end()) {
            sim->createSpring(sim->getMassByIndex(v1),
                              sim->getMassByIndex(v2));

            // Add to edges array
            edges[v1].push_back(v2);
            edges[v2].push_back(v1);
        }

        it = find(edges[v2].begin(), edges[v2].end(), v3);

        if (it == edges[v2].end()) {
            sim->createSpring(sim->getMassByIndex(v2),
                              sim->getMassByIndex(v3));

            // Add to edges array
            edges[v2].push_back(v3);
            edges[v3].push_back(v2);
        }

        it = find(edges[v3].begin(), edges[v3].end(), v1);

        if (it == edges[v3].end()) {
            sim->createSpring(sim->getMassByIndex(v3),
                              sim->getMassByIndex(v1));

            // Add to edges array
            edges[v3].push_back(v1);
            edges[v1].push_back(v3);
        }
    }

    //sim->createPlane(Vec(0, 0, 1), 0);

    qDebug() << "Loaded simulation";

    log(QString("Loaded simulation with %1 masses and %2 springs.")
        .arg(sim->masses.size())
        .arg(sim->springs.size()));
}


void Loader::loadSimFromLattice(simulation_data *arrays, Simulation *sim, vector <LatticeConfig *> lattices) {
    // Include varying springMult by looking at the type of the LatticeConfig
    float springCutoff = 0.0;
    double springMult = lattices[0]->fill == LatticeConfig::CUBIC_FILL ? 1.9 : 2.9;

    qDebug() << "Loading" << springCutoff;
    // Load hull first
    for (ulong i = 0; i < arrays->lattice.size(); i++) {
        sim->createMass(Vec(arrays->lattice.at(i).x,
                            arrays->lattice.at(i).y ,
                            arrays->lattice.at(i).z));
        //sim->getMassByIndex(i)->damping = 0.9999;
    }

    for (int j = 0; j < sim->masses.size() - 1; j++) {
        Mass *massj = sim->masses[j];

        float jCutoff, kCutoff;

        jCutoff = arrays->pointOrigins.at(j)->unit[0] * springMult;

        for (int k = j+1; k < sim->masses.size(); k++) {
            Mass *massk = sim->masses[k];
            double dist = (massj->origpos - massk->origpos).norm();

            kCutoff = arrays->pointOrigins.at(k)->unit[0] * springMult;

            springCutoff = std::min(jCutoff, kCutoff);

            if (dist <= springCutoff) {
                sim->createSpring(massj, massk);
                sim->springs.back()->setRestLength(dist);
            }
        }
    }
    qDebug() << sim->springs.size();


    //int num_masses = int(sim->masses.size());


    /**for (int i = 0; i < num_masses; i++) {
            Mass *m = sim->getMassByIndex(i);
            Mass *n, *n1, *n2;
            Vec npos, n1pos, n2pos;

            npos = m->pos - Vec(springCutoff, 0, 0);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }
            npos = m->pos - Vec(0, springCutoff, 0);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }
            npos = m->pos - Vec(0, 0, springCutoff);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }


            npos = m->pos - Vec(springCutoff, springCutoff, 0);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }
            npos = m->pos - Vec(springCutoff, 0, springCutoff);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }
            npos = m->pos - Vec(0, springCutoff, springCutoff);
            if (massesByPos.count(npos) > 0) {
                n = massesByPos[npos];
                sim->createSpring(m, n);
            }


            n1pos = m->pos - Vec(springCutoff, 0, 0);
            n2pos = m->pos - Vec(0, springCutoff, 0);
            if (massesByPos.count(n1pos) > 0 && massesByPos.count(n2pos) > 0) {
                n1 = massesByPos[n1pos];
                n2 = massesByPos[n2pos];
                sim->createSpring(n1, n2);
            }
            n1pos = m->pos - Vec(springCutoff, 0, 0);
            n2pos = m->pos - Vec(0, 0, springCutoff);
            if (massesByPos.count(n1pos) > 0 && massesByPos.count(n2pos) > 0) {
                n1 = massesByPos[n1pos];
                n2 = massesByPos[n2pos];
                sim->createSpring(n1, n2);
            }
            n1pos = m->pos - Vec(0, springCutoff, 0);
            n2pos = m->pos - Vec(0, 0, springCutoff);
            if (massesByPos.count(n1pos) > 0 && massesByPos.count(n2pos) > 0) {
                n1 = massesByPos[n1pos];
                n2 = massesByPos[n2pos];
                sim->createSpring(n1, n2);
            }
            }**/

    //sim->createPlane(Vec(0, 0, 1), 0);

    log(QString("Loaded simulation with %1 masses and %2 springs.")
        .arg(sim->masses.size())
        .arg(sim->springs.size()));
}

void Loader::loadSimFromLattice(LatticeConfig *lattice, Simulation *sim, double springCutoff) {

    // Create masses
    for (Vec v : lattice->vertices) {
        sim->createMass(v);
    }

    // Create springs
    for (int j = 0; j < sim->masses.size() - 1; j++) {
        for (int k = j+1; k < sim->masses.size(); k++) {

            Mass *massj = sim->masses[j];
            Mass *massk = sim->masses[k];
            double dist = (massj->pos - massk->pos).norm();

            if (dist <= springCutoff) {

                sim->createSpring(massj, massk);
                sim->springs.back()->setRestLength(dist);
            }
        }
    }

    log(QString("Loaded simulation with %1 masses and %2 springs.")
                .arg(sim->masses.size())
                .arg(sim->springs.size()));
}


void Loader::loadBarsFromSim(Simulation *sim, bar_data *output, bool crossSection, bool markers) {

    vector<Vec> springPos;
    vector<Vec> anchorPos = vector<Vec>();
    vector<Vec> forcePos = vector<Vec>();
    output->bars = vector<Bar>();

    for (Spring *s : sim->springs) {

        if (s->_k == 0) continue;

        springPos.push_back(s->_left->origpos);
        springPos.push_back(s->_right->origpos);

        if (s->_left->constraints.fixed) {
            anchorPos.push_back(s->_left->origpos);
        }
        if (s->_right->constraints.fixed) {
            anchorPos.push_back(s->_right->origpos);
        }

        if (!(s->_left->extforce == Vec(0, 0, 0))) {
            forcePos.push_back(s->_left->origpos);
        }
        if (!(s->_right->extforce == Vec(0, 0, 0))) {
            forcePos.push_back(s->_right->origpos);
        }
    }

    boundingBox<Vec> *bounds = new boundingBox<Vec>(springPos);

    double dist = FLT_MAX;
    Vec maxForcePos;
    for (const Vec f : forcePos) {
        if (dist > (f - bounds->maxCorner).norm()) {
            dist = (f - bounds->maxCorner).norm();
            maxForcePos = f;
        }
    }

    if (crossSection) {
        double xDim = bounds->maxCorner[0] - bounds->minCorner[0];
        bounds->maxCorner[0] = 0.5 * xDim + bounds->minCorner[0];
    }

    for (Spring *s : sim->springs) {

        if (s->_k == 0) continue;

        if (bounds->isWithin(s->_left->origpos) && bounds->isWithin(s->_right->origpos)) {

            output->addBar(s->_left->origpos, s->_right->origpos, 0.0002);
        }
        if (bounds->isWithin(s->_left->origpos) && !bounds->isWithin(s->_right->origpos)) {
            Vec triangle[3];
            triangle[0] = bounds->maxCorner;
            triangle[1] = Vec(bounds->maxCorner[0], 0, 0);
            triangle[2] = Vec(bounds->maxCorner[0], 0, 1);
            Vec p;
            double pu, pv;

            Vec r = (s->_right->origpos - s->_left->origpos).normalized();
            Utils::intersectPlane(triangle, s->_left->origpos, r, p, pu, pv);
            output->addBar(s->_left->origpos, p, 0.0002);
        }
        if (bounds->isWithin(s->_right->origpos) && !bounds->isWithin(s->_left->origpos)) {
            Vec triangle[3];
            triangle[0] = bounds->maxCorner;
            triangle[1] = Vec(bounds->maxCorner[0], 0, 0);
            triangle[2] = Vec(bounds->maxCorner[0], 0, 1);
            Vec p;
            double pu, pv;

            Vec r = (s->_left->origpos - s->_right->origpos).normalized();
            Utils::intersectPlane(triangle, s->_right->origpos, r, p, pu, pv);
            output->addBar(p, s->_right->origpos, 0.0002);
        }
    }

    //output->bounds = boundingBox<Vec>(springPos);
    output->bounds = *bounds;
    if (markers) {
        if (!anchorPos.empty()) {
            output->anchorShell = new boundingBox<Vec>(anchorPos);
            output->anchorShell->minCorner -= Vec(0.002, 0.002, 0.002);
            output->anchorShell->maxCorner += Vec(0.002, 0.002, 0.002);
            output->anchorShell->maxCorner[0] = output->anchorShell->minCorner[0] + 0.008;
            output->boxCutoff = 0;
            log(tr("Anchor structure bounds: (%1, %2, %3) by (%4, %5, %6)")
                        .arg(output->anchorShell->minCorner[0]).arg(output->anchorShell->minCorner[1]).arg(
                            output->anchorShell->minCorner[2])
                        .arg(output->anchorShell->maxCorner[0]).arg(output->anchorShell->maxCorner[1]).arg(
                            output->anchorShell->maxCorner[2]));
        }
        if (!forcePos.empty()) {
            output->forceSphere = new Vec(maxForcePos);
            log(tr("Applied forced position: (%1, %2, %3)").arg((*output->forceSphere)[0]).arg(
                    (*output->forceSphere)[1]).arg((*output->forceSphere)[2]));
        }
    }
    /**for (Bar b : output->bars) {
        b.left -= output->bounds.minCorner;
        b.right -= output->bounds.minCorner;
    }
    output->bounds.maxCorner -= output->bounds.minCorner;
    output->bounds.minCorner -= output->bounds.minCorner;**/
    log(tr("Bar structure bounds: (%1, %2, %3) by (%4, %5, %6)")
        .arg(output->bounds.minCorner[0]).arg(output->bounds.minCorner[1]).arg(output->bounds.minCorner[2])
            .arg(output->bounds.maxCorner[0]).arg(output->bounds.maxCorner[1]).arg(output->bounds.maxCorner[2]));

}

void Loader::applyLoadcase(Simulation *sim, Loadcase *load) {

    for (Anchor *anchor : load->anchors) {
        Volume *anchorVol = anchor->volume;
        anchor->masses.clear(); // Clear mass ptr cache

        int fixedMasses = 0;
        int nToFix = sim->masses.size();
        //if (anchor->type == "surface") { nToFix = surfacePoints; }
        
        for (int i = 0; i < nToFix; i++) {
            Mass* mass = sim->masses[i];
            if (anchorVol->model != nullptr) {
                glm::vec3 massPos = glm::vec3(mass->pos[0], mass->pos[1], mass->pos[2]);

                // Check for anchor constraint
                if (anchorVol->model->isInside(massPos, 0)) {

                    mass->fix();
                    anchor->masses.push_back(mass);
                    fixedMasses++;
                }
            } else {
                if (anchorVol->geometry->isInside(mass->pos)) {
                    mass->fix();
                    anchor->masses.push_back(mass);
                    fixedMasses++;
                }
            }
        }

        
        log(tr("Anchored %1 masses with volume '%2'").arg(fixedMasses).arg(anchorVol->id));
        cout << "Anchored " << fixedMasses << " masses with volume " << anchorVol->id.toStdString() << ".\n";
    }

    for (Force *force : load->forces) {
        Volume *forceVol = force->volume;
        qDebug() << "Applying" << force->magnitude[0] << force->magnitude[1] << force->magnitude[2];

        force->masses.clear(); // Clear mass ptr cache

        int forceMasses = 0;
        for (Mass *mass : sim->masses) {
            glm::vec3 massPos = glm::vec3(mass->pos[0], mass->pos[1], mass->pos[2]);

            // Check for force constraint
            if (forceVol->model != nullptr) {
                if (forceVol->model->isInside(massPos, 0)) {

                    mass->extduration += force->duration;

                    if (mass->extduration < 0) {
                        mass->extduration = DBL_MAX;
                    }

                    force->masses.push_back(mass);
                    forceMasses++;
                }

            } else {
                if (forceVol->geometry->isInside(mass->pos)) {

                    mass->extduration += force->duration;

                    if (mass->extduration < 0) {
                        mass->extduration = DBL_MAX;
                    }

                    force->masses.push_back(mass);
                    forceMasses++;
                }
            }
        }
        if (forceMasses > 0) {
            Vec distributedForce = force->magnitude / forceMasses;
            for (Mass *m : force->masses) {
                m->force += distributedForce;
                m->extforce += distributedForce;
            }
            log(tr("Applied %3 N force to %1 masses with volume '%2'").arg(forceMasses).arg(forceVol->id).arg(distributedForce.norm()));
            cout << "Applied " << distributedForce.norm() << "N force to " << forceMasses << " masses with volume " << forceVol->id.toStdString() << ".\n";
        } else
            log(tr("Applied force to %1 masses with volume '%2'").arg(forceMasses).arg(forceVol->id));
    }

    for (Actuation *actuation : load->actuations) {
        Volume *actVol = actuation->volume;

        int actSprings = 0;
        for (Spring *s : sim->springs) {
            glm::vec3 massPos1 = glm::vec3(s->_left->pos[0], s->_left->pos[1], s->_left->pos[2]);
            glm::vec3 massPos2 = glm::vec3(s->_right->pos[0], s->_right->pos[1], s->_right->pos[2]);

            if (actVol->model != nullptr) {
                glm::vec3 springMid = 0.5f * (massPos2 + massPos1);
                bool leftInside = actVol->model->isInside(massPos1, 0);
                bool rightInside = actVol->model->isInside(massPos2, 0);
                bool midInside = actVol->model->isInside(springMid, 0);
                if (midInside && (leftInside || rightInside)) {
                    actuation->springs.push_back(s);
                    actSprings++;
                }
            }
        }
        if (actSprings > 0) {
            int type = 3;
            switch(actuation->wave) {
                case Actuation::SIN:
                    type = 1;
                    break;
                case Actuation::EXPAND_SIN:
                    type = 4;
                    break;
                case Actuation::CONTRACT_SIN:
                    type = 5;
                    break;
                case Actuation::NONE:
                    type = 3;
                    break;
            }

            for (Spring *s : actuation->springs) {
                s->_type = type;
                s->_period = actuation->period;
                s->_offset = actuation->offset;
                s->_omega = actuation->omega;
            }
            cout << "Applied actuation " << type << " to " << actSprings << " springs with volume " << actVol->id.toStdString() << ".\n";
        }
    }

    //sim->masses.front()->force = Vec(100, 0, 0);
}

// Creates a lattice with a grid split dimX x dimY x dimZ
void Loader::createGridLattice(simulation_data *arrays, int dimX, int dimY, int dimZ) {
    float *grid;
    grid = new float(dimX * dimY * dimZ);
    glm::vec3 startCorner = arrays->bounds.minCorner;

    for (int i = 0; i <= dimZ; i++) {
        for (int j = 0; j <= dimY; j++) {
            for (int k = 0; k <= dimX; k++) {

            }
        }
    }
}

// Creates a lattice with a grid based on a cutoff edge length
void Loader::createGridLattice(simulation_data *arrays, SimulationConfig *simConfig) {
    log("Creating grid lattice.");

    vector<glm::vec3> grid = vector<glm::vec3>();
    vector<LatticeConfig *> latticeConfigs = simConfig->lattices;
    vector<LatticeConfig *> pointOrigins = vector<LatticeConfig *>();

    for (LatticeConfig *latticeBox : latticeConfigs) {
        model_data *latticeVol = latticeBox->volume->model;
        float cutoff = float(latticeBox->unit[0]);
        vector<glm::vec3> gridTemp = vector<glm::vec3>();

        glm::vec3 startCorner = latticeVol->bounds.minCorner;
        glm::vec3 endCorner = latticeVol->bounds.maxCorner;
        glm::vec3 gridPoint;

        vector<float> xLines = vector<float>();
        vector<float> yLines = vector<float>();
        vector<float> zLines = vector<float>();

        float endDiff;

        qDebug() << "Bounds top corner " << endCorner.x << "," << endCorner.y << "," << endCorner.z;

        for (float x = startCorner.x; x <= endCorner.x; x += cutoff) {
            xLines.push_back(x);
        }
        endDiff = endCorner.x - xLines.back();

        for (int i = 0; i < xLines.size(); i++) {
            xLines[i] += 0.5f * endDiff;
        }


        for (float y = startCorner.y; y <= endCorner.y; y += cutoff) {
            yLines.push_back(y);
        }

        endDiff = endCorner.y - yLines.back();
        for (int i = 0; i < yLines.size(); i++) {
            yLines[i] += 0.5f * endDiff;
        }


        for (float z = startCorner.z; z <= endCorner.z; z += cutoff) {
            zLines.push_back(z);
        }

        endDiff = endCorner.z - zLines.back();
        for (int i = 0; i < zLines.size(); i++) {
            zLines[i] += 0.5f * endDiff;
        }

        qDebug() << "Lattice bottom corner " << xLines[0] << "," << yLines[0] << "," << zLines[0];
        qDebug() << "Lattice top corner " << xLines.back() << "," << yLines.back() << "," << zLines.back();

        vector <glm::vec3> model = vector<glm::vec3>();

        // Populate grid and check inside
        for (ulong z = 0; z < zLines.size(); z++) {
            for (ulong y = 0; y < yLines.size(); y++) {
                for (ulong x = 0; x < xLines.size(); x++) {
                    gridPoint = glm::vec3(xLines[x], yLines[y], zLines[z]);

                    if (arrays->isInside(gridPoint) && latticeVol->isInside(gridPoint, 0)) {
                        // Add to lattice
                        gridTemp.push_back(gridPoint);
                        pointOrigins.push_back(latticeBox);
                    }
                }
            }
        }

        qDebug() << "Found all points in lattice" << latticeBox->volume->id;
        grid.insert(grid.end(), gridTemp.begin(), gridTemp.end());
    }

    // Set lattice property
    arrays->lattice = grid;
    arrays->pointOrigins = pointOrigins;
    qDebug() << "Created grid lattice" << arrays->lattice.size();
}

// Creates a lattice with a grid based on a cutoff edge length
void Loader::createGridLattice(Polygon *geometryBound, LatticeConfig &lattice, float cutoff) {
    log("Creating grid lattice.");

    vector<Vec> grid = vector<Vec>();

    Vec startCorner, endCorner;
    geometryBound->boundingPoints(startCorner, endCorner);

    vector<float> xLines = vector<float>();
    vector<float> yLines = vector<float>();
    vector<float> zLines = vector<float>();

    Vec gridPoint;
    float endDiff;

    for (float x = startCorner[0]; x <= endCorner[0]; x += cutoff) {
        xLines.push_back(x);
    }
    endDiff = endCorner[0] - xLines.back();

    for (int i = 0; i < xLines.size(); i++) {
        xLines[i] += 0.5f * endDiff;
    }


    for (float y = startCorner[1]; y <= endCorner[1]; y += cutoff) {
        yLines.push_back(y);
    }

    endDiff = endCorner[1] - yLines.back();
    for (int i = 0; i < yLines.size(); i++) {
        yLines[i] += 0.5f * endDiff;
    }


    for (float z = startCorner[2]; z <= endCorner[2]; z += cutoff) {
        zLines.push_back(z);
    }

    endDiff = endCorner[2] - zLines.back();
    for (int i = 0; i < zLines.size(); i++) {
        zLines[i] += 0.5f * endDiff;
    }

    qDebug() << "Lattice bottom corner " << xLines[0] << "," << yLines[0] << "," << zLines[0];
    qDebug() << "Lattice top corner " << xLines.back() << "," << yLines.back() << "," << zLines.back();

    // Populate grid and check inside
    for (ulong z = 0; z < zLines.size(); z++) {
        for (ulong y = 0; y < yLines.size(); y++) {
            for (ulong x = 0; x < xLines.size(); x++) {
                gridPoint = Vec(xLines[x], yLines[y], zLines[z]);

                if (geometryBound->isInside(gridPoint)) {
                    // Add to lattice
                    grid.push_back(gridPoint);
                }
            }
        }
    }

    // Set lattice property
    lattice.vertices = grid;
    qDebug() << "Created grid lattice" << lattice.vertices.size();
}

// Creates a lattice with random pseudo-evenly-spacedd interal points
void Loader::createSpaceLattice(simulation_data *arrays, SimulationConfig *simConfig) {
    log("Creating space lattice.");

    bool includeHull = simConfig->lattices[0]->hull;

    vector <LatticeConfig *> latticeConfigs = simConfig->lattices;

    vector<glm::vec3> lattice = vector<glm::vec3>();
    vector<LatticeConfig *> pointOrigins = vector<LatticeConfig *>();
    surfacePoints = 0;
    
    if (includeHull) {
		
        // Add hull vertices
        for (uint i = 0; i < arrays->vertices.size(); i++) {
            bool existsInLattice = false;

            for (uint l = 0; l < lattice.size(); l++) {
                if (arrays->vertices[i] == lattice[l]) {
                    existsInLattice = true;
                    arrays->hull.push_back(l);
                }
            }
            if (!existsInLattice) {
                arrays->hull.push_back(lattice.size());
                lattice.push_back(arrays->vertices[i]);
				surfacePoints++;
                for(LatticeConfig *latticeBox : latticeConfigs) {
                    if (latticeBox->volume->model->isInside(arrays->vertices[i], 0)) {
                        pointOrigins.push_back(latticeBox);
                        break;
                    }
                }
                // Just in case the hull point wasn't in any of the lattice boxes
                if (lattice.size() != pointOrigins.size())
                    pointOrigins.push_back(latticeConfigs.at(0));
            }
        }
        // Interpolate edges based on cutoff
    }

    for(auto&& latticeBox: latticeConfigs) {


        vector<glm::vec3> latticeTemp = vector<glm::vec3>();
        float cutoff = float(latticeBox->unit[0]);

        // TODO: simplify STL hull according to cutoff

        model_data *latticeVol = latticeBox->volume->model;

        glm::vec3 startCorner = latticeVol->bounds.minCorner;
        glm::vec3 endCorner = latticeVol->bounds.maxCorner;

        qDebug() << "Start corner" << startCorner.x << startCorner.y << startCorner.z;
        qDebug() << "End corner" << endCorner.x << endCorner.y << endCorner.z;

        // Get point estimate to calculate k
        int xLines = int((endCorner.x - startCorner.x) / cutoff) + 1;
        int yLines = int((endCorner.y - startCorner.y) / cutoff) + 1;
        int zLines = int((endCorner.z - startCorner.z) / cutoff) + 1;
        int kNewPoints = xLines * yLines * zLines;
        kNewPoints *= 3;

        qDebug() << "Generating" << kNewPoints << "random point candidates with cutoff" << cutoff;

        glm::vec3 point = Utils::randPoint(startCorner, endCorner);


        if (includeHull) {
            while (arrays->isCloseToEdge(point, cutoff) || !arrays->isInside(point) || !latticeVol->isInside(point, 0)) {

                // Generate a new point if its within the cutoff of the model edge
                //   or its not inside the model
                point = Utils::randPoint(startCorner, endCorner);
            }
        } else {
            while (!arrays->isInside(point) || !latticeVol->isInside(point, 0)) {
                // Generate a new point if its within the cutoff of the model edge
                //   or its not inside the model
                point = Utils::randPoint(startCorner, endCorner);
            }
        }
        // Add point to lattice
        latticeTemp.push_back(point);
        pointOrigins.push_back(latticeBox);
        float maxLength = cutoff;
        int k;
        qDebug() << "First point" << point.x << point.y << point.z;

        vector<glm::vec3> candidates =  vector<glm::vec3>(kNewPoints);

    	time_t tinit = time(0);
		#pragma omp parallel for
        for (k = 0; k < kNewPoints; k++) {

                glm::vec3 newPoint = Utils::randPoint(startCorner, endCorner);

                if (includeHull) {
                    while (arrays->isCloseToEdge(newPoint, cutoff) || !arrays->isInside(newPoint) || !latticeVol->isInside(newPoint, 0)) {
                        // Generate a new point if its within the cutoff of the model edge
                        //   or its not inside the model
                        newPoint = Utils::randPoint(startCorner, endCorner);
                    }
                } else {

                    while (!arrays->isInside(newPoint) || !latticeVol->isInside(newPoint, 0)) {
                        // Generate a new point if its within the cutoff of the model edge
                        //   or its not inside the model
                        newPoint = Utils::randPoint(startCorner, endCorner);
                    }
                }
            candidates[k] = newPoint;
        }

        vector<float> sumDistsStore = vector<float>();
        for (auto c: candidates) {
            sumDistsStore.push_back(0.0f);
        }


        int latticePrintFrequency = 1000;
        time_t tstart,tend;
        tstart = time(0);

        qDebug() << "maxL" << maxLength << " cutoff " << cutoff << " size " << candidates.size();
        while (maxLength >= cutoff && candidates.size() > 0) {
            // Find point furthest from existing points
        	// THIS IS THE ORIGINAL LOOP DONT DELETE IT
        	/*
        	  // Find point furthest from existing points
        	            uint iFarthest = 0;
        	            float maxDistFromPoints = 0.0f;


        	            for (uint i = 0; i < candidates.size(); i++) {
        	                bool reject = false;

        	                float sumDists = 0.0f;
        	                float distFromPoint;

        	                vec3 l = latticeTemp.back();
        	                distFromPoint = length(candidates[i] - l);

        	                if (distFromPoint < cutoff) {
        	                    candidates.erase(candidates.begin() + i);
        	                    sumDistsStore.erase(sumDistsStore.begin() + i);
        	                    i--;
        	                    continue;
        	                }
        	                assert(sumDistsStore.size() == candidates.size());
        	                sumDistsStore[i] += distFromPoint;

        	                if (sumDistsStore[i] > maxDistFromPoints) {
        	                    maxDistFromPoints = sumDistsStore[i];
        	                    iFarthest = i;
        	                }
        	            }
*/

            uint iFarthest = 0;
            float maxDistFromPoints = 0.0f;
/*
            vec3 l = latticeTemp.back();
            float distFromPoint;
            int nToRemove = 0;
            int candidateSize = candidates.size();
            qDebug() << "Candidates: " << candidateSize;
            for (uint i = 0; i < candidateSize; i++) {
                if (candidateSize-nToRemove > i) { /*do nothing}
                else {
                    distFromPoint = length(candidates[i] - l);
                	if (distFromPoint < cutoff) {
                		nToRemove++;
                		candidates[i] = candidates[candidates.size()-nToRemove];
                		sumDistsStore[i] = sumDistsStore[sumDistsStore.size()-nToRemove];
						i--;
                	}
                	else {
						sumDistsStore[i] += distFromPoint;
						if (sumDistsStore[i] > maxDistFromPoints) {
							maxDistFromPoints = sumDistsStore[i];
							iFarthest = i;
						}
                	}
                }
            }

            candidates.erase(candidates.end()-nToRemove,candidates.end());
            sumDistsStore.erase(sumDistsStore.end()-nToRemove,sumDistsStore.end());
            assert(sumDistsStore.size() == candidates.size());
*/
            vec3 l = latticeTemp.back();
            for (uint i = 0; i < candidates.size(); i++) {
                float distFromPoint = length(candidates[i] - l);

                if (distFromPoint < cutoff) {
                    candidates.erase(candidates.begin() + i);
                    sumDistsStore.erase(sumDistsStore.begin() + i);
                    i--;
                }
                else {
                    assert(sumDistsStore.size() == candidates.size());
                    sumDistsStore[i] += distFromPoint;

                    if (sumDistsStore[i] > maxDistFromPoints) {
                            maxDistFromPoints = sumDistsStore[i];
                            iFarthest = i;
                    }

                    }
            }

            if (candidates.size() > 0) {
                maxLength = maxDistFromPoints;
                // Update maxLength to minimum distance between
                //   an existing point and the point chosen

				#pragma omp parallel for
                for (int i = 0; i < latticeTemp.size(); i++) {
                    maxLength = std::min(maxLength, length(candidates[iFarthest] - latticeTemp[i]));
                }
                assert(iFarthest <= candidates.size());
                // Add point to lattice
                latticeTemp.push_back(candidates[iFarthest]);
                pointOrigins.push_back(latticeBox);
                candidates.erase(candidates.begin() + iFarthest);
                sumDistsStore.erase(sumDistsStore.begin() + iFarthest);
            }

            if (latticePrintFrequency == 0) {
            	tend = time(0);
                qDebug() << "Added to lattice" << latticeTemp.size() << " Time: " << difftime(tend, tstart) << " second(s)";
                latticePrintFrequency = 1000;
                tstart = time(0);
            }
            latticePrintFrequency--;
        }

        // PROBLEM HERE
        lattice.insert(lattice.end(), latticeTemp.begin(), latticeTemp.end());
        // Set lattice property
        qDebug() << "Found all points in lattice" << latticeBox->volume->id ;
        qDebug() << "Total time: " << difftime(tend, tinit) << " second(s)";
    }

    arrays->lattice = lattice;
    arrays->pointOrigins = pointOrigins;
    qDebug() << "Set lattice";
}


// Creates a lattice with random pseudo-evenly-spacedd interal points
void Loader::createSpaceLattice(Polygon *geometryBound, LatticeConfig &lattice, float cutoff, bool includeHull) {
    log("Creating space lattice.");

    vector<Vec> space = vector<Vec>();

    // TODO: simplify STL hull according to cutoff
    Vec startCorner, endCorner;
    geometryBound->boundingPoints(startCorner, endCorner);
    qDebug() << "Start corner" << startCorner[0] << startCorner[0] << startCorner[0];
    qDebug() << "End corner" << endCorner[1] << endCorner[1] << endCorner[1];

    // Get point estimate to calculate k
    int xLines = int((endCorner[0] - startCorner[0]) / cutoff) + 1;
    int yLines = int((endCorner[1] - startCorner[1]) / cutoff) + 1;
    int zLines = int((endCorner[2] - startCorner[2]) / cutoff) + 1;
    int kNewPoints = xLines * yLines * zLines;
    kNewPoints *= 3;

    qDebug() << "Generating" << kNewPoints << "random point candidates with cutoff" << cutoff;

    Vec point = Utils::randPointVec(startCorner, endCorner);

    if (includeHull) {

        // Add hull vertices
        for (auto n : geometryBound->nodeMap) {
            bool existsInLattice = false;

            for (uint l = 0; l < space.size(); l++) {
                if (n.first == space[l]) {
                    existsInLattice = true;
                }
            }
            if (!existsInLattice) {
                space.push_back(n.first);
            }
            // save number of hull points 
        }
        // Interpolate edges based on cutoff
        while (geometryBound->isCloseToEdge(point, cutoff) || !geometryBound->isInside(point)) {

            // Generate a new point if its within the cutoff of the model edge
            //   or its not inside the model
            point = Utils::randPointVec(startCorner, endCorner);

        }

    } else {

        while (!geometryBound->isInside(point)) {

            // Generate a new point if its within the cutoff of the model edge
            //   or its not inside the model
            point = Utils::randPointVec(startCorner, endCorner);
        }
    }


    // Add point to lattice
    space.push_back(point);
    float maxLength = cutoff;
    int k;

    vector<Vec> candidates = vector<Vec>();

    // Spawn k new points
    int threads = 64;


#pragma omp parallel for
    for (int t = 0; t < threads; t++) {
        for (k = 0; k < kNewPoints/threads; k++) {

            Vec newPoint = Utils::randPointVec(startCorner, endCorner);

            if (includeHull) {
                while (geometryBound->isCloseToEdge(newPoint, cutoff) || !geometryBound->isInside(newPoint)) {
                    // Generate a new point if its within the cutoff of the model edge
                    //   or its not inside the model
                    newPoint = Utils::randPointVec(startCorner, endCorner);
                }
            } else {

                while (!geometryBound->isInside(newPoint)) {
                    // Generate a new point if its within the cutoff of the model edge
                    //   or its not inside the model
                    newPoint = Utils::randPointVec(startCorner, endCorner);
                }
            }

    #pragma omp critical
            candidates.push_back(newPoint);
        }

        qDebug() << "Lattice Thread" << t << "done";
    }

int latticePrintFrequency = 100; 

    while (maxLength >= cutoff && !candidates.empty()) {

        // Find point furthest from original point
        uint iFarthest = 0;
        float maxDistFromPoints = 0.0f;
        vector<float> sumDistsStore(candidates.size());

		#pragma omp parallel for
        for (size_t i = 0; i < sumDistsStore.size(); ++i) {
            sumDistsStore[i]= maxDistFromPoints;
        }

        vector<int> candidatesToErase();
        Vec l_temp = space.back();

		#pragma omp parallel for
        for (uint i = 0; i < candidates.size(); i++) {
        	// this is defined but never used?

            if (space.size() > 0) {
                float distFromPoint = (candidates[i] - l_temp).norm();

                if (distFromPoint < cutoff ) {
					#pragma omp critical
                   candidates.erase(candidates.begin() + i);
                    i--;
                    continue;
                }
                sumDistsStore[i] += distFromPoint;
            }

            if (sumDistsStore[i] > maxDistFromPoints) {
                maxDistFromPoints = sumDistsStore[i];
                iFarthest = i;
            }
        }

        if (!candidates.empty()) {
            maxLength = maxDistFromPoints;
            // Update maxLength to minimum distance between
            //   an existing point and the point chosen
			#pragma omp parallel for
            for (size_t i = 0; i <space.size(); ++i) {
                maxLength = std::min(maxLength, float((candidates[iFarthest] - space[i]).norm()));
            }
            // Add point to lattice
            space.push_back(candidates[iFarthest]);
            candidates.erase(candidates.begin() + iFarthest);
        }
        if (latticePrintFrequency == 0) {
   	        qDebug() << "Added to lattice" << space.size();
 	       latticePrintFrequency = 100;
        }
        latticePrintFrequency--;
    }

    // Set lattice property
    qDebug() << "Found all points";
    lattice.vertices = space;
    qDebug() << "Set lattice";
}

 // Calculates the largest natural period (smallest natural frequency) of the lattice
 double Loader::calculateNaturalPeriod(Simulation *sim) {


     int n_masses = sim->masses.size();

     int mass_ind[n_masses] = {};

     vector<Eigen::Triplet<double>> m_vals;
     m_vals.reserve(n_masses);

     vector<int> forceIndices;
     forceIndices.reserve(n_masses/50); // A rough guess that about 2% of masses will have a force applied

     int n = 0;

     for (int i = 0; i < n_masses; i++) {
         Mass *m_temp = sim->getMassByIndex(i);

         if (m_temp->constraints.fixed) {
             mass_ind[i] = -1;
         } else {
             mass_ind[i] = n;

             for (int j = 0; j < 3; j++)
                m_vals.push_back(Eigen::Triplet<double>(3*n+j,3*n+j,m_temp->m));

             Vec force = m_temp->extforce;
             if (!(force == Vec(0,0,0))) {
                 for (int j = 0; j < 3; j++) {
                     if (force.data[j] != 0) {
                         forceIndices.push_back(3 * n + j);
                     }
                 }
             }
             n++;
         }
     }

     Eigen::SparseMatrix<double> m(3*n,3*n);
     m.setFromTriplets(m_vals.begin(), m_vals.end());

     Eigen::SparseVector<double> f = Eigen::SparseVector<double>(3*n);

     for (int i : forceIndices) {
         f.coeffRef(i) = 1.0;
     }

     vector<Eigen::Triplet<double>> k_vals;
     k_vals.reserve(n_masses*4);

     Vec dirs[] = {Vec(1,0,0), Vec(0,1,0), Vec(0,0,1)};

     for (Spring * spring: sim->springs) {
         double temp_k = spring->_k; // This is unfortunate, but I don't want to change the Titan library to add a get
         Vec springVec = (spring->_left->pos)-(spring->_right->pos);
         springVec = springVec/springVec.norm();
         int mass_i = mass_ind[spring->getLeft()];
         int mass_j = mass_ind[spring->getRight()];

         for (int c=0; c<3; c++) {
             for (int v=0; v<3; v++) {
                 if (mass_i >= 0) {

                     k_vals.push_back(Eigen::Triplet<double>(mass_i * 3 + c, mass_i * 3 + v,
                                                             temp_k * dot(springVec, dirs[c]) * dot(springVec, dirs[v])));
                 }

                 if (mass_j >= 0) {
                     k_vals.push_back(Eigen::Triplet<double>(mass_j * 3 + c, mass_j * 3 + v,
                                                             temp_k * dot(springVec, dirs[c]) * dot(springVec, dirs[v])));
                 }

                 // add the negative stiffness to the off diagonals connecting the two masses
                 if (mass_i >= 0 && mass_j >= 0) {
                     k_vals.push_back(Eigen::Triplet<double>(mass_i * 3 + c, mass_j * 3 + v,
                             -temp_k * dot(springVec, dirs[c]) * dot(springVec, dirs[v])));
                     k_vals.push_back(Eigen::Triplet<double>(mass_j * 3 + c, mass_i * 3 + v,
                             -temp_k * dot(springVec, dirs[c]) * dot(springVec, dirs[v])));
                 }
             }
         }
     }

     Eigen::SparseMatrix<double> k(3*n,3*n);
     k.setFromTriplets(k_vals.begin(), k_vals.end());

     // Set up the generalized eigenvalue solver
     Spectra::SparseSymMatProd<double> op(m);
     Spectra::SparseCholesky<double> Bop(k);

     /* Creates the eigenvalue solver object
      * This is the class to use, since our m matrix is symmetric (obv. since it's diagonal)
      * and our k matrix is positive definite (because of theories too complicated for me to understand)
      *
      * We are solving this problem backwards. Normally, to find the natural frequencies squared, we would solve
      * Kx=hMx, with h being the eigenvalues. However, we want to find the smallest natural frequencies
      * (i.e. the largest period oscillations), so we want the largest 1/h. Since the solver finds larger eigenvalues
      * faster, this actually works in our favor. So we solve Mx=zKx, where z = 1/h. Taking the square root of the result
      * gives us our natural periods.
      */
     int numEig = 3;
     Spectra::SymGEigsSolver<double, Spectra::LARGEST_MAGN, Spectra::SparseSymMatProd<double>,
             Spectra::SparseCholesky<double>, Spectra::GEIGS_CHOLESKY>
             geigs(&op, &Bop, numEig, numEig*5);

     geigs.init();
     int nconv = geigs.compute();

     Eigen::VectorXd evalues;
     Eigen::MatrixXd evecs;

     if(geigs.info() == Spectra::SUCCESSFUL)
     {
         evalues = geigs.eigenvalues();
         evecs = geigs.eigenvectors();
     } else {
         qDebug() << "Eigenvalues not found";
         return 0;
     }

     qDebug() << "Max Natural Periods" << sqrt(evalues[0])*2*3.1416 << sqrt(evalues[1])*2*3.1416 << sqrt(evalues[2])*2*3.1416;

     float epsilon = 0.1; // If next nat. period squared is 10x as excited as first, (nat. period is 100x), go to next
     int principalEig = 0;
     for (int i = 0; i < numEig-1; i++) {
         float relModalForce = fabs(((evecs.col(i).transpose()*f)/(evecs.col(i+1).transpose()*f))(0,0));

         qDebug() << "Modal Force" << i << ":" << (evecs.col(i).transpose()*f);
         qDebug() << "Relative Modal Force" << relModalForce;

         if (relModalForce < epsilon)
             principalEig = i+1;
         else
             break;
     }

     qDebug() << "Generalized eigenvalues found";

     return sqrt(evalues[principalEig])*2*3.1416;
 }


void Loader::suggestParams(Simulation *sim, SimulationConfig *simConfig) {
    double natPer = calculateNaturalPeriod(sim);

    glm::vec3 minCorner = simConfig->volume->model->bounds.minCorner;
    glm::vec3 maxCorner = simConfig->volume->model->bounds.maxCorner;

    // A very rough approximation of the longest path
    glm::vec3 edges = maxCorner-minCorner;

    double maxDist = 0;

    for (int i=0; i<3; i++) {
        maxDist += glm::abs(edges[i]);
    }

    float minSpringDist = std::numeric_limits<float>::max();

    for (LatticeConfig *lattice : simConfig->lattices) {
        if (lattice->unit[0] < minSpringDist)
            minSpringDist = lattice->unit[0];
    }

    int maxPath = maxDist/minSpringDist;

    log(tr("Render update time should be no less that %1 times GPU update time.\n"
           "Lattice should be given %2 seconds to settle before changing forces.").arg(maxPath).arg(natPer));
}
