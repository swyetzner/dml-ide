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
        loadVolumeModel(&design->volumes[v]);
        design->volumes[v].model->createGraphicsData();
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

    volume->model->model_indices = new int[1];
    volume->model->colors = vector<vec4>();
    volume->model->n_models = 1;

    vec4 color = vec4(volume->color.x(),
                      volume->color.y(),
                      volume->color.z(),
                      volume->color.w());
    volume->model->colors.push_back(color);

    if (volume->primitive == "stl") {
        if (volume->url.isEmpty())
            log("Empty URL for volume " + volume->id + ". Cannot load model");

        if (volume->url.isValid()) {

            Utils::createModelFromFile(volume->url.path().toStdString(), volume->model->vertices, volume->model->normals);


        } else {
            log("Invalid URL: \'" + volume->url.url() + "\' for volume " + volume->id + ". cannot load model");
        }
    } else if (volume->primitive == "cube") {

        Utils::createCube(vec3(0.0, 0.03, 0.0), 0.1f, volume->model->vertices, volume->model->normals);

    }

    volume->model->n_vertices = int(volume->model->vertices.size());
    volume->model->n_normals = int(volume->model->normals.size());
    volume->model->model_indices[0] = volume->model->n_vertices;
    volume->model->bounds = boundingBox<vec3>(volume->model->vertices);

    log(QString("Loaded %1 vertices").arg(volume->model->n_vertices));
}

void Loader::loadSimModel(SimulationConfig *simConfig) {
    Volume *simVolume = simConfig->volume;

    simConfig->model = new simulation_data(simVolume->model);
}


void Loader::loadVolumes(model_data *arrays, Design *design) {
    arrays->model_indices = new int[design->volumes.size()];
    arrays->colors = vector<glm::vec4>(design->volumes.size());
    arrays->n_models = design->volumes.size();
    for (uint v = 0; v < design->volumes.size(); v++) {
       // loadModel(arrays, &design->volumes.at(v), v);
    }
    arrays->n_models = 1;
    loadModel(arrays, &design->volumes.at(0), 0);

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

    // Reload sim volume
    loadSimModel(simConfig);

    if (simConfig->model->lattice.size() == 0) {
        switch (simConfig->lattice.fill) {
            case LatticeConfig::CUBIC_FILL:
                // TODO: change to full unit input
                createGridLattice(simConfig->model, float(simConfig->lattice.unit[0]));
            break;
            case LatticeConfig::SPACE_FILL:
                // TODO: add conform bool as includeHull
                createSpaceLattice(simConfig->model, float(simConfig->lattice.unit[0]), true);
            break;
        }
    }

    double springMult = 2.9;
    if (simConfig->lattice.fill == LatticeConfig::CUBIC_FILL) springMult = 1.9;
    loadSimFromLattice(simConfig->model, sim, simConfig->lattice.unit[0]*springMult);


    // PLANE
    if (simConfig->plane != nullptr) {
        sim->createPlane(simConfig->plane->normal, simConfig->plane->offset);
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
        s->_diam = simConfig->lattice.barDiameter[1];
    }


    // LOADCASE
    if (simConfig->load != nullptr) {
        applyLoadcase(sim, simConfig->load);
    }


    // MATERIAL
    if (simConfig->lattice.material != nullptr) {
        Material *mat = simConfig->lattice.material;

        // TODO: get correct bardiam dimension
        double maxK = 0;
        for (Spring *s : sim->springs) {

            // ELASTICITY -- SPRING CONSTANT
            if (mat->eUnits != nullptr) {
                double pi = atan(1.0)*4;
                double a = pi * (simConfig->lattice.barDiameter[1] / 2) * (simConfig->lattice.barDiameter[1] / 2);
                double unit = 1;

                if (mat->eUnits == "GPa") { unit *= 1000 * 1000 * 1000; }
                if (mat->eUnits == "MPa") { unit *= 1000 * 1000; }

                double k = mat->elasticity * unit * a / s->_rest;
                s->_k = k;
                maxK = std::max(k, maxK);
            }

        }
        // TIMESTEP
        //double timestep = std::min(1/pow(10, to_string(int(maxK)).length()-2), 0.0001);
        sim->setAllDeltaTValues(1e-4);

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
            double v = pow(simConfig->lattice.unit[0],3);
            double d = mat->density;
            double unit = 1;

            if (mat->dUnits == "gcc") { unit *= 1000; }
            qDebug() << v*d*unit;
            sim->setAllMassValues(v*d*unit);
            qDebug() << sim->masses.front()->m;
        }

        //sim->masses.front()->extforce = Vec(-100, 0, 0);
        //sim->masses.front()->extduration = 0.1;
    }
}

void Loader::loadSimulation(simulation_data *arrays, Simulation *sim, uint n_volume) {

    // Create graphics data
    arrays->createGraphicsData();

    // Already existing triangle edges/springs
    // Stored in form key=(mass m), value=(masses connected to mass m)
    vector<vector<uint>> edges = vector<vector<uint>>(arrays->n_vertices, vector<uint>());

    // Create masses at vertices
    int modelStart = 0;
    int modelEnd = arrays->model_indices[n_volume];
    if (n_volume != 0)
        modelStart = arrays->model_indices[n_volume - 1];

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


void Loader::loadSimFromLattice(simulation_data *arrays, Simulation *sim, double springCutoff) {

    // Load hull first
    for (ulong i = 0; i < arrays->lattice.size(); i++) {
        sim->createMass(Vec(arrays->lattice.at(i).x,
                            arrays->lattice.at(i).y ,
                            arrays->lattice.at(i).z));
        //sim->getMassByIndex(i)->damping = 0.9999;
    }

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

void Loader::loadBarsFromSim(Simulation *sim, bar_data *output, bool crossSection, bool markers) {

    vector<Vec> springPos;
    vector<Vec> anchorPos = vector<Vec>();
    vector<Vec> forcePos = vector<Vec>();
    output->bars = vector<Bar>();

    for (Spring *s : sim->springs) {
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

    for (const Anchor &anchor : load->anchors) {
        Volume *anchorVol = anchor.volume;

        for (Mass *mass : sim->masses) {
            glm::vec3 massPos = glm::vec3(mass->pos[0], mass->pos[1], mass->pos[2]);

            // Check for anchor constraint
            if (anchorVol->model->isInside(massPos, 0)) {

                mass->fix();
            }
        }
    }

    for (const Force &force : load->forces) {
        Volume *forceVol = force.volume;

        for (Mass *mass : sim->masses) {
            glm::vec3 massPos = glm::vec3(mass->pos[0], mass->pos[1], mass->pos[2]);

            // Check for force constraint
            if (forceVol->model->isInside(massPos, 0)) {

                mass->force += force.magnitude;
                mass->extforce += force.magnitude;
                mass->extduration += force.duration;


                if (mass->extduration < 0) {
                    mass->extduration = DBL_MAX;
                }
            }
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
void Loader::createGridLattice(simulation_data *arrays, float cutoff) {
    log("Creating grid lattice.");

    vector<glm::vec3> grid = vector<glm::vec3>();
    glm::vec3 startCorner = arrays->bounds.minCorner;
    glm::vec3 endCorner = arrays->bounds.maxCorner;
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

    vector<glm::vec3> model = vector<glm::vec3>();

    // Populate grid and check inside
    for (ulong z = 0; z < zLines.size(); z++) {
        for (ulong y = 0; y < yLines.size(); y++) {
            for (ulong x = 0; x < xLines.size(); x++) {
                gridPoint = glm::vec3(xLines[x], yLines[y], zLines[z]);

                if (arrays->isInside(gridPoint, 0)) {
                    // Add to lattice
                    grid.push_back(gridPoint);
                }
            }
        }
    }

    // Set lattice property
    arrays->lattice = grid;
}

// Creates a lattice with random pseudo-evenly-spacedd interal points
void Loader::createSpaceLattice(simulation_data *arrays, float cutoff, bool includeHull) {
    log("Creating space lattice.");

    vector<glm::vec3> lattice = vector<glm::vec3>();

    // TODO: simplify STL hull according to cutoff
    glm::vec3 startCorner = arrays->bounds.minCorner;
    glm::vec3 endCorner = arrays->bounds.maxCorner;
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
            }
        }
        // Interpolate edges based on cutoff


        while (arrays->isCloseToEdge(point, cutoff, 0) || !arrays->isInside(point, 0)) {

            // Generate a new point if its within the cutoff of the model edge
            //   or its not inside the model
            point = Utils::randPoint(startCorner, endCorner);
        }

    } else {

        while (!arrays->isInside(point, 0)) {
            // Generate a new point if its within the cutoff of the model edge
            //   or its not inside the model
            point = Utils::randPoint(startCorner, endCorner);
        }
    }


    // Add point to lattice
    lattice.push_back(point);
    float maxLength = cutoff;
    int k;
    qDebug() << "First point" << point.x << point.y << point.z;

    vector<glm::vec3> candidates = vector<glm::vec3>();

    // Spawn k new points
    int threads = 64;

#pragma omp parallel for
    for (int t = 0; t < threads; t++) {
        for (k = 0; k < kNewPoints/threads; k++) {

            glm::vec3 newPoint = Utils::randPoint(startCorner, endCorner);

            if (includeHull) {
                while (arrays->isCloseToEdge(newPoint, cutoff, 0) || !arrays->isInside(newPoint, 0)) {
                    // Generate a new point if its within the cutoff of the model edge
                    //   or its not inside the model
                    newPoint = Utils::randPoint(startCorner, endCorner);
                }
            } else {

                while (!arrays->isInside(newPoint, 0)) {
                    // Generate a new point if its within the cutoff of the model edge
                    //   or its not inside the model
                    newPoint = Utils::randPoint(startCorner, endCorner);
                }
            }

#pragma omp critical
            candidates.push_back(newPoint);
        }

        qDebug() << "Lattice Thread" << t << "done";
    }


    while (maxLength >= cutoff && candidates.size() > 0) {

        // Find point furthest from original point
        uint iFarthest = 0;
        float maxDistFromPoints = 0.0f;
        vector<float> sumDistsStore = vector<float>();
        for (auto c: candidates) {
            sumDistsStore.push_back(0.0f);
        }

        for (uint i = 0; i < candidates.size(); i++) {
            bool reject = false;

            float sumDists = 0.0f;
            float distFromPoint;

            if (lattice.size() > 0) {
                vec3 l = lattice.back();
                distFromPoint = length(candidates[i] - l);

                if (distFromPoint < cutoff) {
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

        if (candidates.size() > 0) {
            maxLength = maxDistFromPoints;
            // Update maxLength to minimum distance between
            //   an existing point and the point chosen
            for (vec3 l : lattice)
                maxLength = std::min(maxLength, length(candidates[iFarthest] - l));

            // Add point to lattice
            lattice.push_back(candidates[iFarthest]);
            candidates.erase(candidates.begin() + iFarthest);
        }
        qDebug() << "Added to lattice" << lattice.size();
    }

    // Set lattice property
    qDebug() << "Found all points";
    arrays->lattice = lattice;
    qDebug() << "Set lattice";
}
