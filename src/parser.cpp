//
// Created by sw3390 on 7/23/19.
//

#include "parser.h"

#include <iostream>

// Loads DML into the PugiXML DOM Parser
//---------------------------------------------------------------------------
void Parser::loadDML(std::string filename) {
//---------------------------------------------------------------------------

    const char * filename_char = filename.c_str();
    pugi::xml_parse_result result = doc.load_file(filename_char);

    if (result) {
        std::cout << "DML [" << filename << "] format parsed successfully.\n";
    } else {
        std::cout << "DML [" << filename << "] parsed with errors.\n";
        std::cout << "Error description: " << result.description() << "\n";
        std::cout << "Error offset: " << result.offset << "\n\n";
    }
    filepath = filename;

}

// Parses Design model from DML DOM
//---------------------------------------------------------------------------
void Parser::parseDesign(Design *design) {
//---------------------------------------------------------------------------

    auto root = doc.child("dml");
    // VOLUMES
    for (pugi::xml_node vol : root.children("volume")) {
        Volume * volume = new Volume();
        parseVolume(vol, volume);

        volume->index = design->volumes.size();
        design->volumes.push_back(volume);
        design->volumeMap[volume->id] = volume;
        std::cout << "Volume (" << volume->index << ") '" << volume->id.toStdString() << "' PARSED\n";
    }
    // MATERIALS
    for (pugi::xml_node mat : root.children("material")) {
        Material * material = new Material();

        parseMaterial(mat, material);

        material->index = design->materials.size();
        design->materials.push_back(*material);
        design->materialMap[material->id] = material;
        std::cout << "Material (" << material->index << ") '" << material->id.toStdString() << "' PARSED\n";
    }
    // LOADCASES
    for (pugi::xml_node load : root.children("loadcase")) {
        Loadcase *loadcase = new Loadcase();

        parseLoadcase(load, loadcase, design);

        loadcase->index = design->loadcases.size();
        design->loadcases.push_back(loadcase);
        design->loadcaseMap[loadcase->id] = loadcase;
        std::cout << "Loadcase (" << loadcase->index << ") '" << loadcase->id.toStdString() << "' PARSED\n";
    }
    // SIMULATIONS
    for (pugi::xml_node sim : root.children("simulation")) {
        SimulationConfig *simConfig = new SimulationConfig();

        parseSimulation(sim, simConfig, design);

        simConfig->index = design->simConfigs.size();
        design->simConfigs.push_back(*simConfig);
        design->simConfigMap[simConfig->id] = simConfig;
        std::cout << "Simulation (" << simConfig->index << ") '" << simConfig->id.toStdString() << "' PARSED\n";
    }
    // OPTIMIZATION
    {
        OptimizationConfig *optConfig = new OptimizationConfig();

        parseOptimization(root.child("optimization"), optConfig, design);

        design->optConfig = optConfig;
        std::cout << "Optimization '" << optConfig->simulationConfig->id.toStdString() << "' PARSED\n";
    }
    // OUTPUTS
    for (pugi::xml_node out : root.children("output")) {
        output_data *outputData = new output_data();

        parseOutput(out, outputData, design);

        design->outputs.push_back(outputData);
        design->outputMap[outputData->id] = outputData;
        std::cout << "Output '" << outputData->id.toStdString() << "' PARSED\n";
    }
}

// Parses Volume model from DML DOM
//---------------------------------------------------------------------------
void Parser::parseVolume(pugi::xml_node dml_vol, Volume *volume) {
//---------------------------------------------------------------------------
    QString id = dml_vol.attribute("id").value();
    QString primitive = dml_vol.attribute("primitive").value();
    QString url = dml_vol.attribute("url").value();
    QString units = dml_vol.attribute("units").value();
    double alpha = dml_vol.attribute("alpha").as_double(1.0);
    std::string colorstr = dml_vol.attribute("color").value();
    Vec color = colorstr.empty()? Vec(0.5, 0.5, 0.5): parseVec(colorstr);

    volume->id = id;
    volume->primitive = primitive;
    QString dirpath = QString::fromStdString(filepath);
    dirpath.truncate(dirpath.lastIndexOf("/"));
    volume->url = QUrl(dirpath + "/" + url);
    volume->units = units;
    volume->color = QVector4D(color[0], color[1], color[2], alpha);
}

// Parses Material model from DML DOM
//---------------------------------------------------------------------------
void Parser::parseMaterial(pugi::xml_node dml_mat, Material *material) {
//---------------------------------------------------------------------------
    QString id = dml_mat.attribute("id").value();
    QString name = dml_mat.attribute("name").value();
    QString elaststr = dml_mat.attribute("elasticity").value();
    QString yieldstr = dml_mat.attribute("yield").value();
    QString denstr = dml_mat.attribute("density").value();

    material->id = id;
    material->name = name;
    material->elasticity = elaststr.split(" ")[0].toDouble();
    material->eUnits = elaststr.split(" ").size() > 1 ? elaststr.split(" ")[1] : QString();
    material->yield = yieldstr.split(" ")[0].toDouble();
    material->yUnits = yieldstr.split(" ").size() > 1 ? yieldstr.split(" ")[1] : QString();
    material->density = denstr.split(" ")[0].toDouble();
    material->dUnits = denstr.split(" ").size() > 1 ? denstr.split(" ")[1] : QString();
}


// Parses Loadcase model from DML DOM with Design maps needed
//---------------------------------------------------------------------------
void Parser::parseLoadcase(pugi::xml_node dml_load, Loadcase *loadcase, Design *design) {
//---------------------------------------------------------------------------
    QString id = dml_load.attribute("id").value();
    // ANCHORS
    for (pugi::xml_node anc : dml_load.children("anchor")) {
        Anchor *anchor = new Anchor();
        QString volume = anc.attribute("volume").value();
        anchor->volume = design->volumeMap[volume];
        anchor->type = anc.attribute("type").value();
        
        if (!(anchor->volume)) {
            cerr << "Volume '" << volume.toStdString() << "' not found.";
            exit(EXIT_FAILURE);
        }

        loadcase->anchors.push_back(anchor);
        loadcase->anchorMap[volume] = anchor;
        std::cout << "\tAnchor '" << volume.toStdString() << "' PARSED\n";
    }
    // FORCES
    for (pugi::xml_node frc : dml_load.children("force")) {
        Force *force = new Force();
        QString volume = frc.attribute("volume").value();
        Vec magnitude = parseVec(frc.attribute("magnitude").value());
        double duration = frc.attribute("duration").as_double(-1);
        Vec vary = parseVec(frc.attribute("vary").value());

        force->volume = design->volumeMap[volume];
        if (!(force->volume)) {
            cerr << "Volume '" << volume.toStdString() << "' not found.";
            exit(EXIT_FAILURE);
        }
        force->magnitude = magnitude;
        force->duration = duration;
        force->vary = vary;
        loadcase->forces.push_back(force);
        loadcase->forceMap[volume] = force;
        std::cout << "\tForce '" << volume.toStdString() << "' PARSED\n";
    }
    // TORQUES
    for (pugi::xml_node frc : dml_load.children("torque")) {
        Torque *torque = new Torque();
        QString volume = frc.attribute("volume").value();
        Vec magnitude = parseVec(frc.attribute("magnitude").value());
        double duration = frc.attribute("duration").as_double(-1);
        Vec vary = parseVec(frc.attribute("vary").value());
        Vec origin = parseVec(frc.attribute("origin").value());

        torque->volume = design->volumeMap[volume];
        if (!(torque->volume)) {
            cerr << "Volume '" << volume.toStdString() << "' not found.";
            exit(EXIT_FAILURE);
        }
        torque->magnitude = magnitude;
        torque->duration = duration;
        torque->vary = vary;
        torque->origin = origin;

        loadcase->torques.push_back(torque);
        loadcase->torqueMap[volume] = torque;
        std::cout << "\tTorque '" << volume.toStdString() << "' PARSED\n";
    }
    
    loadcase->id = id;
    loadcase->totalDuration = 0;
}

// Parses SimulationConfig model from DML DOM with Design maps needed
//---------------------------------------------------------------------------
void Parser::parseSimulation(pugi::xml_node dml_sim, SimulationConfig *simConfig, Design *design) {
//---------------------------------------------------------------------------
    QString id = dml_sim.attribute("id").value();
    QString volume = dml_sim.attribute("volume").value();

    // LATTICE
    LatticeConfig *lattice = new LatticeConfig();
    auto dml_lat = dml_sim.child("lattice");
    QString fill = dml_lat.attribute("fill").value();
    Vec unit = parseVec(dml_lat.attribute("unit").value());
    Vec bardiam = parseVec(dml_lat.attribute("bardiam").value());
    QString material = dml_lat.attribute("material").value();
    bool hull = dml_lat.attribute("hull").as_bool(true);
    QString structure = dml_lat.attribute("structure").value();
    QString latvol = dml_lat.attribute("volume").value();

    if (fill == "cubic")
        lattice->fill = LatticeConfig::CUBIC_FILL;
    else if (fill == "space")
        lattice->fill = LatticeConfig::SPACE_FILL;
    else
        lattice->fill = LatticeConfig::CUBIC_FILL;
    lattice->unit = unit;
    lattice->barDiameter = bardiam;
    lattice->material = design->materialMap[material];
    lattice->hull = hull;
    if (structure == "bars")
        lattice->structure = LatticeConfig::BARS;
    else
        lattice->structure = LatticeConfig::FULL;

    if (!(lattice->material)) {
        cerr << "Material '" << material.toStdString() << "' not found.\n";
        exit(EXIT_FAILURE);
    }

    // Damping
    Damping damping;
    auto dml_damp = dml_sim.child("damping");
    double velocity = dml_damp.attribute("velocity").as_double(0);
    damping.velocity = velocity;

    // Global
    Global global;
    auto dml_glo = dml_sim.child("global");
    Vec acceleration = parseVec(dml_glo.attribute("acceleration").value());
    global.acceleration = acceleration;

    // Repeat
    Repeat repeat;
    auto dml_rep = dml_sim.child("repeat");
    QString after = dml_rep.attribute("after").value();
    QString rotation = dml_rep.attribute("reotation").value();
    if (after.isEmpty()) {
        repeat.after = -1;
        repeat.afterExplicit = true;
    } else if (after == "optimize") {
        repeat.afterExplicit = false;
    } else {
        repeat.after = after.toDouble();
        repeat.afterExplicit = true;
    }
    if (rotation.isEmpty()) {
        repeat.rotation = Vec(0,0,0);
        repeat.rotationExplicit = true;
    } else if (rotation == "random") {
        repeat.rotationExplicit = false;
    } else {
        repeat.rotation = parseVec(rotation.toStdString());
        repeat.rotationExplicit = true;
    }

    // Plane
    Plane *plane;
    auto dml_pla = dml_sim.child("plane");
    if (dml_pla.empty()) {
        plane = nullptr;
    } else {
        plane = new Plane();
        Vec normal = parseVec(dml_pla.attribute("normal").value());
        double offset = dml_pla.attribute("offset").as_double(0);
        plane->normal = normal;
        plane->offset = offset;
    }

    // Load
    auto dml_load = dml_sim.child("load");
    QString lid = dml_load.attribute("id").value();
    QString lqueue = dml_load.attribute("queue").value();

    Loadcase *load = nullptr;
    vector<Loadcase *> queue = vector<Loadcase *>();
    if (!dml_load.empty()) {
        if (!lqueue.isEmpty()) {
            for (const auto &lqid : lqueue.split(",")) {
                Loadcase *l = design->loadcaseMap[lqid.trimmed()];
                if (!l) {
                    cerr << "Loadcase '" << lqid.trimmed().toStdString() << "' not found for <simulation><queue>.\n";
                    exit(EXIT_FAILURE);
                }
                queue.push_back(l);
            }
        }
        if (!lid.isEmpty()) {
            load = design->loadcaseMap[lid];
            if (!load) {
                cerr << "Loadcase '" << lid.toStdString() << "' not found for <simulation><load>.\n";
                exit(EXIT_FAILURE);
            }
        }
    }

    simConfig->id = id;
    simConfig->volume = design->volumeMap[volume];
    if (!(simConfig->volume)) {
        cerr << "Volume '" << volume.toStdString() << "' not found for <simulation><volume>.\n";
        exit(EXIT_FAILURE);
    }

    lattice->volume = latvol.isEmpty() ? simConfig->volume : design->volumeMap[latvol];
    if (!(lattice->volume)) {
        cerr << "Volume '" << latvol.toStdString() << "' not found.\n";
        exit(EXIT_FAILURE);
    }

    simConfig->lattices.push_back(lattice);
    simConfig->latticeMap[lattice->volume->id] = lattice;
    simConfig->damping = damping;
    simConfig->global = global;
    simConfig->repeat = repeat;
    simConfig->plane = plane;
    simConfig->load = load;
    simConfig->loadQueue = queue;
}


// Parses OptimizationConfig model from DML DOM with Design maps needed
//---------------------------------------------------------------------------
void Parser::parseOptimization(pugi::xml_node dml_opt, OptimizationConfig *optConfig, Design *design) {
//---------------------------------------------------------------------------
    QString sid = dml_opt.attribute("simulation").value();

    // RULES
    for (pugi::xml_node dml_rul : dml_opt.children("rule")) {
        OptimizationRule rule = OptimizationRule();
        QString method = dml_rul.attribute("method").value();
        QString threshold = dml_rul.attribute("threshold").value();
        int frequency = dml_rul.attribute("frequency").as_int(0);
        QString regenRate = dml_rul.attribute("regenRate").value();
        QString regenThreshold = dml_rul.attribute("regenThreshold").value();
        double memory = dml_rul.attribute("memory").as_double(1);

        if (method == "remove_low_stress") {
            rule.method = OptimizationRule::REMOVE_LOW_STRESS;
        } else if (method == "mass_displace") {
            rule.method = OptimizationRule::MASS_DISPLACE;
        } else {
            rule.method = OptimizationRule::NONE;
        }
        if (threshold.endsWith('%')) {
            rule.threshold = threshold.split('%')[0].trimmed().toDouble() / 100;
        } else {
            rule.threshold = threshold.toDouble();
        }
        if (!regenRate.isEmpty()) {
            if (regenRate.endsWith('%')) {
                rule.regenRate = regenRate.split('%')[0].trimmed().toDouble() / 100;
            } else {
                rule.regenRate = regenRate.toDouble();
            }
        }
        if (!regenThreshold.isEmpty()) {
            if (regenThreshold.endsWith('%')) {
                rule.regenThreshold = regenThreshold.split('%')[0].trimmed().toDouble() / 100;
            } else {
                rule.regenThreshold = regenThreshold.toDouble();
            }
        }
        rule.frequency = frequency;
        rule.memory = memory;
        optConfig->rules.push_back(rule);
        std::cout << "\tOptimization Rule " << rule.methodName().toStdString() << " PARSED\n";
    }

    // STOPS
    for (pugi::xml_node dml_sto : dml_opt.children("stop")) {
        OptimizationStop stop = OptimizationStop();
        QString metric = dml_sto.attribute("metric").value();
        QString threshold = dml_sto.attribute("threshold").value();

        if (metric == "weight") {
            stop.metric = OptimizationStop::WEIGHT;
        } else if (metric == "energy") {
            stop.metric = OptimizationStop::ENERGY;
        } else if (metric == "deflection") {
            stop.metric = OptimizationStop::DEFLECTION;
        } else if (metric == "iterations") {
            stop.metric = OptimizationStop::ITERATIONS;
        } else {
            stop.metric = OptimizationStop::NONE;
        }
        if (threshold.endsWith('%')) {
            stop.threshold = threshold.split('%')[0].trimmed().toDouble() / 100;
        } else {
            stop.threshold = threshold.toDouble();
        }
        optConfig->stopCriteria.push_back(stop);
        std::cout << "\tOptimization Stop " << stop.metricName().toStdString() << " PARSED\n";
    }

    optConfig->simulationConfig = design->simConfigMap[sid];
    if (!(optConfig->simulationConfig)) {
        cerr << "Simulation '" << sid.toStdString() << "' not found.\n";
        exit(EXIT_FAILURE);
    }
}

// Parses output data from DML DOM with Design maps needed
//---------------------------------------------------------------------------
void Parser::parseOutput(pugi::xml_node dml_out, output_data *outputData, Design *design) {
//---------------------------------------------------------------------------
    QString id = dml_out.attribute("id").value();
    QString sim = dml_out.attribute("simulation").value();

    // INCLUDES
    for (pugi::xml_node dml_inc : dml_out.children("include")) {
        QString vid = dml_inc.attribute("volume").value();

        Volume *v = design->volumeMap[vid];
        if (!v) {
            cerr << "Volume '" << vid.toStdString() << "' not found in <output><include>.\n";
            exit(EXIT_FAILURE);
        }
        outputData->includes.push_back(v);
    }
    // EXCLUDES
    for (pugi::xml_node dml_inc : dml_out.children("exclude")) {
        QString vid = dml_inc.attribute("volume").value();

        Volume *v = design->volumeMap[vid];
        if (!v) {
            cerr << "Volume '" << vid.toStdString() << "' not found in <output><exclude>.\n";
            exit(EXIT_FAILURE);
        }
        outputData->excludes.push_back(v);
    }

    outputData->id = id;
    outputData->sim = design->simConfigMap[sim];
    if (!outputData->sim) {
        cerr << "Simulation '" << sim.toStdString() << "' not found in <output>.\n";
        exit(EXIT_FAILURE);
    }
    design->simConfigMap[sim]->output = outputData;
}

// Helper function to parse string to Vec type
//---------------------------------------------------------------------------
Vec Parser::parseVec(const std::string& vecstring) {
//---------------------------------------------------------------------------

    double x, y, z;
    const char * vecConstChar;

    vecConstChar = vecstring.c_str();

    if (3 == sscanf(vecConstChar, "%lf,%lf,%lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }
    if (3 == sscanf(vecConstChar, "%lf, %lf, %lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }
    if (3 == sscanf(vecConstChar, "%lf %lf %lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }

    cerr << R"(Malformed DML: Expected text in the form "value, value, value" but got ")" << vecstring <<"\".\n";
    return Vec(0, 0, 0);

}