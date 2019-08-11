//
// Created by sw3390 on 7/23/19.
//

#ifndef DMLIDE_PARSER_H
#define DMLIDE_PARSER_H

#include "model.h"

#include "pugixml.hpp"

// Class to parse DML
class Parser {

public:
    void loadDML(std::string filename);
    void parseDesign(Design *design);

private:
    std::string filepath;
    pugi::xml_document doc;

    void parseVolume(pugi::xml_node dml_vol, Volume *volume);
    static void parseMaterial(pugi::xml_node dml_mat, Material *material);
    static void parseLoadcase(pugi::xml_node dml_load, Loadcase *loadcase, Design *design);
    static void parseSimulation(pugi::xml_node dml_sim, SimulationConfig *simConfig, Design *design);
    static void parseOptimization(pugi::xml_node dml_opt, OptimizationConfig *optConfig, Design *design);
    void parseOutput(pugi::xml_node dml_out, output_data *output, Design *design);

    static Vec parseVec(const std::string& vecstring);
};


#endif //DMLIDE_PARSER_H
