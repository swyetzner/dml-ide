//
// Created by sw3390 on 5/15/19.
//

#include "commandLine.h"
#include <iostream>

namespace CommandLine {
    args::ArgumentParser parser("DML simulation runner.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::Positional<std::string> inputPath(parser, "PATH", "Path to the input DML file");
    args::ValueFlag<double> gpuTimestep(parser, "SECONDS", "GPU timestep (controls simulation timestep)",
                                       {'t', "timestep"}, 1E-4);
    args::ValueFlag<double> renderTimestep(parser, "SECONDS", "Render timestep (controls video speed)",
                                          {'r', "render"}, 5E-3);
    args::ValueFlag<std::string> modelOutputPath(parser, "PATH", "Output 3D model file (STL supported)", {"model"});
    args::ValueFlag<std::string> videoOutputPath(parser, "PATH", "Output video of simulation", {"video"});

    // Parse command line
    void parse(int argc, char **argv) {
        try {
            parser.ParseCLI(argc, argv);
        } catch (args::Help) {
            std::cout << parser;
            exit(0);
        } catch(args::Error e) {
            std::cerr << e.what() << std::endl;
            std::cerr << parser;
            exit(1);
        }
    }
}