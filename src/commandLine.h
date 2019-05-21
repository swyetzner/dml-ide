//
// Created by sw3390 on 5/15/19.
//

#ifndef DMLIDE_COMMANDLINE_H
#define DMLIDE_COMMANDLINE_H

#include <args.hxx>

namespace CommandLine {
    extern args::ArgumentParser parser;
    extern args::HelpFlag help;
    extern args::Positional<std::string> inputPath;
    extern args::ValueFlag<double> gpuTimestep;
    extern args::ValueFlag<double> renderTimestep;
    extern args::ValueFlag<std::string> outputModelPath;
    extern args::ValueFlag<std::string> outputVideoPath;

    extern void parse(int argc, char **argv);
};


#endif //DMLIDE_COMMANDLINE_H
