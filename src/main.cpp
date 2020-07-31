#include <QApplication>
#include <QSurfaceFormat>
#include "io/commandLine.h"
#include "parser.h"
#include "gui/window.h"

void qtNoDebugMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
        case QtDebugMsg:
            break;
        case QtInfoMsg:
            fprintf(stderr, "Qt Info: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtWarningMsg:
            fprintf(stderr, "Qt Warning: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtCriticalMsg:
            fprintf(stderr, "Qt Critical: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtFatalMsg:
            fprintf(stderr, "Qt Fatal: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            abort();
    }
}

void loadNoGraphics(std::string input, double gstep, double rstep, std::string dpath, bool stlExport) {

    Design *design = new Design();
    Simulation *simulation = new Simulation();

    Parser *parser = new Parser();
    parser->loadDML(input);
    parser->parseDesign(design);
    delete parser;
    cout << "Parsing complete.\n\n";

    Loader *loader = new Loader();
    loader->loadDesignModels(design);
    loader->loadSimulation(simulation, &design->simConfigs[0]);
    cout << "Simulation springs: " << simulation->springs.size() << " masses: " << simulation->masses.size() << "\n";
    cout << "Loading complete.\n\n";

    qInstallMessageHandler(qtNoDebugMessageOutput);
    Simulator *simulator = new Simulator(simulation, loader, &design->simConfigs[0], design->optConfig, false, stlExport);
    simulator->setSimTimestep(gstep);
    simulator->setSyncTimestep(rstep);
    if (!dpath.empty()) simulator->setDataDir(dpath);
    while (simulator->simStatus != Simulator::STOPPED) {
        simulator->runSimulation(true);
    }
    delete loader;
    delete simulator;
    cout << "Simulation complete.\n\n";
}

int main(int argc, char *argv[]) {

    QSurfaceFormat format;
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(3,3);
    QSurfaceFormat::setDefaultFormat(format);

    CommandLine::parse(argc, argv);
    string dmlInput = CommandLine::inputPath.Get();

    if (!dmlInput.empty()) {

        bool graphics = CommandLine::graphicsUI;
        bool noExportSTL = CommandLine::noExportSTL? CommandLine::noExportSTL.Get() : false;
        double gpuTimestep = CommandLine::gpuTimestep? CommandLine::gpuTimestep.Get() : 1E-4;
        double renderTimestep = CommandLine::renderTimestep? CommandLine::renderTimestep.Get() : 5E-3;
        string outputDataPath = CommandLine::outputDataPath? CommandLine::outputDataPath.Get(): "";
        string outputModelPath = CommandLine::outputModelPath? CommandLine::outputModelPath.Get(): "";
        string outputVideoPath = CommandLine::outputVideoPath? CommandLine::outputVideoPath.Get(): "";

        if (!graphics) {

            cout << "\n\nLoading without a graphical user interface...\n\n";
            loadNoGraphics(dmlInput, gpuTimestep, renderTimestep, outputDataPath, !noExportSTL);

        }
        QApplication a(argc, argv);
        Window w;
        w.setWindowTitle("Design Model Language IDE");

        Parser *parser = new Parser();
        parser->loadDML(dmlInput);
        parser->parseDesign(w.design);
        delete parser;

        w.setUpDMLFeatures();
        w.show();
        return a.exec();

    } else {
        QApplication a(argc, argv);
        Window w;
        w.setWindowTitle("Design Model Language IDE");

        w.show();
        return a.exec();
    }

    return 1;
}
