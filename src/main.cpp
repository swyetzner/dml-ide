#include <QApplication>
#include "commandLine.h"
#include "window.h"


int main(int argc, char *argv[]) {
    CommandLine::parse(argc, argv);
    string dmlInput = CommandLine::inputPath.Get();

    QApplication a(argc, argv);
    Window w;
    w.setWindowTitle("Design Model Language IDE");

    if (!dmlInput.empty()) {

        double gpuTimestep = CommandLine::gpuTimestep? CommandLine::gpuTimestep.Get() : 1E-4;
        double renderTimestep = CommandLine::renderTimestep? CommandLine::renderTimestep.Get() : 5E-3;
        string outputModelPath = CommandLine::outputModelPath? CommandLine::outputModelPath.Get(): "";
        string outputVideoPath = CommandLine::outputVideoPath? CommandLine::outputVideoPath.Get(): "";


        w.loadFromCmdLine(QString::fromStdString(dmlInput),
                          gpuTimestep, renderTimestep,
                          QString::fromStdString(outputModelPath),
                          QString::fromStdString(outputVideoPath));
    }

    w.show();
    return a.exec();
}
