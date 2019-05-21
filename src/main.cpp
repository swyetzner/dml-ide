#include <QApplication>
#include "commandLine.h"
#include "window.h"


int main(int argc, char *argv[])
{
    CommandLine::parse(argc, argv);
    string dmlInput = CommandLine::inputPath.Get();
    qDebug() << "input" << QString::fromStdString(dmlInput);

    if (dmlInput.empty()) {
        QApplication a(argc, argv);
        Window w;
        w.setWindowTitle("Design Model Language IDE");
        w.show();
        return a.exec();
    } else {
        QApplication a(argc, argv);
        return a.exec();
    }
}
