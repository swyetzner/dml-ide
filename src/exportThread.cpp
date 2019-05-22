//
// Created by sw3390 on 4/23/19.
//

#include "exportThread.h"


ExportThread::ExportThread(QObject *parent) : QThread(parent) {

    abort = false;

}

ExportThread::~ExportThread() {
    mutex.lock();
    abort = true;
    delete polygonizer;
    mutex.unlock();
    wait();
}

void ExportThread::startExport(string fileName, bar_data *barModel, double resolution, double diameter, int threads) {

    QMutexLocker locker(&mutex);
    this->polygonizer = new Polygonizer(barModel, resolution, diameter, threads);
    this->fileName = fileName;

    if (!isRunning()) {
        start(LowPriority);
    }
}

void ExportThread::run() {
    mutex.lock();
    string path = this->fileName;
    mutex.unlock();

    polygonizer->initBaseSegments();
    if (abort) return;
    polygonizer->calculatePolygon();
    if (abort) return;
    polygonizer->writePolygonToSTL(path);

    emit exportedGeometry(QString::fromStdString(path));
}
