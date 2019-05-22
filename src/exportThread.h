//
// Created by sw3390 on 4/23/19.
//

#ifndef DMLIDE_EXPORTTHREAD_H
#define DMLIDE_EXPORTTHREAD_H

#include <QMutex>
#include <QThread>

#include "polygonizer.h"

class ExportThread : public QThread {

    Q_OBJECT

public:
    explicit ExportThread(QObject *parent = nullptr);
    ~ExportThread() override;

    void startExport(string fileName, bar_data *barModel, double resolution, double diameter, int threads);

signals:
    void exportedGeometry(QString fileName);

protected:
    void run() override;

private:
    bool abort;
    QMutex mutex;

    string fileName;
    Polygonizer *polygonizer;
};


#endif //DMLIDE_EXPORTTHREAD_H
