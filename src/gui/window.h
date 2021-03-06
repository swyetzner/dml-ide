#ifndef WINDOW_H
#define WINDOW_H

#include <QDesktopWidget>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QStyleFactory>
#include <QTextEdit>
#include <QTimer>
#include "gui/exportdialog.h"
#include "gui/dmlTree.h"
#include "io/exportThread.h"
#include "loader.h"
#include "model.h"
#include "polygonizer.h"
#include "gui/properties.h"

#ifdef USE_OpenGL
#include "opengl/designViewer.h"
#include "opengl/simViewer.h"
#endif // USE_OpenGL

#include "simulator.h"

#undef GRAPHICS
#include <Titan/sim.h>

#include <ctime>
#include <iomanip>

namespace Ui {
    class MainWindow;
}

class Window : public QMainWindow
{
    Q_OBJECT
public:
    explicit Window(QWidget *parent = nullptr);
    ~Window();

    static QTextEdit * s_textEdit;
    enum ViewMode { MODEL_VIEW, DESIGN_VIEW, SIM_VIEW };

    Design *design;
    void setUpDMLFeatures();

signals:
    void toggleVolume(int index);

    void displayVolumeProp(QString id);
    void displayMaterialProp(QString id);
    void displayLoadProp(QString id);
    void displaySimProp(QString id);
    void displayOptProp();

public slots:
    void log(const QString message);
    void open();
    void save();
    void load();
    void exportSTL();
    void saveSTLFile(double barDiam, double res);
    void modelItemToggled(QTreeWidgetItem * item, int column);
    void dmlItemClicked(QTreeWidgetItem * item, int column);

    double getTimestep();
    void setTimestep(double dt);
    double getRenderUpdate();
    void setRenderUpdate(double dt);
    double getSpringConst();
    void setSpringConst(double k);
    int getVisualizeScheme();
    bool getShowText();
    void reloadSimulation();

private slots:
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionLoad_triggered();

    void on_actionwireframeView_triggered();
    void on_actionSTLView_triggered();
    void on_actionDMLView_triggered();

    void on_actionSimulation_Mode_toggled(bool toggled);
    void on_actionStepSim_triggered();
    void updateTimeLCD(double time);
    void on_actionStartSim_triggered();
    void on_actionStopSim_triggered();
    void on_actionPauseSim_triggered();

    void on_actionRecordSim_toggled(bool toggled);
    void on_actionSaveSim_triggered();
    void on_actionSTLExport_triggered();

    void simulationFinished();
    void exportThreadFinished(QString fileName);


private:
    Simulation *simulation;

    Ui::MainWindow *ui;
    DMLTree *dmlTreeWidget;
    PropertiesTable *propTable;
    Simulator *simulator;
    Loader *loader;

    #ifdef USE_OpenGL
    DesignViewer *designWidget;
    SimViewer *simWidget;
    #endif

    ExportThread exportThread;

    model_data *arrays;
    simulation_data *arrays_sim;
    bar_data *arrays_bars;

    QString *inputDMLPath;
    QVector<QString> outputSTLPaths;

    // Default values
    const double defaultDt = 0.001;
    const double defaultRenderPeriod = 0.005;
    const double defaultK = 10000;

    void setUpSimulationOptions();
    void setUpPropertyTable();
};

#endif // WINDOW_H
