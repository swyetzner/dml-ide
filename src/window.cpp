#include "window.h"
#include "ui_window.h"

/** Tree toggle tags **/
const QString volumeTag = "Volume";
const QString materialTag = "Material";
const QString loadcaseTag = "Loadcase";
const QString simulationTag = "Simulation";

/** Console widget pointer **/
QTextEdit * Window::s_textEdit = nullptr;

/**
 * @brief Window::Window Main window constructor
 * @param parent
 */
Window::Window(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    propTable(nullptr),
    designWidget(nullptr),
    simWidget(nullptr)
{

    design = new Design();
    simulation = new Simulation();

    inputDMLPath = nullptr;

    s_textEdit = new QTextEdit(this);
    s_textEdit->setReadOnly(true);

    dmlTreeWidget = new DMLTree(design, this);
    propTable = new PropertiesTable(design, this);

    ui->setupUi(this);
    ui->dmlDock->setWidget(dmlTreeWidget);
    ui->consoleDock->setWidget(s_textEdit);
    ui->simSettingsBox->hide();

    arrays = new model_data();
    loader = new Loader(this);

    connect(dmlTreeWidget, &DMLTree::log, this, &Window::log);
    connect(loader, &Loader::log, this, &Window::log);
    connect(propTable, &PropertiesTable::log, this, &Window::log);

    connect(&exportThread, SIGNAL(exportedGeometry(string)), this, SLOT(exportThreadFinished(string)));

    ui->timeLCD->setSegmentStyle(QLCDNumber::Flat);
}

/**
 * @brief Window::~Window
 * Destructor
 */
Window::~Window() {
    delete ui;
    delete dmlTreeWidget;
    delete s_textEdit;
    delete arrays;
}


/**
 * @brief Window::log Outputs message to console widget
 * @param message
 */
void Window::log(const QString message) {
    s_textEdit->append(message);
}


/**
 * @brief Window::open Opens a file
 */
void Window::open() {
    QString fileName =
            QFileDialog::getOpenFileName(this, tr("Open DML File"),
                                         QDir::currentPath(),
                                         tr("DML Files (*.dml *.xml *.txt)"));
    if (fileName.isEmpty())
        return;

    QFile file(fileName);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {

        log(tr("Cannot read file %1:\n%2.")
            .arg(QDir::toNativeSeparators(fileName),
                 file.errorString()));
        return;
    }

    QString dirName = fileName;
    dirName.truncate(dirName.lastIndexOf("/"));

    qDebug() << "About to read DML file";
    if (dmlTreeWidget->read(&file, dirName))
        log(tr("Loaded file: %1.")
            .arg(QDir::toNativeSeparators(file.fileName())));
    qDebug() << "Read DML file";

    inputDMLPath = new QString(fileName);

    QString shortName = fileName;
    shortName = shortName.right(shortName.length() - shortName.lastIndexOf("/") - 1);
    setWindowTitle(shortName + " - " + windowTitle());

    setUpDMLFeatures();
}

void Window::save() {
    if (inputDMLPath != nullptr) {
        qDebug() << "Saving" << *inputDMLPath;
    }
}

void Window::exportSTL() {
    QString fileName =
            QFileDialog::getSaveFileName(this, tr("Save STL File"),
                                         QDir::currentPath());
    if (fileName.isEmpty())
        return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        log(tr("Cannot open file %1:\n%2.")
            .arg(QDir::toNativeSeparators(fileName),
                 file.errorString()));
        return;
    }


    if (arrays_bars == nullptr || arrays_bars->bars.size() == 0) {
        log(tr("Export Error: A simulation must be run and saved before it can be exported."));
        return;
    }
    /**Polygonizer polygonizer = Polygonizer(arrays_bars, 0.00075, 0.0015, 32);
    polygonizer.initBaseSegments();
    qDebug() << "Initialized Base Segments";
    log(tr("Exporting: %1 segments.").arg(polygonizer.segments.size()));
    log(tr("Exporting: %1 bars in segment 0").arg(polygonizer.segments[0].bars.size()));
    polygonizer.calculatePolygon();
    polygonizer.writePolygonToSTL(fileName.toStdString());**/

    exportThread.startExport(fileName.toStdString(), arrays_bars, 0.00075, 0.0015, 32);

    qDebug() << "Exporting STL file";
}

double Window::getTimestep() {
    return ui->timestep->text().toDouble();
}

void Window::setTimestep(double dt) {
    ui->timestep->setText(QString::number(dt));
}

double Window::getRenderUpdate() {
    return ui->renderUpdateEdit->text().toDouble();
}

void Window::setRenderUpdate(double dt) {
    ui->renderUpdateEdit->setText(QString::number(dt));
}

double Window::getSpringConst() {
    return ui->springConstEdit->text().toDouble();
}

void Window::setSpringConst(double k) {
    ui->springConstEdit->setText(QString::number(k));
}

bool Window::getShowStress() {
    return ui->stressCheckBox->isChecked();
}

void Window::reloadSimulation() {
    if (simWidget != nullptr)
        ui->verticalLayout->removeWidget(simWidget);

    simulation = new Simulation();
    //loader->loadSimFromLattice(arrays_sim, simulation, 0.035);
    //loader->applyLoadcase(arrays_sim, simulation, design->loadcaseMap["standing"]);

    loader->loadSimulation(simulation, &design->simConfigs[0]);

    simWidget = new Simulator(simulation, &design->simConfigs[0], design->optConfig, this);
    connect(simWidget, &Simulator::log, this, &Window::log);

    ui->verticalLayout->addWidget(simWidget);
    ui->propLayout->addWidget(ui->simSettingsBox);
    setUpSimulationOptions();

    //simulator = new SimulatorParallel(simulation, &design->simConfigs[0]);
}

void Window::setUpDMLFeatures() {
    loader->loadDesignModels(design);

    designWidget = new DesignViewer(design, this);
    ui->verticalLayout->addWidget(designWidget);

    //loader->loadVolumes(arrays, design);
    //arrays_sim = design->simConfigs[0].model;

    //arrays->createGraphicsData();
    //qDebug() << "Model vertices count " << arrays->vertices.size();

    connect(dmlTreeWidget, &DMLTree::itemDoubleClicked, this, &Window::modelItemToggled);
    connect(this, &Window::toggleVolume, designWidget, &DesignViewer::toggleModel);

    connect(dmlTreeWidget, &DMLTree::itemClicked, this, &Window::dmlItemClicked);
    connect(this, &Window::displayVolumeProp, propTable, &PropertiesTable::displayVolume);
    connect(this, &Window::displayMaterialProp, propTable, &PropertiesTable::displayMaterial);
    connect(this, &Window::displayLoadProp, propTable, &PropertiesTable::displayLoadcase);
    connect(this, &Window::displaySimProp, propTable, &PropertiesTable::displaySimulation);
    connect(propTable, &PropertiesTable::updateGraphics, designWidget, &DesignViewer::updateColors);

    setUpPropertyTable();
    propTable->show();
    propTable->displayVolume(design->volumes.at(0).id);
}

/**
 * @brief Window::on_actionOpen_triggered
 */
void Window::on_actionOpen_triggered()
{
    open();
}


void Window::on_actionSTLExport_triggered()
{
    exportSTL();
}

void Window::on_actionSave_triggered()
{
    save();
}


void Window::exportThreadFinished(string fileName) {
    log(tr("Completed export of %1").arg(QString::fromStdString(fileName)));
}

void Window::on_actionwireframeView_triggered()
{

    if (designWidget != nullptr) {
        designWidget->renderWireframe();
    }
}

void Window::on_actionSTLView_triggered()
{

    if (designWidget != nullptr) {
        designWidget->renderSolid();
    }
}

void Window::on_actionDMLView_triggered()
{
    if (designWidget != nullptr) {
        designWidget->renderDesign();
    }
}

void Window::on_actionSimulation_Mode_toggled(bool toggled)
{
    if (designWidget != nullptr) {
        if (toggled) {
            designWidget->hide();

            //if (arrays_sim->lattice.size() == 0)
                //loader->createGridLattice(arrays_sim, 0.02f);
                //loader->createSpaceLattice(arrays_sim, 0.01f, false);

            reloadSimulation();

            simWidget->show();
            ui->actionDMLView->setVisible(false);
            ui->actionwireframeView->setVisible(false);
            ui->actionSTLView->setVisible(false);
            ui->actionRecordSim->setVisible(true);
            ui->actionSaveSim->setVisible(true);

        } else {
            simWidget->hide();
            designWidget->show();
            ui->propLayout->removeWidget(ui->simSettingsBox);
            ui->actionDMLView->setVisible(true);
            ui->actionwireframeView->setVisible(true);
            ui->actionSTLView->setVisible(true);
            ui->actionRecordSim->setVisible(false);
            ui->actionSaveSim->setVisible(false);
        }
    }
}

void Window::updateTimeLCD(double time)
{
    ui->timeLCD->setDigitCount(6);
    ui->timeLCD->display(time);
}


void Window::on_actionStepSim_triggered()
{
    simWidget->step();
}

void Window::on_actionStartSim_triggered()
{
    simWidget->start();
}

void Window::on_actionStopSim_triggered()
{
    simWidget->stop();
}


void Window::on_actionPauseSim_triggered()
{
    simWidget->pause();
}

void Window::on_actionRecordSim_toggled(bool toggled)
{
    if (toggled)
        simWidget->recordVideo();
    else
        simWidget->saveVideo();
}


void Window::on_actionSaveSim_triggered()
{
    arrays_bars = new bar_data();
    simWidget->prepare();
    loader->loadBarsFromSim(simulation, arrays_bars, false, false);
    log(tr("Saved %1 bars from simulation.").arg(arrays_bars->bars.size()));
    log(tr("X Bounds: (%1, %2)").arg(arrays_bars->bounds.minCorner[0]).arg(arrays_bars->bounds.maxCorner[0]));
}

void Window::setUpSimulationOptions() {
    ui->timestep->setAlignment(Qt::AlignRight);
    ui->renderUpdateEdit->setAlignment(Qt::AlignRight);
    ui->springConstEdit->setAlignment(Qt::AlignRight);
    ui->massValueEdit->setAlignment(Qt::AlignRight);

    updateTimeLCD(0.0);
    if (simulation->masses.size() > 0) {
        ui->timestep->setText(QString::number(simulation->masses.front()->dt));
    } else {
        ui->timestep->setText(QString::number(defaultDt));
    }
    if (simulation->springs.size() > 0) {
        ui->springConstEdit->setText(QString::number(simulation->springs.front()->_k));
    } else {
        ui->springConstEdit->setText(QString::number(defaultK));
    }
    if (simulation->masses.size() > 0) {
        ui->massValueEdit->setText(QString::number(simulation->masses.front()->m));
    } else {
        ui->massValueEdit->setText("0");
    }
    ui->renderUpdateEdit->setText(QString::number(defaultRenderPeriod));

    connect(simWidget, &Simulator::timeChange, this, &Window::updateTimeLCD);
    connect(simWidget, &Simulator::setTimestep, this, &Window::setTimestep);
    connect(simWidget, &Simulator::getTimestep, this, &Window::getTimestep);
    connect(simWidget, &Simulator::setRenderUpdate, this, &Window::setRenderUpdate);
    connect(simWidget, &Simulator::getRenderUpdate, this, &Window::getRenderUpdate);
    connect(simWidget, &Simulator::setSpringConst, this, &Window::setSpringConst);
    connect(simWidget, &Simulator::getSpringConst, this, &Window::getSpringConst);
    connect(simWidget, &Simulator::getShowStress, this, &Window::getShowStress);
    connect(simWidget, &Simulator::reloadSimulation, this, &Window::reloadSimulation);
}


void Window::setUpPropertyTable() {

    QLabel *propTitle = new QLabel();
    propTable->setLabel(propTitle);

    ui->propLayout->addWidget(propTitle);
    ui->propLayout->addWidget(propTable);
}

void Window::modelItemToggled(QTreeWidgetItem *item, int column) {
    Volume *vol;
    QString id = item->data(column, Qt::DisplayRole).toString();
    if (id.startsWith(volumeTag)) {
        // Get 'id' attribute for volume map
        vol = design->volumeMap[item->child(0)->text(1)];
        toggleVolume(int(vol->index));

        if (item->textColor(0) == QColor(150, 150, 150)) {
            item->setTextColor(0, QColor(0, 0, 0));
        } else {
            item->setTextColor(0, QColor(150, 150, 150));
        }
    }
}

void Window::dmlItemClicked(QTreeWidgetItem *item, int column) {

    QString id = item->data(column, Qt::DisplayRole).toString();
    if (id.startsWith(volumeTag)) {
        Volume *vol = design->volumeMap[item->child(0)->text(1)];
        displayVolumeProp(vol->id);
    } else if (id.startsWith(materialTag)) {
        Material *mat = design->materialMap[item->child(0)->text(1)];
        displayMaterialProp(mat->id);
    } else if (id.startsWith(loadcaseTag)) {
        Loadcase *load = design->loadcaseMap[item->child(0)->text(1)];
        displayLoadProp(load->id);
    } else if (id.startsWith(simulationTag)) {
        SimulationConfig *simConfig = design->simConfigMap[item->child(0)->text(1)];
        displaySimProp(simConfig->id);
    }
}