#include "exportdialog.h"
#include "ui_exportdialog.h"

ExportDialog::ExportDialog(double simBarDiam, double defaultRes, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExportDialog),
    simBarDiam(simBarDiam),
    defaultResolution(defaultRes)
{
    ui->setupUi(this);
    this->setWindowTitle("Export to STL");

    // Set up default values
    ui->barDiameterLineEdit->setText(QString::number(simBarDiam));
    ui->resolutionLineEdit->setText(QString::number(defaultResolution));
}

ExportDialog::~ExportDialog()
{
    delete ui;
}

// Check box slot
void ExportDialog::on_useSimulationBarDiameterCheckBox_stateChanged(int checked)
{
    if (checked) {
        simulationBarDiam = true;
        ui->barDiameterLineEdit->setText(QString::number(simBarDiam));
        clipboardBarDiam = simBarDiam;

        ui->barDiameterLineEdit->setReadOnly(true);
    } else {
        simulationBarDiam = false;
        ui->barDiameterLineEdit->setReadOnly(false);
    }
}

// Bar diameter line edit slot
void ExportDialog::on_barDiameterLineEdit_textChanged(const QString &text)
{

    clipboardBarDiam = text.toDouble();;

}

// Resolution line edit slot
void ExportDialog::on_resolutionLineEdit_textChanged(const QString &text)
{

    clipboardResolution = text.toDouble();

}

// Accepted
void ExportDialog::on_buttonBox_accepted()
{

    emit dialogAccepted(clipboardBarDiam, clipboardResolution);
    delete this;

}

// Canceled
void ExportDialog::on_buttonBox_rejected()
{

    delete this;

}
