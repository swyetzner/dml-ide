#ifndef EXPORTDIALOG_H
#define EXPORTDIALOG_H

#include <QDialog>

namespace Ui {
class ExportDialog;
}

class ExportDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ExportDialog(double simBarDiam, double defaultRes, QWidget *parent = nullptr);
    ~ExportDialog();

private slots:
    void on_useSimulationBarDiameterCheckBox_stateChanged(int checked);

    void on_barDiameterLineEdit_textChanged(const QString &text);

    void on_resolutionLineEdit_textChanged(const QString &text);

    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

signals:
    void dialogAccepted(double barDiam, double res);

private:
    Ui::ExportDialog *ui;

    double simBarDiam;
    double defaultResolution;
    double clipboardBarDiam;
    double clipboardResolution;
    bool simulationBarDiam;
};

#endif // EXPORTDIALOG_H
