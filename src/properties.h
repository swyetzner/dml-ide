#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <QObject>
#include <QWidget>
#include <QHeaderView>
#include <QTableWidget>
#include <QLabel>
#include "model.h"

class PropertiesTable : public QTableWidget
{
    Q_OBJECT

public:
    PropertiesTable(Design *design, QWidget *parent = nullptr);

    void displayDesign();

    void setLabel(QLabel *title);

public slots:
    void displayVolume(QString id);
    void displayMaterial(QString id);
    void displayLoadcase(QString id);
    void displaySimulation(QString id);

    void updateProp(int row, int col);

signals:
    void updateGraphics();
    void log(const QString message);

private:
    Design *design;
    QLabel *title;

    enum DisplayObject { DESIGN, VOLUME, MATERIAL, LOADCASE, SIMULATION, NONE };
    DisplayObject displayObject;
    ulong objectIndex;

    void createPropertyItem(int row, int col, QString name);
    void createValueItem(int row, int col, QString name);
    void createNodeItem(int row, int col, QString name);
    void createBlankItem(int row, int col);
    void createColorValueItem(int row, int col, QVector4D colorVec);
    void createVecValueItem(int row, int col, Vec vec);

    Vec parseVecInput(QString vecString);
};


class PropertiesModel : QAbstractTableModel
{
    Q_OBJECT

public:
    PropertiesModel(QObject *parent = nullptr);

    enum DisplayObject { DESIGN, VOLUME, LOADCASE, SIMULATION, NONE };
    DisplayObject displayObject;

    void setDesign(Design *design);

    // --------------------------------------------------------------------
    // TABLE MODEL OVERRIDES
    // --------------------------------------------------------------------
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    //QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;


private:
    Design *design;
};


#endif // PROPERTIES_H
