#ifndef DMLTREE_H
#define DMLTREE_H

#include <QDomDocument>
#include <QIcon>
#include <QString>
#include <QTreeWidget>
#include "model.h"

class DMLTree : public QTreeWidget
{
    Q_OBJECT

public:
    DMLTree(Design *design, QWidget *parent = nullptr);

    bool read(QIODevice *device, QString filePath);
    bool write(QIODevice *device) const;

signals:
    void log(const QString &message);

private:
    Design *design_ptr;
    QVector<Volume> *volumes_ptr;
    int n_volumes;
    QString filePath;

    void parseExpandElement(const QDomElement &element,
                            QTreeWidgetItem *parentItem = nullptr);
    QTreeWidgetItem *createItem(const QDomElement &element,
                                QTreeWidgetItem *parentItem = nullptr);
    QTreeWidgetItem *createAttributeItem(QTreeWidgetItem *parentItem,
                                         QDomNamedNodeMap attrMap, QString attrName);
    Vec parseVec(QString vecString);

    QDomDocument domDocument;
    QIcon expandIcon;
    // TODO: We can add some more icons here
};

#endif // DMLTREE_H
