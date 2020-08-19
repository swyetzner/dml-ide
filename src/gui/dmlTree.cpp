#include <QMessageBox>
#include <QTextStream>
#include <QHeaderView>
#include "dmlTree.h"

enum { DomElementRole = Qt::UserRole + 1 };

Q_DECLARE_METATYPE(QDomElement)

static inline QString volumeElement() { return QStringLiteral("volume"); }
static inline QString materialElement() { return QStringLiteral("material"); }
static inline QString loadcaseElement() { return QStringLiteral("loadcase"); }
static inline QString simulationElement() { return QStringLiteral("simulation"); }\
static inline QString optimizationElement() { return QStringLiteral("optimization"); }
static inline QString outputElement() { return QStringLiteral("output"); }

static inline QString versionAttribute() { return QStringLiteral("version"); }
static inline QString unitsAttribute() { return QStringLiteral("units"); }
static inline QString idAttribute() { return QStringLiteral("id"); }

// Volume attributes
static inline QString primitiveAttribute() { return QStringLiteral("primitive"); }
static inline QString urlAttribute() { return QStringLiteral("url"); }
static inline QString colorAttribute() { return QStringLiteral("color"); }
static inline QString alphaAttribute() { return QStringLiteral("alpha"); }
static inline QString renderingAttribute() { return QStringLiteral("rendering"); }

// Material attributes
static inline QString nameAttribute() { return QStringLiteral("name"); }
static inline QString elasticityAttribute() { return QStringLiteral("elasticity"); }
static inline QString yieldAttribute() { return QStringLiteral("yield"); }
static inline QString densityAttribute() { return QStringLiteral("density"); }
static inline QString expansionAttribute() { return QStringLiteral("expansion"); }

// Loadcase elements
static inline QString anchorElement() { return QStringLiteral("anchor"); }
static inline QString forceElement() { return QStringLiteral("force"); }
static inline QString torqueElement() { return QStringLiteral("torque"); }
static inline QString actuationElement() { return QStringLiteral("actuation"); }

// Anchor attributes
static inline QString volumeAttribute() { return QStringLiteral("volume"); }

// Force attributes
static inline QString magnitudeAttribute() {return QStringLiteral("magnitude"); }
static inline QString durationAttribute() { return QStringLiteral("duration"); }
static inline QString varyAttribute() { return QStringLiteral("vary"); }
static inline QString originAttribute() { return QStringLiteral("origin"); }

// Actuation attributes
static inline QString waveAttribute() {return QStringLiteral("wave");}
static inline QString periodAttribute() {return QStringLiteral("period");}
static inline QString omegaAttribute() {return QStringLiteral("omega");}

// Simulation elements
static inline QString latticeElement() {return QStringLiteral("lattice"); }
static inline QString dampingElement() {return QStringLiteral("damping"); }
static inline QString loadElement() {return QStringLiteral("load"); }
static inline QString stopElement() {return QStringLiteral("stop"); }
static inline QString repeatElement() {return QStringLiteral("repeat"); }
static inline QString planeElement() {return QStringLiteral("plane"); }
static inline QString globalElement() { return QStringLiteral("global"); }

// Lattice attributes
static inline QString fillAttribute() { return QStringLiteral("fill"); }
static inline QString unitAttribute() { return QStringLiteral("unit"); }
static inline QString displayAttribute() { return QStringLiteral("display"); }
static inline QString conformAttribute() { return QStringLiteral("conform"); }
static inline QString offsetAttribute() { return QStringLiteral("offset"); }
static inline QString bardiamAttribute() { return QStringLiteral("bardiam"); }
static inline QString materialAttribute() { return QStringLiteral("material"); }
static inline QString jiggleAttribute() { return QStringLiteral("jiggle"); }
static inline QString hullAttribute() { return QStringLiteral("hull"); }
static inline QString structureAttribute() { return QStringLiteral("structure"); }

// Damping attributes
static inline QString velocityAttribute() { return QStringLiteral("velocity"); }

// Global attributes
static inline QString accelerationAttribute() { return QStringLiteral("acceleration"); }

// Load attributes
static inline QString queueAttribute() { return QStringLiteral("queue"); }

// Stop attributes
static inline QString criterionAttribute() { return QStringLiteral("criterion"); }
static inline QString thresholdAttribute() { return QStringLiteral("threshold"); }
static inline QString metricAttribute() { return QStringLiteral("metric"); }

// Optimization attributes + elements
static inline QString volAttribute() { return QStringLiteral("vol"); }
static inline QString ruleElement() { return QStringLiteral("rule"); }
static inline QString constraintElement() { return QStringLiteral("constraint"); }

// Rule attributes
static inline QString methodAttribute() { return QStringLiteral("method"); }
static inline QString frequencyAttribute() { return QStringLiteral("frequency"); }
static inline QString memoryAttribute() { return QStringLiteral("memory"); }
static inline QString regenRateAttribute() { return QStringLiteral("regenRate"); }
static inline QString regenThresholdAttribute() { return QStringLiteral("regenThreshold"); }


// Repeat attributes
static inline QString afterAttribute() { return QStringLiteral("after"); }
static inline QString rotationAttribute() { return QStringLiteral("rotation"); }

// Plane attributes
static inline QString normalAttribute() { return QStringLiteral("normal"); }

// Output elements
static inline QString includeElement() { return QStringLiteral("include"); }
static inline QString excludeElement() { return QStringLiteral("exclude"); }

DMLTree::DMLTree(Design *design, QWidget *parent) : QTreeWidget(parent)
{
    design_ptr = design;
    n_volumes = 0;

    this->setColumnCount(2);

    QHeaderView *header = new QHeaderView(Qt::Orientation::Horizontal, this);
    this->setHeader(header);
    this->setHeaderHidden(true);
    this->setAlternatingRowColors(true);
    this->setExpandsOnDoubleClick(false);

    header->setSectionResizeMode(QHeaderView::ResizeToContents);
    header->setSectionsClickable(true);
    header->setStretchLastSection(true);
}

bool DMLTree::read(QIODevice *device, QString fp)
{
    filePath = fp;
    QString errorStr;
    int errorLine;
    int errorColumn;

    // Attempt to parse DOM structure
    if (!domDocument.setContent(device, true, &errorStr, &errorLine,
                                &errorColumn)) {
        QMessageBox::information(window(), tr("DML Reader"),
                                 tr("Parse error at line %1, column %2:\n%3")
                                 .arg(errorLine)
                                 .arg(errorColumn)
                                 .arg(errorStr));
        return false;
    }

    // Verify DML tag and version
    QDomElement root = domDocument.documentElement();
    if (root.tagName() != "dml") {
        QMessageBox::information(window(), tr("DML Reader"), tr("The file is not an DML file."));
        return false;
    } else if (root.hasAttribute(versionAttribute())
               && root.attribute(versionAttribute()) != QLatin1String("1.0")) {
        QMessageBox::information(window(), tr("DML Reader"),
                                 tr("The file is not an DML version 1.0 file."));
        return false;
    }

    clear();

    QTreeWidgetItem *rootItem = createItem(root, nullptr);
    rootItem->setText(0, "dml");

    if (root.hasAttribute(unitsAttribute())) {
        QTreeWidgetItem *unitsItem = createItem(root.attributeNode(unitsAttribute()).toElement(), rootItem);
        unitsItem->setText(0, unitsAttribute());
        unitsItem->setText(1, root.attribute(unitsAttribute()));
    }

    //disconnect(this, &QTreeWidget::itemChanged, this, &DMLTree::updateDomElement);

    parseExpandElement(root.firstChildElement(), rootItem);
    log("Volumes: " + QString::number(n_volumes));
   // connect(this, &QTreeWidget::itemChanged, this, &DMLTree::updateDomElement);

    return true;
}

void DMLTree::parseExpandElement(const QDomElement &element,
                                  QTreeWidgetItem *parentItem)
{
    QTreeWidgetItem *item = createItem(element, parentItem);

    QString title = element.tagName();
    title = title.front().toUpper() + title.mid(1);
    qDebug() << title;
    item->setIcon(0, expandIcon);
    QString id = element.attribute(idAttribute());
    if (!id.isEmpty()) {
        item->setText(0, title + "  (" + id + ")");
    } else {
        item->setText(0, title);
    }

    setItemExpanded(item, false);

    QDomNamedNodeMap attrMap = element.attributes();

    // ---- <volume> ----
    if (element.tagName() == volumeElement()) {
        auto *id =          createAttributeItem(item, attrMap, idAttribute());
        auto *primitive =   createAttributeItem(item, attrMap, primitiveAttribute());
        auto *url =         createAttributeItem(item, attrMap, urlAttribute());
        auto *color =       createAttributeItem(item, attrMap, colorAttribute());
        auto *alpha =       createAttributeItem(item, attrMap, alphaAttribute());
        auto *rendering =   createAttributeItem(item, attrMap, renderingAttribute());
        auto *units =       createAttributeItem(item, attrMap, unitsAttribute());
        n_volumes++;

        Volume *v = new Volume(id        ? id->text(1)           : nullptr,
                              primitive ? primitive->text(1)    : nullptr,
                              url       ? filePath + "/" + url->text(1)          : nullptr,
                              units     ? units->text(1)        : nullptr,
                              rendering ? rendering->text(1)    : nullptr,
                              alpha     ? alpha->text(1)        : nullptr,
                              color     ? color->text(1)        : nullptr);
        v->index = design_ptr->volumes.size();
        design_ptr->volumes.push_back(v);
        design_ptr->volumeMap[v->id] = v;
        log(QString("Loaded Volume: '%1'").arg(v->id));
    }

    // ---- <material> ----
    if (element.tagName() == materialElement()) {
        auto *id = createAttributeItem(item, attrMap, idAttribute());
        auto *name = createAttributeItem(item, attrMap, nameAttribute());
        auto *elast = createAttributeItem(item, attrMap, elasticityAttribute());
        auto *yield = createAttributeItem(item, attrMap, yieldAttribute());
        auto *density = createAttributeItem(item, attrMap, densityAttribute());
        auto *expansion = createAttributeItem(item, attrMap, expansionAttribute());

        Material *m = new Material();
        m->id = id ? id->text(1) : nullptr;
        m->name = name ? name->text(1) : nullptr;
        m->elasticity = elast ? elast->text(1).split(" ")[0].toDouble() : 0;
        if (elast) { m->eUnits = elast->text(1).split(" ").size() > 1 ? elast->text(1).split(" ")[1] : nullptr; }
        m->yield = yield ? yield->text(1).split(" ")[0].toDouble() : 0;
        if (yield) { m->yUnits = yield->text(1).split(" ").size() > 1 ? yield->text(1).split(" ")[1] : nullptr; }
        m->density = density ? density->text(1).split(" ")[0].toDouble() : 0;
        if (density) { m->dUnits = density->text(1).split(" ")[1].size() > 1 ? density->text(1).split(" ")[1] : nullptr; }
        m->expansion = expansion ? expansion->text(1).split(",")[0] : nullptr;
        m->expansionCoeff = expansion ? expansion->text(1).split(",")[1].trimmed().toDouble() : 0;

        m->index = design_ptr->materials.size();
        design_ptr->materials.push_back(*m);
        design_ptr->materialMap[m->id] = m;
        log(QString("Loaded Material: '%1'").arg(m->id));
    }

    // ---- <loadcase> ----
    if (element.tagName() == loadcaseElement()) {
        auto *id = createAttributeItem(item, attrMap, idAttribute());

        Loadcase *l = new Loadcase();
        l->id = id ? id->text(1) : nullptr;
        l->index = design_ptr->loadcases.size();
        l->totalDuration = 0;
        design_ptr->loadcases.push_back(l);
        design_ptr->loadcaseMap[l->id] = l;
        log(QString("Loaded Loadcase: '%1'").arg(l->id));
    }

    // ---- <anchor> ----
    if (element.tagName() == anchorElement()) {
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());

        // TODO add error checking for non-existent volume
        Anchor *a = new Anchor();
        a->volume = volume ? design_ptr->volumeMap[volume->text(1)] : nullptr;

        if (!(a->volume))
            qDebug() << "Volume" << volume->text(1) << "not found";

        QString loadId = parentItem->child(0)->text(1);
        design_ptr->loadcaseMap[loadId]->anchors.push_back(a);
        design_ptr->loadcaseMap[loadId]->anchorMap[a->volume->id] = a;
        log(QString("Loaded Anchor: '%1'").arg(a->volume->id));
    }

    // ---- <force> ----
    if (element.tagName() == forceElement()) {
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());
        auto *magnitude = createAttributeItem(item, attrMap, magnitudeAttribute());
        auto *duration = createAttributeItem(item, attrMap, durationAttribute());
        auto *vary = createAttributeItem(item, attrMap, varyAttribute());

        // TODO add error checking for non-existent volume
        Force *f = new Force();
        f->volume = volume ? design_ptr->volumeMap[volume->text(1)] : nullptr;
        f->magnitude = magnitude ? parseVec(magnitude->text(1)) : Vec(0, 0, 0);
        f->duration = duration ? duration->text(1).toDouble() : -1;
        f->vary = vary ? parseVec(vary->text(1)) : Vec(0,0,0);

        if (!(f->volume))
            qDebug() << "Volume" << volume->text(1) << "not found";

        QString loadId = parentItem->child(0)->text(1);
        design_ptr->loadcaseMap[loadId]->forces.push_back(f);
        design_ptr->loadcaseMap[loadId]->forceMap[f->volume->id] = f;
        design_ptr->loadcaseMap[loadId]->totalDuration = std::max(design_ptr->loadcaseMap[loadId]->totalDuration,
                f->duration);
        log(QString("Loaded Force: '%1'").arg(f->volume->id));
    }

     // ---- <force> ----
    if (element.tagName() == torqueElement()) {
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());
        auto *magnitude = createAttributeItem(item, attrMap, magnitudeAttribute());
        auto *duration = createAttributeItem(item, attrMap, durationAttribute());
        auto *vary = createAttributeItem(item, attrMap, varyAttribute());
        auto *origin = createAttributeItem(item, attrMap, originAttribute());

        // TODO add error checking for non-existent volume
        Torque *t = new Torque();
        t->volume = volume ? design_ptr->volumeMap[volume->text(1)] : nullptr;
        t->magnitude = magnitude ? parseVec(magnitude->text(1)) : Vec(0, 0, 0);
        t->duration = duration ? duration->text(1).toDouble() : -1;
        t->vary = vary ? parseVec(vary->text(1)) : Vec(0,0,0);
        t->origin = origin ? parseVec(origin->text(1)) : Vec(0,0,0);
        
        if (!(t->volume))
            qDebug() << "Volume" << volume->text(1) << "not found";

        QString loadId = parentItem->child(0)->text(1);
        design_ptr->loadcaseMap[loadId]->torques.push_back(t);
        design_ptr->loadcaseMap[loadId]->torqueMap[t->volume->id] = t;
        design_ptr->loadcaseMap[loadId]->totalDuration = std::max(design_ptr->loadcaseMap[loadId]->totalDuration,
                t->duration);
        log(QString("Loaded Torque: '%1'").arg(t->volume->id));
    }

    // ---- <actuation> ----
    if (element.tagName() == actuationElement()) {
        auto *wave = createAttributeItem(item, attrMap, waveAttribute());
        auto *period = createAttributeItem(item, attrMap, periodAttribute());
        auto *offset = createAttributeItem(item, attrMap, offsetAttribute());
        auto *omega = createAttributeItem(item, attrMap, omegaAttribute());
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());

        Actuation *a = new Actuation();
        a->wave = wave ? wave->text(1) == "sin"? Actuation::SIN : Actuation::NONE : Actuation::NONE;
        if (wave) {
            if (wave->text(1) == "sin") a->wave = Actuation::SIN;
            else if (wave->text(1) == "expand_sin") a->wave = Actuation::EXPAND_SIN;
            else if (wave->text(1) == "contract_sin") a->wave = Actuation::CONTRACT_SIN;
            else a->wave = Actuation::NONE;
        } else {
            a->wave = Actuation::NONE;
        }
        a->period = period ? period->text(1).toDouble() : 0.5;
        a->offset = offset ? offset->text(1).toDouble() : 0.0;
        a->omega = omega ? omega->text(1).toDouble() : 20.0;
        a->volume = volume ? design_ptr->volumeMap[volume->text(1)] : nullptr;

        if (!(a->volume))
            qDebug() << "Volume" << volume->text(1) << "not found";

        QString loadId = parentItem->child(0)->text(1);
        design_ptr->loadcaseMap[loadId]->actuations.push_back(a);
        design_ptr->loadcaseMap[loadId]->actuationMap[a->volume->id] = a;
        log(QString("Loaded Actuation: '%1'").arg(a->volume->id));
    }

    // ---- <simulation> ----
    if (element.tagName() == simulationElement()) {
        auto *id = createAttributeItem(item, attrMap, idAttribute());
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());


        SimulationConfig *s = new SimulationConfig();
        s->id = id ? id->text(1) : nullptr;
        s->volume = volume ? design_ptr->volumeMap[volume->text(1)] : nullptr;
        s->index = design_ptr->simConfigs.size();
        design_ptr->simConfigs.push_back(*s);
        design_ptr->simConfigMap[s->id] = s;

        if (!(s->volume))
          qDebug() << "Volume" << volume->text(1) << "not found";

        log(QString("Loaded Simulation Config: '%1'").arg(s->id));
    }

    // ---- <lattice> ----
    if (element.tagName() == latticeElement()) {
        qDebug() << "in lattice loader";
        auto *fill = createAttributeItem(item, attrMap, fillAttribute());
        auto *unit = createAttributeItem(item, attrMap, unitAttribute());
        auto *display = createAttributeItem(item, attrMap, displayAttribute());
        auto *conform = createAttributeItem(item, attrMap, conformAttribute());
        auto *offset = createAttributeItem(item, attrMap, offsetAttribute());
        auto *bardiam = createAttributeItem(item, attrMap, bardiamAttribute());
        auto *material = createAttributeItem(item, attrMap, materialAttribute());
        auto *jiggle = createAttributeItem(item, attrMap, jiggleAttribute());
        auto *hull = createAttributeItem(item, attrMap, hullAttribute());
        auto *volume = createAttributeItem(item, attrMap, volumeAttribute());
        auto *structure = createAttributeItem(item, attrMap, structureAttribute());

        qDebug() << "Loading lattice config";

        QString simConfigId = parentItem->child(0)->text(1);

        LatticeConfig *l = new LatticeConfig();
        l->fill = fill ? (fill->text(1) == "cubic"?
                             LatticeConfig::CUBIC_FILL : LatticeConfig::SPACE_FILL) : LatticeConfig::SPACE_FILL;

        l->unit = unit ? parseVec(unit->text(1)) : Vec(0, 0, 0);
        l->display = display ? display->text(1) : nullptr;
        l->conform = conform ? conform->text(1).toInt() : false;
        l->offset = offset ? parseVec(offset->text(1)) : Vec(0, 0, 0);
        l->barDiameter = bardiam ? parseVec(bardiam->text(1)) : Vec(0, 0, 0);
        l->material = material ? design_ptr->materialMap[material->text(1)] : nullptr;
        l->jiggle = jiggle ? parseVec(jiggle->text(1)) : Vec(0, 0, 0);
        l->hull = hull ? hull->text(1).toInt() : true;
        l->structure = structure ? (structure->text(1) == "bars"?
                            LatticeConfig::BARS :
                            LatticeConfig::FULL) : LatticeConfig::FULL;
        l->volume = volume ? design_ptr->volumeMap[volume->text(1)] : design_ptr->simConfigMap[simConfigId]->volume;

        if (!l->material)
            qDebug() << "Material" << material->text(1) << "not found";

        design_ptr->simConfigMap[simConfigId]->lattices.push_back(l);
        design_ptr->simConfigMap[simConfigId]->latticeMap[l->volume->id] = l;
    }

    // ---- <damping> ----
    if (element.tagName() == dampingElement()) {
        auto *velocity = createAttributeItem(item, attrMap, velocityAttribute());

        Damping d = Damping();
        d.velocity = velocity ? velocity->text(1).toDouble() : 0;

        QString simConfigId = parentItem->child(0)->text(1);
        design_ptr->simConfigMap[simConfigId]->damping = d;
    }

    // ---- <acceleration> ----
    if (element.tagName() == globalElement()) {
        auto *acceleration = createAttributeItem(item, attrMap, accelerationAttribute());

        Global g = Global();
        g.acceleration = acceleration ? parseVec(acceleration->text(1)) : Vec(0, 0, 0);

        QString simConfigId = parentItem->child(0)->text(1);
        design_ptr->simConfigMap[simConfigId]->global = g;
    }

    // ---- <repeat> ----
    if (element.tagName() == repeatElement()) {
        auto *after = createAttributeItem(item, attrMap, afterAttribute());
        auto *rotation = createAttributeItem(item, attrMap, rotationAttribute());

        Repeat r = Repeat();
        if (after) {
            if (after->text(1) == "optimize") {
                r.afterExplicit = false;
            } else {
                r.afterExplicit = true;
                r.after = after->text(1).split(" ")[0].toDouble();
            }
        } else {
            r.afterExplicit = true;
            r.after = -1;
        }

        if (rotation) {
            if (rotation->text(1) == "random") {
                r.rotationExplicit = false;
            } else {
                r.rotationExplicit = true;
                r.rotation = parseVec(rotation->text(1));
            }
        } else {
            r.rotationExplicit = true;
            r.rotation = Vec(0, 0, 0);
        }

        QString simConfigId = parentItem->child(0)->text(1);
        design_ptr->simConfigMap[simConfigId]->repeat = r;
    }

    // ---- <plane> ----
    if (element.tagName() == planeElement()) {
        auto *normal = createAttributeItem(item, attrMap, normalAttribute());
        auto *offset = createAttributeItem(item, attrMap, offsetAttribute());

        Plane *p = new Plane();
        p->normal = normal ? parseVec(normal->text(1)) : Vec(0, 1, 0);
        p->offset = offset ? offset->text(1).toDouble() : 0;

        QString simConfigId = parentItem->child(0)->text(1);
        design_ptr->simConfigMap[simConfigId]->plane = p;
    }

    // ---- <load> ----
    if (element.tagName() == loadElement()) {
        auto *id = createAttributeItem(item, attrMap, idAttribute());
        auto *queue = createAttributeItem(item, attrMap, queueAttribute());

        vector<Loadcase *> q = vector<Loadcase *>();
        Loadcase *l = id ? design_ptr->loadcaseMap[id->text(1)] : nullptr;
        if (queue) {
            auto text = queue->text(1).split(",");
            for (auto s : text) {
                l = design_ptr->loadcaseMap[s.trimmed()];
                qDebug() << s << q.size();
                q.push_back(l);
            }
        }

        if (!l)
            qDebug() << "Loadcase" << id->text(1) << "not found";

        QString simConfigId = parentItem->child(0)->text(1);
        design_ptr->simConfigMap[simConfigId]->load = l;
        design_ptr->simConfigMap[simConfigId]->loadQueue = q;
    }

    // ---- <stop> ----
    if (element.tagName() == stopElement()) {
        auto *criterion = createAttributeItem(item, attrMap, criterionAttribute());
        auto *threshold = createAttributeItem(item, attrMap, thresholdAttribute());
        auto *metric = createAttributeItem(item, attrMap, metricAttribute());

        if (parentItem->text(0).toLower().startsWith(simulationElement())) {
            Stop s = Stop();
            s.criterion = criterion ? (criterion->text(1) == "time"?
                                       Stop::SC_TIME : Stop::SC_MOTION) :
                          Stop::SC_TIME;
            QString t = threshold ? threshold->text(1) : "0";
            if (t.endsWith(QChar('%'))) {
                s.threshold = t.remove(QChar('%')).trimmed().toDouble() / 100;
            } else {
                s.threshold = t.toDouble();
            }

            QString simConfigId = parentItem->child(0)->text(1);
            design_ptr->simConfigMap[simConfigId]->stops.push_back(s);
        }
        if (parentItem->text(0).toLower().startsWith(optimizationElement())) {
            OptimizationStop s = OptimizationStop();

            if (metric) {
                if (metric->text(1) == "weight") {
                    s.metric = OptimizationStop::WEIGHT;
                } else if (metric->text(1) == "energy") {
                    s.metric = OptimizationStop::ENERGY;
                } else if (metric->text(1) == "deflection") {
                    s.metric = OptimizationStop::DEFLECTION;
                } else if (metric->text(1) == "iterations") {
                    s.metric = OptimizationStop::ITERATIONS;
                } else {
                    log(tr("Invalid <stop> criteria in <optimization>: '%1'").arg(metric->text(1)));
                    s.metric = OptimizationStop::NONE;
                }
            } else {
                s.metric = OptimizationStop::NONE;
            }
            QString t = threshold ? threshold->text(1) : "0";
            if (t.endsWith(QChar('%'))) {
                s.threshold = t.remove(QChar('%')).trimmed().toDouble() / 100;
            } else {
                s.threshold = t.toDouble();
            }

            if (design_ptr->optConfig != nullptr) {
                design_ptr->optConfig->stopCriteria.push_back(s);
            }
        }
    }

    // ---- <optimization> ----
    if (element.tagName() == optimizationElement()) {
        auto *sim = createAttributeItem(item, attrMap, simulationElement());

        OptimizationConfig *o = new OptimizationConfig();
        o->simulationConfig = sim ? design_ptr->simConfigMap[sim->text(1)] : nullptr;
        design_ptr->optConfig = o;

        if (!(o->simulationConfig))
            qDebug() << "Simulation" << sim->text(1) << "not found";

        log(QString("Loaded Optimization Config: '%1'").arg(o->simulationConfig->id));
    }

    // ---- <rule> ----
    if (element.tagName() == ruleElement()) {
        auto *method = createAttributeItem(item, attrMap, methodAttribute());
        auto *threshold = createAttributeItem(item, attrMap, thresholdAttribute());
        auto *frequency = createAttributeItem(item, attrMap, frequencyAttribute());
        auto *memory = createAttributeItem(item, attrMap, memoryAttribute());
        auto *regenRate = createAttributeItem(item, attrMap, regenRateAttribute());
        auto *regenThreshold = createAttributeItem(item, attrMap, regenThresholdAttribute());

        OptimizationRule r = OptimizationRule();
        if (method) {
            if (method->text(1) == "remove_low_stress") {
                r.method = OptimizationRule::REMOVE_LOW_STRESS;
            } else if (method->text(1) == "mass_displace") {
                r.method = OptimizationRule::MASS_DISPLACE;
            } else {
                r.method = OptimizationRule::NONE;
            }
        } else {
            r.method = OptimizationRule::NONE;
        }
        if (threshold) {
            QString t = threshold->text(1);
            qDebug() << "Threshold" << t;
            if (t.endsWith(QChar('%'))) {
                r.threshold = t.remove(QChar('%')).trimmed().toDouble() / 100;
            } else {
                r.threshold = t.toDouble();
            }
        } else { r.threshold = 0; }
        r.frequency = frequency ? frequency->text(1).toInt() : 0;
        r.memory = memory ? memory->text(1).toDouble() : 1;
        if (regenRate) {
            QString rr = regenRate->text(1);
            qDebug() << "Regeneration Rate" << rr;
            if (rr.endsWith(QChar('%'))) {
                r.regenRate = rr.remove(QChar('%')).trimmed().toDouble() / 100;
            } else {
                r.regenRate = rr.toDouble();
            }
        }
        if (regenThreshold) {
            QString rt = regenThreshold->text(1);
            qDebug() << "Regeneration Threshold" << rt;
            if (rt.endsWith(QChar('%'))) {
                r.regenThreshold = rt.remove(QChar('%')).trimmed().toDouble() / 100;
            } else {
                r.regenThreshold = rt.toDouble();
            }
        }

        if (design_ptr->optConfig != nullptr) {
            design_ptr->optConfig->rules.push_back(r);
        }
        qDebug() << "Rules" << design_ptr->optConfig->rules.front().threshold;

    }

    // ---- <constraint> ----
    if (element.tagName() == constraintElement()) {

    }

    // ---- <output> ----
    if  (element.tagName() == outputElement()) {
        auto *id = createAttributeItem(item, attrMap, idAttribute());
        auto *sim = createAttributeItem(item, attrMap, simulationElement());

        auto *o =  new output_data();
        o->id = id ? id->text(1) : "";
        o->sim = sim ? design_ptr->simConfigMap[sim->text(1)] : nullptr;

        if (!(o->sim))
            qDebug() << "Simulation" << sim->text(1) << "not found";

        design_ptr->simConfigMap[sim->text(1)]->output = o;
        design_ptr->outputs.push_back(o);
        design_ptr->outputMap[o->id] = o;
    }

    // ---- <include> ----
    if (element.tagName() == includeElement()) {
        auto *vol = createAttributeItem(item, attrMap, volumeAttribute());

        QString outputId = parentItem->child(0)->text(1);
        output_data * o = design_ptr->outputMap[outputId];

        if (vol) {
            Volume *v = design_ptr->volumeMap[vol->text(1)];
            o->includes.push_back(v);
        }
    }

    // ---- <exclude> ----
    if (element.tagName() == excludeElement()) {
        auto *vol = createAttributeItem(item, attrMap, volumeAttribute());

        QString outputId = parentItem->child(0)->text(1);
        output_data * o = design_ptr->outputMap[outputId];

        if (vol) {
            Volume *v = design_ptr->volumeMap[vol->text(1)];
            o->excludes.push_back(v);
        }
    }


    QDomElement sibling = element.nextSiblingElement();
    if (!sibling.isNull()) {
        parseExpandElement(sibling, parentItem);
    }
    QDomElement child = element.firstChildElement();
    if (!child.isNull()) {
        parseExpandElement(child, item);
    }
}

QTreeWidgetItem *DMLTree::createAttributeItem(QTreeWidgetItem *parentItem, QDomNamedNodeMap attrMap, QString attrName) {
    if (!attrMap.contains(attrName)) {
        return nullptr;
    }

    QDomNode attrNode = attrMap.namedItem(attrName);

    QTreeWidgetItem *attrItem = createItem(attrNode.toElement(), parentItem);
    QString attrValue = attrNode.nodeValue();
    attrItem->setText(0, attrName);
    attrItem->setText(1, attrValue);

    return attrItem;
}

QTreeWidgetItem *DMLTree::createItem(const QDomElement &element,
                                      QTreeWidgetItem *parentItem)
{

    QTreeWidgetItem *item;
    if (parentItem) {
        item = new QTreeWidgetItem(parentItem);
    } else {
        item = new QTreeWidgetItem(this);
    }
    item->setData(0, DomElementRole, QVariant::fromValue(element));
    this->insertTopLevelItem(0, item);
    return item;
}

bool DMLTree::write(QIODevice *device) const
{
    const int IndentSize = 4;

    QTextStream out(device);
    domDocument.save(out, IndentSize);
    return true;
}

Vec DMLTree::parseVec(QString vecString) {
    double x, y, z;
    std::string vecStdString;
    const char * vecConstChar;

    vecStdString = vecString.toStdString();
    vecConstChar = vecStdString.c_str();

    if (3 == sscanf(vecConstChar, "%lf,%lf,%lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }
    if (3 == sscanf(vecConstChar, "%lf, %lf, %lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }
    if (3 == sscanf(vecConstChar, "%lf %lf %lf", &x, &y, &z)) {
        return Vec(x, y, z);
    }

    log(QString("Malformed DML: Expected text in the form \"value, value, value\" but got \"%1\"").arg(vecString));
    return Vec(0, 0, 0);
}
