//
// Created by sw3390 on 3/2/20.
//


#ifndef DMLIDE_OUTILS_H
#define DMLIDE_OUTILS_H

#endif //DMLIDE_OUTILS_H

#include <Titan/sim.h>
#include <QDebug>

#include "utils.h"
#include <map>

class oUtils {
public:
    void static generateMassesPoisson(double minCut, map<Mass *, vector<Spring *>> mToS, vector<Vec> &lattice);
    void static generateMassesBounded(double minCut, map<Mass *, vector<Spring *>> mToS, vector<Vec> &lattice);

};