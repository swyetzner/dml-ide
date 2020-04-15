//
// Created by sw3390 on 3/2/20.
//

#include "oUtils.h"


void oUtils::generateMassesPoisson(double minCut, map<Mass *, vector<Spring *> > mToS, vector<Vec> &lattice) {
    Vec minPos = Vec(FLT_MAX, FLT_MAX, FLT_MAX);
    Vec maxPos = Vec(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (auto &m : mToS) {
        minPos[0] = std::min(minPos[0], m.first->origpos[0]);
        minPos[1] = std::min(minPos[1], m.first->origpos[1]);
        minPos[2] = std::min(minPos[2], m.first->origpos[2]);
        maxPos[0] = std::max(maxPos[0], m.first->origpos[0]);
        maxPos[1] = std::max(maxPos[1], m.first->origpos[1]);
        maxPos[2] = std::max(maxPos[2], m.first->origpos[2]);
    }

    qDebug() << "Max" << maxPos[0] << maxPos[1] << maxPos[2] << "Min" << minPos[0] << minPos[1] << minPos[2];

    int xLines = int((maxPos[0] - minPos[0]) / minCut) + 1;
    int yLines = int((maxPos[1] - minPos[1]) / minCut) + 1;
    int zLines = int((maxPos[2] - minPos[2]) / minCut) + 1;
    int kNewPoints = xLines * yLines * zLines;
    kNewPoints *= 3;

    qDebug() << "Generating" << kNewPoints << "random point candidates with cutoff" << minCut;

    lattice = vector<Vec>();
    vector<Vec> candidates = vector<Vec>();
    vector<double> sumDists = vector<double>();

    minPos += Vec(1E-4, 1E-4, 1E-4);
    maxPos -= Vec(1E-4, 1E-4, 1E-4);

    lattice.push_back(Utils::randPointVec(minPos, maxPos));
    double maxLength = minCut;

    for (int k = 0; k < kNewPoints; k++) {
        Vec p = Utils::randPointVec(minPos, maxPos);

        bool close = false;
        for (auto &m: mToS) {
            if ((m.first->origpos - p).norm() <= 1E-6) {
                close = true;
            }
        }
        if (!close) {
            candidates.push_back(p);
            sumDists.push_back(0.0);
        }
    }

    while (maxLength >= minCut && !candidates.empty()) {
        uint ifar = 0;
        double maxDist = 0.0;

        for (int i = 0; i < candidates.size(); i++) {
            bool reject = false;

            Vec l = lattice.back();
            double d = (candidates[i] - l).norm();

            if (d < minCut) {
                candidates.erase(candidates.begin() + i);
                sumDists.erase(sumDists.begin() + i);
                i--;
                continue;
            }
            if (sumDists[i] + d > maxDist) {
                maxDist = sumDists[i] + d;
                ifar = i;
            }
        }

        if (!candidates.empty()) {
            maxLength = maxDist;
            for (Vec l : lattice)
                maxLength = std::min(maxLength, (candidates[ifar] - l).norm());
            lattice.push_back(candidates[ifar]);
            candidates.erase(candidates.begin() + ifar);
            for (int i = 0; i < candidates.size(); i++) {
                sumDists[i] += (candidates[i] - lattice.back()).norm();
            }
            qDebug() << "Added to lattice" << lattice.back()[0] << lattice.back()[1] << lattice.back()[2];
        }
    }
}


void oUtils::generateMassesBounded(double minCut, map<Mass *, vector<Spring *>> mToS, vector<Vec> &lattice) {

    // Normalize forces
    vector<double> normForces;
    double totalProb = 0.0;
    for (auto i : mToS) {
        normForces.push_back(0.0);
        for (Spring *s : i->second) {
            normForce.back() += s->getForce().norm();
        }
        totalProb += normForces.back();
    }

    int genN = mToS.size() * 0.1;
    for (int i = 0; i < genN; i++) {

        // Generate new point
        double a = Utils::randUnit() * totalProb;
        auto it = mToS.begin();
        int off = 0;
        while (a -= normForces[off] > 0) {
            off++;
        }

        std::advance(it, off);

        Mass *m = it->first;

        Vec p = m->pos + (Utils::randDirectionVec() * minCut/2);
        qDebug() << "Min cut" << minCut/32 << it->second.size() << it->second.front();
        bool tooClose = false;
        for (auto l : lattice) {
            if ((l - p).norm() <= 1E-6) tooClose = true;
        }
        if (!tooClose) lattice.push_back(p);
    }
}