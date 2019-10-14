//
// Created by sw3390 on 3/25/19.
//

#include "polygonizer.h"

//---------------------------------------------------------------------------
Polygonizer::Polygonizer(bar_data *barModel, double resolution, double barDiameter, int threads) {
//---------------------------------------------------------------------------
    this->barModel = barModel;
    this->threads = threads;
    this->resolution = resolution;
    this->barRadius = barDiameter / 2;
    this->barModel->setRadii(this->barRadius);

    if (barModel->anchorShell != nullptr) {
        barModel->bounds.combine(*barModel->anchorShell);
    }
    if (barModel->forceSphere != nullptr) {
        barModel->bounds.maxCorner += Vec(0.02, 0, 0);
    }

    // Add bar radius to bounds
    barModel->bounds.minCorner -= Vec(this->barRadius*4, this->barRadius*4, this->barRadius*4);
    barModel->bounds.maxCorner += Vec(this->barRadius*4, this->barRadius*4, this->barRadius*4);

    this->xMin = barModel->bounds.minCorner[0];
    this->xMax = barModel->bounds.maxCorner[0];
    this->mSize = barModel->bounds.maxCorner - barModel->bounds.minCorner;

    this->geometry = Polygon();
}


// Constructor using a output config (contains boolean algebra with other volumes)
//---------------------------------------------------------------------------
Polygonizer::Polygonizer(output_data *outputConfig, double resolution, double barDiameter, int threads) {
//---------------------------------------------------------------------------

    this->barModel = outputConfig->barData;
    this->threads = threads;
    this->resolution = resolution;
    this->barRadius = barDiameter / 2;
    this->barModel->setRadii(this->barRadius);
    this->unions = vector<Polygon *>();
    this->unionsNot = vector<Polygon *>();

    qDebug() << outputConfig->includes.size() << "Includes";
    qDebug() << outputConfig->excludes.size() << "Excludes";
    for (auto u : outputConfig->includes) {
        this->unions.push_back(u->geometry);
        boundingBox<Vec> b;
        u->geometry->boundingPoints(b.minCorner, b.maxCorner);
        u->geometry->minc = b.minCorner;
        u->geometry->maxc = b.maxCorner;
        this->barModel->bounds.combine(b);
    }
    for (auto v : outputConfig->excludes) {
        this->unionsNot.push_back(v->geometry);
        boundingBox<Vec> b;
        v->geometry->boundingPoints(b.minCorner, b.maxCorner);
        v->geometry->minc = b.minCorner;
        v->geometry->maxc = b.maxCorner;
        this->barModel->bounds.combine(b);
    }
    qDebug() << this->unions.size() << "Unions";
    qDebug() << this->unionsNot.size() << "Differences";

    cubeMax = -FLT_MAX;


    // Add bar radius to bounds
    barModel->bounds.minCorner -= Vec(this->barRadius*4, this->barRadius*4, this->barRadius*4);
    barModel->bounds.maxCorner += Vec(this->barRadius*4, this->barRadius*4, this->barRadius*4);

    this->xMin = barModel->bounds.minCorner[0];
    this->xMax = barModel->bounds.maxCorner[0];
    this->mSize = barModel->bounds.maxCorner - barModel->bounds.minCorner;

}


// Initializes first round of segments according to model bounds and resolution
// Segments are delineated along the X axis (1, 0, 0)
//---------------------------------------------------------------------------
void Polygonizer::initBaseSegments() {
//---------------------------------------------------------------------------

    double xSize = this->xMax - this->xMin;

    this->nSeg = ulong(this->threads);
    this->segments.resize(nSeg);
    this->sx = xSize / nSeg;
    qDebug() << "Number of Segments:" << nSeg;
    qDebug() << "Size of vector:" << segments.size();

    createSegments(xMin, xMax, this->segments);
    maskBarSegments(xMin, xMax, this->segments);
}


// Creates segments according to given bounds and vector
// Segments are delineated along the X axis (1, 0, 0)
// Assumes given vector is already sized
//---------------------------------------------------------------------------
void Polygonizer::createSegments(double sMin, double sMax, vector<Segment> &s) {
//---------------------------------------------------------------------------

    ulong nSplits = s.size();
    double xSize = sMax - sMin;

    double dx = xSize / nSplits;

    for (int i = 0; i <  nSplits; i++) {
        double sStart = dx*i + sMin;
        s[i] = Segment(sStart, sStart + dx);
        s[i].sidx = i;
        s[i].bsize = this->mSize;
        s[i].bsize[0] = dx;
        s[i].bars = vector<Bar>();
        qDebug() << "Segment" << i << "Begin" << s[i].bmin << "End" << s[i].bmax;
    }
}


// Separates the vertices in a model by which segment they fall into
// Fills given segments' bars vector
// sMin and sMax define the overall width of the segments
//---------------------------------------------------------------------------
void Polygonizer::maskBarSegments(double sMin, double sMax, vector<Segment> &s) {
//---------------------------------------------------------------------------

    double dx = (sMax - sMin) / s.size();

    /**for (const Bar &b : barModel->bars) {

        // Primary segments
        int lSeg = int(floor((b.left[0] - sMin) / dx));
        int rSeg = int(floor((b.right[0] - sMin) / dx));
        int minSeg = (lSeg < rSeg)? lSeg : rSeg;
        int maxSeg = (lSeg > rSeg)? lSeg : rSeg;


        if (lSeg >= 0 && lSeg < s.size()) {
            s[lSeg].bars.push_back(b);
        }
        if (rSeg >= 0 && rSeg < s.size()) {
            s[rSeg].bars.push_back(b);
        }

        // Bar spans more than two segments
        if (maxSeg - minSeg > 1) {
            for (int m = minSeg + 1; m < maxSeg; m++) {
                if (m >= 0 && m < s.size()) {
                    s[m].bars.push_back(b);
                }
            }
        }

        // Bar shell spans segments on either side;
        int minlSeg = int(floor((b.left[0] - sMin - b.diameter) / dx));
        int maxlSeg = int(floor((b.left[0] - sMin + b.diameter) / dx));
        int minrSeg = int(floor((b.right[0] - sMin - b.diameter) / dx));
        int maxrSeg = int(floor((b.right[0] - sMin + b.diameter) / dx));

        //qDebug() << "l" << lSeg << "r" << rSeg << "min" << minSeg << "max" << maxSeg;
        //qDebug() << "l1" << minlSeg << "l2" << maxlSeg << "r1" << minrSeg << "r2" << maxrSeg;
        //qDebug() << sMax << b.right[0] + b.diameter;

        if (minlSeg < minSeg && minlSeg >= 0 && minlSeg < s.size()) {
            s[minlSeg].bars.push_back(b);
        }
        if (maxlSeg > maxSeg && maxlSeg >= 0 && maxlSeg < s.size()) {
            s[maxlSeg].bars.push_back(b);
        }
        if (minrSeg < minSeg && minrSeg >= 0 && minrSeg < s.size()) {
            s[minlSeg].bars.push_back(b);
        }
        if (maxrSeg > maxSeg && maxrSeg >= 0 && maxrSeg < s.size()) {
            s[maxrSeg].bars.push_back(b);
        }
    }

    for (int i = 0; i < segments.size(); i++) {
        qDebug() << "Bars in segment" << i << segments[i].bars.size();
    }**/

    for (Segment &seg : s) {
        for (int i = 0; i < barModel->bars.size(); i++) {
            Bar b = barModel->bars[i];
            double x1p = b.left[0];
            double x2p = b.right[0];

            bool bAdd = false;

            if (x1p + barRadius > seg.bmin && x1p - barRadius < seg.bmax) {
                bAdd = true;
            }
            if (x2p + barRadius > seg.bmin && x2p - barRadius < seg.bmax) {
                bAdd = true;
            }
            if (x1p  + barRadius > seg.bmin && x2p - barRadius < seg.bmax) {
                bAdd = true;
            }
            if (x2p + barRadius > seg.bmin && x1p - barRadius < seg.bmax) {
                bAdd = true;
            }

            if (bAdd) {
                seg.bars.push_back(b);
            }
        }
    }

    for (int i = 0; i < s.size(); i++) {
        qDebug() << "Bars in segment" << i << s[i].bars.size() << "bmin" << s[i].bmin << s[i].bmax;
    }
}


// Runs parallel marching cubes
// One thread per segment
//---------------------------------------------------------------------------
void Polygonizer::calculatePolygon() {
//---------------------------------------------------------------------------

    unsigned long triangleCount = 0;
    unsigned long nodeCount = 0;
    completed = vector<int>(segments.size(), 0);
    cubes = vector<int>(segments.size(), 1);
    printf("\033[2J");
    std::cout << "\033[2;1fCalculating Triangle Mesh..." << std::endl;

    for (int i = 0; i < segments.size(); i++) {
        printProgress(i);
    }


#pragma omp parallel for
    for (int i = 0; i < segments.size(); i++) {
        marchingCubesAdaptive(segments[i], 2);
    }

    printProgress(segments.size() - 1); // Print last status to place cursor at end
    qDebug() << "Marching cubes completed";
    std::cout << "\nMARCHING CUBES COMPLETED" << std::endl;

    for (const Segment &s : segments) {
        triangleCount += s.polygon->triangles->size();
        nodeCount += s.polygon->nodeMap.size();
    }

    this->geometry.triangles = new vector<shared_ptr<Tri>>();
    for (const Segment &s : segments) {
        this->geometry.mergePolygons(*s.polygon);
    }

    qDebug() << "Polygon size" << this->geometry.triangles->size() << "tris" << this->geometry.nodeMap.size() << "nodes";
    std::cout << "Mesh size " << this->geometry.triangles->size() << " tris, " << this->geometry.nodeMap.size() << " nodes\n";

    // Reduce mesh
    //this->geometry.reduceMesh(1E-2, 0.5);
    qDebug() << "Reduced Polygon size" << this->geometry.triangles->size() << "tris" << this->geometry.nodeMap.size() << "nodes";

}


//  Creates an adaptive mesh and calls the marching cube algorithm
//---------------------------------------------------------------------------
void Polygonizer::marchingCubesAdaptive(Segment &s, int level) {
//---------------------------------------------------------------------------

    if (level < 1) {
        return;
    }

    GRIDCELL grid;
    s.triangles = vector<TRIANGLE>();

    int nx = int(ceil(s.bsize[0] / (resolution * level)));
    int ny = int(ceil(s.bsize[1] / (resolution * level)));
    int nz = int(ceil(s.bsize[2] / (resolution * level)));
    Vec dx = Vec(s.bsize[0] / nx, 0, 0);
    Vec dy = Vec(0, s.bsize[1] / ny, 0);
    Vec dz = Vec(0, 0, s.bsize[2] / nz);

    Vec startNode = Vec(s.bmin, barModel->bounds.minCorner[1], barModel->bounds.minCorner[2]);
    vector<double> L0, L1;
    L0.resize(ulong(ny + 1) * (nz + 1));
    L1.resize(ulong(ny + 1) * (nz + 1));
    //qDebug() << "Start Node:" << startNode[0] << startNode[1] << startNode[2];
    //qDebug() << "Segment size:" << s.bsize[0] << s.bsize[1] << s.bsize[2];
    //qDebug() << "Cubes (x,y,z):" << nx << ny << nz << dx[0] << dy[1] << dz[2];
    //qDebug() << "Number of cubes:" << L0.size();
    cubes[s.sidx] = nx * ny * nz;
    completed[s.sidx] = 0;

    cubeMax = std::max(cubeMax, std::max(dx[0], std::max(dx[1], dx[2])));

    // Leftmost points
    for (int j = 0; j < ny + 1; j++) {
        for (int k = 0; k < nz + 1; k++) {
            Vec cubeNode = startNode + j * dy + k * dz;
            L1[j * nz + k] = pointDist(s, cubeNode);
        }
    }

    for (int i = 0; i < nx; i++) {

        L0 = L1;

        for (int j = 0; j < ny + 1; j++) {
            for (int k = 0; k < nz + 1; k++) {
                Vec cubeNode = startNode + (i + 1) * dx + j * dy + k * dz;
                L1[j * nz + k] = pointDist(s, cubeNode);
            }
        }

        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                grid.p[0] = startNode + i * dx + j * dy + k * dz;
                grid.val[0] = L0[j * nz + k];
                grid.p[1] = startNode + (i + 1) * dx + j * dy + k * dz;
                grid.val[1] = L1[j * nz + k];
                grid.p[2] = startNode + (i + 1) * dx + (j + 1) * dy + k * dz;
                grid.val[2] = L1[(j + 1) * nz + k];
                grid.p[3] = startNode + i * dx + (j + 1) * dy + k * dz;
                grid.val[3] = L0[(j + 1) * nz + k];
                grid.p[4] = startNode + i * dx + j * dy + (k + 1) * dz;
                grid.val[4] = L0[j * nz + (k + 1)];
                grid.p[5] = startNode + (i + 1) * dx + j * dy + (k + 1) * dz;
                grid.val[5] = L1[j * nz + (k + 1)];
                grid.p[6] = startNode + (i + 1) * dx + (j + 1) * dy + (k + 1) * dz;
                grid.val[6] = L1[(j + 1) * nz + (k + 1)];
                grid.p[7] = startNode + i * dx + (j + 1) * dy + (k + 1) * dz;
                grid.val[7] = L0[(j + 1) * nz + (k + 1)];

                polygonizeAdaptive(s, grid, 0, level);
                completed[s.sidx]++;
            }
#pragma omp critical
            printProgress(s.sidx);
        }
    }

    //qDebug() << s.sidx << s.polygon->triangles->size();
    //if (s.sidx == 0) qDebug() << "ZERO" << s.polygon->triangles->size();
}


// Adapts marching cubes by recursively splitting gridcells
//---------------------------------------------------------------------------
int Polygonizer::polygonizeAdaptive(Segment &segment,
                                    GRIDCELL grid,
                                    double isolevel,
                                    int level) {
//---------------------------------------------------------------------------

    int nTri = 0;

    /* Exploit the distance that the isolevel indicates
     * to decide if cube contains a bar */
    bool outside = false;
    double gridSpan = (grid.p[7] - grid.p[1]).norm();
    if (grid.val[0] > gridSpan)     outside = true;
    if (grid.val[1] > gridSpan)     outside = true;
    if (grid.val[2] > gridSpan)     outside = true;
    if (grid.val[3] > gridSpan)     outside = true;
    if (grid.val[4] > gridSpan)     outside = true;
    if (grid.val[5] > gridSpan)     outside = true;
    if (grid.val[6] > gridSpan)     outside = true;
    if (grid.val[7] > gridSpan)     outside = true;

    /* Cube is entirely out of the surface */
    if (outside) {
        //qDebug() << grid.val[0] << grid.val[1] << grid.val[2] << grid.val[3]
        //<< grid.val[4] << grid.val[5] << grid.val[6] << grid.val[7] << "Span" << gridSpan;
        return 0;
    }

    /* Call function for base level */
    if (level == 0) {
        return polygonize(segment, grid, isolevel);
    }

    /* Split into eight quadrants */
    GRIDCELL qgrid;
    Vec ip[8][8];
    double iv[8][8];

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < i; j++) {
            ip[i][j] = (grid.p[i] + grid.p[j]) / 2;
            ip[j][i] = (grid.p[i] + grid.p[j]) / 2;
            iv[i][j] = iv[j][i] = pointDist(segment, ip[i][j]);
        }
        ip[i][i] = grid.p[i];
        iv[i][i] = grid.val[i];
    }

    int qt[8][16] = {{0, 0, 0, 1, 0, 2, 3, 0, 0, 4, 0, 5, 0, 6, 3, 4},
                     {0, 1, 1, 1, 1, 2, 0, 2, 0, 5, 1, 5, 2, 5, 0, 6},
                     {0, 2, 1, 2, 2, 2, 2, 3, 0, 6, 2, 5, 2, 6, 2, 7},
                     {3, 0, 0, 2, 2, 3, 3, 3, 3, 4, 0, 6, 3, 6, 7, 3},
                     {0, 4, 0, 5, 0, 6, 3, 4, 4, 4, 4, 5, 5, 7, 4, 7},
                     {0, 5, 1, 5, 2, 5, 0, 6, 4, 5, 5, 5, 5, 6, 4, 6},
                     {0, 6, 2, 5, 2, 6, 3, 6, 4, 6, 5, 6, 6, 6, 6, 7},
                     {3, 4, 0, 6, 3, 6, 7, 3, 7, 4, 4, 6, 6, 7, 7, 7}};

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            qgrid.p[j] = ip[qt[i][2*j]][qt[i][2*j + 1]];
            qgrid.val[j] = iv[qt[i][2*j]][qt[i][2*j + 1]];
        }
        nTri += polygonizeAdaptive(segment, qgrid, isolevel, level-1);
    }

    return nTri;
}

// Implemented adapted from Paul Bourke
// "Polygonising a scalar field"
// http://paulbourke.net/geometry/polygonise/
//---------------------------------------------------------------------------
int Polygonizer::polygonize(Segment &segment,
                                    GRIDCELL grid,
                                    double isolevel) {
//---------------------------------------------------------------------------

/* Given a grid cell and an isolevel, calculate the triangular
facets required to represent the isosurface through the cell.
Return the number of triangular facets, the array "triangles"
will be loaded up with the vertices at most 5 triangular facets.
0 will be returned if the grid cell is either totally above
of totally below the isolevel. */

    int i, nTri;
    int cubeIndex;
    Vec vertList[12];

    /* Determine the index into the edge table which
     * tells us which vertices are inside of the surface */
    cubeIndex = 0;
    if (grid.val[0] < isolevel)     cubeIndex |= 1;
    if (grid.val[1] < isolevel)     cubeIndex |= 2;
    if (grid.val[2] < isolevel)     cubeIndex |= 4;
    if (grid.val[3] < isolevel)     cubeIndex |= 8;
    if (grid.val[4] < isolevel)     cubeIndex |= 16;
    if (grid.val[5] < isolevel)     cubeIndex |= 32;
    if (grid.val[6] < isolevel)     cubeIndex |= 64;
    if (grid.val[7] < isolevel)     cubeIndex |= 128;

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeIndex] == 0)  return 0;

    /* Find vertices where the surface intersects the cube */
    if (edgeTable[cubeIndex] & 1)
        vertList[0] = vertexInterp(isolevel, grid.p[0], grid.p[1], grid.val[0], grid.val[1]);
    if (edgeTable[cubeIndex] & 2)
        vertList[1] = vertexInterp(isolevel, grid.p[1], grid.p[2], grid.val[1], grid.val[2]);
    if (edgeTable[cubeIndex] & 4)
        vertList[2] = vertexInterp(isolevel, grid.p[2], grid.p[3], grid.val[2], grid.val[3]);
    if (edgeTable[cubeIndex] & 8)
        vertList[3] = vertexInterp(isolevel, grid.p[3], grid.p[0], grid.val[3], grid.val[0]);
    if (edgeTable[cubeIndex] & 16)
        vertList[4] = vertexInterp(isolevel, grid.p[4], grid.p[5], grid.val[4], grid.val[5]);
    if (edgeTable[cubeIndex] & 32)
        vertList[5] = vertexInterp(isolevel, grid.p[5], grid.p[6], grid.val[5], grid.val[6]);
    if (edgeTable[cubeIndex] & 64)
        vertList[6] = vertexInterp(isolevel, grid.p[6], grid.p[7], grid.val[6], grid.val[7]);
    if (edgeTable[cubeIndex] & 128)
        vertList[7] = vertexInterp(isolevel, grid.p[7], grid.p[4], grid.val[7], grid.val[4]);
    if (edgeTable[cubeIndex] & 256)
        vertList[8] = vertexInterp(isolevel, grid.p[0], grid.p[4], grid.val[0], grid.val[4]);
    if (edgeTable[cubeIndex] & 512)
        vertList[9] = vertexInterp(isolevel, grid.p[1], grid.p[5], grid.val[1], grid.val[5]);
    if (edgeTable[cubeIndex] & 1024)
        vertList[10] = vertexInterp(isolevel, grid.p[2], grid.p[6], grid.val[2], grid.val[6]);
    if (edgeTable[cubeIndex] & 2048)
        vertList[11] = vertexInterp(isolevel, grid.p[3], grid.p[7], grid.val[3], grid.val[7]);

    /* Create the triangle */
    nTri = 0;
    TRIANGLE tri;

    for (i = 0; triTable[cubeIndex][i] != -1; i += 3) {
        tri.p[0] = vertList[triTable[cubeIndex][i]];
        tri.p[1] = vertList[triTable[cubeIndex][i + 1]];
        tri.p[2] = vertList[triTable[cubeIndex][i + 2]];
        addTriangle(segment, tri);
        nTri++;
    }

    return nTri;
}


// Given a TRIANGLE filled with positions,
// calculate the normal and add triangle to array
// Uses segment locality to shorten distance calculation
//---------------------------------------------------------------------------
void Polygonizer::addTriangle(Segment &s, TRIANGLE tri) {
//---------------------------------------------------------------------------

    Vec n = findNormal(tri.p[0], tri.p[1], tri.p[2]);

    Vec mid = (tri.p[0] + tri.p[1] + tri.p[2]) / 3;
    Vec nsm = n * 1E-4;

    if (pointDist(s, mid + nsm) < 0)
        n = -n;

    tri.n = n;
    //s.triangles.push_back(tri);
    s.polygon->addTriangle(tri.p[0], tri.p[1], tri.p[2], tri.n);
}

// Calculate isovalue
//---------------------------------------------------------------------------
double Polygonizer::pointDist(Segment &s, Vec qp) {
//---------------------------------------------------------------------------

    double minDist = pointDistFromBars(s.bars, qp);

    if (this->barModel->anchorShell != nullptr) {
        double boxDist = pointDistFromBox(this->barModel->anchorShell, this->barModel->boxCutoff, qp);
        minDist = (minDist < boxDist? minDist : boxDist);
    }
    if (this->barModel->forceSphere != nullptr) {
        double sphereDist = pointDistFromSphere(*this->barModel->forceSphere, 0.015, qp);
        minDist = (minDist < sphereDist? minDist : sphereDist);
    }

    for (Polygon *u : unions) {
        if (u->withinBounds(u->minc - Vec(cubeMax,cubeMax,cubeMax), u->maxc + Vec(cubeMax,cubeMax,cubeMax), qp)) {
            double surfaceDist = pointDistFromSurface(*u, qp);
            //minDist = (minDist < -surfaceDist? minDist : -surfaceDist); intersection
            minDist = (minDist < surfaceDist ? minDist : surfaceDist);
        }
    }
    for (Polygon *v : unionsNot) {
        if (v->withinBounds(v->minc - Vec(cubeMax,cubeMax,cubeMax), v->maxc + Vec(cubeMax,cubeMax,cubeMax), qp)) {
            double diffSurfaceDist = pointDistFromSurface(*v, qp);
            if (minDist < 0 && diffSurfaceDist < 0) minDist = -diffSurfaceDist;
        }
    }

    return minDist;
}

// Calculate minimum distance from surface
// Surface defined by bars and bar diameter
//---------------------------------------------------------------------------
double Polygonizer::pointDistFromBars(vector<Bar> &surface, Vec qp) {
//---------------------------------------------------------------------------
    double minDist = FLT_MAX;
    Vec minPoint;
    QString minType;

    // Distance to bar endpoints
    for (const Bar &b : surface) {
        if (b.diameter != -1) {
            double distL = (qp - b.left).norm();
            double distR = (qp - b.right).norm();

            /**if (distL > b.diameter * 2 && distR > b.diameter * 2) {
               continue;
            }**/

            distL -= b.diameter / 2;
            distR -= b.diameter / 2;
            if (distL < minDist) {
                minDist = distL;
                minPoint = b.left;
                minType = "Left";
            }
            if (distR < minDist) {
                minDist = distR;
                minPoint = b.right;
                minType = "Right";
            }
        }
    }
    // Distance to bar
    for (const Bar &b : surface) {
        if (b.diameter != -1) {
            Vec barVec = b.right - b.left;
            double barLen = barVec.norm();
            double radius = b.diameter / 2;

            barVec = barVec.normalized();
            double projDist = dot(qp - b.left, barVec);
            if (projDist > 0 && projDist < barLen) {
                Vec proj = b.left + projDist * barVec;
                double dist = (qp - proj).norm() - radius;

                if (dist < minDist) {
                    minDist = dist;
                    minPoint = proj;
                    minType = "Middle";
                }
            }
        }
    }

    return minDist;
}


// Calculate minimum distance from surface
// Surface defined by box bounds and cutoff for hollowness
//---------------------------------------------------------------------------
double Polygonizer::pointDistFromBox(boundingBox<Vec> *box, double shellWidth, Vec qp) {
//---------------------------------------------------------------------------

    double dist;
    double distX = -std::min(box->maxCorner[0] - qp[0], qp[0] - box->minCorner[0]);
    double distY = -std::min(box->maxCorner[1] - qp[1], qp[1] - box->minCorner[1]);
    double distZ = -std::min(box->maxCorner[2] - qp[2], qp[2] - box->minCorner[2]);

    if (distX > 0 && distY > 0 && distZ > 0) {
        dist = sqrt(distX*distX + distY*distY + distZ*distZ);
    } else {
        dist = std::min(std::min(-distX, -distY), -distZ);
        dist = -dist;
    }

    // Inside shell
    if (shellWidth > 0) {
        double distI;
        double distIX = std::min(box->maxCorner[0] - shellWidth - qp[0], qp[0] - box->minCorner[0] - shellWidth);
        double distIY = std::min(box->maxCorner[1] - shellWidth - qp[1], qp[1] - box->minCorner[1] - shellWidth);
        double distIZ = std::min(box->maxCorner[2] - shellWidth - qp[2], qp[2] - box->minCorner[2] - shellWidth);

        if (distIX < 0 && distIY < 0 && distIZ < 0) {
            distI = -sqrt(distIX * distIX + distIY * distIY + distIZ * distIZ);
        } else {
            distI = std::min(std::min(distIX, distIY), distIZ);
        }

        return std::max(dist, distI);
    }
    return dist;
}


// Calculate minimum distance from surface
// Surface defined by sphere center and diameter
//---------------------------------------------------------------------------
double Polygonizer::pointDistFromSphere(Vec spherePos, double sphereDiam, Vec qp) {
//---------------------------------------------------------------------------

    double dist = (qp - spherePos).norm();
    dist -= sphereDiam/2;

    return dist;
}


// Calculate minimum distance from a convex surface
// Surface defined by Polygon made of triangles
// Negative values indicate inside surface
//---------------------------------------------------------------------------
double Polygonizer::pointDistFromSurface(Polygon &surface, Vec qp) {
//---------------------------------------------------------------------------

    double minDist = FLT_MAX;

    for (auto n : surface.nodeMap) {
        double d = (qp - n.first).norm();
        if (d < minDist) {
            minDist = d;
        }
    }

    for (auto t : *surface.triangles) {
        minDist = std::min(minDist, Utils::distPointLine(qp, t->v1->p, t->v2->p));
        minDist = std::min(minDist, Utils::distPointLine(qp, t->v2->p, t->v3->p));
        minDist = std::min(minDist, Utils::distPointLine(qp, t->v3->p, t->v1->p));

        Vec ax = t->v1->p - qp;
        Vec n = cross((t->v2->p - t->v1->p), (t->v3->p - t->v1->p)).normalized();
        double d = fabs(dot(ax, n));
        minDist = std::min(minDist, d);

    }

    return (surface.isInside(qp))? -minDist : minDist;
}

// Linearly interpolates the position where an isosurface cuts
// an edge between two vertices
//---------------------------------------------------------------------------
Vec Polygonizer::vertexInterp(double isolevel, Vec p1, Vec p2, double val1, double val2) {
//---------------------------------------------------------------------------

/* isolevel -- value of the isosurface
p1 -- cube vertex 1
p2 -- cube vertex 2
val1 -- scalar value at p1
val2 -- scalar value at p2 */

    double rat = (isolevel - val1) / (val2 - val1);
    double x, y, z;

    x = rat * (p2[0] - p1[0]) + p1[0];
    y = rat * (p2[1] - p1[1]) + p1[1];
    z = rat * (p2[2] - p1[2]) + p1[2];

    return Vec(x, y, z);
}


// Given three triangle vertices
// Returns triangle normal
//---------------------------------------------------------------------------
Vec Polygonizer::findNormal(Vec v1, Vec v2, Vec v3) {
//---------------------------------------------------------------------------

    Vec n = cross((v2 - v1), (v3 - v2));
    return n.normalized();
}


bool Polygonizer::writeTrianglesToSTL(string path) {

    Vec n, p1, p2, p3;

    qDebug() << "Writing to Binary STL";
    if (this->polygon.empty()) {
        return false;
    }

    ofstream file;
    file.open(path, ios::out | ios::binary);
    if (!file.is_open()) {
        qDebug() << "Error opening output file";
        return false;
    }
    file.eof();

    Utils::STLHEADER header = Utils::STLHEADER();
    Utils::STLFACET facet = Utils::STLFACET();

    strcpy(header.description, "STL Generator / Creative Machines Lab (2019)");
    header.nfacets = int(this->polygon.size());
    file.write((char*)&header, sizeof(header));

    facet.pad = 0;

    qDebug() << "Wrote header description" << header.nfacets << sizeof(header.description) << sizeof(header.nfacets);

    for (int i = 0; i < this->polygon.size(); i++) {

        n = this->polygon[i].n;
        p1 = this->polygon[i].p[0];
        p2 = this->polygon[i].p[1];
        p3 = this->polygon[i].p[2];

        facet.x1 = float(p1[0]);
        facet.y1 = float(p1[1]);
        facet.z1 = float(p1[2]);
        facet.x2 = float(p2[0]);
        facet.y2 = float(p2[1]);
        facet.z2 = float(p2[2]);
        facet.x3 = float(p3[0]);
        facet.y3 = float(p3[1]);
        facet.z3 = float(p3[2]);
        facet.nx = float(n[0]);
        facet.ny = float(n[1]);
        facet.nz = float(n[2]);

        file.write((char *)&facet, 50);
    }

    qDebug() << sizeof(facet);

    file.close();
    return  true;
}


bool Polygonizer::writeTrianglesToASTL(string path) {

    Vec n, p1, p2, p3;

    if (this->polygon.empty()) {
        return false;
    }

    ofstream file;
    file.open(path, ios::out | ios::binary);
    if (!file.is_open()) {
        return false;
    }
    file.eof();

    file << "solid polygon" << endl;

    int lowX = 0;
    int lowY = 0;
    int lowZ = 0;

    for (int i = 0; i < this->polygon.size(); i++) {
        n = this->polygon[i].n;
        p1 = this->polygon[i].p[0];
        p2 = this->polygon[i].p[1];
        p3 = this->polygon[i].p[2];

        if (p1[0] - barModel->bounds.minCorner[0] < 0.0001) lowX++;
        if (p2[0] - barModel->bounds.minCorner[0] < 0.0001) lowX++;
        if (p3[0] - barModel->bounds.minCorner[0] < 0.0001) lowX++;
        if (p1[1] - barModel->bounds.minCorner[1] < 0.0001) lowY++;
        if (p2[1] - barModel->bounds.minCorner[1] < 0.0001) lowY++;
        if (p3[1] - barModel->bounds.minCorner[1] < 0.0001) lowY++;
        if (p1[2] - barModel->bounds.minCorner[2] < 0.0001) lowZ++;
        if (p2[2] - barModel->bounds.minCorner[2] < 0.0001) lowZ++;
        if (p3[2] - barModel->bounds.minCorner[2] < 0.0001) lowZ++;


        file << "  facet normal " << float(n[0]) << " " << float(n[1]) << " " << float(n[2]) << endl;
        file << "    outer loop" << endl;
        file << "      vertex " << float(p1[0]) << " " << float(p1[1]) << " " << float(p1[2]) << endl;
        file << "      vertex " << float(p2[0]) << " " << float(p2[1]) << " " << float(p2[2]) << endl;
        file << "      vertex " << float(p3[0]) << " " << float(p3[1]) << " " << float(p3[2]) << endl;
        file << "    endloop" << endl;
        file << "  endfacet" << endl;
    }

    file << "endsolid polygon" << endl;

    file.close();

    qDebug() << "LOWS" << lowX << lowY << lowZ;
    return true;
}


bool Polygonizer::writePolygonToSTL(string path){

    qDebug() << "Fixing normals";
    int fixed = geometry.fixNormals();
    qDebug() << "Fixed" << fixed << "normals";

    Vec n, p1, p2, p3;

    qDebug() << "Writing to Binary STL";
    if (this->geometry.triangles->empty()) {
        return false;
    }

    ofstream file;
    file.open(path, ios::out | ios::binary);
    if (!file.is_open()) {
        qDebug() << "Error opening output file";
        return false;
    }
    file.eof();

    Utils::STLHEADER header = Utils::STLHEADER();
    Utils::STLFACET facet = Utils::STLFACET();

    strcpy(header.description, "STL Generator / Creative Machines Lab (2019)");
    header.nfacets = int(this->geometry.triangles->size());
    file.write((char*)&header, sizeof(header));

    facet.pad = 0;

    qDebug() << "Wrote header description" << header.nfacets << sizeof(header.description) << sizeof(header.nfacets);

    for (auto & i : *this->geometry.triangles) {

        /**qDebug() << "Attempting to write" << &i << (i == nullptr);
        qDebug() << "Attempting to write n" << i->n[0];
        qDebug() << "Attempting to write v1" << i->v1->p[0] << i->v1->p[1] << i->v1->p[2];
        qDebug() << "Attempting to write v2" << i->v2->p[0] << i->v2->p[1] << i->v2->p[2];
        qDebug() << "Attempting to write v3" << i->v3->p[0] << i->v3->p[1] << i->v3->p[2];**/

        if (i != nullptr) {
            Tri triangle = *i;

            n = triangle.n;
            // Convert to millimeters
            p1 = triangle.v1->p * 1000;
            p2 = triangle.v2->p * 1000;
            p3 = triangle.v3->p * 1000;

            facet.x1 = float(p1[0]);
            facet.y1 = float(p1[1]);
            facet.z1 = float(p1[2]);
            facet.x2 = float(p2[0]);
            facet.y2 = float(p2[1]);
            facet.z2 = float(p2[2]);
            facet.x3 = float(p3[0]);
            facet.y3 = float(p3[1]);
            facet.z3 = float(p3[2]);
            facet.nx = float(n[0]);
            facet.ny = float(n[1]);
            facet.nz = float(n[2]);

            file.write((char *) &facet, 50);
        }

    }

    qDebug() << sizeof(facet);
    std::cout << "Wrote mesh to binary STL [" << path << "]\n\n";

    file.close();
    return  true;
}

// Prints progress of a marching cubes thread
void Polygonizer::printProgress(int sidx) {
    int line = 5 + sidx - 1;
    printf("\033[%d;1f", line);
    cout << "\033[0K" << "\033[92mSegment " << std::setw(2) << std::setfill(' ')<< sidx << " \033[97m[";

    float progress = float(completed[sidx]) / cubes[sidx];
    int pos = 40 * progress;
    for (int i = 0; i < 40; i++) {
        if (i < pos) cout << "=";
        else if (i == pos) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(progress * 100) << "%" << std::endl;
}