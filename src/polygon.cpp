#include <utility>

//
// Created by sw3390 on 4/4/19.
//

#include "polygon.h"

// == operator for Nodes
// ------------------------------------------------------------
bool Node::operator==(const Node &node) {
// ------------------------------------------------------------
    return (this->p == node.p);
}

// < operator for Nodes
// ------------------------------------------------------------
bool Node::operator<(const Node &node) {
// ------------------------------------------------------------
    return (this->p[0] < node.p[0] || (this->p[0] <= node.p[0] && this->p[1] < node.p[1]))
    || (this->p[0] <= node.p[0] && this->p[1] <= node.p[1] && this->p[2] < node.p[2]);
}

// Add node to polygon, returns ptr if existing
// ------------------------------------------------------------
shared_ptr<Node> Polygon::addNode(Vec p) {
// ------------------------------------------------------------

    if (nodeMap.find(p) == nodeMap.end()) {
        shared_ptr<Node> n = make_shared<Node>(Node(p));
        n->index = nodeMap.size(); // Update index
        nodeMap[p] = move(n);
    }
    return nodeMap[p];

}


// Add triangle to polygon
// ------------------------------------------------------------
void Polygon::addTriangle(Vec v1, Vec v2, Vec v3, Vec n) {
// ------------------------------------------------------------

    shared_ptr<Node> n1 = nullptr, n2 = nullptr, n3 = nullptr;

    // Add or find nodes
    n1 = addNode(v1);
    n2 = addNode(v2);
    n3 = addNode(v3);

    // Add triangle
    addTriangle(n1, n2, n3, n);
}


// Add triangle to polygon from existing nodes
// ------------------------------------------------------------
void Polygon::addTriangle(shared_ptr<Node> n1, shared_ptr<Node> n2, shared_ptr<Node> n3, Vec n) {
// ------------------------------------------------------------

    // Add triangle
    shared_ptr<Tri> tri = make_shared<Tri>(Tri());
    tri->v1 = n1;
    tri->v2 = n2;
    tri->v3 = n3;
    tri->n = n;
    triangles->push_back(move(tri));
    tri = triangles->back();

    // Add triangle to nodes
    n1->tris.push_back(tri);
    n2->tris.push_back(tri);
    n3->tris.push_back(tri);
}

// Remove triangle from polygon
// -----------------------------------------------------------
void Polygon::removeTriangle(shared_ptr<Tri> t) {
// ------------------------------------------------------------

    qDebug() << "Going to remove" << t->v1->p[0] << t->v1->p[1] << t->v1->p[2] << t->v2->p[0] << t->v2->p[1] << t->v2->p[2] << t->v3->p[0] << t->v3->p[1] << t->v3->p[2];
    t->v1->removeTri(t);
    t->v2->removeTri(t);
    t->v3->removeTri(t);
    qDebug() << "Removed node tris" << t->v1->p[0] << t->v1->p[1] << t->v1->p[2];

    triangles->erase(remove(triangles->begin(), triangles->end(), t), triangles->end());
    qDebug() << "Removed triangle";

}

// Merges current polygon with given polygon
// Uses current polygon as combined model
// ------------------------------------------------------------
void Polygon::mergePolygons(const Polygon &polygon) {
// ------------------------------------------------------------

    auto nodesToAdd = vector<shared_ptr<Node>>();
    for (auto nodePair : polygon.nodeMap) {

        // Get node in this Polygon's node map
        shared_ptr<Node> nodeInMap = nodeMap[nodePair.first];

        if (nodeInMap != nullptr) {

            // Set triangle connections
            for (const auto &tri : nodePair.second->tris) {
                if (tri->v1 == nodePair.second) {
                    tri->v1 = nodeInMap;
                }
                if (tri->v2 == nodePair.second) {
                    tri->v2 = nodeInMap;
                }
                if (tri->v3 == nodePair.second) {
                    tri->v3 = nodeInMap;
                }
                nodeInMap->tris.push_back(tri);
            }
        } else {

            // Add node
            this->nodeMap[nodePair.first] = move(nodePair.second);
        }
    }

    // Add triangles
    triangles->insert(triangles->end(), polygon.triangles->begin(), polygon.triangles->end());
}


// Checks normal consistency and fixes bad normals
// ------------------------------------------------------------
int Polygon::fixNormals() {
// ------------------------------------------------------------

    int flipcnt = 0;

    for (int k = 0; k  < 10; k++) {
        int fixed = 0;

        for (auto t : *triangles) {
            int neighbors = 0;
            int inconsistent = 0;

            // Remove if degenerate
            if (t == nullptr || t->v1 == nullptr || t->v2 == nullptr || t->v3 == nullptr) {
                triangles->erase(remove(triangles->begin(), triangles->end(), t), triangles->end());
                continue;
            }

            /**for (auto t1 : t->v1->tris) {
                if (t1 != t && sharedNodes(*t, *t1) == 2) {
                    neighbors++;
                    if (!isNormalConsistent(t, t1)) {
                        inconsistent++;
                    }
                }
            }
            for (auto t2 : t->v2->tris) {
                if (t2 != t && sharedNodes(*t, *t2) == 2) {
                    neighbors++;
                    if (!isNormalConsistent(t, t2)) {
                        inconsistent++;
                    }
                }
            }
            for (auto t3 : t->v3->tris) {
                if (t3 != t && sharedNodes(*t, *t3) == 2) {
                    neighbors++;
                    if (!isNormalConsistent(t, t3)) {
                        inconsistent++;
                    }
                }
            }

            // If inconsistent with most neighbors, flip
            if (neighbors > 1 && (inconsistent > neighbors / 4)) {
                t->n = -t->n;
                flipcnt++;
                fixed++;
            }**/
        }
        qDebug() << "Fixed" << fixed;
        if (fixed == 0) {
            break;
        }
    }

    // Make sure triangles are arranged clockwise
    for (auto t : *triangles) {
        Vec ax1 = (t->v2->p - t->v1->p).normalized();
        Vec ax2 = (t->v3->p - t->v2->p).normalized();
        Vec norm = cross(ax1, ax2).normalized();
        if (dot(norm, t->n) < 0) {
            shared_ptr<Node> c = t->v1;
            t->v1 = t->v3;
            t->v3 = c;
            c.reset();
        }
    }

    // Remove degenerates
    for (auto t : *triangles) {
        if (t == nullptr) {
            qDebug() << "Found null degenerate";
            triangles->erase(remove(triangles->begin(), triangles->end(), t), triangles->end());
            continue;
        }
        if (t->v1 == t->v2 || t->v2 == t->v3 || t->v3 == t->v1) {
            removeTriangle(t);
            qDebug() << "Found degenerate";
        }
    }
    qDebug() << "End of fix normals";
    return flipcnt;
}

// Finds coplanar triangles and attempts to combine them
// ------------------------------------------------------------
void Polygon::reduceMesh(double eps, double factor) {
// ------------------------------------------------------------

    int origSize = this->triangles->size();
    int totalReduced = 0;
    shared_ptr<Node> n0;

    RESTART:
    int simplified = 0;
    flipTriangleEdges(eps);

    for (const auto &nodePair : this->nodeMap) {

        vector<shared_ptr<Node>> nodeQuad = vector<shared_ptr<Node>>();
        vector<shared_ptr<Tri>> triQuad = vector<shared_ptr<Tri>>(4);
        vector<int> nodeCounts = vector<int>(5);

        // Find nodes attached to 4 triangles
        if (nodePair.second->tris.size() == 4) {

            n0 = nodePair.second;
            nodeQuad.push_back(n0);
            nodeCounts[0] = 4;

            for (int i = 0; i < 4; i++) {
                shared_ptr<Tri> t = n0->tris[i];
                triQuad[i] = t;

                auto i1 = find(nodeQuad.begin(), nodeQuad.end(), t->v1);
                auto i2 = find(nodeQuad.begin(), nodeQuad.end(), t->v2);
                auto i3 = find(nodeQuad.begin(), nodeQuad.end(), t->v3);

                if (i1 == nodeQuad.end() || i1 - nodeQuad.begin() >= nodeQuad.size()) {
                    nodeCounts[nodeQuad.size()]++;
                    nodeQuad.push_back(t->v1);
                } else {
                    nodeCounts[i1 - nodeQuad.begin()]++;
                }
                if (i2 == nodeQuad.end() || i2 - nodeQuad.begin() >= nodeQuad.size()) {
                    nodeCounts[nodeQuad.size()]++;
                    nodeQuad.push_back(t->v2);
                } else {
                    nodeCounts[i2 - nodeQuad.begin()]++;
                }
                if (i3 == nodeQuad.end() || i3 - nodeQuad.begin() >= nodeQuad.size()) {
                    nodeCounts[nodeQuad.size()]++;
                    nodeQuad.push_back(t->v3);
                } else {
                    nodeCounts[i3 - nodeQuad.begin()]++;
                }

            }

            // Check that we found 5 nodes, one attached to all 4 triangles,
            // and the rest attached to 2 relevant triangles each
            int scnt = 0;

            for (int i = 0; i < 5; i++) {
                if (nodeCounts[i] != 2 && nodeCounts[i] != 4) {
                    continue;
                }
                scnt += nodeCounts[i];
            }
            if (scnt != 12) {
                // Sum should be 12
                //qDebug() << "Total node count" << scnt;
                continue;
            }

            qDebug() << "Found quad node";

            // Attempt to consolidate triangles
            if (sharedNodes(*triQuad[0], *triQuad[1]) == 2 && sharedNodes(*triQuad[2], *triQuad[3]) == 2) {
                if (consolidateTriangle(triQuad[0], triQuad[1], n0, eps)) simplified++;
                if (consolidateTriangle(triQuad[2], triQuad[3], n0, eps)) simplified++;
            } else if (sharedNodes(*triQuad[0], *triQuad[2]) == 2 && sharedNodes(*triQuad[1], *triQuad[3]) == 2) {
                if (consolidateTriangle(triQuad[0], triQuad[2], n0, eps)) simplified++;
                if (consolidateTriangle(triQuad[1], triQuad[3], n0, eps)) simplified++;
            }

            qDebug() << "Simplified" << simplified;
        }

    }

    totalReduced += simplified*2;

    if (1.0*totalReduced/origSize < factor && simplified > 0) {
        goto RESTART;
    }
}


// Swaps diagonal of adjacent coplanar triangles
// ------------------------------------------------------------
void Polygon::flipTriangleEdges(double eps) {
// ------------------------------------------------------------

    qDebug() << "Flipping diagonals";
    for (shared_ptr<Tri> tri1 : *this->triangles) {

        // Search adjacent triangles
        vector<shared_ptr<Tri>> adjTris = tri1->v1->tris;
        adjTris.insert(adjTris.end(), tri1->v2->tris.begin(), tri1->v2->tris.end());
        adjTris.insert(adjTris.end(), tri1->v3->tris.begin(), tri1->v3->tris.end());

        for (shared_ptr<Tri> tri2 : adjTris) {
            if (sharedNodes(*tri1, *tri2) == 2) {

                shared_ptr<Node> ncom1 = nullptr, ncom2 = nullptr, nsep1 = nullptr, nsep2 = nullptr;

                findTwoCommon(*tri1, *tri2, ncom1, ncom2, nsep1, nsep2);

                if (Utils::areCloseToCoplanar(ncom1->p, nsep1->p, ncom2->p, nsep2->p, eps) &&
                    !Utils::areCloseToColinear(nsep1->p, nsep2->p, ncom1->p, 0.01) &&
                    !Utils::areCloseToColinear(nsep1->p, nsep2->p, ncom2->p, 0.01)) {

                    Vec normalAvg = (tri1->n + tri2->n) * 0.5;

                    tri1->v1 = nsep1;
                    tri1->v2 = nsep2;
                    tri1->v3 = ncom1;
                    tri2->v1 = nsep1;
                    tri2->v2 = nsep2;
                    tri2->v3 = ncom2;
                    tri1->n = normalAvg;
                    tri2->n = normalAvg;

                    if (find(nsep1->tris.begin(), nsep1->tris.end(), tri1) == nsep1->tris.end()) {
                        nsep1->tris.push_back(tri1);
                    } else {
                        nsep2->tris.push_back(tri1);
                    }
                    if (find(nsep1->tris.begin(), nsep1->tris.end(), tri2) == nsep1->tris.end()) {
                        nsep1->tris.push_back(tri2);
                    } else {
                        nsep2->tris.push_back(tri2);
                    }

                    ncom1->removeTri(tri2);
                    ncom2->removeTri(tri1);
                }
            }
        }
    }
}


// Calls Util functions to read STL in vertex vectors
// Converts vertex vectors into Polygon
// ------------------------------------------------------------
void Polygon::createPolygonFromFile(string path, float scale) {
// ------------------------------------------------------------

    // Check that this is an empty Polygon
    assert(this->triangles->empty());
    assert(this->nodeMap.empty());

    // Vertex vectors
    vector<Vec> vs = vector<Vec>(); // Every 3 vertices constitutes a triangle
    vector<Vec> ns = vector<Vec>(); // Normals are copied to correspond to vertices

    Utils::createModelFromFile(std::move(path), scale, vs, ns);
    assert(vs.size() == ns.size());

    // Fill Polygon data structures
    for (int i = 0; i < vs.size(); i+=3) {
        addTriangle(vs[i], vs[i+1], vs[i+2], ns[i]);
    }

}


// Convertes Polygon into float arrays with position and normal values
// Arranged sequentially
// ------------------------------------------------------------
void Polygon::createGraphicsData(float *vs, float *ns) {
// ------------------------------------------------------------

    // Reallocate arrays
    delete vs;
    delete ns;
    vs = new float[triangles->size() * 3 * 3];
    ns = new float[triangles->size() * 3 * 3];

    int vn = 0;
    for (const auto &t : *this->triangles) {

        for (int i = 0; i < 3; i++) {
            vs[vn * 3 + i] = t->v1->p.data[i];
        }
        for (int j = 0; j < 3; j++) {
            ns[vn * 3 + j] = t->n[j];
        }
        vn++;
        for (int i = 0; i < 3; i++) {
            vs[vn * 3 + i] = t->v2->p.data[i];
        }
        for (int j = 0; j < 3; j++) {
            ns[vn * 3 + j] = t->n[j];
        }
        vn++;
        for (int i = 0; i < 3; i++) {
            vs[vn * 3 + i] = t->v3->p.data[i];
        }
        for (int j = 0; j < 3; j++) {
            ns[vn * 3 + j] = t->n[j];
        }
        vn ++;

    }

}

// Clear nodes and triangles from polygon
// ------------------------------------------------------------
void Polygon::clearPolygon() {
// ------------------------------------------------------------
    for (auto n : nodeMap) {
        for (auto t : n.second->tris) {
            t.reset();
        }
        n.second.reset();
    }
    for (auto t : *triangles) {
        t->v1.reset();
        t->v2.reset();
        t->v3.reset();
        t.reset();
    }
    nodeMap.clear();
    triangles->clear();
}

// Returns the UNION of two Polygons
// Gang Mei and John C. Tipper "Simple and Robust Boolean Operations for Triangulated Surfaces" (2013)
// ------------------------------------------------------------
void Polygon::unionPolygons(Polygon &other, Polygon &polygon) {
// ------------------------------------------------------------

    Vec bmin1, bmax1, bmin2, bmax2;
    this->boundingPoints(bmin1, bmax1);
    other.boundingPoints(bmin2, bmax2);

    // Find overlapping volumes
    Vec imin, imax;
    imin = Vec(std::max(bmin1[0], bmin2[0]), std::max(bmin1[1], bmin2[1]), std::max(bmin1[2], bmin2[2]));
    imax = Vec(std::min(bmax1[0], bmax2[0]), std::min(bmax1[1], bmax2[1]), std::min(bmax1[2], bmax2[2]));

    qDebug() << "Intersection box" << imin[0] << imin[1] << imin[2] << "x" << imax[0] << imax[1] << imax[2];

    vector<shared_ptr<Tri>> trisInIntersection1 = vector<shared_ptr<Tri>>();
    vector<shared_ptr<Tri>> trisInIntersection2 = vector<shared_ptr<Tri>>();
    for (const auto &t : *this->triangles) {

        if (withinBounds(imin, imax, *t)) {
            trisInIntersection1.push_back(t);
        }

    }
    for (const auto &t : *other.triangles) {

        if (withinBounds(imin, imax, *t)) {
            trisInIntersection2.push_back(t);
        }

    }
    qDebug() << "There are" << trisInIntersection1.size() + trisInIntersection2.size() << "triangles in intersection range";

    for (const auto &t1 : trisInIntersection1) {
        for (const auto &t2 : trisInIntersection2) {

        }
    }
}

// Returns true if given point is inside the convex geometry
// ------------------------------------------------------------
bool Polygon::isInside(Vec point) {
// ------------------------------------------------------------

    int intersections = 0;

    Vec p;
    Vec dir = Vec(Utils::randUnit(), Utils::randUnit(), Utils::randUnit()).normalized();

    for (auto t : *triangles) {

        bool ip = intersectPlane(*t, point, dir, p);
        if (ip && p[0] > point[0] && p[1] > point[1] && p[2] > point[2]) {
            if (Utils::insideTriangle(t->v1->p, t->v2->p, t->v3->p, p)) {
                intersections++;
            }
        }
    }

    return (intersections % 2 == 1);
}

// Returns true if given point is within some eps of the edge
// of the convex geometry
// ------------------------------------------------------------
bool Polygon::isCloseToEdge(const Vec &point, double eps) {
// ------------------------------------------------------------

    for (const auto &n : nodeMap) {

        if ((n.first - point).norm() <= eps) {
            return true;
        }
    }

    return false;
}

// Returns minimum and maximum corner
// Of the geometry's bounding box
// ------------------------------------------------------------
void Polygon::boundingPoints(Vec &minp, Vec &maxp){
// ------------------------------------------------------------

    Vec minCorner = Vec(DBL_MAX, DBL_MAX, DBL_MAX);
    Vec maxCorner = Vec(-DBL_MAX, -DBL_MAX, -DBL_MAX);

    for (auto n : nodeMap) {
        Vec p = n.first;
        minCorner[0] = std::min(minCorner[0], p[0]);
        minCorner[1] = std::min(minCorner[1], p[1]);
        minCorner[2] = std::min(minCorner[2], p[2]);
        maxCorner[0] = std::max(maxCorner[0], p[0]);
        maxCorner[1] = std::max(maxCorner[1], p[1]);
        maxCorner[2] = std::max(maxCorner[2], p[2]);
    }

    minp = minCorner;
    maxp = maxCorner;
}


// Returns whether a point is within some dimensional bounds (inclusive)
// ------------------------------------------------------------
bool Polygon::withinBounds(const Vec &bmin, const Vec &bmax, const Vec &p) {
// ------------------------------------------------------------

    bool inside = true;
    if (p[0] > bmax[0] && p[0] < bmin[0]) {
        inside = false;
    }
    if (p[1] > bmax[1] && p[1] < bmin[1]) {
        inside = false;
    }
    if (p[2] > bmax[2] && p[2] < bmin[2]) {
        inside = false;
    }
    return inside;

}

// Returns whether a triangle is within some dimensional bounds (inclusive)
// ------------------------------------------------------------
bool Polygon::withinBounds(const Vec &bmin, const Vec &bmax, const Tri &t) {
// ------------------------------------------------------------

    bool inside = false;
    if (withinBounds(bmin, bmax, t.v1->p)) {
        inside = true;
    }
    if (withinBounds(bmin, bmax, t.v2->p)) {
        inside = true;
    }
    if (withinBounds(bmin, bmax, t.v3->p)) {
        inside = true;
    }

    return inside;

}


// Calculate intersecion point of a ray and a triangle plane
// ray has origin o and direction dir
// if there is no intersection, return false
// see http://www.devmaster.net/wiki/Ray-triangle_intersection
// ------------------------------------------------------------
bool Polygon::intersectPlane(const Tri &t, Vec o, Vec dir, Vec &p) {
// ------------------------------------------------------------

    double EPSILON = 1E-5;
    assert(fabs(fabs(dir.norm()) - 1) < EPSILON); // dir must be a unit vector

    Vec a = t.v1->p;
    Vec b = t.v2->p;
    Vec c = t.v3->p;
    Vec ba = b - a;
    Vec ca = c - a;
    Vec n = cross(ba, ca); // triangle normal unit vector
    n = n.normalized();

    double dn = dot(dir, n);
    if (fabs(dn) < EPSILON) {
        return false; // ray is parallel to the triangle
    }

    double dist = -dot((o - a), n) / dn;
    p = o + dir * dist;

    return true;
}


// Returns number of nodes shared by two triangles
// ------------------------------------------------------------
int Polygon::sharedNodes(const Tri &t1, const Tri &t2) {
// ------------------------------------------------------------

    int shared = 0;

    if (t1.v1 == t2.v1) shared++;
    if (t1.v1 == t2.v2) shared++;
    if (t1.v1 == t2.v3) shared++;
    if (t1.v2 == t2.v1) shared++;
    if (t1.v2 == t2.v2) shared++;
    if (t1.v2 == t2.v3) shared++;
    if (t1.v3 == t2.v1) shared++;
    if (t1.v3 == t2.v2) shared++;
    if (t1.v3 == t2.v3) shared++;

    return shared;
}

// For cases where two triangles share an edge
// Sets common nodes and separate nodes
// ------------------------------------------------------------
void Polygon::findTwoCommon(const Tri &t1, const Tri &t2,
        shared_ptr<Node> &ncom1, shared_ptr<Node> &ncom2,
        shared_ptr<Node> &nsep1, shared_ptr<Node> &nsep2) {
// ------------------------------------------------------------

    // Find common vertices
    if (t1.v1 == t2.v1 || t1.v1 == t2.v2 || t1.v1 == t2.v3) {
        ncom1 = t1.v1;
    }
    if (t1.v2 == t2.v1 || t1.v2 == t2.v2 || t1.v2 == t2.v3) {
        if (ncom1 == nullptr) {
            ncom1 = t1.v2;
        } else {
            ncom2 = t1.v2;
        }
    }
    if (t1.v3 == t2.v1 || t1.v3 == t2.v2 || t1.v3 == t2.v3) {
        ncom2 = t1.v3;
    }

    // Find separate vertices (one per triangle)
    if (t1.v1 != ncom1 && t1.v1 != ncom2) {
        nsep1 = t1.v1;
    }
    if (t1.v2 != ncom1 && t1.v2 != ncom2) {
        nsep1 = t1.v2;
    }
    if (t1.v3 != ncom1 && t1.v3 != ncom2) {
        nsep1 = t1.v3;
    }

    if (t2.v1 != ncom1 && t2.v1 != ncom2) {
        nsep2 = t2.v1;
    }
    if (t2.v2 != ncom1 && t2.v2 != ncom2) {
        nsep2 = t2.v2;
    }
    if (t2.v3 != ncom1 && t2.v3 != ncom2) {
        nsep2 = t2.v3;
    }

}

// Consolidates a pair of triangles that share an edge
// and colinearity of a joined edge defined by eps
// centerNode is the center node of the colinear edge
// ------------------------------------------------------------
bool Polygon::consolidateTriangle(shared_ptr<Tri> t1, shared_ptr<Tri> t2,
        shared_ptr<Node> centerNode, double eps) {
// ------------------------------------------------------------

    shared_ptr<Node> ncom1 = nullptr, ncom2 = nullptr, nsep1 = nullptr, nsep2 = nullptr;

    findTwoCommon(*t1, *t2, ncom1, ncom2, nsep1, nsep2);
    if (Utils::distPointLine(centerNode->p, nsep1->p, nsep2->p) < eps) {

        Vec normalAvg = (t1->n + t2->n) * 0.5;

        shared_ptr<Node> out = (ncom1 == centerNode) ? ncom2 : ncom1;
        this->addTriangle(nsep1, nsep2, out, normalAvg);
        this->removeTriangle(t1);
        this->removeTriangle(t2);

        return true;
    }

    return false;
}

// Checks normals between two neighboring triangles
// Returns true if normals are consistent
// ------------------------------------------------------------
bool Polygon::isNormalConsistent(const shared_ptr<Tri> &t1, const shared_ptr<Tri> &t2) {
// ------------------------------------------------------------

    assert(sharedNodes(*t1, *t2) > 1);

    shared_ptr<Node> ncom1 = nullptr, ncom2 = nullptr, nsep1 = nullptr, nsep2 = nullptr;

    findTwoCommon(*t1, *t2, ncom1, ncom2, nsep1, nsep2);

    Vec jax = (ncom2->p - ncom1->p).normalized();
    Vec t1n = cross(jax, ncom2->p - nsep1->p).normalized();
    double t1s = dot(t1n, t1->n);
    Vec t2n = cross(ncom2->p, nsep2->p).normalized();
    double t2s = dot(t2n, t2->n);

    return (t1s * t2s < 0);
}
