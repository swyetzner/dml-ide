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

// Add triangle to polygon
// ------------------------------------------------------------
void Polygon::addTriangle(Vec v1, Vec v2, Vec v3, Vec n) {
// ------------------------------------------------------------

    shared_ptr<Node> n1 = nullptr, n2 = nullptr, n3 = nullptr;

    // Search for existing nodes
    n1 = nodeMap[v1];
    n2 = nodeMap[v2];
    n3 = nodeMap[v3];

    // Add nodes if not found
    if (n1 == nullptr) {
        n1 = make_shared<Node>(Node(v1));
        nodeMap[v1] = move(n1);
        n1 = nodeMap[v1];
    }
    if (n2 == nullptr) {
        n2 = make_shared<Node>(Node(v2));
        nodeMap[v2] = move(n2);
        n2 = nodeMap[v2];
    }
    if (n3 == nullptr) {
        n3 = make_shared<Node>(Node(v3));
        nodeMap[v3] = move(n3);
        n3 = nodeMap[v3];
    }

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

    t->v1->removeTri(t);
    t->v2->removeTri(t);
    t->v3->removeTri(t);

    triangles->erase(remove(triangles->begin(), triangles->end(), t), triangles->end());
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
        if (t->v1 == t->v2 || t->v2 == t->v3 || t->v3 == t->v1) {
            removeTriangle(t);
            qDebug() << "Found degenerate";
        }
    }

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