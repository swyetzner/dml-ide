//
// Created by sw3390 on 4/4/19.
//

#ifndef DMLIDE_POLYGON_H
#define DMLIDE_POLYGON_H

#include <QDebug>
#include <unordered_map>

#include "utils.h"

class Node;
class Tri;
class Polygon;


struct vec_hash {
    size_t operator()(const Vec &vec) const {
        return hash<double>()(vec[0]) ^ hash<double>()(vec[1]) ^ hash<double>()(vec[2]);
    }
};

class Node {

public:
    Vec p;
    vector<shared_ptr<Tri>> tris;

    Node() = default;
    explicit Node(const Vec &p) { this->p = p; }

    bool operator==(const Node &n);
    bool operator<(const Node &n);

    void removeTri(shared_ptr<Tri> t) {
        tris.erase(remove(tris.begin(), tris.end(), t), tris.end());
    }
};

class Tri {

public:
    Vec n;
    shared_ptr<Node> v1;
    shared_ptr<Node> v2;
    shared_ptr<Node> v3;

    Tri() = default;
};

class Polygon {

public:
    Polygon() {
        triangles = new vector<shared_ptr<Tri>>();
    }

    ~Polygon() {
        delete triangles;
        nodeMap.clear();
    }

    void addTriangle(Vec v1, Vec v2, Vec v3, Vec n);
    void addTriangle(shared_ptr<Node> n1, shared_ptr<Node> n2, shared_ptr<Node> n3, Vec n);
    void removeTriangle(shared_ptr<Tri> t);
    void mergePolygons(const Polygon &polygon);
    void reduceMesh(double eps, double factor);
    void flipTriangleEdges(double eps);
    int fixNormals();

    int sharedNodes(const Tri &t1, const Tri &t2);
    void findTwoCommon(const Tri &t1, const Tri &t2, shared_ptr<Node> &ncom1, shared_ptr<Node> &ncom2,
                       shared_ptr<Node> &nsep1, shared_ptr<Node> &nsep2);
    bool consolidateTriangle(shared_ptr<Tri> t1, shared_ptr<Tri> t2, shared_ptr<Node> centerNode, double eps);
    bool isNormalConsistent(const shared_ptr<Tri> &t1, const shared_ptr<Tri> &t2);

    unordered_map<Vec, shared_ptr<Node>, vec_hash> nodeMap;
    vector<shared_ptr<Tri>> *triangles;
};


#endif //DMLIDE_POLYGON_H
