#include "utils.h"

#include <QDebug>

#define EPSILON 1E-5

Utils::Utils()
{

}

inline bool Utils::startsWith(const string &s, const string &p) {
    if (p.size() > s.size()) {
        return false;
    }
    return equal(p.begin(), p.end(), s.begin());
}

inline bool Utils::endsWith(const string &s, const string &e) {
    if (e.size() > s.size()) {
        return false;
    }
    return equal(e.rbegin(), e.rend(), s.rbegin());
}

string Utils::left(const string &s, int n) {
    char buffer[n];
    s.copy(buffer, n);
    string t = buffer;
    return t;
}

string Utils::trim(string &s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](char c) {
        return !isspace(c);
    }));
    return  s;
}

float Utils::randFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

float Utils::randUnit() {
    return randFloat(0.0f, 1.0f);
}

vec3 Utils::randDirection() {
    return normalize(vec3(randUnit(), randUnit(), randUnit()));
}

Vec Utils::randDirectionVec() {
    return Vec(randFloat(-1, 1), randFloat(-1, 1), randFloat(-1, 1)).normalized();
}

vec3 Utils::randPoint(vec3 lowBound, vec3 highBound) {
    return glm::vec3(Utils::randFloat(lowBound.x, highBound.x),
                     Utils::randFloat(lowBound.y, highBound.y),
                     Utils::randFloat(lowBound.z, highBound.z));
}

Vec Utils::randPointVec(Vec lowBound, Vec highBound) {
    return Vec(Utils::randFloat(lowBound[0], highBound[0]),
             Utils::randFloat(lowBound[1], highBound[1]),
             Utils::randFloat(lowBound[2], highBound[2]));
}

// Clamps an input value from imin and imax to a range defined by omin and omax
double Utils::clamp(double imin, double imax, double omin, double omax, double input) {
    return omin + (omax - omin) * (input - imin) / (imax - imin);
}

// Interpolates value i bounded by [imin, imax] to range [omin, omax]
//---------------------------------------------------------------------------
double Utils::interpolate(double omin, double omax, double imin, double imax, double i) {
//---------------------------------------------------------------------------
    return omin + ((omax - omin) * (i - imin) / (imax - imin));
}

bool Utils::getPlaneEquation(Vec normal, Vec pt, double &d) {
    d = -(normal[0] * pt[0] + normal[1] * pt[1] + normal[2] * pt[2]);
    return true;
}

double Utils::getAngle(Vec v1, Vec v2) {
    return acos(dot(v1, v2) / (v1.norm() * v2.norm()));
}

bool Utils::isAcute(Vec v1, Vec v2) {
    double d = dot(v1, v2);
    return (d > 0);
}

Vec Utils::bisect(Vec p1, Vec p2) {
    Vec line = p2 - p1;
    double mag = line.norm();
    Vec dir = line.normalized();

    return p1 + (mag / 2) * dir;
}

// Given three triangle vertices
// Returns triangle normal
vec3 Utils::findNormal(vec3 v1, vec3 v2, vec3 v3) {
    vec3 n = cross((v2 - v1), (v3 - v2));
    return normalize(n);
}

// Given three triangle vertices
// Returns triangle normal
Vec Utils::findNormal(Vec v1, Vec v2, Vec v3) {
    Vec n = cross((v2 - v1), (v3 - v2));
    return n.normalized();
}

// Returns distance of point p from line p2-p1
double Utils::distPointLine(Vec p, Vec p1, Vec p2) {

    Vec line = p2 - p1;
    double l = line.norm();
    if (l <= 0) {
        return (p - p1).norm();
    }

    double d = dot((p - p1), line);
    Vec c = p1 + d * line;
    return (p - c).norm();
}

// Returns distance of point p from plane with normal n and offset o
// Sets v to projection point
double Utils::distPointPlane(Vec p, Vec n, double o, Vec &v) {

    Vec w = p - o * n;
    double dist = dot(w, n);
    v = p - dist*n;

    return dist;
}

// Check if a point is within rectangular bounds
bool Utils::inBounds(Vec p, Vec minc, Vec maxc) {

    bool ret = true;
    if (p[0] < minc[0] || p[0] > maxc[0]) ret = false;
    if (p[1] < minc[1] || p[1] > maxc[1]) ret = false;
    if (p[2] < minc[2] || p[2] > maxc[2]) ret = false;

    return ret;
}

// Check if three vertices are colinear (within eps)
bool Utils::areCloseToColinear(Vec p1, Vec p2, Vec p3, double eps) {

    Vec v1 = (p2 - p1).normalized();
    Vec v2 = (p3 - p2).normalized();

    return (fabs(cross(v1, v2).norm() <= eps));
}

// Check if four vertices are coplanar (within eps)
// See http://mathworld.wolfram.com/Coplanar.html
bool Utils::areCloseToCoplanar(Vec p1, Vec p2, Vec p3, Vec p4, double eps) {

    Vec v1 = (p2 - p1).normalized();
    Vec v2 = (p3 - p2).normalized();
    Vec v3 = (p4 - p3).normalized();
    Vec v4 = (p1 - p4).normalized();

    Vec c1 = cross(v1, v2).normalized();
    Vec c2 = cross(v2, v3).normalized();
    Vec c3 = cross(v3, v4).normalized();
    Vec c4 = cross(v4, v1).normalized();

    double d1 = 1 - fabs(dot(c1, c2));
    double d2 = 1 - fabs(dot(c2, c3));
    double d3 = 1 - fabs(dot(c3, c4));
    double d4 = 1 - fabs(dot(c4, c1));

    return (fabs(d1) < eps && fabs(d2) < eps && fabs(d3) < eps && fabs(d4) < eps);
}


// intersectTriangle(Vec *triangle, Vec o, Vec dir, Vec &p, double &pu, double &pv)
//
// Calculate intersecion point of a ray and a triangle plane
// ray has origin o and direction dir
// if there is no intersection, return false
// u and v are barcentric coordinates of the intersection point p = (1-u-v)A + uB + vC
// see http://www.devmaster.net/wiki/Ray-triangle_intersection
//
bool Utils::intersectPlane(Vec *triangle, Vec o, Vec dir, Vec &p, double &pu, double &pv) {
    assert(fabs(fabs(dir.norm()) - 1) < EPSILON); // dir must be a unit vector

    Vec a = triangle[0];
    Vec b = triangle[1];
    Vec c = triangle[2];
    Vec ba = b - a;
    Vec ca = c - a;
    Vec n = cross(ba, ca).normalized(); // triangle normal unit vector

    double dn = dot(dir, n);
    if (fabs(dn) < EPSILON)
        return false; // ray is parallel to the triangle

    double dist = -dot((o-a), n)/dn;
    p = o + dir*dist;

    pu = (p[1]*c[0] - p[0]*c[1])/(b[1]*c[0] - b[0]*c[1]);
    pv = (p[1]*b[0] - p[0]*b[1])/(c[1]*b[0] - c[0]*b[1]);

    return (pu >= 0) && (pv >= 0) && (pu + pv < 1);
}

// intersectTriangle(Vec *triangle, Vec o, Vec dir, Vec &p, double &pu, double &pv)
//
// Calculate intersecion point of a ray and a triangle plane
// ray has origin o and direction dir
// if there is no intersection, return false
// u and v are barcentric coordinates of the intersection point p = (1-u-v)A + uB + vC
// see http://www.devmaster.net/wiki/Ray-triangle_intersection
//
bool Utils::intersectPlane(vec3 *triangle, vec3 o, vec3 dir, vec3 &p, float &pu, float &pv) {

    assert(fabs(fabs(length(dir)) - 1) < EPSILON);

    vec3 a = triangle[0];
    vec3 b = triangle[1];
    vec3 c = triangle[2];
    vec3 n = cross((b - a), (c - a));
    n = normalize(n);


    float dn = dot(dir, n);

    if (fabs(dn) < EPSILON) {
        return false;
    }

    float dist = -dot((o-a), n)/dn;
    p = o + dir*dist;

    pu = (p[1]*c[0] - p[0]*c[1])/(b[1]*c[0] - b[0]*c[1]);
    pv = (p[1]*b[0] - p[0]*b[1])/(c[1]*b[0] - c[0]*b[1]);

    return true;

}



// True if point p projects to within triangle (v0;v1;v2)
// http://www.blackpawn.com/texts/pointinpoly/
bool Utils::insideTriangle(vec3 a, vec3 b, vec3 c, vec3 &point) {
    vec3 ca = c - a;
    vec3 ba = b - a;
    vec3 pa = point - a;

    float dotCACA = dot(ca, ca);
    float dotCABA = dot(ca, ba);
    float dotCAPA = dot(ca, pa);
    float dotBABA = dot(ba, ba);
    float dotBAPA = dot(ba, pa);

    float invDenom = 1.0f / (dotCACA * dotBABA - dotCABA * dotCABA);
    float u = (dotBABA * dotCAPA - dotCABA * dotBAPA) * invDenom;
    float v = (dotCACA * dotBAPA - dotCABA * dotCAPA) * invDenom;

    return (u >= 0.0f) && (v >= 0.0f) && (u + v < 1.0f);
}

// True if point p projects to within triangle (v0;v1;v2)
// http://www.blackpawn.com/texts/pointinpoly/
bool Utils::insideTriangle(Vec &a, Vec &b, Vec &c, Vec &point) {
    Vec ca = c - a;
    Vec ba = b - a;
    Vec pa = point - a;

    float dotCACA = dot(ca, ca);
    float dotCABA = dot(ca, ba);
    float dotCAPA = dot(ca, pa);
    float dotBABA = dot(ba, ba);
    float dotBAPA = dot(ba, pa);

    float invDenom = 1.0f / (dotCACA * dotBABA - dotCABA * dotCABA);
    float u = (dotBABA * dotCAPA - dotCABA * dotBAPA) * invDenom;
    float v = (dotCACA * dotBAPA - dotCABA * dotCAPA) * invDenom;

    return (u >= 0.0f) && (v >= 0.0f) && (u + v < 1.0f);
}

// createCube(vec3 center, float edgeLength, vector<vec3> &vs, vector<vec3> &ns)
//
// Populates a model with a cube.
// Creates triangles and normals from edges.
// Takes a center and an edgeLength.
//
void Utils::createCube(vec3 center, float edgeLength, vector<vec3> &vs, vector<vec3> &ns) {
    vs = vector<vec3>();
    ns = vector<vec3>();

    float halfEdge = edgeLength / 2;

    float left = center.x - halfEdge;
    float right = center.x + halfEdge;
    float front = center.y - halfEdge;
    float back = center.y + halfEdge;
    float bottom = center.z - halfEdge;
    float top = center.z + halfEdge;

    vec3 fll = vec3(left, front, bottom);
    vec3 flr = vec3(right, front, bottom);
    vec3 ful = vec3(left, front, top);
    vec3 fur = vec3(right, front, top);
    vec3 bll = vec3(left, back, bottom);
    vec3 blr = vec3(right, back, bottom);
    vec3 bul = vec3(left, back, top);
    vec3 bur = vec3(right, back, top);

    vec3 nleft = vec3(-1.0f, 0.0f, 0.0f);
    vec3 nright = vec3(1.0f, 0.0f, 0.0f);
    vec3 nfront = vec3(0.0f, -1.0f, 0.0f);
    vec3 nback = vec3(0.0f, 1.0f, 0.0f);
    vec3 nbottom = vec3(0.0f, 0.0f, -1.0f);
    vec3 ntop = vec3(0.0f, 0.0f, 1.0f);

    // Front face triangles
    vs.push_back(fll);    vs.push_back(flr);    vs.push_back(ful);
    ns.push_back(nfront); ns.push_back(nfront); ns.push_back(nfront);

    vs.push_back(fur);    vs.push_back(ful);    vs.push_back(flr);
    ns.push_back(nfront); ns.push_back(nfront); ns.push_back(nfront);

    // Left face triangles
    vs.push_back(bll);    vs.push_back(fll);    vs.push_back(bul);
    ns.push_back(nleft); ns.push_back(nleft); ns.push_back(nleft);

    vs.push_back(ful);    vs.push_back(bul);    vs.push_back(fll);
    ns.push_back(nleft); ns.push_back(nleft); ns.push_back(nleft);

    // Back face triangles
    vs.push_back(blr);    vs.push_back(bll);    vs.push_back(bur);
    ns.push_back(nback); ns.push_back(nback); ns.push_back(nback);

    vs.push_back(bul);    vs.push_back(bur);    vs.push_back(bll);
    ns.push_back(nback); ns.push_back(nback); ns.push_back(nback);

    // Right face triangles
    vs.push_back(flr);    vs.push_back(blr);    vs.push_back(fur);
    ns.push_back(nright); ns.push_back(nright); ns.push_back(nright);

    vs.push_back(bur);    vs.push_back(fur);    vs.push_back(blr);
    ns.push_back(nright); ns.push_back(nright); ns.push_back(nright);

    // Bottom face triangles
    vs.push_back(bll);    vs.push_back(blr);    vs.push_back(fll);
    ns.push_back(nbottom); ns.push_back(nbottom); ns.push_back(nbottom);

    vs.push_back(flr);    vs.push_back(fll);    vs.push_back(blr);
    ns.push_back(nbottom); ns.push_back(nbottom); ns.push_back(nbottom);

    // Top face triangles
    vs.push_back(ful);    vs.push_back(fur);    vs.push_back(bul);
    ns.push_back(ntop); ns.push_back(ntop); ns.push_back(ntop);

    vs.push_back(bur);    vs.push_back(bul);    vs.push_back(fur);
    ns.push_back(ntop); ns.push_back(ntop); ns.push_back(ntop);
}

// Qdebug overload to accept std::string class
inline QDebug operator<<(QDebug dbg, const std::string& str)
{
    dbg.nospace() << QString::fromStdString(str);
    return dbg.space();
}


// createModelFromFile(string fileName, vector<vec3> &vs, vector<vec3> &ns)
//
// Populates a model from a file.
// Current supported file formats:
//   -- ASCII STL
//   -- Binary STL
//
void Utils::createModelFromFile(string path, float scale, vector<vec3> &vs, vector<vec3> &ns) {
    enum Format {
        STL_ASCII,
        STL_BINARY,
        OTHER
    };

    Format fileFormat = OTHER;

    ifstream file(path, ios::in | ios::binary);
    string header;
    qDebug() << "Header...";
    qDebug() << "*** " << header << " ***";
    qDebug() << "Path...";
    qDebug() << "*** " << path << " ***";

    if (!file) { 
        qDebug() << "Attempting to correct file endings...";
        // this  is gonna check to see if the file exists with 
        // a different capitalization
        if (endsWith(path,".stl")) { 
            qDebug() << 2; 
            path.replace(path.end()-3,path.end(),"STL");
        } 
        
        if (endsWith(path,".STL")) {
            qDebug() << 1;  
            path.replace(path.end()-3,path.end(),"stl"); 
        } 
        
        qDebug() << "New path: " << path; 
        file.close();
        file.open(path);        
    }
    if (!file) {
            qDebug() << "File in path: " << path << " not found!";
            return;
    }

    if (endsWith(path, ".stl") || endsWith(path, ".STL")) {
        getline(file, header);

        qDebug() << "Header...";
        qDebug() << "*** " << header << " ***";



        if (startsWith(trim(header), "solid")) {
            fileFormat = STL_ASCII;
        } else {
            fileFormat = STL_BINARY;
        }
    }

    // Parse based on format
    switch (fileFormat) {
        case STL_ASCII:
            parseStlASCII(file, vs, ns);
            break;
        case STL_BINARY:
            parseStlBinary(file, vs, ns);
            break;
        default:
            break;
    }

    file.close();

    // Apply units
    if (scale != 1) {
        for (int i = 0; i < vs.size(); i++) {
            vs[i] = scale * vs[i];
        }
    }
}


// createModelFromFile(string fileName, vector<Vec> &vs, vector<Vec> &ns)
//
// Populates a model from a file.
// Current supported file formats:
//   -- ASCII STL
//   -- Binary STL
//
void Utils::createModelFromFile(string path, float scale, vector<Vec> &vs, vector<Vec> &ns) {
    enum Format {
        STL_ASCII,
        STL_BINARY,
        OTHER
    };

    Format fileFormat = OTHER;

    ifstream file(path, ios::in | ios::binary);
    string header;
    qDebug() << "Header...";
    qDebug() << "*** " << header << " ***";
    qDebug() << "Path...";
    qDebug() << "*** " << path << " ***";

    if (!file) { 
        qDebug() << "Attempting to correct file endings...";
        // this  is gonna check to see if the file exists with 
        // a different capitalization
        if (endsWith(path,".stl")) { 
            qDebug() << 3; 
            path.replace(path.end()-3,path.end(),"STL");
        //   path = path.substr(0,path.length()-3) + "STL";
        }
        else if (endsWith(path,".STL")) { 
            qDebug() << 4; 
            path.replace(path.end()-3,path.end(),"stl");
        //  path = path.substr(0,path.length()-3) + "stl";
        } 
        
        qDebug() << "New path: " << path;  
        file.close();
        file.open(path);       
    }
    if (!file) {
            qDebug() << "File in path: " << path << " not found!";
            return;
    }

    if (endsWith(path, ".stl") || endsWith(path, ".STL")) {
        getline(file, header);

        qDebug() << "Header...";
        qDebug() << "*** " << header << " ***";

        if (startsWith(trim(header), "solid")) {
            fileFormat = STL_ASCII;
        } else {
            fileFormat = STL_BINARY;
        }
    }

    // Parse based on format
    switch (fileFormat) {
        case STL_ASCII:
            parseStlASCII(file, vs, ns);
            break;
        case STL_BINARY:
            parseStlBinary(file, vs, ns);
            break;
        default:
            break;
    }

    file.close();

    // Apply units
    if (scale != 1) {
        for (int i = 0; i < vs.size(); i++) {
            vs[i] = scale * vs[i];
        }
    }
}

// parseStlASCII(ifstream &text, vector<vec3> &vs, vector<vec3> &ns)
//
// Parses vertex positions and normals from a Qt file in STL (ASCII) format
// See https://en.wikipedia.org/wiki/STL_(file_format)
//
void Utils::parseStlASCII(ifstream &text, vector<vec3> &vs, vector<vec3> &ns) {

    qDebug() << "PARSING STL ASCII";
    vec3 n, v;
    string sdead;

    for (string line; getline(text, line); ) {

        if (left(line, 8).compare("endsolid") == 0) {
            return;
        }

        if (startsWith(trim(line), "facet normal")) {

            istringstream iss(line);
            if (!(iss >> sdead >> sdead >> n.x >> n.y >> n.z)) {
                return;
            }
            ns.push_back(n); ns.push_back(n); ns.push_back(n);
        }

        if (startsWith(trim(line), "vertex")) {
            istringstream iss(line);
            if (!(iss >> sdead >> v.x >> v.y >> v.z)) {
                return;
            }
            vs.push_back(v);
        }

    }

}
//t

// parseStlASCII(ifstream &text, vector<Vec> &vs, vector<Vec> &ns)
//
// Parses vertex positions and normals from a Qt file in STL (ASCII) format
// See https://en.wikipedia.org/wiki/STL_(file_format)
//
void Utils::parseStlASCII(ifstream &text, vector<Vec> &vs, vector<Vec> &ns) {

    qDebug() << "PARSING STL ASCII";
    Vec n, v;
    string sdead;

    for (string line; getline(text, line); ) {

        if (left(line, 8).compare("endsolid") == 0) {
            return;
        }

        if (startsWith(trim(line), "facet normal")) {

            istringstream iss(line);
            if (!(iss >> sdead >> sdead >> n[0] >> n[1] >> n[2])) {
                return;
            }
            ns.push_back(n); ns.push_back(n); ns.push_back(n);
        }

        if (startsWith(trim(line), "vertex")) {
            istringstream iss(line);
            if (!(iss >> sdead >> v[0] >> v[1] >> v[2])) {
                return;
            }
            vs.push_back(v);
        }

    }

}

// parseStlBinary(ifstream &text, vector<vec3> &vs, vector<vec3> &ns)
//
// Parses vertex positions and normals from a Qt file in STL (binary) format
// See https://en.wikipedia.org/wiki/STL_(file_format)
//
void Utils::parseStlBinary(ifstream &text, vector<vec3> &vs, vector<vec3> &ns) {

    qDebug() << "PARSING STL BINARY";

    STLHEADER header;
    STLFACET facet;

    char h[80];
    char nt[4];
    text.read(h, 80);
    text.read(nt, 4);

    uint num_triangles = * (long *) nt;
    qDebug() << num_triangles;

    //text.read(reinterpret_cast<char *>(&header), sizeof(header));

    //uint nfacets = * reinterpret_cast<uint *>(header.nfacets);

    for (uint t = 0; t < * (int *) header.nfacets; t++) {
        text.read(reinterpret_cast<char *>(&facet), 50);

        ns.push_back(vec3(facet.nx, facet.ny, facet.nz));
        ns.push_back(vec3(facet.nx, facet.ny, facet.nz));
        ns.push_back(vec3(facet.nx, facet.ny, facet.nz));

        vs.push_back(vec3(facet.x1, facet.y1, facet.z1));
        vs.push_back(vec3(facet.x2, facet.y2, facet.z2));
        vs.push_back(vec3(facet.x3, facet.y3, facet.z3));
    }
}


// parseStlBinary(ifstream &text, vector<Vec> &vs, vector<Vec> &ns)
//
// Parses vertex positions and normals from a Qt file in STL (binary) format
// See https://en.wikipedia.org/wiki/STL_(file_format)
//
void Utils::parseStlBinary(ifstream &text, vector<Vec> &vs, vector<Vec> &ns) {

    qDebug() << "PARSING STL BINARY";

    STLHEADER header;
    STLFACET facet;

    char h[80];
    char nt[4];
    text.read(h, 80);
    text.read(nt, 4);

    uint num_triangles = * (long *) nt;
    qDebug() << num_triangles;

    //text.read(reinterpret_cast<char *>(&header), sizeof(header));

    //uint nfacets = * reinterpret_cast<uint *>(header.nfacets);

    for (uint t = 0; t < * (int *) header.nfacets; t++) {
        text.read(reinterpret_cast<char *>(&facet), 50);

        ns.push_back(Vec(facet.nx, facet.ny, facet.nz));
        ns.push_back(Vec(facet.nx, facet.ny, facet.nz));
        ns.push_back(Vec(facet.nx, facet.ny, facet.nz));

        vs.push_back(Vec(facet.x1, facet.y1, facet.z1));
        vs.push_back(Vec(facet.x2, facet.y2, facet.z2));
        vs.push_back(Vec(facet.x3, facet.y3, facet.z3));
    }
}

vec3 Utils::vecToVec3(Vec &vector) {
   return vec3(vector[0], vector[1], vector[2]);
}
