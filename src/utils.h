#ifndef UTILS_H
#define UTILS_H

#include <bitset>
#include <fstream>
#include <sstream>

#include <glm/glm.hpp>
#include <Titan/sim.h>

typedef glm::vec3 vec3;

class Utils
{

public:
    Utils();

    // STRING UTILS
    static inline bool startsWith(const string &s, const string &p);
    static inline bool endsWith(const string &s, const string &e);
    static inline string left(const string &s, int n);
    static inline string trim(string &s);

    // GEOMETRY UTILS
    static float randFloat(float min, float max);
    static float randUnit();
    static vec3 randDirection();
    static Vec randDirectionVec();
    static vec3 randPoint(vec3 lowBound, vec3 highBound);
    static Vec randPointVec(Vec lowBound, Vec highBound);
    static double clamp(double imin, double imax, double omin, double omax, double input);
    static double getAngle(Vec v1, Vec v2);
    static bool isAcute(Vec v1, Vec v2);
    static Vec bisect(Vec p1, Vec p2);
    static vec3 findNormal(vec3 v1, vec3 v2, vec3 v3);
    static Vec findNormal(Vec v1, Vec v2, Vec v3);
    static double distPointLine(Vec p, Vec p1, Vec p2);
    static double distPointPlane(Vec p, Vec n, double o, Vec &v);
    static bool areCloseToColinear(Vec p1, Vec p2, Vec p3, double eps);
    static bool areCloseToCoplanar(Vec p1, Vec p2, Vec p3, Vec p4, double eps);
    static bool getPlaneEquation(Vec normal, Vec pt, double &d);
    static bool intersectPlane(Vec *triangle, Vec o, Vec dir, Vec &p, double &pu, double &pv);
    static bool intersectPlane(vec3 *triangle, vec3 o, vec3 dir, vec3 &p, float &pu, float &pv);
    static bool insideTriangle(vec3 a, vec3 b, vec3 c, vec3 &point);
    static bool insideTriangle(Vec &a, Vec &b, Vec &c, Vec &point);

    // MODEL UTILS
    struct STLHEADER {
        char    description[80];
        int     nfacets;
    };

    struct STLFACET {
        float   nx, ny, nz;
        float   x1, y1, z1;
        float   x2, y2, z2;
        float   x3, y3, z3;
        short   pad;
    };

    static void createCube(vec3 center, float edgeLength, vector<vec3> &vs, vector<vec3> &ns);
    static void createModelFromFile(string path, float scale, vector<vec3> &vs, vector<vec3> &ns);
    static void createModelFromFile(string path, float scale, vector<Vec> &vs, vector<Vec> &ns);
    static void parseStlASCII(ifstream &text, vector<vec3> &vs, vector<vec3> &ns);
    static void parseStlASCII(ifstream &text, vector<Vec> &vs, vector<Vec> &ns);
    static void parseStlBinary(ifstream &text, vector<vec3> &vs, vector<vec3> &ns);
    static void parseStlBinary(ifstream &text, vector<Vec> &vs, vector<Vec> &ns);
    static vec3 vecToVec3(Vec &vector);
};

#endif // UTILS_H
