#include "lib3mf_implicit.hpp"
using namespace Lib3MF;
// Utility functions to create vertices and beams
sLib3MFPosition fnCreateVertex(float x, float y, float z);

// v0, v1 are the first and second vertex, r0 r1 are the radius at each vertex, cap mode is how to end the beam (no cap, sphere, or hemisphere)
sLib3MFBeam fnCreateBeam(int v0, int v1, double r0, double r1, eLib3MFBeamLatticeCapMode c0, eLib3MFBeamLatticeCapMode c1);
