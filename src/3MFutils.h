#include "lib3mf_implicit.hpp"
#include <iostream>
#include <string>
#include <Titan/sim.h>
#include <glm/glm.hpp>
typedef glm::vec3 vec3;
using namespace Lib3MF;
// Utility functions to create vertices and beams
sLib3MFPosition fnCreateVertex(float x, float y, float z);

// v0, v1 are the first and second vertex, r0 r1 are the radius at each vertex, cap mode is how to end the beam (no cap, sphere, or hemisphere)
sLib3MFBeam fnCreateBeam(int v0, int v1, double r0, double r1, eLib3MFBeamLatticeCapMode c0, eLib3MFBeamLatticeCapMode c1);

void loadGeometry3MF(std::string path, std::vector<vec3> &vs, std::vector<vec3> &ns);
void loadGeometry3MF(string path, vector<Vec> &vs, vector<Vec> &ns);
float getScaleFromUnit(eModelUnit units);

// utilities from 3MF examples
void ShowObjectProperties(PObject object);
void printVersion(PWrapper wrapper);
void ShowMeshObjectInformation(PMeshObject meshObject);
void ShowSliceStack(PSliceStack sliceStack, std::string indent);
void ShowMetaDataInformation(PMetaDataGroup metaDataGroup);