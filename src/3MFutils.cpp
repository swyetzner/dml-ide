#include "3MFutils.h"
sLib3MFPosition fnCreateVertex(float x, float y, float z)
{
    sLib3MFPosition result;
    result.m_Coordinates[0] = x;
    result.m_Coordinates[1] = y;
    result.m_Coordinates[2] = z;
    return result;
}

sLib3MFBeam fnCreateBeam(int v0, int v1, double r0, double r1, eLib3MFBeamLatticeCapMode c0, eLib3MFBeamLatticeCapMode c1)
{
    sLib3MFBeam result;
    result.m_Indices[0] = v0;
    result.m_Indices[1] = v1;
    result.m_Radii[0] = r0;
    result.m_Radii[1] = r1;
    result.m_CapModes[0] = c0;
    result.m_CapModes[1] = c1;
    return result;
}