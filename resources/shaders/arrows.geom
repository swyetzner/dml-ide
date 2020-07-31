#version 330 core

// See http://paulbourke.net/geometry/circlesphere/

layout (points) in;
layout (triangle_strip, max_vertices = 64) out;
uniform mat4 projMatrix;
uniform mat4 mvMatrix;
uniform float scale;

in VertexOut {
    vec3 force;
} geomIn[];

// Takes radius of each end and a resolution
void createCylinder(float r1, float r2, float offset, float length, int res) {

    float pi = atan(1.0)*4;
    float twoPi = 2 * pi;
    vec3 forceVec = normalize(geomIn[0].force);

    // Get line vector
    vec3 pt = gl_in[0].gl_Position.xyz;
    vec3 p1 = pt - offset * forceVec;
    vec3 p2 = p1 - length * forceVec;
    vec3 p = p2 - p1;

    // Find two perpendicular vectors (a, b)
    vec3 ax = vec3(0.0, 0.0, 1.0);
    if (forceVec[0] == ax[0] && forceVec[1] == ax[1]) {
        ax = vec3(0.0, 1.0, 0.0);
    }
    vec3 a = cross(p, ax);
    vec3 b = cross(p, a);
    a = normalize(a);
    b = normalize(b);

    // Iterate around line
    for (int i = 0; i < res + 1; i++) {
        float theta = i * twoPi / res;

        //gs_out.normal = cos(theta1)*a + sin(thet1)*b;

        vec3 q1 = p1 + r1*cos(theta)*a + r1*sin(theta)*b;
        gl_Position = projMatrix * mvMatrix * vec4(q1, 1.0);
        EmitVertex();

        vec3 q2 = p2 + r2*cos(theta)*a + r2*sin(theta)*b;
        gl_Position = projMatrix * mvMatrix * vec4(q2, 1.0);
        EmitVertex();
    }

    EndPrimitive();
}


void main() {
    createCylinder(0, 0.005 * scale, 0, 0.02 * scale, 8);
    createCylinder(0.002 * scale, 0.002 * scale, 0.02 * scale, 0.04 * scale, 8);
}