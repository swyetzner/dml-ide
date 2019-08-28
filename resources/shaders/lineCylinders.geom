#version 330 core

// See http://paulbourke.net/geometry/circlesphere/

layout (lines) in;
layout (triangle_strip, max_vertices = 32) out;
uniform mat4 projMatrix;
uniform mat4 mvMatrix;

in VertexOut {
    float diam;
    vec4 color;
} geomIn[];
out vec4 fColor;
out vec4 vPosition;
out vec3 vNormal;

// Takes radius of each end and a resolution
void createCylinders(float r1, float r2, int res) {

    float pi = atan(1.0)*4;
    float twoPi = 2 * pi;

    // Get line vector
    vec3 p1 = gl_in[0].gl_Position.xyz;
    vec3 p2 = gl_in[1].gl_Position.xyz;
    vec3 p = normalize(p1 - p2);

    // Find two perpendicular vectors (a, b)
    vec3 ax = vec3(0.0, 1.0, 0.0);
    if (p == ax) {
        ax = vec3(0.0, 0.0, 1.0);
    }
    vec3 a = cross(p, ax);
    vec3 b = cross(p, a);
    a = normalize(a);
    b = normalize(b);

    // Iterate around line
    for (int i = 0; i < res + 2; i++) {
        float theta = i * twoPi / res;

        //gs_out.normal = cos(theta1)*a + sin(thet1)*b;

        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q1;
        vNormal = normalize(q1.xyz - p1);
        gl_Position = vPosition;
        EmitVertex();

        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q2;
        vNormal = normalize(q2.xyz - p2);
        gl_Position = vPosition;
        EmitVertex();
    }
    EndPrimitive();

    // Close each end
    vec3 p3 = p1 + r1*p;  // Extrapolate center of end
    for (int i = 0; i < res + 1; i++) {
        float theta = i * twoPi / res;

        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q1;
        vNormal = normalize(q1.xyz - p1);
        gl_Position = vPosition;
        EmitVertex();

        vPosition = projMatrix * mvMatrix * vec4(p3, 1.0);
        vNormal = p;
        gl_Position = vPosition;
        EmitVertex();
    }
    EndPrimitive();

    vec3 p4 = p2 + r2*p;  // Extrapolate center of end
    for (int i = 0; i < res + 1; i++) {
        float theta = i * twoPi / res;

        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q2;
        vNormal = normalize(q2.xyz - p2);
        gl_Position = vPosition;
        EmitVertex();

        vPosition = projMatrix * mvMatrix * vec4(p4, 1.0);
        vNormal = p;
        gl_Position = vPosition;
        EmitVertex();
    }
    EndPrimitive();


/**
    // Top and bottom faces
    for (int i = 0; i < res + 2; i++) {
        float theta = i * twoPi / res;
        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q1;
        vNormal = normalize(q1.xyz - q2.xyz);
        gl_Position = vPosition;
        EmitVertex();
    }
    EndPrimitive();

    for (int i = 0; i < res + 2; i++) {
        float theta = i * twoPi / res;
        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        vPosition = projMatrix * mvMatrix * q2;
        vNormal = normalize(q2.xyz - q1.xyz);
        gl_Position = vPosition;
        EmitVertex();
    }
    //gl_Position = (gl_in[0].gl_Position + gl_in[1].gl_Position) * 0.5;
    //EmitVertex();
    EndPrimitive();**/
}


void main() {
    fColor = geomIn[0].color;
    createCylinders(geomIn[0].diam, geomIn[0].diam, 8);
}