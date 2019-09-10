#version 330

// See http://paulbourke.net/geometry/circlesphere/

layout (lines) in;
layout (triangle_strip, max_vertices = 32) out;

in VertexOut {
    float diam;
} geomIn[];

uniform mat4 lightSpaceMatrix;


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

        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        gl_Position = lightSpaceMatrix * q1;
        EmitVertex();

        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        gl_Position = lightSpaceMatrix * q2;
        EmitVertex();
    }
    EndPrimitive();

    // Close each end
    vec3 p3 = p1 + r1*p;  // Extrapolate center of end
    for (int i = 0; i < res + 1; i++) {
        float theta = i * twoPi / res;

        vec4 q1 = vec4(p1 + r1*cos(theta)*a + r1*sin(theta)*b, 1.0);
        gl_Position = lightSpaceMatrix * q1;
        EmitVertex();

        gl_Position = lightSpaceMatrix * vec4(p3, 1.0);
        EmitVertex();
    }
    EndPrimitive();

    vec3 p4 = p2 + r2*p;  // Extrapolate center of end
    for (int i = 0; i < res + 1; i++) {
        float theta = i * twoPi / res;

        vec4 q2 = vec4(p2 + r2*cos(theta)*a + r2*sin(theta)*b, 1.0);
        gl_Position = lightSpaceMatrix * q2;
        EmitVertex();

        gl_Position = lightSpaceMatrix * vec4(p4, 1.0);
        EmitVertex();
    }
    EndPrimitive();
}


void main() {
    createCylinders(geomIn[0].diam, geomIn[0].diam, 8);
}