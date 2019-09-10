#version 330 core
layout (location = 0) in vec3 vertexPos;
layout (location = 1) in float diameter;

out VertexOut {
    float diam;
} vertexOut;


void main() {
    gl_Position = vec4(vertexPos, 1.0);
    vertexOut.diam = diameter;
}
