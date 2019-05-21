#version 330

layout (location = 0) in vec3 vertexPos;
layout (location = 2) in vec4 colorPos;
layout (location = 3) in float diameter;

out VertexOut {
    float diam;
    vec4 color;
} vertexOut;

void main() {
    gl_Position = vec4(vertexPos, 1.0);
    vertexOut.color = colorPos;
    vertexOut.diam = diameter;
}
