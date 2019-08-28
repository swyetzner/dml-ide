#version 330

uniform mat4 projMatrix;
uniform mat4 mvMatrix;

layout (location = 0) in vec3 vertexPos;
layout (location = 1) in vec3 vertexNor;
layout (location = 2) in vec4 colorPos;

out VertexOut {
    vec3 position;
    vec3 normal;
    vec4 color;
} vertexOut;

void main() {
    gl_Position = projMatrix * mvMatrix * vec4(vertexPos, 1.0);
    vertexOut.position = vertexPos;
    vertexOut.normal = vertexNor;
    vertexOut.color = colorPos;
}