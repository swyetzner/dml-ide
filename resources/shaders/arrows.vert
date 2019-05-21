#version 330

layout (location = 5) in vec3 vertexPos;
layout (location = 6) in vec3 forceVec;

out VertexOut {
    vec3 force;
} vertexOut;

void main() {
    gl_Position = vec4(vertexPos, 1.0);
    vertexOut.force = forceVec;
}
