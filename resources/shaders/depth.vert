#version 330 core
layout (location = 0) in vec3 vertexPos;

uniform mat4 lightSpaceMatrix;

void main() {
    gl_Position = lightSpaceMatrix * vec4(vertexPos, 1.0);
}
