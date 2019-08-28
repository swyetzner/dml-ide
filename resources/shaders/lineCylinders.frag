#version 330

// 3D bar shader

uniform mat4 mvMatrix;
uniform mat3 normMatrix;
uniform vec3 lightPos;

out vec4 fragColor;
in vec4 fColor;
in vec4 vPosition;
in vec3 vNormal;

void main() {
    vec3 L = normalize(lightPos - vPosition.xyz);
    float NL = max(dot(normalize(normMatrix * vNormal), L), 0.0);
    vec3 color = fColor.rgb;
    vec3 clColor = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);
    fragColor = vec4(vec3(clColor), 1.0);
}
