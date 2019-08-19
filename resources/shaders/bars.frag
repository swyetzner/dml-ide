#version 330

// 3D bar shader

uniform mat3 normMatrix;
uniform vec3 lightPos;

out vec4 fragColor;
in VertexOut {
    vec3 position;
    vec3 normal;
    vec4 color;
} fragIn;

void main() {
    vec3 L = normalize(lightPos - fragIn.position);
    float NL = max(dot(normalize(normMatrix * fragIn.normal), L), 0.0);
    vec3 color = fragIn.color.rgb;
    vec3 clColor = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);
    fragColor = vec4(clColor, 1.0);
}
