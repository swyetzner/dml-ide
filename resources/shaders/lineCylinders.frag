#version 330

// 3D bar shader

uniform mat4 mvMatrix;
uniform mat3 normMatrix;
uniform vec3 lightPos;
uniform vec3 viewPos;

uniform mat4 lightSpaceMatrix;
uniform sampler2D shadowMap;

out vec4 fragColor;
in vec4 fColor;
in vec4 vPosition;
in vec3 vNormal;

float shadowCalc(vec4 posLight, float normalLight) {
   vec3 projCoords = posLight.xyz / posLight.w;
   projCoords = projCoords * 0.5 + 0.5;
   float closestDepth = texture(shadowMap, projCoords.xy).r;
   float currentDepth = projCoords.z;
   float bias = max(0.05 * (1.0 - normalLight), 0.005);
   float shadow = 0.0;
   vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
   for (int x = -1; x <= 1; ++x) {
       for (int y = -1; y <= 1; ++y) {
           float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
           shadow += currentDepth - bias > pcfDepth? 1.0 : 0.0;
       }
   }
   return shadow / 9.0;
}

void main() {
    vec3 N = normalize(normMatrix * vNormal);
    vec3 V = normalize(viewPos - vPosition.xyz);
    vec3 L = normalize(lightPos - vPosition.xyz);
    vec3 R = reflect(-L, N);
    float S = 0.5 * pow(max(dot(V, R), 0.0), 64); // Specular

    float NL = max(dot(N, L), 0.0);
    // shadow
    float shadow = shadowCalc(lightSpaceMatrix * vPosition, NL);

    vec3 color = fColor.rgb;
    vec3 clColor = clamp(color * 0.4 + color * 0.6 * NL, 0.0, 1.0);
    fragColor = vec4(clColor, 1.0);
}
