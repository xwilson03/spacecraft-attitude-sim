#version 330 core

in float vFresnelTerm;
out vec4 fColor;

void main() {
    float clampedFresnelTerm = clamp(vFresnelTerm, 0.0, 1.0);
    float glow = pow(1 - clampedFresnelTerm, 2.0) * smoothstep(0.0, 0.5, clampedFresnelTerm);

    vec3 baseColor = vec3(0.3, 0.55, 1.0);
    fColor = vec4(baseColor * glow, 1.0);
}
