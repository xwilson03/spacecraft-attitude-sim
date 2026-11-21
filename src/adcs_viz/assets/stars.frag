#version 330 core

in vec3 vColor;
out vec4 fColor;

void main() {
    float distToPointCenter = length(gl_PointCoord - vec2(0.5)) * 2.0;
    float glow = clamp(1.0 - distToPointCenter, 0.0, 1.0);
    fColor = vec4(vColor * glow * glow, 1.0);
    fColor = vec4(vColor * pow(glow, 4), 1.0);
}
