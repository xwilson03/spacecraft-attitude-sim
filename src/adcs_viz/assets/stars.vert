#version 330 core

layout (location = 0) in vec3  aPos;
layout (location = 1) in float aSize;
layout (location = 2) in float aTint;

uniform mat4 aMVP;
out vec3 vColor;

void main() {
    gl_Position = aMVP * vec4(aPos, 1.0);
    gl_PointSize = aSize;

    vec3 blue_white = vec3(0.75, 0.85, 1.0);
    vec3 orange = vec3(1.0, 0.6, 0.3);
    vColor = mix(blue_white, orange, aTint);
}
