#version 330 core

layout (location = 0) in vec3 aPos;

uniform mat4 aMVP;
uniform vec3 aCameraPosAtmo;
out float vFresnelTerm;

void main() {
    gl_Position = aMVP * vec4(aPos, 1.0);

    vec3 posToCamera = aCameraPosAtmo - aPos;
    vFresnelTerm = dot(normalize(aPos), normalize(posToCamera));
}
