#version 330 core

uniform sampler2D aTexture;
in vec2 vUV;
out vec4 fColor;

void main() {
    fColor = texture(aTexture, vUV);
}
