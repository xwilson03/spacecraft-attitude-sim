#version 330 core

uniform sampler2D aTexture;
in vec2 vUV;
out vec4 fColor;

void main() {
    // phi is flipped; we flipped cull dir for better tex resolution
    fColor = texture(aTexture, vec2(1 - vUV.x, vUV.y));
}
