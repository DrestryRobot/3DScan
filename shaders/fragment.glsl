#version 430 core
in vec3 vColor;
out vec4 fragOutput;

void main() {
    fragOutput = vec4(vColor, 1.0);
}
