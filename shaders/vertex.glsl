#version 430 core
layout(std430, binding = 0) buffer PointCloud {
    vec4 position[MAX_POINTS];
    vec4 color[MAX_POINTS];
};

out vec3 vColor;

void main() {
    gl_Position = position[gl_VertexID];
    vColor = color[gl_VertexID].rgb;
    gl_PointSize = 3.0; // 控制点大小
}
