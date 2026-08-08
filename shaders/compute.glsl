#version 430
layout(local_size_x = 256) in;

struct Point {
    vec4 position;
    vec4 color;
};

// 固定大小数组，和 C++ 侧 MAX_POINTS 保持一致
layout(std430, binding = 0) buffer PointCloud {
    Point points[MAX_POINTS];
};

uniform int currentIndex;
uniform vec3 newPoint;
uniform vec3 newColor;

void main() {
    uint id = gl_GlobalInvocationID.x;
    if (id == uint(currentIndex)) {
        points[id].position = vec4(newPoint, 1.0);
        points[id].color = vec4(newColor, 1.0);
    }
}
