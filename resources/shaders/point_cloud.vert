#version 450 core

// Vertex attributes
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;

// Uniform buffer
layout(std140, binding = 0) uniform Matrices {
    mat4 model;
    mat4 view;
    mat4 projection;
    mat4 normalMatrix;
    vec4 lightPosition;
    vec4 lightColor;
    vec4 ambientColor;
    float pointSize;
    float nearPlane;
    float farPlane;
    float time;
} ubo;

// Output to fragment shader
out vec3 FragPos;
out vec3 Normal;
out vec4 Color;
out float PointSize;

void main() {
    // Transform position
    vec4 worldPos = ubo.model * vec4(aPosition, 1.0);
    FragPos = worldPos.xyz;
    
    // Transform normal
    Normal = normalize(mat3(ubo.normalMatrix) * aNormal);
    
    // Pass through color
    Color = aColor;
    
    // Calculate point size with distance attenuation
    vec4 viewPos = ubo.view * worldPos;
    float distance = length(viewPos.xyz);
    
    // Adaptive point size based on distance
    PointSize = ubo.pointSize * sqrt(1000.0 / distance);
    gl_PointSize = max(1.0, PointSize);
    
    // Final position
    gl_Position = ubo.projection * viewPos;
    
    // Enable point sprites
    gl_PointCoord = vec2(0.5);
} 