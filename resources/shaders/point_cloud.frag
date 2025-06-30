#version 450 core

// Input from vertex shader
in vec3 FragPos;
in vec3 Normal;
in vec4 Color;
in float PointSize;

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

// Output
out vec4 FragColor;

void main() {
    // Create circular points
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = length(coord);
    
    // Discard fragments outside circle
    if (dist > 0.5) {
        discard;
    }
    
    // Smooth edge
    float alpha = 1.0 - smoothstep(0.4, 0.5, dist);
    
    // Calculate lighting (Phong model)
    vec3 lightDir = normalize(ubo.lightPosition.xyz - FragPos);
    vec3 viewDir = normalize(-FragPos); // Camera at origin in view space
    
    // Ambient
    vec3 ambient = ubo.ambientColor.rgb * Color.rgb;
    
    // Diffuse
    float diff = max(dot(Normal, lightDir), 0.0);
    vec3 diffuse = diff * ubo.lightColor.rgb * Color.rgb;
    
    // Specular
    vec3 reflectDir = reflect(-lightDir, Normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = spec * ubo.lightColor.rgb * 0.5;
    
    // Combine lighting
    vec3 result = ambient + diffuse + specular;
    
    // Apply fog for depth perception
    float fogStart = 50.0;
    float fogEnd = 200.0;
    float fogFactor = clamp((length(FragPos) - fogStart) / (fogEnd - fogStart), 0.0, 1.0);
    vec3 fogColor = vec3(0.1, 0.1, 0.1);
    result = mix(result, fogColor, fogFactor * 0.5);
    
    // Output with alpha
    FragColor = vec4(result, Color.a * alpha);
    
    // Depth correction for point sprites
    vec3 normal = vec3(coord * 2.0, sqrt(1.0 - dot(coord, coord)));
    vec4 projPos = ubo.projection * ubo.view * vec4(FragPos + normal * PointSize * 0.01, 1.0);
    gl_FragDepth = (projPos.z / projPos.w + 1.0) * 0.5;
} 