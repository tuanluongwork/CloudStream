#include "visualization/PointCloudRenderer.h"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

namespace CloudStream {

PointCloudRenderer::PointCloudRenderer() {
}

PointCloudRenderer::~PointCloudRenderer() {
    cleanup();
}

bool PointCloudRenderer::initialize() {
    initializeOpenGLFunctions();
    
    Logger::instance().info("Initializing PointCloudRenderer");
    
    // Create shader program
    if (!createDefaultShaders()) {
        Logger::instance().error("Failed to create default shaders");
        return false;
    }
    
    // Create VAO and VBO
    vao_ = std::make_unique<QOpenGLVertexArrayObject>();
    vao_->create();
    
    vbo_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    vbo_->create();
    
    // Create UBO for uniforms
    glGenBuffers(1, &ubo_);
    glBindBuffer(GL_UNIFORM_BUFFER, ubo_);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(UniformBlock), nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo_);
    
    // Create timer query for performance monitoring
    glGenQueries(1, &query_id_);
    
    Logger::instance().info("PointCloudRenderer initialized successfully");
    return true;
}

void PointCloudRenderer::cleanup() {
    if (ubo_) {
        glDeleteBuffers(1, &ubo_);
        ubo_ = 0;
    }
    
    if (query_id_) {
        glDeleteQueries(1, &query_id_);
        query_id_ = 0;
    }
    
    vao_.reset();
    vbo_.reset();
    shader_program_.reset();
}

void PointCloudRenderer::setPointCloud(PointCloud::Ptr cloud) {
    point_cloud_ = cloud;
    needs_update_ = true;
    
    if (cloud) {
        Logger::instance().info("Point cloud set with {} points", cloud->size());
        stats_.points_rendered = cloud->size();
    }
}

void PointCloudRenderer::updatePointCloud(PointCloud::Ptr cloud) {
    setPointCloud(cloud);
}

void PointCloudRenderer::clearPointCloud() {
    point_cloud_.reset();
    vertex_buffer_.clear();
    needs_update_ = false;
    stats_.points_rendered = 0;
}

void PointCloudRenderer::render(const QMatrix4x4& view, const QMatrix4x4& projection) {
    if (!shader_program_ || !point_cloud_ || point_cloud_->empty()) {
        return;
    }
    
    ScopedTimer timer("Render");
    
    // Start GPU timer
    glBeginQuery(GL_TIME_ELAPSED, query_id_);
    
    // Update vertex buffer if needed
    if (needs_update_) {
        updateVertexBuffer();
        uploadVertexData();
        needs_update_ = false;
    }
    
    // Update uniforms
    updateUniforms(view, projection);
    
    // Bind shader and VAO
    shader_program_->bind();
    vao_->bind();
    
    // Set render state
    if (settings_.use_depth_test) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    
    glPointSize(settings_.point_size);
    
    // Draw points
    size_t points_to_render = std::min(vertex_buffer_.size(), settings_.max_points_per_frame);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points_to_render));
    
    stats_.points_rendered = points_to_render;
    stats_.draw_calls = 1;
    
    // Unbind
    vao_->release();
    shader_program_->release();
    
    // End GPU timer
    glEndQuery(GL_TIME_ELAPSED);
    
    // Get GPU time (non-blocking)
    GLuint64 gpu_time_ns = 0;
    GLint available = 0;
    glGetQueryObjectiv(query_id_, GL_QUERY_RESULT_AVAILABLE, &available);
    if (available) {
        glGetQueryObjectui64v(query_id_, GL_QUERY_RESULT, &gpu_time_ns);
        stats_.gpu_time_ms = gpu_time_ns / 1000000.0f;
    }
    
    stats_.cpu_time_ms = timer.elapsedMilliseconds();
}

void PointCloudRenderer::renderWithCustomShader(QOpenGLShaderProgram* shader,
                                               const QMatrix4x4& view,
                                               const QMatrix4x4& projection) {
    if (!shader || !point_cloud_ || point_cloud_->empty()) {
        return;
    }
    
    // Update vertex buffer if needed
    if (needs_update_) {
        updateVertexBuffer();
        uploadVertexData();
        needs_update_ = false;
    }
    
    // Use custom shader
    shader->bind();
    
    // Set uniforms manually for custom shader
    shader->setUniformValue("model", QMatrix4x4());
    shader->setUniformValue("view", view);
    shader->setUniformValue("projection", projection);
    
    // Bind VAO and draw
    vao_->bind();
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertex_buffer_.size()));
    vao_->release();
    
    shader->release();
}

void PointCloudRenderer::setViewport(int width, int height) {
    viewport_width_ = width;
    viewport_height_ = height;
}

QMatrix4x4 PointCloudRenderer::getProjectionMatrix() const {
    QMatrix4x4 projection;
    float aspect = static_cast<float>(viewport_width_) / viewport_height_;
    projection.perspective(settings_.fov, aspect, settings_.near_plane, settings_.far_plane);
    return projection;
}

bool PointCloudRenderer::loadShaders(const QString& vertex_path, const QString& fragment_path) {
    auto shader = std::make_unique<QOpenGLShaderProgram>();
    
    // Load vertex shader
    QFile vertex_file(vertex_path);
    if (!vertex_file.open(QFile::ReadOnly | QFile::Text)) {
        Logger::instance().error("Failed to open vertex shader: {}", vertex_path.toStdString());
        return false;
    }
    
    QTextStream vertex_stream(&vertex_file);
    QString vertex_source = vertex_stream.readAll();
    
    if (!shader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_source)) {
        Logger::instance().error("Failed to compile vertex shader: {}", 
                               shader->log().toStdString());
        return false;
    }
    
    // Load fragment shader
    QFile fragment_file(fragment_path);
    if (!fragment_file.open(QFile::ReadOnly | QFile::Text)) {
        Logger::instance().error("Failed to open fragment shader: {}", fragment_path.toStdString());
        return false;
    }
    
    QTextStream fragment_stream(&fragment_file);
    QString fragment_source = fragment_stream.readAll();
    
    if (!shader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_source)) {
        Logger::instance().error("Failed to compile fragment shader: {}", 
                               shader->log().toStdString());
        return false;
    }
    
    // Link program
    if (!shader->link()) {
        Logger::instance().error("Failed to link shader program: {}", 
                               shader->log().toStdString());
        return false;
    }
    
    shader_program_ = std::move(shader);
    return true;
}

void PointCloudRenderer::updateVertexBuffer() {
    if (!point_cloud_) return;
    
    vertex_buffer_.clear();
    vertex_buffer_.reserve(point_cloud_->size());
    
    // Convert point cloud to vertex buffer
    for (const auto& point : *point_cloud_) {
        VertexData vertex;
        
        vertex.position[0] = point.position.x;
        vertex.position[1] = point.position.y;
        vertex.position[2] = point.position.z;
        
        vertex.normal[0] = point.normal.x;
        vertex.normal[1] = point.normal.y;
        vertex.normal[2] = point.normal.z;
        
        vertex.color[0] = point.color.r;
        vertex.color[1] = point.color.g;
        vertex.color[2] = point.color.b;
        vertex.color[3] = point.color.a;
        
        vertex.padding[0] = point.intensity;
        vertex.padding[1] = 0.0f;
        
        vertex_buffer_.push_back(vertex);
    }
    
    stats_.vbo_memory_bytes = vertex_buffer_.size() * sizeof(VertexData);
}

void PointCloudRenderer::uploadVertexData() {
    if (vertex_buffer_.empty()) return;
    
    vao_->bind();
    vbo_->bind();
    
    // Upload data
    vbo_->allocate(vertex_buffer_.data(), 
                   static_cast<int>(vertex_buffer_.size() * sizeof(VertexData)));
    
    // Setup vertex attributes
    setupVertexAttributes();
    
    vbo_->release();
    vao_->release();
}

void PointCloudRenderer::setupVertexAttributes() {
    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         reinterpret_cast<void*>(offsetof(VertexData, position)));
    
    // Normal attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         reinterpret_cast<void*>(offsetof(VertexData, normal)));
    
    // Color attribute
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(VertexData),
                         reinterpret_cast<void*>(offsetof(VertexData, color)));
}

void PointCloudRenderer::updateUniforms(const QMatrix4x4& view, const QMatrix4x4& projection) {
    // Update uniform block
    uniforms_.model = QMatrix4x4();
    uniforms_.view = view;
    uniforms_.projection = projection;
    uniforms_.normal_matrix = (view * uniforms_.model).normalMatrix();
    uniforms_.point_size = settings_.point_size;
    uniforms_.near_plane = settings_.near_plane;
    uniforms_.far_plane = settings_.far_plane;
    uniforms_.time = static_cast<float>(QTime::currentTime().msecsSinceStartOfDay()) / 1000.0f;
    
    // Upload to GPU
    glBindBuffer(GL_UNIFORM_BUFFER, ubo_);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(UniformBlock), &uniforms_);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void PointCloudRenderer::performFrustumCulling(const QMatrix4x4& mvp) {
    if (!settings_.use_frustum_culling || !point_cloud_) return;
    
    // Extract frustum planes
    frustum_.extract(mvp);
    
    // Count culled points
    stats_.points_culled = 0;
    
    // In a real implementation, we would cull points here
    // For now, this is a placeholder
}

bool PointCloudRenderer::createDefaultShaders() {
    // Try to load from files first
    if (loadShaders("resources/shaders/point_cloud.vert", 
                    "resources/shaders/point_cloud.frag")) {
        return true;
    }
    
    // Fall back to embedded shaders
    const char* vertex_source = R"(
#version 450 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;

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

out vec3 FragPos;
out vec3 Normal;
out vec4 Color;

void main() {
    vec4 worldPos = ubo.model * vec4(aPosition, 1.0);
    FragPos = worldPos.xyz;
    Normal = normalize(mat3(ubo.normalMatrix) * aNormal);
    Color = aColor;
    
    vec4 viewPos = ubo.view * worldPos;
    float distance = length(viewPos.xyz);
    gl_PointSize = max(1.0, ubo.pointSize * sqrt(1000.0 / distance));
    
    gl_Position = ubo.projection * viewPos;
}
)";
    
    const char* fragment_source = R"(
#version 450 core

in vec3 FragPos;
in vec3 Normal;
in vec4 Color;

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

out vec4 FragColor;

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = length(coord);
    
    if (dist > 0.5) {
        discard;
    }
    
    float alpha = 1.0 - smoothstep(0.4, 0.5, dist);
    
    vec3 lightDir = normalize(ubo.lightPosition.xyz - FragPos);
    float diff = max(dot(Normal, lightDir), 0.0);
    vec3 diffuse = diff * ubo.lightColor.rgb * Color.rgb;
    vec3 ambient = ubo.ambientColor.rgb * Color.rgb;
    
    vec3 result = ambient + diffuse;
    FragColor = vec4(result, Color.a * alpha);
}
)";
    
    auto shader = std::make_unique<QOpenGLShaderProgram>();
    
    if (!shader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_source)) {
        Logger::instance().error("Failed to compile embedded vertex shader");
        return false;
    }
    
    if (!shader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_source)) {
        Logger::instance().error("Failed to compile embedded fragment shader");
        return false;
    }
    
    if (!shader->link()) {
        Logger::instance().error("Failed to link embedded shader program");
        return false;
    }
    
    shader_program_ = std::move(shader);
    return true;
}

void PointCloudRenderer::Frustum::extract(const QMatrix4x4& mvp) {
    // Extract frustum planes from MVP matrix
    // Left plane
    planes[0] = glm::vec4(
        mvp(0, 3) + mvp(0, 0),
        mvp(1, 3) + mvp(1, 0),
        mvp(2, 3) + mvp(2, 0),
        mvp(3, 3) + mvp(3, 0)
    );
    
    // Right plane
    planes[1] = glm::vec4(
        mvp(0, 3) - mvp(0, 0),
        mvp(1, 3) - mvp(1, 0),
        mvp(2, 3) - mvp(2, 0),
        mvp(3, 3) - mvp(3, 0)
    );
    
    // Bottom plane
    planes[2] = glm::vec4(
        mvp(0, 3) + mvp(0, 1),
        mvp(1, 3) + mvp(1, 1),
        mvp(2, 3) + mvp(2, 1),
        mvp(3, 3) + mvp(3, 1)
    );
    
    // Top plane
    planes[3] = glm::vec4(
        mvp(0, 3) - mvp(0, 1),
        mvp(1, 3) - mvp(1, 1),
        mvp(2, 3) - mvp(2, 1),
        mvp(3, 3) - mvp(3, 1)
    );
    
    // Near plane
    planes[4] = glm::vec4(
        mvp(0, 3) + mvp(0, 2),
        mvp(1, 3) + mvp(1, 2),
        mvp(2, 3) + mvp(2, 2),
        mvp(3, 3) + mvp(3, 2)
    );
    
    // Far plane
    planes[5] = glm::vec4(
        mvp(0, 3) - mvp(0, 2),
        mvp(1, 3) - mvp(1, 2),
        mvp(2, 3) - mvp(2, 2),
        mvp(3, 3) - mvp(3, 2)
    );
    
    // Normalize planes
    for (auto& plane : planes) {
        float length = glm::length(glm::vec3(plane));
        plane /= length;
    }
}

bool PointCloudRenderer::Frustum::containsPoint(const glm::vec3& point) const {
    for (const auto& plane : planes) {
        float distance = plane.x * point.x + plane.y * point.y + 
                        plane.z * point.z + plane.w;
        if (distance < 0) {
            return false;
        }
    }
    return true;
}

bool PointCloudRenderer::Frustum::containsSphere(const glm::vec3& center, float radius) const {
    for (const auto& plane : planes) {
        float distance = plane.x * center.x + plane.y * center.y + 
                        plane.z * center.z + plane.w;
        if (distance < -radius) {
            return false;
        }
    }
    return true;
}

} // namespace CloudStream 