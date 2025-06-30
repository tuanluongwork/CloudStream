#pragma once

#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <memory>
#include <atomic>
#include <array>
#include "core/PointCloud.h"

namespace CloudStream {

/**
 * @brief High-performance OpenGL renderer for point clouds using modern OpenGL
 */
class PointCloudRenderer : protected QOpenGLFunctions_4_5_Core {
public:
    struct RenderSettings {
        float point_size{2.0f};
        bool use_lighting{true};
        bool use_depth_test{true};
        bool use_point_sprites{true};
        bool use_instancing{false};
        float near_plane{0.1f};
        float far_plane{1000.0f};
        float fov{45.0f};
        glm::vec4 background_color{0.1f, 0.1f, 0.1f, 1.0f};
        
        // Performance settings
        size_t max_points_per_frame{1000000};
        bool use_frustum_culling{true};
        bool use_lod{true};
        float lod_distance_threshold{100.0f};
    };
    
    struct RenderStatistics {
        size_t points_rendered{0};
        size_t points_culled{0};
        float gpu_time_ms{0.0f};
        float cpu_time_ms{0.0f};
        size_t draw_calls{0};
        size_t vbo_memory_bytes{0};
    };
    
    PointCloudRenderer();
    ~PointCloudRenderer();
    
    // Initialization
    bool initialize();
    void cleanup();
    
    // Point cloud management
    void setPointCloud(PointCloud::Ptr cloud);
    void updatePointCloud(PointCloud::Ptr cloud);
    void clearPointCloud();
    
    // Rendering
    void render(const QMatrix4x4& view, const QMatrix4x4& projection);
    void renderWithCustomShader(QOpenGLShaderProgram* shader,
                               const QMatrix4x4& view, 
                               const QMatrix4x4& projection);
    
    // Settings
    void setRenderSettings(const RenderSettings& settings) { settings_ = settings; }
    const RenderSettings& getRenderSettings() const { return settings_; }
    RenderStatistics getRenderStatistics() const { return stats_; }
    
    // Camera helpers
    void setViewport(int width, int height);
    QMatrix4x4 getProjectionMatrix() const;
    
    // Shader management
    bool loadShaders(const QString& vertex_path, const QString& fragment_path);
    QOpenGLShaderProgram* getShaderProgram() { return shader_program_.get(); }
    
private:
    struct VertexData {
        float position[3];
        float normal[3];
        float color[4];
        float padding[2]; // Align to 48 bytes
    };
    
    struct UniformBlock {
        QMatrix4x4 model;
        QMatrix4x4 view;
        QMatrix4x4 projection;
        QMatrix4x4 normal_matrix;
        glm::vec4 light_position{10.0f, 10.0f, 10.0f, 1.0f};
        glm::vec4 light_color{1.0f, 1.0f, 1.0f, 1.0f};
        glm::vec4 ambient_color{0.2f, 0.2f, 0.2f, 1.0f};
        float point_size{2.0f};
        float near_plane{0.1f};
        float far_plane{1000.0f};
        float time{0.0f};
    };
    
    // OpenGL resources
    std::unique_ptr<QOpenGLShaderProgram> shader_program_;
    std::unique_ptr<QOpenGLVertexArrayObject> vao_;
    std::unique_ptr<QOpenGLBuffer> vbo_;
    std::unique_ptr<QOpenGLBuffer> ibo_;
    GLuint ubo_{0};
    GLuint query_id_{0};
    
    // Point cloud data
    PointCloud::Ptr point_cloud_;
    std::vector<VertexData> vertex_buffer_;
    std::atomic<bool> needs_update_{false};
    
    // Render state
    RenderSettings settings_;
    RenderStatistics stats_;
    int viewport_width_{800};
    int viewport_height_{600};
    UniformBlock uniforms_;
    
    // Methods
    void updateVertexBuffer();
    void uploadVertexData();
    void setupVertexAttributes();
    void updateUniforms(const QMatrix4x4& view, const QMatrix4x4& projection);
    void performFrustumCulling(const QMatrix4x4& mvp);
    bool createDefaultShaders();
    
    // Frustum culling
    struct Frustum {
        std::array<glm::vec4, 6> planes;
        
        void extract(const QMatrix4x4& mvp);
        bool containsPoint(const glm::vec3& point) const;
        bool containsSphere(const glm::vec3& center, float radius) const;
    };
    
    Frustum frustum_;
};

} // namespace CloudStream 