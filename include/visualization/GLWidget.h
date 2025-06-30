#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QMatrix4x4>
#include <QVector3D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <memory>
#include "core/PointCloud.h"
#include "visualization/PointCloudRenderer.h"

namespace CloudStream {

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
    Q_OBJECT

public:
    explicit GLWidget(QWidget* parent = nullptr);
    ~GLWidget();
    
    // Point cloud management
    void setPointCloud(PointCloud::Ptr cloud);
    PointCloud::Ptr getPointCloud() const { return point_cloud_; }
    void clearPointCloud();
    
    // Camera control
    void resetView();
    void setViewDirection(const QVector3D& direction);
    void fitToScreen();
    
    // Rendering settings
    void setRenderSettings(const PointCloudRenderer::RenderSettings& settings);
    PointCloudRenderer::RenderSettings getRenderSettings() const;
    
    // Display options
    void setShowAxes(bool show) { show_axes_ = show; update(); }
    void setShowGrid(bool show) { show_grid_ = show; update(); }
    void setOrthographic(bool ortho);
    
    // Statistics
    float getFPS() const { return fps_; }
    PointCloudRenderer::RenderStatistics getRenderStatistics() const;

signals:
    void pointCloudChanged();
    void cameraChanged();
    void fpsUpdated(float fps);

protected:
    // QOpenGLWidget interface
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    
    // Mouse interaction
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    
private:
    // Camera control
    struct Camera {
        QVector3D position{0.0f, 0.0f, 5.0f};
        QVector3D target{0.0f, 0.0f, 0.0f};
        QVector3D up{0.0f, 1.0f, 0.0f};
        float fov{45.0f};
        float near_plane{0.1f};
        float far_plane{1000.0f};
        bool orthographic{false};
        float ortho_size{10.0f};
    };
    
    void updateCamera();
    void drawAxes();
    void drawGrid();
    QMatrix4x4 getViewMatrix() const;
    QMatrix4x4 getProjectionMatrix() const;
    
    // Rendering
    std::unique_ptr<PointCloudRenderer> renderer_;
    PointCloud::Ptr point_cloud_;
    
    // Camera
    Camera camera_;
    
    // Mouse interaction
    QPoint last_mouse_pos_;
    Qt::MouseButtons mouse_buttons_;
    float rotation_speed_{0.5f};
    float pan_speed_{0.01f};
    float zoom_speed_{0.1f};
    
    // Display options
    bool show_axes_{true};
    bool show_grid_{true};
    QColor background_color_{25, 25, 25};
    
    // Performance monitoring
    float fps_{0.0f};
    int frame_count_{0};
    std::chrono::steady_clock::time_point last_fps_update_;
};

} // namespace CloudStream 