#include "visualization/GLWidget.h"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include <QOpenGLContext>
#include <QMouseEvent>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

namespace CloudStream {

GLWidget::GLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , last_fps_update_(std::chrono::steady_clock::now()) {
    
    // Enable mouse tracking
    setMouseTracking(true);
    
    // Set format
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setSamples(4);
    format.setVersion(4, 5);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);
}

GLWidget::~GLWidget() {
    makeCurrent();
    renderer_.reset();
    doneCurrent();
}

void GLWidget::setPointCloud(PointCloud::Ptr cloud) {
    makeCurrent();
    point_cloud_ = cloud;
    
    if (renderer_) {
        renderer_->setPointCloud(cloud);
    }
    
    doneCurrent();
    
    emit pointCloudChanged();
    update();
}

void GLWidget::clearPointCloud() {
    setPointCloud(nullptr);
}

void GLWidget::resetView() {
    camera_.position = QVector3D(0.0f, 0.0f, 5.0f);
    camera_.target = QVector3D(0.0f, 0.0f, 0.0f);
    camera_.up = QVector3D(0.0f, 1.0f, 0.0f);
    
    updateCamera();
    update();
}

void GLWidget::setViewDirection(const QVector3D& direction) {
    camera_.position = camera_.target - direction * 5.0f;
    updateCamera();
    update();
}

void GLWidget::fitToScreen() {
    if (!point_cloud_ || point_cloud_->empty()) return;
    
    auto bbox = point_cloud_->getBoundingBox();
    glm::vec3 center = bbox.center();
    float diagonal = bbox.diagonal();
    
    camera_.target = QVector3D(center.x, center.y, center.z);
    camera_.position = camera_.target + QVector3D(0, 0, diagonal * 2.0f);
    
    updateCamera();
    update();
}

void GLWidget::setRenderSettings(const PointCloudRenderer::RenderSettings& settings) {
    if (renderer_) {
        renderer_->setRenderSettings(settings);
        update();
    }
}

PointCloudRenderer::RenderSettings GLWidget::getRenderSettings() const {
    if (renderer_) {
        return renderer_->getRenderSettings();
    }
    return PointCloudRenderer::RenderSettings{};
}

void GLWidget::setOrthographic(bool ortho) {
    camera_.orthographic = ortho;
    updateCamera();
    update();
}

PointCloudRenderer::RenderStatistics GLWidget::getRenderStatistics() const {
    if (renderer_) {
        return renderer_->getRenderStatistics();
    }
    return PointCloudRenderer::RenderStatistics{};
}

void GLWidget::initializeGL() {
    initializeOpenGLFunctions();
    
    Logger::instance().info("Initializing OpenGL widget");
    Logger::instance().info("OpenGL Version: {}", 
                          reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    Logger::instance().info("GLSL Version: {}", 
                          reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)));
    
    // Create renderer
    renderer_ = std::make_unique<PointCloudRenderer>();
    if (!renderer_->initialize()) {
        Logger::instance().error("Failed to initialize point cloud renderer");
        return;
    }
    
    // Set default OpenGL state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);
    
    // Set background color
    glClearColor(background_color_.redF(), 
                 background_color_.greenF(), 
                 background_color_.blueF(), 
                 background_color_.alphaF());
    
    // Initialize camera
    resetView();
}

void GLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    
    if (renderer_) {
        renderer_->setViewport(w, h);
    }
    
    updateCamera();
}

void GLWidget::paintGL() {
    // Clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Update FPS
    frame_count_++;
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_update_).count();
    
    if (duration >= 1000) {
        fps_ = frame_count_ * 1000.0f / duration;
        frame_count_ = 0;
        last_fps_update_ = now;
        emit fpsUpdated(fps_);
    }
    
    // Get matrices
    QMatrix4x4 view = getViewMatrix();
    QMatrix4x4 projection = getProjectionMatrix();
    
    // Render point cloud
    if (renderer_ && point_cloud_) {
        renderer_->render(view, projection);
    }
    
    // Draw helpers
    if (show_axes_) {
        drawAxes();
    }
    
    if (show_grid_) {
        drawGrid();
    }
}

void GLWidget::mousePressEvent(QMouseEvent* event) {
    last_mouse_pos_ = event->pos();
    mouse_buttons_ = event->buttons();
}

void GLWidget::mouseMoveEvent(QMouseEvent* event) {
    if (mouse_buttons_ == Qt::NoButton) return;
    
    QPoint delta = event->pos() - last_mouse_pos_;
    last_mouse_pos_ = event->pos();
    
    if (mouse_buttons_ & Qt::LeftButton) {
        // Rotate camera
        float dx = delta.x() * rotation_speed_;
        float dy = delta.y() * rotation_speed_;
        
        // Calculate rotation around target
        QVector3D to_camera = camera_.position - camera_.target;
        float distance = to_camera.length();
        
        // Horizontal rotation (around Y axis)
        QMatrix4x4 yaw_rotation;
        yaw_rotation.rotate(-dx, QVector3D(0, 1, 0));
        
        // Vertical rotation (around camera's right axis)
        QVector3D right = QVector3D::crossProduct(camera_.up, to_camera).normalized();
        QMatrix4x4 pitch_rotation;
        pitch_rotation.rotate(-dy, right);
        
        // Apply rotations
        to_camera = yaw_rotation * pitch_rotation * to_camera;
        camera_.position = camera_.target + to_camera.normalized() * distance;
        
        // Update up vector
        camera_.up = pitch_rotation * camera_.up;
        camera_.up.normalize();
        
        updateCamera();
    } else if (mouse_buttons_ & Qt::MiddleButton) {
        // Pan camera
        QVector3D to_camera = (camera_.position - camera_.target).normalized();
        QVector3D right = QVector3D::crossProduct(camera_.up, to_camera).normalized();
        QVector3D up = QVector3D::crossProduct(to_camera, right).normalized();
        
        float distance = (camera_.position - camera_.target).length();
        float pan_factor = pan_speed_ * distance;
        
        QVector3D pan = right * (-delta.x() * pan_factor) + up * (delta.y() * pan_factor);
        
        camera_.position += pan;
        camera_.target += pan;
        
        updateCamera();
    }
    
    update();
}

void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
    mouse_buttons_ = Qt::NoButton;
}

void GLWidget::wheelEvent(QWheelEvent* event) {
    float delta = event->angleDelta().y() / 120.0f;
    
    // Zoom camera
    QVector3D to_camera = camera_.position - camera_.target;
    float distance = to_camera.length();
    
    distance *= (1.0f - delta * zoom_speed_);
    distance = std::max(0.1f, std::min(1000.0f, distance));
    
    camera_.position = camera_.target + to_camera.normalized() * distance;
    
    if (camera_.orthographic) {
        camera_.ortho_size *= (1.0f - delta * zoom_speed_);
        camera_.ortho_size = std::max(0.1f, std::min(1000.0f, camera_.ortho_size));
    }
    
    updateCamera();
    update();
}

void GLWidget::updateCamera() {
    emit cameraChanged();
}

void GLWidget::drawAxes() {
    // Simple axes drawing using lines
    if (!renderer_ || !renderer_->getShaderProgram()) return;
    
    glLineWidth(2.0f);
    
    // Create axis vertices
    const float axis_length = 5.0f;
    struct AxisVertex {
        glm::vec3 position;
        glm::vec4 color;
    };
    
    std::vector<AxisVertex> axes = {
        // X axis (red)
        {{0, 0, 0}, {1, 0, 0, 1}},
        {{axis_length, 0, 0}, {1, 0, 0, 1}},
        // Y axis (green)
        {{0, 0, 0}, {0, 1, 0, 1}},
        {{0, axis_length, 0}, {0, 1, 0, 1}},
        // Z axis (blue)
        {{0, 0, 0}, {0, 0, 1, 1}},
        {{0, 0, axis_length}, {0, 0, 1, 1}}
    };
    
    // Create temporary VAO and VBO for axes
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, axes.size() * sizeof(AxisVertex), axes.data(), GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(AxisVertex), (void*)0);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(AxisVertex), (void*)offsetof(AxisVertex, color));
    
    // Use renderer's shader with custom uniforms
    auto shader = renderer_->getShaderProgram();
    shader->bind();
    shader->setUniformValue("model", QMatrix4x4());
    shader->setUniformValue("view", getViewMatrix());
    shader->setUniformValue("projection", getProjectionMatrix());
    
    glDrawArrays(GL_LINES, 0, 6);
    
    shader->release();
    
    // Cleanup
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
}

void GLWidget::drawGrid() {
    // Simple grid drawing
    if (!renderer_ || !renderer_->getShaderProgram()) return;
    
    glLineWidth(1.0f);
    
    const float grid_size = 20.0f;
    const int grid_lines = 21;
    const float step = grid_size / (grid_lines - 1);
    
    std::vector<glm::vec3> grid_vertices;
    glm::vec4 grid_color(0.3f, 0.3f, 0.3f, 0.5f);
    
    // Generate grid lines along X
    for (int i = 0; i < grid_lines; ++i) {
        float pos = -grid_size / 2 + i * step;
        grid_vertices.push_back({pos, 0, -grid_size / 2});
        grid_vertices.push_back({pos, 0, grid_size / 2});
    }
    
    // Generate grid lines along Z
    for (int i = 0; i < grid_lines; ++i) {
        float pos = -grid_size / 2 + i * step;
        grid_vertices.push_back({-grid_size / 2, 0, pos});
        grid_vertices.push_back({grid_size / 2, 0, pos});
    }
    
    // Create temporary VAO and VBO for grid
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, grid_vertices.size() * sizeof(glm::vec3), grid_vertices.data(), GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    
    // Use renderer's shader
    auto shader = renderer_->getShaderProgram();
    shader->bind();
    shader->setUniformValue("model", QMatrix4x4());
    shader->setUniformValue("view", getViewMatrix());
    shader->setUniformValue("projection", getProjectionMatrix());
    
    // Set uniform color for all grid lines
    glVertexAttrib4f(2, grid_color.r, grid_color.g, grid_color.b, grid_color.a);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(grid_vertices.size()));
    
    glDisable(GL_BLEND);
    shader->release();
    
    // Cleanup
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
}

QMatrix4x4 GLWidget::getViewMatrix() const {
    QMatrix4x4 view;
    view.lookAt(camera_.position, camera_.target, camera_.up);
    return view;
}

QMatrix4x4 GLWidget::getProjectionMatrix() const {
    QMatrix4x4 projection;
    
    float aspect = static_cast<float>(width()) / height();
    
    if (camera_.orthographic) {
        float half_width = camera_.ortho_size * aspect * 0.5f;
        float half_height = camera_.ortho_size * 0.5f;
        projection.ortho(-half_width, half_width, 
                        -half_height, half_height,
                        camera_.near_plane, camera_.far_plane);
    } else {
        projection.perspective(camera_.fov, aspect, 
                             camera_.near_plane, camera_.far_plane);
    }
    
    return projection;
}

} // namespace CloudStream 