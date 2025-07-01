#include "visualization/MainWindow.h"
#include "visualization/GLWidget.h"
#include "network/Client.h"
#include "core/PointCloudProcessor.h"
#include "utils/Logger.h"
#include "utils/MathUtils.h"
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QColorDialog>
#include <QCloseEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QSettings>

namespace CloudStream {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    
    // Set window properties
    setWindowTitle("CloudStream - Point Cloud Viewer");
    resize(1200, 800);
    
    // Create central widget
    gl_widget_ = new GLWidget(this);
    setCentralWidget(gl_widget_);
    
    // Create UI elements
    createActions();
    createMenus();
    createToolBars();
    createDockWindows();
    createStatusBar();
    connectSignals();
    
    // Start timers
    stats_timer_ = new QTimer(this);
    stats_timer_->setInterval(100);
    stats_timer_->start();
    
    network_timer_ = new QTimer(this);
    network_timer_->setInterval(1000);
    
    // Load settings
    QSettings settings("CloudStream", "MainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    
    Logger::instance().info("MainWindow initialized");
}

MainWindow::~MainWindow() {
    // Save settings
    QSettings settings("CloudStream", "MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::loadPointCloudFile(const QString& filename) {
    if (filename.isEmpty()) return;
    
    Logger::instance().info("Loading point cloud from: {}", filename.toStdString());
    
    // In a real implementation, load from file using PCL or custom loader
    // For now, generate demo data
    generateDemoPointCloud();
    
    current_file_ = filename;
    modified_ = false;
    updateWindowTitle();
}

void MainWindow::loadDemoPointCloud() {
    Logger::instance().info("Loading demo point cloud");
    generateDemoPointCloud();
    
    current_file_ = "Demo Point Cloud";
    modified_ = false;
    updateWindowTitle();
}

void MainWindow::clearPointCloud() {
    gl_widget_->clearPointCloud();
    current_cloud_.reset();
    current_file_.clear();
    modified_ = false;
    updateWindowTitle();
}

bool MainWindow::connectToServer(const QString& host, uint16_t port) {
    if (!network_client_) {
        network_client_ = std::make_unique<Client>();
        
        // Set callbacks
        network_client_->setPointCloudCallback(
            [this](PointCloud::Ptr cloud) {
                QMetaObject::invokeMethod(this, [this, cloud]() {
                    onPointCloudReceived(cloud);
                }, Qt::QueuedConnection);
            });
        
        network_client_->setStatusCallback(
            [this](const Protocol::StatusUpdate& status) {
                QMetaObject::invokeMethod(this, [this]() {
                    updateNetworkStatus();
                }, Qt::QueuedConnection);
            });
    }
    
    network_client_->connect(host.toStdString(), port,
        [this](bool success, const std::string& error) {
            QMetaObject::invokeMethod(this, [this, success, error]() {
                if (success) {
                    network_timer_->start();
                    updateNetworkStatus();
                } else {
                    QMessageBox::critical(this, "Connection Error", 
                                        QString("Failed to connect: %1").arg(QString::fromStdString(error)));
                }
            }, Qt::QueuedConnection);
        });
    
    return true;
}

void MainWindow::disconnectFromServer() {
    if (network_client_) {
        network_client_->disconnect();
        network_timer_->stop();
        updateNetworkStatus();
    }
}

bool MainWindow::isConnected() const {
    return network_client_ && network_client_->isConnected();
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (modified_) {
        auto reply = QMessageBox::question(this, "Unsaved Changes",
                                         "Do you want to save your changes?",
                                         QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        
        if (reply == QMessageBox::Save) {
            onFileSave();
            event->accept();
        } else if (reply == QMessageBox::Discard) {
            event->accept();
        } else {
            event->ignore();
        }
    } else {
        event->accept();
    }
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event) {
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent* event) {
    const QMimeData* mimeData = event->mimeData();
    
    if (mimeData->hasUrls()) {
        QList<QUrl> urls = mimeData->urls();
        if (!urls.isEmpty()) {
            loadPointCloudFile(urls.first().toLocalFile());
        }
    }
}

void MainWindow::onFileOpen() {
    QString filename = QFileDialog::getOpenFileName(this,
        "Open Point Cloud",
        QString(),
        "Point Cloud Files (*.pcd *.ply *.xyz *.las);;All Files (*)");
    
    if (!filename.isEmpty()) {
        loadPointCloudFile(filename);
    }
}

void MainWindow::onFileSave() {
    if (current_file_.isEmpty()) {
        QString filename = QFileDialog::getSaveFileName(this,
            "Save Point Cloud",
            QString(),
            "Point Cloud Files (*.pcd);;All Files (*)");
        
        if (!filename.isEmpty()) {
            current_file_ = filename;
            // Save implementation
            modified_ = false;
            updateWindowTitle();
        }
    } else {
        // Save to current file
        modified_ = false;
        updateWindowTitle();
    }
}

void MainWindow::onFileExit() {
    close();
}

void MainWindow::onViewReset() {
    gl_widget_->resetView();
}

void MainWindow::onViewOrthographic(bool checked) {
    gl_widget_->setOrthographic(checked);
}

void MainWindow::onViewShowAxes(bool checked) {
    gl_widget_->setShowAxes(checked);
}

void MainWindow::onViewShowGrid(bool checked) {
    gl_widget_->setShowGrid(checked);
}

void MainWindow::onViewFullscreen() {
    if (isFullScreen()) {
        showNormal();
    } else {
        showFullScreen();
    }
}

void MainWindow::onToolsDownsample() {
    bool ok;
    double voxel_size = QInputDialog::getDouble(this, "Downsample",
                                               "Voxel size:", 0.01, 0.001, 10.0, 3, &ok);
    
    if (ok && current_cloud_) {
        PointCloudProcessor processor;
        auto downsampled = processor.voxelGridFilter(current_cloud_, voxel_size);
        
        current_cloud_ = downsampled;
        gl_widget_->setPointCloud(current_cloud_);
        modified_ = true;
        updateWindowTitle();
    }
}

void MainWindow::onToolsRemoveOutliers() {
    if (!current_cloud_) return;
    
    PointCloudProcessor processor;
    auto filtered = processor.removeStatisticalOutliers(current_cloud_);
    
    current_cloud_ = filtered;
    gl_widget_->setPointCloud(current_cloud_);
    modified_ = true;
    updateWindowTitle();
}

void MainWindow::onToolsComputeNormals() {
    if (!current_cloud_) return;
    
    PointCloudProcessor processor;
    processor.computeNormals(current_cloud_);
    
    gl_widget_->updatePointCloud(current_cloud_);
    modified_ = true;
    updateWindowTitle();
}

void MainWindow::onNetworkConnect() {
    bool ok;
    QString host = QInputDialog::getText(this, "Connect to Server",
                                        "Server address:", QLineEdit::Normal,
                                        "localhost", &ok);
    
    if (ok && !host.isEmpty()) {
        connectToServer(host, 8080);
    }
}

void MainWindow::onNetworkDisconnect() {
    disconnectFromServer();
}

void MainWindow::onNetworkStartServer() {
    // In a full implementation, start embedded server
    QMessageBox::information(this, "Server",
                           "Server functionality not implemented in this demo");
}

void MainWindow::onPointSizeChanged(int value) {
    auto settings = gl_widget_->getRenderSettings();
    settings.point_size = value;
    gl_widget_->setRenderSettings(settings);
    
    if (point_size_spinbox_) {
        point_size_spinbox_->setValue(value);
    }
}

void MainWindow::onFOVChanged(int value) {
    auto settings = gl_widget_->getRenderSettings();
    settings.fov = value;
    gl_widget_->setRenderSettings(settings);
    
    if (fov_spinbox_) {
        fov_spinbox_->setValue(value);
    }
}

void MainWindow::onBackgroundColorChanged() {
    QColor color = QColorDialog::getColor(Qt::black, this, "Select Background Color");
    if (color.isValid()) {
        auto settings = gl_widget_->getRenderSettings();
        settings.background_color = glm::vec4(color.redF(), color.greenF(), color.blueF(), 1.0f);
        gl_widget_->setRenderSettings(settings);
    }
}

void MainWindow::updateStatistics() {
    if (!gl_widget_) return;
    
    // Update FPS
    if (fps_label_) {
        fps_label_->setText(QString("FPS: %1").arg(gl_widget_->getFPS(), 0, 'f', 1));
    }
    
    // Update point count
    if (points_label_ && current_cloud_) {
        points_label_->setText(QString("Points: %1").arg(current_cloud_->size()));
    }
    
    // Update memory usage
    if (memory_label_) {
        auto stats = gl_widget_->getRenderStatistics();
        double memory_mb = stats.vbo_memory_bytes / (1024.0 * 1024.0);
        memory_label_->setText(QString("Memory: %1 MB").arg(memory_mb, 0, 'f', 2));
    }
}

void MainWindow::updateNetworkStatus() {
    if (!network_label_) return;
    
    if (isConnected()) {
        auto status = network_client_->getLastStatus();
        network_label_->setText(QString("Connected - %1 Mbps").arg(status.bandwidth_mbps, 0, 'f', 1));
    } else {
        network_label_->setText("Disconnected");
    }
}

void MainWindow::onPointCloudReceived(PointCloud::Ptr cloud) {
    current_cloud_ = cloud;
    gl_widget_->setPointCloud(cloud);
    
    current_file_ = "Network Stream";
    modified_ = false;
    updateWindowTitle();
}

void MainWindow::onPointCloudUpdated() {
    modified_ = true;
    updateWindowTitle();
}

void MainWindow::createActions() {
    // File actions
    open_action_ = new QAction(QIcon(":/icons/open.png"), "&Open...", this);
    open_action_->setShortcut(QKeySequence::Open);
    
    save_action_ = new QAction(QIcon(":/icons/save.png"), "&Save", this);
    save_action_->setShortcut(QKeySequence::Save);
    
    exit_action_ = new QAction("E&xit", this);
    exit_action_->setShortcut(QKeySequence::Quit);
    
    // View actions
    reset_view_action_ = new QAction(QIcon(":/icons/reset.png"), "&Reset View", this);
    reset_view_action_->setShortcut(Qt::Key_R);
    
    orthographic_action_ = new QAction("&Orthographic", this);
    orthographic_action_->setCheckable(true);
    
    show_axes_action_ = new QAction("Show &Axes", this);
    show_axes_action_->setCheckable(true);
    show_axes_action_->setChecked(true);
    
    show_grid_action_ = new QAction("Show &Grid", this);
    show_grid_action_->setCheckable(true);
    show_grid_action_->setChecked(true);
    
    fullscreen_action_ = new QAction("&Fullscreen", this);
    fullscreen_action_->setShortcut(Qt::Key_F11);
    
    // Tools actions
    downsample_action_ = new QAction("&Downsample...", this);
    remove_outliers_action_ = new QAction("Remove &Outliers", this);
    compute_normals_action_ = new QAction("Compute &Normals", this);
    
    // Network actions
    connect_action_ = new QAction(QIcon(":/icons/connect.png"), "&Connect...", this);
    disconnect_action_ = new QAction(QIcon(":/icons/disconnect.png"), "&Disconnect", this);
    start_server_action_ = new QAction("Start &Server", this);
}

void MainWindow::createMenus() {
    // File menu
    file_menu_ = menuBar()->addMenu("&File");
    file_menu_->addAction(open_action_);
    file_menu_->addAction(save_action_);
    file_menu_->addSeparator();
    file_menu_->addAction(exit_action_);
    
    // View menu
    view_menu_ = menuBar()->addMenu("&View");
    view_menu_->addAction(reset_view_action_);
    view_menu_->addSeparator();
    view_menu_->addAction(orthographic_action_);
    view_menu_->addAction(show_axes_action_);
    view_menu_->addAction(show_grid_action_);
    view_menu_->addSeparator();
    view_menu_->addAction(fullscreen_action_);
    
    // Tools menu
    tools_menu_ = menuBar()->addMenu("&Tools");
    tools_menu_->addAction(downsample_action_);
    tools_menu_->addAction(remove_outliers_action_);
    tools_menu_->addAction(compute_normals_action_);
    
    // Network menu
    network_menu_ = menuBar()->addMenu("&Network");
    network_menu_->addAction(connect_action_);
    network_menu_->addAction(disconnect_action_);
    network_menu_->addSeparator();
    network_menu_->addAction(start_server_action_);
    
    // Help menu
    help_menu_ = menuBar()->addMenu("&Help");
    help_menu_->addAction("&About", this, [this]() {
        QMessageBox::about(this, "About CloudStream",
                          "CloudStream - Real-time Point Cloud Streaming & Visualization\n\n"
                          "Version 1.0.0\n\n"
                          "A high-performance C++ application for point cloud processing.");
    });
}

void MainWindow::createToolBars() {
    // File toolbar
    file_toolbar_ = addToolBar("File");
    file_toolbar_->addAction(open_action_);
    file_toolbar_->addAction(save_action_);
    
    // View toolbar
    view_toolbar_ = addToolBar("View");
    view_toolbar_->addAction(reset_view_action_);
    
    // Tools toolbar
    tools_toolbar_ = addToolBar("Tools");
    tools_toolbar_->addAction(connect_action_);
    tools_toolbar_->addAction(disconnect_action_);
}

void MainWindow::createDockWindows() {
    // Properties dock
    properties_dock_ = new QDockWidget("Properties", this);
    properties_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget* properties_widget = new QWidget();
    QVBoxLayout* properties_layout = new QVBoxLayout(properties_widget);
    
    // Point size control
    QHBoxLayout* point_size_layout = new QHBoxLayout();
    point_size_layout->addWidget(new QLabel("Point Size:"));
    
    point_size_slider_ = new QSlider(Qt::Horizontal);
    point_size_slider_->setRange(1, 20);
    point_size_slider_->setValue(2);
    point_size_layout->addWidget(point_size_slider_);
    
    point_size_spinbox_ = new QSpinBox();
    point_size_spinbox_->setRange(1, 20);
    point_size_spinbox_->setValue(2);
    point_size_layout->addWidget(point_size_spinbox_);
    
    properties_layout->addLayout(point_size_layout);
    
    // FOV control
    QHBoxLayout* fov_layout = new QHBoxLayout();
    fov_layout->addWidget(new QLabel("Field of View:"));
    
    fov_slider_ = new QSlider(Qt::Horizontal);
    fov_slider_->setRange(10, 120);
    fov_slider_->setValue(45);
    fov_layout->addWidget(fov_slider_);
    
    fov_spinbox_ = new QSpinBox();
    fov_spinbox_->setRange(10, 120);
    fov_spinbox_->setValue(45);
    fov_spinbox_->setSuffix("°");
    fov_layout->addWidget(fov_spinbox_);
    
    properties_layout->addLayout(fov_layout);
    
    // Render options
    lighting_checkbox_ = new QCheckBox("Enable Lighting");
    lighting_checkbox_->setChecked(true);
    properties_layout->addWidget(lighting_checkbox_);
    
    point_sprites_checkbox_ = new QCheckBox("Point Sprites");
    point_sprites_checkbox_->setChecked(true);
    properties_layout->addWidget(point_sprites_checkbox_);
    
    // Background color button
    QPushButton* bg_color_button = new QPushButton("Background Color...");
    connect(bg_color_button, &QPushButton::clicked, this, &MainWindow::onBackgroundColorChanged);
    properties_layout->addWidget(bg_color_button);
    
    properties_layout->addStretch();
    
    properties_dock_->setWidget(properties_widget);
    addDockWidget(Qt::RightDockWidgetArea, properties_dock_);
    
    // Statistics dock
    statistics_dock_ = new QDockWidget("Statistics", this);
    statistics_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget* stats_widget = new QWidget();
    QVBoxLayout* stats_layout = new QVBoxLayout(stats_widget);
    
    points_label_ = new QLabel("Points: 0");
    stats_layout->addWidget(points_label_);
    
    fps_label_ = new QLabel("FPS: 0.0");
    stats_layout->addWidget(fps_label_);
    
    memory_label_ = new QLabel("Memory: 0.0 MB");
    stats_layout->addWidget(memory_label_);
    
    network_label_ = new QLabel("Disconnected");
    stats_layout->addWidget(network_label_);
    
    stats_layout->addStretch();
    
    statistics_dock_->setWidget(stats_widget);
    addDockWidget(Qt::LeftDockWidgetArea, statistics_dock_);
}

void MainWindow::createStatusBar() {
    statusBar()->showMessage("Ready");
}

void MainWindow::connectSignals() {
    // File actions
    connect(open_action_, &QAction::triggered, this, &MainWindow::onFileOpen);
    connect(save_action_, &QAction::triggered, this, &MainWindow::onFileSave);
    connect(exit_action_, &QAction::triggered, this, &MainWindow::onFileExit);
    
    // View actions
    connect(reset_view_action_, &QAction::triggered, this, &MainWindow::onViewReset);
    connect(orthographic_action_, &QAction::toggled, this, &MainWindow::onViewOrthographic);
    connect(show_axes_action_, &QAction::toggled, this, &MainWindow::onViewShowAxes);
    connect(show_grid_action_, &QAction::toggled, this, &MainWindow::onViewShowGrid);
    connect(fullscreen_action_, &QAction::triggered, this, &MainWindow::onViewFullscreen);
    
    // Tools actions
    connect(downsample_action_, &QAction::triggered, this, &MainWindow::onToolsDownsample);
    connect(remove_outliers_action_, &QAction::triggered, this, &MainWindow::onToolsRemoveOutliers);
    connect(compute_normals_action_, &QAction::triggered, this, &MainWindow::onToolsComputeNormals);
    
    // Network actions
    connect(connect_action_, &QAction::triggered, this, &MainWindow::onNetworkConnect);
    connect(disconnect_action_, &QAction::triggered, this, &MainWindow::onNetworkDisconnect);
    connect(start_server_action_, &QAction::triggered, this, &MainWindow::onNetworkStartServer);
    
    // Property controls
    connect(point_size_slider_, &QSlider::valueChanged, this, &MainWindow::onPointSizeChanged);
    connect(point_size_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged),
            point_size_slider_, &QSlider::setValue);
    
    connect(fov_slider_, &QSlider::valueChanged, this, &MainWindow::onFOVChanged);
    connect(fov_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged),
            fov_slider_, &QSlider::setValue);
    
    connect(lighting_checkbox_, &QCheckBox::toggled, [this](bool checked) {
        auto settings = gl_widget_->getRenderSettings();
        settings.use_lighting = checked;
        gl_widget_->setRenderSettings(settings);
    });
    
    connect(point_sprites_checkbox_, &QCheckBox::toggled, [this](bool checked) {
        auto settings = gl_widget_->getRenderSettings();
        settings.use_point_sprites = checked;
        gl_widget_->setRenderSettings(settings);
    });
    
    // GL widget signals
    connect(gl_widget_, &GLWidget::pointCloudChanged, this, &MainWindow::onPointCloudUpdated);
    connect(gl_widget_, &GLWidget::fpsUpdated, [this](float fps) {
        if (fps_label_) {
            fps_label_->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
        }
    });
    
    // Timers
    connect(stats_timer_, &QTimer::timeout, this, &MainWindow::updateStatistics);
    connect(network_timer_, &QTimer::timeout, this, &MainWindow::updateNetworkStatus);
}

void MainWindow::generateDemoPointCloud() {
    auto cloud = std::make_shared<PointCloud>();
    
    // Generate a colorful point cloud (torus shape)
    const int u_steps = 100;
    const int v_steps = 50;
    const float R = 3.0f; // Major radius
    const float r = 1.0f; // Minor radius
    
    for (int u = 0; u < u_steps; ++u) {
        float theta = 2.0f * M_PI * u / u_steps;
        
        for (int v = 0; v < v_steps; ++v) {
            float phi = 2.0f * M_PI * v / v_steps;
            
            Point point;
            
            // Torus parametric equation
            point.position = glm::vec3(
                (R + r * cos(phi)) * cos(theta),
                r * sin(phi),
                (R + r * cos(phi)) * sin(theta)
            );
            
            // Normal vector
            glm::vec3 center = glm::vec3(R * cos(theta), 0, R * sin(theta));
            point.normal = glm::normalize(point.position - center);
            
            // Colorful gradient based on position
            point.color = glm::vec4(
                0.5f + 0.5f * sin(theta),
                0.5f + 0.5f * cos(phi),
                0.5f + 0.5f * sin(theta + phi),
                1.0f
            );
            
            point.intensity = 1.0f;
            
            cloud->addPoint(point);
        }
    }
    
    current_cloud_ = cloud;
    gl_widget_->setPointCloud(cloud);
}

void MainWindow::updateWindowTitle() {
    QString title = "CloudStream";
    
    if (!current_file_.isEmpty()) {
        title += " - " + QFileInfo(current_file_).fileName();
        if (modified_) {
            title += "*";
        }
    }
    
    setWindowTitle(title);
}

} // namespace CloudStream 