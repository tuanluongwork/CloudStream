#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include "core/PointCloud.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QToolBar;
class QStatusBar;
class QDockWidget;
class QSlider;
class QSpinBox;
class QCheckBox;
class QLabel;
QT_END_NAMESPACE

namespace CloudStream {

class GLWidget;
class Client;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
    
    // Point cloud operations
    void loadPointCloudFile(const QString& filename);
    void loadDemoPointCloud();
    void clearPointCloud();
    
    // Network operations
    bool connectToServer(const QString& host, uint16_t port);
    void disconnectFromServer();
    bool isConnected() const;

protected:
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private slots:
    // File menu
    void onFileOpen();
    void onFileSave();
    void onFileExit();
    
    // View menu
    void onViewReset();
    void onViewOrthographic(bool checked);
    void onViewShowAxes(bool checked);
    void onViewShowGrid(bool checked);
    void onViewFullscreen();
    
    // Tools menu
    void onToolsDownsample();
    void onToolsRemoveOutliers();
    void onToolsComputeNormals();
    
    // Network menu
    void onNetworkConnect();
    void onNetworkDisconnect();
    void onNetworkStartServer();
    
    // Settings
    void onPointSizeChanged(int value);
    void onFOVChanged(int value);
    void onBackgroundColorChanged();
    
    // Updates
    void updateStatistics();
    void updateNetworkStatus();
    
    // Point cloud updates
    void onPointCloudReceived(PointCloud::Ptr cloud);
    void onPointCloudUpdated();

private:
    void createActions();
    void createMenus();
    void createToolBars();
    void createDockWindows();
    void createStatusBar();
    void connectSignals();
    
    void generateDemoPointCloud();
    void updateWindowTitle();
    
    // Central widget
    GLWidget* gl_widget_;
    
    // Menus
    QMenu* file_menu_;
    QMenu* view_menu_;
    QMenu* tools_menu_;
    QMenu* network_menu_;
    QMenu* help_menu_;
    
    // Actions
    QAction* open_action_;
    QAction* save_action_;
    QAction* exit_action_;
    QAction* reset_view_action_;
    QAction* orthographic_action_;
    QAction* show_axes_action_;
    QAction* show_grid_action_;
    QAction* fullscreen_action_;
    QAction* downsample_action_;
    QAction* remove_outliers_action_;
    QAction* compute_normals_action_;
    QAction* connect_action_;
    QAction* disconnect_action_;
    QAction* start_server_action_;
    
    // Toolbars
    QToolBar* file_toolbar_;
    QToolBar* view_toolbar_;
    QToolBar* tools_toolbar_;
    
    // Dock widgets
    QDockWidget* properties_dock_;
    QDockWidget* statistics_dock_;
    
    // Controls
    QSlider* point_size_slider_;
    QSpinBox* point_size_spinbox_;
    QSlider* fov_slider_;
    QSpinBox* fov_spinbox_;
    QCheckBox* lighting_checkbox_;
    QCheckBox* point_sprites_checkbox_;
    
    // Statistics labels
    QLabel* points_label_;
    QLabel* fps_label_;
    QLabel* memory_label_;
    QLabel* network_label_;
    
    // Timers
    QTimer* stats_timer_;
    QTimer* network_timer_;
    
    // Data
    PointCloud::Ptr current_cloud_;
    std::unique_ptr<Client> network_client_;
    
    // State
    QString current_file_;
    bool modified_{false};
};

} // namespace CloudStream 