#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QToolButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
#include <QLabel>
#include <qtimer.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "videostreamer.h"
#include "opencvimageprovider.h"
#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>
#include "animatedgridlayout.h"
using std::stringstream;
#include <stdio.h>
#include <string>
using std::string;
#include <sstream>
#include "qnamespace.h"
//#include "ui_mainwindow.h"
#include <QQuickView>
#include <QQuickItem>
#include <QPushButton>
#include <QtQml>
#include <QtQuick>
#include <QGraphicsOpacityEffect>
#include <QColor>
#include <thread>
#include <unistd.h>
#include "touchpad.h"
#include "logterminal.h"
#include "roundedbutton.h"
#include "roshandler.h"
#include <QComboBox>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

namespace rviz_common
{
class Display;
class RenderPanel;
class VisualizationManager;
}
/*
class ROSMessageDetector : public rclcpp::Node
{
public:

    std::string camera_topic = "/rrbot/camera1/image_raw";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodynesub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imusub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camerasub;

    explicit ROSMessageDetector()
        :Node("message_detector")
    {
        velodynesub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 20, std::bind(&MyViz::updateVelodyneStatus, myviz, _1));
        imusub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 20, std::bind(&MyViz::updateImuStatus, myviz, _1));
        camerasub = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 20, std::bind(&MyViz::updateCameraStatus, myviz, _1));
    }

};
*/
class MyViz: public QMainWindow, public rviz_common::WindowManagerInterface
{
Q_OBJECT
signals:
    void rvizRenderSignal(QString);
    void powerButtonPressed();
    void scanButtonPressed();
    void recordButtonPressed();
    void powerSignal(QString);
    void scanSignal(QString);
    void recordSignal(QString);
    void exit(QString);
    
    void reconstructionToggled(int);

public:
  MyViz(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent = 0);

  QWidget * getParentWindow() override;
  rviz_common::PanelDockWidget * addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
  void setStatus(const QString & message) override;

  void DisplayGrid();

  void resizeEvent(QResizeEvent* event);
  void paintStatus();
  void updateVelodyneStatus(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
  void updateImuStatus(const nav_msgs::msg::Odometry::SharedPtr &msg);
  void updateCameraStatus(const sensor_msgs::msg::Image::SharedPtr &msg);

  QQuickView *qmlView;
  QQuickView *settingsqmlView;
  AnimatedGridLayout* central_widget_layout;
  QObject *power_button;
  QPushButton* fullscreen_button;
  QObject *power_button_bg;
  QObject *scan_button;
  QObject *record_button;
  QObject *settings_toggle_button;
  QObject *velodyne_indicator;
  QObject *imu_indicator;
  QObject *lidar_canvas;
  QObject *imu_canvas;
  QObject *camera_canvas;
  QObject *lidar_status_text;
  QObject *imu_status_text;
  QObject *camera_status_text;
  QObject *lidar_status_pic;
  QObject *imu_status_pic;
  QObject *camera_status_pic;
  QObject *video_combo;

  QObject *opencv_image;
  QWidget *container;
  QWidget *settings_container;
  int velodyne_status = 0;
  int imu_status = 0;
  int camera_status = 0;
  int scan_status = 0;
  int record_status = 0;
  QLabel* countdown_widget;
  //ROSHandler* roshandler;
  bool power_toggled = false;
  bool counter;

  RoundedButton* zoomin_button;
  RoundedButton* zoomout_button;
  RoundedButton* reset_button;
  QComboBox* combo;
  QRect* rect_1;
  QRect* rect_2;
  QRect* rect_3;

  TouchPad* touchpad;
  LogTerminal* logterminal;
  QGridLayout* secondary_layout;
  QGridLayout* tertiary_layout;
  QWidget* secondary_widget;

  QTimer *velodyne_timer;
  QTimer *imu_timer;
  QTimer *camera_timer;

  VideoStreamer *videoStreamer;
  QPoint previous_touchp;
  double current_pitch;
  double current_yaw;
  double current_f_distance;
  double current_f_point_x;
  double current_f_point_y;
  double current_f_point_z;
  double fixed_f_point_x;
  double fixed_f_point_y;
  double fixed_f_point_z;
  bool wheel_e_inprogress;
  QString current_pointcloud_pattern;
  bool eventFilter(QObject * p_obj, QEvent * p_event);
    rviz_common::VisualizationManager * manager_;

private slots:
  void setThickness( int thickness_percent );
  void setCellSize( int cell_size_percent );
  //void closeEvent(QCloseEvent *event);

  public Q_SLOTS:
      //void createRVizEvent();
      void fullscreenToggle();
      void resetVelodyneStatus();
      void resetImuStatus();
      void resetCameraStatus();
      //void spinOnce();
      void imageReload();
      void powerClickedEmit();
      void scanClickedEmit();
      void recordClickedEmit();
      void updateCountDownNum();
      void toggleSettings();
      void closeWindow();
      void resetView();
      void manualZoomIn();
      void manualZoomOut();
      void colourPatternChanged();
      //void systemOn();
private:
  //rclcpp::Node::SharedPtr ros_message_detector;
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodynesub;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imusub;
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camerasub;
  void initializeRViz();
  
  OpencvImageProvider *liveimageprovider;
    QTimer *ros_timer;
    
    QTimer *scan_countdown_timer;

  QApplication * app_;
  QWidget * central_widget;

  QSize newSize;
  bool settings_shown = false;
  bool changeInDatabaseResponse();

  QPropertyAnimation *settings_animation;
  QPropertyAnimation *rviz_animation;
  QPropertyAnimation *qml_animation;

  rviz_common::RenderPanel * render_panel_;
  rviz_common::Display * grid_, * pointcloud_, *tf_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;



//virtual: ~MyViz();
};
