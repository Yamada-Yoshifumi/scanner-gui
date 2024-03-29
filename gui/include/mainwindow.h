#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
#include <QLabel>
#include "myviz.h"
#include <qtimer.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
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

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
    void rvizRenderSignal(QString);
    void powerButtonPressed();
    void scanButtonPressed();
    void recordButtonPressed();
    void powerSignal(QString);
    void scanSignal(QString);
    void exit(QString);
    void recordSignal(QString);
    void reconstructionToggled(int);

private:
    Ui::MainWindow *ui;
    QTimer *ros_timer;
    QTimer *velodyne_timer;
    QTimer *imu_timer;
    QTimer *camera_timer;
    QTimer* scan_countdown_timer;
    ros::NodeHandlePtr n_;
    ros::Subscriber velodynesub;
    ros::Subscriber imusub;
    ros::Subscriber camerasub;
    VideoStreamer *videoStreamer;
    OpencvImageProvider *liveimageprovider;
    QSize newSize;
    bool settings_shown = false;
    bool changeInDatabaseResponse();

    QPropertyAnimation *settings_animation;
    QPropertyAnimation *rviz_animation;
    QPropertyAnimation *qml_animation;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    void paintStatus();
    void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);
    void updateImuStatus(const nav_msgs::OdometryConstPtr &msg);
    void updateCameraStatus(const sensor_msgs::ImageConstPtr &msg);
    void switchVideoSource(int source);
    QQuickView *qmlView;
    QQuickView *settingsqmlView;
    AnimatedGridLayout* central_widget_layout;
    QObject *power_button;
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

    QObject *opencv_image;
    QWidget *container;
    QWidget *settings_container;
    int velodyne_status = 0;
    int imu_status = 0;
    int camera_status = 0;
    int scan_status = 0;
    int record_status = 0;
    MyViz* myviz;
    QLabel* countdown_widget;
    //ROSHandler* roshandler;
    bool power_toggled = false;
    bool counter;


public Q_SLOTS:
    void createRVizEvent();
    void fullscreenToggle();
    void resetVelodyneStatus();
    void resetImuStatus();
    void resetCameraStatus();
    void spinOnce();
    void imageReload();
    void powerClickedEmit();
    void scanClickedEmit();
    void recordClickedEmit();
    void updateCountDownNum();
    void toggleSettings();
    void closeWindow();
    //void systemOn();
};

#endif // MAINWINDOW_H
