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
    void powerSignal(QString);
    void scanSignal(QString);

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
    bool counter;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    void paintStatus();
    void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);
    void updateImuStatus(const nav_msgs::OdometryConstPtr &msg);
    void updateCameraStatus(const sensor_msgs::ImageConstPtr &msg);
    QQuickView *qmlView;
    QQuickView *settingsqmlView;
    QGridLayout* central_widget_layout;
    QObject *power_button;
    QObject *power_button_bg;
    QObject *scan_button;
    QObject *scan_button_bg;
    QObject *settings_show_button;
    QObject *settings_close_button;
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
    MyViz* myviz;
    QLabel* countdown_widget;
    //ROSHandler* roshandler;
    bool power_toggled = false;

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
    void updateCountDownNum();
    void showSettings();
    void closeSettings();
    //void systemOn();

};

#endif // MAINWINDOW_H
