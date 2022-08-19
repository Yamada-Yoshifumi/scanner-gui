#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
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
    void powerSignal(QString);

private:
    Ui::MainWindow *ui;
    QTimer *ros_timer;
    QTimer *velodyne_timer;
    QTimer *imu_timer;
    QTimer *camera_timer;
    ros::NodeHandlePtr n_;
    ros::Subscriber velodynesub;
    ros::Subscriber imusub;
    ros::Subscriber camerasub;
    VideoStreamer *videoStreamer;
    OpencvImageProvider *liveimageprovider;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    void paintStatus();
    void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);
    void updateImuStatus(const nav_msgs::OdometryConstPtr &msg);
    void updateCameraStatus(const sensor_msgs::ImageConstPtr &msg);
    QQuickView *qmlView;
    QGridLayout* central_widget_layout;
    QObject *power_button;
    QObject *power_button_bg;
    QObject *velodyne_indicator;
    QObject *imu_indicator;
    QObject *lidar_canvas;
    QObject *imu_canvas;
    QObject *camera_canvas;
    QObject *opencv_image;
    QWidget *container;
    int velodyne_status = 0;
    int imu_status = 0;
    int camera_status = 0;
    MyViz* myviz;
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
    //void systemOn();

};

#endif // MAINWINDOW_H
