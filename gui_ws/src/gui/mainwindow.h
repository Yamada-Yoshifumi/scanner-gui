#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
#include "myviz.h"
#include <qtimer.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

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

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    void paintStatus();
    void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);
    QQuickView *qmlView;
    QGridLayout* central_widget_layout;
    QObject *power_button;
    QObject *power_button_bg;
    QObject *velodyne_indicator;
    QObject *lidar_canvas;
    QWidget *container;
    bool velodyne_status = false;
    MyViz* myviz;
    //ROSHandler* roshandler;
    bool power_toggled = false;

public Q_SLOTS:
    void createRVizEvent();
    void fullscreenToggle();
    void resetVelodyneStatus();
    void spinOnce();
    void powerClickedEmit();
    //void systemOn();

};

#endif // MAINWINDOW_H
