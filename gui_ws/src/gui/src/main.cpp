#include <QQmlApplicationEngine>
#include <QApplication>
#include <QQuickWidget>
#include "mainwindow.h"

#include <ros/ros.h>
#include "roshandler.h"
#include <QQuickView>
#include <QQuickItem>
#include <QHBoxLayout>
#include <QPushButton>
#include <QtQml>
#include <QThread>
#include <QTimer>
#include <chrono>
#include <thread>


class PowerThread: public QThread{
    ROSHandler* roshandler;
    MainWindow *mainwindow;
    private:
        QTimer *thread_timer;
        int current_velodyne_status = 0;
        int current_imu_status = 0;

    public:

        explicit PowerThread(MainWindow *mainwindow_)
            : QThread(), mainwindow(mainwindow_) {}

        void spinThreadOnce(){
            if (current_velodyne_status != mainwindow->velodyne_status){
                current_velodyne_status = mainwindow->velodyne_status;
                roshandler->velodyneCmd = (mainwindow->velodyne_status == 1) ? 0 : 1;
            }
            if (current_imu_status != mainwindow->imu_status){
                current_imu_status = mainwindow->imu_status;
                roshandler->imuCmd = (mainwindow->imu_status == 1) ? 0 : 1;
            }
        }

        void run() override{

            thread_timer = new QTimer();
            thread_timer->start(100);

            roshandler = new ROSHandler();

            connect(mainwindow, &MainWindow::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle, Qt::QueuedConnection);
            connect(thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
            exec();
        }
};

int main(int argc, char **argv)
{
    ros::init( argc, argv, "qt_gui");
    ros::NodeHandlePtr n_;
    n_.reset(new ros::NodeHandle("~"));

    QApplication app( argc, argv );

    MainWindow* mainwindow = new MainWindow();


    mainwindow->setStyleSheet("background-color : #620b66");
    mainwindow->show();
    PowerThread* power_thread = new PowerThread(mainwindow);
    power_thread->start();

    app.exec();
    system("rosnode kill gazebo");
    system("rosnode kill usb_cam");
    system("killall -9 gzserver");
    delete mainwindow;
    delete power_thread;
}
