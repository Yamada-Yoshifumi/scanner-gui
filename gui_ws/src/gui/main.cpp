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
    MainWindow *mainwindow;
    private:
        ros::NodeHandlePtr n_;
        QTimer *ros_thread_timer;

    public:
        explicit PowerThread(MainWindow *mainwindow_, ros::NodeHandle &n)
            : QThread(), mainwindow(mainwindow_) { n_.reset(&n);}

        void spinThreadOnce(){
            if(ros::ok()){
                ros::spinOnce();
            }
            else
                QApplication::quit();
        }

        void run() override{

            ROSHandler *roshandler = new ROSHandler();
            ros_thread_timer = new QTimer();
            ros_thread_timer->start(200);
            connect(mainwindow, &MainWindow::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle);
            connect(ros_thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
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
    PowerThread* power_thread = new PowerThread(mainwindow, *n_);
    power_thread->start();

    app.exec();
    delete mainwindow;
    delete power_thread;
}
