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
        sqlite3* db;
        sqlite3_stmt* stmt;
        int database_camera_exposure_t = 20;

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
            changeInDatabaseResponse();
        }

        void run() override{

            thread_timer = new QTimer();
            thread_timer->start(100);

            roshandler = new ROSHandler();

            connect(mainwindow, &MainWindow::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle, Qt::QueuedConnection);
            connect(mainwindow, &MainWindow::scanButtonPressed, roshandler, &ROSHandler::scanToggle, Qt::QueuedConnection);
            connect(thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
            exec();
        }

        bool changeInDatabaseResponse(){
            stringstream ss1;
            if(sqlite3_open((mainwindow->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/0f77255fc0fdc1559526d7eca42b4253.sqlite").c_str(), &db) != SQLITE_OK) {
                printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                return false;
            }

            //Callback service for day/night light mode toggle
            ss1 << "SELECT * FROM BooleanSettings where name = \"Daylight Mode\" LIMIT 1;";
            string sql(ss1.str());
            if(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_finalize(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) == 1){
                    mainwindow->setStyleSheet("background-color: white;");
                }
                else{
                    mainwindow->setStyleSheet("background-color: #442e5d;");
                }
            }

            //Callback service for camera_exposure time
            stringstream ss2;
            ss2 << "SELECT * FROM BooleanSettings where name = \"Exposure Time(ms)\" LIMIT 1;";
            string sql2(ss2.str());
            if(sqlite3_prepare_v2(db, sql2.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_finalize(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_camera_exposure_t){
                    database_camera_exposure_t = sqlite3_column_int(stmt, 1);
                    ROS_INFO("camera_exposure changed");
                    roshandler->cameraExposureUpdate(database_camera_exposure_t);
                }
            }

            sqlite3_finalize(stmt);
            sqlite3_close(db);
            return true;
        }
};

int main(int argc, char **argv)
{
    ros::init( argc, argv, "qt_gui");
    ros::NodeHandlePtr n_;
    n_.reset(new ros::NodeHandle("~"));

    QApplication app( argc, argv );

    MainWindow* mainwindow = new MainWindow();

    mainwindow->setStyleSheet("background-color : #442e5d");
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
