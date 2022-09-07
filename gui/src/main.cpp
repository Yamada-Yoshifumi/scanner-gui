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
#include <QScrollBar>
#include <thread>
#include <chrono>
#include <ctime>

/*
This main script boots up the qt window, and calls a thread --PowerThread to take care of communication with the qml frontend via an sqlite3 database.

The PowerThread spins once every second, checking if any of the settings in the database have been updated by the qml frontend. Upon some change in settings, the thread calls methods
in the ROSHandler class which subsequently call some ROS services.

For some events and settings that require fast response speed and don't need to be stored locally, connections joining signals in MainWindow and methods in ROSHandler are declared. Those signals
in MainWindow are emitted when signals from qml are detected in MainWindow.
*/

class PowerThread: public QThread{
    ROSHandler* roshandler;
    MainWindow *mainwindow;
    public:
        sqlite3* db;
        sqlite3_stmt* stmt;
    private:
        QTimer *thread_timer;
        int current_velodyne_status = 0;
        int current_imu_status = 0;
        int current_camera_status = 0;
        int current_scan_status = 0;
        int database_camera_exposure_t = 20;
        int database_daylight_mode = 0;
        int database_debug_mode = 0;
        int database_reconstruction = 0;
        int database_colour_pattern = 0;
        int database_current_camera = 0;
        std::string debug_text = "";
        std::string previous_time_stamp = "0";
        std::string this_time_stamp = "0";
        QTextEdit* debug_text_box;
        QByteArray array = QCryptographicHash::hash("ScannerSettingsDB", QCryptographicHash::Md5);

    public:
        explicit PowerThread(MainWindow *mainwindow_)
            : QThread(), mainwindow(mainwindow_) {}

        void spinThreadOnce(){
            if (current_velodyne_status != mainwindow->velodyne_status){
                current_velodyne_status = mainwindow->velodyne_status;
                roshandler->velodyneCmd = (current_velodyne_status == 1) ? 0 : 1;
            }
            if (current_imu_status != mainwindow->imu_status){
                current_imu_status = mainwindow->imu_status;
                roshandler->imuCmd = (current_imu_status == 1) ? 0 : 1;
            }
            if (current_camera_status != mainwindow->camera_status){
                current_camera_status = mainwindow->camera_status;
                roshandler->cameraCmd = (current_camera_status == 1) ? 0 : 1;
            }
            if (current_scan_status != mainwindow->scan_status){
                current_scan_status = mainwindow->scan_status;
                roshandler->scanCmd = (current_scan_status == 1) ? 0 : 1;
            }
            changeInDatabaseResponse();
        }

        void run() override{

            thread_timer = new QTimer();
            thread_timer->start(1000);

            roshandler = new ROSHandler();

            bool opened = false;
            while(!opened){
                if(sqlite3_open_v2((mainwindow->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/" + array.toHex().toStdString()
                                  + ".sqlite").c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
                    printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
                    sqlite3_close(db);
                }
                else
                    opened = true;
            }

            connect(mainwindow, &MainWindow::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle, Qt::QueuedConnection);
            connect(mainwindow, &MainWindow::scanButtonPressed, roshandler, &ROSHandler::scanToggle, Qt::QueuedConnection);

            connect(thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
            connect(roshandler, &ROSHandler::hardwareOnSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::hardwareOffSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::cameraExposureUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::reconstructionUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::scanToggledSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            exec();
        }

        void updateDebugText(const QString &s){

            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            this_time_stamp = std::ctime(&end_time);
            debug_text.append(std::ctime(&end_time));
            debug_text.append(s.toStdString() + "\n");
        }

        bool changeInDatabaseResponse(){
            //Callback service for day/night light mode toggle
            stringstream ss1;
            ss1 << "SELECT * FROM BooleanSettings where name = \"Daylight Mode\" LIMIT 1;";
            string sql(ss1.str());
            if(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                if(sqlite3_open_v2((mainwindow->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/" + array.toHex().toStdString()
                                     + ".sqlite").c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
                    printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
                    sqlite3_reset(stmt);
                    sqlite3_close(db);
                }
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_daylight_mode){
                    if(sqlite3_column_int(stmt, 1) == 1){
                        mainwindow->setStyleSheet("background-color: white;");
                        mainwindow->myviz->logterminal->text_box->setStyleSheet("background-color: #ffffbd; color: black; font-size: 20px");
                        mainwindow->myviz->reset_button->setStyleSheet("background-color:#fafaa2; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                        mainwindow->myviz->fullscreen_button->setStyleSheet("background-color: #fafaa2;");
                        mainwindow->myviz->zoomin_button->setStyleSheet("background-color:#fafaa2; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                        mainwindow->myviz->zoomout_button->setStyleSheet("background-color:#fafaa2; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    }
                    else{
                        mainwindow->setStyleSheet("background-color: #442e5d;");
                        mainwindow->myviz->logterminal->text_box->setStyleSheet("background-color: black; color: white; font-size: 20px");
                        mainwindow->myviz->reset_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                        mainwindow->myviz->fullscreen_button->setStyleSheet("background-color: #442e5d;");
                        mainwindow->myviz->zoomin_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                        mainwindow->myviz->zoomout_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    }
                    database_daylight_mode = sqlite3_column_int(stmt, 1);
                }
            }

            //Callback service for camera_exposure time
            sqlite3_reset(stmt);
            stringstream ss2;
            ss2 << "SELECT * FROM BooleanSettings where name = \"Exposure Time(ms)\" LIMIT 1;";
            string sql2(ss2.str());
            if(sqlite3_prepare_v2(db, sql2.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_camera_exposure_t){
                    database_camera_exposure_t = sqlite3_column_int(stmt, 1);
                    ROS_INFO("camera_exposure changed");
                    roshandler->cameraExposureUpdate(database_camera_exposure_t);
                }
            }

            //Callback service for reconstruction
            sqlite3_reset(stmt);
            stringstream ss3;
            ss3 << "SELECT * FROM BooleanSettings where name = \"Reconstruction\" LIMIT 1;";
            string sql3(ss3.str());
            if(sqlite3_prepare_v2(db, sql3.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_reconstruction){
                    database_reconstruction = sqlite3_column_int(stmt, 1);
                    ROS_INFO("Reconstruction status changed");
                    roshandler->reconstructionUpdate(database_reconstruction);
                }
            }

            //Callback service for default lidar colour pattern
            sqlite3_reset(stmt);
            stringstream ss4;
            ss4 << "SELECT * FROM BooleanSettings where name = \"Default Colour\" LIMIT 1;";
            string sql4(ss4.str());
            if(sqlite3_prepare_v2(db, sql4.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_colour_pattern){
                    database_colour_pattern = sqlite3_column_int(stmt, 1);
                    ROS_INFO("Default colour pattern changed");
                    mainwindow->myviz->combo->setCurrentIndex(database_colour_pattern);
                }
            }

            //Callback service for default lidar colour pattern
            sqlite3_reset(stmt);
            stringstream ss5;
            ss5 << "SELECT * FROM BooleanSettings where name = \"Video Source\" LIMIT 1;";
            string sql5(ss5.str());
            if(sqlite3_prepare_v2(db, sql5.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_current_camera){
                    database_current_camera = sqlite3_column_int(stmt, 1);
                    ROS_INFO("camera changed");
                    mainwindow->switchVideoSource(database_current_camera);
                }
            }

            //Callback service for debug mode
            sqlite3_reset(stmt);
            stringstream ss_d;
            ss_d << "SELECT * FROM BooleanSettings where name = \"Debug Mode\" LIMIT 1;";
            string sql_d(ss_d.str());
            if(sqlite3_prepare_v2(db, sql_d.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_debug_mode){
                    database_debug_mode = sqlite3_column_int(stmt, 1);
                    ROS_INFO("debug mode toggled");
                }
            }

            if(database_debug_mode == 1){
                mainwindow->myviz->logterminal->setHidden(false);
                debug_text_box = mainwindow->myviz->logterminal->text_box;

                if(this_time_stamp != previous_time_stamp){
                        debug_text_box->moveCursor (QTextCursor::End);
                        debug_text_box->insertPlainText(QString(debug_text.c_str()));
                        debug_text_box->verticalScrollBar()->setValue(debug_text_box->verticalScrollBar()->maximum());
                        previous_time_stamp = this_time_stamp;
                        debug_text = "";

                }
            }
            else{
                debug_text_box = mainwindow->myviz->logterminal->text_box;
                mainwindow->myviz->logterminal->setHidden(true);
                debug_text_box->setPlainText(QString(""));
                debug_text = "";
            }
            sqlite3_reset(stmt);
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
    sqlite3_close(power_thread->db);
    sqlite3_finalize(power_thread->stmt);
    delete mainwindow;
    delete power_thread;
}
