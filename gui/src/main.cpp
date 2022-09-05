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
        int database_debug_mode = 0;
        int database_reconstruction = 0;
        int database_colour_pattern = 0;
        std::string debug_text = "";
        std::string previous_time_stamp = "0";
        std::string this_time_stamp = "0";
        QTextEdit* debug_text_box;

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
            thread_timer->start(1000);

            roshandler = new ROSHandler();

            connect(mainwindow, &MainWindow::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle, Qt::QueuedConnection);
            connect(mainwindow, &MainWindow::scanButtonPressed, roshandler, &ROSHandler::scanToggle, Qt::QueuedConnection);

            connect(thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
            connect(roshandler, &ROSHandler::hardwareOnSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::hardwareOffSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::cameraExposureUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
            connect(roshandler, &ROSHandler::reconstructionUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
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
            stringstream ss1;
            QByteArray array = QCryptographicHash::hash("ScannerSettingsDB", QCryptographicHash::Md5);
            if(sqlite3_open((mainwindow->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/" + array.toHex().toStdString()
                              + ".sqlite").c_str(), &db) != SQLITE_OK) {
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
                    mainwindow->myviz->logterminal->text_box->setStyleSheet("background-color: white; color: black; font-size: 20px");
                    mainwindow->myviz->reset_button->setStyleSheet("background-color: #fafaa2;");
                    mainwindow->myviz->fullscreen_button->setStyleSheet("background-color: #fafaa2;");
                    mainwindow->myviz->zoomin_button->setStyleSheet("background-color: #fafaa2;");
                    mainwindow->myviz->zoomout_button->setStyleSheet("background-color: #fafaa2;");
                }
                else{
                    mainwindow->setStyleSheet("background-color: #442e5d;");
                    mainwindow->myviz->logterminal->text_box->setStyleSheet("background-color: black; color: white; font-size: 20px");
                    mainwindow->myviz->reset_button->setStyleSheet("background-color: #442e5d;");
                    mainwindow->myviz->fullscreen_button->setStyleSheet("background-color: #442e5d;");
                    mainwindow->myviz->zoomin_button->setStyleSheet("background-color: #442e5d;");
                    mainwindow->myviz->zoomout_button->setStyleSheet("background-color: #442e5d;");
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

            //Callback service for reconstruction
            stringstream ss3;
            ss3 << "SELECT * FROM BooleanSettings where name = \"Reconstruction\" LIMIT 1;";
            string sql3(ss3.str());
            if(sqlite3_prepare_v2(db, sql3.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_finalize(stmt);
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
            stringstream ss4;
            ss4 << "SELECT * FROM BooleanSettings where name = \"Default Colour\" LIMIT 1;";
            string sql4(ss4.str());
            if(sqlite3_prepare_v2(db, sql4.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_finalize(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_colour_pattern){
                    database_colour_pattern = sqlite3_column_int(stmt, 1);
                    ROS_INFO("Default colour pattern changed");
                    mainwindow->myviz->combo->setCurrentIndex(database_colour_pattern);
                }
            }

            //Callback service for debug mode
            stringstream ss_d;
            ss_d << "SELECT * FROM BooleanSettings where name = \"Debug Mode\" LIMIT 1;";
            string sql_d(ss_d.str());
            if(sqlite3_prepare_v2(db, sql_d.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_finalize(stmt);
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
