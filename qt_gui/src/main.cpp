#include <QQmlApplicationEngine>
#include <QApplication>
#include <QQuickWidget>
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
#include <QTextEdit>
#include <sqlite3.h>
#include <memory>
#include <functional>
#include "std_msgs/msg/string.hpp"
using std::stringstream;
using std::string;
using namespace std::placeholders;
using namespace std::chrono_literals;
#include <memory>

#include "myviz/myviz.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

std::shared_ptr<MyViz> myviz;

class PowerThread: public QThread{
    ROSHandler* roshandler;
    std::shared_ptr<MyViz> myviz;
public:
    sqlite3* db;
    sqlite3_stmt* stmt;
private:
    QTimer *thread_timer;
    int current_velodyne_status = 0;
    int current_imu_status = 0;
    int current_camera_status = 0;
    int current_scan_status = 0;
    int current_record_status = 0;
    int database_camera_exposure_t = 20;
    int database_daylight_mode = 0;
    int database_debug_mode = 0;
    int database_reconstruction = 0;
    int database_colour_pattern = 0;
    int database_current_camera = 0;
    int database_current_slam_mode = 1;
    std::string debug_text = "";
    std::string previous_time_stamp = "0";
    std::string this_time_stamp = "0";
    QTextEdit* debug_text_box;
    QByteArray array = QCryptographicHash::hash("ScannerSettingsDB", QCryptographicHash::Md5);
    QObject *power_button;
    QObject *record_button;
    QObject *scan_button;

signals:
    void powerSignal(QString);
    void scanSignal(QString);
    void recordSignal(QString);

/*
    void systemPowerToggle(){
        roshandler->systemPowerToggle();
    };
    void scanToggle(){
        roshandler->scanToggle();
    };
    void recordToggle(){
        roshandler->recordToggle();
    };
    */
public:
    explicit PowerThread(std::shared_ptr<MyViz> myviz_)
        : QThread(), myviz(myviz_) {}

    void spinThreadOnce(){
        if (current_velodyne_status != myviz->velodyne_status){
            current_velodyne_status = myviz->velodyne_status;
            roshandler->velodyneCmd = (current_velodyne_status == 1) ? 0 : 1;
        }
        if (current_imu_status != myviz->imu_status){
            current_imu_status = myviz->imu_status;
            roshandler->imuCmd = (current_imu_status == 1) ? 0 : 1;
        }
        if (current_camera_status != myviz->camera_status){
            current_camera_status = myviz->camera_status;
            roshandler->cameraCmd = (current_camera_status == 1) ? 0 : 1;
        }
        if (current_scan_status != myviz->scan_status){
            current_scan_status = myviz->scan_status;
            roshandler->scanCmd = (current_scan_status == 1) ? 0 : 1;
        }
        if (current_record_status != myviz->record_status){
            current_record_status = myviz->record_status;
            roshandler->recordCmd = (current_record_status == 1) ? 0 : 1;
        }
        changeInDatabaseResponse();
    }

    void run() override{

        thread_timer = new QTimer();
        thread_timer->start(1000);

        roshandler = new ROSHandler();

        bool opened = false;
        while(!opened){
            if(sqlite3_open_v2((myviz->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/" + array.toHex().toStdString()
                                 + ".sqlite").c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
                printf("ERROR: can't open database: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
            }
            else
                opened = true;
        }
/*
        connect(myviz, &MyViz::powerButtonPressed, roshandler, &ROSHandler::systemPowerToggle, Qt::QueuedConnection);
        connect(myviz, &MyViz::scanButtonPressed, roshandler, &ROSHandler::scanToggle, Qt::QueuedConnection);
        connect(myviz, &MyViz::recordButtonPressed, roshandler, &ROSHandler::recordToggle, Qt::QueuedConnection);
*/
        connect(thread_timer, &QTimer::timeout, this, &PowerThread::spinThreadOnce);
        connect(roshandler, &ROSHandler::hardwareOnSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::hardwareOffSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::cameraExposureUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::reconstructionUpdatedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::scanToggledSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::recordToggledSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);
        connect(roshandler, &ROSHandler::slamModeSwitchedSignal, this, &PowerThread::updateDebugText, Qt::QueuedConnection);

        QObject *settingsItem = myviz->settingsqmlView->rootObject();
        QObject *item = myviz->qmlView->rootObject();
        //while(power_button==nullptr)
            power_button = power_button = item->findChild<QObject*>("power_button");
        //while(scan_button == nullptr)
            scan_button = settingsItem->findChild<QObject*>("scan_button");
        //while(record_button == nullptr)
            record_button = settingsItem->findChild<QObject*>("record_button");

        connect(power_button, SIGNAL(powerSignal(QString)), roshandler, SLOT(systemPowerToggle()), Qt::QueuedConnection);
        connect(scan_button, SIGNAL(scanSignal(QString)), roshandler, SLOT(scanToggle()), Qt::QueuedConnection);
        connect(record_button, SIGNAL(recordSignal(QString)), roshandler, SLOT(recordToggle()), Qt::QueuedConnection);

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
        std::stringstream ss1;
        ss1 << "SELECT * FROM BooleanSettings where name = \"Daylight Mode\" LIMIT 1;";
        std::string sql(ss1.str());
        if(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            if(sqlite3_open_v2((myviz->settingsqmlView->engine()->offlineStoragePath().toStdString() + "/Databases/" + array.toHex().toStdString()
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
                    myviz->setStyleSheet("background-color: white;");
                    myviz->logterminal->text_box->setStyleSheet("background-color: #e1f2f7; color: black; font-size: 20px");
                    myviz->reset_button->setStyleSheet("background-color:#b5cef7; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    myviz->fullscreen_button->setStyleSheet("background-color: #b5cef7;");
                    myviz->zoomin_button->setStyleSheet("background-color:#b5cef7; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    myviz->zoomout_button->setStyleSheet("background-color:#b5cef7; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                }
                else{
                    myviz->setStyleSheet("background-color: #442e5d;");
                    myviz->logterminal->text_box->setStyleSheet("background-color: black; color: white; font-size: 20px");
                    myviz->reset_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    myviz->fullscreen_button->setStyleSheet("background-color: #442e5d;");
                    myviz->zoomin_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                    myviz->zoomout_button->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
                }
                database_daylight_mode = sqlite3_column_int(stmt, 1);
            }
        }

        //Callback service for camera_exposure time
        sqlite3_reset(stmt);
        std::stringstream ss2;
        ss2 << "SELECT * FROM BooleanSettings where name = \"Exposure Time(ms)\" LIMIT 1;";
        std::string sql2(ss2.str());
        if(sqlite3_prepare_v2(db, sql2.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            printf("exposure time issue");
            sqlite3_close(db);
            sqlite3_reset(stmt);
            return false;
        }

        if((sqlite3_step(stmt)) == SQLITE_ROW) {
            if(sqlite3_column_int(stmt, 1) != database_camera_exposure_t){
                database_camera_exposure_t = sqlite3_column_int(stmt, 1);
                printf("camera_exposure changed");
                roshandler->cameraExposureUpdate(database_camera_exposure_t);
            }
        }

        //Callback service for reconstruction
        sqlite3_reset(stmt);
        std::stringstream ss3;
        ss3 << "SELECT * FROM BooleanSettings where name = \"Reconstruction\" LIMIT 1;";
        std::string sql3(ss3.str());
        if(sqlite3_prepare_v2(db, sql3.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            sqlite3_reset(stmt);
            return false;
        }

        if((sqlite3_step(stmt)) == SQLITE_ROW) {
            if(sqlite3_column_int(stmt, 1) != database_reconstruction){
                database_reconstruction = sqlite3_column_int(stmt, 1);
                printf("Reconstruction status changed");
                roshandler->reconstructionUpdate(database_reconstruction);
            }
        }

        //Callback service for default lidar colour pattern
        sqlite3_reset(stmt);
        std::stringstream ss4;
        ss4 << "SELECT * FROM BooleanSettings where name = \"Default Colour\" LIMIT 1;";
        std::string sql4(ss4.str());
        if(sqlite3_prepare_v2(db, sql4.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            sqlite3_reset(stmt);
            return false;
        }

        if((sqlite3_step(stmt)) == SQLITE_ROW) {
            if(sqlite3_column_int(stmt, 1) != database_colour_pattern){
                database_colour_pattern = sqlite3_column_int(stmt, 1);
                printf("Default colour pattern changed");
                myviz->combo->setCurrentIndex(database_colour_pattern);
            }
        }

        //Callback service for default lidar colour pattern
        sqlite3_reset(stmt);
        std::stringstream ss5;
        ss5 << "SELECT * FROM BooleanSettings where name = \"Video Source\" LIMIT 1;";
        std::string sql5(ss5.str());

        if(sqlite3_prepare_v2(db, sql5.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            sqlite3_close(db);
            sqlite3_reset(stmt);
            return false;
        }
    

        if((sqlite3_step(stmt)) == SQLITE_ROW) {
            if(sqlite3_column_int(stmt, 1) != database_current_camera){
                database_current_camera = sqlite3_column_int(stmt, 1);
                printf("camera changed");
                myviz->switchVideoSource(database_current_camera);
            }
        }

        //Callback service for SLAM Mode
        sqlite3_reset(stmt);
        std::stringstream ss6;
        ss6 << "SELECT * FROM BooleanSettings where name = \"SLAM Options\" LIMIT 1;";
        std::string sql6(ss6.str());

            if(sqlite3_prepare_v2(db, sql6.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
                printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db);
                sqlite3_reset(stmt);
                return false;
            }

            if((sqlite3_step(stmt)) == SQLITE_ROW) {
                if(sqlite3_column_int(stmt, 1) != database_current_slam_mode - 1){
                    database_current_slam_mode = sqlite3_column_int(stmt, 1) + 1;
                    printf("slam changed");
                    roshandler->slamModeUpdate(database_current_slam_mode);
                }
            }

        //Callback service for debug mode
        sqlite3_reset(stmt);
        std::stringstream ss_d;
        ss_d << "SELECT * FROM BooleanSettings where name = \"Debug Mode\" LIMIT 1;";
        std::string sql_d(ss_d.str());
        if(sqlite3_prepare_v2(db, sql_d.c_str(), -1, &stmt, NULL) != SQLITE_OK) {
            printf("ERROR: while compiling sql: %s\n", sqlite3_errmsg(db));
            printf("debug mode issue");
            sqlite3_close(db);
            sqlite3_reset(stmt);
            return false;
        }

        if((sqlite3_step(stmt)) == SQLITE_ROW) {
            if(sqlite3_column_int(stmt, 1) != database_debug_mode){
                database_debug_mode = sqlite3_column_int(stmt, 1);
                printf("debug mode toggled");
            }
        }

        if(database_debug_mode == 1){
            myviz->logterminal->setHidden(false);
            debug_text_box = myviz->logterminal->text_box;

            if(this_time_stamp != previous_time_stamp){
                debug_text_box->moveCursor (QTextCursor::End);
                debug_text_box->insertPlainText(QString(debug_text.c_str()));
                debug_text_box->verticalScrollBar()->setValue(debug_text_box->verticalScrollBar()->maximum());
                previous_time_stamp = this_time_stamp;
                debug_text = "";

            }
        }
        else{
            debug_text_box = myviz->logterminal->text_box;
            myviz->logterminal->setHidden(true);
            debug_text_box->setPlainText(QString(""));
            debug_text = "";
        }
        sqlite3_reset(stmt);
        return true;
    }
};

class ROSMessageDetector : public rclcpp::Node
{
    //std::shared_ptr<MyViz> myviz;

public:
    ROSMessageDetector()
        :Node("message_detector")
    {
        rclcpp::QoS video_qos(30);
        video_qos.keep_last(30);
        video_qos.best_effort();
        video_qos.durability_volatile();
        velodynesub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", video_qos, std::bind(&ROSMessageDetector::updateVelodyneStatus, this, _1));
        imusub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", video_qos, std::bind(&ROSMessageDetector::updateImuStatus, this, _1));
        camerasub = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", video_qos, std::bind(&ROSMessageDetector::updateCameraStatus, this, _1));
            /*
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ROSMessageDetector::timer_callback, this));*/
    }

private:

    //std::string camera_topic = "/rrbot/camera1/image_raw";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodynesub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imusub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camerasub;

    void updateVelodyneStatus(const sensor_msgs::msg::PointCloud2 &msg) const 
    {

        myviz->velodyne_timer->start(1000);
        myviz->velodyne_status = 1;
        myviz->paintStatus();
    }

    void updateImuStatus(const nav_msgs::msg::Odometry &msg) const
    {

        myviz->imu_timer->start(1000);
        myviz->imu_status = 1;
        myviz->paintStatus();
    }

    void updateCameraStatus(const sensor_msgs::msg::Image &msg) const
    {
        
        myviz->videoStreamer->convertROSImage(msg);
        myviz->camera_timer->start(1000);
        myviz->camera_status = 1;

        myviz->paintStatus();
    }
    /*
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;*/

};

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);

  auto ros_node_abs =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");

  myviz = std::make_shared<MyViz>(&app, ros_node_abs);
  myviz->show();

  std::shared_ptr<ROSMessageDetector> ros_message_detector = std::make_shared<ROSMessageDetector>();


  PowerThread* power_thread = new PowerThread(myviz);
  power_thread->start();

  rclcpp::Rate rate(60);
  while (rclcpp::ok()) {
    app.processEvents();
    rclcpp::spin_some(ros_message_detector);
    rate.sleep();
  }
  sqlite3_close(power_thread->db);
  sqlite3_finalize(power_thread->stmt);
  system("rosnode kill gazebo");
  system("rosnode kill usb_cam");
  system("killall -9 gzserver");
  return 0;
}
