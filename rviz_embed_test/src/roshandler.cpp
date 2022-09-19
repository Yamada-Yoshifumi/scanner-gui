#include "qapplication.h"
#include "roshandler.h"
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>

/* Send service calls to the backend
*/

ROSHandler::ROSHandler()

{
    n_ = rclcpp::Node::make_shared("ros_handler");
    ros_timer = new QTimer();
    ros_timer->start(100);

    velodyneSwitchClient = n_->create_client<ros_srv::srv::VelodyneSwitch>("/hardware_signal/velodyneSwitch");
    cameraExposureUpdateClient = n_->create_client<ros_srv::srv::CameraExposure>("/hardware_signal/cameraExposureUpdate");
    reconstructionUpdateClient = n_->create_client<ros_srv::srv::Reconstruction>("/hardware_signal/reconstructionUpdate");
    scanToggleClient = n_->create_client<ros_srv::srv::ScanToggle>("/hardware_signal/scanToggle");
    recordToggleClient = n_->create_client<ros_srv::srv::RecordToggle>("/hardware_signal/recordToggle");
    slamModeSwitchClient = n_->create_client<ros_srv::srv::SLAMModeSwitch>("/hardware_signal/slamModeSwitch");
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
}

void ROSHandler::spinOnce(){
    if(rclcpp::ok()){
        rclcpp::spin_some(n_);
    }
    else
        QApplication::quit();
}

void ROSHandler::systemPowerToggle(){
    if (velodyneCmd == 1 && imuCmd == 1 && cameraCmd == 1){
        velodyneOn();
    }
    else{
        velodyneOff();
    }
}

void ROSHandler::scanToggle(){
    auto request = std::make_shared<ros_srv::srv::ScanToggle::Request>();
    request->command = scanCmd;

    scanToggleClient->async_send_request(request);
    if(scanCmd == 1)
        emit scanToggledSignal("Scanning On");
    else if(scanCmd == 0)
        emit scanToggledSignal("Scanning Off");
}

void ROSHandler::recordToggle(){
    auto request = std::make_shared<ros_srv::srv::RecordToggle::Request>();
    request->command = recordCmd;

    recordToggleClient->async_send_request(request);
    if(recordCmd == 1)
        emit recordToggledSignal("Recording On");
    else if(recordCmd == 0)
        emit recordToggledSignal("Recording Off");
}

void ROSHandler::velodyneOn(){
    auto request = std::make_shared<ros_srv::srv::VelodyneSwitch::Request>();
    request->command = 1;

    velodyneSwitchClient->async_send_request(request);
    emit hardwareOnSignal("Hardware on requested");
}

void ROSHandler::velodyneOff(){
    auto request = std::make_shared<ros_srv::srv::VelodyneSwitch::Request>();
    request->command = 0;

    velodyneSwitchClient->async_send_request(request);
    emit hardwareOffSignal("Hardware off requested");
}
void ROSHandler::cameraExposureUpdate(int database_camera_exposure_t){
    auto request = std::make_shared<ros_srv::srv::CameraExposure::Request>();
    request->command = database_camera_exposure_t;
    cameraExposureUpdateClient->async_send_request(request);
    emit cameraExposureUpdatedSignal("Camera exposure time update requested");
    
}

void ROSHandler::reconstructionUpdate(int database_reconstruction){
    auto request = std::make_shared<ros_srv::srv::Reconstruction::Request>();
    request->command = database_reconstruction;
    reconstructionUpdateClient->async_send_request(request);
    emit reconstructionUpdatedSignal("Reconstruction status update requested");
}

void ROSHandler::slamModeUpdate(int database_current_slam_mode){
    auto request = std::make_shared<ros_srv::srv::SLAMModeSwitch::Request>();
    request->command = database_current_slam_mode;
    slamModeSwitchClient->async_send_request(request);
    emit slamModeSwitchedSignal("slam mode changed");
}

void ROSHandler::closeEvent(QCloseEvent * event)
{
    rclcpp::shutdown();
}
