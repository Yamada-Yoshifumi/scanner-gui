#include <ros/ros.h>
#include "qapplication.h"
#include "roshandler.h"
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>

/* Send service calls to the backend
*/

ROSHandler::ROSHandler()

{
    //velodyne_timer = new QTimer();
    //velodyne_timer->start(1000);
    ros_timer = new QTimer();
    ros_timer->start(100);
    n_.reset(new ros::NodeHandle("ros_handler"));

    velodyneSwitchClient = n_->serviceClient<ros_srv::VelodyneSwitch>("/hardware_signal/velodyneSwitch");
    cameraExposureUpdateClient = n_->serviceClient<ros_srv::CameraExposure>("/hardware_signal/cameraExposureUpdate");
    reconstructionUpdateClient = n_->serviceClient<ros_srv::Reconstruction>("/hardware_signal/reconstructionUpdate");
    scanToggleClient = n_->serviceClient<ros_srv::ScanToggle>("/hardware_signal/scanToggle");
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
}

void ROSHandler::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
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
    scanToggleSrv.request.command = scanCmd;

    scanToggleClient.call(scanToggleSrv);
    if(scanCmd == 1)
        emit scanToggledSignal("Scanning On");
    else if(scanCmd == 0)
        emit scanToggledSignal("Scanning Off");
}

void ROSHandler::velodyneOn(){
    velodynePowerSrv.request.command = 1;

    velodyneSwitchClient.call(velodynePowerSrv);
    emit hardwareOnSignal("Hardware on requested");
}

void ROSHandler::velodyneOff(){
    velodynePowerSrv.request.command = 0;

    velodyneSwitchClient.call(velodynePowerSrv);
    emit hardwareOffSignal("Hardware off requested");
}
void ROSHandler::cameraExposureUpdate(int database_camera_exposure_t){
    cameraExposureUpdateSrv.request.command = database_camera_exposure_t;
    if(cameraExposureUpdateClient.call(cameraExposureUpdateSrv)){
        ROS_INFO("camera update request received");
        emit cameraExposureUpdatedSignal("Camera exposure time update requested");
    }
}

void ROSHandler::reconstructionUpdate(int database_reconstruction){
    reconstructionUpdateSrv.request.command = database_reconstruction;
    reconstructionUpdateClient.call(reconstructionUpdateSrv);
    emit reconstructionUpdatedSignal("Reconstruction status update requested");
}
