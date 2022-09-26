#include "qapplication.h"
#include "roshandler.h"
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>

/* Send service calls to the backend
*/

using namespace std::chrono_literals;

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
    client_change_state_ = n_->create_client<lifecycle_msgs::srv::ChangeState>("lc_nodes/change_state");
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

void ROSHandler::closeWindow()
{
    if(!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        return;
    if(!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        return;
    if(!this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
        return;

    rclcpp::shutdown();
}

bool
ROSHandler::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
        RCLCPP_ERROR(
            n_->get_logger(),
            "Service %s is not available.",
            client_change_state_->get_service_name());
        return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
        RCLCPP_ERROR(
            n_->get_logger(), "Server time out while getting current state for node");
        return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
        RCLCPP_INFO(
            n_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
        return true;
    } else {
        RCLCPP_WARN(
            n_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
        return false;
    }
}
