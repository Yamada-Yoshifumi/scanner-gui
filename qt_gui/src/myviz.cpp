#include "myviz/myviz.hpp"

#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <rclcpp/qos.hpp>

#include "rclcpp/clock.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/view_manager.hpp"
using std::placeholders::_1;


MyViz::MyViz(
  QApplication * app,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  QWidget * parent)
: app_(app), rviz_ros_node_(rviz_ros_node), QMainWindow(parent)
{

    central_widget = new QWidget(this);
    secondary_widget = new QWidget(this);
    secondary_widget->setMinimumSize(1280, 480);
    //tertiary_widget = new QWidget(central_widget);
    QSizePolicy sp_retain = secondary_widget->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    secondary_widget->setSizePolicy(sp_retain);
    // QML Wrapper

    qDebug() << "10";
    qmlView = new QQuickView();

    qmlView->setSource(QUrl(QStringLiteral("qrc:/qml/App.qml")));

    container = QWidget::createWindowContainer(qmlView, central_widget);
    container->setMinimumSize(1280, 240);
    container->adjustSize();
    qDebug() << "9";
    //Other stuff
    countdown_widget = new QLabel(secondary_widget);
    countdown_widget -> setStyleSheet("background-color: rgba(10,10,10,1);color : white;font-weight: bold; font-size: 200px; qproperty-alignment: AlignCenter;");
    countdown_widget -> setNum(3);
    countdown_widget -> setVisible(false);


    qDebug() << "8";

    //timers
    QObject *item = qmlView->rootObject();
    QObject *reboot_button = item->findChild<QObject*>("continue");
    power_button = item->findChild<QObject*>("power_button");
    power_button_bg = item->findChild<QObject*>("power_button_bg");
    scan_button = item->findChild<QObject*>("scan_button");
    record_button = item->findChild<QObject*>("record_button");
    velodyne_timer = new QTimer();
    velodyne_timer->start(1000);
    imu_timer = new QTimer();
    imu_timer->start(1000);
    camera_timer = new QTimer();
    camera_timer->start(1000);
    ros_timer = new QTimer();
    ros_timer->start(50);

  // Add visualization

  // Initialize the classes we need from rviz
  //secondary_widget = new QWidget(this);
  initializeRViz();
  DisplayGrid();
  qDebug() << "2";

  logterminal = new LogTerminal(render_panel_);
  sp_retain = logterminal->sizePolicy();
  sp_retain.setRetainSizeWhenHidden(true);
  logterminal->setSizePolicy(sp_retain);
  logterminal->setHidden(true);
  touchpad = new TouchPad(render_panel_);
  fullscreen_button = new QPushButton(secondary_widget);
  zoomin_button = new RoundedButton(secondary_widget);
  zoomout_button = new RoundedButton(secondary_widget);
  reset_button = new RoundedButton(secondary_widget);
  combo = new QComboBox(secondary_widget);

  touchpad->setStyleSheet("background-color: rgba(10,10,10,0.8);");

  fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
  fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
  fullscreen_button->setStyleSheet("background-color:#442e5d;");
  //fullscreen_button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
  fullscreen_button->setIconSize(QSize(32,32));
  qDebug() << "3";

  zoomin_button->setObjectName(QStringLiteral("zoomin_button"));
  zoomin_button->setIcon(QIcon(":/qml/images/zoom_in.png"));
  zoomin_button->setIconSize(QSize(64,64));
  //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>


  zoomout_button->setObjectName(QStringLiteral("zoomout_button"));
  zoomout_button->setIcon(QIcon(":/qml/images/zoom_out.png"));
  zoomout_button->setIconSize(QSize(64,64));
  //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>
  qDebug() << "2";

  reset_button->setObjectName(QStringLiteral("reset_button"));
  reset_button->setIcon(QIcon(":/qml/images/reset_rviz.png"));
  reset_button->setIconSize(QSize(64,64));
  //<a href="https://www.flaticon.com/free-icons/axis" title="axis icons">Axis icons created by Smashicons - Flaticon</a>

  QStringList commands = { "Intensity", "AxisColor", "Uncertainty", "FlatColor" };

  combo->addItems(commands);
  combo->setStyleSheet("font-size: 30px;selection-background-color: #111;selection-color: yellow;color: white;background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #565656, stop: 0.1 #525252, stop: 0.5 #4e4e4e, stop: 0.9 #4a4a4a, stop: 1 #464646);");
  //combo->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));

  secondary_layout = new QGridLayout(secondary_widget);
  secondary_layout->setContentsMargins(0,0,0,0);
  secondary_layout->addWidget(touchpad, 0, 30, 8, 15);
  secondary_layout->addWidget(logterminal, 0, 30, 8, 15);
  secondary_layout->addWidget(zoomin_button, 8, 30, 2, 5);
  secondary_layout->addWidget(zoomout_button, 8, 35, 2, 5);
  secondary_layout->addWidget(reset_button, 8, 40, 2, 5);
  secondary_layout->addWidget(render_panel_, 0, 0, 10, 30 );
  //secondary_layout->addWidget(fullscreen_button, 9, 30, 1, 1 );
  //secondary_layout->addWidget(combo, 0, 30, 1, 5);
  secondary_widget->setLayout(secondary_layout);



  central_widget_layout = new AnimatedGridLayout();
  central_widget_layout->setSpacing(0);
  central_widget_layout->setContentsMargins(0, 0, 50, 0);
  central_widget_layout->addWidget(container, 2, 0, 1, 1);
  central_widget_layout->addWidget(secondary_widget, 0, 0, 2, 1);
  central_widget->setLayout(central_widget_layout);
  setCentralWidget(central_widget);

  qDebug() << "1";

  // Signals
  /*

  // Intialize the sliders


  connect(imu_timer, SIGNAL(timeout()), this, SLOT(resetImuStatus()));
  connect(camera_timer, SIGNAL(timeout()), this, SLOT(resetCameraStatus()));
  connect(velodyne_timer, SIGNAL(timeout()), this, SLOT(resetVelodyneStatus()));

  */  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  connect( fullscreen_button, &QPushButton::clicked, this, &MyViz::fullscreenToggle);


  settingsqmlView = new QQuickView();
  settingsqmlView->setSource(QUrl(QStringLiteral("qrc:/qml/Settings.qml")));
  auto offlineStoragePath = QUrl::fromLocalFile(settingsqmlView->engine()->offlineStoragePath());
  settingsqmlView->engine()->rootContext()->setContextProperty("offlineStoragePath", offlineStoragePath);
  settings_container = QWidget::createWindowContainer(settingsqmlView, central_widget);
  settings_container->setMinimumSize(480, 720);
  settings_container->adjustSize();
  settings_container->move(QPoint(this->width()*3/2, 0));
  settings_container->raise();
  QObject *settingsItem = settingsqmlView->rootObject();

  settings_toggle_button = settingsItem->findChild<QObject*>("settings_toggle_button");

  connect(settings_toggle_button, SIGNAL(settingsToggle(QString)), this, SLOT(toggleSettings()));

  connect(settings_toggle_button, SIGNAL(exit(QString)), this, SLOT(closeWindow()));
  /*
    */

  //while(scan_button == nullptr)
      scan_button = settingsItem->findChild<QObject*>("scan_button");
  //while(record_button == nullptr)
      record_button = settingsItem->findChild<QObject*>("record_button");
  //while(power_button == nullptr)
        power_button = item->findChild<QObject*>("power_button");

  //while(opencv_image == nullptr)
        opencv_image = item->findChild<QObject*>("opencv_image");

  connect(
      scan_button,
      SIGNAL(scanSignal(QString)),
      this,
      SLOT(scanClickedEmit()));

  connect(
      record_button,
      SIGNAL(recordSignal(QString)),
      this,
      SLOT(recordClickedEmit()));
  videoStreamer = new VideoStreamer();
  liveimageprovider = new OpencvImageProvider();

    connect(
        power_button,
        SIGNAL(powerSignal(QString)),
        this,
        SLOT(powerClickedEmit()));
    connect(videoStreamer,&VideoStreamer::newImage,liveimageprovider,&OpencvImageProvider::updateImage);
    connect(liveimageprovider,
            SIGNAL(imageChanged()),
            this,
            SLOT(imageReload()));
    videoStreamer->openVideoCamera();
    qmlView->engine()->addImageProvider("live",liveimageprovider);
    counter = false;

    zoomin_button->setAutoRepeat(true);
    zoomout_button->setAutoRepeat(true);

    connect(reset_button, &QPushButton::clicked, this, &MyViz::resetView);
    connect(zoomin_button, &QPushButton::pressed, this, &MyViz::manualZoomIn);
    connect(zoomout_button, &QPushButton::pressed, this, &MyViz::manualZoomOut);

  QTimer::singleShot(0, this, SLOT(showFullScreen()));
}
/*
MyViz::~MyViz(){

}
*/
QWidget *
MyViz::getParentWindow()
{
  return this;
}

rviz_common::PanelDockWidget *
MyViz::addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating)
{
  // TODO(mjeronimo)
  return nullptr;
}

void
MyViz::setStatus(const QString & message)
{
  // TODO(mjeronimo)
}

void MyViz::DisplayGrid()
{
  grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
  assert(grid_ != NULL);
  grid_->subProp("Line Style")->setValue("Billboards");
  grid_->subProp("Color")->setValue(QColor(Qt::white));
}

void MyViz::initializeRViz()
{
  app_->processEvents();
  render_panel_ = new rviz_common::RenderPanel(secondary_widget);
  app_->processEvents();
  render_panel_->getRenderWindow()->initialize();
  auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node_, this, clock);
  render_panel_->initialize(manager_);
  app_->processEvents();
  manager_->initialize();
  manager_->startUpdate();
  //render_panel_->setHidden(true);
}

void MyViz::setThickness(int thickness_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Line Style")->subProp("Line Width")->setValue(thickness_percent / 100.0f);
  }
}

void MyViz::setCellSize(int cell_size_percent)
{
  if (grid_ != NULL) {
    grid_->subProp("Cell Size")->setValue(cell_size_percent / 10.0f);
  }
}

void MyViz::closeEvent(QCloseEvent * event)
{
  rclcpp::shutdown();
}

void MyViz::closeWindow(){
    close();
}

void MyViz::spinOnce(){
    //this->changeInDatabaseResponse();
    if(rclcpp::ok()){
        //rclcpp::spin_some(ros_message_detector);
    }
    else
        QApplication::quit();
}

void MyViz::resizeEvent(QResizeEvent* event)
{
    QQuickItem* rootObject =  qmlView->rootObject();
    newSize = event->size();
    if(rootObject) rootObject->setProperty("width",QVariant::fromValue(newSize.width() - 50));
    if(rootObject) rootObject->setProperty("height",QVariant::fromValue(newSize.height()/3));

    rootObject =  settingsqmlView->rootObject();
    if(rootObject) rootObject->setProperty("height",QVariant::fromValue(newSize.height()));

    //secondary_widget->resize(newSize.width() - 60, newSize.height()*2/3 - 10);
    //secondary_widget->raise();
    if(!settings_shown){
        settings_container->resize(settings_container->width(), newSize.height());
        settings_container->move(QPoint(newSize.width() - 50, 0));
    }
    else{
        settings_container->resize(settings_container->width(), newSize.height());
        settings_container->move(QPoint(newSize.width() - settings_container->width(), 0));
    }
}

void MyViz::switchVideoSource(int source){
    if(source == 0){
        std::string camera_stream;
        //ros_message_detector->camera_topic = "/rrbot/camera1/image_raw";
        //camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);
    }
    else if(source == 1){
        std::string camera_stream;
        //ros_message_detector->camera_topic = "/rrbot/camera2/image_raw";
        //camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);
    }
}
/*
void MyViz::createRVizEvent()
{
    logterminal->setHidden(true);
    QObject *item = qmlView->rootObject();
    while(power_button == nullptr)
        power_button = item->findChild<QObject*>("power_button");

    while(opencv_image == nullptr)
        opencv_image = item->findChild<QObject*>("opencv_image");

    //videoStreamer = new VideoStreamer();

    //liveimageprovider = new OpencvImageProvider();

    connect(
        power_button,
        SIGNAL(powerSignal(QString)),
        this,
        SLOT(powerClickedEmit()));
    connect(videoStreamer,&VideoStreamer::newImage,liveimageprovider,&OpencvImageProvider::updateImage);
    connect(liveimageprovider,
            SIGNAL(imageChanged()),
            this,
            SLOT(imageReload()));
    videoStreamer->openVideoCamera();
    qmlView->engine()->addImageProvider("live",liveimageprovider);
    counter = false;

}
*/
void MyViz::powerClickedEmit(){

    emit powerButtonPressed();
}

void MyViz::scanClickedEmit(){
    if (!scan_button->property("scanning").toBool())
    {
        countdown_widget -> setVisible(true);
        central_widget_layout->addWidget(countdown_widget, 0, 0, 3, 1);
        emit scanButtonPressed();
        render_panel_->setHidden(true);
        container->setHidden(true);
        scan_countdown_timer = new QTimer(this);
        scan_countdown_timer->start(1000);
        connect(scan_countdown_timer, SIGNAL(timeout()), this, SLOT(updateCountDownNum()));
        scan_button->setProperty("scanning", true);
    }
    else{
        emit scanButtonPressed();
        scan_button->setProperty("scanning", false);
    }
    scan_status = scan_status == 0 ? 1: 0;
}

void MyViz::recordClickedEmit(){
    if (!record_button->property("recording").toBool())
    {
        emit recordButtonPressed();
        record_button->setProperty("recording", true);
    }
    else{
        emit recordButtonPressed();
        record_button->setProperty("recording", false);
    }
    record_status = record_status == 0 ? 1: 0;
}

void MyViz::updateCountDownNum(){
    int current_number = countdown_widget->text().toInt();
    if(current_number == 0){
        central_widget_layout->removeWidget(countdown_widget);
        render_panel_->setHidden(false);
        container->setHidden(false);
        countdown_widget ->setVisible(false);
        countdown_widget ->setNum(3);
        scan_countdown_timer -> stop();
        //fullscreen_button->click();
        scan_button->setProperty("scanning", true);
    }
    else{
        current_number -= 1;
        countdown_widget->clear();
        countdown_widget->setNum(current_number);
    }
}

void MyViz::toggleSettings(){

    if (settings_shown){
        //central_widget_layout->setContentsMargins(0, 0, 50, 0);
        rviz_animation = new QPropertyAnimation(central_widget_layout, "intContentsMargins");

        rviz_animation->setDuration(50);

        int temp =central_widget_layout->readCM();
        rviz_animation->setStartValue(QVariant(temp));

        temp = 50;
        rviz_animation->setEndValue(QVariant(temp));
        rviz_animation->start();

        QQuickItem* rootObject =  qmlView->rootObject();
        if(rootObject) rootObject->setProperty("width",QVariant::fromValue(newSize.width()));

        settings_animation = new QPropertyAnimation(settings_container, "pos");
        settings_animation->setDuration(200);
        settings_animation->setStartValue(settings_container->pos());
        settings_animation->setEndValue(QPoint(newSize.width() - 50, 0));
        settings_animation->start();
        settings_shown = false;
        /*         settings_container->move(QPoint(newSize.width() - 50, 0));
        settings_container->adjustSize();
        settings_shown = false;
*/
    }
    else{
        QQuickItem* rootObject =  qmlView->rootObject();
        if(rootObject) rootObject->setProperty("width",QVariant::fromValue(newSize.width() - settings_container->width()));

        settings_animation = new QPropertyAnimation(settings_container, "pos");
        settings_animation->setDuration(200);
        settings_animation->setStartValue(settings_container->pos());
        settings_animation->setEndValue(QPoint(newSize.width() - settings_container->width(), 0));
        settings_animation->start();
        settings_shown = true;

        rviz_animation = new QPropertyAnimation(central_widget_layout, "intContentsMargins");

        rviz_animation->setDuration(50);

        int temp =central_widget_layout->readCM();
        rviz_animation->setStartValue(QVariant(temp));

        temp = settings_container->width();
        rviz_animation->setEndValue(QVariant(temp));
        rviz_animation->start();
        /*
        settings_container->move(QPoint(newSize.width() - settings_container->width(), 0));
        settings_container->adjustSize();
        settings_shown = true;
*/
    }
}

/*
void MyViz::updateVelodyneStatus(const sensor_msgs::msg::PointCloud2::SharedPtr &msg){
    velodyne_timer->start(1000);
    velodyne_status = 1;
    paintStatus();
}

void MyViz::updateImuStatus(const nav_msgs::msg::Odometry::SharedPtr &msg){
    imu_timer->start(1000);
    imu_status = 1;
    //paintStatus();
}

void MyViz::updateCameraStatus(const sensor_msgs::msg::Image::SharedPtr &msg){
    videoStreamer->convertROSImage(msg);
    camera_timer->start(1000);
    camera_status = 1;

    //paintStatus();
}
*/

    void MyViz::resetVelodyneStatus(){
    velodyne_status = 0;
    if(power_button_bg != nullptr)
        paintStatus();
}

void MyViz::resetImuStatus(){
    imu_status = 0;
    //if(power_button_bg != nullptr)
    //    paintStatus();
}

void MyViz::resetCameraStatus(){
    camera_status = 0;
    //if(power_button_bg != nullptr)
    //    paintStatus();
}

void MyViz::imageReload(){
    QObject *item = qmlView->rootObject();
    opencv_image = item->findChild<QObject*>("opencv_image");
    if(videoStreamer->init && videoStreamer->current_frame_ptr != nullptr)
    {
        //ROS_INFO("1");
        if(counter){
            //ROS_INFO("2");
            opencv_image->setProperty("source", "image://live/image?id=1");
        }
        else
        {
            //ROS_INFO("2");
            opencv_image->setProperty("source", "image://live/image?id=0");
        }
        counter = !counter;
    }
}

void MyViz::paintStatus(){
    QObject *item = qmlView->rootObject();
    power_button_bg = item->findChild<QObject*>("power_button_bg");
    lidar_canvas = item->findChild<QObject*>("lidar_status");
    imu_canvas = item->findChild<QObject*>("imu_status");
    camera_canvas = item->findChild<QObject*>("camera_status");
    lidar_status_text = item->findChild<QObject*>("lidar_status_text");
    imu_status_text = item->findChild<QObject*>("imu_status_text");
    camera_status_text = item->findChild<QObject*>("camera_status_text");
    lidar_status_pic = item->findChild<QObject*>("lidar_status_overlay");
    imu_status_pic = item->findChild<QObject*>("gyro_status_overlay");
    camera_status_pic = item->findChild<QObject*>("camera_status_overlay");
    if(velodyne_status == 1 && imu_status == 1 && camera_status == 1){
        QColor color(Qt::white);
        power_button_bg -> setProperty("color", color);
    }
    else if(velodyne_status == 1 || imu_status == 1 || camera_status == 1)
    {
        QColor color(Qt::yellow);
        power_button_bg -> setProperty("color", color);
    }
    else{
        QColor color("#343434");
        power_button_bg -> setProperty("color", color);
    }
    if(velodyne_status == 1){
        lidar_canvas -> setProperty("colour", "green");
        lidar_status_text -> setProperty("color", "green");
        lidar_status_pic -> setProperty("color", "green");
    }
    else{
        lidar_canvas -> setProperty("colour", "red");
        lidar_status_text -> setProperty("color", "red");
        lidar_status_pic -> setProperty("color", "#00000000");
    }
    if(imu_status == 1){
        imu_canvas -> setProperty("colour", "green");
        imu_status_text -> setProperty("color", "green");
        imu_status_pic -> setProperty("color", "green");
    }
    else{
        imu_canvas -> setProperty("colour", "red");
        imu_status_text -> setProperty("color", "red");
        imu_status_pic -> setProperty("color", "#00000000");
    }
    if(camera_status == 1){
        camera_canvas -> setProperty("colour", "green");
        camera_status_text -> setProperty("color", "green");
        camera_status_pic -> setProperty("color", "green");
    }
    else{
        camera_canvas -> setProperty("colour", "red");
        camera_status_text -> setProperty("color", "red");
        camera_status_pic -> setProperty("color", "#00000000");
    }
}

void MyViz::fullscreenToggle()
{
    if (fullscreen_button->objectName() == (QStringLiteral("fullscreen_button")))
    {
        central_widget_layout->removeWidget(render_panel_);
        central_widget_layout->addWidget(render_panel_, 0, 0, 3, 3);
        fullscreen_button->setObjectName(QStringLiteral("exit_fullscreen_button"));
        fullscreen_button->setIcon(QIcon(":/qml/images/exit_fullscreen.png"));
    }
    else if(fullscreen_button->objectName() == (QStringLiteral("exit_fullscreen_button")))
    {
        central_widget_layout->removeWidget(render_panel_);
        central_widget_layout->addWidget(render_panel_, 0, 0, 2, 3);
        fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
        fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
    }
}

bool MyViz::eventFilter(QObject * p_obj, QEvent * p_event)
{
    /*
    if (p_event->type() == QEvent::MouseButtonDblClick ||
        p_event->type() == QEvent::GraphicsSceneMouseMove ||
        p_event->type() == QEvent::GraphicsSceneMouseDoubleClick ||
        p_event->type() == QEvent::GraphicsSceneMousePress ||
        p_event->type() == QEvent::GraphicsSceneMouseRelease ||
        p_event->type() == QEvent::NonClientAreaMouseButtonPress ||
        p_event->type() == QEvent::NonClientAreaMouseButtonRelease ||
        p_event->type() == QEvent::Wheel)
    {
        ROS_INFO("ignored something");
        QMouseEvent* pMouseEvent = dynamic_cast<QMouseEvent*>(p_event);
    }*/

    if(p_event->type() == QEvent::MouseMove){
        QMouseEvent* pMouseEvent = dynamic_cast<QMouseEvent*>(p_event);
        if(pMouseEvent->source() == Qt::MouseEventSource::MouseEventSynthesizedBySystem || wheel_e_inprogress){
            p_event->ignore();
            pMouseEvent->ignore();
            return true;
        }

        p_event->ignore();
        pMouseEvent->ignore();
        if(pMouseEvent->button() == Qt::MiddleButton ||pMouseEvent->buttons() == Qt::MiddleButton){
            if(previous_touchp == QPoint(0,0)){
                previous_touchp = pMouseEvent->globalPos();
                return true;
            }
            else{
                if((pMouseEvent->globalPos().x() - previous_touchp.x()) > 20){
                    current_f_point_x += 1*sin(current_yaw);
                    current_f_point_y -= 1*cos(current_yaw);
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("X")->setValue( current_f_point_x );
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue( current_f_point_y );
                    previous_touchp = pMouseEvent->globalPos();

                }
                else if(pMouseEvent->globalPos().x() - previous_touchp.x() < -20){
                    current_f_point_x -= 1*sin(current_yaw);
                    current_f_point_y += 1*cos(current_yaw);
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("X")->setValue( current_f_point_x );
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue( current_f_point_y );
                    previous_touchp = pMouseEvent->globalPos();
                }
                if((pMouseEvent->globalPos().y() - previous_touchp.y()) > 20){
                    current_f_point_z += 0.2;
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue( current_f_point_z );
                    previous_touchp = pMouseEvent->globalPos();

                }
                else if(pMouseEvent->globalPos().y() - previous_touchp.y() < -20){
                    current_f_point_z -= 0.2;
                    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue( current_f_point_z );
                    previous_touchp = pMouseEvent->globalPos();
                }
            }
        }
        else{
            if(previous_touchp == QPoint(0,0))
                return true;
            else{
                if((pMouseEvent->globalPos().x() - previous_touchp.x()) > 5){
                    current_yaw += 0.05;
                    //ROS_INFO("Yaw: %f", current_yaw);
                    manager_->getViewManager()->getCurrent()->subProp("Yaw")->setValue( current_yaw );
                    previous_touchp = pMouseEvent->globalPos();

                }
                else if(pMouseEvent->globalPos().x() - previous_touchp.x() < -5){
                    current_yaw -= 0.05;
                    //ROS_INFO("Yaw: %f", current_yaw);
                    manager_->getViewManager()->getCurrent()->subProp("Yaw")->setValue( current_yaw );
                    previous_touchp = pMouseEvent->globalPos();
                }
                if(current_pitch < 1.57 && pMouseEvent->globalPos().y() - previous_touchp.y() > 5){
                    current_pitch += 0.05;
                    manager_->getViewManager()->getCurrent()->subProp("Pitch")->setValue( current_pitch );
                    previous_touchp = pMouseEvent->globalPos();

                }
                else if(current_pitch > -1.57 && pMouseEvent->globalPos().y() - previous_touchp.y() < -5){
                    current_pitch -= 0.05;
                    manager_->getViewManager()->getCurrent()->subProp("Pitch")->setValue( current_pitch );
                    previous_touchp = pMouseEvent->globalPos();
                }
            }
        }

        return true;
    }
    else if(p_event->type() == QEvent::MouseButtonPress){
        QMouseEvent* pMouseEvent = dynamic_cast<QMouseEvent*>(p_event);
        if(pMouseEvent->source() == Qt::MouseEventSource::MouseEventSynthesizedBySystem){
            p_event->ignore();
            pMouseEvent->ignore();
            return true;
        }
        p_event->ignore();
        pMouseEvent->ignore();
        previous_touchp = pMouseEvent->globalPos();
        return true;
    }
    else if(p_event->type() == QEvent::MouseButtonRelease){
        QMouseEvent* pMouseEvent = dynamic_cast<QMouseEvent*>(p_event);
        if(pMouseEvent->source() == Qt::MouseEventSource::MouseEventSynthesizedBySystem){
            p_event->ignore();
            pMouseEvent->ignore();
            return true;
        }
        p_event->ignore();
        pMouseEvent->ignore();
        previous_touchp = QPoint(0,0);
        wheel_e_inprogress = false;
        return true;
    }
    else if(p_event->type() == QEvent::Wheel)
    {
        QWheelEvent* pWheelEvent = dynamic_cast<QWheelEvent*>(p_event);
        wheel_e_inprogress = true;
        if(pWheelEvent->source() == Qt::MouseEventSource::MouseEventSynthesizedBySystem)
        {
            p_event->ignore();
            pWheelEvent->ignore();
            return true;
        }
        else{
            //ROS_INFO("%d", pWheelEvent->pixelDelta().y());

            if ((current_f_distance >= 50 && pWheelEvent->pixelDelta().y() > 0) || (current_f_distance <= 1 && pWheelEvent->pixelDelta().y() < 0))
                return true;

            current_f_distance += pWheelEvent->pixelDelta().y()/10;
            manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue( current_f_distance );
        }
        p_event->ignore();
        pWheelEvent->ignore();
        return true;
    }
    else{
        //qDebug() << "handling an event" << p_event;

    }
    return false;
}

void MyViz::resetView()
{
    manager_->getViewManager()->getCurrent()->subProp("Pitch")->setValue(1.57);
    manager_->getViewManager()->getCurrent()->subProp("Yaw")->setValue(0);
    manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue(10);
    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "X" )->setValue(0);
    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "Y" )->setValue(0);
    manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "Z" )->setValue(0);
}

void MyViz::manualZoomOut()
{
    if (current_f_distance >= 50)
    {
    }
    else{
        current_f_distance += 0.5;
        manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue( current_f_distance );
    }
}

void MyViz::manualZoomIn()
{
    if (current_f_distance <= 1)
    {
    }
    else{
        current_f_distance -= 0.5;
        manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue( current_f_distance );
    }
}

