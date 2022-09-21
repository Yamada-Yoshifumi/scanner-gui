#include "mainwindow.h"
#include "qnamespace.h"
#include "ui_mainwindow.h"
#include "opencvimageprovider.h"
#include "videostreamer.h"
#include <QQuickView>
#include <QQuickItem>
#include <QHBoxLayout>
#include <QPushButton>
#include <QtQml>
#include <QGraphicsOpacityEffect>
#include <QColor>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <unistd.h>

/*
The MainWindow class:
    Initialises the RViz window
    Wraps qml code in a container and initialises it
    Set layout
    Identifies and wraps useful QQuick Objects in the qml code, executes methods, or may as well transmit signals to other C++ objects upon detecting signals
    Subscribes to ROS topics to which hardware packages publish, in order to determine if certain hardware is online
    Calls methods in VideoStreamer to stream ROS sensor_msgs/Image
    Change color/shape/status of various indicators in the qml frontend when hardware status changes
*/

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);

    qmlView = new QQuickView();

    qmlView->setSource(QUrl(QStringLiteral("qrc:/qml/App.qml")));

    container = QWidget::createWindowContainer(qmlView, this);
    container->setMinimumSize(1280, 720);
    container->adjustSize();

    myviz = new MyViz(this);
    QSizePolicy sp_retain = myviz->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    myviz->setSizePolicy(sp_retain);
    myviz->setHidden(true);

    countdown_widget = new QLabel(this);
    countdown_widget -> setStyleSheet("background-color: rgba(10,10,10,1);color : white;font-weight: bold; font-size: 200px; qproperty-alignment: AlignCenter;");
    countdown_widget -> setNum(3);
    countdown_widget -> setVisible(false);

    central_widget_layout = new AnimatedGridLayout();
    central_widget_layout->setContentsMargins(0, 0, 50, 0);
    central_widget_layout->addWidget(container, 0, 0, 3, 3);
    central_widget_layout->addWidget(myviz, 0, 0, 2, 3);

    //central_widget_layout->addWidget(settings_container, 0, 2, 3, 1);
    //central_widget_layout->addWidget(countdown_widget, 0, 0, 3, 3);

    central_widget_layout->setSpacing(0);
    ui->centralwidget->setLayout(central_widget_layout);
    statusBar()->hide();

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
    n_.reset(new ros::NodeHandle("status"));

    std::string velodyne_points;
    n_->param<std::string>("velodyne_points", velodyne_points, "/velodyne_points");
    std::string imu_odom;
    n_->param<std::string>("imu_odom", imu_odom, "/odom");
    std::string camera_stream;
    n_->param<std::string>("camera_stream", camera_stream, "/rrbot/camera1/image_raw");


    velodynesub = n_->subscribe<sensor_msgs::PointCloud2>(velodyne_points, 1, &MainWindow::updateVelodyneStatus, this);
    imusub = n_->subscribe<nav_msgs::Odometry>(imu_odom, 1, &MainWindow::updateImuStatus, this);
    camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);

    connect(imu_timer, SIGNAL(timeout()), this, SLOT(resetImuStatus()));
    connect(camera_timer, SIGNAL(timeout()), this, SLOT(resetCameraStatus()));
    connect(velodyne_timer, SIGNAL(timeout()), this, SLOT(resetVelodyneStatus()));

    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    connect( myviz->fullscreen_button, &QPushButton::clicked, this, &MainWindow::fullscreenToggle);
    connect( reboot_button, SIGNAL(rvizRenderSignal(QString)), this, SLOT(createRVizEvent()));


    settingsqmlView = new QQuickView();
    settingsqmlView->setSource(QUrl(QStringLiteral("qrc:/qml/Settings.qml")));
    auto offlineStoragePath = QUrl::fromLocalFile(settingsqmlView->engine()->offlineStoragePath());
    settingsqmlView->engine()->rootContext()->setContextProperty("offlineStoragePath", offlineStoragePath);
    settings_container = QWidget::createWindowContainer(settingsqmlView, this);
    settings_container->setMinimumSize(480, 720);
    settings_container->adjustSize();
    settings_container->move(QPoint(this->width()*3/2, 0));
    settings_container->raise();
    QObject *settingsItem = settingsqmlView->rootObject();

    settings_toggle_button = settingsItem->findChild<QObject*>("settings_toggle_button");
    connect(settings_toggle_button, SIGNAL(settingsToggle(QString)), this, SLOT(toggleSettings()));
    connect(settings_toggle_button, SIGNAL(exit(QString)), this, SLOT(closeWindow()));

    
    while(scan_button == nullptr)
        scan_button = settingsItem->findChild<QObject*>("scan_button");
    while(record_button == nullptr)
        record_button = settingsItem->findChild<QObject*>("record_button");

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

    QTimer::singleShot(0, this, SLOT(showFullScreen()));

    ROS_INFO("%s", settingsqmlView->engine()->offlineStoragePath().toStdString().c_str());
}

MainWindow::~MainWindow()
{
    delete ui;
    delete velodyne_timer;
    delete ros_timer;
    delete imu_timer;
}

void MainWindow::closeWindow(){
    close();
}

void MainWindow::spinOnce(){
    //this->changeInDatabaseResponse();
    if(ros::ok()){
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QQuickItem* rootObject =  qmlView->rootObject();
    newSize = event->size();
    if(rootObject) rootObject->setProperty("width",QVariant::fromValue(newSize.width()));
    if(rootObject) rootObject->setProperty("height",QVariant::fromValue(newSize.height()));

    rootObject =  settingsqmlView->rootObject();
    if(rootObject) rootObject->setProperty("height",QVariant::fromValue(newSize.height()));

    if(!settings_shown){
        settings_container->resize(settings_container->width(), newSize.height());
        settings_container->move(QPoint(newSize.width() - 50, 0));
    }
    else{
        settings_container->resize(settings_container->width(), newSize.height());
        settings_container->move(QPoint(newSize.width() - settings_container->width(), 0));
    }
}

void MainWindow::switchVideoSource(int source){
    if(source == 0){
        std::string camera_stream;
        n_->param<std::string>("camera_stream", camera_stream, "/rrbot/camera1/image_raw");
        camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);
    }
    else if(source == 1){
        std::string camera_stream;
        n_->param<std::string>("camera_stream", camera_stream, "/rrbot/camera2/image_raw");
        camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);
    }
}

void MainWindow::createRVizEvent()
{
    myviz->setHidden(false);
    QObject *item = qmlView->rootObject();
    while(power_button == nullptr)
        power_button = item->findChild<QObject*>("power_button");
    
    while(opencv_image == nullptr)
        opencv_image = item->findChild<QObject*>("opencv_image");

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

}

void MainWindow::powerClickedEmit(){

    emit powerButtonPressed();
}

void MainWindow::scanClickedEmit(){
    if (!scan_button->property("scanning").toBool())
    {
        countdown_widget -> setVisible(true);
        central_widget_layout->addWidget(countdown_widget, 0, 0, 3, 3);
        emit scanButtonPressed();
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

void MainWindow::recordClickedEmit(){
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

void MainWindow::updateCountDownNum(){
    int current_number = countdown_widget->text().toInt();
    if(current_number == 0){
        central_widget_layout->removeWidget(countdown_widget);
        countdown_widget ->setVisible(false);
        countdown_widget ->setNum(3);
        scan_countdown_timer -> stop();
        myviz->fullscreen_button->click();
        scan_button->setProperty("scanning", true);
    }
    else{
        current_number -= 1;
        countdown_widget->clear();
        countdown_widget->setNum(current_number);
    }
}

void MainWindow::toggleSettings(){

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

void MainWindow::updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg){
    velodyne_timer->start(1000);
    velodyne_status = 1;
    paintStatus();
}

void MainWindow::updateImuStatus(const nav_msgs::OdometryConstPtr &msg){
    imu_timer->start(1000);
    imu_status = 1;
    //paintStatus();
}

void MainWindow::updateCameraStatus(const sensor_msgs::ImageConstPtr &msg){
    videoStreamer->convertROSImage(msg);
    camera_timer->start(1000);
    camera_status = 1;

    //paintStatus();
}

void MainWindow::resetVelodyneStatus(){
    velodyne_status = 0;
    if(power_button_bg != nullptr)
        paintStatus();
}

void MainWindow::resetImuStatus(){
    imu_status = 0;
    //if(power_button_bg != nullptr)
    //    paintStatus();
}

void MainWindow::resetCameraStatus(){
    camera_status = 0;
    //if(power_button_bg != nullptr)
    //    paintStatus();
}

void MainWindow::imageReload(){
    QObject *item = qmlView->rootObject();
    opencv_image = item->findChild<QObject*>("opencv_image");
    if(videoStreamer->init && videoStreamer->current_frame_ptr != nullptr)
    {
        ROS_INFO("1");
        if(counter){
            ROS_INFO("2");
            opencv_image->setProperty("source", "image://live/image?id=1");
        }
        else
        {
            ROS_INFO("2");
            opencv_image->setProperty("source", "image://live/image?id=0");
        }
        counter = !counter;
    }
}

void MainWindow::paintStatus(){
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

void MainWindow::fullscreenToggle()
{
    if (myviz->fullscreen_button->objectName() == (QStringLiteral("fullscreen_button")))
    {
        central_widget_layout->removeWidget(myviz);
        central_widget_layout->addWidget(myviz, 0, 0, 3, 3);
        myviz->fullscreen_button->setObjectName(QStringLiteral("exit_fullscreen_button"));
        myviz->fullscreen_button->setIcon(QIcon(":/qml/images/exit_fullscreen.png"));
    }
    else if(myviz->fullscreen_button->objectName() == (QStringLiteral("exit_fullscreen_button")))
    {
        central_widget_layout->removeWidget(myviz);
        central_widget_layout->addWidget(myviz, 0, 0, 2, 3);
        myviz->fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
        myviz->fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
    }
}


