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

    central_widget_layout = new QGridLayout();
    central_widget_layout->setContentsMargins(0, 0, 0, 0);
    central_widget_layout->addWidget(container, 0, 0, 3, 3);
    central_widget_layout->addWidget(myviz, 0, 0, 2, 3);

    central_widget_layout->setSpacing(0);
    ui->centralwidget->setLayout(central_widget_layout);
    centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
    statusBar()->hide();

    QObject *item = qmlView->rootObject();
    QObject *reboot_button = item->findChild<QObject*>("continue");
    power_button = item->findChild<QObject*>("power_button");
    power_button_bg = item->findChild<QObject*>("power_button_bg");
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
    n_->param<std::string>("camera_stream", camera_stream, "/usb_cam/image_raw");


    velodynesub = n_->subscribe<sensor_msgs::PointCloud2>(velodyne_points, 1, &MainWindow::updateVelodyneStatus, this);
    imusub = n_->subscribe<nav_msgs::Odometry>(imu_odom, 1, &MainWindow::updateImuStatus, this);
    camerasub = n_->subscribe<sensor_msgs::Image>(camera_stream, 1, &MainWindow::updateCameraStatus, this);

    connect(imu_timer, SIGNAL(timeout()), this, SLOT(resetImuStatus()));
    connect(camera_timer, SIGNAL(timeout()), this, SLOT(resetCameraStatus()));
    connect(velodyne_timer, SIGNAL(timeout()), this, SLOT(resetVelodyneStatus()));

    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    connect( myviz->fullscreen_button, &QPushButton::clicked, this, &MainWindow::fullscreenToggle);
    connect( reboot_button, SIGNAL(rvizRenderSignal(QString)), this, SLOT(createRVizEvent()));

}

MainWindow::~MainWindow()
{
    delete ui;
    delete velodyne_timer;
    delete ros_timer;
    delete imu_timer;
}

void MainWindow::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QQuickItem* rootObject =  qmlView->rootObject();
    QSize newSize = event->size();
    if(rootObject) rootObject->setProperty("width",QVariant::fromValue(newSize.width()));
    if(rootObject) rootObject->setProperty("height",QVariant::fromValue(newSize.height()));
}

void MainWindow::createRVizEvent()
{

    //ROS_INFO("0");
    myviz->setHidden(false);
    //ROS_INFO("1");
    QObject *item = qmlView->rootObject();
    //ROS_INFO("1");
    while(power_button == nullptr)
        power_button = item->findChild<QObject*>("power_button");
    while(opencv_image == nullptr)
        opencv_image = item->findChild<QObject*>("opencvImage");
    //ROS_INFO("2");
    //unsigned int microsecond = 100000;

    //ROS_INFO("2.5");

    //while(opencv_image->property("source") ==  QString("./images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg"))
    //{
    //   usleep(3 * microsecond);
    //}
    //    opencv_image->setProperty("visible", true);
    //ROS_INFO("4");
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
    //ROS_INFO("connections done");
    //ROS_INFO("5");
    videoStreamer->openVideoCamera();
    qmlView->engine()->addImageProvider("live",liveimageprovider);

}

void MainWindow::powerClickedEmit(){

    emit powerButtonPressed();
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
    //ROS_INFO("6");
    bool counter = opencv_image->property("counter").toBool();
    //ROS_INFO("7");
    opencv_image->setProperty("counter", !counter);
    //ROS_INFO("8");
    if(videoStreamer->init && videoStreamer->current_frame_ptr != nullptr)
    {
        if(counter){
            opencv_image->setProperty("source", "image://live/image?id=1");
        }
        else
        {
            opencv_image->setProperty("source", "image://live/image?id=0");
        }
        opencv_image->setProperty("counter", !counter);
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


