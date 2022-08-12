#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/ros.h"
#include <QQuickView>
#include <QQuickItem>
#include <QHBoxLayout>
#include <QPushButton>
#include <QtQml>
#include <QGraphicsOpacityEffect>
#include <QColor>

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

    myviz = new MyViz();
    QSizePolicy sp_retain = myviz->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    myviz->setSizePolicy(sp_retain);
    myviz->setHidden(true);

    central_widget_layout = new QGridLayout();
    central_widget_layout->setContentsMargins(0, 0, 0, 0);
    central_widget_layout->addWidget(container, 0, 0, 3, 3);
    central_widget_layout->addWidget(myviz, 0, 0, 2, 2);

    central_widget_layout->setSpacing(0);
    ui->centralwidget->setLayout(central_widget_layout);
    centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
    statusBar()->hide();

    QObject *item = qmlView->rootObject();
    QObject *reboot_button = item->findChild<QObject*>("reboot_button");
    power_button = item->findChild<QObject*>("power_button");
    connect( myviz->fullscreen_button, &QPushButton::clicked, this, &MainWindow::fullscreenToggle);
    connect( reboot_button, SIGNAL(rvizRenderSignal(QString)), this, SLOT(createRVizEvent()));

    n_.reset(new ros::NodeHandle("qt_gui"));
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(20);

    roshandler = new ROSHandler(*n_);

}

MainWindow::~MainWindow()
{
    delete ui;
    delete ros_timer;
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
    myviz->setHidden(false);
    QObject *item = qmlView->rootObject();
    while(power_button == nullptr)
        power_button = item->findChild<QObject*>("power_button");
    connect(
        power_button,
        SIGNAL(powerSignal(QString)),
        this,
        SLOT(systemOn()));
}

void MainWindow::systemOn()
{
    bool success = roshandler->systemPowerOn();

    QObject *item = qmlView->rootObject();
    QObject *power_button_bg = item->findChild<QObject*>("power_button_bg");

    if(success){
        QColor color(Qt::green);
        power_button_bg -> setProperty("color", color);
    }
    else{
        QColor color(Qt::red);
        power_button_bg -> setProperty("color", color);
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
        central_widget_layout->addWidget(myviz, 0, 0, 2, 2);
        myviz->fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
        myviz->fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
    }
}
