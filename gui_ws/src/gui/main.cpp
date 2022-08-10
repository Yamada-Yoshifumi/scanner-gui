#include <QQmlApplicationEngine>
#include <QApplication>
#include <QQuickWidget>
//#include <QGuiApplication>
#include "mainwindow.h"
#include <ros/ros.h>

MainWindow* mainwindow;

int main(int argc, char **argv)
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
    }

    QApplication app( argc, argv );
    mainwindow = new MainWindow();
    mainwindow->setStyleSheet("background-color : #620b66");
    mainwindow->show();

    app.exec();

    delete mainwindow;
}
/*
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
        qmlRegisterType<MyViz>("Qt.examples.my_rviz", 1, 0, "MyViz");
    }

    QQmlApplicationEngine engine;
    //const QUrl url(u"qrc:/qml/App.qml"_qs);
    const QUrl url("qrc:/qml/App.qml");

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);
    engine.load(url);

    QQuickWidget *view = new QQuickWidget;
    view->setSource(url);
    view->show();
    //MyViz* myviz = new MyViz();

    //myviz->show();
    return app.exec();
}
*/
/*
#include <QQuickView>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->pushButton, SIGNAL(clicked()), qApp, SLOT(quit()));

    <strong>QQuickView *view = new QQuickView();
    QWidget *container = QWidget::createWindowContainer(view, this);
    container->setMinimumSize(200, 200);
    container->setMaximumSize(200, 200);
    container->setFocusPolicy(Qt::TabFocus);</strong>
        view->setSource(QUrl("qrc:/qml/App.qml"));
    <strong>ui->verticalLayout->addWidget(container);</strong>
}

MainWindow::~MainWindow()
{
    delete ui;
}
*/
