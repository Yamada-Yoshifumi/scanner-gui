#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
#include "myviz.h"
#include <qtimer.h>
#include "roshandler.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
    void rvizRenderSignal(QString);
    //void powerSignal(QString);

private:
    Ui::MainWindow *ui;
    QTimer *ros_timer;
    //ros::NodeHandlePtr n_;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    QQuickView *qmlView;
    QGridLayout* central_widget_layout;
    QObject *power_button;
    QWidget *container;
    MyViz* myviz;
    ROSHandler* roshandler;
    bool power_toggled = false;

public Q_SLOTS:
    void createRVizEvent();
    void fullscreenToggle();
    void spinOnce();
    void systemOn();

};

#endif // MAINWINDOW_H
