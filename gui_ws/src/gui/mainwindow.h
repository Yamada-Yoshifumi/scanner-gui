#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QQuickView>
#include <QMainWindow>
#include <QGridLayout>
#include "myviz.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
    void qmlSignal(QString);

private:
    Ui::MainWindow *ui;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void resizeEvent(QResizeEvent* event);
    QQuickView *qmlView;
    QGridLayout* central_widget_layout;
    QWidget *container;
    MyViz* myviz;

public Q_SLOTS:
    void createRVizEvent();
    void fullscreenToggle();

};

#endif // MAINWINDOW_H
