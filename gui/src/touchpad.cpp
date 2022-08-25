#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>
#include <QtWidgets/QPushButton>
#include <QtCore>
#include <ros/ros.h>

#include "qapplication.h"
#include "touchpad.h"


// BEGIN_TUTORIAL
// Constructor for TouchPad.  This does most of the work of the class.
TouchPad::TouchPad( QWidget* parent )
    : QWidget( parent )
{

    setAttribute(Qt::WA_AcceptTouchEvents);
    setAutoFillBackground(true);

    panel = new QWidget(this);
    panel->setStyleSheet("background-color:#473f4d;");

    zoomin_button = new QPushButton(this);
    zoomin_button->setObjectName(QStringLiteral("zoomin_button"));
    zoomin_button->setIcon(QIcon(":/qml/images/zoom_in.png"));
    //zoomin_button->resize(this->width()*7/15, this->height()/5);
    zoomin_button->setStyleSheet("background-color:gray;");
    //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>

    zoomout_button = new QPushButton(this);
    zoomout_button->setObjectName(QStringLiteral("zoomout_button"));
    zoomout_button->setIcon(QIcon(":/qml/images/zoom_out.png"));
    //zoomout_button->resize(this->width()*7/15, this->height()/5);
    zoomout_button->setStyleSheet("background-color:gray;");
    //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>

    QGridLayout* gridlayout = new QGridLayout();
    gridlayout->setContentsMargins(3,3,3,3);
    gridlayout->addWidget(panel, 0, 0, 9, 2);
    gridlayout->addWidget(zoomin_button, 9, 0, 1, 1);
    gridlayout->addWidget(zoomout_button, 9, 1, 1, 1);
    setLayout(gridlayout);
}

// Destructor.
TouchPad::~TouchPad()
{
}

bool TouchPad::event(QEvent* event){
    /*
    QMouseEvent scrollevent(QEvent::MouseButtonPress, pos, 0, 0, 0);
    QApplication::sendEvent(isWindow, &event);
    */
    event->accept();
    if(event->type() == QEvent::TouchBegin ||
       event->type() == QEvent::TouchUpdate ||
        event->type() == QEvent::TouchEnd ){

        QList<QTouchEvent::TouchPoint> touchPoints = static_cast<QTouchEvent *>(event)->touchPoints();
        if(touchPoints.count() == 2){
            _pos_1 = touchPoints[0].pos();
            _pos_2 = touchPoints[1].pos();
            QPointF _global_pos = parentWidget()->mapToGlobal(QPoint(0,0));
            QPointF _win_pos_1 = _global_pos + _pos_1;
            QPointF _win_pos_2 = _global_pos + _pos_2;

            ROS_INFO("2 detected");
            QPointF _trunc_pos = QPointF(trunc((_pos_1.x() + _pos_2.x())/2), trunc((_pos_1.y() + _pos_2.y())/2));
            QPoint _int_win_pos = QPoint(int((_win_pos_1.x() + _win_pos_2.x()+10)/2), int((_win_pos_1.y() + _win_pos_2.y()+10)/2));
            if(event->type() == QEvent::TouchUpdate){

                double _current_spacing = sqrt(pow(_win_pos_1.x() - _win_pos_2.x(), 2) + pow(_win_pos_1.y() - _win_pos_2.y(), 2));
                if (previous_spacing == 0){
                    previous_spacing = _current_spacing;
                    return true;
                }
                if (abs(_current_spacing - previous_spacing) < 20){
                    QMouseEvent event(QEvent::MouseMove, _trunc_pos, _trunc_pos, _int_win_pos, Qt::NoButton, Qt::MiddleButton, Qt::NoModifier);
                    this->parentWidget()->eventFilter(this->parentWidget(), &event);
                    previous_spacing = _current_spacing;
                }
            }
            else if(event->type() == QEvent::TouchBegin){
                previous_spacing = sqrt(pow(_win_pos_1.x() - _win_pos_2.x(), 2) + pow(_win_pos_1.y() - _win_pos_2.y(), 2));
                QMouseEvent event(QEvent::MouseButtonPress, _trunc_pos, _trunc_pos, _int_win_pos, Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
            else if(event->type() == QEvent::TouchEnd){
                previous_spacing = 0;
                QMouseEvent event(QEvent::MouseButtonRelease, _trunc_pos, _trunc_pos, _int_win_pos, Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
        }
        else if(touchPoints.count() == 1){
            _pos_1 = touchPoints[0].pos();
            QPointF _global_pos = parentWidget()->mapToGlobal(QPoint(0,0));
            QPointF _win_pos = _global_pos + _pos_1;
            //ROS_INFO("%f", _pos_1.x());
            QPointF _trunc_pos_1 = QPointF(trunc(_pos_1.x()), trunc(_pos_1.y()));
            QPoint _int_win_pos = QPoint(int(_win_pos.x()) + 5, int(_win_pos.y()) + 5);
            if(event->type() == QEvent::TouchUpdate){
                QMouseEvent event(QEvent::MouseMove, _trunc_pos_1, _trunc_pos_1, _int_win_pos, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
            else if(event->type() == QEvent::TouchBegin){
                QMouseEvent event(QEvent::MouseButtonPress, _trunc_pos_1, _trunc_pos_1, _int_win_pos, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
            else if(event->type() == QEvent::TouchEnd){
                QMouseEvent event(QEvent::MouseButtonRelease, _trunc_pos_1, _trunc_pos_1, _int_win_pos, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
    }
    }
    return true;
}

bool TouchPad::eventFilter(QObject * p_obj, QEvent * p_event)
{
    if (p_event->type() == QEvent::MouseButtonDblClick ||
        p_event->type() == QEvent::MouseButtonPress ||
        p_event->type() == QEvent::MouseButtonRelease ||
        p_event->type() == QEvent::GraphicsSceneMouseMove ||
        p_event->type() == QEvent::GraphicsSceneMouseDoubleClick ||
        p_event->type() == QEvent::GraphicsSceneMousePress ||
        p_event->type() == QEvent::GraphicsSceneMouseRelease ||
        p_event->type() == QEvent::NonClientAreaMouseButtonPress ||
        p_event->type() == QEvent::NonClientAreaMouseButtonRelease ||
        p_event->type() == QEvent::Wheel)
    {
        QMouseEvent* pMouseEvent = dynamic_cast<QMouseEvent*>(p_event);
        p_event->ignore();
        return true;
    }
    return false;
}
