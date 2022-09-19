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
#include <QPinchGesture>
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "qapplication.h"
#include "touchpad.h"
#include "myviz/myviz.hpp"
#include <QMainWindow>

// BEGIN_TUTORIAL
// Touch pad reads touchevents, and "maps" the touchevents to corresponding mouse/wheel events, and sends to RViz
TouchPad::TouchPad( QWidget* parent )
    : QWidget( parent )
{

    setAttribute(Qt::WA_AcceptTouchEvents);
    setAutoFillBackground(true);

    panel = new QWidget(this);
    panel->setStyleSheet("background-color:#473f4d;");

    QGridLayout* gridlayout = new QGridLayout();
    gridlayout->setContentsMargins(3,3,3,3);
    gridlayout->addWidget(panel, 0, 0, 9, 2);
    setLayout(gridlayout);
}

// Destructor.
TouchPad::~TouchPad()
{
}

bool TouchPad::event(QEvent* event){

    event->accept();
    if(event->type() == QEvent::TouchBegin ||
       event->type() == QEvent::TouchUpdate ||
        event->type() == QEvent::TouchEnd ){
        auto mainWin = qobject_cast<MyViz*>(window());
        QList<QTouchEvent::TouchPoint> touchPoints = static_cast<QTouchEvent *>(event)->touchPoints();
        if(touchPoints.count() == 2){
            _pos_1 = touchPoints[0].pos();
            _pos_2 = touchPoints[1].pos();
            QPointF _global_pos = parentWidget()->mapToGlobal(QPoint(0,0));
            QPointF _win_pos_1 = _global_pos + _pos_1;
            QPointF _win_pos_2 = _global_pos + _pos_2;
            QPointF _trunc_pos = QPointF(trunc((_pos_1.x() + _pos_2.x())/2), trunc((_pos_1.y() + _pos_2.y())/2));
            QPoint _int_win_pos = QPoint(int((_win_pos_1.x() + _win_pos_2.x()+10)/2), int((_win_pos_1.y() + _win_pos_2.y()+10)/2));
            if(event->type() == QEvent::TouchUpdate){

                double _current_spacing = sqrt(pow(_win_pos_1.x() - _win_pos_2.x(), 2) + pow(_win_pos_1.y() - _win_pos_2.y(), 2));
                if (previous_spacing == 0 || previous_mid_point == QPoint(0,0)){
                    previous_spacing = _current_spacing;
                    previous_mid_point = _int_win_pos;
                    return true;
                }

                if (abs(_current_spacing - previous_spacing) < 10 ){
                    QMouseEvent event(QEvent::MouseMove, _trunc_pos, _trunc_pos, _int_win_pos, Qt::NoButton, Qt::MiddleButton, Qt::NoModifier);
                    this->parentWidget()->eventFilter(this->parentWidget(), &event);
                    previous_spacing = _current_spacing;
                    if((_int_win_pos.x() - mainWin->previous_touchp.x()) > 20){
                        mainWin->current_f_point_x += 1*sin(mainWin->current_yaw);
                        mainWin->current_f_point_y -= 1*cos(mainWin->current_yaw);
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("X")->setValue( mainWin->current_f_point_x );
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue( mainWin->current_f_point_y );
                        mainWin->previous_touchp = _int_win_pos;

                    }
                    else if(_int_win_pos.x() - mainWin->previous_touchp.x() < -20){
                        mainWin->current_f_point_x -= 1*sin(mainWin->current_yaw);
                        mainWin->current_f_point_y += 1*cos(mainWin->current_yaw);
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("X")->setValue( mainWin->current_f_point_x );
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue( mainWin->current_f_point_y );
                        mainWin->previous_touchp = _int_win_pos;
                    }
                    if((_int_win_pos.y() - mainWin->previous_touchp.y()) > 20){
                        mainWin->current_f_point_z += 0.2;
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue( mainWin->current_f_point_z );
                        mainWin->previous_touchp = _int_win_pos;

                    }
                    else if(_int_win_pos.y() - mainWin->previous_touchp.y() < -20){
                        mainWin->current_f_point_z -= 0.2;
                        mainWin->manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue( mainWin->current_f_point_z );
                        mainWin->previous_touchp = _int_win_pos;
                    }
                }

                else if (abs(_current_spacing - previous_spacing) > 20){
                    QPoint pixelDelta;
                    QPoint angleDelta;
                    if(abs(_current_spacing - previous_spacing) > 40){
                        pixelDelta = QPoint(0, 0);
                        angleDelta = QPoint(0, 0);
                    }
                    else{
                        pixelDelta = QPoint(0, -int((_current_spacing - previous_spacing)));
                        angleDelta = QPoint(0, -int((_current_spacing - previous_spacing)/8));
                    }
                    if ((mainWin->current_f_distance >= 50 && pixelDelta.y() > 0) || (mainWin->current_f_distance <= 1 && pixelDelta.y() < 0))
                        return true;

                    mainWin->current_f_distance += pixelDelta.y()/10;
                    mainWin->manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue( mainWin->current_f_distance );
                    previous_spacing = _current_spacing;
                }
            }
            else if(event->type() == QEvent::TouchBegin){
                previous_spacing = sqrt(pow(_win_pos_1.x() - _win_pos_2.x(), 2) + pow(_win_pos_1.y() - _win_pos_2.y(), 2));
                previous_mid_point = QPoint(int((_win_pos_1.x() + _win_pos_2.x()+10)/2), int((_win_pos_1.y() + _win_pos_2.y()+10)/2));
                QMouseEvent event(QEvent::MouseButtonPress, _trunc_pos, _trunc_pos, _int_win_pos, Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
            else if(event->type() == QEvent::TouchEnd){
                previous_spacing = 0;
                previous_mid_point = QPoint(0,0);
                QMouseEvent event(QEvent::MouseButtonRelease, _trunc_pos, _trunc_pos, _int_win_pos, Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
                this->parentWidget()->eventFilter(this->parentWidget(), &event);
            }
        }
        else if(touchPoints.count() == 1){

            _pos_1 = touchPoints[0].pos();
            QPointF _global_pos = parentWidget()->mapToGlobal(QPoint(0,0));
            QPointF _win_pos = _global_pos + _pos_1;
            QPointF _trunc_pos_1 = QPointF(trunc(_pos_1.x()), trunc(_pos_1.y()));
            QPoint _int_win_pos = QPoint(int(_win_pos.x()) + 5, int(_win_pos.y()) + 5);
            if(event->type() == QEvent::TouchUpdate){
                if((_int_win_pos.x() - mainWin->previous_touchp.x()) > 5){
                    mainWin->current_yaw += 0.05;
                    mainWin->manager_->getViewManager()->getCurrent()->subProp("Yaw")->setValue( mainWin->current_yaw );
                    mainWin->previous_touchp = _int_win_pos;

                    }
                else if(_int_win_pos.x() - mainWin->previous_touchp.x() < -5){
                    mainWin->current_yaw -= 0.05;
                    mainWin->manager_->getViewManager()->getCurrent()->subProp("Yaw")->setValue( mainWin->current_yaw );
                    mainWin->previous_touchp = _int_win_pos;
                    }
                if(mainWin->current_pitch < 1.57 && _int_win_pos.y() - mainWin->previous_touchp.y() > 5){
                    mainWin->current_pitch += 0.05;
                    mainWin->manager_->getViewManager()->getCurrent()->subProp("Pitch")->setValue( mainWin->current_pitch );
                    mainWin->previous_touchp = _int_win_pos;

                    }
                else if(mainWin->current_pitch > -1.57 && _int_win_pos.y() - mainWin->previous_touchp.y() < -5){
                    mainWin->current_pitch -= 0.05;
                    mainWin->manager_->getViewManager()->getCurrent()->subProp("Pitch")->setValue( mainWin->current_pitch );
                    mainWin->previous_touchp = _int_win_pos;
                    }
                }
            else if(event->type() == QEvent::TouchBegin){
                mainWin->previous_touchp = _int_win_pos;
            }
            else if(event->type() == QEvent::TouchEnd){
                mainWin->previous_touchp = QPoint(0,0);

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
