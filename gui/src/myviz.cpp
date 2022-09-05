/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>
#include <QtWidgets/QPushButton>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/view_manager.h"
#include "myviz.h"
#include <QMetaEnum>
#include <QDebug>

/// Gives human-readable event type information.
QDebug operator<<(QDebug str, const QEvent * ev) {
    static int eventEnumIndex = QEvent::staticMetaObject
                                    .indexOfEnumerator("Type");
    str << "QEvent";
    if (ev) {
        QString name = QEvent::staticMetaObject
                           .enumerator(eventEnumIndex).valueToKey(ev->type());
        if (!name.isEmpty()) str << name; else str << ev->type();
    } else {
        str << (void*)ev;
    }
    return str.maybeSpace();
}

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
    : QWidget( parent )
{

    setAttribute(Qt::WA_NoSystemBackground);
    render_panel_ = new rviz::RenderPanel(this);
    render_panel_->installEventFilter(this);

    logterminal = new LogTerminal(render_panel_);
    QSizePolicy sp_retain = logterminal->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    logterminal->setSizePolicy(sp_retain);
    logterminal->setHidden(true);

    touchpad = new TouchPad(render_panel_);
    touchpad->setStyleSheet("background-color: rgba(10,10,10,0.8);");

    fullscreen_button = new QPushButton(this);
    fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
    fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
    fullscreen_button->setStyleSheet("background-color:#442e5d;");
    //fullscreen_button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    fullscreen_button->setIconSize(QSize(32,32));

    zoomin_button = new QPushButton(this);
    zoomin_button->setObjectName(QStringLiteral("zoomin_button"));
    zoomin_button->setIcon(QIcon(":/qml/images/zoom_in.png"));
    zoomin_button->setStyleSheet("background-color:#442e5d;");
    zoomin_button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    zoomin_button->setIconSize(QSize(64,64));
    //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>

    zoomout_button = new QPushButton(this);
    zoomout_button->setObjectName(QStringLiteral("zoomout_button"));
    zoomout_button->setIcon(QIcon(":/qml/images/zoom_out.png"));
    zoomout_button->setStyleSheet("background-color:#442e5d;");
    zoomout_button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    zoomout_button->setIconSize(QSize(64,64));
    //<a href="https://www.flaticon.com/free-icons/zoom-out" title="zoom out icons">Zoom out icons created by Freepik - Flaticon</a>

    reset_button = new QPushButton(this);
    reset_button->setObjectName(QStringLiteral("reset_button"));
    reset_button->setIcon(QIcon(":/qml/images/reset_rviz.png"));
    reset_button->setStyleSheet("background-color:#442e5d;");
    reset_button->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    reset_button->setIconSize(QSize(64,64));
    //<a href="https://www.flaticon.com/free-icons/axis" title="axis icons">Axis icons created by Smashicons - Flaticon</a>

    QStringList commands = { "Intensity", "AxisColor", "Uncertainty", "FlatColor" };
    combo = new QComboBox(this);
    combo->addItems(commands);

    main_layout = new QGridLayout;
    main_layout->setContentsMargins(5,5,5,5);
    main_layout->addWidget( touchpad, 0, 30, 8, 15);
    main_layout->addWidget( logterminal, 0, 30, 8, 15);
    main_layout->addWidget(zoomin_button, 8, 30, 2, 5);
    main_layout->addWidget(zoomout_button, 8, 35, 2, 5);
    main_layout->addWidget(reset_button, 8, 40, 2, 5);
    main_layout->addWidget( render_panel_, 0, 0, 10, 30 );
    main_layout->addWidget( fullscreen_button, 9, 29, 1, 1 );
    main_layout->addWidget(combo, 0, 25, 1, 5);

    setLayout(main_layout);
    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the touchpadController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );

    manager_->initialize();
    manager_->startUpdate();

    manager_->setFixedFrame("base_footprint");
    // Create a Grid display.
    grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( QColor( Qt::white ) );


    /**E.g. For velodyne **/
    pointcloud_ = manager_->createDisplay("rviz/PointCloud2","PointCloudAir", true);
    pointcloud_->subProp("Topic")->setValue("velodyne_points");
    pointcloud_->subProp("Style")->setValue("Points");
    pointcloud_->subProp("Size (Pixels)")->setValue("2");
    pointcloud_->subProp("Channel Name")->setValue("ring");
    pointcloud_->subProp("Color Transformer")->setValue("Intensity");
    //pointcloud_->subProp("Invert Rainbow")->setValue("true");

    /**E.g. For TF **/
    tf_ = manager_->createDisplay("rviz/RobotModel","lidar tf", true);
    // Initialize the slider values.
    this->setThickness( 10 );
    this->setCellSize( 10 );

    current_pitch = manager_->getViewManager()->getCurrent()->subProp("Pitch")->getValue().toDouble();
    current_yaw = manager_->getViewManager()->getCurrent()->subProp("Yaw")->getValue().toDouble();
    current_f_distance = manager_->getViewManager()->getCurrent()->subProp("Distance")->getValue().toDouble();
    current_f_point_x = manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "X" )->getValue().toDouble();
    current_f_point_y = manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "Y" )->getValue().toDouble();
    current_f_point_z = manager_->getViewManager()->getCurrent()->subProp("Focal Point")->subProp( "Z" )->getValue().toDouble();
    current_pointcloud_pattern = pointcloud_->subProp("Color Transformer")->getValue().toString();

    previous_touchp = QPoint(0,0);

    zoomin_button->setAutoRepeat(true);
    zoomout_button->setAutoRepeat(true);

    connect(reset_button, &QPushButton::clicked, this, &MyViz::resetView);
    connect(zoomin_button, &QPushButton::pressed, this, &MyViz::manualZoomIn);
    connect(zoomout_button, &QPushButton::pressed, this, &MyViz::manualZoomOut);
    connect( combo, &QComboBox::currentTextChanged, this, &MyViz::colourPatternChanged);
}

// Destructor.
MyViz::~MyViz()
{
    delete manager_;
}

void MyViz::setThickness( int thickness_percent )
{
    if( grid_ != NULL )
    {
        grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
    }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
    if( grid_ != NULL )
    {
        grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
    }
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

void MyViz::colourPatternChanged()
{
    QString current_selection = combo->currentText();
    pointcloud_->subProp("Color Transformer")->setValue(current_selection);
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
            ROS_INFO("%d", pWheelEvent->pixelDelta().y());

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

