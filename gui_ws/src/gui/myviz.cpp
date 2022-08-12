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
#include "myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
    : QWidget( parent )
{

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
    fullscreen_button = new QPushButton();
    fullscreen_button->setObjectName(QStringLiteral("fullscreen_button"));
    fullscreen_button->setIcon(QIcon(":/qml/images/fullscreen.svg"));
    fullscreen_button->setStyleSheet("background-color:gray;");

    QGridLayout* main_layout = new QGridLayout;
    main_layout->addWidget( render_panel_, 0, 0, 10, 30 );
    main_layout->addWidget(fullscreen_button, 9, 29, 1, 1);

    // Set the top-level layout for this MyViz widget.
    setLayout( main_layout );

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
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
    pointcloud_->subProp("Color Transformer")->setValue("AxisColor");
    pointcloud_->subProp("Invert Rainbow")->setValue("true");

    /**E.g. For TF **/
    tf_ = manager_->createDisplay("rviz/RobotModel","lidar tf", true);
    // Initialize the slider values.
    this->setThickness( 10 );
    this->setCellSize( 10 );

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
