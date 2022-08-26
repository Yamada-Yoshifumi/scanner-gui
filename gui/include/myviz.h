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
#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>
#include <QtQml>
#include <QPushButton>
#include <QWheelEvent>
#include <QTextEdit>
#include "touchpad.h"
#include <QComboBox>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
    Q_OBJECT
    QML_ELEMENT
public:
    MyViz( QWidget* parent = 0 );
    QPushButton* fullscreen_button;
    TouchPad* touchpad;
    QGridLayout* main_layout;
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_, * pointcloud_, *tf_;
    QPushButton* zoomin_button;
    QPushButton* zoomout_button;
    QPushButton* reset_button;
    QComboBox* combo;

    virtual ~MyViz();
public Q_SLOTS:
    void resetView();
    void manualZoomIn();
    void manualZoomOut();

private Q_SLOTS:
    void setThickness( int thickness_percent );
    void setCellSize( int cell_size_percent );
    void colourPatternChanged();

private:
    QPoint previous_touchp;
    double current_pitch;
    double current_yaw;
    double current_f_distance;
    double current_f_point_x;
    double current_f_point_y;
    double current_f_point_z;
    QString current_pointcloud_pattern;
protected:
    bool eventFilter(QObject * p_obj, QEvent * p_event);
    bool focus_on_wheel = false;
};
// END_TUTORIAL
#endif // MYVIZ_H
