#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QObject>
#include <QTimer>
#include <QImage>
#include <iostream>
#include <QtQml>
#include "sensor_msgs/msg/image.hpp"
//#include "sensor_msgs/include/sensor_msgs/image_encodings.hpp"
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include "cv_bridge_ros2/cv_bridge.h"

class VideoStreamer: public QObject
{
    Q_OBJECT

public:
    VideoStreamer();
    ~VideoStreamer();

public:
    void streamVideo();
    void convertROSImage(const sensor_msgs::msg::Image &msg);
    cv_bridge::CvImagePtr current_frame_ptr;
    bool init = false;


public slots:
    void openVideoCamera();

private:
    QTimer tUpdate;

signals:
    void newImage(QImage img);
};

#endif // VIDEOSTREAMER_H
