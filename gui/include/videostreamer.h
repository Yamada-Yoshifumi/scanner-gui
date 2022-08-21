#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QObject>
#include <QTimer>
#include <QImage>
#include <iostream>
#include <QtQml>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoStreamer: public QObject
{
    Q_OBJECT
    QML_ELEMENT


public:
    VideoStreamer();
    ~VideoStreamer();

public:
    void streamVideo();
    void convertROSImage(const sensor_msgs::ImageConstPtr &msg);
    //ros::Subscriber camerasub;
    cv_bridge::CvImagePtr current_frame_ptr;
    //ros::NodeHandlePtr nh_;
    bool init = false;


public slots:
    void openVideoCamera();

private:
    //void spinOnce();
    //cv::Mat frame;
    //cv::VideoCapture cap;
    QTimer tUpdate;
    //QTimer rostimer;



signals:
    void newImage(QImage img);
};

#endif // VIDEOSTREAMER_H
