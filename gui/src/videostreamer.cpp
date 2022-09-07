#include "videostreamer.h"
#include <ros/ros.h>

//Convert ROS Images to opencv copy, and emit a signal for OpencvImageProvider to update the stream

VideoStreamer::VideoStreamer()
{
    connect(&tUpdate,&QTimer::timeout,this,&VideoStreamer::streamVideo);

}

VideoStreamer::~VideoStreamer()
{
    tUpdate.stop();
}

void VideoStreamer::convertROSImage(const sensor_msgs::ImageConstPtr &msg){
    init = true;
    current_frame_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
}

void VideoStreamer::streamVideo()
{
    /*
    if(init && current_frame_ptr){
        QImage img = QImage(current_frame_ptr->image.data,current_frame_ptr->image.cols,current_frame_ptr->image.rows,QImage::Format_BGR888).rgbSwapped();
        emit newImage(img);
    }else{
        QImage image = QImage(200,200,QImage::Format_BGR888).rgbSwapped();
        image.fill(QColor("black"));
        //ROS_INFO("bg");

        emit newImage(image);
    }*/
    if(current_frame_ptr != nullptr && init){
        //ROS_INFO("UPdated");
        QImage img = QImage(current_frame_ptr->image.data,current_frame_ptr->image.cols,current_frame_ptr->image.rows,QImage::Format_BGR888).rgbSwapped();
        emit newImage(img);
    }
}

void VideoStreamer::openVideoCamera()
{
    //ROS_INFO("openned");
    double fps = 10.0;
    tUpdate.start(1000/fps);
    //rostimer.start(1000/fps);
}
