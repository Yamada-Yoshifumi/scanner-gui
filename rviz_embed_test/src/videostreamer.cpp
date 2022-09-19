#include "videostreamer.h"
//#include <ros/ros.h>

//Convert ROS Images to opencv copy, and emit a signal for OpencvImageProvider to update the stream

VideoStreamer::VideoStreamer()
{
    connect(&tUpdate,&QTimer::timeout,this,&VideoStreamer::streamVideo);

}

VideoStreamer::~VideoStreamer()
{
    tUpdate.stop();
}

void VideoStreamer::convertROSImage(const sensor_msgs::msg::Image & msg){
    init = true;
    current_frame_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
}

void VideoStreamer::streamVideo()
{
    if(current_frame_ptr != nullptr && init){
        QImage img = QImage(current_frame_ptr->image.data,current_frame_ptr->image.cols,current_frame_ptr->image.rows,QImage::Format_BGR888).rgbSwapped();
        emit newImage(img);
    }
}

void VideoStreamer::openVideoCamera()
{
    double fps = 30.0;
    tUpdate.start(30/fps);
}
