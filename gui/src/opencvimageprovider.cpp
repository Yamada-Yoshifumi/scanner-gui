#include "opencvimageprovider.h"
#include <ros/ros.h>

OpencvImageProvider::OpencvImageProvider(QObject *parent) : QObject(parent), QQuickImageProvider(QQuickImageProvider::Image)
{
}

QImage OpencvImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    Q_UNUSED(id);

    if(size){
        *size = image.size();
    }

    if(requestedSize.width() > 0 && requestedSize.height() > 0) {
        image = image.scaled(requestedSize.width(), requestedSize.height(), Qt::KeepAspectRatio);
    }
    return image;
}

void OpencvImageProvider::updateImage(const QImage &image)
{

    if(!image.isNull() && this->image != image) {
        this->image = image;
        //ROS_INFO("156575");
        emit imageChanged();
    }
}
