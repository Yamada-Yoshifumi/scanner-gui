#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QObject>
#include <QTimer>
#include <opencv4/opencv2/highgui.hpp>
#include <QImage>
#include <iostream>
#include <QtQml>

class VideoStreamer: public QObject
{
    Q_OBJECT
    QML_ELEMENT
public:
    VideoStreamer();
    ~VideoStreamer();

public:
    void streamVideo();

public slots:
    void openVideoCamera(QString path);

private:
    cv::Mat frame;
    cv::VideoCapture cap;
    QTimer tUpdate;

signals:
    void newImage(QImage &);
};

#endif // VIDEOSTREAMER_H
