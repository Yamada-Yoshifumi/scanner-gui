#include "roundedbutton.h"

RoundedButton::RoundedButton(QWidget* parent)
    : QPushButton(parent)
{
    /*
    this->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    QRect* rect_1 = new QRect(0,0,this->size().width() - 10, this->size().height() - 10);
    QRegion region_1(*rect_1, QRegion::Ellipse);
    this->setMask(region_1);
    */
    this->setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
    this->setStyleSheet("background-color:#b452fa; border-top-right-radius: 10; border-top-left-radius: 10; border-bottom-right-radius: 10; border-bottom-left-radius: 10;");
}

RoundedButton::~RoundedButton(){

}

void RoundedButton::resizeEvent(QResizeEvent* event)
{
}
