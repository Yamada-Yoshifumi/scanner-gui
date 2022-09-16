#include "animatedgridlayout.h"

AnimatedGridLayout::AnimatedGridLayout(QWidget* parent):
    QGridLayout(parent)
{
    intContentsMargins = contentsMargins().right();
}

int AnimatedGridLayout::readCM(){
    return intContentsMargins;
}

void AnimatedGridLayout::setCM(int right){
    intContentsMargins = right;
    setContentsMargins(0, 0, right, 0);
}
