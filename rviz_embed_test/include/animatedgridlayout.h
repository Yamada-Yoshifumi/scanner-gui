#ifndef ANIMATEDGRIDLAYOUT_H
#define ANIMATEDGRIDLAYOUT_H
#include <QGridLayout>

class AnimatedGridLayout: public QGridLayout
{
    Q_OBJECT
    Q_PROPERTY(int intContentsMargins READ readCM WRITE setCM)

    public:
        AnimatedGridLayout( QWidget* parent = 0 );
        int intContentsMargins;
        int readCM();
        void setCM(int right);
};

#endif // ANIMATEDGRIDLAYOUT_H
