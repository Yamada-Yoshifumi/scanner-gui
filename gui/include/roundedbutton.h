#ifndef ROUNDEDBUTTON_H
#define ROUNDEDBUTTON_H
#include <QWidget>
#include <QPushButton>
#include <QResizeEvent>


class RoundedButton: public QPushButton
{
    Q_OBJECT
    public:
        RoundedButton(QWidget* parent = 0);
    virtual ~RoundedButton();
    private:
        void resizeEvent(QResizeEvent* event);
};

#endif // ROUNDEDBUTTON_H
