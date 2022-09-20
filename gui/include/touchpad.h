#ifndef TOUCHPAD_H
#define TOUCHPAD_H

#include <QWidget>
#include <QGridLayout>
#include <QtQml>
#include <QPushButton>
#include <QWheelEvent>
#include <QTextEdit>
#include <QPainter>

// BEGIN_TUTORIAL
// Class "TouchPad" implements the top level widget for this example.
class TouchPad: public QWidget
{
    Q_OBJECT
    public:
        TouchPad( QWidget* parent = 0 );
        QWidget* panel;

    virtual ~TouchPad();
    private:
        QPointF pos_1;
        QPointF pos_2;
        QPointF pos_3;
        QPointF _pos_1;
        QPointF _pos_2;
        QPointF _pos_3;
        double previous_spacing = 0;
        QPoint previous_mid_point;
    protected:
        bool eventFilter(QObject * p_obj, QEvent * p_event);
        bool event(QEvent *event);
};
// END_TUTORIAL
#endif // TOUCHPAD_H
