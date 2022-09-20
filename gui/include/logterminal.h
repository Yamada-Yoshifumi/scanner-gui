#ifndef LOGTERMINAL_H
#define LOGTERMINAL_H
#include <QWidget>
#include <QtQml>
#include <QPushButton>
#include <QWheelEvent>
#include <QTextEdit>
#include <QLabel>
#include <QVBoxLayout>

class LogTerminal: public QWidget
{
    Q_OBJECT
    public:
        LogTerminal( QWidget* parent = 0 );
        QTextEdit* text_box;
        QLabel* top_label;

    virtual ~LogTerminal();

};

#endif // LOGTERMINAL_H
