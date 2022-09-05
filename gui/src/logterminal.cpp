#include "logterminal.h"

LogTerminal::LogTerminal(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_NoSystemBackground);

    text_box = new QTextEdit(this);
    text_box->setStyleSheet("background-color: black; color: white; font-size: 20px");
    text_box->setAcceptRichText(true);

    top_label = new QLabel( "ROS Log Terminal" );
    top_label->setStyleSheet("background-color: grey; color: #d6d2d6;");
    top_label->setAlignment(Qt::AlignCenter);

    QVBoxLayout* v_layout = new QVBoxLayout(this);
    v_layout->setContentsMargins(3,3,3,3);
    v_layout->addWidget(top_label);
    v_layout->addWidget(text_box);
    setLayout(v_layout);
}

LogTerminal::~LogTerminal(){

}
