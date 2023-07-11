#include "dialogenlarge.h"
#include "ui_dialogenlarge.h"
#include "qdebug.h"


DialogEnlarge::DialogEnlarge(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEnlarge)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::Window|
                         Qt::CustomizeWindowHint|
                         Qt::WindowCloseButtonHint|
                         Qt::WindowMaximizeButtonHint);
}

DialogEnlarge::~DialogEnlarge()
{
    delete ui;
}

void DialogEnlarge::attachSpect(QWidget *spect)
{
    spect->setParent(ui->scrollAreaWidgetContents);
    ui->horizontalSliderEn->setValue(-3);
    this->show();
}

QSlider *DialogEnlarge::giveSlider()
{
    return ui->horizontalSliderEn;
}

void DialogEnlarge::changeEvent(QEvent *e)
{
/*
    if( e->type() == QEvent::WindowStateChange)
    {
        QWindowStateChangeEvent* event = static_cast< QWindowStateChangeEvent* >( e );

        if( event->oldState() & Qt::WindowMinimized )
        {
            qDebug() << "Window restored (to normal or maximized state)!";
        }
        //else if( event->oldState() == Qt::WindowNoState && this->windowState() == Qt::WindowMaximized )
        //{
        //    qDebug() << "Window Maximized!";
        //}
        //else if( event->oldState() == Qt::WindowNoState && this->windowState() == Qt::WindowMinimized)
        else if( this->windowState() == Qt::WindowMinimized ||
                 this->windowState() == Qt::WindowMinimized|Qt::WindowMaximized)
        {
            qDebug() << "Window Minimized!";
        }
        qDebug()<<"Event Type: "<<e->type()<<
                  "Current State: " <<this->windowState();
    }
*/
    QDialog::changeEvent(e);
}
