#ifndef DIALOGENLARGE_H
#define DIALOGENLARGE_H

#include <QDialog>
#include <QSlider>5
#include <QWindowStateChangeEvent>


namespace Ui {
class DialogEnlarge;
}

class DialogEnlarge : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEnlarge(QWidget *parent = 0);
    ~DialogEnlarge();

    void attachSpect(QWidget *spect);
    void changeEvent(QEvent *e);
    QSlider* giveSlider();

private:
    Ui::DialogEnlarge *ui;
};

#endif // DIALOGENLARGE_H
