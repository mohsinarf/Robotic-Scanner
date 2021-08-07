#ifndef BANDPASSCONTROLLER_H
#define BANDPASSCONTROLLER_H

#include <QObject>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>

class bandpassController : public QObject
{
    Q_OBJECT
public:
    explicit bandpassController();
    ~bandpassController();

   bool config(short ch,short gain, short highpass, short lowpass);


private:
    QSerialPort *serial;
    QString filterComPort;
    void initSerialPort();
    bool rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString *qsRsp=NULL);

public slots:
};

#endif // BANDPASSCONTROLLER_H
