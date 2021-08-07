#ifndef LASERCONTROLLER_H
#define LASERCONTROLLER_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include "structDef.h"

class laserController : public QObject
{
    Q_OBJECT
public:
    explicit laserController(structScan *laserInfoPtrArg = 0, QString portName = 0);
    ~laserController();

    bool initLaserController();
    bool onSHT();
    bool offSHT();
    bool onEXT();
    bool offEXT();
    bool setCurrent();
    bool getCurrent(QString *sCurrent);
    bool setPRF();
    bool offDIO();
    bool onDIO();
    bool offQS();
    bool onQS();
    bool initLaserControllerDone;
    bool srchPortandConnect(QString comPortHint, QString* laserComPortSel);


private:
    QSerialPort *serial;
    structScan *laserInfoPtr;
    QString LaserComPort;

    void initSerialPort();
    bool rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString *qsRsp=NULL);

signals:
    void updateProgressBar_laserControllerSignal(int Percentage);
    void updateStatusBar_laserControllerSignal(QString StatusTip);

};



#endif // SERIALPORT_DIALOG_H
