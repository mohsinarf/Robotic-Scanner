#ifndef LDVCONTROLLER_H
#define LDVCONTROLLER_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include "structDef.h"

class ldvController : public QObject
{
    Q_OBJECT
public:
    explicit ldvController(structDaq *ldvInfoPtrArg = 0, QString portName = 0);
    ~ldvController();

    bool setAutoFocus();
    bool setRange();
    bool setRangeAux();
    bool getAutoFocusResults();
    bool getControllerRemoteStatus();
    bool connectLdv(QString LdvComPortArg);
public slots:
    bool setMainPage();
    bool getSignalLevel();


private:
    QSerialPort *serial;
    structDaq *ldvInfoPtr;
    int baudRate;
    int veloDecNum;
    int veloDecNumAux;
    QString LdvComPort;
    QTimer *sigLevelTimer;

    void initSerialPort();
    bool rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString *strRsp=NULL);

signals:
    void updateProgressBar_ldvControllerSignal(int Percentage);
    void updateStatusBar_ldvControllerSignal(QString StatusTip);
    void ldvSignalLevel(int percentage);

};



#endif // LDVCONTROLLER_H
