#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QTcpSocket>
#include "structDef.h"

class robotController : public QObject
{
    Q_OBJECT
public:
    explicit robotController(structScan * scanInfoPtr_arg);
    ~robotController();

    float scanStartPosX,scanStartPosZ;
    structRobTarget homePos;
public slots:
    bool sendStrCmd(char *sendString, QByteArray* rspArray = NULL);
    bool home();
    void stop();
    void Jogxp();
    void Jogxn();
    void Jogyp();
    void Jogyn();
    void Jogzp();
    void Jogzn();
    void jogStart(int axis, int direction);
    void jogStop();
    void jogSlot();
    void moveToTarget(structRobTarget target, bool updateToolRotation = false,bool useMoveJ = false);
    structRobTarget getPos(bool getHomePos = false);
    bool markScanArea();
    bool startScan();
    void gotoScanPrePos();
    void gotoScanStart();
    void genTestPattern(short testPatternNum);
    bool uploadScanGrid();
    inline structRobTarget offs(structRobTarget inPos, float xOffset, float yOffset, float zOffset);
    bool setIsVcCont(bool isVcContSlected);
    void turnOffPos();
    void loadMountPos();
    void bracketAttachPos();
    void genTrig();
    void ldvLaserTurnOn(bool turnOn);

private:
    QTcpSocket *tcpSocketIrc;
    QTimer *jogTimer;
    structScan * scanInfoPtr;
    structRobTarget scanStartPos;
    structRobTarget posForJog;

    float *posForJogPtr;
    int jogAxis;
    int jogDirection;
    bool useVcCont;
    QString robContIp;

signals:
    void updateProgressBar_robotControllerSignal(int Percentage);
    void updateStatusBar_robotControllerSignal(QString StatusTip);

    void Xpos(QString);
    void Ypos(QString);
    void Zpos(QString);
};

#endif // ROBOTCONTROLLER_H

