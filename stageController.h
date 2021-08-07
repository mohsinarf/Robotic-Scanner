#ifndef STAGECONTROLLER_H
#define STAGECONTROLLER_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QTcpSocket>
#include "structDef.h"

class stageController : public QObject
{
    Q_OBJECT
public:
    explicit stageController(structScan * scanInfoPtr_arg);
    ~stageController();

    float scanStartPosX,scanStartPosZ;
public slots:
    void JogzpStop();
    void JogznStop();
    void JogxpStop();
    void JogxnStop();

    void JogzpStart();
    void JogznStart();
    void JogxpStart();
    void JogxnStart();

    void Jogzp(bool enableJog);
    void Jogzn(bool enableJog);
    void Jogxp(bool enableJog);
    void Jogxn(bool enableJog);
    void originSet(void);
    void originReset(void);
    unsigned int getPosZ();
    unsigned int getPosX();
    void setPosZ(unsigned int,bool relToScanStart=false);
    void setPosX(unsigned int, bool relToScanStart=false);
    bool startScan(float height, float width, float PRF, float interval, bool startAtCurPos=true);
    void updatePos();
    void clearErrorReset(void);
    void setErrorReset(void);
    void setMovetoX();
    void setMovetoZ();
    void resetMovetoX();
    void resetMovetoZ();
    void setServoStop(void);
    void clearServoStop(void);
    bool markScanArea(float height, float width, float interval);
    void clearScanAreaMarker();
    bool isReadyForRptScan();
    void gotoScanStart();
    void gotoScanPrePos();
    bool startScanAbstract(unsigned int PRF, float interval, bool startAtCurPos = true, short testPatternNum=-1);
    void genTestPattern(short testPatternNum);

private:
    QTimer *updateTimer;
    structScan * scanInfoPtr;
    bool writePlcMem(short address, unsigned short value);
    bool writePlcMem32(short address, unsigned int value);
    bool readPlcMem(short address, unsigned short *returnVal);
    bool readPlcMem32(short address, unsigned int *returnVal);



signals:
    void updateProgressBar_stageControllerSignal(int Percentage);
    void updateStatusBar_stageControllerSignal(QString StatusTip);

    void Xpos(QString);
    void Zpos(QString);
};

#endif // STAGECONTROLLER_H
