#ifndef LMSCONTROLLER_H
#define LMSCONTROLLER_H

#include "RTC5impl.h"
#include "structDef.h"
#include <QThread.h>
#include <QObject>
#include <QMessageBox>
#include <QDebug>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTimer>


class lmsController : public QObject
{
    Q_OBJECT
public:
    explicit lmsController(structLms *lmsInfoPtrArg = 0, structScan *scanInfoPtrArg = 0);

public slots:

    void lmsMoveyp();
    void lmsMoveyn();
    void lmsMovexp();
    void lmsMovexn();
    void lmsScan();
    void extStart();
    void lmsInitialize();
    void lmsScanAreaError(QString direction);
    void lmsJumpAbs(long xAbs, long yAbs);
    void lmsMarkAbs(long xAbs, long yAbs);
    void lmsDisplayArea();
    void lmsAreaLimitCheck();
    void lmsLaserTiming();
    void lmsSetPos(float xLoc, float yLoc, float zLoc);
    void lmsClickToPointPos(structRobTarget robTarget);
    void printLmsScanGrid();
    void genTrig();
    void getTrigCount();
    void LmsDebug();
    void GenerateCorFile();
    void initCorFileParameters(int corXaxisRightLimitArg,int corXaxisLeftLimitArg,int corYaxisUpLimitArg,int corXaxisDownLimitArg,int corFileResolutionArg);

    void LmssetPixelPeriod(float PixelPeriodArg);
    void LmssetTrigCompress(float TrigCompressArg);

private:
    structLms *lmsInfoPtr;
    structScan *scanInfoPtr;
    long** posBundle;
    void UpdateLmsParStruct();
    void updatePos();
    int calFactorK;//used for mm to bit conversion

    float lastPixelPeriodInfo;
    float trigCompressInfo;

    //*************************//
int xAxisRightLimit = 0,xAxisLeftLimit = 0 ;
int yAxisUpLimit = 0,xAxisDownLimit = 0 ;
int CorFileResolution = 0;

signals:
    void Xpos(QString);
    void Ypos(QString);
    void Zpos(QString);
    void finished();
};

#endif // LMSCONTROLLER_H
