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
    explicit lmsController(structLms *lmsInfoPtrArg = 0, structScan *scanInfoPtrArg = 0, long **posBundleArg = NULL);

public slots:

    void lmsMoveyp();
    void lmsMoveyn();
    void lmsMovexp();
    void lmsMovexn();
    void lmsScan();
    void lmsInitialize();
    void lmsScanAreaError(QString direction);
    void lmsJumpAbs(long xAbs, long yAbs);
    void lmsMarkAbs(long xAbs, long yAbs);
    void lmsDisplayArea();
    void lmsAreaLimitCheck();
    void lmsAlgorithm(long xLoc, long yLoc, unsigned int PIXELS, unsigned int LINES, double DotDist, unsigned int DotFreq);
    void lmsLaserTiming();
    void lmsSetPos(int xLoc, int yLoc);
    void extMode();
    void extTriggerCounter();
    int lmsImagePrint(strucLmsImage *picture, bool quitScan);

private:
    structLms *lmsInfoPtr;
    structScan *scanInfoPtr;
    long** posBundle;

    void UpdateLmsParStruct();
    void updatePos();

signals:
    void Xpos(QString);
    void Ypos(QString);
    void finished();
};

#endif // LMSCONTROLLER_H
