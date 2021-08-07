#ifndef DAQCONTROLLERS_H
#define DAQCONTROLLERS_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QMessageBox>
#include "structDef.h"

/*
#include "dlltyp.h"
#include "regs.h"
#include "spcerr.h"
#include "spcm_drv.h"
*/
// ----- include of common example librarys -----
#include "spcm_lib_card.h"
#include "spcm_lib_thread.h"
#include "spcm_lib_data.h"
#include "ostools/spcm_ostools.h"

class fetcher : public QObject
{
    Q_OBJECT

public:
    fetcher();
    ~fetcher();

public slots:
    void fetch();
    void fetchOsciMode();
    void setPars(ST_SPCM_CARDINFO* pstCard_arg, qint32 reqWfmCnt_arg, void * pvDataBuffer_arg, bool OsciModeEn_arg, unsigned long dwDataBufLen_arg, structScan *scanInfoPtr_arg);
    void reset();
    void resetWaitForCopy();

signals:
    void newArrival(int,long);
    void errorSignal(QString err);
    void finished(int);

private:
    unsigned int  reqWfmCnt_f;//total number of waveforms to be acquired based on scan settings.
    unsigned int  fetchedWfmCnt_f; //Wavforms read so far.
    QTimer timer;
    bool OsciModeEn_f;
    ST_SPCM_CARDINFO*    pstCard_f;
    void *pvDataBuffer_f;
    uint32 dwDataBufLen_f;
    bool waitForCopy;

    //just for debug
    int64 accLineTrigCount,LineTrigCount,prvAccLineTrigCount, lineNo;
    structScan *scanInfoPtr_f;

};

class daqController : public QWidget
{
    Q_OBJECT
public:
    explicit daqController(structDaq *daqInfoPtrArg = 0, structScan *scanInfoPtrArg = 0);
    ~daqController();

    //void testConfigureAndFetch();
    void StartAcquisition();
    void StopAcquisition();
    bool OsciModeEn;

public slots:
    void Configure(bool OsciModeEn=false);
    void errorHandler(QString);
    void newArrivalInDaq(int, long lAvailPos);
    void acqFinished(int);
    void setallowOsciUpdate();
    void wfmCopyDone(int);

private:
    void daqErrorHandler(uint32 dwError, ST_SPCM_CARDINFO *pstCardInfo);

    structDaq *daqInfoPtr;
    structScan *scanInfoPtr;

    void* binaryWfmBuff;

    fetcher *fetcherObj;
    QThread* fetcherThread ;
    bool ConfigDone;
    bool allowOsciUpdate;

    QElapsedTimer timer;
    QTimer *updateTimerdaq;

    void getHandle();
    void configFetchThread();

    ST_SPCM_CARDINFO*    pstCard;
    bool bContMemUsed;
    uint32 dwDataBufLen;
    uint32 dwDataNotify;
    long isDemoCard;
    //ST_WORKDATA*         stWorkData;
    //ST_BUFFERDATA*       stBufferData;       // buffer and start information

signals:
    void updateProgressBar_daqControllerSignal(int Percentage);
    void updateStatusBar_daqControllerSignal(QString StatusTip);
    void newWfmReadyForCopy(short *WfmPointer,int waveformNumber);
    void scanFinished(int);
    void updatePlotOsci(short *);
};



#endif // DAQCONTROLLER_H
