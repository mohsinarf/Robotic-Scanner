#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#define DATABUFFNUM 4
//for processor thread
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define SORT(a,b) { if ((a)>(b)) SWAP((a),(b)); }
#define SWAP(a,b) { short temp=(a);(a)=(b);(b)=temp; }

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QMessageBox>
#include "structDef.h"
#include "spectrogram.h"
#include <QFileDialog>
#include <QTreeView>
#include <QLabel>
// AVI utilities -- for creating avi files
#include <windows.h>
#include <Vfw.h>
#define WIN32_LEAN_AND_MEAN
#include <pshpack1.h>
#include <poppack.h>

class coreProcessor : public QObject
{
    Q_OBJECT

public:
    coreProcessor(char filtType = 1, char krnlRadius = 1, short **Input = NULL,
                  short **Output = NULL, int x_length = 50, int y_length = 50, int startFrame=0,
                  int endFrame=SAMPLESPERPOINT, unsigned int buffScanPoints = 0, unsigned int * EnhancedAlgoResultArg = NULL,
                  structResult *resultInfoPtr_arg = NULL, int selectedPt_arg=0);
    ~coreProcessor();
    short filtType;
    short krnlRadius;
    int startFrame;
    int endFrame;
    int wvfrmAcquired;
    int selectedPt;
    bool abortThread;
    bool flushThread;

public slots:
    void process();
    void processVtwam();
    void processXCor();
    void processParallel();
    void updateRowColNumAcquired(int wvfrmAcquiredArg);
    void setAbortThread();
    void setFlushThread();

signals:
    void updateProgress(int curFrame,short FilterRadius, short filterType);
    //void errorSignal(QString err);
    void finished();

private:

    short **input;
    short **output;
    int x_length;
    int y_length;
    unsigned int buffScanPoints;
    unsigned int *EnhancedAlgoResult;
    structResult *resultInfoPtr;

    inline short optMed9(short *p);
    inline short optMed25(short *p);
    inline short optMed49(short *p);
    inline void IIRfilter(short* in,short* outBuff, short bandNo);


};

class dataProcessor : public QObject
{
    Q_OBJECT
public:
    explicit dataProcessor(structDaq *daqInfoPtrArg, structScan *scanInfoPtrArg, structResult *resultInfoArg,
                           spectrogram* qwtSpectrogramArg, spectrogram* qwtSpectrogram2Arg, QString progDataPathArg,
                           spectrogram** qwtSpectrogramSubbandArrPtrArg, QLabel *labelMovieDebug);
    ~dataProcessor();
    bool allocateMem();


    //The processing that is applied to the data present in dataBundleProcessed
    bool filtPass1en;
    short filterType;
    short filterRadius;
    short filterItr;
    bool filtPass2en;
    short filterType2;
    short filterRadius2;
    short filterItr2;
    unsigned int buffScanPoints;


public slots:
    void newWfmCopyToArray_slot(short *wfmPtr, int waveformNumber);
    void setframeNum(int frameNum_arg);
    inline unsigned countNonZeroInFrame();
    void selectDisplayBuffer(bool Filtered);
    void postProcessingFilteringRequested();
    void postProcessingVtwamRequested(QString VtwamTitleArg);
    void scanFinished(int);
    void rptScanFinished();
    void updateProcProgress(int curframeNum, short FilterRadius=1, short filterType=1);
    void procFinished();
    QString saveData(bool status);
    QString loadData(bool status);
    void saveDataComp(bool status);
    void loadDataComp(bool status);
    void plotResultTime(int pointToPlot);
    void saveScreenshot();
    void setframeNumSubband(int frameNum_arg);
    void chooseSubband(short subband);
    void saveMovie();
    void updateResultSpect();
    void parallelprocFinished();
    void saveScreenshotVtwam();
    void stop();
    void setRlTmMdFlt(bool applyRealTimeFilterArg);
    void procFinishedGather();
    void setBoostSpeedEn(bool val);
    void postProcessingXCorRequested();
    void saveScreenshotXcor();



private:
    structFilterTabs *filterTabDataBase;
    structFilterTabs *chosenFilterTabs;
    structDaq *daqInfoPtr;
    structScan *scanInfoPtr;
    structResult *resultInfoPtr;
    short*** dataBundle;
    int** rptScanAccDataBundle;
    unsigned int* EnhancedAlgoResult;
    short* screenShotResult;
    //short** dataBundleProcessed;
    short dispBuffIndex;
    bool dataBundleSaved[NUMOFBANDS];
    short* resultTimeSig;
    //short** displayFrame; // no memory assigned to this just a pointer

    int frameNum;
    coreProcessor *coreProcObj;
    QThread* procThread ;

    coreProcessor ** coreProcObjArr;
    QThread** procThreadArr ;

    bool defaultFilter;
    QElapsedTimer timer;
    spectrogram *qwtSpectrogram,*qwtSpectrogram2;
    spectrogram** qwtSpectrogramSubbandArrPtr;

    float buffScanWidth;
    float buffScanHeight;
    float buffScanInterval;
    QString progDataPath;
    bool EnhanceAlgoProcessing;
    QWidget *screenshotPlace;
    QWidget *EnhanceAlgoPlace;
    short bufToProcess;
    short frameNumSubband;
    short chosenSubband;
    bool subbandDecomp;
    QLabel * labelMovieDebug;
    short rptScansPerInspection;
    bool useCurrentResults;
    bool applyRealTimeFilter;
    int threadFinishedCount;
    int totalThreads;
    bool boostSpeedEn;
    bool postProcDone;
    QString EnhancedAlgoTitle;
    int EnhancedAlgoSelect; // -1 = no Algo, 0 = VTWAM, 1 = XCor,
    int selectedPoint;

    bool alloc2dArray16bit(int rows, int columns, bool allocRptScanBuffOnly=false);
    bool clear2dArray16bit(int rows,int columns,bool clearRptScanBuffOnly=false);
    void startProcThread(short filtType ,short krnlRadius,
                                        short **Input,short **Output,int x_length,int y_length);
    void startEnhanceAlgoThread(short **Input, unsigned int* result, unsigned int buffScanPoints, int selectedPoint, int EnhancedAlgoSelectArg);
    inline short IIRfilterB1(short in, bool reset);
    inline short IIRfilterB2(short in, bool reset);
    inline short IIRfilterB3(short in, bool reset);
    inline short IIRfilter(short in, bool reset, int filtCoeffIndex);
    void startParallelProcThread(short filtType ,short krnlRadius, short **Input,short **Output,int x_length,int y_length);
    void addToRepeatScanBuffer();
    void initFilterTabs();
    int getFiltCoeffIndex();
    inline bool isMemValid(short*** dataBundle, int dispBuffIndex, int frameNum , const char* callername,bool isSilent=false);

    //void imageFilter(char FiltType, char krnlSz, short * Input);

signals:
    void updateProgressBar_dataProcessorSignal(int Percentage);
    void updateStatusBar_dataProcessorSignal(QString StatusTip);
    void updateOsciPlot(double * WfmPointer);
    void updateResultTimePlot(short * y_pt, QString title);
    void updateRowColNumAcquired_sig(int);
    void abortThread_sig();
    void flush_sig();
    void setStagePosX(uint,bool);
    void setStagePosZ(uint,bool);
    void wfmCopyDone_sig(int);
};

DECLARE_HANDLE(HAVI);
// An HAVI identifies an avi file that is being created
HAVI CreateAvi(const char *fn, int frameperiod, const WAVEFORMATEX *wfx);
HRESULT AddAviFrame(HAVI avi, HBITMAP hbm);
HRESULT CloseAvi(HAVI avi);


#endif // DATAPROCESSOR_H
