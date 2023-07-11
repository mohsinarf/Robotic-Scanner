#ifndef STRUCTDEF_H
#define STRUCTDEF_H


#include <Qvector>
#include <Qstring>
#include <vector>

struct structPos
{
    float x;
    float y;
    float z;
};

struct structOrient
{
    float q1;
    float q2;
    float q3;
    float q4;
};

struct structRobConf
{
    char cf1;
    char cf2;
    char cf4;
    char cfx;
};

struct structRobTarget
{
    structPos trans;
    structOrient rot;
    structPos transAfterAngle;
};

struct structRobTargetWConf
{
    structPos trans;
    structOrient rot;
    structRobConf cfg;
};

struct structScanSetting
{
    int speed;
    float pathLen;
    float trigInterval;
};

struct structDaq
{
    short freqMode;
    int SamplingFreq;
    unsigned int ScanPoints;
    bool subbandDecomp;
    int startFreqBandPass;
    int stopFreqBandPass;
    short totalNumOfScans;
    short Range; //LDV range
    unsigned short chMap; // 1-> selects channel-1 , 2-> selects channel->2, 3->selects average of channel 1&2
    int daqVoltage;
    int daqTrigDelay;
    short osciChan;
    QString settingStr;

};

struct structScanLineRobot
{
    QVector<structRobTarget> scanLine;
};

struct structScanLineAS
{
    float y;
    float yEnd;
    float length;
    unsigned int accNumOfTrigs; //acc per line
    //for easy debugging
    unsigned int trigPerLineExp; //trigger per line expected
    unsigned int trigPerLineAct; //trigger per line Actual

};
struct structScan
{
    float scanHeight;
    float scanWidth;
    float scanInterval;
    QString PRF;
    QString Current;
    short scansPerInspection;
    bool useCurrentResults;
    bool enableTT;
    bool enableAS; //abstract Scanning
    int scanSpeedIndex;
    QVector<structScanLineAS> scanLinesASvec; //Robot based project uses this information in data processor to place the incoming data.
    QVector<QVector<structRobTarget>> scanGrid;
    QVector<unsigned char> scLnPtCntArr;
    structScanSetting scanSpeedArr [7];
    structRobTarget startScanPosRob;
    QPointF startScanPos;
    QPointF startRectPos;
    QPointF rightBottomRectPos;
    QPoint preScanPos;
    int ldvStandOffDistance;
    int NumOfGenTrig;
    bool enableDepthControl;
    bool enableOrientControl;
};

struct structResult
{
    bool filtPass1en;
    short filterType;
    short filterRadius;
    short filterItr;
    bool filtPass2en;
    short filterType2;
    short filterRadius2;
    short filterItr2;
    int vtwamStartFr[MAXVTWAMRANGES];
    int vtwamEndFr[MAXVTWAMRANGES];
    int vtwamRangeNo;
};

struct structFilterTabs
{
    double a2=0,a3=0,a4=0,a5=0,b1=0,b2=0,b3=0,b4=0,b5=0;
    int startFreqBandPassKhz;
    int stopFreqBandPassKhz;
    int SamplingFreqMhz;
} ;
struct filtParStruct{
    short lowPassCut;
    short hiPassCut;
    short gain;
};

struct settingsStruct{
    short ldvRange;
    short samplingFreq;
    short chNum;
    int trigDelay;
    int daqVoltage;
    filtParStruct filtPar[4];
};

struct structLms
{
    QString SOD;
    double laserStep;
    double initXabs;
    double initYabs;
    double currentXabs;
    double currentYabs;
    double currentX;
    double currentY;
    double currentZ;
    bool diplayArea;
    int trigMulFac;
    bool loadCorrectionFile=false;
};
#endif // STRUCTDEF_H

