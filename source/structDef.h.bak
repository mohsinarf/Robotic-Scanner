#ifndef STRUCTDEF_H
#define STRUCTDEF_H

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
    unsigned short chMap;
    int daqVoltage;
    int daqTrigDelay;
    short osciChan;
    QString settingStr;
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
#endif // STRUCTDEF_H
