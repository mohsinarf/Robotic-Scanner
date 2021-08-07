#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <Qstring>
#include <QFile>
#include "structDef.h"
#include "laserController.h"
#include "ldvController.h"
#include "daqControllerS.h"
#include "dataProcessor.h"
#include "plot.h"
#include "spectrogram.h"
#include "stageController.h"
#include "robotController.h"
#include "qtextedit.h"
#include "dialogenlarge.h"
#include <QScrollArea>

#include "vfw.h"
#include "bandpassController.h"
#include "specimageprocessor.h"
#include "lmsController.h"

class MySlider: public QSlider
{
    Q_OBJECT
public:
    int xAtpress;
    QFrame *line;
    MySlider(QWidget *parent = 0);
    void mouseReleaseEvent ( QMouseEvent * event );
    void mousePressEvent ( QMouseEvent * event );

signals:
    void mouseMidButton(bool press,int currentVal); // press == 0 ->released
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

//Combo Box Dbs
QStringList comboScanInterval(QStringList listScanInterval)
{
    listScanInterval.append("0.125");
    listScanInterval.append("0.25");
    listScanInterval.append("0.45");
    listScanInterval.append("0.50");
    listScanInterval.append("1.00");
    listScanInterval.append("3.00");

    return listScanInterval;
}
/***Laser PRF******/
/*
QStringList comboLaserPRF(QStringList listLaserPRF)
{
    listLaserPRF.append("50");
    listLaserPRF.append("100");
    listLaserPRF.append("200");
    listLaserPRF.append("250");
    listLaserPRF.append("300");
    listLaserPRF.append("350");
    listLaserPRF.append("500");
    listLaserPRF.append("600");
    listLaserPRF.append("700");
    listLaserPRF.append("800");
    listLaserPRF.append("1000");
    listLaserPRF.append("1250");
    listLaserPRF.append("2500");

    return listLaserPRF;
}
*/
/*** Laser current ******/
QStringList comboLaserCurrent(QStringList listLaserCurrent)
{
    QString sCurrent;
    for (int i = 0; i < 30; i++)
    {
        float current = 13.5 + i*0.5;
        sCurrent = QString::number(current);
        listLaserCurrent.append(sCurrent);
    }
    return listLaserCurrent;

//    QString sCurrent;
//    for (int i = 0; i < 101; i=i+10)
//    {
//        sCurrent = QString::number(i);
//        listLaserCurrent.append(sCurrent);
//    }
//    return listLaserCurrent;
}

QStringList comboLdvRange(QStringList listLdvRange)
{
    listLdvRange.append("5");
    listLdvRange.append("10");
    listLdvRange.append("20");
    listLdvRange.append("50");
    listLdvRange.append("200");
    return listLdvRange;
}

QStringList comboDaqCh(QStringList listDaqCh,int SkipChNum)
{
    for (int i = 0;i<=7;i++)
    {
        if (i==SkipChNum) continue;
        listDaqCh.append(QString::number(i));
    }

    return listDaqCh;
}

QStringList comboDaqConfig(QStringList listDaqConfig)
{

    listDaqConfig.append("15");
    listDaqConfig.append("60");

    return listDaqConfig;
}

QStringList comboFilterBandStart(QStringList FilterBandStart)
{
    FilterBandStart.append("50");
    return FilterBandStart;
}

QStringList comboFilterBandStop(QStringList FilterBandStop)
{
    FilterBandStop.append("200");
    return FilterBandStop;
}

QStringList comboPlaySpeed(QStringList PlaySpeed)
{
    PlaySpeed.append("x1");
    PlaySpeed.append("x2");
    PlaySpeed.append("x4");
    PlaySpeed.append("x8");
    PlaySpeed.append("x16");
    PlaySpeed.append("x32");

    return PlaySpeed;
}

QStringList comboMaterials(QStringList list_InspectMaterial)
{
    list_InspectMaterial.append("-None-");
    list_InspectMaterial.append("Aluminum");
    list_InspectMaterial.append("Graphite");
    list_InspectMaterial.append("Honeycomb");

    return list_InspectMaterial;
}

QStringList comboThicknessAl(QStringList listThickness)
{
    listThickness.append("0.5");
    listThickness.append("1.0");
    listThickness.append("1.5");
    listThickness.append("2.0");
    listThickness.append("2.5");
    listThickness.append("3.0");
    listThickness.append("4.0");
    listThickness.append("5.0");
    listThickness.append("6.0");
    listThickness.append("8.0");

    return listThickness;
}

QStringList comboThicknessCfrp(QStringList listThickness)
{
    float num;
    for (int i = 0;i<19;i++)
    {
        num = 0.6+i*0.3;
        if (num == 3.0)
            listThickness.append("3.0");

        else if (num == 6.0)
            listThickness.append("6.0");

        else
        listThickness.append(QString::number(num,'g',2));
    }

    listThickness.append("7.5");
    return listThickness;
}

QStringList comboRef(QStringList listRef)
{
    listRef.append("-None-");
    listRef.append("16A11033-7");
    listRef.append("16A11033-9");
    listRef.append("16A11033-11");
    listRef.append("16A11033-13");
    listRef.append("16A11033-15");
    listRef.append("16A11033-17");
    listRef.append("16A11033-109");
    listRef.append("16A11039-7");
    listRef.append("16A11039-9");


    return listRef;
}

QStringList comboTotalScans(QStringList listTotalScans)
{
    for (int i = 1;i<=10;i++)
        listTotalScans.append(QString::number(i));

    return listTotalScans;
}

private slots:
    void updateProgressBar_mainwindowSlot(int);

    void endProgressBar_mainwindowSlot();

    void updateStatusBar_mainwindowSlot(QString);

    void on_pushButtonDaqSet_clicked();

    void on_pushButtonLdvAutoFocus_clicked();

    void on_lineEditScanHeight_editingFinished();

    void on_lineEditScanWidth_editingFinished();

    void playPauseResult(bool play);
    void playPauseResultSubband(bool play);

    void incrSlider();
    void incrSliderSubband();

    void on_enumPlaySpeed_currentIndexChanged(int index);

    void scanFinished_main();

    void on_pushButtonProcessFilter_clicked();

    void updateVtwamInputs(bool press, int frValue);

    void on_pushButtonProcessVtwam_clicked();

    void on_enumPlaySpeedSubband_currentIndexChanged(int index);

    void on_pushButtonSetpos_released();

    void on_pushButtonLaserConfigPrfCurr_clicked();

    void on_pushButtonInit_clicked();

    void on_groupBoxFilterStep2_toggled(bool arg1);

    void on_groupBoxFilterStep1_toggled(bool arg1);

    void on_lineEditXpos_editingFinished();

    void on_lineEditZPos_editingFinished();

    void on_radioButtonSubband1_toggled(bool checked);

    void on_radioButtonSubband2_toggled(bool checked);

    void on_radioButtonSubband3_toggled(bool checked);

    void on_pushButtonLaserControl_toggled(bool checked);
    void Stop();
    void deviceConnect();

    void on_pushButtonLdvAutoFocus_2_clicked();

    void on_pushButtonCapture_clicked();

    void on_actionAbout_triggered();

    void setlabelFrame(int frameNumber);
    void setlabelFrameSubband(int frameNumber);

    void saveSetting(bool defaultFile=false);
    bool loadSetting(bool defaultFile=false);
    void initDoneMsgBox();

    void on_pushButtonQuit_clicked();
    void settingBeforeNewScan();

    void on_pushButtonEnlarge_clicked();
    void resizeToNormal();
    void populatePrfList(int intervalIndex);
    void hiLaserPwrWarning();

    void on_actionPulse_Energy_Table_triggered();

    void on_pushButtonVtwamAddRange_clicked();

    void on_pushButtonVtwamClearRanges_clicked();
    void printVtwamInfo();

    void on_pushButtonDaqConfig_pressed();

    void on_checkBoxEnableMultiBand_toggled(bool checked);

    void on_enumMultiBandSettingLev1_currentIndexChanged(int index);

    void on_pushButtonFilterConfig_pressed();

    void setSettingStruct();

    void UpdateDaqParStruct();
    void UpdateLmsParStruct();

    void windowLayOutUpdate(bool);

    void on_pushButtonProcessXCor_clicked();

    void on_actionBand_Divider_Scenario_triggered();

    void loadDataMain(bool);

    void printDataSpec();

    void on_checkBoxAbstractShapeScanning_toggled(bool checked);

    void saveDataMain(bool status);

    void on_enumSpeed_activated(int index);

    void on_pushButtonSetpos_LMS_released();

    void on_checkBoxDisplayScanArea_clicked(bool checked);

    void on_spinBoxLmsStep_editingFinished();

    void on_checkBox_clicked(bool checked);



    void on_spinBoxTrigStep_editingFinished();

    void on_lineEditPixelPeriod_editingFinished();

    void on_LineEditTrigPeriodCompress_editingFinished();

    void on_pushButtonConfigTrigCompressPixelPeriod_clicked();



    void on_pushButtonGenerateCorFile_clicked();

    void on_pushButtonConfigCorFileParameters_clicked();

    void on_checkBoxEnDepthControl_toggled(bool checked);

    void on_checkBoxEnOrientControl_toggled(bool checked);

    void configOsciPlotForKinect(bool);

    void on_pushButtonGenerateCorFileWRob();

    void on_pushButtonCalibrateOrient_clicked();

    void on_pushButtonGenerateCorFileWRob_toggled(bool checked);

    void on_pushButtonCalibrateOrientY_clicked();

    void lmsScanMain();

private:
    Ui::MainWindow *ui;

    structLms lmsInfo;
    lmsController *lms;

    structScan scanInfo;
    dataProcessor *dataProc;

    laserController *laser;

    ldvController *ldv;

    structDaq daqInfo;
    daqController *daq;

    Plot *qwtPlotOsc;
    Plot *qwtPlotResult;
    spectrogram *qwtSpectrogram,*qwtSpectrogram2;
    spectrogram *qwtSpectrogramSubband[NUMOFBANDS];

    structResult resultInfo;

    QTimer *mainTimer,*mainTimerSubband;
    QString progDataPath;
    stageController *stage;
    robotController *robot;

    DialogEnlarge *enlargeResultDlgn;
    QWidget* subbandWidget;
    bandpassController* filter;
    settingsStruct settingArr[12];
    short settingNumber;
    short maxScanRangemm;

    short sliderIncVal;

    short scansDone;
    bool stopPressed;
    bool isHighRange;

    //pars for lms cor file generation
    int xAxisRightLimitBits=0, xAxisLeftLimitBits=0;
    int yAxisUpLimitBits=0, yAxisDownLimitBits=0;
    int xOriginBits=0;
    int yOriginBits=0;
    int CorFileSize=10; // a cor file of size CorFileSizexCorFileSize is generated

    specImageProcessor *specImageProc;
    QWidget *widgetInteractiveAS;

    void LoadComboBoxLists();
    void InitSettingPars();
    void laserError();

    void UpdateSettingsStruct();
    void UpdateScanParStruct();
    void updateResultParStruct();
    void setLdvRange();
    float boundScanPars(float enteredPar);

    signals:
    void postProcessingFilteringRequired();
    void postProcessingVtwamRequired(QString);


};

#endif // MAINWINDOW_H
