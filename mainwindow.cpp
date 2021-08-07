#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    scanInfo.enableTT           = false; // true for tt and false for pe-upi
    setSettingStruct();
    ui->setupUi(this);

    this->setWindowTitle(QStringLiteral("Robotic Pulse Echo Ultrasonic Propogation Imaging System "));

    scanInfo.scanSpeedIndex = 0;
    //progress bar
    ui->statusBar->addPermanentWidget(ui->progressBar,0);
    ui->progressBar->hide();
    ui->progressBar->setTextVisible(false);
    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(100);

    //hide&seek
    ui->mainToolBar->hide();
    subbandWidget = ui->tabWidget->widget(1);
    ui->lineEditDaqTrigDelay->hide();
    ui->labelTrigDelay->hide();
    ui->labelRefWarning->hide();
    //ui->checkBoxRealTimeMedian->hide();
    ui->lineEditDaqVoltage->hide();
    ui->pushButtonDaqConfig->hide();
    ui->labelScanHeight_2->hide();
    ui->checkBoxConAngData->hide();
    ui->enumOsciChannel->hide();
    ui->labelScanInterval_4->hide();
    //ui->checkBoxEnableMultiBand->hide();

    //Tabwidget
    this->setCentralWidget(ui->tabWidget);

    //ui->tabWidget->removeTab(1); // remove the sub-band tab

    progDataPath = "D:\\FF_PE_UPI_Data";
    //Combo boxes
    LoadComboBoxLists();

    //Isntantiate different controllers
    laser       = new laserController((structScan *)&scanInfo);
    ldv         = new ldvController((structDaq *)&daqInfo);
    daq         = new daqController((structDaq *)&daqInfo,(structScan *)&scanInfo);
    stage       = new stageController((structScan *)&scanInfo);
    filter      = new bandpassController();
    robot       = new robotController((structScan *)&scanInfo);
    lms         = new lmsController((structLms *)&lmsInfo,(structScan *)&scanInfo);


    //plots
    qwtPlotOsc = new Plot(ui->widgetOscilloscope,false,"Oscilloscope",(structDaq *)&daqInfo);
    qwtPlotOsc->setObjectName(QStringLiteral("qwtPlotOsc"));
    qwtPlotOsc->setGeometry(QRect(0, 0, 1100, 300));

    qwtPlotResult = new Plot(ui->widgetResultTime,true,NULL,(structDaq *)&daqInfo);
    qwtPlotResult->setObjectName(QStringLiteral("qwtPlotResult"));
    qwtPlotResult->setGeometry(QRect(0, 0, 1300, 210));
    qwtPlotResult->updateGeometry();

    //spects
    qwtSpectrogram = new spectrogram(ui->widgettResultSpect,(structScan *)&scanInfo,0, 0, SPECTBASESIZE+150, SPECTBASESIZE,NULL,1,650,1500);
    qwtSpectrogram->setObjectName(QStringLiteral("ResultSpectrogram"));


    qwtSpectrogram2 = new spectrogram(ui->widgettResultSpect,(structScan *)&scanInfo,0, 0, SPECTBASESIZE+150, SPECTBASESIZE,NULL,0);
    qwtSpectrogram2->setObjectName(QStringLiteral("ScreenShotSpectrogram"));

    for (int i=0;i<NUMOFBANDS;i++)
    {
        QString name = "Sub-band  " + QString::number(i+1);

        //qwtSpectrogramSubband[i] = new spectrogram(ui->widgetSubbandSpect,(structScan *)&scanInfo,
        //                                           0,0,SPECTBASESIZE+150,SPECTBASESIZE,name, NUMOFBANDS-1-i,860,1920);

        qwtSpectrogramSubband[i] = new spectrogram(ui->widgetSubbandSpect,(structScan *)&scanInfo,
                                                   0,0,SPECTBASESIZE+150,SPECTBASESIZE,name, NUMOFBANDS-1-i,780,1600);


        qwtSpectrogramSubband[i]->setObjectName(name);
        connect(ui->dial_intensity_Subband,SIGNAL(valueChanged(int)),qwtSpectrogramSubband[i],SLOT(setIntensity(int)));
    }

    for (int i=0;i<(NUMOFBANDS-1);i++)
        connect(qwtSpectrogramSubband[i],SIGNAL(placeSlave(int,int,int,int,bool)),qwtSpectrogramSubband[i+1],SLOT(placeSlaveslot(int,int,int,int,bool)));

    dataProc    = new dataProcessor((structDaq *)&daqInfo,(structScan *)&scanInfo, (structResult *) &resultInfo,
                                    qwtSpectrogram,qwtSpectrogram2,progDataPath,qwtSpectrogramSubband,NULL);
    //timers
    mainTimer = new QTimer(this);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(incrSlider()));

    mainTimerSubband = new QTimer(this);
    connect(mainTimerSubband, SIGNAL(timeout()), this, SLOT(incrSliderSubband()));

    //connections

    //laser
    connect(laser,SIGNAL(updateProgressBar_laserControllerSignal(int)),this,SLOT(updateProgressBar_mainwindowSlot(int)));
    connect(laser,SIGNAL(updateStatusBar_laserControllerSignal(QString)),this,SLOT(updateStatusBar_mainwindowSlot(QString)));

    //dataproc Incoming Signals
    //controllers
    connect(this,SIGNAL(postProcessingVtwamRequired(QString)),dataProc,SLOT(postProcessingVtwamRequested(QString)));
    connect(this,SIGNAL(postProcessingFilteringRequired()),dataProc,SLOT(postProcessingFilteringRequested()));
    connect(daq,SIGNAL(scanFinished(int)),dataProc,SLOT(scanFinished(int)));
    connect(daq,SIGNAL(newWfmReadyForCopy(short *,int)),dataProc,SLOT(newWfmCopyToArray_slot(short *,int)));
    connect(qwtSpectrogram,SIGNAL(pointToPlot(int)),dataProc,SLOT(plotResultTime(int)));

    //dataproc Outgoing Signals
    connect(dataProc,SIGNAL(updateProgressBar_dataProcessorSignal(int)),this,SLOT(updateProgressBar_mainwindowSlot(int)));
    connect(dataProc,SIGNAL(updateStatusBar_dataProcessorSignal(QString)),this,SLOT(updateStatusBar_mainwindowSlot(QString)));
    connect(dataProc,SIGNAL(updateResultTimePlot(short *, QString)),qwtPlotResult,SLOT(UpdateCurve(short *, QString)));
    connect(dataProc,SIGNAL(wfmCopyDone_sig(int)),daq,SLOT(wfmCopyDone(int)));

    //gui controls
    connect(ui->pushButtonLoadData,SIGNAL(clicked(bool)),this,SLOT(loadDataMain(bool)));
    connect(ui->pushButtonSaveData,SIGNAL(clicked(bool)),this,SLOT(saveDataMain(bool)));

    connect(ui->groupBoxFilter,SIGNAL(toggled(bool)),dataProc,SLOT(selectDisplayBuffer(bool)));
    connect(ui->groupBoxFilter,SIGNAL(toggled(bool)),ui->groupBoxFilterStep1,SLOT(setChecked(bool)));

    connect(ui->horizontalSliderFrame,SIGNAL(valueChanged(int)),dataProc, SLOT(setframeNum(int)));
    //connect(ui->pushButtonCapture, SIGNAL(clicked()),dataProc,SLOT(saveScreenshot()));
    //connect(ui->pushButtonCapture, SIGNAL(clicked()),ui->dial_intensity_2,SLOT(hide()));
    connect(ui->pushButtonSaveMovie, SIGNAL(clicked()),dataProc,SLOT(saveMovie()));

    //daq
    connect(daq,SIGNAL(updateProgressBar_daqControllerSignal(int)),this,SLOT(updateProgressBar_mainwindowSlot(int)));
    connect(daq,SIGNAL(updateStatusBar_daqControllerSignal(QString)),this,SLOT(updateStatusBar_mainwindowSlot(QString)));
    connect(daq,SIGNAL(scanFinished(int )),this,SLOT(scanFinished_main()));
    connect(daq,SIGNAL(updatePlotOsci(short*)),qwtPlotOsc,SLOT(UpdateCurve(short*)));

    //navigator
    connect(ui->pushButtonJogzp,SIGNAL(pressed()),robot, SLOT(Jogzp()));
    connect(ui->pushButtonJogzn,SIGNAL(pressed()),robot, SLOT(Jogzn()));
    connect(ui->pushButtonJogxp,SIGNAL(pressed()),robot, SLOT(Jogxp()));
    connect(ui->pushButtonJogxn,SIGNAL(pressed()),robot, SLOT(Jogxn()));
    connect(ui->pushButtonJogyp,SIGNAL(pressed()),robot, SLOT(Jogyp()));
    connect(ui->pushButtonJogyn,SIGNAL(pressed()),robot, SLOT(Jogyn()));

    connect(ui->pushButtonJogzp,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonJogzn,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonJogxp,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonJogxn,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonJogyp,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonJogyn,SIGNAL(released()),robot, SLOT(jogStop()));
    connect(ui->pushButtonUploadScanGrid,SIGNAL(pressed()),robot, SLOT(uploadScanGrid()));

    connect(ui->pushButtonOrigin,SIGNAL(clicked(bool)),robot, SLOT(home()));
    connect(ui->pushButtonServoStop,SIGNAL(pressed()),this, SLOT(Stop()));
    connect(ui->pushButtonServoStop2,SIGNAL(pressed()),this, SLOT(Stop()));
    connect(ui->pushButtonServoStop3,SIGNAL(pressed()),this, SLOT(Stop()));
    connect(ui->pushButtonGetpos,SIGNAL(pressed()),robot, SLOT(getPos()));

    connect(robot,SIGNAL(Xpos(QString)),ui->lcdNumberXpos, SLOT(display(QString)));
    connect(robot,SIGNAL(Ypos(QString)),ui->lcdNumberYpos, SLOT(display(QString)));
    connect(robot,SIGNAL(Zpos(QString)),ui->lcdNumberZpos, SLOT(display(QString)));

    connect(ui->pushButtonLoadMountPos,SIGNAL(clicked(bool)),robot, SLOT(loadMountPos()));
    connect(ui->pushButtonBracketAttachPos,SIGNAL(clicked(bool)),robot, SLOT(bracketAttachPos()));
    connect(ui->pushButtonTurnOffPos,SIGNAL(clicked(bool)),robot, SLOT(turnOffPos()));

    //navigator LMS
    connect(ui->pushButtonJogyp_LMS,SIGNAL(pressed()),lms, SLOT(lmsMoveyp()));
    connect(ui->pushButtonJogyn_LMS,SIGNAL(pressed()),lms, SLOT(lmsMoveyn()));
    connect(ui->pushButtonJogxp_LMS,SIGNAL(pressed()),lms, SLOT(lmsMovexp()));
    connect(ui->pushButtonJogxn_LMS,SIGNAL(pressed()),lms, SLOT(lmsMovexn()));
    connect(ui->pushButtonOrigin_LMS,SIGNAL(clicked(bool)),lms, SLOT(lmsInitialize()));
    //connect(ui->lineEditStandofDistance,SIGNAL(editingFinished()),lms, SLOT(lmsInitialize()));
    //connect(ui->lineEditStandofDistance,SIGNAL(editingFinished()),this, SLOT(UpdateLmsParStruct()));
    connect(ui->lineEditScanHeight,SIGNAL(editingFinished()),lms, SLOT(lmsInitialize()));
    connect(ui->lineEditScanWidth,SIGNAL(editingFinished()),lms, SLOT(lmsInitialize()));
    connect(ui->checkBoxDisplayScanArea,SIGNAL(clicked()),lms, SLOT(lmsDisplayArea()));
    connect(lms,SIGNAL(Xpos(QString)),ui->lcdNumberXpos_LMS, SLOT(display(QString)));
    connect(lms,SIGNAL(Ypos(QString)),ui->lcdNumberYpos_LMS, SLOT(display(QString)));
    connect(lms,SIGNAL(Zpos(QString)),ui->lcdNumberZpos_LMS, SLOT(display(QString)));
    connect(ui->pushButtonLmsScan_3,SIGNAL(clicked(bool)),this,SLOT(lmsScanMain()));
    connect(ui->pushButtonTrigCount_3,SIGNAL(clicked(bool)),lms,SLOT(getTrigCount()));
    connect(ui->pushButtonExtStart_3,SIGNAL(clicked(bool)),lms,SLOT(extStart()));

    connect(ui->pushButtonLMSDebug,SIGNAL(clicked()),lms, SLOT(LmsDebug()));

    //result tab
    connect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram,SLOT(setIntensity(int)));
    //connect(ui->horizontalSliderFrame,SIGNAL(valueChanged(int)),ui->labelFrame, SLOT(setNum(int)));
    connect(ui->horizontalSliderFrame,SIGNAL(valueChanged(int)),this, SLOT(setlabelFrame(int)));
    connect(ui->horizontalSliderFrame,SIGNAL(valueChanged(int)),qwtPlotResult, SLOT(updateVertMarker(int)));
    connect(ui->horizontalSliderFrame,SIGNAL(mouseMidButton(bool,int)),this, SLOT(updateVtwamInputs(bool,int)));
    //connect(ui->dial_intensity_2,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    ui->dial_intensity_2->hide();
    ui->labelVtwamRangeTitle->hide();
    connect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    connect(ui->pushButtonPlayPause,SIGNAL(toggled(bool)),this, SLOT(playPauseResult(bool)));
    connect(qwtSpectrogram,SIGNAL(placeSlave(int,int,int,int,bool)),qwtSpectrogram2,SLOT(placeSlaveslot(int,int,int,int,bool)));

    //subband tab
    connect(ui->horizontalSliderSubband,SIGNAL(valueChanged(int)),dataProc, SLOT(setframeNumSubband(int)));
    connect(ui->horizontalSliderSubband,SIGNAL(valueChanged(int)),this, SLOT(setlabelFrameSubband(int)));
    connect(ui->pushButtonPlayPauseSubband,SIGNAL(toggled(bool)),this, SLOT(playPauseResultSubband(bool)));

    connect(ui->pushButtonCaptureVtwam,SIGNAL(clicked(bool)),dataProc, SLOT(saveScreenshotVtwam()));
    connect(ui->pushButtonCaptureXcor,SIGNAL(clicked(bool)),dataProc, SLOT(saveScreenshotXcor()));

    connect(ui->pushButtonSaveSettings,SIGNAL(pressed()),this,SLOT(saveSetting()));
    connect(ui->pushButtonLoadSettings,SIGNAL(pressed()),this,SLOT(loadSetting()));
    connect(ui->checkBoxGreyscale,SIGNAL(toggled(bool)),this->qwtSpectrogram,SLOT(toggleUWPIGreyScale(bool)));
    connect(ui->checkBoxGreyscale,SIGNAL(toggled(bool)),this->qwtSpectrogram2,SLOT(toggleUWPIGreyScale(bool)));

    connect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
    connect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));

    connect(ldv,SIGNAL(ldvSignalLevel(int)),ui->ldvSignalLevel,SLOT(setValue(int)));

    connect(ui->enumScanInterval,SIGNAL(currentIndexChanged(int)),this,SLOT(populatePrfList(int)));

    enlargeResultDlgn = new DialogEnlarge(this);
    connect(this->enlargeResultDlgn,SIGNAL(finished(int)),this,SLOT(resizeToNormal()));

    connect(enlargeResultDlgn->giveSlider(),SIGNAL(valueChanged(int)),this->qwtSpectrogram,SLOT(updateAxisXY(int)));
    connect(ui->checkBoxRealTimeMedian,SIGNAL(toggled(bool)),this->dataProc,SLOT(setRlTmMdFlt(bool)));
    connect(this,SIGNAL(destroyed(QObject*)),this->enlargeResultDlgn,SLOT(close()));
    connect(ui->checkBoxBoostSpeed,SIGNAL(toggled(bool)),dataProc,SLOT(setBoostSpeedEn(bool)));
    connect(ui->enumOsciChannel,SIGNAL(currentIndexChanged(int)),this,SLOT(UpdateDaqParStruct()));
    widgetInteractiveAS = new QWidget();
    specImageProc = new specImageProcessor((structDaq *)&daqInfo ,
                                           (structScan *)&scanInfo,  lms,"D:\\OpenCV\\Sources\\samples\\data\\lena.jpg",
                                           widgetInteractiveAS,robot,ui->labelProcessed);
    //connect(specImageProc,SIGNAL(processedImage(QPixmap)),ui->labelProcessed,SLOT(setPixmap(QPixmap)));

    connect(ui->pushButtonZoomIn,SIGNAL(pressed()),specImageProc,SLOT(incZoom()));
    connect(ui->pushButtonZoomOut,SIGNAL(pressed()),specImageProc,SLOT(decZoom()));

    connect(ui->sliderCannyTh1,SIGNAL(valueChanged(int)),ui->labelCanyTh1Val,SLOT(setNum(int)));
    connect(ui->sliderCannyTh2,SIGNAL(valueChanged(int)),ui->labelCanyTh2Val,SLOT(setNum(int)));
    connect(ui->sliderBlurKernel,SIGNAL(valueChanged(int)),ui->labelBlurKernel,SLOT(setNum(int)));
    connect(ui->sliderCloseKernel,SIGNAL(valueChanged(int)),ui->labelCloseKernel,SLOT(setNum(int)));
    connect(ui->sliderCloseItr,SIGNAL(valueChanged(int)),ui->labelCloseItr,SLOT(setNum(int)));

    connect(ui->sliderCannyTh1,SIGNAL(valueChanged(int)),specImageProc,SLOT(setCannyThLo(int)));
    connect(ui->sliderCannyTh2,SIGNAL(valueChanged(int)),specImageProc,SLOT(setCannyThHi(int)));
    connect(ui->sliderBlurKernel,SIGNAL(valueChanged(int)),specImageProc,SLOT(setBlurKerSz(int)));
    connect(ui->sliderCloseKernel,SIGNAL(valueChanged(int)),specImageProc,SLOT(setCloseKerSz(int)));
    connect(ui->sliderCloseItr,SIGNAL(valueChanged(int)),specImageProc,SLOT(setCloseKerItr(int)));

    connect(ui->checkBoxRefreshCamera,SIGNAL(toggled(bool)),specImageProc,SLOT(enASScanUpdateTimer(bool)));
    connect(ui->pushButtonCalibrate,SIGNAL(released()),specImageProc,SLOT(depthCalibrate()));
    connect(ui->pushButtonCalibrateXY,SIGNAL(released()),specImageProc,SLOT(xyCalibrate()));


    connect(specImageProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
    connect(specImageProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));
    connect(specImageProc,SIGNAL(plotGraph(short *, QString)),qwtPlotOsc,SLOT(UpdateCurve(short *, QString)));
    //connect(ui->lineEditUpdateTime,SIGNAL(),specImageProc,SLOT(setCloseKerItr(int)));

    connect(ui->pushButtonSaveContImage,SIGNAL(released()),specImageProc,SLOT(save()));
    connect(ui->pushButtonChangeImageType,SIGNAL(released()),specImageProc,SLOT(changeImageType()));
    connect(ui->checkBoxRobotControllerSelect,SIGNAL(toggled(bool)),robot, SLOT(setIsVcCont(bool)));
    connect(ui->pushButtonGenTrigs,SIGNAL(pressed()),robot,SLOT(genTrig()));
    connect(ui->pushButtonGenTrigs,SIGNAL(pressed()),lms,SLOT(genTrig()));

    connect(ui->pushButtonFlipYSubBand,SIGNAL(pressed()),qwtSpectrogramSubband[0],SLOT(updateAxisXY()));

    connect(ui->lineImageRefreshInterval,SIGNAL(textChanged(QString)),specImageProc,SLOT(setRefreshInterval(QString)));

    connect(ui->pushButtonSaveKinectData,SIGNAL(released()),specImageProc,SLOT(saveKinectData()));
    connect(ui->pushButtonLoadKinectData,SIGNAL(released()),specImageProc,SLOT(loadKinectData()));


    connect(ui->sliderNormSmoothingSize,SIGNAL(valueChanged(int)),ui->labelNormSmoothingSize,SLOT(setNum(int)));
    connect(ui->slidersetMaxDepthChangeFactor,SIGNAL(valueChanged(int)),ui->labelsetMaxDepthChangeFactor,SLOT(setNum(int)));
    connect(ui->sliderThetaMax,SIGNAL(valueChanged(int)),ui->labelThetaMax,SLOT(setNum(int)));
    connect(ui->sliderPhiMax,SIGNAL(valueChanged(int)),ui->labelPhiMax,SLOT(setNum(int)));


    connect(ui->sliderNormSmoothingSize,SIGNAL(valueChanged(int)),specImageProc,SLOT(setNormSmoothingSize(int)));
    connect(ui->slidersetMaxDepthChangeFactor,SIGNAL(valueChanged(int)),specImageProc,SLOT(setMaxDepthChangeFactor(int)));
    connect(ui->radioButtonDepthDependentSmoothing,SIGNAL(toggled(bool)),specImageProc,SLOT(setEnableDepthDependentSmoothing(bool)));
    connect(ui->pushButtonChangeNormCalcMethod,SIGNAL(released()),specImageProc,SLOT(changeNormCalcMethod()));
    connect(ui->sliderThetaMax,SIGNAL(valueChanged(int)),specImageProc,SLOT(setThetaMaxAngle(int)));
    connect(ui->sliderPhiMax,SIGNAL(valueChanged(int)),specImageProc,SLOT(setPhiMaxAngle(int)));

    connect(ui->sliderSmoothKernelSize,SIGNAL(valueChanged(int)),specImageProc,SLOT(setSmoothKernelSize(int)));
    connect(ui->sliderSmoothSigmaColour,SIGNAL(valueChanged(int)),specImageProc,SLOT(setSmoothSigmaColour(int)));
    connect(ui->sliderSmoothSigmaSpace,SIGNAL(valueChanged(int)),specImageProc,SLOT(setSmoothSigmaSpace(int)));

    connect(ui->sliderSmoothKernelSize,SIGNAL(valueChanged(int)),ui->labelSmoothKernelSize,SLOT(setNum(int)));
    connect(ui->sliderSmoothSigmaColour,SIGNAL(valueChanged(int)),ui->labelSmoothSigmaColour,SLOT(setNum(int)));
    connect(ui->sliderSmoothSigmaSpace,SIGNAL(valueChanged(int)),ui->labelSmoothSigmaSpace,SLOT(setNum(int)));

    connect(ui->checkBoxConAngData,SIGNAL(toggled(bool)),this,SLOT(configOsciPlotForKinect(bool)));
    connect(ui->pushButtonLoadMesh,SIGNAL(pressed()),specImageProc, SLOT(loadMesh()));

    connect(ui->checkBoxEnloadKinFusCloud,SIGNAL(toggled(bool)),specImageProc, SLOT(setLoadKinFusCloud(bool)));
    connect(ui->pushButtonCalcNormals,SIGNAL(pressed()),specImageProc, SLOT(calcNormalsPcl()));

    InitSettingPars(); // The init settings should be loaded from the last settings used.
    //update the x-axis of the plots
    UpdateSettingsStruct();
    qwtPlotOsc->updateAxisScale();
    qwtPlotResult->updateAxisScale();
    qwtSpectrogram->updateAxisXY();

    qwtSpectrogramSubband[0]->updateAxisXY();


    isHighRange = false;

#if ACTUALSYSTEM
    QTimer::singleShot(50,this,SLOT(deviceConnect()));
    //Stop();
    stage->resetMovetoX();
    stage->resetMovetoZ();
    stage->clearErrorReset();
    stage->clearServoStop();

    on_pushButtonFilterConfig_pressed();
#endif

}

MainWindow::~MainWindow()
{
#if ACTUALSYSTEM
    //Stop();
#endif

    //wait for more than a second shoulkd servo stop can be reset in the stage
    //Sleep(3000);

    //delete enlargeResultDlgn;
    delete mainTimer;
    delete mainTimerSubband;
    delete laser;
    delete ldv;
    delete daq;
    delete dataProc;
    delete qwtPlotOsc;
    delete qwtSpectrogram;
    delete ui;
}

void MainWindow::lmsScanMain()
{
        this->UpdateSettingsStruct();
    if (specImageProc->calcScanGrid() == false)
        return;

    lms->lmsScan();
}

void MainWindow::configOsciPlotForKinect(bool setForKinect)
{
    short temp[2048];
    if (setForKinect)
    {
        qwtPlotOsc->UpdateCurve(temp, "Smoothed Angle");
        qwtPlotOsc->updateAxisScale(430,90);
        disconnect(daq,SIGNAL(updatePlotOsci(short*)),qwtPlotOsc,SLOT(UpdateCurve(short*)));
    }
    else
    {
        qwtPlotOsc->UpdateCurve(temp, "Oscilloscope");
        qwtPlotOsc->updateAxisScale();
        connect(daq,SIGNAL(updatePlotOsci(short*)),qwtPlotOsc,SLOT(UpdateCurve(short*)));
    }
}

void MainWindow::setSettingStruct()
{
    settingNumber = 0;

    //0-Broadband bulk wave
    settingArr[0].ldvRange              = 5;
    settingArr[0].samplingFreq          = 10;
    settingArr[0].chNum                 = NUMOFBANDS;
    settingArr[0].trigDelay             = 0;
    settingArr[0].daqVoltage            = 200;

    settingArr[0].filtPar[1].hiPassCut   = 50;
    settingArr[0].filtPar[1].lowPassCut  = 250;
    settingArr[0].filtPar[1].gain        = 25;

    settingArr[0].filtPar[2].hiPassCut   = 250;
    settingArr[0].filtPar[2].lowPassCut  = 1000;
    settingArr[0].filtPar[2].gain        = 25;

    settingArr[0].filtPar[3].hiPassCut   = 1000;
    settingArr[0].filtPar[3].lowPassCut  = 1500;
    settingArr[0].filtPar[3].gain        = 28;


    //1-Narrowband - 1
    settingArr[1].ldvRange              = 10;
    settingArr[1].samplingFreq          = 20;
    settingArr[1].chNum                 = 1;
    settingArr[1].trigDelay             = 304;
    settingArr[1].daqVoltage            = 500;

    settingArr[1].filtPar[1].hiPassCut   = 50;
    settingArr[1].filtPar[1].lowPassCut  = 250;
    settingArr[1].filtPar[1].gain        = 18;

    //2-Narrowband - 2
    settingArr[2].ldvRange              = 20;
    settingArr[2].samplingFreq          = 60;
    settingArr[2].chNum                 = NUMOFBANDS;
    settingArr[2].trigDelay             = 560;
    settingArr[2].daqVoltage            = 500;

    settingArr[2].filtPar[1].hiPassCut   = 250;
    settingArr[2].filtPar[1].lowPassCut  = 500;
    settingArr[2].filtPar[1].gain        = 18;

    settingArr[2].filtPar[2].hiPassCut   = 500;
    settingArr[2].filtPar[2].lowPassCut  = 750;
    settingArr[2].filtPar[2].gain        = 18;

    settingArr[2].filtPar[3].hiPassCut   = 750;
    settingArr[2].filtPar[3].lowPassCut  = 1000;
    settingArr[2].filtPar[3].gain        = 18;

    //3-Narrowband - 3
    settingArr[3].ldvRange              = 50;
    settingArr[3].samplingFreq          = 60;
    settingArr[3].chNum                 = NUMOFBANDS;
    settingArr[3].trigDelay             = 520;
    settingArr[3].daqVoltage            = 200;

    settingArr[3].filtPar[1].hiPassCut   = 1000;
    settingArr[3].filtPar[1].lowPassCut  = 1160;
    settingArr[3].filtPar[1].gain        = 25;

    settingArr[3].filtPar[2].hiPassCut   = 1160;
    settingArr[3].filtPar[2].lowPassCut  = 1320;
    settingArr[3].filtPar[2].gain        = 25;

    settingArr[3].filtPar[3].hiPassCut   = 1320;
    settingArr[3].filtPar[3].lowPassCut  = 1500;
    settingArr[3].filtPar[3].gain        = 25;

    //4- guided wave HighFreq
    settingArr[4].ldvRange              = 5;
    settingArr[4].samplingFreq          = 10;
    settingArr[4].chNum                 = NUMOFBANDS;
    settingArr[4].trigDelay             = 0;
    settingArr[4].daqVoltage            = 200;

    settingArr[4].filtPar[1].hiPassCut   = 50;
    settingArr[4].filtPar[1].lowPassCut  = 250;
    settingArr[4].filtPar[1].gain        = 25;

    settingArr[4].filtPar[2].hiPassCut   = 250;
    settingArr[4].filtPar[2].lowPassCut  = 1000;
    settingArr[4].filtPar[2].gain        = 25;

    settingArr[4].filtPar[3].hiPassCut   = 1000;
    settingArr[4].filtPar[3].lowPassCut  = 1500;
    settingArr[4].filtPar[3].gain        = 28;

    //5-SingleChan-0
    settingArr[5].ldvRange              = 5;
    settingArr[5].samplingFreq          = 20;
    settingArr[5].chNum                 = 1;
    settingArr[5].trigDelay             = 0;//304
    settingArr[5].daqVoltage            = 200; //500

    settingArr[5].filtPar[1].hiPassCut   = 50;
    settingArr[5].filtPar[1].lowPassCut  = 250;
    settingArr[5].filtPar[1].gain        = 18;

    //6-SingleChan-1
    settingArr[6].ldvRange              = 20;
    settingArr[6].samplingFreq          = 60;
    settingArr[6].chNum                 = 1;
    settingArr[6].trigDelay             = 576;
    settingArr[6].daqVoltage            = 500;

    settingArr[6].filtPar[1].hiPassCut   = 250;
    settingArr[6].filtPar[1].lowPassCut  = 500;
    settingArr[6].filtPar[1].gain        = 18;

    //7-SingleChan-2
    settingArr[7].ldvRange              = 20;
    settingArr[7].samplingFreq          = 60;
    settingArr[7].chNum                 = 1;
    settingArr[7].trigDelay             = 568;
    settingArr[7].daqVoltage            = 500;

    settingArr[7].filtPar[1].hiPassCut   = 500;
    settingArr[7].filtPar[1].lowPassCut  = 750;
    settingArr[7].filtPar[1].gain        = 18;

    //8-SingleChan-3
    settingArr[8].ldvRange              = 20;
    settingArr[8].samplingFreq          = 60;
    settingArr[8].chNum                 = 1;
    settingArr[8].trigDelay             = 560;
    settingArr[8].daqVoltage            = 500;

    settingArr[8].filtPar[1].hiPassCut   = 750;
    settingArr[8].filtPar[1].lowPassCut  = 1000;
    settingArr[8].filtPar[1].gain        = 18;

    //9-SingleChan-4
    settingArr[9].ldvRange               = 50;
    settingArr[9].samplingFreq           = 60;
    settingArr[9].chNum                  = 1;
    settingArr[9].trigDelay             = 536;
    settingArr[9].daqVoltage            = 200;

    settingArr[9].filtPar[1].hiPassCut   = 1000;
    settingArr[9].filtPar[1].lowPassCut  = 1166;
    settingArr[9].filtPar[1].gain        = 25;

    //10-SingleChan-5
    settingArr[10].ldvRange               = 50;
    settingArr[10].samplingFreq           = 60;
    settingArr[10].chNum                  = 1;
    settingArr[10].trigDelay             = 528;
    settingArr[10].daqVoltage            = 200;

    settingArr[10].filtPar[1].hiPassCut   = 1166;
    settingArr[10].filtPar[1].lowPassCut  = 1332;
    settingArr[10].filtPar[1].gain        = 25;

    //11-SingleChan-6
    settingArr[11].ldvRange               = 50;
    settingArr[11].samplingFreq           = 60;
    settingArr[11].chNum                  = 1;
    settingArr[11].trigDelay             = 520;
    settingArr[11].daqVoltage            = 200;

    settingArr[11].filtPar[1].hiPassCut   = 1332;
    settingArr[11].filtPar[1].lowPassCut  = 1500;
    settingArr[11].filtPar[1].gain        = 25;
}
void MainWindow::saveSetting(bool defaultFile)
{
    QFile myfileout;

    QString captureStoragePath = progDataPath + "\\Setting\\";
    QString captureFileName;

    if (defaultFile)
    {
        captureFileName = "LastSetting.bin";
    }
    else
    {
        QDateTime now = QDateTime::currentDateTime();
        captureFileName = "Setting"+now.toString("ddMMyy_hhmmss")+".bin";
    }

    if( QDir(captureStoragePath).exists() == false)
        QDir().mkpath(captureStoragePath);

    myfileout.setFileName(captureStoragePath+captureFileName);

    if(!myfileout.open(QIODevice::WriteOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open file for writing.");
        errBox.exec();
        return;
    }
    QDataStream out(&myfileout);
    out.setByteOrder(QDataStream::LittleEndian);

    //Inspection Settings
    out<<ui->checkBoxEnableMultiBand->isChecked();
    out<<ui->enumMultiBandSettingLev1->currentIndex();
    out<<ui->enumMultiBandSettingLev2->currentIndex();

    //Scan
    out<<ui->lineEditScanWidth->text();
    out<<ui->lineEditScanHeight->text();
    out<<ui->enumScanInterval->currentIndex();
    out<<ui->enumSpeed->currentIndex();
    out<<ui->enumTotalScans->currentIndex();

    out<<ui->enumPRF->currentIndex();
    out<<ui->enumCurrent->currentIndex();


    //Navigator
    //press the get pos button
    ui->pushButtonGetpos->pressed();
    out<<QString::number(ui->lcdNumberXpos->value());
    out<<QString::number(ui->lcdNumberZpos->value());
    out<<daqInfo.daqTrigDelay;
/*
    out<<stage->scanStartPosX;
    out<<stage->scanStartPosZ;
*/
    //Data Acquisition
    //out<<ui->radioButtonLowFreq->isChecked();
    //out<<ui->radioButtonMidFreq->isChecked();
    //out<<ui->radioButtonHighFreq->isChecked();

    myfileout.close();

    if (defaultFile == false)
    {
        QMessageBox msgBox(QMessageBox::Information, tr("Scan Setting"),tr("Successfully saved settings in \n")+captureStoragePath+captureFileName, 0, this);
        msgBox.exec();
        return;
    }
}

bool MainWindow::loadSetting(bool defaultFile)
{
    QString FileName;
    QFile myfilein;
    int intTemp;
    float floatTemp;
    bool boolTemp;
    QString stringTemp;

    if (defaultFile)
    {
        FileName = progDataPath + "\\Setting\\LastSetting.bin";
    }
    else
    {
        QFileDialog *fd = new QFileDialog;
        //QTreeView *tree = fd->findChild <QTreeView*>();
        //tree->setRootIsDecorated(true);
        //tree->setItemsExpandable(true);
        fd->setFileMode(QFileDialog::ExistingFile);
        fd->setViewMode(QFileDialog::Detail);
        fd->setDirectory(progDataPath+"\\Setting");
        if (fd->exec())
        {
            FileName = fd->selectedFiles()[0];
            qDebug()<<FileName;
        }
        else
        {
            qDebug()<<"Can't open the Setting file.";
            return false;
        }
    }

    myfilein.setFileName(FileName);

    if(!myfilein.open(QIODevice::ReadOnly))
    {
        //QMessageBox msgBox(QMessageBox::Critical, tr("File Error"),tr("Could'nt open parameter file for data."), 0);
        //msgBox.exec();
        qDebug()<<"Can't open the Setting file.";
        return false;
    }
    QDataStream in(&myfilein);
    in.setByteOrder(QDataStream::LittleEndian);

    // Inspection Settings
    in>>boolTemp;
    ui->checkBoxEnableMultiBand->setChecked(boolTemp);

    in>>intTemp;
    ui->enumMultiBandSettingLev1->setCurrentIndex(intTemp);

    in>>intTemp;
    ui->enumMultiBandSettingLev2->setCurrentIndex(intTemp);

    //Scan
    in>>stringTemp;
    ui->lineEditScanWidth->setText(stringTemp);
    in>>stringTemp;
    ui->lineEditScanHeight->setText(stringTemp);
    in>>intTemp;
    ui->enumScanInterval->setCurrentIndex(intTemp);
    in>>intTemp;
    ui->enumSpeed->setCurrentIndex(intTemp);
    in>>intTemp;
    ui->enumTotalScans->setCurrentIndex(intTemp);

    in>>intTemp;
    ui->enumPRF->setCurrentIndex(intTemp);
    in>>intTemp;
    ui->enumCurrent->setCurrentIndex(intTemp);

    //Navigator
    //press the get pos button
    //ui->pushButtonGetpos->pressed();


    in>>stringTemp;
    ui->lineEditXpos->setText(stringTemp);
    in>>stringTemp;
    ui->lineEditZPos->setText(stringTemp);

    in>>intTemp;
    ui->lineEditDaqTrigDelay->setText(QString::number(intTemp));

    /*
    in>>floatTemp;
    ui->lineEditXpos->setText(QString::number(floatTemp/1000));
    in>>floatTemp;
    ui->lineEditZPos->setText(QString::number(floatTemp/1000));
    */

    //Data Acquisition
    //in>>boolTemp;
    //ui->radioButtonLowFreq->setChecked(boolTemp);
    //in>>boolTemp;
    //ui->radioButtonMidFreq->setChecked(boolTemp);
    //in>>boolTemp;
    //ui->radioButtonHighFreq->setChecked(boolTemp);

    myfilein.close();

    if (defaultFile == false)
    {
        QMessageBox msgBox(QMessageBox::Information, tr("Scan Setting"),tr("Successfully loaded settings from \n ")+FileName, 0, this);
        msgBox.exec();
    }
    return true;
}

void MainWindow::setlabelFrame(int frameNumber)
{

    QString microSec = " \xC2\xB5s";
    QString labelString = "0"+microSec;
    if (daqInfo.SamplingFreq != 0)
        labelString = QString::number(((double)frameNumber/daqInfo.SamplingFreq) ,'g',3)+microSec;
    ui->labelFrame->setText(labelString);
}

void MainWindow::setlabelFrameSubband(int frameNumber)
{
    QString microSec = " \xC2\xB5s";
    QString labelString = "0"+microSec;
    if (daqInfo.SamplingFreq != 0)
        labelString = QString::number(((double)frameNumber/daqInfo.SamplingFreq) ,'g',3)+microSec;
    ui->labelFrameSubband->setText(labelString);
}
void MainWindow::deviceConnect()
{
    qwtSpectrogramSubband[0]->updateAxisXY();
    QString ldvPortName;
    QString laserComPort;
    QList<QSerialPortInfo> serialPortInfo = QSerialPortInfo::availablePorts();
    int comPortNum;
    if ((comPortNum = serialPortInfo.size()) > 4)
    {
        QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("More than expected COM ports. Please only connect LDV and laser controller."), 0, this);
        msgBox.exec();
        return;
    }
/*
    if (laser->srchPortandConnect("COM8",&laserComPort) == true)
    {
        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            if (info.portName() != "COM1" && info.portName() != laserComPort )
            {
                ldvPortName = info.portName();
                if (ldv->connectLdv(ldvPortName) == false)
                {//make an error
                    QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("LDV not found. Please reconnect and try again."), 0, this);
                    msgBox.exec();
                    return;
                }
                else
                {
                    return;
                }
            }
        }
    }
    else
        */
    {
        //QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("Laser Controller not found. Please reconnect and try again."), 0, this);
        //msgBox.exec();
        if (ldv->connectLdv("COM6") == false)
        {
            QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("LDV not found.Please reconnect and try again."), 0, this);
            msgBox.exec();
        }
        return;
    }
}

void MainWindow::updateStatusBar_mainwindowSlot(QString StatusTip)
{
    ui->statusBar->showMessage(StatusTip);
}

void MainWindow::updateProgressBar_mainwindowSlot(int Progress)
{
    ui->progressBar->show();
    ui->progressBar->setValue(Progress);
    if (Progress == 100)
    {
        QTimer::singleShot( 500, this, SLOT(endProgressBar_mainwindowSlot() ));
    }
}

void MainWindow::endProgressBar_mainwindowSlot()
{
    ui->progressBar->hide();
}

void MainWindow::populatePrfList(int intervalIndex)
{
    ui->enumPRF->clear();
    if (intervalIndex == 0) //0.10
    {
        ui->enumPRF->addItem("1000");
        ui->enumPRF->addItem("1500");
        ui->enumPRF->addItem("2000");
        ui->enumPRF->addItem("2500");
        maxScanRangemm = 300;
    }
    else if (intervalIndex == 1) //0.25
    {
        ui->enumPRF->addItem("400");
        ui->enumPRF->addItem("600");
        ui->enumPRF->addItem("800");
        ui->enumPRF->addItem("1000");
        ui->enumPRF->addItem("1200");
        ui->enumPRF->addItem("1400");
        ui->enumPRF->addItem("1600");
		ui->enumPRF->addItem("1800");
        ui->enumPRF->addItem("2000");
        maxScanRangemm = 500;
    }
    else if(intervalIndex == 2) //0.50
    {
        ui->enumPRF->addItem("200");
        ui->enumPRF->addItem("300");
        ui->enumPRF->addItem("500");
        ui->enumPRF->addItem("700");
        ui->enumPRF->addItem("800");
        maxScanRangemm = 500;
    }
    else if (intervalIndex == 3) //1.00
    {
        ui->enumPRF->addItem("100");
        ui->enumPRF->addItem("150");
        ui->enumPRF->addItem("350");
        ui->enumPRF->addItem("400");
        maxScanRangemm = 500;
    }

    ui->enumPRF->setCurrentIndex(0);
    on_lineEditScanHeight_editingFinished();
}

void MainWindow::LoadComboBoxLists()
{
    QStringList listScanInterval,listLaserPRF,listLaserCurrent,listPlaySpeed,listMaterials,
            listTotalScans,listRef;
    //scan
    ui->enumScanInterval->addItems(comboScanInterval(listScanInterval));

    //laser
    ui->enumCurrent->addItems(comboLaserCurrent(listLaserCurrent));
    ui->enumPlaySpeed->addItems(comboPlaySpeed(listPlaySpeed));

    ui->enumImageFilterIterations->addItem("1");
    ui->enumImageFilterIterations->addItem("2");

    ui->enumImageFilterType->addItem("Median");
    ui->enumImageFilterType->addItem("Spatial Avg");

    ui->enumImageFilterSize->addItem("3x3");
    ui->enumImageFilterSize->addItem("5x5");
    ui->enumImageFilterSize->addItem("7x7");

    ui->enumImageFilterIterations_2->addItem("1");
    ui->enumImageFilterIterations_2->addItem("2");

    ui->enumImageFilterType_2->addItem("Median");
    ui->enumImageFilterType_2->addItem("Spatial Avg");

    ui->enumImageFilterSize_2->addItem("3x3");
    ui->enumImageFilterSize_2->addItem("5x5");
    ui->enumImageFilterSize_2->addItem("7x7");

    ui->enumPlaySpeedSubband->addItems(comboPlaySpeed(listPlaySpeed));
    ui->enumTotalScans->addItems(comboTotalScans(listTotalScans));

    populatePrfList(ui->enumScanInterval->currentIndex());

    ui->enumMultiBandSettingLev2->addItem("Sub-band 1");
    ui->enumMultiBandSettingLev2->addItem("Sub-band 2");
    ui->enumMultiBandSettingLev2->addItem("Sub-band 3");

    ui->enumOsciChannel->addItem("Sub-band 1");
    ui->enumOsciChannel->addItem("Sub-band 2");
    #if NUMOFBANDS == 3
        ui->enumOsciChannel->addItem("Sub-band 3");
    #endif

    ui->enumSpeed->addItem("100");
    ui->enumSpeed->addItem("200");
    ui->enumSpeed->addItem("300");
    ui->enumSpeed->addItem("400");
    ui->enumSpeed->addItem("600");
    ui->enumSpeed->addItem("50");
}

void MainWindow::InitSettingPars()
{
    //toggle to load all the enums
    ui->checkBoxEnableMultiBand->setChecked(0);
    ui->checkBoxEnableMultiBand->setChecked(1);

    //laser
    if (loadSetting(true)==false)
    { //file for the last setting does not exist so set to default initial values
        ui->enumPRF->setCurrentIndex(6);
        ui->enumCurrent->setCurrentIndex(14);
        ui->lineEditScanWidth->setText("50");
        ui->lineEditScanHeight->setText("50");
        ui->enumScanInterval->setCurrentIndex(2);
        ui->lineEditDaqTrigDelay->setText("864");
    }

    ui->labelLaserStatus->setText("Laser Beam: OFF");
    //ui->pushButtonLaserControl->setEnabled(false);
    ui->pushButtonLaserControl->setText("Activate");
    ui->pushButtonLaserControl->setStatusTip("Turn-on the laser beam.");

    ui->dial_intensity_2->setMinimum(1000);
    ui->dial_intensity_2->setMaximum((1<<25)-1);
    ui->dial_intensity_2->setValue(18000);

    ui->dial_intensity->setMinimum(1000);
    ui->dial_intensity->setMaximum((1<<15)-1);
    ui->dial_intensity->setValue(5000);

    ui->horizontalSliderFrame->setMinimum(0);
    ui->horizontalSliderFrame->setMaximum(SAMPLESPERPOINT-1);
    ui->horizontalSliderFrame->setValue(0);

    ui->enumPlaySpeed->setCurrentIndex(1);

    ui->groupBoxFilter->setChecked(false);
    ui->lineEditVtwamStart->setText("0");
    ui->lineEditVtwamEnd->setText("0");

    ui->dial_intensity_Subband->setMinimum(1000);
    ui->dial_intensity_Subband->setMaximum(32768);
    ui->dial_intensity_Subband->setValue(5000);

    ui->horizontalSliderSubband->setMinimum(0);
    ui->horizontalSliderSubband->setMaximum(SAMPLESPERPOINT-1);
    ui->horizontalSliderSubband->setValue(0);

    ui->enumPlaySpeedSubband->setCurrentIndex(1);
    //make sure to toggle to hit the toggle slot
    //ui->radioButtonLowFreq->setChecked(true);
    //ui->radioButtonHighFreq->setChecked(false);
    //ui->groupBoxFilter->setChecked(true);

    ui->groupBoxFilterStep1->setChecked(false);
    ui->groupBoxFilterStep2->setChecked(false);

    QPalette* palRedText = new QPalette();
    palRedText->setColor(QPalette::ButtonText, Qt::red);

    ui->pushButtonServoStop->setPalette(*palRedText);
    ui->pushButtonServoStop2->setPalette(*palRedText);
    ui->pushButtonServoStop3->setPalette(*palRedText);

    ui->pushButtonDaqSet->setEnabled(true);

    ui->lineEditDaqVoltage->setText("500");


    UpdateSettingsStruct();

    ui->labelVtwamStart->setText(" \xC2\xB5s");
    ui->labelVtwamEnd->setText(" \xC2\xB5s");

    resultInfo.vtwamRangeNo = -1;

    ui->checkBoxRealTimeMedian->setChecked(true);
    ui->checkBoxRealTimeMedian->setChecked(false);

    ui->lineEditPixelPeriod_2->setText("1.2");
    ui->LineEditTrigPeriodCompress_2->setText("5.5");

    maxScanRangemm = 500;

    #if NUMOFBANDS == 2
        ui->radioButtonSubband3->hide();
    #endif

    ui->sliderBlurKernel->setValue(2);
    ui->sliderCannyTh1->setValue(45);
    ui->sliderCannyTh2->setValue(90);
    ui->sliderCloseKernel->setValue(9);
    ui->sliderCloseItr->setValue(9);
    ui->checkBoxAbstractShapeScanning->setChecked(false);
    ui->checkBoxAbstractShapeScanning->setChecked(true);
    ui->checkBoxRefreshCamera->setChecked(true);

    ui->checkBoxAbstractShapeScanning->setChecked(false);
    ui->checkBoxAbstractShapeScanning->setChecked(true);


    scanInfo.scanSpeedArr[0].speed          = 100;
    scanInfo.scanSpeedArr[0].pathLen        = 5;
    scanInfo.scanSpeedArr[0].trigInterval   = 5;

    scanInfo.scanSpeedArr[1].speed          = 150;
    scanInfo.scanSpeedArr[1].pathLen        = 2.5;
    scanInfo.scanSpeedArr[1].trigInterval   = 2.5;

    scanInfo.scanSpeedArr[2].speed          = 300;
    scanInfo.scanSpeedArr[2].pathLen        = 6;
    scanInfo.scanSpeedArr[2].trigInterval   = 3;

    scanInfo.scanSpeedArr[3].speed          = 400;
    scanInfo.scanSpeedArr[3].pathLen        = 8;
    scanInfo.scanSpeedArr[3].trigInterval   = 4;

    scanInfo.scanSpeedArr[4].speed          = 600;
    scanInfo.scanSpeedArr[4].pathLen        = 12;
    scanInfo.scanSpeedArr[4].trigInterval   = 6;

    scanInfo.scanSpeedArr[5].speed          = 50;
    scanInfo.scanSpeedArr[5].pathLen        = 1;
    scanInfo.scanSpeedArr[5].trigInterval   = 1;

    scanInfo.scanSpeedArr[6].speed          = 25;
    scanInfo.scanSpeedArr[6].pathLen        = 1;
    scanInfo.scanSpeedArr[6].trigInterval   = 1;

    ui->checkBoxRobotControllerSelect->setChecked(true);
    ui->checkBoxRobotControllerSelect->setChecked(false);

    ui->lineImageRefreshInterval->setText("20");

    //MR_range  185,234,438
    //LR_range  530,642,846

    //MR = 438;
    //LR = 642;

    //438 for MR
    // lets take it from GUI lateron;
    scanInfo.ldvStandOffDistance        =   234;//234,438;
    lmsInfo.trigMulFac = scanInfo.scanSpeedArr[scanInfo.scanSpeedIndex].trigInterval/scanInfo.scanInterval;

//    ui->lineEditCorFileXaxisRightLimit->setText("-50000");
//    ui->lineEditCorFileXaxisLeftLimit->setText("-280000");
//    ui->lineEditCorFileYaxisUpLimit->setText("396010");
//    ui->lineEditCorFileYaxisDownLimit->setText("206010");
//    ui->lineEditCorFileResolution->setText("13");
//    ui->lineEditXpos_LMS->setText("-180000");
//    ui->lineEditYPos_LMS->setText("206010");

    //pars used for GFRP radial specimen
//    ui->lineEditCorFileXaxisRightLimit->setText("470000");
//    ui->lineEditCorFileXaxisLeftLimit->setText("260000");
//    ui->lineEditCorFileYaxisUpLimit->setText("440000");
//    ui->lineEditCorFileYaxisDownLimit->setText("260000");
//    ui->lineEditCorFileResolution->setText("7");
//    ui->lineEditXpos_LMS->setText("380000");
//    ui->lineEditYPos_LMS->setText("260000");

    //pars used for CFRP radial + conic specimen
    ui->lineEditCorFileXaxisRightLimit->setText("260000");
    ui->lineEditCorFileXaxisLeftLimit->setText("40000");
    ui->lineEditCorFileYaxisUpLimit->setText("370000");
    ui->lineEditCorFileYaxisDownLimit->setText("0");
    ui->lineEditCorFileResolution->setText("13");
    ui->lineEditXpos_LMS->setText("150000");
    ui->lineEditYPos_LMS->setText("0");

    //pars used for lattice cut-out specimen
//    ui->lineEditCorFileXaxisRightLimit->setText("500000");
//    ui->lineEditCorFileXaxisLeftLimit->setText("200000");
//    ui->lineEditCorFileYaxisUpLimit->setText("500000");
//    ui->lineEditCorFileYaxisDownLimit->setText("260000");
//    ui->lineEditCorFileResolution->setText("7");
//    ui->lineEditXpos_LMS->setText("380000");
//    ui->lineEditYPos_LMS->setText("260000");

    //pars used for lattice cut-out specimen - bigger calibration area
//    ui->lineEditCorFileXaxisRightLimit->setText("490000");
//    ui->lineEditCorFileXaxisLeftLimit->setText("180000");
//    ui->lineEditCorFileYaxisUpLimit->setText("500000");
//    ui->lineEditCorFileYaxisDownLimit->setText("140000");
//    ui->lineEditCorFileResolution->setText("7");
//    ui->lineEditXpos_LMS->setText("380000");
//    ui->lineEditYPos_LMS->setText("140000");

    ui->pushButtonConfigTrigCompressPixelPeriod_2->click();


    ui->checkBoxEnDepthControl->setChecked(true);
    ui->checkBoxEnDepthControl->setChecked(false);

    ui->checkBoxEnOrientControl->setChecked(true);
    ui->checkBoxEnOrientControl->setChecked(false);

    ui->checkBoxEnGuidedWave->setChecked(true);
    ui->checkBoxEnGuidedWave->setChecked(false);
}

void MainWindow::updateResultParStruct()
{
    resultInfo.filtPass1en          = ui->groupBoxFilterStep1->isChecked();
    resultInfo.filterType           = ui->enumImageFilterType->currentIndex()+1;
    resultInfo.filterRadius         = ui->enumImageFilterSize->currentIndex()+1;
    resultInfo.filterItr            = ui->enumImageFilterIterations->currentIndex()+1;

    resultInfo.filtPass2en           = ui->groupBoxFilterStep2->isChecked();
    resultInfo.filterType2           = ui->enumImageFilterType_2->currentIndex()+1;
    resultInfo.filterRadius2         = ui->enumImageFilterSize_2->currentIndex()+1;
    resultInfo.filterItr2            = ui->enumImageFilterIterations_2->currentIndex()+1;

    //resultInfo.vtwamStartFr          = ui->lineEditVtwamStart->text().toInt();
    //resultInfo.vtwamEndFr            = ui->lineEditVtwamEnd->text().toInt();

    if(resultInfo.vtwamRangeNo > -1)
    {
        resultInfo.vtwamStartFr[resultInfo.vtwamRangeNo]          = qRound(daqInfo.SamplingFreq * ui->lineEditVtwamStart->text().toFloat());
        resultInfo.vtwamEndFr[resultInfo.vtwamRangeNo]            = qRound(daqInfo.SamplingFreq * ui->lineEditVtwamEnd->text().toFloat());
    }
}
void MainWindow::UpdateScanParStruct()
{
    scanInfo.scanHeight             = ui->lineEditScanHeight->text().toFloat();
    scanInfo.scanWidth              = ui->lineEditScanWidth->text().toFloat();
    scanInfo.scanInterval           = ui->enumScanInterval->currentText().toFloat();
    scanInfo.scanSpeedIndex         = ui->enumSpeed->currentIndex();
    scanInfo.Current                = ui->enumCurrent->currentText();
    scanInfo.PRF                    = ui->enumPRF->currentText();
    scanInfo.scansPerInspection     = ui->enumTotalScans->currentText().toInt();
    scanInfo.useCurrentResults      = ui->checkBox_useCurrentResults->isChecked();
    //scanInfo.enableTT             = true; //ui->checkBoxTT->isChecked();
    scanInfo.enableAS               = ui->checkBoxAbstractShapeScanning->isChecked();
    lmsInfo.trigMulFac              = scanInfo.scanSpeedArr[scanInfo.scanSpeedIndex].trigInterval/scanInfo.scanInterval;
}

void MainWindow::UpdateDaqParStruct()
{
    daqInfo.freqMode = 0;
    daqInfo.SamplingFreq    = settingArr[settingNumber].samplingFreq;
    daqInfo.Range           = settingArr[settingNumber].ldvRange;

	if(scanInfo.enableAS                 == true &&
       scanInfo.scanLinesASvec.isEmpty() == false)
        daqInfo.ScanPoints              = scanInfo.scanLinesASvec.last().accNumOfTrigs;
    else
        daqInfo.ScanPoints              = ((scanInfo.scanHeight/scanInfo.scanInterval)+1)*((scanInfo.scanWidth/scanInfo.scanInterval)+1);

    daqInfo.subbandDecomp           = settingArr[settingNumber].chNum > 1 ? true: false ; //( ui->checkBoxEnableMultiBand->isChecked() && ;
    daqInfo.totalNumOfScans         = (ui->enumTotalScans->currentText()).toInt();
    if (daqInfo.subbandDecomp)
#if NUMOFBANDS ==  3
        daqInfo.chMap                   = 15; // 4 channels
#elif NUMOFBANDS == 2
        daqInfo.chMap                   = 3; // 2 channels
#endif
    else
        daqInfo.chMap                   = 1;

    //daqInfo.daqTrigDelay    = ui->lineEditDaqTrigDelay->text().toInt();
    daqInfo.daqTrigDelay    =   settingArr[settingNumber].trigDelay;
    daqInfo.daqVoltage      = settingArr[settingNumber].daqVoltage;
    daqInfo.osciChan        = ui->enumOsciChannel->currentIndex();

    if(daqInfo.osciChan == 1)
        qwtPlotOsc->updateAxisScale(SAMPLESPERPOINT,32000);
    else
        qwtPlotOsc->updateAxisScale();

    qwtPlotResult->updateAxisScale();
    qwtSpectrogram->updateAxisXY();
}

void MainWindow::UpdateSettingsStruct()
{
    UpdateScanParStruct();
    UpdateDaqParStruct();
    UpdateLmsParStruct();
}

void MainWindow::on_pushButtonLaserControl_toggled(bool checked)
{
    if (checked)
    {
        //ui->statusBar->showMessage("Activating Laser.");
        if(laser->onSHT())
        {
            QString sCurrent;
            QString outText;
            laser->getCurrent(&sCurrent);
            float numberFloat = sCurrent.toFloat()/10;

            ui->pushButtonLaserControl->setText("Deactivate");
            ui->pushButtonLaserControl->setStatusTip("Deactivate the laser beam.");
            outText = "Laser Beam: ON (" + QString::number(numberFloat) + "A)";
            ui->labelLaserStatus->setText(outText );
        }
        else
        {
            laserError();
        }
    }
    else
    {
        //ui->statusBar->showMessage("Deactivating Laser.");
        if(laser->offSHT() )
        {
            ui->labelLaserStatus->setText("Laser Beam: OFF");
            ui->pushButtonLaserControl->setText("Activate");
            ui->pushButtonLaserControl->setStatusTip("Activate the laser beam.");
        }
        else
        {
            laserError();
        }
    }
}

void MainWindow::hiLaserPwrWarning()
{
//    QString sCurrent;
//    laser->getCurrent(&sCurrent);

//    sCurrent.toFloat()/10;
//    if ((sCurrent.toFloat()/10)>18.0)
//    {
//        QMessageBox msgBox(QMessageBox::Warning, tr("Laser Power Warning"),tr("High laser power!!!"), 0, this);
//        msgBox.exec();
//    }
}

void MainWindow::on_pushButtonLaserConfigPrfCurr_clicked()
{
    UpdateScanParStruct();
    if (laser->initLaserControllerDone == false)
    {// if init was not done then paras will be set as part of init.
        ui->statusBar->showMessage("Initializing the laser and setting parameters.");
        if(laser->initLaserController() == false)
        {
            laserError();
        }
        else
        {
            ui->statusBar->showMessage("Laser has been configured.");
            ui->pushButtonLaserControl->setEnabled(true);
            hiLaserPwrWarning();
        }
    }
    else
    {//set current and PRF separately.
        ui->statusBar->showMessage("Setting laser parameters.");
        if(!( laser->setPRF() && laser->setCurrent() ))
        {
            laserError();
        }
        else
        {
            ui->statusBar->showMessage("Laser has been configured.");
            ui->pushButtonLaserControl->setEnabled(true);
            hiLaserPwrWarning();
        }
    }
}

void MainWindow::laserError()
{
    QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("Laser controller not found.Please reconnect and try again."), 0, this);
    msgBox.exec();

    //The laser may have been turned off so make sure to initialize again next time.
    laser->initLaserControllerDone = true;
    //ui->pushButtonLaserControl->setEnabled(false);

    ui->labelLaserStatus->setText("Laser Beam: OFF");
    ui->pushButtonLaserControl->setText("Activate");
    ui->pushButtonLaserControl->setStatusTip("Activate the laser beam.");
}

void MainWindow::on_pushButtonLdvAutoFocus_clicked()
{
    if ( ! (ldv->setAutoFocus()) )
    {
        QMessageBox msgBox(QMessageBox::Critical, tr("Error 03-Connection Error"),tr("LDV not found.Please reconnect and try again."), 0, this);
        msgBox.exec();
    }
    else
    {
        ui->statusBar->showMessage("Ldv has been configured.");
    }
}

void MainWindow::setLdvRange()
{
    bool success;
    success = ldv->setRange();

    if ( !(success) )
    {
    //    QMessageBox msgBox(QMessageBox::Critical, tr("Connection Error"),tr("LDV not found.Please reconnect and try again."), 0, this);
      //  msgBox.exec();
    }
    else
    {
        ui->statusBar->showMessage("Ldv has been configured.");
    }
}

void MainWindow::on_lineEditScanHeight_editingFinished()
{
    ui->lineEditScanHeight->setText(QString::number(boundScanPars(ui->lineEditScanHeight->text().toFloat())));
    UpdateScanParStruct();
}

float MainWindow::boundScanPars(float enteredPar)
{
    /*
    int validPar;

    if(enteredPar >= 5)
    {
        if(enteredPar <= 1500)
        {
            if (enteredPar%10 == 0)
                validPar = enteredPar;
            else
                validPar = (10-enteredPar%10) +  enteredPar;
        }
        else
           validPar = 1500;
    }
    else
        validPar = 5;

    return validPar;
    */
    return enteredPar;
}

void MainWindow::on_lineEditScanWidth_editingFinished()
{
    ui->lineEditScanWidth->setText(QString::number(boundScanPars(ui->lineEditScanWidth->text().toFloat())));
}

void MainWindow::on_pushButtonSetpos_released()
{
    structRobTarget target;

    target.trans.x = ui->lineEditXpos->text().toDouble();
    target.trans.y = ui->lineEditYPos->text().toDouble();
    target.trans.z = ui->lineEditZPos->text().toDouble();

    //if (ui->lineEditRobAngleTheta->text().toInt() != 0 || ui->lineEditRobAngleTheta->text().toInt() != 0) // some angle needs to be given
    {
        target.transAfterAngle = target.trans;
        specImageProc->eulerToQuaternion(ui->lineEditRobAngleTheta->text().toInt(),ui->lineEditRobAnglePhi->text().toInt(),target.rot);
    }

    robot->moveToTarget(target,(ui->lineEditRobAngleTheta->text().toInt() != 0 || ui->lineEditRobAnglePhi->text().toInt() != 0));
}

void MainWindow::on_lineEditXpos_editingFinished()
{
    double val;
    val = ui->lineEditXpos->text().toDouble();
    ui->lineEditXpos->setText(QString::number(val,'f',2));
}

void MainWindow::on_lineEditZPos_editingFinished()
{
    double val;
    val = ui->lineEditZPos->text().toDouble();
    ui->lineEditZPos->setText(QString::number(val,'f',2));
}


void MainWindow::on_pushButtonInit_clicked()
{
    //stage->originSet();
    on_pushButtonLdvAutoFocus_clicked();
    ui->statusBar->showMessage("Initializing the laser and setting parameters.");
    if(laser->initLaserController() == false)
    {
        laserError();
        return;
    }
    else
    {
        ui->statusBar->showMessage("Laser has been configured.");
        ui->pushButtonLaserControl->setEnabled(true);
    }

    setLdvRange();//ldv->setRange();
    //on_pushButtonLaserConfigPrfCurr_clicked(); // this will automatically initilize the laser if not done so far.

    ui->pushButtonDaqSet->setEnabled(true);
    QTimer::singleShot(6500, this, SLOT(initDoneMsgBox()));

    hiLaserPwrWarning();
}

void MainWindow::initDoneMsgBox()
{
    QMessageBox msgBox(QMessageBox::Information, tr("Scan initialization"),tr("Scan initialization done."), 0, this);
    msgBox.exec();
}

void MainWindow::on_pushButtonLdvAutoFocus_2_clicked()
{
    UpdateSettingsStruct();
    //intentionally switched Height/Width
    stage->markScanArea((unsigned int)scanInfo.scanWidth,(unsigned int)scanInfo.scanHeight,(float)scanInfo.scanInterval);
}
void MainWindow::Stop()
{
    stopPressed = true;
    robot->stop();
    if (daq->OsciModeEn==false)
        daq->StopAcquisition();
    dataProc->stop();

    on_pushButtonLaserControl_toggled(false);//turn on SHT
    laser->offEXT();

    connect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
    connect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));
#ifdef ACTUALSYSTEM
    daq->Configure(true);//goto oscilo mode
#endif
    scanInfo.NumOfGenTrig = 10;
    robot->genTrig();
}

void MainWindow::settingBeforeNewScan()
{
    connect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    disconnect(ui->dial_intensity_2,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    ui->dial_intensity_2->hide();
    emit ui->dial_intensity->valueChanged(ui->dial_intensity->value()+1);
    emit ui->dial_intensity->valueChanged(ui->dial_intensity->value()-1);

    // reset the result - filter - group box
    ui->enumImageFilterIterations->setCurrentIndex(0);
    ui->enumImageFilterType->setCurrentIndex(0);
    ui->enumImageFilterSize->setCurrentIndex(0);

    ui->enumImageFilterIterations_2->setCurrentIndex(0);
    ui->enumImageFilterType_2->setCurrentIndex(0);
    ui->enumImageFilterSize_2->setCurrentIndex(0);

    ui->groupBoxFilterStep1->setChecked(false);
    ui->groupBoxFilterStep2->setChecked(false);

    ui->groupBoxFilter->setChecked(false);


    // reset the result - time - plot
    this->qwtPlotResult->initPlot();
    this->ui->horizontalSliderFrame->setValue(0);
    this->ui->horizontalSliderSubband->setValue(0);
}

void MainWindow::on_pushButtonDaqSet_clicked()
{
    ui->checkBoxRefreshCamera->setChecked(false);
    //ui->checkBoxRefreshCamera->setChecked(true);
    //ui->checkBoxRefreshCamera->setChecked(false);

    if (ui->checkBox_useCurrentResults->isChecked())
    {//load all the pars of last scan from file to make sure these settings are all same

    }

    qDebug()<<"\nMainWindow::on_pushButtonDaqSet_clicked *******Starting a new inspection********";
    on_lineEditScanHeight_editingFinished();
    on_lineEditScanWidth_editingFinished();
    UpdateSettingsStruct();
    scansDone = 0;
    stopPressed = false;

    saveSetting(true);
    if (scanInfo.enableAS)
    {
        ui->checkBoxRefreshCamera->setChecked(true);
        ui->checkBoxRefreshCamera->setChecked(false);
        if (specImageProc->calcScanGrid() == false)
            return;
    }
    on_pushButtonFilterConfig_pressed();
    if(dataProc->allocateMem()) // MEDIANDEBUG
    {
        settingBeforeNewScan();
        saveSetting(true);
#if ACTUALSYSTEM
        disconnect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
        disconnect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));

        if (ui->checkBox_useCurrentResults->isChecked() == true)
        {
            stage->gotoScanStart();
            while(stage->isReadyForRptScan() == false)
                QCoreApplication::processEvents();
        }

        laser->onEXT();
        setLdvRange();//ldv->setRange();
        if (ui->checkBoxEnGuidedWave->isChecked()==false)
            on_pushButtonLaserControl_toggled(true);//turn on SHT
        QThread::msleep(2000);
        laser->onEXT();
        QThread::msleep(2000);

        daq->Configure(false);

    if (scanInfo.enableAS)
    {
        lms->lmsScan();
        robot->startScan();
    }
    else
    {
            stage->startScanAbstract((unsigned int )scanInfo.PRF.toUInt(),
                                     (float)scanInfo.scanInterval,
                                     false);
    }
#else
        daq->Configure(false);
        /*
       //dataProc->scanFinished(100);
       //emit daq->scanFinished(0);
       //dataProc->setframeNum(0);
       */
#endif
        ui->tabWidget->setCurrentIndex(1);
        printDataSpec();
    }
    else
    {
        QMessageBox msgBox(QMessageBox::Critical, tr("Error 04-Out Of Memory"),tr("Reduce the number of scan points and try again."), 0, this);
        msgBox.exec();
    }
}

void MainWindow::scanFinished_main()
{
    qDebug()<<"MainWindow::scanFinished_main *******Scan finished********\n";
    //go to start position again
    laser->offSHT();
    ui->checkBox_useCurrentResults->setEnabled(true);
    scansDone++;
#if ACTUALSYSTEM
    on_pushButtonLaserControl_toggled(false);//turn on SHT
    laser->offEXT();
#endif

    if (scanInfo.scansPerInspection == 1)
        daq->Configure(true);//goto oscilo mode

    if (this->scanInfo.enableAS)
    {
        QTimer::singleShot(3000,stage,SLOT(gotoScanPrePos()));

        lms->getTrigCount();
        //ui->pushButtonTrigCount->click();
        robot->home();
        //lms->lmsInitialize();
    }

    // more scans needed
     if (scanInfo.scansPerInspection > 1 && stopPressed == false )
     {
         if (scansDone < scanInfo.scansPerInspection)
         {
             qDebug()<<"MainWindow::on_pushButtonDaqSet_clicked *******Starting a new scan. scansDone:"<<scansDone<<
                       "scanInfo.scansPerInspection"<<scanInfo.scansPerInspection;
             //issue another scan
             //wait for 5 seconds in between the respective scans to allow for stage to return to original position
             /*
             {
                 QElapsedTimer timer;
                 timer.start();
                 while(timer.elapsed()<15000)
                     QCoreApplication::processEvents();
            }*/

             while(stage->isReadyForRptScan() == false)
                 QCoreApplication::processEvents();

            #if ACTUALSYSTEM
                    laser->onEXT();
                    daq->Configure(false);
                    setLdvRange();//ldv->setRange();
                    on_pushButtonLaserControl_toggled(true);//turn on SHT
                    stage->startScan((unsigned int)scanInfo.scanWidth,
                                     (unsigned int)scanInfo.scanHeight,
                                     (unsigned int )scanInfo.PRF.toUInt(),
                                     (float)scanInfo.scanInterval,
                                     false);
                    ui->tabWidget->setCurrentIndex(1);
            #else
                    //emit daq->scanFinished(0);

                    //daq->Configure(false);
            #endif

         }
         else
         {
             // trigger rpt scan finished in data processor.
             dataProc->rptScanFinished();
             daq->Configure(true);//goto oscilo mode
             connect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
             connect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));
         }
     }
     else if (scanInfo.useCurrentResults)
     {
         dataProc->rptScanFinished();
         daq->Configure(true);//goto oscilo mode
         connect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
         connect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));
     }
     else
     {
         connect(dataProc,SIGNAL(setStagePosX(uint,bool)),stage,SLOT(setPosX(uint,bool)));
         connect(dataProc,SIGNAL(setStagePosZ(uint,bool)),stage,SLOT(setPosZ(uint,bool)));
     }
}

void MainWindow::on_pushButtonQuit_clicked()
{
    int result;
    QMessageBox msgBox(QMessageBox::Question, tr("Exit"),tr("Do you want to end this session ?"),
                       QMessageBox::Yes|QMessageBox::No);
    result = msgBox.exec();

    if (result ==QMessageBox::No)
    return;

    qDebug()<<"Now turnoff everything.";
    updateStatusBar_mainwindowSlot("Preparing for system turn-off.");

#ifdef ACTUALSYSTEM
    //1-stop the stage.
    Stop();

    {
        QElapsedTimer timer;
        timer.start();
        while(timer.elapsed()<100)
            QCoreApplication::processEvents();
   }

    stage->clearServoStop();

    //stage->setPosX(250000);
    //stage->setPosZ(500000);
    stage->setPosX(0);
    stage->setPosZ(0);


    //stage->clearServoStop();
    //2-take stage to desired position.


    //3-set laser all off.
    laser->offDIO();
    laser->offQS();
#endif

    QCoreApplication::quit();
}

void MainWindow::on_actionAbout_triggered()
{
    //QMessageBox msgBox(QMessageBox::about, tr("Memory Error"),tr("dataProcessor::scanFinished - wfmPtr = NULL"));
    QMessageBox::about(this, trUtf8("About"), trUtf8("Full Field Pulse Echo Ultrasonic Propogation Imaging System (ver2.0) \n "
                                                     "by XNDT"));
    //msgBox.exec();
    return;
}

void MainWindow::on_actionBand_Divider_Scenario_triggered()
{
    QLabel* help=new QLabel();
    help->setWindowTitle("Band Divider Scenario");
    help->setWindowFlags(Qt::Tool); //or Qt::Tool, Qt::Dialog if you like
    help->setPixmap(QPixmap("://images/BandDividerScenario.GIF"));
    help->setScaledContents(true);
    help->show();
}

void MainWindow::on_actionPulse_Energy_Table_triggered()
{
    QLabel* help=new QLabel();
    help->setWindowTitle("Pulse Energy Table");
    help->setWindowFlags(Qt::Tool); //or Qt::Tool, Qt::Dialog if you like
    help->setPixmap(QPixmap("://images/PulseEnergyTableV4.gif"));
    help->show();
}

void MainWindow::on_pushButtonDaqConfig_pressed()
{
    UpdateDaqParStruct();
    daq->Configure(true);
}

void MainWindow::on_checkBoxAbstractShapeScanning_toggled(bool checked)
{
    ui->checkBoxRefreshCamera->setChecked(checked);
    if (checked)
    {
        widgetInteractiveAS->show();
        ui->groupBoxASControls->show();
    }
    else
    {
        widgetInteractiveAS->hide();
        ui->groupBoxASControls->hide();
    }
}

void MainWindow::on_checkBoxEnableMultiBand_toggled(bool checked)
{
    ui->enumMultiBandSettingLev1->clear();
    ui->checkBox_useCurrentResults->setChecked(false);
    ui->enumTotalScans->setCurrentIndex(0);
    if (checked == true)
    {
        ui->enumMultiBandSettingLev1->addItem("Bulk");
        ui->enumMultiBandSettingLev1->addItem("Narrowband");
        ui->enumMultiBandSettingLev1->addItem("Guided");

        windowLayOutUpdate(true);
    }
    else
    {
        ui->enumMultiBandSettingLev1->addItem("050-250 kHz");
        ui->enumMultiBandSettingLev1->addItem("250-500 kHz");
        ui->enumMultiBandSettingLev1->addItem("500-750 kHz");
        ui->enumMultiBandSettingLev1->addItem("750-1000 kHz");
        ui->enumMultiBandSettingLev1->addItem("1000-1160 kHz");
        ui->enumMultiBandSettingLev1->addItem("1160-1320 kHz");
        ui->enumMultiBandSettingLev1->addItem("1320-1500 kHz");

        windowLayOutUpdate(false);
    }
}

void MainWindow::windowLayOutUpdate(bool isMultiCh)
{
    if (isMultiCh)
    {
        ui->groupBoxMultiScan->hide();
        if (ui->tabWidget->tabText(1) != "Sub-band decomposition")
            ui->tabWidget->insertTab(1,subbandWidget,"Sub-band decomposition");
        ui->enumOsciChannel->clear();
        ui->enumOsciChannel->addItem("Sub-band 1");
        ui->enumOsciChannel->addItem("Sub-band 2");
#if NUMOFBANDS == 3
        ui->enumOsciChannel->addItem("Sub-band 3");
#endif
        ui->enumOsciChannel->setCurrentIndex(0);
    }
    else
    {
        ui->groupBoxMultiScan->show();
        if (ui->tabWidget->tabText(1) == "Sub-band decomposition")
            ui->tabWidget->removeTab(1); // remove the sub-band tab
        ui->enumOsciChannel->clear();
        ui->enumOsciChannel->addItem("Sub-band 1");
        ui->enumOsciChannel->setCurrentIndex(0);
    }
}
void MainWindow::on_enumMultiBandSettingLev1_currentIndexChanged(int index)
{
    if (index == 1 && ui->checkBoxEnableMultiBand->isChecked() == true)
       ui->enumMultiBandSettingLev2->show();
    else
       ui->enumMultiBandSettingLev2->hide();
}

void MainWindow::on_pushButtonFilterConfig_pressed()
{
    //ascertain the setting number to use
    if (ui->checkBoxEnableMultiBand->isChecked() == true)
    {
        //the broadBand
        settingNumber = ui->enumMultiBandSettingLev1->currentIndex();

        if (ui->enumMultiBandSettingLev1->currentIndex() == 1)//narrow
            settingNumber = ui->enumMultiBandSettingLev2->currentIndex()+1;

        else if (ui->enumMultiBandSettingLev1->currentIndex() == 2)//hi
            settingNumber = 4;
    }
    else
    {
        settingNumber = ui->enumMultiBandSettingLev1->currentIndex()+5;
    }

    UpdateDaqParStruct();

    windowLayOutUpdate(settingArr[settingNumber].chNum > 1);

    /*
    daqInfo.settingStr = "Number of Bands = "+ QString::number(settingArr[settingNumber].chNum)+
            ", Sample Freq = "+ QString::number(settingArr[settingNumber].samplingFreq)+" MHz"
            ", LDV Range = "+ QString::number(settingArr[settingNumber].ldvRange)+" mm/s/V";
    */
    daqInfo.settingStr.clear();
    for (int chNumber = 1; chNumber<=settingArr[settingNumber].chNum; chNumber++ )
    {
        filter->config(chNumber,settingArr[settingNumber].filtPar[chNumber].gain,
                       settingArr[settingNumber].filtPar[chNumber].hiPassCut,
                       settingArr[settingNumber].filtPar[chNumber].lowPassCut);

        daqInfo.settingStr += "Sub-band " + QString::number(chNumber) +
                " : "+ QString::number(settingArr[settingNumber].filtPar[chNumber].hiPassCut)+" KHz"
                " ~ "+ QString::number(settingArr[settingNumber].filtPar[chNumber].lowPassCut)+" KHz\n";
    }

    setLdvRange();//ldv->setRange();
    daq->Configure(true);

    qDebug()<<daqInfo.settingStr;
    this->ui->labelConfigSetting->setText(daqInfo.settingStr);
    this->ui->labelConfigSettingSubBand->setText(daqInfo.settingStr);
}

void MainWindow::loadDataMain(bool click)
{
    QString dataPath;

    ui->checkBoxEnableMultiBand->setChecked(false);
    //on_pushButtonFilterConfig_pressed();
    dataPath = dataProc->loadData(click);

    printDataSpec();
    qwtPlotOsc->updateAxisScale();
    qwtPlotResult->updateAxisScale();

    if (dataPath!=NULL)
    {
        bool result;

        //save the scan area delimited pic
        QPixmap pixmap;
        result = pixmap.load(dataPath+"\\ScannedArea.BMP",0);
        ui->labelProcessed->setPixmap(pixmap);

        //for polygon pic
        specImageProc->load(dataPath);
    }
}

void MainWindow::saveDataMain(bool status)
{
    QString dataPath;
    dataPath = dataProc->saveData(status);

    if (dataPath!=NULL)
    {
        bool result;
        //save the scan area delimited pic
        const QPixmap *pixmapPtr = ui->labelProcessed->pixmap();
        result = pixmapPtr->save(dataPath+"ScannedArea.BMP",0,100);
        //for polygon pic
        specImageProc->save(dataPath);
    }
}

void MainWindow::printDataSpec()
{
    QString label;
    label.sprintf("Inspection Settings : Height = %0.2f mm, Width = %0.2f mm, Interval = %0.2f mm, Sampling Frequency = %d MHz",
                  scanInfo.scanHeight,scanInfo.scanWidth,scanInfo.scanInterval,daqInfo.SamplingFreq);
    ui->labelCurrentDataSpec->setText(label);
    //qwtPlotResult->updateAxisScale();
}

//-------------------------------------------------Result tab----------------------------------------
#define SPEEDX1INTERVALMS 100
//Spectrogram takes 23~25ms to render. Using fastes 30ms per update to give breathing room to worker threads
//x1->100ms+10ms(overhead)
//x2->50+10ms(overhead)
//x4->25+10ms(overhead)
//anything beyond this will just choke up the timer thread Q since the processing is bottlenecked by the spectrogram update
void MainWindow::playPauseResult(bool play)
{
    sliderIncVal = 1;
    if (play)
    {
       incrSlider();
       if (ui->enumPlaySpeed->currentIndex()<4) // only speed up till X4
       {
           mainTimer->start(SPEEDX1INTERVALMS /(1 << ui->enumPlaySpeed->currentIndex()) );
           sliderIncVal = 1;
       }
       else
       {
           mainTimer->start(SPEEDX1INTERVALMS/4 );
           sliderIncVal = ui->enumPlaySpeed->currentIndex() - 3;
       }


       ui->pushButtonPlayPause->setText("Pause");
       ui->pushButtonPlayPause->setStatusTip("Pause the automatic display of data.");
       ui->pushButtonPlayPause->setChecked(true);
    }
    else
    {
        mainTimer->stop();
        ui->pushButtonPlayPause->setText("Play");
        ui->pushButtonPlayPause->setChecked(false);
        ui->pushButtonPlayPause->setStatusTip("Automatically traverse the data in a frame-wise fashion. ");
    }
}

void MainWindow::incrSlider()
{
    int curVal = ui->horizontalSliderFrame->value();
    curVal += sliderIncVal;
    if (curVal >= SAMPLESPERPOINT)
    {
        //playPauseResult(false);
        curVal = 0;
    }

    ui->horizontalSliderFrame->setValue(curVal);
    /*
    {
        static QElapsedTimer t;
        qDebug()<<"incrSlider: curVal" << curVal <<"sliderIncVal"<<sliderIncVal: <<"Inter increment time: "<<t.elapsed();
        t.start();
    }
    */
}

void MainWindow::on_enumPlaySpeed_currentIndexChanged(int index)
{
    if(ui->pushButtonPlayPause->isChecked()== TRUE)
    {

        playPauseResult(false);
        playPauseResult(true);
/*
        mainTimer->stop();

        mainTimer->start(100/(1 << index ));
*/
    }
}

void MainWindow::on_groupBoxFilterStep1_toggled(bool arg1)
{
    if (arg1==false)
        ui->groupBoxFilterStep1->setChecked(false);//must always have a step 1`
}

void MainWindow::on_groupBoxFilterStep2_toggled(bool arg1)
{
    if (arg1==true)
        ui->groupBoxFilterStep1->setChecked(true);//must always have a step 1`
}

void MainWindow::on_pushButtonProcessFilter_clicked()
{
    //ui->pushButton->setEnabled(false);
    this->updateResultParStruct();
    emit postProcessingFilteringRequired();
}


void MainWindow::on_pushButtonCapture_clicked()
{
    ui->labelVtwamRangeTitle->hide();
    connect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    disconnect(ui->dial_intensity_2,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
    ui->dial_intensity_2->hide();
    dataProc->saveScreenshot();
    emit ui->dial_intensity->valueChanged(ui->dial_intensity->value()+1);
    emit ui->dial_intensity->valueChanged(ui->dial_intensity->value()-1);
}

void MainWindow::on_pushButtonProcessXCor_clicked()
{
    QString tempStr;
    int index = 0;

    ui->labelVtwamRangeTitle->hide();
   //this->updateResultParStruct();
   //show the second knob and allocate proper scale
   disconnect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
   connect(ui->dial_intensity_2,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
   ui->dial_intensity_2->show();
   emit ui->dial_intensity_2->valueChanged(ui->dial_intensity_2->value()+1);
   emit ui->dial_intensity_2->valueChanged(ui->dial_intensity_2->value()-1);
   emit ui->dial_intensity_2->setValue(1<<19);
   dataProc->postProcessingXCorRequested();
}

void MainWindow::on_pushButtonProcessVtwam_clicked()
{
    QString VTWAMtitle;
    QString tempStr;
    int index = 0;

    if(resultInfo.vtwamRangeNo == -1)
    {
        //on_pushButtonVtwamAddRange_clicked();
        return;
    }

    VTWAMtitle = "VTWAM Result(";
    do
    {
        if (index>0)
            VTWAMtitle +=",";

        double startTime = (double)resultInfo.vtwamStartFr[index]/(double)daqInfo.SamplingFreq;
        double endTime =   (double)resultInfo.vtwamEndFr[index]/(double)daqInfo.SamplingFreq;

        tempStr.sprintf("%.3f \xC2\xB5s ~ %.3f \xC2\xB5s",startTime,endTime);

        VTWAMtitle += tempStr;
        index++;

    }while(index<=resultInfo.vtwamRangeNo);

    VTWAMtitle +=")";

   //this->updateResultParStruct();
   //show the second knob and allocate proper scale
   disconnect(ui->dial_intensity,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
   connect(ui->dial_intensity_2,SIGNAL(valueChanged(int)),qwtSpectrogram2,SLOT(setIntensity(int)));
   ui->dial_intensity_2->show();
   emit ui->dial_intensity_2->valueChanged(ui->dial_intensity_2->value()+1);
   emit ui->dial_intensity_2->valueChanged(ui->dial_intensity_2->value()-1);
   emit postProcessingVtwamRequired(VTWAMtitle);
}

void MainWindow::on_pushButtonVtwamAddRange_clicked()
{
    resultInfo.vtwamRangeNo = (resultInfo.vtwamRangeNo + 1) % MAXVTWAMRANGES;
    this->updateResultParStruct();
    printVtwamInfo();
}

void MainWindow::on_pushButtonVtwamClearRanges_clicked()
{
    resultInfo.vtwamRangeNo = -1;
    ui->labelVtwamInfo->setText("");
    ui->labelVtwamRangeTitle->hide();
}

void MainWindow::printVtwamInfo()
{
    QString label,tempStr;
    double startTime = (double)resultInfo.vtwamStartFr[resultInfo.vtwamRangeNo]/(double)daqInfo.SamplingFreq;
    double endTime = (double)resultInfo.vtwamEndFr[resultInfo.vtwamRangeNo]/(double)daqInfo.SamplingFreq;

    tempStr.sprintf("%.3f \xC2\xB5s ~ %.3f \xC2\xB5s",startTime,endTime);

    if (resultInfo.vtwamRangeNo>0)
        label = ui->labelVtwamInfo->text() + ", " + tempStr + ",";

    else
        label = tempStr;

    ui->labelVtwamInfo->setText(label);
    ui->labelVtwamRangeTitle->show();

    //qwtPlotResult->updateAxisScale();
}

void MainWindow::updateVtwamInputs(bool press, int frValue)
{
    QString timeVal;// = QString::number(((double)frValue/daqInfo.SamplingFreq) ,'g',3);

    timeVal.sprintf("%.3f",((double)frValue/daqInfo.SamplingFreq));

    if (press)
    {
        //ui->lineEditVtwamStart->setText(QString::number(value));
        ui->lineEditVtwamStart->setText(timeVal);
    }
    else
    {
        //ui->lineEditVtwamEnd->setText(QString::number(value));
        ui->lineEditVtwamEnd->setText(timeVal);
    }
}

void MainWindow::on_pushButtonEnlarge_clicked()
{
    enlargeResultDlgn->attachSpect(qwtSpectrogram);

    ui->groupBoxResultControl->setParent(enlargeResultDlgn);
    ui->groupBoxResultControl->move(0,0);
    ui->groupBoxResultControl->showMaximized();

    qwtSpectrogram->enlargeEnabled = TRUE;
    qwtSpectrogram->updateAxisXY(enlargeResultDlgn->giveSlider()->value());
}

void MainWindow::resizeToNormal()
{
    qwtSpectrogram->setParent(ui->widgettResultSpect);
    qwtSpectrogram->enlargeEnabled = FALSE;
    qwtSpectrogram->updateAxisXY();
    qwtSpectrogram->show();

    ui->groupBoxResultControl->setParent(this->ui->widgetResultControl);
    ui->groupBoxResultControl->move(-5,10);
    ui->groupBoxResultControl->show();
}

//-------------------------------------------------Subband tab----------------------------------------
void MainWindow::playPauseResultSubband(bool play)
{
    if (play)
    {
       incrSliderSubband();
       mainTimerSubband->start(50 /(1 << ui->enumPlaySpeed->currentIndex()) );
       ui->pushButtonPlayPauseSubband->setText("Pause");
       ui->pushButtonPlayPauseSubband->setStatusTip("Pause the automatic display of data.");
       ui->pushButtonPlayPauseSubband->setChecked(true);
    }
    else
    {
        mainTimerSubband->stop();
        ui->pushButtonPlayPauseSubband->setText("Play");
        ui->pushButtonPlayPauseSubband->setChecked(false);
        ui->pushButtonPlayPauseSubband->setStatusTip("Automatically traverse the data in a frame-wise fashion. ");
    }
}

void MainWindow::incrSliderSubband()
{
    int curVal = ui->horizontalSliderSubband->value();
    ui->horizontalSliderSubband->setValue(++curVal);
    if (curVal == SAMPLESPERPOINT)
        playPauseResultSubband(false);
}

void MainWindow::on_enumPlaySpeedSubband_currentIndexChanged(int index)
{
    if(ui->pushButtonPlayPauseSubband->isChecked()== TRUE)
    {
        mainTimerSubband->stop();
        mainTimerSubband->start(50/(1 << index ));
    }
}

void MainWindow::on_radioButtonSubband1_toggled(bool checked)
{
    if(checked)
        dataProc->chooseSubband(0);

    //ui->tabWidget->setCurrentIndex(2);
}

void MainWindow::on_radioButtonSubband2_toggled(bool checked)
{
    if(checked)
        dataProc->chooseSubband(1);

    //ui->tabWidget->setCurrentIndex(2);
}

void MainWindow::on_radioButtonSubband3_toggled(bool checked)
{
    if(checked)
        dataProc->chooseSubband(2);

    //ui->tabWidget->setCurrentIndex(2);
}
//-------------------------------------------------------------------------------- slider
MySlider::MySlider(QWidget *parent):
QSlider(parent)
{
/* // line beneath the scaleDraw
    line = new QFrame(this);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Raised);
    line->setVisible(false); //not working
*/
}

void MySlider::mousePressEvent ( QMouseEvent * event )
{
    if (event->button() == Qt::MiddleButton)
    {
        qDebug()<<"middle button pressed"<<event->pos().x()<<event->pos().y();
        xAtpress = event->pos().x();

        emit mouseMidButton(1,this->value());
    }
  QSlider::mousePressEvent(event);

}
void MySlider::mouseReleaseEvent ( QMouseEvent * event )
{
    if (event->button() == Qt::MiddleButton)
    {
        qDebug()<<"middle button released"<<event->pos().x()<<event->pos().y();
/*
        //line->setGeometry(QRect(xAtpress, (this->pos().y()), event->pos().x(), (this->pos().y())+5));
        this->line->setGeometry(QRect(xAtpress, 13, event->pos().x(), 13));
        //line->setGeometry(this->rect());
        this->line->setFrameShape(QFrame::HLine);
        this->line->setFrameShadow(QFrame::Sunken);
*/

        emit mouseMidButton(0,this->value());
    }
  QSlider::mouseReleaseEvent(event);
}

void MainWindow::on_enumSpeed_activated(int index)
{
    scanInfo.scanSpeedIndex = index;
}

void MainWindow::on_pushButtonSetpos_LMS_released()
{
    lms->lmsSetPos(ui->lineEditXpos_LMS->text().toFloat(),ui->lineEditYPos_LMS->text().toFloat(),ui->lineEditZPos_LMS->text().toFloat());
}

void MainWindow::UpdateLmsParStruct()
{
    lmsInfo.SOD = ui->lineEditStandofDistance_3->text();
    lmsInfo.laserStep = ui->spinBoxLmsStep->text().toDouble();
    qDebug()<<"standoff distance = "<<lmsInfo.SOD;
}

void MainWindow::on_checkBoxDisplayScanArea_clicked(bool checked)
{
    UpdateLmsParStruct();
    lmsInfo.diplayArea=checked;
}

void MainWindow::on_spinBoxLmsStep_editingFinished()
{
    UpdateLmsParStruct();
}

void MainWindow::on_checkBox_clicked(bool checked)
{
    if(checked)
        lmsInfo.loadCorrectionFile=true;
    else
        lmsInfo.loadCorrectionFile=false;

    lms->lmsInitialize();
}

void MainWindow::on_spinBoxTrigStep_editingFinished()
{
    scanInfo.NumOfGenTrig=ui->spinBoxTrigStep->text().toInt();
}

void MainWindow::on_lineEditPixelPeriod_editingFinished()
{
    lms->LmssetPixelPeriod(ui->lineEditPixelPeriod_2->text().toFloat());
}

void MainWindow::on_LineEditTrigPeriodCompress_editingFinished()
{
    lms->LmssetTrigCompress(ui->LineEditTrigPeriodCompress_2->text().toFloat());
}

void MainWindow::on_pushButtonConfigTrigCompressPixelPeriod_clicked()
{
    lms->LmssetTrigCompress(ui->LineEditTrigPeriodCompress_2->text().toFloat());
    lms->LmssetPixelPeriod(ui->lineEditPixelPeriod_2->text().toFloat());
}

void MainWindow::on_pushButtonConfigCorFileParameters_clicked()
{
    xAxisRightLimitBits =   ui->lineEditCorFileXaxisRightLimit->text().toInt();
    xAxisLeftLimitBits  =   ui->lineEditCorFileXaxisLeftLimit->text().toInt();
    yAxisUpLimitBits    =   ui->lineEditCorFileYaxisUpLimit->text().toInt();
    //yAxisDownLimitBits  =   ui->lineEditCorFileYaxisDownLimit->text().toInt();
    CorFileSize         =   ui->lineEditCorFileResolution->text().toInt();

    xOriginBits         =   ui->lineEditXpos_LMS->text().toInt();//lmsInfo.currentXabs;
    yOriginBits         =   ui->lineEditYPos_LMS->text().toInt();//lmsInfo.currentYabs;
//    xOriginBits         =   -230000;
//    yOriginBits         =   206010;
    yAxisDownLimitBits  =   yOriginBits;
    qDebug("MainWindow::on_pushButtonConfigCorFileParameters_clicked -- xOriginBits: %d, yOriginBits: %d ",
           xOriginBits,yOriginBits);
}

void MainWindow::on_pushButtonGenerateCorFile_clicked()
{
    QElapsedTimer timer;
    timer.start();
    on_pushButtonConfigCorFileParameters_clicked();
    //take a BG snapshot
    specImageProc->greenLaserTrackerWithBgSubtraction(true);

    //laser control
    scanInfo.Current            = "13.5";
    scanInfo.PRF                = "35";
    laser->setPRF();
    laser->setCurrent();
    laser->onSHT();

//    on_checkBox_clicked(false);//Dont load the correction file.

    //File time
    QDateTime now = QDateTime::currentDateTime();
    QString fileName = "D:\\Mohsin_Data\\Pincushion_removal_camera_based_second_study\\corFiles\\"+now.toString("ddMMyy_hhmmss")+"_"
            +QString::number(CorFileSize)+"CorFile.dat";
//    QString fileName = "D:\\Hasan\\LMS\\correXionPro104\\AUTOMATED_CORRECTION_FILES\\"+now.toString("ddMMyy_hhmmss")+"_"
//            +QString::number(CorFileSize)+"CorFile.dat";
    QFile mFile(fileName);
    if(!mFile.open(QFile::WriteOnly | QFile::Text))
    {
      qDebug()<<" Could not open file for writing";
      return;
    }

    QTextStream out(&mFile);
    //file header
    out<<"OLDCTFILE      =  cor_1to1.ct5 \n";
    out<<"NEWCTFILE      =  "<<CorFileSize<<"X"<<CorFileSize<<"CorrectionFile.ct5\n";
    out<<"TOLERANCE      =  10.0  \n\n";

    out<<"//RTC-Y[bit]      RTC-X[bit]      REAL-Y[mm]      REAL-X[mm] \n\n";

    Point originPixelValueInt,originPixelValueSum;
    Point2f originPixelValueFloat;

    Point pixelValueInt,pixelValueSum;
    Point2f pixelValueFloat;

    int avgCount;

    int counter=0;
    float CorYaxisStep = (yAxisUpLimitBits-yAxisDownLimitBits)/(CorFileSize-1);
    float CorXaxisStep = (xAxisRightLimitBits-xAxisLeftLimitBits)/(CorFileSize-1);
    bool valCorrect = false;
    float xPosCurBits=0;
    float yPosCurBits=0;
    float xDistValue=0, yDistValue=0;
    float xBitValue=0, yBitValue=0;
    float pixelPerMMx = 1.298;  // 1.4371 for black CFRP small Specimen EXP1~4 17th June
    float pixelPerMMy = 1.298;  // 1.4371 for black CFRP small Specimen EXP1~4 17th June
    int   pixelValueUpperThreshold =0;
    int   pixelValueLowerThreshold =0;
    float pixelPerBit   = 0.0009375;
    //  First jump to the origin of the correction file generation area
    //lms->lmsJumpAbs(0,0);
    lms->lmsJumpAbs(xOriginBits,yOriginBits);
    qDebug()<<" Origin x-Bits:"<<xOriginBits<<" Origin y-Bits:"<<yOriginBits;
    Sleep(500);

    for (avgCount = 0;avgCount<5;avgCount++)
    {
        originPixelValueInt  = (Point)(-1,-1);
        while (originPixelValueInt.x == -1 || originPixelValueInt.y == -1)
            originPixelValueInt = specImageProc->greenLaserTrackerWithBgSubtraction();

        originPixelValueSum += originPixelValueInt;

        qDebug(" avgCount: %d - originPixelValueSum(x,y): (%d,%d) - originPixelValueInt(x,y): (%d,%d)",
               avgCount,originPixelValueSum.x,originPixelValueSum.y,originPixelValueInt.x,originPixelValueInt.y);
    }
    originPixelValueFloat = originPixelValueSum / avgCount;
    qDebug()<<"originPixelValueFloat.x: "<<originPixelValueFloat.x<<"originPixelValueFloa.yt:  "<<originPixelValueFloat.y;

    for(int i=0; i<CorFileSize;i++)
    {
        yPosCurBits = yAxisDownLimitBits + i*CorYaxisStep;

        for(int j=0; j<CorFileSize;j++)
        {
            qDebug()<<" ---------Counter:  "<<counter;
            xPosCurBits = xAxisLeftLimitBits + j*CorXaxisStep;
            lms->lmsJumpAbs(xPosCurBits,yPosCurBits);
            Sleep(500);

            pixelValueSum = (Point)(0,0);
            for (avgCount = 0;avgCount<5;avgCount++)
            {
                pixelValueInt = (Point)(-1,-1);
                while (pixelValueInt.x == -1 || pixelValueInt.y == -1)
                {
                    //Sleep(2000);
                    pixelValueInt =  specImageProc->greenLaserTrackerWithBgSubtraction();

//                    pixelValueUpperThreshold = (originPixelValueFloat.y - i*CorYaxisStep*pixelPerBit) + 7;
//                    pixelValueLowerThreshold = (originPixelValueFloat.y - i*CorYaxisStep*pixelPerBit) - 7;
//                    qDebug(" pixelValueUpperThreshold: %d - pixelValueLowerThreshold: %d ",
//                           pixelValueUpperThreshold,pixelValueLowerThreshold);
//                    if(pixelValueInt.y > pixelValueLowerThreshold && pixelValueInt.y < pixelValueUpperThreshold)
//                    {
//                        valCorrect = true;
//                    }
                }

                pixelValueSum += pixelValueInt;

                qDebug(" avgCount: %d - pixelValueSum(x,y): (%d,%d) - pixelValueInt(x,y): (%d,%d)",
                       avgCount,pixelValueSum.x,pixelValueSum.y,pixelValueInt.x,pixelValueInt.y);
            }
            pixelValueFloat = pixelValueSum/avgCount;

            xDistValue = -1*(originPixelValueFloat.x - pixelValueFloat.x)/pixelPerMMx;
            yDistValue = -1*(originPixelValueFloat.y - pixelValueFloat.y)/pixelPerMMy;
            xBitValue  = xPosCurBits;
            yBitValue  = -1*yPosCurBits;

            out<< yBitValue <<"\t\t"<< xBitValue<<"\t\t"<< yDistValue<<"\t\t" << xDistValue<<"\n";
            qDebug()<<" xPosCurBits:  "<<xPosCurBits<<" yPosCurBits: "<<yPosCurBits;
            qDebug()<<" pixelValueFloat.x "<<pixelValueFloat.x<<" pixelValueFloat.y "<<pixelValueFloat.y;
            qDebug()<<" X Bit Value "<<xBitValue<<" Y Bit Value "<<yBitValue;
            qDebug()<<" X Distance  "<<xDistValue<<" Y Distance  "<<yDistValue;
            counter++;

            //take a BG snapshot
            if(j%2 == 0)
            {
                laser->offSHT();
                Sleep(500);
                specImageProc->greenLaserTrackerWithBgSubtraction(true);
                laser->onSHT();
            }
        }
        out<<"\n\n";
    }

    //wrap up
    laser->offSHT();

    mFile.flush();
    mFile.close();

    qDebug()<<"on_pushButtonGenerateCorFile_clicked took: "<<(double)timer.elapsed()/(double)60000 << "minutes";;
}

void MainWindow::on_checkBoxEnDepthControl_toggled(bool checked)
{
     scanInfo.enableDepthControl = checked;
}

void MainWindow::on_checkBoxEnOrientControl_toggled(bool checked)
{
    scanInfo.enableOrientControl = checked;
}

void MainWindow::on_pushButtonGenerateCorFileWRob()
{
    QElapsedTimer timer;
    timer.start();
    on_pushButtonConfigCorFileParameters_clicked();
    //take a BG snapshot
    specImageProc->greenLaserTrackerWithBgSubtraction(true,1); // 1 for excitation laser tracking
    //laser control
    scanInfo.Current            = "13.5";
    scanInfo.PRF                = "50";
    laser->setPRF();
    laser->setCurrent();
    laser->onSHT();

    on_checkBox_clicked(false);//Dont load the correction file.

    bool localRecord = scanInfo.enableOrientControl;
    scanInfo.enableOrientControl = false;

    //File time
    QDateTime now = QDateTime::currentDateTime();
    QString fileName = "D:\\Hasan\\LMS\\correXionPro104\\AUTOMATED_CORRECTION_FILES\\"+now.toString("ddMMyy_hhmmss")+"_"
            +QString::number(CorFileSize)+"CorFileRob.dat";

    QFile mFile(fileName);
    if(!mFile.open(QFile::WriteOnly | QFile::Text))
    {
      qDebug()<<" Could not open file for writing";
      return;
    }
    QTextStream out(&mFile);
    //file header
    out<<"OLDCTFILE      =  cor_1to1.ct5 \n";
    out<<"NEWCTFILE      =  "<<CorFileSize<<"X"<<CorFileSize<<"CorrectionFile.ct5\n";
    out<<"TOLERANCE      =  10.0  \n\n";

    out<<"//RTC-Y[bit]      RTC-X[bit]      REAL-Y[mm]      REAL-X[mm] \n\n";

    QString fileName1 = "D:\\Hasan\\LMS\\correXionPro104\\AUTOMATED_CORRECTION_FILES\\"+now.toString("ddMMyy_hhmmss")+"_"
            +QString::number(CorFileSize)+"CorFileRobCord.dat";

    QFile mFile1(fileName1);
    if(!mFile1.open(QFile::WriteOnly | QFile::Text))
    {
      qDebug()<<" Could not open file for writing";
      return;
    }

    QTextStream out1(&mFile1);
    //file header
    out1<<"OLDCTFILE      =  cor_1to1.ct5 \n";
    out1<<"NEWCTFILE      =  "<<CorFileSize<<"X"<<CorFileSize<<"CorrectionFile.ct5\n";
    out1<<"TOLERANCE      =  10.0  \n\n";

    out1<<"//RTC-Y[bit]      RTC-X[bit]      REAL-Y[mm]      REAL-X[mm] \n\n";

    Point originPixelValueInt,originPixelValueSum;
    Point2f originPixelValueFloat;

    Point pixelValueInt,pixelValueSum;
    Point2f pixelValueFloat;

    int avgCount;

    int counter=0;
    float CorYaxisStep = (yAxisUpLimitBits-yAxisDownLimitBits)/(CorFileSize-1);
    float CorXaxisStep = (xAxisRightLimitBits-xAxisLeftLimitBits)/(CorFileSize-1);
    bool valCorrect = false;
    float xPosCurBits=0;
    float yPosCurBits=0;
    float xDistValue=0, yDistValue=0;
    float xDistValueRob=0, yDistValueRob=0;
    float xBitValue=0, yBitValue=0;
    float pixelPerMMx = 2*1.86;  //1.55 for the gfrp speci // 1.4371 for black CFRP small Specimen EXP1~4 17th June
    float pixelPerMMy = 2*1.86;  // 1.4371 for black CFRP small Specimen EXP1~4 17th June
    int   pixelValueUpperThreshold =0;
    int   pixelValueLowerThreshold =0;
    float pixelPerBit   = 0.0009375;

    int colIndex;
    structRobTarget robPointOrig;
    structRobTarget robPoint;
    int angle = 0;
    Point retLdvLaserPoint,retLdvLaserPointSum;
    Point2f retLdvLaserPointValAvg;
    Point laserDiff;
    float adjustStep = 1/pixelPerMMx;

    //  First jump to the origin of the correction file generation area
    //lms->lmsJumpAbs(0,0);
    lms->lmsJumpAbs(xOriginBits,yOriginBits);
    Sleep(2000);

    for (avgCount = 0;avgCount<5;avgCount++)
    {
        originPixelValueInt  = (Point)(-1,-1);
        while (originPixelValueInt.x == -1 || originPixelValueInt.y == -1)
            originPixelValueInt = specImageProc->greenLaserTrackerWithBgSubtraction(false,1);

        originPixelValueSum += originPixelValueInt;

        qDebug(" avgCount: %d - originPixelValueSum(x,y): (%d,%d) - originPixelValueInt(x,y): (%d,%d)",
               avgCount,originPixelValueSum.x,originPixelValueSum.y,originPixelValueInt.x,originPixelValueInt.y);
    }
    originPixelValueFloat = originPixelValueSum / avgCount;
    qDebug()<<"originPixelValueFloat.x: "<<originPixelValueFloat.x<<"originPixelValueFloa.yt:  "<<originPixelValueFloat.y;

    laser->offSHT();

    robot->ldvLaserTurnOn(false);
    Sleep(1000);
    specImageProc->greenLaserTrackerWithBgSubtraction(true,0); // bg img 0 for LDV laser detection

//    //the origin of robot
//    robPointOrig.trans.x = 500;
//    robPointOrig.trans.y = 290.25;
//    robPointOrig.trans.z = 364.25;

    //the origin of robot - 26Sep file
    robPointOrig.trans.x = 440;
    robPointOrig.trans.y = 321;
    robPointOrig.trans.z = 283.25;

   // if (specImageProc->isRobPtWihinRange(robPointOrig.trans))
    {
        robot->moveToTarget(robPointOrig,false,true);
        Sleep(2000);
    }

    ///// get LDV pixel value
    laserDiff.x = 10;
    laserDiff.y = 10;
    //while (laserDiff.x != 0 || laserDiff.y != 0)
    while((abs(laserDiff.x) > 1 || abs(laserDiff.y) > 1) && ui->pushButtonGenerateCorFileWRob->isChecked() )
    {
        if (specImageProc->isRobPtWihinRange(robPointOrig.trans))
        {
            robot->moveToTarget(robPointOrig,false,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,false,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,false,true);
            Sleep(100);
        }

        retLdvLaserPointSum = (Point)(0,0);
        for (avgCount = 0;avgCount<5;avgCount++)
        {
            retLdvLaserPoint  = (Point)(-1,-1);
            while ((retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1) && ui->pushButtonGenerateCorFileWRob->isChecked())
                retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

            retLdvLaserPointSum += retLdvLaserPoint;

            qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                   avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
        }
        retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
        qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

        laserDiff.x = retLdvLaserPointValAvg.x - originPixelValueFloat.x;
        laserDiff.y = retLdvLaserPointValAvg.y - originPixelValueFloat.y;

//        if (laserDiff.x > 0)
//            robPointOrig.trans.y += 0.5;
//        else if (laserDiff.x != 0)
//            robPointOrig.trans.y -= 0.5;

//        if (laserDiff.y > 0)
//            robPointOrig.trans.z += 0.5;
//        else if (laserDiff.y != 0)
//            robPointOrig.trans.z -= 0.5;

        if (laserDiff.x != 0)
        {
            if (abs(laserDiff.x) >2)
               adjustStep = 1/pixelPerMMx;
            else
                adjustStep = 0.25;

            robPointOrig.trans.y += laserDiff.x*adjustStep;
        }

        if (laserDiff.y != 0)
        {
            if (abs(laserDiff.y) >2)
               adjustStep = 1/pixelPerMMx;
            else
                adjustStep = 0.25;

            robPointOrig.trans.z += laserDiff.y*adjustStep;
        }

        qDebug("on_pushButtonGenerateCorFileWRob_clicked - laserDiff(x,y):(%d,%d)",laserDiff.x,laserDiff.y);
    }
    qDebug("robPointOrig(x,y,z): (%f,%f,%f)",robPointOrig.trans.x,robPointOrig.trans.y,robPointOrig.trans.z);

    for(int i=0; i<CorFileSize;i++)
    {
        yPosCurBits = yAxisDownLimitBits + i*CorYaxisStep;

        ///// get LDV pixel bg image only needed when cant turn off the light
        ///
//        Sleep(1000);
//        robot->ldvLaserTurnOn(false);
//        Sleep(1000);
//        specImageProc->greenLaserTrackerWithBgSubtraction(true,0);

        for(int j=0; (j<CorFileSize && ui->pushButtonGenerateCorFileWRob->isChecked()) ;j++)
        {
            qDebug()<<" ---------Counter:  "<<counter;

            xPosCurBits = xAxisLeftLimitBits + j*CorXaxisStep;
            lms->lmsJumpAbs(xPosCurBits,yPosCurBits);
            Sleep(2000);
            specImageProc->greenLaserTrackerWithBgSubtraction(true,1);
            if(j > (CorFileSize/2+2) )
            {
                scanInfo.Current            = "13.6";
                laser->setCurrent();
            }
            else
            {
                scanInfo.Current            = "13.6";
                laser->setCurrent();
            }

            laser->onSHT();

            pixelValueSum = (Point)(0,0);
            for (avgCount = 0;avgCount<5;avgCount++)
            {
                pixelValueInt = (Point)(-1,-1);
                while ( (pixelValueInt.x == -1 || pixelValueInt.y == -1) && ui->pushButtonGenerateCorFileWRob->isChecked())
                {
                    //Sleep(2000);
                    pixelValueInt =  specImageProc->greenLaserTrackerWithBgSubtraction(false,1);
                }

                pixelValueSum += pixelValueInt;

                qDebug(" avgCount: %d - pixelValueSum(x,y): (%d,%d) - pixelValueInt(x,y): (%d,%d)",
                       avgCount,pixelValueSum.x,pixelValueSum.y,pixelValueInt.x,pixelValueInt.y);
            }
            pixelValueFloat = pixelValueSum/avgCount;

            xDistValue = -1*(originPixelValueFloat.x - pixelValueFloat.x)/pixelPerMMx;
            yDistValue = -1*(originPixelValueFloat.y - pixelValueFloat.y)/pixelPerMMy;
            xBitValue  = xPosCurBits;
            yBitValue  = -1*yPosCurBits;

            //time to match the LDV to the point
            laser->offSHT();

            //the origin of robot
            robPoint.trans.x = robPointOrig.trans.x;
            robPoint.trans.y = robPointOrig.trans.y - xDistValue;
            robPoint.trans.z = robPointOrig.trans.z - yDistValue;

            if (specImageProc->isRobPtWihinRange(robPoint.trans))
            {
                robot->moveToTarget(robPoint,false,true);
                Sleep(500);
                robot->moveToTarget(robPoint,false,true);
                Sleep(500);
            }
            laserDiff.x = 10;
            laserDiff.y = 10;
            //while (laserDiff.x != 0 || laserDiff.y != 0)
            while ((abs(laserDiff.x) > 1 || abs(laserDiff.y) > 1) && ui->pushButtonGenerateCorFileWRob->isChecked() )
            {
                if (specImageProc->isRobPtWihinRange(robPoint.trans))
                {
                    robot->moveToTarget(robPoint,false,true);
                    robot->moveToTarget(robPoint,false,true);
                    Sleep(100);
                    robot->moveToTarget(robPoint,false,true);
                    Sleep(500);
                }

                retLdvLaserPointSum = (Point)(0,0);
                for (avgCount = 0;avgCount<5;avgCount++)
                {
                    retLdvLaserPoint  = (Point)(-1,-1);
                    while ((retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1) && ui->pushButtonGenerateCorFileWRob->isChecked() )
                        retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

                    retLdvLaserPointSum += retLdvLaserPoint;

                    qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                           avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
                }
                retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
                qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

                laserDiff.x = retLdvLaserPointValAvg.x - pixelValueFloat.x;
                laserDiff.y = retLdvLaserPointValAvg.y - pixelValueFloat.y;

//                if (laserDiff.x > 0)
//                    robPoint.trans.y += laserDiff.x*0.5;
//                else if (laserDiff.x != 0)
//                    robPoint.trans.y -= laserDiff.x*0.5;

//                if (laserDiff.y > 0)
//                    robPoint.trans.z += laserDiff.y*0.5;
//                else if (laserDiff.y != 0)
//                    robPoint.trans.z -= laserDiff.y*0.5;

                if (laserDiff.x != 0)
                {
                    if (abs(laserDiff.x) >2)
                       adjustStep = 1/pixelPerMMx;
                    else
                        adjustStep = 0.25;

                    robPoint.trans.y += laserDiff.x*adjustStep;
                }

                if (laserDiff.y != 0)
                {
                    if (abs(laserDiff.y) >2)
                       adjustStep = 1/pixelPerMMx;
                    else
                        adjustStep = 0.25;

                    robPoint.trans.z += laserDiff.y*adjustStep;
                }

                qDebug("on_pushButtonGenerateCorFileWRob_clicked - laserDiff(x,y):(%d,%d)",laserDiff.x,laserDiff.y);
            }

            ////////////////////

            xDistValueRob = (robPointOrig.trans.y - robPoint.trans.y);
            yDistValueRob = (robPointOrig.trans.z - robPoint.trans.z);

            out<< yBitValue <<"\t\t"<< xBitValue<<"\t\t"<< yDistValueRob<<"\t\t" << xDistValueRob<<"\n";
            out1<< yBitValue <<"\t\t"<< xBitValue<<"\t\t"<< robPoint.trans.y<<"\t\t" << robPoint.trans.z<<"\n";

            qDebug()<<" xPosCurBits:  "<<xPosCurBits<<" yPosCurBits: "<<yPosCurBits;
            qDebug()<<" pixelValueFloat.x "<<pixelValueFloat.x<<" pixelValueFloat.y "<<pixelValueFloat.y;
            qDebug()<<" xBitValue: "<<xBitValue<<"yBitValue: "<<yBitValue;
            qDebug()<<" xDistValue:  "<<xDistValue<<" yDistValue:  "<<yDistValue;
            qDebug("robPoint(x,y,z): (%f,%f,%f)",robPoint.trans.x,robPoint.trans.y,robPoint.trans.z);
            qDebug()<<" xDistValueRob:  "<<xDistValueRob<<" yDistValueRob:  "<<yDistValueRob;

            counter++;
            mFile.flush();
        }
        out<<"\n\n";
        out1<<"\n\n";
    }

    //wrap up
    laser->offSHT();

    scanInfo.enableOrientControl = localRecord;

    mFile.close();

    qDebug() << "on_pushButtonGenerateCorFileWRob_clicked took:" << (double)timer.elapsed()/(double)60000 << "minutes";
}

void MainWindow::on_pushButtonCalibrateOrient_clicked()
{
    Point originPixelValueInt,originPixelValueSum;
    Point2f originPixelValueFloat;

    structRobTarget robPointOrig;
    structRobTarget robPoint;
    int angle = 0;
    Point retLdvLaserPoint,retLdvLaserPointSum;
    Point2f retLdvLaserPointValAvg;
    Point laserDiff;
    Point3f calibPt;
    float adjustStep;
    int avgCount;
    bool firstTime = true;
    float totalAdjustForZeroAngle = 0;


    calibPt = specImageProc->clickedPt;

    calibPt.x = 85;
    calibPt.y = 161.5;
    calibPt.z = 617;

    QElapsedTimer timer;
    timer.start();
    on_pushButtonConfigCorFileParameters_clicked();
    //take a BG snapshot
    specImageProc->greenLaserTrackerWithBgSubtraction(true,1); // 1 for excitation laser tracking
    //laser control
    scanInfo.Current            = "13.5";
    scanInfo.PRF                = "50";
    laser->setPRF();
    laser->setCurrent();


    robot->ldvLaserTurnOn(false); //remove ldv laser from the picture
    Sleep(500);
    robot->ldvLaserTurnOn(false); //remove ldv laser from the picture

    specImageProc->greenLaserTrackerWithBgSubtraction(true,1); // 1 - take a BG image for excitation laser tracking.

    specImageProc->kinToRobUnits(calibPt,robPoint,false);
    structRobTarget robTarget;
    robTarget.trans.x = robPoint.trans.x;
    robTarget.trans.y = robPoint.trans.y;
    robTarget.trans.z = robPoint.trans.z;

    lms->lmsClickToPointPos(robTarget);

    laser->onSHT();

//avg 5 for finding grn laser

    for (avgCount = 0;avgCount<5;avgCount++)
    {
        originPixelValueInt  = (Point)(-1,-1);
        while (originPixelValueInt.x == -1 || originPixelValueInt.y == -1)
            originPixelValueInt = specImageProc->greenLaserTrackerWithBgSubtraction(false,1);

        originPixelValueSum += originPixelValueInt;

        qDebug(" avgCount: %d - originPixelValueSum(x,y): (%d,%d) - originPixelValueInt(x,y): (%d,%d)",
               avgCount,originPixelValueSum.x,originPixelValueSum.y,originPixelValueInt.x,originPixelValueInt.y);
    }
    originPixelValueFloat = originPixelValueSum / avgCount;
    qDebug()<<"originPixelValueFloat.x: "<<originPixelValueFloat.x<<"originPixelValueFloa.yt:  "<<originPixelValueFloat.y;

    laser->offSHT();

    //LDV positioning and calibration loop
    specImageProc->greenLaserTrackerWithBgSubtraction(true,0); // 1 - take a BG image for ldv laser tracking.


    /////////////////////first sync without angle - strictly speaking the excitation laser should have been moved while keeping the LDV stationary
    /// total adjust offset should be reflected in the ROB_TO_LMS_OFFSET_X in the LMS class
    /// Ideally this should find 0 pixel difference even on its first run.
    ///// get LDV pixel value
    laserDiff.x = 10;
    laserDiff.y = 10;

    specImageProc->kinToRobUnits(calibPt,robPointOrig,0,0,false);

    //while (laserDiff.x != 0 || laserDiff.y != 0)
    while(abs(laserDiff.x) > 0)
    {
        if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
        {
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
        }

        if (firstTime)
        {
            ldv->setAutoFocus();
            Sleep(10000);
//                ldv->setAutoFocus();
//                Sleep(10000);
            firstTime = false;
        }

        retLdvLaserPointSum = (Point)(0,0);
        for (avgCount = 0;avgCount<5;avgCount++)
        {
            retLdvLaserPoint  = (Point)(-1,-1);
            while (retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1)
                retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

            retLdvLaserPointSum += retLdvLaserPoint;

            qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                   avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
        }
        retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
        qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

        laserDiff.x = retLdvLaserPointValAvg.x - originPixelValueFloat.x;
        laserDiff.y = retLdvLaserPointValAvg.y - originPixelValueFloat.y;

        qDebug("on_pushButtonGenerateCorFileWRob_clicked - Before Adjustment Make this correction in LMS class - "
               "laserDiff(x,y):(%d,%d), robPointOrig.transAfterAngle.y: %f",laserDiff.x,laserDiff.y,robPointOrig.transAfterAngle.y);

        if (laserDiff.x != 0)
        {
            if (abs(laserDiff.x) >2)
               adjustStep = 0.5;
            else
                adjustStep = 0.25;

            if (laserDiff.x<0)
            {
                robPointOrig.transAfterAngle.y      -= adjustStep;
                totalAdjustForZeroAngle             += adjustStep;
            }
            else
            {
                robPointOrig.transAfterAngle.y      += adjustStep;
                totalAdjustForZeroAngle             += adjustStep;
            }
        }
        qDebug("on_pushButtonGenerateCorFileWRob_clicked - After Adjustment Make this correction in LMS class - "
               "laserDiff(x,y):(%d,%d), robPointOrig.transAfterAngle.z: %f, totalAdjustForZeroAngle :%f "
               "(make ROB_TO_LMS_OFFSET_X - totalAdjustForZeroAngle)",laserDiff.x,laserDiff.y,robPointOrig.transAfterAngle.z,totalAdjustForZeroAngle);
    }

    //angle = 30;
    ////////////////////for case with angle
    for (angle = 40;angle > -25;angle=angle-10)
    {
        qDebug()<<"Angle: "<<angle;
        firstTime = true;

        //robot->ldvLaserTurnOn(true);

        //robPointOrig = robot->getPos();

//        if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
//        {
//            robot->moveToTarget(robPointOrig,true,true);
//            Sleep(2000);
//        }

        ///// get LDV pixel value
        laserDiff.x = 10;
        laserDiff.y = 10;

        //while (laserDiff.x != 0 || laserDiff.y != 0)
        while(abs(laserDiff.x) > 0)
        {
            specImageProc->kinToRobUnits(calibPt,robPointOrig,angle,0,false);
            if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
            {
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
            }

            if (firstTime)
            {
                ldv->setAutoFocus();
                Sleep(10000);
//                ldv->setAutoFocus();
//                Sleep(10000);
                firstTime = false;
            }

            retLdvLaserPointSum = (Point)(0,0);
            for (avgCount = 0;avgCount<5;avgCount++)
            {
                retLdvLaserPoint  = (Point)(-1,-1);
                while (retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1)
                    retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

                retLdvLaserPointSum += retLdvLaserPoint;

                qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                       avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
            }
            retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
            qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

            laserDiff.x = retLdvLaserPointValAvg.x - originPixelValueFloat.x;
            laserDiff.y = retLdvLaserPointValAvg.y - originPixelValueFloat.y;

            if (laserDiff.x != 0)
            {
                if (abs(laserDiff.x) >2)
                   adjustStep = 0.5;
                else
                    adjustStep = 0.25;

                if (angle < 0) // opposite adjustment for negative angles
                        adjustStep *=-1;

                if (laserDiff.x <0 )
                    specImageProc->calibData.kinCordOffset.z += adjustStep;
                else
                    specImageProc->calibData.kinCordOffset.z -= adjustStep;

            }
            qDebug("on_pushButtonGenerateCorFileWRob_clicked - laserDiff(x,y):(%d,%d), calibData.kinCordOffset.z: %f",laserDiff.x,laserDiff.y,specImageProc->calibData.kinCordOffset.z);
        }
    }
}

void MainWindow::on_pushButtonCalibrateOrientY_clicked()
{
    Point originPixelValueInt,originPixelValueSum;
    Point2f originPixelValueFloat;

    structRobTarget robPointOrig;
    structRobTarget robPoint;
    int angle = 0;
    Point retLdvLaserPoint,retLdvLaserPointSum;
    Point2f retLdvLaserPointValAvg;
    Point laserDiff;
    Point3f calibPt;
    float adjustStep;
    int avgCount;
    bool firstTime = true;
    float totalAdjustForZeroAngle = 0;

    calibPt = specImageProc->clickedPt;
    calibPt.x = 45;
    calibPt.y = 161.5;
    calibPt.z = 617;

    QElapsedTimer timer;
    timer.start();
    on_pushButtonConfigCorFileParameters_clicked();
    //take a BG snapshot
    specImageProc->greenLaserTrackerWithBgSubtraction(true,1); // 1 for excitation laser tracking
    //laser control
    scanInfo.Current            = "13.5";
    scanInfo.PRF                = "50";
    laser->setPRF();
    laser->setCurrent();


    robot->ldvLaserTurnOn(false); //remove ldv laser from the picture
    Sleep(500);
    robot->ldvLaserTurnOn(false); //remove ldv laser from the picture

    specImageProc->greenLaserTrackerWithBgSubtraction(true,1); // 1 - take a BG image for excitation laser tracking.

    specImageProc->kinToRobUnits(calibPt,robPoint,false);
    structRobTarget robTarget;
    robTarget.trans.x = robPoint.trans.x;
    robTarget.trans.y = robPoint.trans.y;
    robTarget.trans.z = robPoint.trans.z;

    lms->lmsClickToPointPos(robTarget);

    laser->onSHT();

//avg 5 for finding grn laser

    for (avgCount = 0;avgCount<5;avgCount++)
    {
        originPixelValueInt  = (Point)(-1,-1);
        while (originPixelValueInt.x == -1 || originPixelValueInt.y == -1)
            originPixelValueInt = specImageProc->greenLaserTrackerWithBgSubtraction(false,1);

        originPixelValueSum += originPixelValueInt;

        qDebug(" avgCount: %d - originPixelValueSum(x,y): (%d,%d) - originPixelValueInt(x,y): (%d,%d)",
               avgCount,originPixelValueSum.x,originPixelValueSum.y,originPixelValueInt.x,originPixelValueInt.y);
    }
    originPixelValueFloat = originPixelValueSum / avgCount;
    qDebug()<<"originPixelValueFloat.x: "<<originPixelValueFloat.x<<"originPixelValueFloa.yt:  "<<originPixelValueFloat.y;

    laser->offSHT();

    //LDV positioning and calibration loop
    specImageProc->greenLaserTrackerWithBgSubtraction(true,0); // 1 - take a BG image for ldv laser tracking.


    /////////////////////first sync without angle - strictly speaking the excitation laser should have been moved while keeping the LDV stationary
    /// total adjust offset should be reflected in the ROB_TO_LMS_OFFSET_X in the LMS class
    /// Ideally this should find 0 pixel difference even on its first run.
    ///// get LDV pixel value
    laserDiff.x = 10;
    laserDiff.y = 10;

    specImageProc->kinToRobUnits(calibPt,robPointOrig,0,0,false);

    //while (laserDiff.x != 0 || laserDiff.y != 0)
    while(abs(laserDiff.y) > 0)
    {
        if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
        {
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
            robot->moveToTarget(robPointOrig,true,true);
            Sleep(100);
        }

        if (firstTime)
        {
            ldv->setAutoFocus();
            Sleep(10000);
//                ldv->setAutoFocus();
//                Sleep(10000);
            firstTime = false;
        }

        retLdvLaserPointSum = (Point)(0,0);
        for (avgCount = 0;avgCount<5;avgCount++)
        {
            retLdvLaserPoint  = (Point)(-1,-1);
            while (retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1)
                retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

            retLdvLaserPointSum += retLdvLaserPoint;

            qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                   avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
        }
        retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
        qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

        laserDiff.x = retLdvLaserPointValAvg.x - originPixelValueFloat.x;
        laserDiff.y = retLdvLaserPointValAvg.y - originPixelValueFloat.y;

        qDebug("on_pushButtonGenerateCorFileWRob_clicked - Before Adjustment Make this correction in LMS class - laserDiff(x,y):(%d,%d), robPointOrig.transAfterAngle.z: %f",laserDiff.x,laserDiff.y,robPointOrig.transAfterAngle.z);

        if (laserDiff.y != 0)
        {
            if (abs(laserDiff.y) >2)
               adjustStep = 0.5;
            else
                adjustStep = 0.25;

            if (laserDiff.y<0)
            {
                robPointOrig.transAfterAngle.z  -= adjustStep;
                totalAdjustForZeroAngle         -= adjustStep;
            }
            else
            {
                robPointOrig.transAfterAngle.z  += adjustStep;
                totalAdjustForZeroAngle         += adjustStep;
            }
        }

        qDebug("on_pushButtonGenerateCorFileWRob_clicked - After Adjustment Make this correction in LMS class - laserDiff(x,y):(%d,%d), robPointOrig.transAfterAngle.z: %f, totalAdjustForZeroAngle :%f (make ROB_TO_LMS_OFFSET_Y + totalAdjustForZeroAngle)",laserDiff.x,laserDiff.y,robPointOrig.transAfterAngle.z,totalAdjustForZeroAngle);
    }

    //angle = 30;
    ////////////////////for case with angle
    for (angle = 40;angle > -1;angle=angle-10)
    {
        qDebug()<<"Angle: "<<angle;
        firstTime = true;

        //robot->ldvLaserTurnOn(true);

        //robPointOrig = robot->getPos();

//        if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
//        {
//            robot->moveToTarget(robPointOrig,true,true);
//            Sleep(2000);
//        }

        ///// get LDV pixel value
        laserDiff.x = 10;
        laserDiff.y = 10;

        //while (laserDiff.x != 0 || laserDiff.y != 0)
        while(abs(laserDiff.y) > 0)
        {
            specImageProc->kinToRobUnits(calibPt,robPointOrig,0,angle,false);
            if (specImageProc->isRobPtWihinRange(robPointOrig.transAfterAngle))
            {
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
                robot->moveToTarget(robPointOrig,true,true);
                Sleep(100);
            }

            if (firstTime)
            {
                ldv->setAutoFocus();
                Sleep(10000);
//                ldv->setAutoFocus();
//                Sleep(10000);
                firstTime = false;
            }

            retLdvLaserPointSum = (Point)(0,0);
            for (avgCount = 0;avgCount<5;avgCount++)
            {
                retLdvLaserPoint  = (Point)(-1,-1);
                while (retLdvLaserPoint.x == -1 || retLdvLaserPoint.y == -1)
                    retLdvLaserPoint = specImageProc->greenLaserTrackerWithBgSubtraction(false,0);

                retLdvLaserPointSum += retLdvLaserPoint;

                qDebug(" avgCount: %d - retLdvLaserPointSum(x,y): (%d,%d) - retLdvLaserPoint(x,y): (%d,%d)",
                       avgCount,retLdvLaserPointSum.x,retLdvLaserPointSum.y,retLdvLaserPoint.x,retLdvLaserPoint.y);
            }
            retLdvLaserPointValAvg = retLdvLaserPointSum / avgCount;
            qDebug()<<"retLdvLaserPointValAvg.x: "<<retLdvLaserPointValAvg.x<<"retLdvLaserPointValAvg.y:  "<<retLdvLaserPointValAvg.y;

            laserDiff.x = retLdvLaserPointValAvg.x - originPixelValueFloat.x;
            laserDiff.y = retLdvLaserPointValAvg.y - originPixelValueFloat.y;

            if (laserDiff.y != 0)
            {
                if (abs(laserDiff.y) >2)
                   adjustStep = 0.5;
                else
                    adjustStep = 0.25;

                if (angle < 0) // opposite adjustment for negative angles
                        adjustStep *=-1;

                if (laserDiff.y <0 )
                    specImageProc->calibData.kinCordOffset.z += adjustStep;
                else
                    specImageProc->calibData.kinCordOffset.z -= adjustStep;
            }
            qDebug("on_pushButtonGenerateCorFileWRob_clicked - laserDiff(x,y):(%d,%d), calibData.kinCordOffset.z: %f",laserDiff.x,laserDiff.y,specImageProc->calibData.kinCordOffset.z);
        }
    }
}

void MainWindow::on_pushButtonGenerateCorFileWRob_toggled(bool checked)
{
    if (checked)
        on_pushButtonGenerateCorFileWRob();
}


