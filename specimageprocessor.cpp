#include "specimageprocessor.h"

//#define ENABLE_SPECPROC_DETAIL_LOGS

#define ROUND(x) round(x*10)/10

//Distance from laserhead end to specimen = 528
//Distance from camera to specimen = 515
// Resolution 640x480 - some error seen in calcs
//#define PXPERMMX  1.243
//#define PXPERMMY  1.243

//Resolution 1280x720 - quite exact
#define PXPERMMX  1.862295
#define PXPERMMY  1.862295
#define LASEROFFSETX  796
#define LASEROFFSETY  328

//KINECT
//#define FLIPIMAGE
#define ROBOTRANGEXMAX 200 //depth
#define ROBOTRANGEXMIN 210 //depth // 210 for the calibration file generation
#define ROBOTRANGEY 500
#define ROBOTRANGEZ 450

specImageProcessor::specImageProcessor(structDaq *daqInfoPtrArg, structScan *scanInfoPtrArg,lmsController *lmsPtrArg,
                                       QString ImgPath_arg, QWidget *widgetInteractive_arg, robotController* robContArg,
                                       QLabel *labelProcessImageDebug_arg)
{
    if(widgetInteractive_arg == NULL)
        return;
    ImgPath = ImgPath_arg;

    colorTableSize = 5;
    colorTable.push_back(Vec4b((uchar)255, (uchar)0, (uchar)0,(uchar)255)); // Blue
    colorTable.push_back(Vec4b((uchar)0, (uchar)255, (uchar)0,(uchar)255)); // Green
    colorTable.push_back(Vec4b((uchar)0, (uchar)0, (uchar)255,(uchar)255)); // Red
    colorTable.push_back(Vec4b((uchar)255, (uchar)255, (uchar)0,(uchar)255));
    colorTable.push_back(Vec4b((uchar)255, (uchar)0, (uchar)255,(uchar)255));

    scanInfoPtr         = scanInfoPtrArg;
    daqInfoPtr          = daqInfoPtrArg;
    blurKerSz           = 2;
    cannyThLo           = 45;
    cannyThHi           = 90;
    closeKerItr         = 9;
    closeKerSz          = 9;
    zoomFactor          = 0;
    pxPermm             = new Point2f (PXPERMMX,PXPERMMY);
    stageOffset         = new Point2f((float)220.0,(float)90.0) ; // Current position of stage, Mapped to bottom left corner of image.
    preSelectShapeIndex     = 2;
    //preSelectShapePoint   = Point(719,120);
    //preSelectShapePoint   = Point(635,210);
    //preSelectShapePoint   = Point(270,180);

    //preSelectShapePoint   = Point(876,210);
    preSelectShapePoint     = Point(975,125);
    widgetInteractive       = widgetInteractive_arg;
    labelProcessImageDebug  = labelProcessImageDebug_arg;
    pMultiSourceFrameReader = NULL;
    pMultiSourceFrame   = NULL;
    pDepthFrame         = NULL;
    pColorFrame         = NULL;
    pLongIRFrame        = NULL;
    pColor2depth        = NULL;
    pDepth2xyz          = NULL;
    pColor2xyz          = NULL;
    pColor2xyzAvg       = NULL;
    imageType           = COL_DEP_IMAGE;
    robHomeclPx         = Point2i(0,0);
    robBotRightclPx     = Point2i(0,0);
    robRangeDrawn       = true;
    refreshInterval     = 500;
    lmsPtr              = lmsPtrArg;
    originDrawn         = true;
    loadKinFusCloud     = false;
    useKinect           = false;
    useAzure            = false;
    azureHandle         = NULL;

#ifdef CALCNORMALS
    normCalcMethod                  = 3;
    normSmoothingSize               = 15.0;//6.0;
    maxDepthChangeFactor            = 1;
    enableDepthDependentSmoothing   = false;
    phiMaxAngle                     = 20;
    thetaMaxAngle                   = 20;
    smoothKernelSize                = 25;
    smoothSigmaColour               = 40;
    smoothSigmaSpace                = 40;
#endif

    // Offset in Pixels considering top-left origin.
    // re-mapping using the position should be done before conversion to mm
    laserOffset = new Point2f(LASEROFFSETX,LASEROFFSETY);
    robotCont   = robContArg;

    //widgetInteractive = new QWidget();
    //widgetInteractive->show();

    selectedContour = 0;
    numOfCountours  = 0;
    maxContOnly     = true;

#ifdef ACTUALSYSTEM
    if ((useAzure = initAzure()) == false)
    {
        if ((useKinect = initKinect()) == false)
        {
            // Open OpenCV capture for webcam
            camHndl.open(0);
            if (camHndl.isOpened())
            {
                camHndl.set(CAP_PROP_FRAME_WIDTH,1920);
                camHndl.set(CAP_PROP_FRAME_HEIGHT,1080);
                camHndl.set(CAP_PROP_FPS,60);

                qDebug() <<
                    ": width=" << camHndl.get(CAP_PROP_FRAME_WIDTH) <<
                    ", height=" << camHndl.get(CAP_PROP_FRAME_HEIGHT) <<
                    ", fps=" << camHndl.get(CAP_PROP_FPS)<<
                    ", GUID="<<camHndl.get(CAP_PROP_AUTOFOCUS);
            }
        }
        else
        {//Kinect image dimensions
            depthWidth  = 512;
            depthHeight = 424;

            colorWidth  = 1920;
            colorHeight = 1080;

            flipImage = true;
        }
    }
    else
    {
        flipImage = false;
    }
#endif
    //QTimer::singleShot(1000, this, SLOT(edgeDetection() ));

    labelOrigImage = new HLabel(widgetInteractive);
    labelOrigImage->setMouseTracking(true);
    labelOrigImage->showMaximized();

    //labelOrigImage->setGeometry(widgetInteractive->rect());
    //labelOrigImage->updateGeometry();
    //labelOrigImage->aut
    connect(labelOrigImage,SIGNAL(clicked(QMouseEvent*)),this,SLOT(mousePressEvent(QMouseEvent*)));
    connect(labelOrigImage,SIGNAL(moved(QMouseEvent*)),this,SLOT(mouseMoveEvent(QMouseEvent*)));
    connect(labelOrigImage,SIGNAL(wheelMove(QWheelEvent*)),this,SLOT(mouseWheelEvent(QWheelEvent*)));
#ifdef ACTUALSYSTEM
    //measLaserCenter();
#endif

    ASScanUpdateTimer = new QTimer(this);
    connect(ASScanUpdateTimer,SIGNAL(timeout()),this,SLOT(edgeDetection())); // triggered from the main window via refresh radio box
    //ASScanUpdateTimer->start(5000);//100ms update

    //following kinect axis notation
    //specImageProcessor::laserTrackerRobotArm-
    //laserPointPx:(559,286) -
    //calibData.kinCordOffset.x: -13.858673 -
    //calibData.kinCordOffset.y: -129.475769 -
    //calibData.kinCordOffset.z: -242.000000 -
    //calibData.kinPt.z: 1181.000122

//    //Kinect to the left of robot arm on 2 brackets
//    calibData.kinCordOffset.x = -206;//-12; //19
//    calibData.kinCordOffset.y = -370;//-114; //-133
//    calibData.kinCordOffset.z = -99;//-247; //241
//    calibData.kinPt.z         = 1132;//967; // stores the depth at whcih this calibration was performed.

    //Kinect to the left of robot arm on 3 brackets
//    calibData.kinCordOffset.x = -211;//-12; //19
//    calibData.kinCordOffset.y = -470;//-114; //-133
//    calibData.kinCordOffset.z = -105;//-247; //241
//    calibData.kinPt.z         = 1125;//967; // stores the depth at whcih this calibration was performed.

//    //    //Kinect to the left of robot arm on 2 brackets depth reduced
//        calibData.kinCordOffset.x = -231;//-12; //19
//        calibData.kinCordOffset.y = -337;//-114; //-133
//        calibData.kinCordOffset.z = -141;//-247; //241
//        calibData.kinPt.z         = 743;//967; // stores the depth at whcih this calibration was performed.

    //    //Kinect to the left of robot arm on 2 brackets depth reduced
//    calibData.kinCordOffset.x = -267;//-12; //19
//    calibData.kinCordOffset.y = -263;//-114; //-133

    //for Al guided wave experiment 9OCT
//        calibData.kinCordOffset.x = -268;//-12; //19 // increasing the - value moves the LDV leftwards
//        calibData.kinCordOffset.y = -258;//-114; //-133 // increasing the - value moves the LDV upwards


    calibData.kinCordOffset.x = -308;//-12; //19 // increasing the - value moves the LDV leftwards
    calibData.kinCordOffset.y = -262;//-114; //-133 // increasing the - value moves the LDV upwards

     //ADD:5~6 to the calibration value to align the depth and TCP point. So that the laser point doesnot move after angle adjustment.
    //e.g -144 becomes -150
    calibData.kinCordOffset.z = -131;//-137  // increasing - value moves LDV rightwards for negative yaw  (ldv looking leftwards)
    //calibData.kinCordOffset.z = -135;//-137 // increasing - value moves LDV upwards for positive pitch (ldv looking downwards)
    //if LDV moves to the left of excitation with
    calibData.kinPt.z         = 727;//967; // stores the depth at whcih this calibration was performed.

    depImgSumMat.create(depthHeight,depthWidth,CV_64FC1);
    depImgSumMat = Scalar::all(0);
    depImgAvgMat.create(depthHeight,depthWidth,CV_32FC1);
    depImgAvgMat = Scalar::all(0);
    depImgAvgDisMat.create(depthHeight,depthWidth,CV_32FC1);
    depImgAvgDisMat = Scalar::all(0);
    depImgAvgCnt = 0;

    normalAngles.create(depImgAvgMat.size(), CV_32FC3);
    depImgAvgMatFliped.create(depthHeight,depthWidth,CV_32FC1);
    depImgAvgMatFliped = Scalar::all(0);
    thetaMat.create(depImgAvgMatFliped.size(), CV_16SC1);
    phiMat.create(depImgAvgMatFliped.size(), CV_16SC1);
    thetaFiltMat.create(depImgAvgMatFliped.size(), CV_16SC1);
    phiFiltMat.create(depImgAvgMatFliped.size(), CV_16SC1);

    pDepthBufferAvg = (unsigned short* )new unsigned short[depthWidth * depthHeight];
    pDepthBuffer    = (unsigned short* )new unsigned short[depthWidth * depthHeight];

    pColor2xyz     =  new CameraSpacePoint[colorWidth * colorHeight];
    pColor2xyzAvg  =  new CameraSpacePoint[colorWidth * colorHeight];

}

specImageProcessor::~specImageProcessor()
{
    widgetInteractive->deleteLater();
    if (useAzure)
    {
        // Shut down the camera when finished with application logic
        k4a_device_stop_cameras(azureHandle);
        k4a_device_close(azureHandle);
    }
    else
    {
        SAFE_RELEASE(pMultiSourceFrameReader);
    }
}

void specImageProcessor::changeImageType()
{
    if (imageType == 2)
        imageType = 3;
    else if (imageType == 3)
        imageType = 4;
    else if (imageType == 4)
        imageType = 2;

    //imageType += 1;
    //imageType = imageType%4;
    qDebug()<<imageType;
}
void specImageProcessor::enASScanUpdateTimer(bool enable)
{
    if (enable == true && ASScanUpdateTimer->isActive() == false)
        ASScanUpdateTimer->start(refreshInterval);
    else if (enable == false && ASScanUpdateTimer->isActive() == true)
        ASScanUpdateTimer->stop();
}

void specImageProcessor::setRefreshInterval(QString refIntervalStr)
{
    this->refreshInterval =  refIntervalStr.toInt();
    ASScanUpdateTimer->start(refreshInterval);
}

void specImageProcessor::setCannyThLo(int cannyThLo_arg)
{
    cannyThLo = cannyThLo_arg;
    edgeDetection();
}

void specImageProcessor::setCannyThHi(int cannyThHi_arg)
{
    cannyThHi = cannyThHi_arg;
    edgeDetection();
}

void specImageProcessor::setBlurKerSz(int blurKerSz_arg)
{
    blurKerSz = blurKerSz_arg;
    edgeDetection();
}

void specImageProcessor::setCloseKerSz(int closeKerSz_arg)
{
    closeKerSz = closeKerSz_arg;
    edgeDetection();
}

void specImageProcessor::setCloseKerItr(int closeKerItr_arg)
{
    closeKerItr = closeKerItr_arg;
    edgeDetection();
}

void specImageProcessor::incZoom()
{
    zoomFactor++;
}

void specImageProcessor::decZoom()
{
    if (zoomFactor>0)
        zoomFactor--;
}

void specImageProcessor::mouseWheelEvent(QWheelEvent *ev)
{

    if((ev->delta()/120)<0)
        preSelectShapeIndex++;
    else
        if (preSelectShapeIndex > -1)
            preSelectShapeIndex--;

    qDebug()<<"specImageProcessor::mouseWheelEvent: angleDelta"<<ev->delta()/120<<
            "preSelectShapeIndex"<<preSelectShapeIndex;
}
void specImageProcessor::mouseMoveEvent(QMouseEvent *ev)
{
    QString toolTipStr;
    cv::Point2f outPoint;
    structRobTarget robPoint;
    int theta,phi;
#ifndef ACTUALSYSTEM
    //convToStageUnits(cv::Point2f(ev->pos().x(),ev->pos().y()),outPoint);
    //toolTipStr = QString(("Orignal(%1,%2), Converted(%3,%4)")).arg(ev->pos().x()).arg(ev->pos().y()).arg(outPoint.x).arg(outPoint.y); // original Coordinates
#endif
    int x = ev->pos().x();
    int y = ev->pos().y();
    int xKinectData = x;

    //for depth
    if (flipImage)
        xKinectData = (depthWidth - 1) - x;

     int depthIndex = xKinectData + (y * depthWidth);

    if (imageType == DEP_IMAGE && pDepthBuffer !=NULL) // depth image
    {
        if (useAzure)
        {
            depthIndex = x+(y*colorWidth);
            Vec4b rgbaPixel = origCvImg.at<Vec4b>(y,x);
            BYTE red = rgbaPixel[0];
            BYTE grn = rgbaPixel[1];
            short grey16 =  (short)grn<<8 | (short)red;

            toolTipStr = QString(("depth(%1,%2) = %3 (%4)")).arg(ev->pos().x()).arg(ev->pos().y()).arg(grey16).arg(pDepthBuffer[depthIndex]); // original Coordinates []
        }
        else if (useKinect)
        {
            Vec3f normalVect;
            Vec2f normalAngleVect;

            Vec4b rgbaPixel = origCvImg.at<Vec4b>(y,x);
            if (normals.cols != 0)
            {
                normalVect = normals.at<Vec3f>(y,xKinectData);
                normalAngleVect = normalAngles.at<Vec2f>(y,xKinectData);
            }

            BYTE red = rgbaPixel[0];
            BYTE grn = rgbaPixel[1];

            short grey16 =  (short)grn<<8 | (short)red;

            toolTipStr = QString(("depth(%1,%2) = %3 (%4)")).arg(ev->pos().x()).arg(ev->pos().y()).arg(grey16).arg(pDepthBuffer[depthIndex]); // original Coordinates []
            if (normals.cols != 0)
                toolTipStr += QString((" - normalVec(-dzdx,-dzdy,1) = (%1,%2,%3) , angles(phi,theta) = (%4,%5)")).arg(normalVect[0]).arg(normalVect[1]).arg(normalVect[2]).arg(normalAngleVect[0]).arg(normalAngleVect[1]);
        }
    }


    if (imageType == IR_DEP_IMAGE && pDepthBuffer !=NULL && pDepth2xyz != NULL) //depth image
    {
        Vec4b rgbaPixel = origCvImg.at<Vec4b>(ev->pos().y(),ev->pos().x());
        BYTE red = rgbaPixel[0];
        BYTE grn = rgbaPixel[1];

        short grey16 =  (short)grn<<8 | (short)red;
        toolTipStr = QString(("depth(%1,%2) = %3 ")).arg(ev->pos().x()).arg(ev->pos().y()).arg(pDepthBuffer[depthIndex]); // original Coordinates []
        toolTipStr += QString((" - camera(%1,%2) = (%3,%4,%5) ")).arg(x).arg(y).arg(pDepth2xyz[depthIndex].X).arg(pDepth2xyz[depthIndex].Y).arg(pDepth2xyz[depthIndex].Z);
    }

    if (imageType == COL_DEP_IMAGE)
    {
        toolTipStr.clear();
        for (int k = 0;k<5;k++)
        {
            y = ev->pos().y() - k;
            if (y<0) break;
            int colorIndex = x+(y*colorWidth);
            if (useAzure)
            {
                toolTipStr += QString(("(x,y) = (%1,%2): depth = %3 - depthAvg = %4")).arg(x).arg(y).arg(pDepthBufferAvg[colorIndex]).arg(depImgAvgMat.at<float>(y,x));
                toolTipStr += QString((" - camera = (%3,%4,%5) ")).arg(pColor2xyz[colorIndex].X).arg(pColor2xyz[colorIndex].Y).arg(pColor2xyz[colorIndex].Z);
                toolTipStr += QString((" - spherical(r,phi,theta) = (%1,%2,%3) ")).arg(normalAngles.at<Vec3f>(y,x)[0]).arg(normalAngles.at<Vec3f>(y,x)[1]).arg(normalAngles.at<Vec3f>(y,x)[2]);

                //toolTipStr += QString(("-theta: %1 ")).arg(this->thetaMat.at<short>(depthY,(depthWidth - 1)-depthX));
                //toolTipStr += QString(("-theta_filtered: %1 ")).arg(this->phiMat.at<short>(depthY,(depthWidth - 1)-depthX));

                //angle = thetaFiltMat.at<short>(depthY,(depthWidth - 1)-depthX);

                pColorToAngle(x,y,phi,theta);

                kinToRobUnits(Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,pColor2xyz[colorIndex].Z*1000),robPoint,theta,phi);

                toolTipStr += QString((" - (phi,theta) = (%2,%3) (q1,q2,q3,q4) = (%4,%5,%6,%7)")).arg(phi).arg(theta).arg(robPoint.rot.q1).arg(robPoint.rot.q2).arg(robPoint.rot.q3).arg(robPoint.rot.q4) ;
                toolTipStr += QString((" - robPoint.trans(y,z,x) = (%1,%2,%3)")).arg(robPoint.trans.y).arg(robPoint.trans.z).arg(robPoint.trans.x);
                toolTipStr += QString((" - robPoint.transAfterAngle(y,z,x) = (%1,%2,%3)\n")).arg(robPoint.transAfterAngle.y).arg(robPoint.transAfterAngle.z).arg(robPoint.transAfterAngle.x);
            }
            else if (useKinect)
            {
                if(flipImage)
                    xKinectData = (colorWidth - 1) - x;

                //int colorIndex = xKinectData*y;
                int colorIndex = xKinectData+((y)*colorWidth);

                if (pColor2depth != NULL && pColor2xyz != NULL)
                {
                    DepthSpacePoint p = pColor2depth[colorIndex];
                    //p.X = 30;
                    //p.Y = 30;
                    if (p.X!=-std::numeric_limits<float>::infinity() &&
                       p.Y!=-std::numeric_limits<float>::infinity())
                    {
                        int depthX = static_cast<int>(p.X + 0.5f);
                        int depthY = static_cast<int>(p.Y + 0.5f);
                        toolTipStr += QString(("(x,y) = (%1,%2): depth = %3 - depthAvg = %4")).arg(ev->pos().x()).arg(ev->pos().y()).arg(pDepthBuffer[depthX + (depthY * depthWidth)]).arg(depImgAvgMat.at<float>(depthY,depthX));
                        toolTipStr += QString((" - camera = (%3,%4,%5) ")).arg(pColor2xyz[colorIndex].X).arg(pColor2xyz[colorIndex].Y).arg(pColor2xyz[colorIndex].Z);
                        toolTipStr += QString((" - cameraAvg(%3,%4,%5) ")).arg(pColor2xyzAvg[colorIndex].X).arg(pColor2xyzAvg[colorIndex].Y).arg(pColor2xyzAvg[colorIndex].Z);
                        toolTipStr += QString((" - spherical(r,phi,theta) = (%1,%2,%3) ")).arg((normalAngles.at<Vec3f>(depthY,(depthWidth - 1)-depthX)[0])).arg((normalAngles.at<Vec3f>(depthY,(depthWidth - 1)-depthX)[1])).arg((normalAngles.at<Vec3f>(depthY,(depthWidth - 1)-depthX)[2]));

                        //toolTipStr += QString(("-theta: %1 ")).arg(this->thetaMat.at<short>(depthY,(depthWidth - 1)-depthX));
                        //toolTipStr += QString(("-theta_filtered: %1 ")).arg(this->phiMat.at<short>(depthY,(depthWidth - 1)-depthX));

                        //angle = thetaFiltMat.at<short>(depthY,(depthWidth - 1)-depthX);

                        pColorToAngle(x,y,phi,theta);

                        kinToRobUnits(Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,pColor2xyz[colorIndex].Z*1000),robPoint,phi,theta);

                        toolTipStr += QString((" - (phi,theta) = (%2,%3) (q1,q2,q3,q4) = (%4,%5,%6,%7)")).arg(phi).arg(theta).arg(robPoint.rot.q1).arg(robPoint.rot.q2).arg(robPoint.rot.q3).arg(robPoint.rot.q4) ;
                        toolTipStr += QString((" - robPoint.trans(y,z,x) = (%1,%2,%3)")).arg(robPoint.trans.y).arg(robPoint.trans.z).arg(robPoint.trans.x);
                        toolTipStr += QString((" - robPoint.transAfterAngle(y,z,x) = (%1,%2,%3)\n")).arg(robPoint.transAfterAngle.y).arg(robPoint.transAfterAngle.z).arg(robPoint.transAfterAngle.x);

                    }
                    else
                        toolTipStr = QString(("depth(%1,%2) = %3 \n")).arg(ev->pos().x()).arg(ev->pos().y()).arg(-std::numeric_limits<float>::infinity());
                }
            }
        }
    }

    //labelOrigImage->setToolTip(toolTipStr);
    //labelOrigImage->setToolTipDuration(10000);
    //toolTipStr = QString(("Orignal(%1,%2)")).arg(ev->pos().x()).arg(ev->pos().y()); // original Coordinates
    QToolTip::showText(ev->screenPos().toPoint(),toolTipStr);
}

void specImageProcessor::mousePressEvent(QMouseEvent *ev)
{
    qDebug()<<"specImageProcessor::mousePressEvent"<<ev->type();
    if(ev->button() == Qt::MouseButton::LeftButton &&
            ev->type() == QEvent::MouseButtonPress)
    {
        // move considering LDV standoff distance and orientation
        if (ev->modifiers() == Qt::CTRL)
        {
            QString debugString;
            if (imageType == COL_DEP_IMAGE)
            {
                int x = ev->pos().x();
                int y = ev->pos().y();

                int colorIndex = ptToDataIdxColor(x,y);
                structRobTarget robPoint;
                int theta,phi;

                if (pColor2depth != NULL && pColor2xyz != NULL)
                {
                    DepthSpacePoint p = pColor2depth[colorIndex];
                    if (p.X!=-std::numeric_limits<float>::infinity() &&
                       p.Y!=-std::numeric_limits<float>::infinity())
                    {
                        int depthX = static_cast<int>(p.X + 0.5f);
                        int depthY = static_cast<int>(p.Y + 0.5f);
                        //Point3f clickedPtAvg;
                        clickedPt       = Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,pColor2xyzAvg[colorIndex].Z*1000);
                        //clickedPtAvg    = Point3f(pColor2xyzAvg[colorIndex].X*1000,pColor2xyzAvg[colorIndex].Y*1000,pColor2xyzAvg[colorIndex].Z*1000);

                        //searchPtTempAvg = mTomm(pColor2xyzAvg[ptToDataIdxColor(clPx.x,clPx.y+pixelIndex)]);

                        pColorToAngle(x,y,phi,theta);
                        kinToRobUnits(clickedPt,robPoint,phi,theta,true);

                        //angle = phiMat.at<short>(depthY,(depthWidth - 1)-depthX);

                        //open this for checking the beam alignments of the two laser beams
//                        if (x>650 && x<750)
//                            kinToRobUnits(clickedPt,robPoint,-30,0,true);
//                        else if (x>750 && x<800)
//                            kinToRobUnits(clickedPt,robPoint,0,30,true);
//                        else if (x>800)
//                            kinToRobUnits(clickedPt,robPoint,30,30,true);


                        //kinToRobUnits(Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,depImgAvgMat.at<float>(depthY,depthX)),robPoint,angle,true);

//                        if (scanInfoPtr->enableDepthControl == false)
//                            robPoint.x = robotCont->homePos.trans.x;


                        debugString = QString(("depth(%1,%2) = %3 - depthAvg: %4")).arg(ev->pos().x()).arg(ev->pos().y()).arg(pDepthBuffer[depthX + (depthY * depthWidth)]).arg(depImgAvgMat.at<float>(depthY,depthX));
                        debugString += QString((" - camera(%1,%2) = (%3,%4,%5) ")).arg(x).arg(y).arg(pColor2xyz[colorIndex].X).arg(pColor2xyz[colorIndex].Y).arg(pColor2xyz[colorIndex].Z);
                        debugString += QString(("robPoint.trans(y,z,x) = (%1,%2,%3)")).arg(robPoint.trans.y).arg(robPoint.trans.z).arg(robPoint.trans.x);
                        debugString += QString(("robPoint.transAfterAngle(y,z,x) = (%1,%2,%3)")).arg(robPoint.transAfterAngle.y).arg(robPoint.transAfterAngle.z).arg(robPoint.transAfterAngle.x);
                        debugString += QString(("angleFromFunc(theta,phi) = (%2,%3) (q1,q2,q3,q4) = (%4,%5,%6,%7)\n ")).arg(theta).arg(phi).arg(robPoint.rot.q1).arg(robPoint.rot.q2).arg(robPoint.rot.q3).arg(robPoint.rot.q4) ;

                        {
                            short *angleValArray  = new short[SAMPLESPERPOINT];
                            for(int i =0; i<depthHeight;i++)
                                angleValArray[i] = phiMat.at<short>(i,(depthWidth - 1)-depthX);

                            emit plotGraph(angleValArray, NULL);
                        }
                        if (isRobPtWihinRange(robPoint.transAfterAngle))
                        {
                            structRobTarget robTarget;
                            robTarget.trans.x = robPoint.trans.x;
                            robTarget.trans.y = robPoint.trans.y;
                            robTarget.trans.z = robPoint.trans.z;

                            lmsPtr->lmsClickToPointPos(robTarget);

                            robotCont->moveToTarget(robPoint,true,true);
                            debugString += QString(" - Moving");
                        }
                        else
                        {
                            debugString += QString(" - Selected point not in Range");
                        }
                    }
                }
            }
            qDebug()<<debugString;
            //specImageProcessor::enASScanUpdateTimer(true);
            return;
        }

        // move considering LDV standoff distance
        if (ev->modifiers() == Qt::ALT)
        {
            bool localRecord = scanInfoPtr->enableOrientControl;
            scanInfoPtr->enableOrientControl = false;
            QString debugString;
            if (imageType == COL_DEP_IMAGE)
            {
                int x = ev->pos().x();
                int y = ev->pos().y();

                int colorIndex = ptToDataIdxColor(x,y);
                structRobTarget robPoint;
                int theta,phi;

                if (pColor2depth != NULL && pColor2xyz != NULL)
                {
                    DepthSpacePoint p = pColor2depth[colorIndex];
                    if (p.X!=-std::numeric_limits<float>::infinity() &&
                       p.Y!=-std::numeric_limits<float>::infinity())
                    {
                        int depthX = static_cast<int>(p.X + 0.5f);
                        int depthY = static_cast<int>(p.Y + 0.5f);
                        pColorToAngle(x,y,phi,theta);

                        //angle = phiMat.at<short>(depthY,(depthWidth - 1)-depthX);

                        clickedPt = Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,pColor2xyz[colorIndex].Z*1000);
                        //clickedPt       = Point3f(pColor2xyzAvg[colorIndex].X*1000,pColor2xyzAvg[colorIndex].Y*1000,pColor2xyzAvg[colorIndex].Z*1000);
                        kinToRobUnits(clickedPt,robPoint,phi,theta,true);
                        //kinToRobUnits(Point3f(pColor2xyz[colorIndex].X*1000,pColor2xyz[colorIndex].Y*1000,depImgAvgMat.at<float>(depthY,depthX)),robPoint,angle,true);

//                        if (scanInfoPtr->enableDepthControl == false)
//                            robPoint.x = robotCont->homePos.trans.x;

                        debugString = QString(("depth(%1,%2) = %3 - depthAvg: %4")).arg(ev->pos().x()).arg(ev->pos().y()).arg(pDepthBuffer[depthX + (depthY * depthWidth)]).arg(depImgAvgMat.at<float>(depthY,depthX));
                        debugString += QString((" - camera(%1,%2) = (%3,%4,%5) ")).arg(x).arg(y).arg(pColor2xyz[colorIndex].X).arg(pColor2xyz[colorIndex].Y).arg(pColor2xyz[colorIndex].Z);
                        debugString += QString(("robPoint.trans(y,z,x) = (%1,%2,%3)")).arg(robPoint.trans.y).arg(robPoint.trans.z).arg(robPoint.trans.x);
                        debugString += QString(("robPoint.transAfterAngle(y,z,x) = (%1,%2,%3)")).arg(robPoint.transAfterAngle.y).arg(robPoint.transAfterAngle.z).arg(robPoint.transAfterAngle.x);
                        debugString += QString(("angleFromFunc(theta,phi) = (%2,%3) (q1,q2,q3,q4) = (%4,%5,%6,%7)\n ")).arg(theta).arg(phi).arg(robPoint.rot.q1).arg(robPoint.rot.q2).arg(robPoint.rot.q3).arg(robPoint.rot.q4) ;

                        {
                            short *angleValArray  = new short[SAMPLESPERPOINT];
                            for(int i =0; i<depthHeight;i++)
                                angleValArray[i] = phiMat.at<short>(i,(depthWidth - 1)-depthX);

                            emit plotGraph(angleValArray, NULL);
                        }

                        if (isRobPtWihinRange(robPoint.transAfterAngle))
                        {
                            structRobTarget robTarget;
                            robTarget.trans.x = robPoint.trans.x;
                            robTarget.trans.y = robPoint.trans.y;
                            robTarget.trans.z = robPoint.trans.z;

                            lmsPtr->lmsClickToPointPos(robTarget);

                            robotCont->moveToTarget(robPoint,true,true);
                            debugString += QString(" - Moving");
                        }
                        else
                        {
                            debugString += QString(" - Selected point not in Range");
                        }
                    }
                }
            }
            scanInfoPtr->enableOrientControl = localRecord;

            qDebug()<<debugString;
            //specImageProcessor::enASScanUpdateTimer(true);
            return;
        }
        if (ev->modifiers() == Qt::SHIFT)
        {
            Point tempPt;
            tempPt = greenLaserTrackerWithBgSubtraction();
            qDebug()<<tempPt.x <<tempPt.y;
        }
        else
        {
            //enASScanUpdateTimer(false);
            //select the contor with the nearest click.
            if (!contours.empty())
            {
                float distance;
                for (size_t i = 0; i<contours.size(); i++)
                {
                    distance = 0 ;
                    distance = pointPolygonTest(contours[i],Point2f((float)ev->pos().x(),(float)ev->pos().y()),true);
                    qDebug("Point(x,y): (%d,%d), Contour #: %d, distance: %f ",ev->pos().x(),ev->pos().y(),i,distance);

                    if (fabs(distance)<20)
                    {
                        selectedContour = i;
                        break;
                    }
                }
            }
            if (selectedContour>-1)
                calcScanGrid();
        }
    }
    else if(ev->button() == Qt::MouseButton::LeftButton &&
               ev->type() == QEvent::MouseButtonDblClick) // re- capture the image and do edge detection
    {
        selectedContour = -1;
        edgeDetection();
    }
    else if(ev->button() == Qt::MouseButton::MidButton && // enter custom polygon
            ev->type() == QEvent::MouseButtonPress)
    {
        if (ev->modifiers() == Qt::CTRL)
        {
            if (numOfCountours > 0)
            {
                contours[(numOfCountours-1)].push_back(Point(ev->pos().x(),(float)ev->pos().y()));
                selectedContour = 0;
            }
        }
        else            
        {
            if (preSelectShapeIndex>-1)
                   preSelectShapePoint = Point(ev->pos().x(),ev->pos().y());
            else
            {
                numOfCountours++;
                vector<Point2i> userContour;
                userContour.push_back(Point(ev->pos().x(),(float)ev->pos().y()));
                contours.push_back(userContour);
            }
        }
    }
    else if(ev->button() == Qt::MouseButton::RightButton && // enter custom polygon
            ev->type() == QEvent::MouseButtonPress)
    {
            contours.clear();
            selectedContour = -1;
            numOfCountours = 0;
    }

    //if (selectedContour >= 0)
        //calParsForStage();

    labelOrigImage->setPixmap(QPixmap::fromImage(drawContoursAndMarkers(ctrImg,false,selectedContour)));
    labelOrigImage->adjustSize();
}
void specImageProcessor::captureImage()
{
    Mat startImg;
    if (camHndl.isOpened())
        camHndl.read(startImg);

    imwrite("d:\\bgImage.bmp",startImg);
}

bool specImageProcessor::initAzure()
{
    bool retVal = false;
    uint32_t count = k4a_device_get_installed_count();
    if(count == 0)
    {
          qDebug() <<" No Azure Kinect Attached. ";
          return retVal;
    }

    // Open the first plugged in Kinect device
    k4a_device_open(K4A_DEVICE_DEFAULT, &azureHandle);

    // Get the size of the serial number
//    size_t serial_size = 0;
//    k4a_device_get_serialnum(azureHandle, NULL, &serial_size);

//    // Allocate memory for the serial, then acquire it
//    char *serial = (char*)(malloc(serial_size));
//    k4a_device_get_serialnum(azureHandle, serial, &serial_size);
//    qDebug() <<" Serial address for Azure Kinect is "<<serial;

    AzureConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    AzureConfig.camera_fps                  = K4A_FRAMES_PER_SECOND_15;
    AzureConfig.color_format                = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    AzureConfig.color_resolution            = K4A_COLOR_RESOLUTION_2160P;
    AzureConfig.depth_mode                  = K4A_DEPTH_MODE_WFOV_UNBINNED;
    AzureConfig.synchronized_images_only    = true;

    zoomFactor = 10;

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(azureHandle, &AzureConfig)))
    {
        qDebug()<<"Failed to start cameras!\n";
        k4a_device_close(azureHandle);
    }

    k4a_capture_t capture_handle;
    //capture a test colour image to ascertian the width and height
    k4a_device_get_capture (azureHandle, &capture_handle, 5000);

    k4a_image_t image_color = NULL;
    image_color = k4a_capture_get_color_image(capture_handle);
    depthWidth  = colorWidth  = k4a_image_get_width_pixels(image_color);
    depthHeight = colorHeight = k4a_image_get_height_pixels(image_color);

    // To acquire sensor calibration
    k4a_calibration_t  sensor_calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(azureHandle, AzureConfig.depth_mode, AzureConfig.color_resolution, &sensor_calibration))
    {
        qDebug()<<"Get depth camera calibration failed!\n";
    }
    else
    {
        // To create a transformation handle
        transformation_handle = k4a_transformation_create(&sensor_calibration);
    }

    k4a_capture_release(capture_handle);

    //distortion solutions
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    // Distortion Coefficeints Values
    distCoeffs.at<double>(0)=   0.103017;
    distCoeffs.at<double>(1)=   -0.0540293;
    distCoeffs.at<double>(2)=   -0.00125169;
    distCoeffs.at<double>(3)=   0.0000729814;
    distCoeffs.at<double>(4)=   0.0206589;


    // Camera Matrix Values
    // Calibration parameters
    //cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix = Mat::ones(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0)=   922.962;
    cameraMatrix.at<double>(0,1)=   0;
    cameraMatrix.at<double>(0,2)=   968.195;
    cameraMatrix.at<double>(1,0)=   0;
    cameraMatrix.at<double>(1,1)=   922.05;
    cameraMatrix.at<double>(1,2)=   577.934;
    cameraMatrix.at<double>(2,0)=   0;
    cameraMatrix.at<double>(2,1)=   0;
    cameraMatrix.at<double>(2,2)=   1;

    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, Size(colorWidth,colorHeight),CV_32FC1 ,undistMap1,undistMap2);


    retVal = true;
    return retVal;
}
bool specImageProcessor::initKinect()
{
    HRESULT hr;
    hr = GetDefaultKinectSensor(&pMyKinect);
    if (hr != S_OK)
    {
        qWarning()<<"Could not get Kinect";
        return 0;
    }

    hr = pMyKinect->Open();
    if (hr != S_OK)
    {
        qWarning()<<"Kinect Open Failed";
        return 0;
    }

    BOOLEAN bIsOpen = false;
    pMyKinect->get_IsOpen(&bIsOpen);
    if (!bIsOpen)
    {
        qDebug()<< "ERROR Kinect open Failed !" << endl;
        return 0;
    }
    qDebug()<< "specImageProcessor::initKinect - Kinect is opening..." << endl;

    //check if available
    BOOLEAN bAvaliable = 0;
    hr = pMyKinect->get_IsAvailable(&bAvaliable);
    if (hr != S_OK)
    {
        qWarning()<<"Kinect is not available";
        return 0;
    }

    int conTrialCounter = 0;

    while (conTrialCounter < 2)
    {
        QThread::msleep(1000);
        hr = pMyKinect->get_IsAvailable(&bAvaliable);
        if (bAvaliable == false)
        {
            qWarning("No Kinect Found! Retrying...\n");
        }
        else
        {
            break;
        }
        conTrialCounter++;
    }

    hr = pMyKinect->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth |
                                               FrameSourceTypes::FrameSourceTypes_Color |
                                               FrameSourceTypes::FrameSourceTypes_Infrared |
                                               FrameSourceTypes::FrameSourceTypes_LongExposureInfrared,
                                               &pMultiSourceFrameReader);

    //QThread::msleep(2000);
    return bAvaliable;
}

//for normalization of IR frame
#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)
#define InfraredOutputValueMinimum 0.01f
#define InfraredOutputValueMaximum 1.0f
#define InfraredSceneValueAverage 0.08f
#define InfraredSceneStandardDeviations 3.0f

//for converting 16bit greyscal images to rgba 8888 format
//RGBA is used by the colour image so depth and IR are also converted to the same.
void specImageProcessor::grey16ToRgb8(unsigned short *src, uchar *dest, int width, int height, bool isDepthData )
{
/*
    Mat m(height, width, CV_16U, src);
    Mat mean,stddev;

    meanStdDev(m,mean,stddev);

    qDebug()<<mean.data[0]<<stddev.data[0];
*/
    const unsigned short* curr = src;
    const unsigned short* dataEnd = curr + (width*height);

    BYTE red;
    BYTE grn;
    BYTE blu;
    unsigned short depth;

    while (curr < dataEnd)
    {
        // Get depth in millimeters
        depth = (*curr++);

        if (isDepthData)
        {
            blu = (BYTE)(depth & 0xff);
            grn = (BYTE)(depth>>8 & 0xff);
            red = 0;
        }
        else
        {
            float intensityRatio = static_cast<float>(depth) / InfraredSourceValueMaximum;

            // 2. dividing by the (average scene value * standard deviations)
            intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

            // 3. limiting the value to InfraredOutputValueMaximum
            intensityRatio = MIN(InfraredOutputValueMaximum, intensityRatio);

            // 4. limiting the lower value InfraredOutputValueMinimym
            intensityRatio = MAX(InfraredOutputValueMinimum, intensityRatio);

            intensityRatio = pow(intensityRatio,0.32); //a trick being used by the Kinect studio

            // 5. converting the normalized value to a byte and using the result
            // as the RGB components required by the image
            byte intensity = static_cast<byte>(intensityRatio * 255.0f);

            red = grn = blu = intensity;
        }

        for (int i = 0; i < 3; ++i)
        {
            //*dest++ = (BYTE)depth % 256;
            if (i == 0)
                *dest++ = blu;
            else if (i == 1)
                *dest++ = grn;
            else
                *dest++ = red;
        }
        *dest++ = 0xff;
    }
}

bool specImageProcessor::getImageFromKinect(Mat *ptrToMat,int imageTypeArg, int depthRange ) // pass a MAT pointer to be populated with the read data.
{
    Mat pLocImage;
    static QElapsedTimer elTimer;
    static int normalCalcCount = 0;

    if (useAzure)
    {
//        k4a_capture_t capture_handle = NULL;
//        k4a_image_t image_color = NULL;
//        k4a_image_t azDepthImage = NULL;
//        k4a_image_t azDepthTransImage = NULL;
//        k4a_image_t azPtCloud = NULL;

//        if (K4A_RESULT_SUCCEEDED != k4a_device_get_capture (azureHandle, &capture_handle, 5000))
//        {
//            qDebug()<<"getImageFromKinect::k4a_device_get_capture failed\n";
//            return 0;
//        }

//        if(imageType == COL_IMAGE || imageType == COL_DEP_IMAGE) //just colour image
//        {
//            image_color = k4a_capture_get_color_image(capture_handle);
//            if (image_color != NULL)
//            {
//                Mat distortImage;
//                uint8_t * image_ptr_color = NULL;
//                image_ptr_color = k4a_image_get_buffer(image_color);

//                distortImage = cv::Mat(colorHeight,colorWidth,CV_8UC4,(void *)image_ptr_color,cv::Mat::AUTO_STEP);
//                //pLocImage = distortImage.clone();
//                pLocImage = distortImage;
//                //undistort(distortImage,pLocImage,cameraMatrix,distCoeffs);
//                //remap(distortImage, pLocImage, undistMap1,undistMap2, cv::INTER_CUBIC);
//            }
//        }

//        if (imageType == COL_DEP_IMAGE )
//        //if (0)
//        {
//            elTimer.start();
//            static int normalCalcCount = 0;
//            // Created an image
//            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,colorHeight,colorWidth*sizeof(uint16_t),&azDepthTransImage ))
//                qDebug()<<"azDepthTransImage could not be written!\n";

//            double *depImgSumPtr            =  (double*)depImgSumMat.data;
//            float *depImgAvgDisPtr          =  (float*)depImgAvgDisMat.data;
//            float *depImgAvgPtr             =  (float*)depImgAvgMat.data;
//            int i;

//            while(scanInfoPtr->enableDepthControl && normalCalcCount < 1)
//            {
//                k4a_capture_release(capture_handle);
//                k4a_device_get_capture (azureHandle, &capture_handle, 5000);

//                //QCoreApplication::processEvents();
//                azDepthImage = k4a_capture_get_depth_image(capture_handle);

//                if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera( transformation_handle, azDepthImage, azDepthTransImage))
//                    qDebug()<<"transformation from depth to color image could n't be done. !\n";


//                pDepthBuffer = (unsigned short *) k4a_image_get_buffer(azDepthTransImage);

//    //            if (scanInfoPtr->enableOrientControl
//    //                    && normalCalcCount < 2)
//                {
//                    //averaging
//                    for (i=0;i<depthWidth*depthHeight;i++)
//                        depImgSumPtr[i] += pDepthBuffer[i]; //accumulate the current depth frame

//                    depImgAvgCnt++;
//                    if (depImgAvgCnt == DEPTHIMGAVGCNT) //depth buffer average count
//                    {
//                        //qDebug()<<depImgAvgCnt<<" frames DEPTHIMGAVGCNT averaging done.";

//                        for (i=0;i<depthWidth*depthHeight;i++)
//                        {
//                            depImgAvgDisPtr[i]     = depImgSumPtr[i] / DEPTHIMGAVGCNT ; //accumulate the current depth frame
//                            //pDepthBufferAvg[i]  = round(depImgAvgPtr[i]);
//                        }

//                        //depImgAvgMat = depImgAvgDisMat;
//                        //undistort(depImgAvgDisMat,depImgAvgMat,cameraMatrix,distCoeffs);
//                        remap(depImgAvgDisMat, depImgAvgMat, undistMap1,undistMap2, cv::INTER_CUBIC);

//                        //apply a smoothing filter to depImgAvgMat
//                        GaussianBlur(depImgAvgMat,depImgAvgMat,Size(11,11),0);

//                        for (int i=0;i<depthWidth*depthHeight;i++)
//                            pDepthBufferAvg[i]  = round(depImgAvgPtr[i]);

//                        //make pt cloud from the avergaed buffer
//                        k4a_image_t azDepthTransAvgImage = NULL;
//                        // Create image from the buffer
//                        k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16,colorWidth,colorHeight,
//                                                     colorWidth*sizeof(uint16_t),(uint8_t *) pDepthBufferAvg,colorWidth*colorHeight*sizeof(uint16_t),
//                                                     0, 0,&azDepthTransAvgImage);

//                        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,colorHeight,6*colorWidth,&azPtCloud ))
//                            qDebug()<<"azPtCloud could not be created!\n";

//                        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud( transformation_handle, azDepthTransAvgImage,
//                                                                                                   K4A_CALIBRATION_TYPE_COLOR, azPtCloud))
//                            qDebug()<<"transformation from depth to point cloud could n't be done. !\n";


//                        int16_t * azPtCloudp      = (int16_t *) k4a_image_get_buffer(azPtCloud);

//                        for (int i=0;i<depthWidth*depthHeight;i++)
//                        {
//                            pColor2xyz[i].X = -1*azPtCloudp[i*3+0];
//                            pColor2xyz[i].Y = -1*azPtCloudp[i*3+1];
//                            pColor2xyz[i].Z = azPtCloudp[i*3+2];
//                        }

//                        k4a_image_release(azDepthTransAvgImage);

//                        #ifdef CALCNORMALS
//                            depImgAvgMatFliped = depImgAvgMat;

//                        {
//                            //if (normalCalcCount < 2)
//                            if (scanInfoPtr->enableOrientControl)
//                            {
//                                calcNormalsPcl(true);

//                                qDebug("specImageProcessor::getImageFromKinect - Took %d ms to accumulate %d frames",elTimer.elapsed(),DEPTHIMGAVGCNT);


//                             }
//                            normalCalcCount++;

//                        }
//                            depImgAvgCnt = 0;
//                            depImgSumMat = Scalar::all(0);
//                        #endif
//                    }
//                }
//            }
//        }

//        if(imageType == DEP_IMAGE) //just depth image
//        {
//            azDepthImage = k4a_capture_get_depth_image(capture_handle);
//            // Created an image
//            if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, k4a_image_get_width_pixels(image_color),
//                                                         k4a_image_get_height_pixels(image_color),
//                                                         k4a_image_get_width_pixels(image_color)*sizeof(uint16_t),
//                                                         &azDepthTransImage ))
//            {
//                qDebug()<<"azDepthTransImage could not be written!\n";
//            }
//            if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera( transformation_handle, azDepthImage, azDepthTransImage))
//            {
//                qDebug()<<"transformation from depth to color image could n't be done. !\n";
//            }
//            if (azDepthImage != NULL)
//            {
////                qDebug()<<" Depth Image "<<k4a_image_get_height_pixels(azDepthImage)<<k4a_image_get_width_pixels(azDepthImage)<<k4a_image_get_stride_bytes(azDepthImage);
//                pDepthBuffer        = (unsigned short *) k4a_image_get_buffer(azDepthTransImage);
//                depthWidth          = k4a_image_get_width_pixels(azDepthTransImage);
//                depthHeight         = k4a_image_get_height_pixels(azDepthTransImage);

//                pLocImage.create(depthHeight,depthWidth,CV_8UC4);
//                uchar *dest  = pLocImage.data;
//                grey16ToRgb8(pDepthBuffer,dest,depthWidth,depthHeight,TRUE); // for display
//            }
//        }

//        // Release the capture
//        k4a_image_release(image_color);
//        k4a_image_release(azDepthImage);
//        k4a_image_release(azDepthTransImage);
//        k4a_image_release(azPtCloud);
////        k4a_image_release(azDepthTransAvgImage);
//        k4a_capture_release(capture_handle);
    }
    else if (useKinect)
    {
        HRESULT hr;

        IFrameDescription* pFrameDescription = NULL;
        QTime currentTime;

        //first release the previous acquisitions
        SAFE_RELEASE(pLongIRFrame);
        SAFE_RELEASE(pColorFrame);
        SAFE_RELEASE(pDepthFrame);
        SAFE_RELEASE(pMultiSourceFrame);

        imageType = imageTypeArg;

        static bool normalCalcFlag;

        if (pDepth2xyz!=NULL)
        {
            delete pDepth2xyz;
            pDepth2xyz=NULL;
        }
        if (pColor2depth !=NULL)
        {
            delete pColor2depth;
            pColor2depth=NULL;
        }

        //pDepthBuffer=NULL;

        hr = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

        if(SUCCEEDED(hr) == false)
        {
            //qWarning()<<currentTime.currentTime()<<"Acquire pMultiSourceFrame failed.."<<QString::number(hr,16).toUpper();
            return 0;
        }

        //else
          //  qWarning("Acquire pMultiSourceFrame success..");

        //Acquire the depth frame
        IDepthFrameReference* pDepthFrameReference = NULL;
        hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
        if (SUCCEEDED(hr) == false)
        {
            //qWarning()<<currentTime.currentTime()<<hr<<"pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference) Failed"<<QString::number(hr,16).toUpper();
            return 0;
        }
        else
        {   hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
            if(SUCCEEDED(hr) == false)
            {
                //qWarning()<<currentTime.currentTime()<<"pDepthFrameReference->AcquireFrame(&pDepthFrame) - FAILED - hr:"<<QString::number(hr,16).toUpper();
                return 0;
            }
        }
        SAFE_RELEASE(pDepthFrameReference);

        if (imageType == IR_IMAGE || imageType == IR_DEP_IMAGE)
        {
            //Acquire the long exposure IR frame
            ILongExposureInfraredFrameReference* pLongIRFrameReference = NULL;
            hr = pMultiSourceFrame->get_LongExposureInfraredFrameReference(&pLongIRFrameReference);
            if (SUCCEEDED(hr) == false)
            {
                //qWarning()<<currentTime.currentTime()<<hr<<"pMultiSourceFrame->get_LongExposureInfraredFrameReference(&pLongIRFrameReference) Failed"<<QString::number(hr,16).toUpper();
                return 0;
            }
            else
            {   hr = pLongIRFrameReference->AcquireFrame(&pLongIRFrame);
                if(SUCCEEDED(hr) == false)
                {
                    //qWarning()<<currentTime.currentTime()<<"pLongIRFrameReference->AcquireFrame(&pLongIRFrame) - FAILED - hr:"<<QString::number(hr,16).toUpper();
                    return 0;
                }
            }
            SAFE_RELEASE(pLongIRFrameReference);
        }

        //Acquire the colour frame
        IColorFrameReference* pColorFrameReference = NULL;
        hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pColorFrameReference->AcquireFrame(&pColorFrame);
            if(SUCCEEDED(hr) == false)
            {
                //qWarning()<<currentTime.currentTime()<<"pColorFrameReference->AcquireFrame(&pColorFrame) - FAILED - hr:"<<QString::number(hr,16).toUpper();
                SAFE_RELEASE(pColorFrameReference);
                return 0;
            }
        }
        else
        {
            //qWarning()<<currentTime.currentTime()<<hr<<"getColourImageFromKinect (multisource) - Acquire colour Frame Failed"<<QString::number(hr,16).toUpper();
            SAFE_RELEASE(pColorFrameReference);
            return 0;
        }

        SAFE_RELEASE(pColorFrameReference);

        ICoordinateMapper* m_pCoordinateMapper = NULL;
        hr = pMyKinect->get_CoordinateMapper(&m_pCoordinateMapper);

        if(imageType == COL_IMAGE || imageType == COL_DEP_IMAGE) //just colour image
        {
            pColorFrame->get_FrameDescription(&pFrameDescription);
            pFrameDescription->get_Height(&colorHeight);
            pFrameDescription->get_Width(&colorWidth);

            pLocImage.create(colorHeight,colorWidth,CV_8UC4);
            pColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(pLocImage.data), ColorImageFormat_Bgra);
        }

        if((imageType == DEP_IMAGE || imageType == COL_DEP_IMAGE || imageType == IR_DEP_IMAGE)
                && normalCalcCount < 1) //just depth image
        {
            unsigned int sz;
            unsigned short* pDepthBufferTemp;

            pDepthFrame->get_FrameDescription(&pFrameDescription);
            pFrameDescription->get_Height(&depthHeight);
            pFrameDescription->get_Width(&depthWidth);
            pDepthFrame->AccessUnderlyingBuffer(&sz, &pDepthBufferTemp);

            memcpy(pDepthBuffer,pDepthBufferTemp,depthHeight*depthWidth*sizeof(unsigned short));

            if (scanInfoPtr->enableDepthControl)
            {
                static QElapsedTimer t;
                //averaging
                double *depImgSumPtr =  (double*)depImgSumMat.data;
                float *depImgAvgMatPtr =  (float*)depImgAvgMat.data;
                //average the depth buffer to remove noise as much as possible.

                unsigned short Temp;
                int j;
                for (int i=0;i<depthWidth*depthHeight;i++)
                {
                    Temp = pDepthBuffer[i];
                    //incase of a bad reading
                    j = i-1;
                    while (Temp == -std::numeric_limits<float>::infinity() || Temp == 0 && j>0)
                    {
                        Temp = pDepthBuffer[j];
                        j--;
                    }
                    depImgSumPtr[i] += Temp; //accumulate the current depth frame
                }

                depImgAvgCnt++;

                //qDebug()<<depImgAvgCnt;

                if (depImgAvgCnt == DEPTHIMGAVGCNT) //depth buffer average count
                {
                    //qDebug()<<depImgAvgCnt<<" frames DEPTHIMGAVGCNT averaging done.";

                    for (int i=0;i<depthWidth*depthHeight;i++)
                    {
                        depImgAvgMatPtr[i]     = depImgSumPtr[i] / DEPTHIMGAVGCNT ; //accumulate the current depth frame
                        //pDepthBufferAvg[i]  = round(depImgAvgPtr[i]);
                    }

                    //apply a smoothing filter to depImgAvgMat

                    //GaussianBlur(depImgAvgMat,depImgAvgMat,Size(9,9),0);
                      blur(depImgAvgMat,depImgAvgMat, Size(13,13));

                    for (int i=0;i<depthWidth*depthHeight;i++)
                        pDepthBufferAvg[i]  = round(depImgAvgMatPtr[i]);

                    pDepth2xyz = new CameraSpacePoint[depthWidth * depthHeight];

                    m_pCoordinateMapper->MapDepthFrameToCameraSpace(
                            depthWidth*depthHeight, pDepthBufferAvg,        // Depth frame data and size of depth frame
                            depthWidth*depthHeight, pDepth2xyz);          // Output CameraSpacePoint array and size

                    depImgAvgCnt = 0;
                    depImgSumMat = Scalar::all(0);
                    #ifdef CALCNORMALS
                        if(flipImage)
                            flip(depImgAvgMat, depImgAvgMatFliped, 1);
                        else
                            depImgAvgMatFliped = depImgAvgMat;

                        if (scanInfoPtr->enableDepthControl<1)
                        {
                            calcNormalsPcl(false);
                            setRefreshInterval("500");
                            qDebug("specImageProcessor::getImageFromKinect - Took %d ms to accumulate %d frames",t.elapsed(),DEPTHIMGAVGCNT);
                            t.start();
                        }
                        normalCalcCount++;
                    #endif
                }
            }

            if (imageType == COL_DEP_IMAGE)
            {
//                pColor2xyz      =  new CameraSpacePoint[colorWidth * colorHeight];
//                pColor2xyzAvg   =  new CameraSpacePoint[colorWidth * colorHeight];

                m_pCoordinateMapper->MapColorFrameToCameraSpace(
                        depthWidth*depthHeight, pDepthBuffer,        // Depth frame data and size of depth frame
                        colorWidth*colorHeight, pColor2xyz);          // Output CameraSpacePoint array and size

                m_pCoordinateMapper->MapColorFrameToCameraSpace(
                        depthWidth*depthHeight, pDepthBufferAvg,        // Depth frame data and size of depth frame
                        colorWidth*colorHeight, pColor2xyzAvg);          // Output CameraSpacePoint array and size

                // pColor2xyzAvg is not used for actual processing as it has random negative vals for reasons un-known

                /*
                CameraSpacePoint csOrigin;
                csOrigin.X = 0;
                csOrigin.Y = 0;
                csOrigin.Z = 0;

                pColorSpaceOrigin.X = 0;
                pColorSpaceOrigin.Y = 0;

                m_pCoordinateMapper->MapCameraPointToColorSpace(csOrigin,&pColorSpaceOrigin);

                qDebug()<<"pColorSpaceOrigin"<<pColorSpaceOrigin.X;
                */
            }
            if (imageType == DEP_IMAGE) // for COL_DEP_IMAGE and IR_DEP_IMAGE keep Colour/IR in opencv container
            {
                pLocImage.create(depthHeight,depthWidth,CV_8UC4);
                uchar *dest  = pLocImage.data;
                grey16ToRgb8(pDepthBuffer,dest,depthWidth,depthHeight,TRUE); // for display

                /*
                //for normal calulations
                depImgForNormalCalc.create(depthHeight,depthWidth,CV_32FC1);
                float *destData =  (float*)depImgForNormalCalc.data;
                //copy
                for (int i=0;i<depthWidth*depthHeight;i++)
                {
                    destData[i] = (pDepthBuffer[i] & 0xfff8);//quantize to recude noise for surface normal calcs
                }
                */
            }
        }

        if(imageType == COL_DEP_IMAGE) // for mapping b/w colour and depth; not successful so far
        {
            pColor2depth = new DepthSpacePoint[colorWidth * colorHeight];

            HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(depthWidth * depthHeight,
                                                                        (UINT16*)pDepthBuffer, colorWidth * colorHeight, pColor2depth);
            /*
            for (int colorIndex = 0; colorIndex < (colorWidth*colorHeight); ++colorIndex)
            {
                DepthSpacePoint p = pDepthCoordinates[colorIndex];
                qDebug()<<p.X<<p.Y;
            }
            */
        }

        if (imageType == IR_IMAGE || imageType == IR_DEP_IMAGE)
        {
            unsigned int sz;
            unsigned short* buf;

            pLongIRFrame->get_FrameDescription(&pFrameDescription);
            pFrameDescription->get_Height(&depthHeight);
            pFrameDescription->get_Width(&depthWidth);

            pLocImage.create(depthHeight,depthWidth,CV_8UC4);

            uchar *dest  = pLocImage.data;
            pLongIRFrame->AccessUnderlyingBuffer(&sz, &buf);

            grey16ToRgb8(buf, dest,depthWidth,depthHeight,false);
        }
    }

    if(flipImage)
        flip(pLocImage, *ptrToMat, 1);     // because you can't flip in-place (leads to segfault)
    else
        *ptrToMat = pLocImage.clone();     // because you can't flip in-place (leads to segfault)

    //qDebug("specImageProcessor::getImageFromKinect - Time taken: %d ms ",t.elapsed());

    return 1;
}

#ifdef CALCNORMALS
#define PIXELJUMP   1
void specImageProcessor::calcNormals()
{
/////////////////////self written normal calcs////////////////////////
    //genTestDepth();
    Mat dxSobel,dySobel;
    float phi;
    float theta;
    float r;
    Mat thetaMatTemp,phiMatTemp;

    normals.create(depImgAvgMatFliped.size(), CV_32FC3);

/*
    for (int i =0;i<15;i++)
    {
        bilateralFilter(depImgAvgMatFliped,depImgAvgLoc,15,100,100);
        depImgAvgMatFliped = depImgAvgLoc.clone();
    }
*/

    //for (int i =0;i<1;i++)
        //blur(depImgAvgMatFliped,depImgAvgMatFliped,Size(21,21));

    //take sobel in x direction
    //Sobel(depImgAvgMatFliped,dxSobel,CV_32F,1,0,3);
    //Sobel(depImgAvgMatFliped,dySobel,CV_32F,0,1,3);

    for(int x = PIXELJUMP; x < depImgAvgMatFliped.cols-PIXELJUMP; x++)
    {
        for(int y = PIXELJUMP; y < depImgAvgMatFliped.rows-PIXELJUMP; y++)
        {
            // use float instead of double otherwise you will not get the correct result
            // check my updates in the original post. I have not figure out yet why this
            // is happening.

            float dzdx = (depImgAvgMatFliped.at<float>(y, x+PIXELJUMP) - depImgAvgMatFliped.at<float>(y, x-PIXELJUMP)) / 2.0*PIXELJUMP; // col vector
            float dzdy = (depImgAvgMatFliped.at<float>(y+PIXELJUMP, x) - depImgAvgMatFliped.at<float>(y-PIXELJUMP, x)) / 2.0*PIXELJUMP; // row vector

            //float dzdy = dySobel.at<float>(y,x);
            //float dzdx = dxSobel.at<float>(y,x);

            //need to flip for correct angles .... WHY ????
            //DONT need this flip
            //float temp;
            //temp = dzdx;
            //dzdx = dzdy;
            //dzdy = temp;

            Vec3f d(-dzdx, -dzdy, 1.0f);

            Vec3f n = normalize(d); //BGR

            normals.at<Vec3f>(y, x) = d;

            //now the angles
            if (dzdx == 0 && dzdy == 0)
            {
                phi     = 0;
                theta   = 0;//normal aligned to Z so phi doesnot matter
            }
            else
            {
                phi     = atan(dzdy/dzdx)*(180/M_PI);
                theta   = atan(sqrt(dzdx*dzdx+dzdy*dzdy))*(180/M_PI);
            }

            Vec3f a(0,phi,theta);  // r,phi,theta
            normalAngles.at<Vec3f>(y,x) = a;

            phiMat.at<short>(y,x)   = phi;
            thetaMat.at<short>(y,x) = theta;
        }
    }

     imshow("normals", normals);
     imshow("phi", phiMat);
     imshow("theta", thetaMat);
     //waitKey(0);

     depImgAvgMatFliped.convertTo(depImgTemp,CV_8U,(float)255/(float)1500);

      cv::namedWindow("depth", CV_WINDOW_AUTOSIZE );
      setMouseCallback("depth", onMouseOpenCvWrapper, this );
      imshow("depth", depImgTemp);
      cv::moveWindow("depth", 0, 0);

      cv::namedWindow("normals", CV_WINDOW_AUTOSIZE );
      imshow("normals", normals);
      cv::moveWindow("normals", 515, 0);

      phiMat.convertTo(phiMatTemp,CV_8U,(float)127/(float)phiMaxAngle, 127);

      cv::namedWindow("phi", CV_WINDOW_AUTOSIZE );
      setMouseCallback("phi", onMouseOpenCvWrapper, this );
      imshow("phi", phiMatTemp);
      cv::moveWindow("phi", 0, 500);

      //thetaMat.convertTo(thetaMatTemp,CV_8U,(float)127/(float)thetaMaxAngle,127);
      thetaMat.convertTo(thetaMatTemp,CV_8U,(float)255/(float)thetaMaxAngle);
//      Mat thetaMatTemp1;
//      applyColorMap(thetaMatTemp,thetaMatTemp1,COLORMAP_JET);
//      cv::namedWindow("theta", CV_WINDOW_AUTOSIZE );
//      setMouseCallback("theta", onMouseOpenCvWrapper, this );
//      imshow("theta", thetaMatTemp1);
//      cv::moveWindow("theta", 515, 500);

      colouredDisplay(1100,500,COLORMAP_JET,thetaMatTemp,"theta",thetaMaxAngle);
}

void specImageProcessor::genTestDepth()
{
    for(int x = 0; x < depImgAvgMat.cols; ++x)
    {
        for(int y = 0; y < depImgAvgMat.rows; ++y)
        {
            if (x <= 100)
                depImgAvgMat.at<float>(y,x) = 400;
            else if (x > 100 && x <= 300)
                depImgAvgMat.at<float>(y,x) = 400+x;
            else
                depImgAvgMat.at<float>(y,x) = 400-x;

            depImgAvgMat.at<float>(y,x) = 400+y;
        }
    }
}

void specImageProcessor::setNormSmoothingSize(int normSmoothingSizeArg)
{
    normSmoothingSize = normSmoothingSizeArg;
    //calcNormalsPcl();
}

void specImageProcessor::setMaxDepthChangeFactor(int maxDepthChangeFactorArg)
{
    maxDepthChangeFactor = maxDepthChangeFactorArg;
    //calcNormalsPcl();
}

void specImageProcessor::changeNormCalcMethod()
{
    normCalcMethod = (normCalcMethod + 1)%4;
    //calcNormalsPcl();
}

void specImageProcessor::setEnableDepthDependentSmoothing(bool val)
{
    enableDepthDependentSmoothing = val;
    //calcNormalsPcl();
}

void specImageProcessor::setThetaMaxAngle(int thetaMaxAngleArg)
{
    thetaMaxAngle = thetaMaxAngleArg;

//    thetaPosMat = abs(thetaMat);
//    thetaPosMat.convertTo(thetaPosDispMat,CV_8U,(float)255/(float)thetaMaxAngle);

    thetaPosMat = thetaMat+thetaMaxAngle;
    thetaPosMat.convertTo(thetaPosDispMat,CV_8U,(float)255/((float)2*thetaMaxAngle),0);
    colouredDisplayNeg(500,0,COLORMAP_JET,thetaPosDispMat,"Theta",thetaMaxAngle);

    //thetaFiltPosMat = abs(thetaFiltMat);
    //thetaFiltPosMat.convertTo(thetaFiltPosDispMat,CV_8U,(float)255/(float)thetaMaxAngle);

    thetaFiltPosMat = thetaFiltMat+thetaMaxAngle;
    thetaFiltPosMat.convertTo(thetaFiltPosDispMat,CV_8U,(float)255/((float)2*thetaMaxAngle),0);
    colouredDisplayNeg(500,500,COLORMAP_JET,thetaFiltPosDispMat,"Theta Filtered",thetaMaxAngle);
}

void specImageProcessor::setPhiMaxAngle(int phiMaxAngleArg)
{
    phiMaxAngle = phiMaxAngleArg;

//    phiPosMat = abs(phiMat);
//    phiPosMat.convertTo(phiPosDispMat,CV_8U,(float)255/(float)phiMaxAngle);
//    colouredDisplay(1200,0,COLORMAP_JET,phiPosDispMat,"Phi",phiMaxAngle);

//    phiFiltPosMat = abs(phiFiltMat);
//    phiFiltPosMat.convertTo(phiFiltPosDispMat,CV_8U,(float)255/(float)phiMaxAngle);
//    colouredDisplay(1200,500,COLORMAP_JET,phiFiltPosDispMat,"Phi Filtered",phiMaxAngle);

    phiPosMat = phiMat+phiMaxAngle;
    phiPosMat.convertTo(phiPosDispMat,CV_8U,(float)255/(float)(2*phiMaxAngle),0);
    colouredDisplayNeg(1200,0,COLORMAP_JET,phiPosDispMat,"Phi",phiMaxAngle);

    phiFiltPosMat = phiFiltMat+phiMaxAngle;
    phiFiltPosMat.convertTo(phiFiltPosDispMat,CV_8U,(float)255/(float)(2*phiMaxAngle));
    colouredDisplayNeg(1200,500,COLORMAP_JET,phiFiltPosDispMat,"Phi Filtered",phiMaxAngle);
}

void specImageProcessor::setSmoothSigmaSpace(int smoothSigmaSpaceArg)
{
    smoothSigmaSpace    =   smoothSigmaSpaceArg;
    smoothFilter();
    setPhiMaxAngle(phiMaxAngle);
}

void specImageProcessor::setSmoothSigmaColour(int smoothSigmaColourArg)
{
    smoothSigmaColour    =   smoothSigmaColourArg;
    smoothFilter();
    setPhiMaxAngle(phiMaxAngle);
}

void specImageProcessor::setSmoothKernelSize(int smoothKernelSizeArg)
{
    smoothKernelSize    =   smoothKernelSizeArg;
    smoothFilter();
    setPhiMaxAngle(phiMaxAngle);
    setThetaMaxAngle(thetaMaxAngle);
}
void specImageProcessor::colouredDisplayNeg(int x,int y, cv::ColormapTypes colBar, Mat inMat, cv::String winName, int maxScaleVal )
{
//    Mat newMap (256,1,CV_8UC3);
////    for (int i=0;i<=255;i++)
////        newMap.at<Vec3b>(i,0) = Vec3b(255-2*i,i,128+i); //B,G,R

//    unsigned char b[] = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,253,251,249,247,245,242,241,238,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,145,143,141,138,136,134,132,131,129,126,125,122,121,118,116,115,113,111,109,107,105,102,100,98,97,94,93,91,89,87,84,83,81,79,77,75,73,70,68,66,64,63,61,59,57,54,52,51,49,47,44,42,40,39,37,34,33,31,29,27,25,22,20,18,17,14,13,11,9,6,4,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


//    unsigned char g[] = { 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,252,250,248,246,244,242,240,238,236,234,232,230,228,226,224,222,220,218,216,214,212,210,208,206,204,202,200,198,196,194,192,190,188,186,184,182,180,178,176,174,171,169,167,165,163,161,159,157,155,153,151,149,147,145,143,141,139,137,135,133,131,129,127,125,123,121,119,117,115,113,111,109,107,105,103,101,99,97,95,93,91,89,87,85,83,82,80,78,76,74,72,70,68,66,64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0 };


//    unsigned char r[] = {195,194,193,191,190,189,188,187,186,185,184,183,182,181,179,178,177,176,175,174,173,172,171,170,169,167,166,165,164,163,162,161,160,159,158,157,155,154,153,152,151,150,149,148,147,146,145,143,142,141,140,139,138,137,136,135,134,133,131,130,129,128,127,126,125,125,125,125,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126};

//    //Mat newMap[] = {Mat(256,1, CV_8U, b), Mat(256,1, CV_8U, g), Mat(256,1, CV_8U, r)};

//    for (int i=0;i<=255;i++)
//    {
//            b[i] = 0;
//            if(i<128)
//                b[i] = 255-(i*2);
//    }

//    for (int i=0;i<=255;i++)
//    {
//            g[i] = 0;
//            if(i>128)
//                g[i] = 3*(i-128);
//    }

//    for (int i=0;i<=255;i++)
//    {
//            r[i] = 0;
//           if(i<128)
//                r[i] = 2*i ;
//           else
//                r[i] = 255-2*i;

//    }

//    for (int i=0;i<=255;i++)
//        newMap.at<Vec3b>(i,0) = Vec3b(b[i],g[i],r[i]); //B,G,R


    Mat colMat;
    applyColorMap(inMat,colMat,COLORMAP_JET);

    if (useKinect)
        cv::namedWindow(winName, CV_WINDOW_AUTOSIZE );
    else
    {
        cv::namedWindow(winName, WINDOW_NORMAL);
        cv::resizeWindow(winName,colorWidth/4,colorHeight/4);
    }
    setMouseCallback(winName, onMouseOpenCvWrapper, this );
    imshow(winName, colMat);
    cv::moveWindow(winName, x, y);

    cv::namedWindow(winName+"colBar", CV_WINDOW_AUTOSIZE );
    cv::moveWindow(winName+"colBar", x+520, y);

    //INPUT, colourbar position
     Mat colourBarGrey,colourBar;
     colourBarGrey.create(255,150,CV_8U);
     colourBarGrey = Scalar::all(100);

     for (int row = 0;row < 255;row++)
     {
         for (int col = 0;col < 50;col++)
             colourBarGrey.at<char>(row,col) = row;
     }

     applyColorMap(colourBarGrey,colourBar,COLORMAP_JET);

     //put the labels for the colour bar
     cv::putText(colourBar,cv::format("%d",-maxScaleVal),cv::Point(60,15),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",-maxScaleVal/2) ,cv::Point(60,70),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",0) ,cv::Point(60,125),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",maxScaleVal/2) ,cv::Point(60,188),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",maxScaleVal) ,cv::Point(60,250),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);

     imshow(winName+"colBar", colourBar);
}

//position of the colour bar (x,y)
void specImageProcessor::colouredDisplay(int x,int y, cv::ColormapTypes colBar, Mat inMat, cv::String winName, int maxScaleVal )
{
    Mat colMat;
    applyColorMap(inMat,colMat,COLORMAP_JET);

    if (useKinect)
        cv::namedWindow(winName, CV_WINDOW_AUTOSIZE );
    else
    {
        cv::namedWindow(winName, WINDOW_NORMAL);
        cv::resizeWindow(winName,colorWidth/4,colorHeight/4);
    }
    setMouseCallback(winName, onMouseOpenCvWrapper, this );
    imshow(winName, colMat);
    cv::moveWindow(winName, x, y);

    cv::namedWindow(winName+"colBar", CV_WINDOW_AUTOSIZE );
    cv::moveWindow(winName+"colBar", x+520, y);

    //INPUT, colourbar position
     Mat colourBarGrey,colourBar;
     colourBarGrey.create(255,150,CV_8U);
     colourBarGrey = Scalar::all(100);

     for (int row = 0;row < 255;row++)
     {
         for (int col = 0;col < 50;col++)
             colourBarGrey.at<char>(row,col) = row;
     }

     applyColorMap(colourBarGrey,colourBar,colBar);

     //put the labels for the colour bar
     cv::putText(colourBar,"0",cv::Point(60,15),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",maxScaleVal/4) ,cv::Point(60,70),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",maxScaleVal/2) ,cv::Point(60,125),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",3*maxScaleVal/4) ,cv::Point(60,188),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);
     cv::putText(colourBar,cv::format("%d",maxScaleVal) ,cv::Point(60,250),cv::FONT_HERSHEY_PLAIN, 1,(255,255,255),2);

     imshow(winName+"colBar", colourBar);
}

void specImageProcessor::smoothFilter()
{
    //Filtering the Theta and Phi vals
    //Column filtering of thetaMat
    Mat thetaFloatLocMat,thetaFiltFloatLocMat,phiFloatLocMat,phiFiltFloatLocMat;
    Mat colKernel;
    Mat rowKernel;
    int colKernelSize = smoothKernelSize;

    if (colKernelSize>1)
    {
        if (smoothSigmaColour>1)
        {
            thetaMat.convertTo(thetaFloatLocMat,CV_32F);
            bilateralFilter(thetaFloatLocMat,thetaFiltFloatLocMat,smoothKernelSize,smoothSigmaColour,smoothSigmaSpace);
            thetaFiltFloatLocMat.convertTo(thetaFiltMat,CV_16SC1);

            phiMat.convertTo(phiFloatLocMat,CV_32F);
            bilateralFilter(phiFloatLocMat,phiFiltFloatLocMat,smoothKernelSize,smoothSigmaColour,smoothSigmaSpace);
            phiFiltFloatLocMat.convertTo(phiFiltMat,CV_16SC1);
        }
        else
        {
            colKernel = Mat::ones(colKernelSize,1,CV_32F)/(float)(colKernelSize);
            filter2D(thetaMat,thetaFiltMat,-1,colKernel,(Point)(0,0),0,BORDER_DEFAULT);

            rowKernel = Mat::ones(1,colKernelSize,CV_32F)/(float)(colKernelSize);
            filter2D(phiMat,phiFiltMat,-1,rowKernel,(Point)(0,0),0,BORDER_DEFAULT);
        }
    }
    else
    {
        thetaFiltMat    = thetaMat.clone();
        phiFiltMat      = phiMat.clone();
    }
}

void specImageProcessor::calcNormalsPcl(bool showCloud)
{
    //cvDestroyWindow("depth");
    //genTestDepth();
    float phi;
    float theta;
    float r;

   //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
   //load point cloud

    // 1 - Populate the pcl cloud from KINECT sdk data
    //--init the PCL cloud
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   cloud->width = static_cast<uint32_t>( depthWidth );
   cloud->height = static_cast<uint32_t>( depthHeight );
   cloud->points.resize( cloud->height * cloud->width );
   cloud->is_dense = false;

   int num_of_points_added = 0;
   int depthIndex = 0;

   //--copy the data
   for(int y=0; y<depImgAvgMatFliped.rows; y++)
   { //2-D indexing
           for(int x=0; x< depImgAvgMatFliped.cols; x++)
           {
                 depthIndex = ptToDataIdxDepth(x,y);
                 pcl::PointXYZ p = pcl::PointXYZ();

                 //gives results similar to that of self written code
                 p.x = (depthWidth - 1) - x;
                 //p.x = x;
                 p.y = y;
                 //p.z = depImgAvgMatFliped.at<float>(y,x);

                 // should avergae this for noise removal
/*               //does not work for some reason
                 p.x = pDepth2xyz[depthIndex].X*1000;
                 p.y = pDepth2xyz[depthIndex].Y*1000;

                 if(pDepth2xyz[depthIndex].Z == 0)
                    p.z = NAN;
                 else
                    p.z = pDepth2xyz[depthIndex].Z*1000;
*/

                 // mixed depth with x y z

                 if (useKinect)
                 {
                     //if (pDepth2xyz != NULL)
                     if(0)
                     {
                         p.x = pDepth2xyz[depthIndex].Y*1000;
                         p.y = pDepth2xyz[depthIndex].X*1000;
                     }
                     else
                     {
                         p.x = y;
                         p.y = -x;
                     }
                     p.z = depImgAvgMatFliped.at<float>(y,x);
                 }
                 else if (useAzure)
                 {
                     //p.x = pColor2xyz[depthIndex].X;
                     //p.y = pColor2xyz[depthIndex].Y;
                     p.x = x;
                     p.y = y;
                     p.z = depImgAvgMatFliped.at<float>(y,x);
                 }

                 cloud->at(x,y) = p;
                 num_of_points_added++;
           }
   }

   //view the cloud
    if(showCloud)
    {
#ifdef SHOWCLOUD
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );
        viewer->addPointCloud(cloud, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
#endif
    }

   //qDebug("specImageProcessor::calcNormalsPcl() - normCalcMethod: %d -- maxDepthChangeFactor/100: %f  -- normSmoothingSize: %d",normCalcMethod,(float)maxDepthChangeFactor/(float)100,normSmoothingSize);
   //normal estimation time
   normalsPcl = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);

   if(loadKinFusCloud)
   {
       loadMesh(false);//*******KinFusion Normal Estimation - normals automatically copied to the normalsPcl
   }
   else
   {
       //*******Integral Normal Estimation******************
       pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
       //ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
       //ne.setMaxDepthChangeFactor(0.02f); //set to default
       //ne.setNormalSmoothingSize(20.0f); //set to default

       //if (normCalcMethod == 0)
       ne.setNormalEstimationMethod ((pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod)normCalcMethod);
       ne.setMaxDepthChangeFactor((float)maxDepthChangeFactor/(float)100);
       ne.setNormalSmoothingSize(normSmoothingSize);
       //ne.setViewPoint(0,0,1500);
       //ne.useSensorOriginAsViewPoint();

       if (enableDepthDependentSmoothing)
       {
           ne.setDepthDependentSmoothing(true);
           ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
       }

       //ne.setKSearch(3);
       ne.setInputCloud(cloud);
       ne.compute(*normalsPcl);
   }

   //3-Copy the normals to Mat container for display
   normalsByPcl_mat.create(depImgAvgMatFliped.size(), CV_32FC3);
   normalsByPcl_mat.create(depImgAvgMatFliped.size(), CV_32FC3);

   //calculate angles
   for(int y=0; y<depImgAvgMatFliped.rows; y++)
   { //2-D indexing
           for(int x=0; x<depImgAvgMatFliped.cols; x++)
           {
               //Just for display
                Vec3f nVector(normalsPcl->at(x,y).normal_x, normalsPcl->at(x,y).normal_y, normalsPcl->at(x,y).normal_z);
                normalsByPcl_mat.at<Vec3f>(y, x) = -1*nVector;

                {
//                    float nx = -1*normalsPcl->at(x,y).normal_x;
//                    float ny = -1*normalsPcl->at(x,y).normal_y;
//                    float nz = -1*normalsPcl->at(x,y).normal_z;

                    float nx = normalsPcl->at(x,y).normal_x;
                    float ny = normalsPcl->at(x,y).normal_y;
                    float nz = normalsPcl->at(x,y).normal_z;

                    r       =  nx*nx + ny*ny + nz*nz;
                    //phi     = atan(ny/nx)*((float)180/M_PI);
                    //theta   = atan(sqrt(nx*nx+ny*ny)/nz)*((float)180/M_PI);
                    double debugPhi     = ny/nx;
                    double debugTheta   = nz/r;

//                    phi     = atan(ny/nx)*((float)180/M_PI);
//                    theta   = acos(nz/r)*((float)180/M_PI);

//                    phi     = atan(nx/nz)*((float)180/M_PI);
//                    theta   = acos(ny/r)*((float)180/M_PI);


                    phi     = atan(ny/nz)*((float)180/M_PI);
                    theta   = asin(nx/r)*((float)180/M_PI);


                    //Phi --> Yaw,
                    //Gives angle for rotation around Y axis of the normal angle.
                    //Gives angle for rotation around X axis of TCP ,
                    //results in right-left movement of LDV.

                    //90-Theta --> Pitch,
                    //Gives angle for rotation around X axis of the normal angle.
                    //Gives angle for rotation around Y axis of TCP ,
                    //results in up-down movement of LDV.

                    //roll is not needed thanks to the symmetry of LDV laser beam

//                    if (theta>45.0)
//                        theta = 45.0;

//                    if (nx<0.0)
//                        theta *= -1;

                    theta *= -1;
                    phi *= -1;

                    Vec3f spherical(r,phi,theta);  // phi,theta
                    normalAngles.at<Vec3f>(y,x) = spherical;

                    phiMat.at<short>(y,x)   = phi;
                    thetaMat.at<short>(y,x) = theta;
                }
           }
   }

   //cv::putText( depImgAvgMat,"text",cv::Point(10,10),FONT_HERSHEY_PLAIN,10,(255,255,0),2,cv::LINE_AA);

    depImgAvgMatFliped.convertTo(depImgTemp,CV_8U,(float)255/(float)1500);
    if (useKinect)
        cv::namedWindow("depth", WINDOW_AUTOSIZE );
    else
    {
        cv::namedWindow("depth", WINDOW_NORMAL );
        cv::resizeWindow("depth", colorWidth/4,colorHeight/4);
    }

    setMouseCallback("depth", onMouseOpenCvWrapper, this );
    imshow("depth", depImgTemp);
    cv::moveWindow("depth", 0, 0);

    QString winLabel = "normalsByPcl_mat";
    switch(normCalcMethod)
    {
        case 0:
            winLabel += "(0: COVARIANCE_MATRIX (9-Images))";
            break;

        case 1:
            winLabel += "(1: AVERAGE_3D_GRADIENT  (6-Images) )";
            break;

        case 2:
            winLabel += "(2: AVERAGE_DEPTH_CHANGE  (3-Images))";
            break;

        case 3:
            winLabel += "(SIMPLE_3D_GRADIENT (direct))";
            break;
    }

    if (useKinect)
        cv::namedWindow(winLabel.toStdString(), WINDOW_AUTOSIZE );
    else
    {
        cv::namedWindow(winLabel.toStdString(), WINDOW_NORMAL );//CV_WINDOW_AUTOSIZE
        cv::resizeWindow(winLabel.toStdString(),colorWidth/4,colorHeight/4);
    }
    imshow(winLabel.toStdString(), normalsByPcl_mat);
    cv::moveWindow(winLabel.toStdString(), 0, 500);


//    thetaPosMat = abs(thetaMat);
//    thetaPosMat.convertTo(thetaPosDispMat,CV_8U,(float)255/(float)thetaMaxAngle);
//    colouredDisplay(500,0,COLORMAP_JET,thetaPosDispMat,"Theta",thetaMaxAngle);

    thetaPosMat = thetaMat+thetaMaxAngle;
    thetaPosMat.convertTo(thetaPosDispMat,CV_8U,(float)255/(float)(2*thetaMaxAngle),0);
    colouredDisplayNeg(500,0,COLORMAP_JET,thetaPosDispMat,"Theta",thetaMaxAngle);

//    phiPosMat = abs(phiMat);
//    phiPosMat.convertTo(phiPosDispMat,CV_8U,(float)255/(float)phiMaxAngle);
//    colouredDisplay(1200,0,COLORMAP_JET,phiPosDispMat,"Phi",phiMaxAngle);

    phiPosMat = phiMat+phiMaxAngle;
    phiPosMat.convertTo(phiPosDispMat,CV_8U,(float)255/(float)(2*phiMaxAngle),0);
    colouredDisplayNeg(1200,0,COLORMAP_JET,phiPosDispMat,"Phi",phiMaxAngle);

    smoothFilter();

//    thetaFiltPosMat = abs(thetaFiltMat);
//    thetaFiltPosMat.convertTo(thetaFiltPosDispMat,CV_8U,(float)255/(float)thetaMaxAngle);
//    colouredDisplay(500,500,COLORMAP_JET,thetaFiltPosDispMat,"Theta Filtered",thetaMaxAngle);

    thetaFiltPosMat = thetaFiltMat+thetaMaxAngle;
    thetaFiltPosMat.convertTo(thetaFiltPosDispMat,CV_8U,(float)255/(float)(2*thetaMaxAngle),0);
    colouredDisplayNeg(500,500,COLORMAP_JET,thetaFiltPosDispMat,"Theta Filtered",thetaMaxAngle);

//    phiFiltPosMat = abs(phiFiltMat);
//    phiFiltPosMat.convertTo(phiFiltPosDispMat,CV_8U,(float)255/(float)phiMaxAngle);
//    colouredDisplay(1200,500,COLORMAP_JET,phiFiltPosDispMat,"Phi Filtered",phiMaxAngle);

    phiFiltPosMat = phiFiltMat+phiMaxAngle;
    phiFiltPosMat.convertTo(phiFiltPosDispMat,CV_8U,(float)255/(float)(2*phiMaxAngle));
    colouredDisplayNeg(1200,500,COLORMAP_JET,phiFiltPosDispMat,"Phi Filtered",phiMaxAngle);

    //waitKey(0);
}

void onMouseOpenCvWrapper( int event, int x, int y, int, void* userdata)
{
    specImageProcessor* speImageProc = reinterpret_cast<specImageProcessor*>(userdata);
    speImageProc->onMouseOpenCv(event, x, y);
}

void specImageProcessor::onMouseOpenCv( int event, int x, int y )
{
    if ( event != CV_EVENT_LBUTTONDOWN )
           return;

       Point pt = Point(x,y);
       //qDebug()<<"onMouseDepthOpenCvWindow - ("<<pt.x<<", "<<pt.y<<") ...... "<< normalsByPcl_mat(y,x) << '\n';

// // for calc normals
//       qDebug("onMouseDepthOpenCvWindow - Pt(%d,%d) -- depth (%f) --  NormalsSelf(%f,%f,%f) -- spehrical (r,phi,theta) = (%f,%f,%f) ) \n",
//              pt.x,pt.y,
//              depImgAvgMatFliped.at<float>(y,x),
//              normals.at<Vec3f>(y,x)[0],normals.at<Vec3f>(y,x)[1],normals.at<Vec3f>(y,x)[2],
//              normalAngles.at<Vec3f>(y,x)[0],normalAngles.at<Vec3f>(y,x)[1],normalAngles.at<Vec3f>(y,x)[2]); // change (x,y) to (y,x) for correct results


       //for calc normals pcl
       qDebug("onMouseDepthOpenCvWindow - depth(%d,%d): %f -- normalsPcl:(%f,%f,%f) curv: %f -- normalsByPcl_mat:(%f,%f,%f) -- spehrical(r,phi,theta): radian(%f,%f,%f) , degree(%f,%f,%f) , "
              "thetaMat: (%d), thetaPosMat: (%d), thetaPosDispMat:(%d), thetaFiltMat: (%d), thetaFiltPosMat: (%d), thetaFiltPosDispMat:(%d) ) "
              "phiMat: (%d), phiPosMat: (%d), phiPosDispMat:(%d), phiFiltMat: (%d), phiFiltPosMat: (%d), phiFiltPosDispMat:(%d) )\n",
              pt.x,pt.y,
              depImgAvgMatFliped.at<float>(y,x),
              normalsPcl->at(x,y).normal_x,normalsPcl->at(x,y).normal_y,normalsPcl->at(x,y).normal_z,normalsPcl->at(x,y).curvature,
              normalsByPcl_mat.at<Vec3f>(y,x)[0],normalsByPcl_mat.at<Vec3f>(y,x)[1],normalsByPcl_mat.at<Vec3f>(y,x)[2],
              normalAngles.at<Vec3f>(y,x)[0],normalAngles.at<Vec3f>(y,x)[1]*M_PI/180,normalAngles.at<Vec3f>(y,x)[2]*M_PI/180,
              normalAngles.at<Vec3f>(y,x)[0],normalAngles.at<Vec3f>(y,x)[1],normalAngles.at<Vec3f>(y,x)[2],
              thetaMat.at<short>(y,x),thetaPosMat.at<short>(y,x),thetaPosDispMat.at<char>(y,x),thetaFiltMat.at<short>(y,x),thetaFiltPosMat.at<short>(y,x),thetaFiltPosDispMat.at<unsigned char>(y,x),
              phiMat.at<short>(y,x),phiPosMat.at<short>(y,x),phiPosDispMat.at<char>(y,x),phiFiltMat.at<short>(y,x),phiFiltPosMat.at<short>(y,x),phiFiltPosDispMat.at<unsigned char>(y,x));
}

void specImageProcessor::loadMesh( bool showPtCloud)
{
    qDebug("specImageProcessor::loadMesh");
    struct PointCloudValue
    {
        float Position[3];
        float Normal[3];
    };

    PointCloudValue point;
    int cloudSize = depthWidth*depthHeight;
    PointCloudValue *ptCloudframe = new PointCloudValue[cloudSize];

    FILE* pFile;
    int itemsRead;
    pFile = fopen("D:\\Hasan\\KinFusionFiles\\OrganizedPtCloud\\organizedPtCloud.bin", "rb");
    itemsRead = fread(ptCloudframe, 1, cloudSize*sizeof(PointCloudValue), pFile);
    qDebug("specImageProcessor::itemsRead: %d",itemsRead);

    int ptCloudframeIndex;

    pcl::PointCloud<pcl::PointXYZ>::Ptr kinFusCloud (new pcl::PointCloud<pcl::PointXYZ>);
    kinFusCloud->width = static_cast<uint32_t>( depthWidth );
    kinFusCloud->height = static_cast<uint32_t>( depthHeight );
    kinFusCloud->points.resize( kinFusCloud->height * kinFusCloud->width );
    kinFusCloud->is_dense = false;

    pcl::PointCloud<pcl::Normal>::Ptr kinFusNormal (new pcl::PointCloud<pcl::Normal>);
    kinFusNormal->width = static_cast<uint32_t>( depthWidth );
    kinFusNormal->height = static_cast<uint32_t>( depthHeight );
    kinFusNormal->points.resize( kinFusNormal->height * kinFusNormal->width );
    kinFusNormal->is_dense = false;

    //--copy the data
    for(int y=0; y < depthHeight; y++)
    { //2-D indexing
            for(int x=0; x < depthWidth; x++)
            {
                  ptCloudframeIndex     = x+(y*depthWidth);
                  pcl::PointXYZ p       = pcl::PointXYZ();
                  pcl::Normal n       = pcl::Normal();

                  point = ptCloudframe[ptCloudframeIndex];
                  p.x   = point.Position[0];
                  p.y   = point.Position[1];
                  p.z   = point.Position[2];

                  n.normal_x = -1*point.Normal[0];
                  n.normal_y = -1*point.Normal[1];
                  n.normal_z = -1*point.Normal[2];

                  kinFusCloud->at(x,y)  = p;
                  kinFusNormal->at(x,y) = n;
            }
    }
    if (showPtCloud)
    {
#ifdef SHOWCLOUD
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
        viewer2 = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );

        viewer2->setBackgroundColor (0, 0, 0);
        viewer2->addPointCloud(kinFusCloud, "cloud");
        viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

        //if(!kinFusNormal->empty())
            //viewer2->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (kinFusCloud, kinFusNormal, 10, 0.05, "normals");

        viewer2->addCoordinateSystem (1.0);
        viewer2->initCameraParameters ();
#endif
    }
    else
    {
        copyPointCloud(*kinFusNormal, *normalsPcl);
    }

    fclose(pFile);
    delete[] ptCloudframe;


//    //pcl::PointCloud<pcl::Normal>::Ptr sourceNormals(new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);


////    //pcl::io::loadOBJFile("D:\\Hasan\\KinFusionFiles\\gfrpRadialObj128.obj", *sourceNormals);
//    pcl::io::loadOBJFile("D:\\Hasan\\KinFusionFiles\\gfrpRadialObj128.obj", * sourceCloudWithNormals);
//    qDebug("Obj File: cloud width: %d, height: %d",sourceCloudWithNormals->width,sourceCloudWithNormals->height);


//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::io::loadOBJFile("D:\\Hasan\\KinFusionFiles\\gfrpRadialObj128.obj", * cloudXYZ);
//        qDebug("Obj File: cloud width: %d, height: %d",cloudXYZ->width,cloudXYZ->height);


//    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloudWithNormalsStl(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileSTL("D:\\Hasan\\KinFusionFiles\\gfrpRadialStl128.stl", mesh);
//    pcl::fromPCLPointCloud2(mesh.cloud, *sourceCloudWithNormalsStl);
//    qDebug("Stl File: cloud width: %d, height: %d",sourceCloudWithNormalsStl->width,sourceCloudWithNormalsStl->height);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud<pcl::Normal>);

//    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFileSTL("D:\\Hasan\\KinFusionFiles\\gfrpRadialStl384.stl", mesh);
//    pcl::fromPCLPointCloud2(mesh.cloud, *cloudXYZ);
//    pcl::fromPCLPointCloud2(mesh.cloud, *cloudNormal);

//    qDebug("Stl File: cloud width: %d, height: %d",cloudXYZ->width,cloudXYZ->height);
//    qDebug("Stl File: cloud width: %d, height: %d",cloudNormal->width,cloudNormal->height);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );

//    viewer->addPointCloud<pcl::PointNormal>(sourceCloudWithNormals, "foo", 1);
    //viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal> (sourceCloudWithNormals, sourceCloudWithNormals, 25, 0.15, "normals"); // It throws an exception here:

//    viewer->addPointCloud(cloudXYZ, "cloud" );
//    viewer->addPointCloudNormals(cloudXYZ,cloudNormal,10,0.05, "normals" );
}

#endif

void specImageProcessor::edgeDetection()
{
#ifndef ACTUALSYSTEM
    return;
#endif
    Mat procCvImgGray;
    Mat procCvImgEdged;
    Mat startImg,imagRoi;
    bool success= true;
    //Mat bgImg;
    if (useAzure)
    {
        success = getImageFromKinect(&startImg,imageType,0); // colour image
    }
    else if (useKinect)
    {
        success = getImageFromKinect(&startImg,imageType,0); // colour image
        if (robRangeDrawn == false && imageType == COL_DEP_IMAGE && pColor2xyz != NULL)
        {
            convHomeToKinCol();
        }
    }
    else
    {
        if (camHndl.isOpened() == false)
            startImg = imread(ImgPath.toStdString());
        else
            camHndl.read(startImg);
    }

    if (success && originDrawn == false && scanInfoPtr->enableOrientControl)
    {
        //draw Kinect origin
        //get Kinect origin in colour space

        CameraSpacePoint csPtCurr;
        CameraSpacePoint csPtOrigin;
        CameraSpacePoint csPtDiffStored,csPtDiffCurr;

        ColorSpacePoint clPt;

        csPtOrigin.X = 0;
        csPtOrigin.Y = 0;
        csPtOrigin.Z = 0;

        pColorSpaceOrigin.X = 0;
        pColorSpaceOrigin.Y = 0;

        csPtDiffStored.X = 50000;
        csPtDiffStored.Y = 50000;
        csPtDiffStored.Z = 50000;

        for (int x=0;x<colorWidth;x++)
        {
            for (int y=0;y<colorHeight;y++)
            {
                if (useKinect)
                    csPtCurr = mTomm(pColor2xyz[ptToDataIdxColor(x,y)],true,1.0);
                else if (useAzure)
                    csPtCurr = pColor2xyz[ptToDataIdxColor(x,y)];

                csPtDiffCurr.X = abs(abs(csPtCurr.X) - abs(csPtOrigin.X));
                csPtDiffCurr.Y = abs(abs(csPtCurr.Y) - abs(csPtOrigin.Y));
                csPtDiffCurr.Z = abs(abs(csPtCurr.Z) - abs(csPtOrigin.Z));

                if (lessOrEqual(csPtDiffCurr.X,csPtDiffStored.X) &&
                        lessOrEqual(csPtDiffCurr.Y,csPtDiffStored.Y) &&
                        csPtCurr.Z != 0)
                {
                    csPtDiffStored  =   csPtDiffCurr;
                    //csPtOrigin      =   csPtCurr;
                    pColorSpaceOrigin.X          =   x;
                    pColorSpaceOrigin.Y          =   y;
                }
            }
        }

        originDrawn = true;
        qDebug()<<pColorSpaceOrigin.X<<" "<<pColorSpaceOrigin.Y;
    }

    if (success == false)
        return;
    //bgImg = imread("d:\\bgImage.bmp");

    //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);

    //captured the zoomed in portion of the image
    Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);

    //draw a rectangle but dont zoom
    //startImg.copyTo(origCvImg);
    //rectangle(origCvImg,roi,CvScalar (255,0,0));

    origCvImg  = startImg(roi).clone();

    /*
    if (depthImageOnly)
    {
        QImage qimgChanged((uchar*) origCvImg.data,origCvImg.cols,origCvImg.rows,origCvImg.step, QImage::Format_RGBA8888 ); // for grayscale images

        labelOrigImage->setPixmap(QPixmap::fromImage(qimgChanged));
        labelOrigImage->adjustSize();
        labelOrigImage->parentWidget()->adjustSize();
        labelOrigImage->updateGeometry();
        labelOrigImage->parentWidget()->updateGeometry();
        return;
    }
*/
    //if(bgImg.empty()==false) bgImg       = bgImg(roi).clone();


    //*******let the processing begin********

    //bilateral filterting to reduce noise
    //bilateralFilter(bgImg,bgImg,5,15,15);
    //bilateralFilter(origCvImg,origCvImg,5,150,150);

    //median filterting to reduce noise
    //if(bgImg.empty()==false) medianBlur(bgImg,bgImg,7);
    //medianBlur(origCvImg,origCvImg,3);

    //convert to grayscale
    //if(bgImg.empty()==false) cvtColor(bgImg,bgImg,CV_BGR2GRAY);
    cvtColor(origCvImg,procCvImgGray,CV_BGR2GRAY);

    //background subtraction.
    //if(bgImg.empty()==false) absdiff(procCvImgGray,bgImg,procCvImgGray);

    if(preSelectShapeIndex==-1)
    {
        //blurring
        if (blurKerSz>0)
        {
            GaussianBlur(procCvImgGray,procCvImgGray,Size(3,3),0);
            //blur(procCvImgGray,procCvImgGray,Size(blurKerSz,blurKerSz));
        }

        //edging
        Canny(procCvImgGray, procCvImgEdged, cannyThLo,cannyThHi,3,true);

        //alternate edging technique
        /*
        Mat dx,dy;
        Scharr(procCvImgGray,dx,CV_16S,1,0);
        Scharr(procCvImgGray,dy,CV_16S,0,1);
        Canny( dx,dy, procCvImgEdged, cannyThLo, cannyThHi );
    */
        //closing
        if (closeKerSz>0)
        {
            Mat closingKernel = getStructuringElement(MORPH_RECT,Size(closeKerSz,closeKerSz));
            dilate(procCvImgEdged,procCvImgEdged,closingKernel,Point(-1,-1),closeKerItr);
            erode(procCvImgEdged,procCvImgEdged,closingKernel,Point(-1,-1),closeKerItr);
        }

        //random color generation code if needed
        //RNG rng(12345);
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

        contours.clear();
        //contouring
        findContours(procCvImgEdged,contours,hierarchy,CV_RETR_TREE,CHAIN_APPROX_SIMPLE);

        //delete edge touching countour
        for (int i = 0 ;i<contours.size();i++)
        {
            if(contourTouchesImageBorder(contours[i],procCvImgEdged.size()) ||
                contourArea(contours[i])<100)
            {//delete that contour right away
    #ifdef ENABLE_SPECPROC_DETAIL_LOGS
                    qDebug("edgeDetection - Contour no: %d, Contour area: %f, vector size: %d, contourTouchingBorder: %d",
                           i,contourArea(contours[i]),contours[i].size(),contourTouchesImageBorder(contours[i],procCvImgEdged.size()));
    #endif
                contours.erase(contours.begin()+i);
            }
        }

        if (maxContOnly && contours.size()>0)
        {
            int maxArea = 0,maxAreaIdx = 0;
            int cntrArea;
            for (int i = 0 ;i<contours.size();i++)
            {
                cntrArea = contourArea(contours[i]);
                if (cntrArea > maxArea)
                {
                    maxArea =cntrArea;
                    maxAreaIdx = i;
                }
            }

            vector<Point> tempCntr;
            vector<Point> hull;
            for (int j = 0 ;j<contours[maxAreaIdx].size();j++)
                tempCntr.push_back(contours[maxAreaIdx][j]);

            contours.clear();
            convexHull(tempCntr,hull,true,true);
            contours.push_back(hull);
        }
        numOfCountours = contours.size();
    }
    else if (preSelectShapeIndex < 10)
    {
        genTestPolygon();
    }

    //draw contours and markers
    labelOrigImage->setPixmap(QPixmap::fromImage(drawContoursAndMarkers(ctrImg,false,-1,true)));
    //labelOrigImage->setPixmap(QPixmap::fromImage((openCVtoQimg(startImg))));
    labelOrigImage->adjustSize();
    labelOrigImage->parentWidget()->adjustSize();
    labelOrigImage->updateGeometry();
    labelOrigImage->parentWidget()->updateGeometry();

    //proc image
    //emit processedImage(QPixmap::fromImage(openCVtoQimg(procCvImgEdged)));

    //draw contours and markers
    labelProcessImageDebug->setPixmap(QPixmap::fromImage(openCVtoQimg(procCvImgEdged)));
    //labelProcessImageDebug->adjustSize();
    //labelProcessImageDebug->parentWidget()->adjustSize();
    //labelProcessImageDebug->updateGeometry();
    //labelProcessImageDebug->parentWidget()->updateGeometry();
}

bool specImageProcessor::contourTouchesImageBorder(std::vector<cv::Point>& contour, cv::Size& imageSize)
{
    cv::Rect bb = cv::boundingRect(contour);

    bool retval = false;

    int xMin, xMax, yMin, yMax;

    xMin = 0;
    yMin = 0;
    xMax = imageSize.width - 1;
    yMax = imageSize.height - 1;

    // Use less/greater comparisons to potentially support contours outside of
    // image coordinates, possible future workarounds with cv::copyMakeBorder where
    // contour coordinates may be shifted and just to be safe.
    if( bb.x <= xMin ||
        bb.y <= yMin ||
        bb.width >= xMax ||
        bb.height >= yMax)
    {
        retval = true;
    }

    return retval;
}

QImage specImageProcessor::drawContoursAndMarkers(Mat &contorImg, bool enLog,int selectedContOnly, bool firstLastMarkerOnly)
{
    origCvImg.copyTo(contorImg);
    size_t i;
    int colSel;
    for (i = 0;i<contours.size();i++)
    {

        //if (contourArea(contours[i])>100 && contourArea(contours[i])<400000)
        //if(hierarchy[i][2] < 0) //&& hierarchy[i][3] < 0) //no child and no parent i.e be at the top
        {
            colSel = (i != selectedContOnly) ? 0 : 2;

            //drawContours( contorImg, contours, (int)i, colorTable[colSel], 2,LINE_8, hierarchy, 0, Point() );
            drawContours( contorImg, contours, (int)i, colorTable[i%4], 2,LINE_8, hierarchy, 0, Point() );
            //draw enclosing circle as well
            {
                //enclosing rectangle
                //rectangle(contorImg,boundingRect(contours[i]),colorTable[(colSel+1)%colorTableSize],2);
#if 0
                //enclosing cirlce
                Point2f cntr;
                float radius;
                minEnclosingCircle(contours[i],cntr,radius);
                circle(contorImg,cntr,radius,colorTable[(colSel+1)%colorTableSize],2);

                //min rotated rectangle
                RotatedRect rect = minAreaRect(contours[i]);

                Point2f rect_points[4];
                rect.points( rect_points );
                for( int j = 0; j < 4; j++ )
                   line( contorImg, rect_points[j], rect_points[(j+1)%4], colorTable[(colSel+1)%colorTableSize], 1, 8 );
#endif
            }
#ifdef ENABLE_SPECPROC_DETAIL_LOGS
            if (enLog)
                qDebug("drawContoursAndMarkers - Contour no: %d, Contour area: %f, vector size: %d",
                       i,contourArea(contours[i]),contours[i].size());
#endif

            //qDebug("***Contour # %d***",i);
            for (int j = 0;j<contours[i].size();j++)
            {
                drawMarker(contorImg,contours[i][j],colorTable[(colSel+2)%colorTableSize],MARKER_STAR,5);
                if (enLog)
                    //qDebug("Pt(x,y) # %d = (%d,%d)",j,contours[i][j].x,contours[i][j].y);
                if(firstLastMarkerOnly)
                {
                    j = contours[i].size() - 1;
                    drawMarker(contorImg,contours[i][j],colorTable[(colSel+3)%colorTableSize],MARKER_DIAMOND,5);
                }
            }

            //draw scannable area
            if(imageType == COL_DEP_IMAGE)
            {
                if (useKinect)
                {
                    if (robHomeclPx != Point2i(0,0) && robBotRightclPx != Point2i(0,0) &&
                        robHomeclPx != Point2i(1920,1080) && robBotRightclPx != Point2i(1920,1080))
                    {
                        cv::Rect robRange(robHomeclPx,robBotRightclPx+cv::Point(1,1));
                        rectangle(contorImg,robRange,colorTable[2],2,LINE_AA);
                        robRangeDrawn = true;
                    }
                }

                //if(originDrawn == false)
                {
                    Point2f cntr;
                    cntr.x = pColorSpaceOrigin.X;
                    cntr.y = pColorSpaceOrigin.Y;

                    circle(contorImg,cntr,3.0,colorTable[2],3.0);
                }
                //pColorSpaceOrigin.X<<" "<<pColorSpaceOrigin.Y;
            }
        }
    }
    return openCVtoQimg(contorImg);
}

QImage specImageProcessor::openCVtoQimg(Mat opencvImg, bool convKinectImage)
{
    bool coloured = opencvImg.channels() > 1;

    if (useKinect || convKinectImage || useAzure)
    {
        if (coloured)
        {
            if (opencvImg.type() == CV_32FC3) // for displaying normal map view
            {
                opencvImg.convertTo(globImg,CV_8UC4);
            }
            cvtColor(opencvImg,globImg,CV_BGRA2RGBA);
        }

        QImage qimgChanged((uchar*) globImg.data,globImg.cols,globImg.rows,globImg.step,coloured ? QImage::Format_RGBA8888: QImage::Format_Indexed8 ); // for grayscale images
        return  qimgChanged;
    }
    else
    {
        if (coloured)
            cvtColor(opencvImg,opencvImg,CV_BGR2RGB);

        QImage qimgChanged((uchar*) opencvImg.data,opencvImg.cols,opencvImg.rows,opencvImg.step,coloured ? QImage::Format_RGB888: QImage::Format_Indexed8 ); // for grayscale images
        return  qimgChanged;
    }
}

//This generates a poly gon for testing overall system.
//The edge detected or user input polygon is replaced by this one.
void specImageProcessor::genTestPolygon(int testNo)
{
    int Height;
    int Width;
    Point StartPoint;
    vector<Point> tempCntr;
    //StartPoint = Point(400,300);

    StartPoint  = preSelectShapePoint;
    testNo      = preSelectShapeIndex;

    Height      = scanInfoPtr->scanHeight;
    Width       = scanInfoPtr->scanWidth;

    if(testNo == 0)
    {//square
        //Width = 100;
        //Height = 250;

        tempCntr.push_back(StartPoint);
        tempCntr.push_back(StartPoint+Point(Width,0));
        tempCntr.push_back(StartPoint+Point(Width,Height));
        tempCntr.push_back(StartPoint+Point(0,Height));
    }
    else if (testNo == 1)
    {
        //Rectangle
        Width = 2* Height;

        Width  = 100;
        Height = 100;

        tempCntr.push_back(StartPoint);
        tempCntr.push_back(StartPoint+Point(Width,0));
        tempCntr.push_back(StartPoint+Point(Width,Height));
        tempCntr.push_back(StartPoint+Point(0,Height));
    }
    else if (testNo == 2)
    { //

        Width = Height;
        Width  = 5;
        Height = 200;

        tempCntr.push_back(StartPoint);
        tempCntr.push_back(StartPoint+Point(Width,0));
        tempCntr.push_back(StartPoint+Point(Width,Height));
        tempCntr.push_back(StartPoint+Point(0,Height));

//        tempCntr.push_back(StartPoint);
//        tempCntr.push_back(StartPoint+Point(Width,-Height));
//        tempCntr.push_back(StartPoint+Point(Width,2*Height));
//        tempCntr.push_back(StartPoint+Point(0,Height));
    }

    else if (testNo == 3)
    {
        //Polygon
        Width =Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(2*Width,-Height));//3
        tempCntr.push_back(StartPoint+Point(3*Width,0));//4
        tempCntr.push_back(StartPoint+Point(3*Width,Height));//5
        tempCntr.push_back(StartPoint+Point(2*Width,2*Height));//6
        tempCntr.push_back(StartPoint+Point(Width,2*Height));//7
        tempCntr.push_back(StartPoint+Point(0,Height));//8
    }

    else if (testNo == 4)
    {
        //Diamond
        Width = 2* Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(2*Width,0));//3
        tempCntr.push_back(StartPoint+Point(Width,Height));//4
    }

    else if (testNo == 5)
    {
        //make a diamond
        Width = Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(2*Width,-Height));//3
        tempCntr.push_back(StartPoint+Point(3*Width,0));//4
        tempCntr.push_back(StartPoint+Point(3*Width,Height));//5
        tempCntr.push_back(StartPoint+Point(2*Width,2*Height));//6
        tempCntr.push_back(StartPoint+Point(Width,2*Height));//7
        tempCntr.push_back(StartPoint+Point(0,Height));//8
    }
    else if (testNo == 6)
    {
        //rightangle - triangle
        Width = Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(Width,0));//3
    }
    else if (testNo == 7)
    {
        //triangle
        Width = Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(2*Width,0));//3
    }
    else if (testNo == 8)
    {
        //triangle
        Width = Height;
        contours.clear();
        return;
    }
    else //default shape
    {
        //triangle
        Width = Height;

        tempCntr.push_back(StartPoint);//1
        tempCntr.push_back(StartPoint+Point(Width,-Height));//2
        tempCntr.push_back(StartPoint+Point(2*Width,0));//3

    }

    //custom contour for debugging
    /*
     *
     *  tempCntr.push_back(Point(336,112));//1
        tempCntr.push_back(Point(327,273));//2
        tempCntr.push_back(Point(370,270));//3
        tempCntr.push_back(Point(394,189));//4
*/

    contours.clear();

    contours.push_back(tempCntr);
    selectedContour = 0;

    //labelOrigImage->setPixmap(QPixmap::fromImage(drawContoursAndMarkers(ctrImg,false,selectedContour)));
    //labelOrigImage->adjustSize();
}
void specImageProcessor::pColorToAngle(int xPx, int yPx, int &phiAngle, int &thetaAngle)
{
    int colorIndex;
    DepthSpacePoint p;
    int depthX;
    int depthY;

    if (useAzure)
    {
        thetaAngle  = thetaFiltMat.at<short>(yPx,xPx);
        phiAngle    = phiFiltMat.at<short>(yPx,xPx);
    }
    else if (useKinect)
    {
        if (flipImage)
            xPx = (colorWidth - 1) - xPx;

        colorIndex = xPx+((yPx)*colorWidth);
        p = pColor2depth[colorIndex];

        depthX = static_cast<int>(p.X + 0.5f);
        depthY = static_cast<int>(p.Y + 0.5f);

        thetaAngle  = thetaFiltMat.at<short>(depthY,(depthWidth - 1)-depthX);
        phiAngle    = phiFiltMat.at<short>(depthY,(depthWidth - 1)-depthX);
    }
}

bool specImageProcessor::calcScanGrid()
{
    //stop the image update if required
    enASScanUpdateTimer(true);
    while(pColor2xyz == NULL) // wait to get a valid read from KINECT
        QCoreApplication::processEvents();

    enASScanUpdateTimer(false);
    float pathLength;
    float interval;
    int scLnPtCnt;
    ICoordinateMapper* m_pCoordinateMapper = NULL;
    QVector<QVector<Point5f>> scanGridKinect;
    QVector<Point5f> scanLineKinect;
    Point5f scanPthWithAngleKinect;
    QVector<unsigned char> scLnPtCntArrKinect;
    int scanStartTheta,curLineTheta,thetaForGrid,currentPhi,currentTheta;
    int scanStartPhi,curLinePhi,phiForGrid;

    int phiTemp;
    _CameraSpacePoint csPtLineStart,exactCsPt,csPtScanStart;
    _CameraSpacePoint csPtLineStartPrev; // stores the line start of prev line
    _CameraSpacePoint csPtLineEnd;

    Point2i clPxLineStart,clPxLineEnd,clPxForLineStartX;
    int clPxLineStartX,scanLineNumber;
    float csPtLineStartNextX;

    scanGridKinect.clear();
    scLnPtCntArrKinect.clear(); // reduncdant but safe

    interval    = scanInfoPtr->scanInterval;
    pathLength  = scanInfoPtr->scanSpeedArr[scanInfoPtr->scanSpeedIndex].pathLen;

    if (selectedContour==-1)
    {
        if (contours.empty() == false)
            selectedContour = 0;
    }

    //allowDistanceAdjust = scanInfoPtr->enableDepthControl;
    allowDistanceAdjust = false;
    //************
    //1.Get the min/max value of X from the selected contour. This can be done via bounding rect calculation
    int minX,maxX,i,j,k,curX;
    float tempX;
    maxX = 0;
    minX = origCvImg.cols;
    curX = 0;

    for (i=0;i<contours[selectedContour].size();i++)
    {
        curX = contours[selectedContour][i].x;

        if (curX > maxX)
            maxX = curX;
        if (curX < minX)
            minX = curX;
    }
    qDebug("specImageProcessor::calcScanGrid() - maxX = %d, minX = %d" ,maxX,minX);

    scanLineNumber  = 0;
    scLnPtCnt       = 0;
    clPxLineStartX  = minX;
    while(clPxLineStartX <= maxX)
    {
        //2. Make a vertical line at the start position to calc the start and end pixel location.
        lineData vertLine;
        vertLine.pt1 = Point2f(clPxLineStartX,0);
        vertLine.pt2 = Point2f(clPxLineStartX,colorHeight);

        //3. Check intersection of vertical line to find the start and end pixel location.
        Point2f intersecPx = (0,0);
        QVector<Point2i> intersecPxVec;

        bool isIntersecting = false;
        for (j = 0; j<contours[selectedContour].size(); j++)
        {
            isIntersecting = lineIntersection(vertLine.pt1,vertLine.pt2,
                             contours[selectedContour][j],contours[selectedContour][(j+1)%(contours[selectedContour].size())],
                             intersecPx);

            if (isIntersecting)
            {
                bool isNewPoint = true;
                for (k = 0 ; k<intersecPxVec.size();k++)
                {
                    if (almostEqual(intersecPxVec[k].x,intersecPx.x) &&
                        almostEqual(intersecPxVec[k].y,intersecPx.y))
                    {
                        isNewPoint = false;
                        break;
                    }
                }
                if (isNewPoint)
                    intersecPxVec.push_back(intersecPx);
            }
        }

        clPxLineStart   = intersecPxVec[0];
        clPxLineEnd     = intersecPxVec[1];
        csPtLineStart   = mTomm(pColor2xyz[ptToDataIdxColor(clPxLineStart.x,clPxLineStart.y)],true,1.0);
        csPtLineEnd     = mTomm(pColor2xyz[ptToDataIdxColor(clPxLineEnd.x,clPxLineEnd.y)],true,1.0);

        pColorToAngle(clPxLineStart.x,clPxLineStart.y,curLinePhi,curLineTheta);
        if (scanLineNumber == 0)
        {
            csPtScanStart       =   csPtLineStart;
            csPtLineStartPrev   =   csPtLineStart;
            scanStartPhi        =   curLinePhi;
            scanStartTheta      =   curLineTheta;
        }

        scLnPtCnt       = 0;
        scanLineKinect.clear();

        _CameraSpacePoint searchPtTemp;
        _CameraSpacePoint searchPtTempAvg;
        _CameraSpacePoint csPt,csPtNext; //camera spcae point
        Point2i clPx,clPxNext; //colour pixel point

        clPx    = clPxLineStart;
        {// beacuse of the coarse resolution in x-axis compared to interval we may not move a pixel but the movement in x-axis must be ensured
            csPt.Y = csPtLineStart.Y;
            csPt.X = csPtScanStart.X - scanLineNumber * interval;
            //if ( allowDistanceAdjust )
            if (true)
            {
                if(/*fabs(csPtScanStart.Z - csPtLineStart.Z) > 3.0  &&
                     fabs(csPtLineStartPrev.Z - csPtLineStart.Z)>5.0 &&*/
                     csPtLineStart.Z != -std::numeric_limits<float>::infinity() &&
                     csPtLineStart.Z != 0 )
                    {
                        csPt.Z = csPtLineStart.Z;
                    }
            }
            else
                csPt.Z = csPtScanStart.Z;

            if (scanInfoPtr->enableOrientControl)
            {
                phiForGrid      = curLinePhi;
                thetaForGrid    = curLineTheta;
            }
            else
            {
                phiForGrid      = 0;
                thetaForGrid    = 0;
            }

            if (scanLineNumber == 0)
            {
                csPt.Z          =   csPtScanStart.Z;
            }

            csPtLineStartPrev   =   csPtLineStart;
        }

        while(clPx.y <= clPxLineEnd.y )
        //while(greaterOrEqual(csPt.Y,csPtLineEnd.Y))
        {
            //csPt.Z = csPtScanStart.Z; // Always keep the same depth just for test.

            scanPthWithAngleKinect.X = csPt.X;scanPthWithAngleKinect.Y = csPt.Y;scanPthWithAngleKinect.Z = csPt.Z;
            scanPthWithAngleKinect.theta = thetaForGrid;scanPthWithAngleKinect.phi = phiForGrid;

            scanLineKinect.append(scanPthWithAngleKinect);

            scLnPtCnt++;
            // THE SEARCH FOR NEXT CSPt/CLPx
            // The next pixel - The Y value for a worthy pixel will be searched below.
            clPxNext.x  = clPx.x;

            //The next point - the Z value for a pixel nearest to the Y value will be searched below.
            csPtNext.X  = csPt.X;
            csPtNext.Y  = csPt.Y - pathLength;

            //search this cs point nearest in value to Y in the camera space
            int pixelIndex = 0;
            float prevDiff = 500,curDiff = 499;
            while(greaterOrEqual(prevDiff,curDiff))
            {
                pixelIndex ++;
                prevDiff = curDiff;
                searchPtTemp    = mTomm(pColor2xyz[ptToDataIdxColor(clPx.x,clPx.y+pixelIndex)]);
                searchPtTempAvg = mTomm(pColor2xyzAvg[ptToDataIdxColor(clPx.x,clPx.y+pixelIndex)]);
                curDiff = fabs(csPtNext.Y - searchPtTemp.Y);

                if (lessOrEqual(curDiff,prevDiff))
                { //note the Z for next CS point from the CS array
                    clPxNext.y  = clPx.y+pixelIndex;
                    exactCsPt   =   searchPtTempAvg;
                    {
                        csPtNext.Z = csPt.Z;
                        //allow only when the depth is decreasing for
                        if ( allowDistanceAdjust &&
                                /*fabs(csPtScanStart.Z - exactCsPt.Z) > 20.0  &&*/
                                 /*fabs(csPt.Z - exactCsPt.Z)>5.0 &&*/
                                 /*lessOrEqual(exactCsPt.Z, csPt.Z) && //specific to raidal+conic specimen can be generalized by measuring the depth slope.*/
                                 /*fabs(csPt.Z - exactCsPt.Z)<5.0 &&*/
                                 exactCsPt.Z != -std::numeric_limits<float>::infinity() &&
                                 exactCsPt.Z != 0 )
                        {
                            //get depth from the avergaed buffer
                            csPtNext.Z   = exactCsPt.Z;
                        }

                        if (scanInfoPtr->enableOrientControl &&
                            exactCsPt.Z != -std::numeric_limits<float>::infinity())
                        {
                            pColorToAngle(clPx.x,clPx.y,currentPhi,currentTheta);

                            //yaw angle control
                            if( abs(phiForGrid - currentPhi)>10 )
                            {
                                phiForGrid = currentPhi;
                            }

                            //pitch angle control
                            if( abs(thetaForGrid - currentTheta)>10 )
                            {
                                thetaForGrid = currentTheta;
                            }
                        }
                    }
                }
                //qDebug("calcScanGrid - search nearest to desired Y : %f, pixelIndex: %d, currDiff: %f, prevDiff: %f",
                  //     csPtNext.Y,pixelIndex,curDiff,prevDiff);
            }
            clPx = clPxNext;
            csPt = csPtNext;
        }

        //add this scan line to the array.
        scanGridKinect.append(scanLineKinect);
        scLnPtCntArrKinect.append(scanLineKinect.length());

        qDebug("calcScanGrid - Added scanLine number: %d, length(scanLineKinect): %d, scLnPtCnt: %d, length(scLnPtCntArrKinect): %d, scanLineNumber: %d",
               scanLineNumber,scanLineKinect.length(),scLnPtCnt,scLnPtCntArrKinect.length(),scanLineNumber);

        scanLineNumber++;
        //pixel.x for the next scan line
        //search to the right for the CS point with minimum error to the csPtLineStartNext.X
        //use that as the next X value to drive a vertical line and find the start and point clPx

        int pixelIndex = 0;
        float prevDiff = 500,curDiff = 499;

        csPtLineStartNextX = csPtScanStart.X - (scanLineNumber*interval);

        while(greaterOrEqual(prevDiff,curDiff) /*&& curDiff < interval*/)
        {
            prevDiff     = curDiff;
            searchPtTemp = mTomm(pColor2xyz[ptToDataIdxColor(clPxLineStart.x + pixelIndex,clPxLineStart.y)]);

            curDiff      = fabs(csPtLineStartNextX - searchPtTemp.X);

            if (lessOrEqual(curDiff,prevDiff))
                clPxLineStartX = clPxLineStart.x + pixelIndex;

            qDebug("calcScanGrid - search nearest to desired X - pixelIndex: %d,csPtLineStartNextX : %f, searchPtTemp.X: %f, curDiff: %f, prevDiff: %f, clPxLineStartX: %d",
                   pixelIndex,csPtLineStartNextX,searchPtTemp.X,curDiff,prevDiff,clPxLineStartX);

            pixelIndex ++; //not necessary to move the pixel index everytime

//            if (searchPtTemp.X<0)
//            {//exception
//                qDebug()<<"Caught an Exception searchPtTemp.X < 0";
//                return 0 ;
//            }
        }

         //HRESULT hr = pMyKinect->get_CoordinateMapper(&m_pCoordinateMapper);
         //m_pCoordinateMapper->MapCameraPointToColorSpace(csPtLineStartNext,&clPxForLineStartX);
    }

    //.4 Convert scangrid from Kinect to robot units.
    scanInfoPtr->scLnPtCntArr.clear();
    scanInfoPtr->scanGrid.clear();
    QVector<structRobTarget> tempScaLineRobot;
    Point3f KinPt;
    Point3f robPtTrans,robPtTransWAngle;
    structRobTarget robPoint;
    structRobTarget robTarget;

    // always perpendicular
    robTarget.rot.q1 = 0.707106781;
    robTarget.rot.q2 = 0;
    robTarget.rot.q3 = 0.707106781;
    robTarget.rot.q4 = 0;

    int scLnIdx,ptIdx;
    for(scLnIdx = 0;scLnIdx<scLnPtCntArrKinect.length();scLnIdx++)
    {
        scanInfoPtr->scLnPtCntArr.append(scLnPtCntArrKinect[scLnIdx]);
        tempScaLineRobot.clear();
        for(ptIdx = 0;ptIdx<scLnPtCntArrKinect[scLnIdx];ptIdx++)
        {
            KinPt.x = scanGridKinect[scLnIdx][ptIdx].X;
            KinPt.y = scanGridKinect[scLnIdx][ptIdx].Y;
            KinPt.z = scanGridKinect[scLnIdx][ptIdx].Z;

            kinToRobUnits(KinPt,robTarget,scanGridKinect[scLnIdx][ptIdx].phi,scanGridKinect[scLnIdx][ptIdx].theta);

            if(isRobPtWihinRange(robTarget.transAfterAngle) == false)
            {
                qDebug("A scan point is outside the range. scanPoint[%d][%d]",scLnIdx,ptIdx);
                return false;
            }

            // change angle to orientation and write *********************************

            tempScaLineRobot.append(robTarget);

            if (ptIdx == 0 || ptIdx == scLnPtCntArrKinect[scLnIdx] - 1 ) //verbose time
            {
               qDebug("calcScanGrid - (x,y,z,phi,theta)scanGridKinect[%d][%d]: %.1f,%.1f,%.1f,%.1f,%.1f scanGridTrans[%d][%d]: %.1f,%.1f,%.1f scanGridTransAfterAngle[%d][%d]: %.1f,%.1f,%.1f",
                      scLnIdx,ptIdx,scanGridKinect[scLnIdx][ptIdx].X,scanGridKinect[scLnIdx][ptIdx].Y,scanGridKinect[scLnIdx][ptIdx].Z,scanGridKinect[scLnIdx][ptIdx].phi,scanGridKinect[scLnIdx][ptIdx].theta,
                      scLnIdx,ptIdx,tempScaLineRobot[ptIdx].trans.x,tempScaLineRobot[ptIdx].trans.y,tempScaLineRobot[ptIdx].trans.z,
                      scLnIdx,ptIdx,tempScaLineRobot[ptIdx].transAfterAngle.x,tempScaLineRobot[ptIdx].transAfterAngle.y,tempScaLineRobot[ptIdx].transAfterAngle.z);
            }
        }
        scanInfoPtr->scanGrid.append(tempScaLineRobot);
    }
////////////////////////////////////////////////////////////////////////////////////////////////
    //6. Extract info for the robot interface
    //6-1 - Start point of scan.

    scanInfoPtr->startScanPosRob = scanInfoPtr->scanGrid[0][0];

    //6-2 - Length of each line and supporting pars for easy access lateron, store into the global vector.
    scanInfoPtr->scanLinesASvec.clear();

    unsigned int accTrigCalc    = 0;
    unsigned int trigPerLine    = 0;
    float scanIntervalLoc       = scanInfoPtr->scanInterval;
    float distance;
    float maxDistance = 0;
    float maxDistanceY = 0;
    float startPt,endPt;

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
        if (scLnIdx%2 == 0)
        {
            startPt         = scanInfoPtr->scanGrid[scLnIdx][0].trans.z;
            endPt           = scanInfoPtr->scanGrid[scLnIdx][(scanInfoPtr->scLnPtCntArr[scLnIdx]-1)].trans.z;
        }
        else
        {
            startPt         = scanInfoPtr->scanGrid[scLnIdx][(scanInfoPtr->scLnPtCntArr[scLnIdx]-1)].trans.z;
            endPt           = scanInfoPtr->scanGrid[scLnIdx][0].trans.z;
        }

        distance        = fabs(scanInfoPtr->scanGrid[scLnIdx][0].trans.z - scanInfoPtr->scanGrid[scLnIdx][(scanInfoPtr->scLnPtCntArr[scLnIdx]-1)].trans.z);
        //trigPerLine   = (distance/scanIntervalLoc+1); //robot generates 1 extra trigger???? NEED TO CHECK
        trigPerLine     = (distance/scanIntervalLoc); //robot generates 1 extra trigger
        accTrigCalc    += trigPerLine;

        scanInfoPtr->scanLinesASvec.append({startPt,endPt,distance,accTrigCalc,trigPerLine,0});

        //used in stage AS , maybe not necessary
//        if (distance>maxDistance)
//        {
//            maxDistance  = distance;
//            maxDistanceY = allIntersecPts[i][0].y;
//        }
    }
    daqInfoPtr->ScanPoints = accTrigCalc;

    //compute the bounding box i.e min max x,y by self.
    float minXf,maxXf,curXf,minYf,maxYf,curYf;
    maxXf = 0;
    minXf = origCvImg.cols;
    curXf = 0;

    maxYf = 0;
    minYf = origCvImg.rows;
    curYf = 0;

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
        for(ptIdx  = 0;ptIdx<scanInfoPtr->scLnPtCntArr[scLnIdx];ptIdx++)
        {
            curXf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // y axis of robot is x axis in kinect/image
            if (greaterOrEqual(curXf,maxXf))
                maxXf = curXf;
            if ((lessOrEqual(curXf,minXf)))
                minXf = curXf;

            curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z; // z axis of robot is y axis in kinec/image
            if (greaterOrEqual(curYf,maxYf))
                maxYf = curYf;
            if ((lessOrEqual(curYf,minYf)))
                minYf = curYf;
        }
    }

    qDebug("calcScanGrid() - Last Step - maxX = %f, minX = %f, maxY = %f, minY = %f" ,
           maxXf,minXf,maxYf,minYf);

    //6-3 - Width and height of the bounding rect - useful for the spectrogram
    scanInfoPtr->scanHeight = maxYf - minYf;
    scanInfoPtr->scanWidth = maxXf - minXf;
    scanInfoPtr->startRectPos.setX(maxXf);
    scanInfoPtr->startRectPos.setY(maxYf);

    scanInfoPtr->rightBottomRectPos.setX(minXf);
    scanInfoPtr->rightBottomRectPos.setY(minYf);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    enASScanUpdateTimer(true);

    lmsPtr->printLmsScanGrid();
    return true;
}
inline _CameraSpacePoint specImageProcessor::mTomm(_CameraSpacePoint InM, bool enStickToGrid, float gridSpacingmm)
{
    _CameraSpacePoint loc;

    if (enStickToGrid)
    {
       loc.X =  stickToGrid(InM.X * 1000,gridSpacingmm);
       loc.Y =  stickToGrid(InM.Y * 1000,gridSpacingmm);
       loc.Z =  stickToGrid(InM.Z * 1000,gridSpacingmm);
       //loc.Z    =   InM.Z * 1000;
    }
    else
    {
        loc.X = InM.X * 1000;
        loc.Y = InM.Y * 1000;
        loc.Z = InM.Z * 1000;
    }
    return loc;
}

bool specImageProcessor::calParsForStage()
{
    if (selectedContour==-1)
    {
        if (contours.empty() == false)
            selectedContour = 0;
    }
    //***PRE_REQS
    //needs interval to be provided lets use the local var for now.
    float interval  = 2.0;//0.1,0.25,0.5
    interval        = scanInfoPtr->scanInterval;
    //needs to be calculated accurately later on this->
    //pxPermmX     = 2.4 ;
    //pxPermmY     = 3.2 ;

    //************
    //1.Get the min/max value of X from the selected contour. This can be done via bounding rect calculation
    int minX,maxX,i,j,k,curX;
    float tempX;
    maxX = 0;
    minX = origCvImg.cols;
    curX = 0;

    for (i=0;i<contours[selectedContour].size();i++)
    {
        curX = contours[selectedContour][i].x;

        if (curX > maxX)
            maxX = curX;
        if (curX < minX)
            minX = curX;
    }
    qDebug("specImageProcessor::calParsForStage() - maxX = %d, minX = %d" ,maxX,minX);

    //efficient to just use the boundingRect.
    //Rect boundRect;
    //boundRect = boundingRect(contours[selectedContour]);
    //qDebug("specImageProcessor::calParsForStage() - maxX = %d, minX = %d --via bounding Rect", boundRect.x+boundRect.width,boundRect.x);

    //2. Make vertical lines separated by interval throughout the X-range (minX~maxX).
    vector<float> x;
    i=0;
    do
    {
        tempX = minX+((i++)*interval*pxPermm->x);
        x.push_back(tempX);
    }while (tempX<maxX);

    vector<lineData> scanLines;
    lineData tempLine;
    int maxY = origCvImg.rows;

    for (i = 0;i<x.size();i++ )
    {
        tempLine.pt1 = Point2f(x[i],0);
        tempLine.pt2 = Point2f(x[i],maxY);
        scanLines.push_back(tempLine);
    }

    //3. Check intersection of each scan line one by one with all of the lines of contour.
    Point2f tempIntersecPt = (0,0);
    QVector<Point2f> tempIntersecPtVec;
    QVector<QVector<Point2f>> allIntersecPts;
    bool isIntersecting = false;
    for (i = 0;i<x.size();i++ )
    {
        for (j = 0; j<contours[selectedContour].size(); j++)
        {

            isIntersecting = lineIntersection(scanLines[i].pt1,scanLines[i].pt2,
                             contours[selectedContour][j],contours[selectedContour][(j+1)%(contours[selectedContour].size())],
                             tempIntersecPt);

            if (isIntersecting)
            {
                bool isNewPoint = true;
                for (k = 0 ; k<tempIntersecPtVec.size();k++)
                {
                    if (almostEqual(tempIntersecPtVec[k].x,tempIntersecPt.x) &&
                        almostEqual(tempIntersecPtVec[k].y,tempIntersecPt.y))
                    {
                        isNewPoint = false;
                        break;
                    }
                }
                if (isNewPoint)
                    tempIntersecPtVec.push_back(tempIntersecPt);
            }
        }
        if (tempIntersecPtVec.size()>0)
        {  //only add to record if an intersecting point was indeed found
            allIntersecPts.push_back(tempIntersecPtVec);
            tempIntersecPtVec.clear();
        }
    }
    //4. Reduce intersecting points to 2 per scan line - Choose the extreme ones along the y-axis.
    for (i = 0;i<allIntersecPts.size();i++ )
    {
        //del the non-extreme index and sort the remaining 2 in ascending order
        float tempMin=100000,tempMax=0;
        int tempMinIdx, tempMaxIdx;
        for (j = 0;j<allIntersecPts[i].size();j++)
        {
            if (greaterOrEqual( allIntersecPts[i][j].y,tempMax))
            {
                tempMax = allIntersecPts[i][j].y;
                tempMaxIdx = j;
            }
            if (lessOrEqual( allIntersecPts[i][j].y,tempMin))
            {
                tempMin = allIntersecPts[i][j].y;
                tempMinIdx = j;
            }
        }
        QVector<Point2f>tempVec;
        tempVec.push_back(allIntersecPts[i][tempMinIdx]);
        tempVec.push_back(allIntersecPts[i][tempMaxIdx]);
        allIntersecPts[i].clear();

        allIntersecPts[i].push_back(tempVec[0]); //min first
        allIntersecPts[i].push_back(tempVec[1]); //max second
    }

    //Debugging code to veiw the results
    Mat tempImage;
    origCvImg.copyTo(tempImage);
    for (i = 0;i<allIntersecPts.size();i++ )
    {
        for (k = 0;k<allIntersecPts[i].size();k++ ) // this should be only 2 as the extremes are selected in the code above.
            drawMarker(tempImage,Point(allIntersecPts[i][k].x,allIntersecPts[i][k].y),colorTable[(k+3)%5],MARKER_STAR,5);
    }
    //proc image
    //emit processedImage(QPixmap::fromImage(openCVtoQimg(tempImage)));
    //draw contours and markers
    labelProcessImageDebug->setPixmap(QPixmap::fromImage(openCVtoQimg(tempImage)));
    labelProcessImageDebug->adjustSize();
    labelProcessImageDebug->parentWidget()->adjustSize();
    labelProcessImageDebug->updateGeometry();
    labelProcessImageDebug->parentWidget()->updateGeometry();

    //5-1 Convert from pixel to stage mm.
    for (i = 0;i<allIntersecPts.size();i++ )
    {
        convToStageUnits(allIntersecPts[i][0],allIntersecPts[i][0],1,i==0);
        convToStageUnits(allIntersecPts[i][1],allIntersecPts[i][1],1,i==0);

        if (allIntersecPts[i][0].x < 0  ||
            allIntersecPts[i][0].y < 0  ||
            allIntersecPts[i][1].x < 0  ||
            allIntersecPts[i][1].y < 0  ||
            allIntersecPts[i][0].x > 500.0 ||
            allIntersecPts[i][0].y > 900.0 ||
            allIntersecPts[i][1].x > 500.0 ||
            allIntersecPts[i][1].y > 900.0)
        {
            qDebug("specImageProcessor::calParsForStage() - Point out of stage limits - LineNumber: %d/ %d", i,allIntersecPts.size());
            qDebug()<<allIntersecPts[i][0].x<<allIntersecPts[i][0].y<<allIntersecPts[i][1].x<<allIntersecPts[i][1].x;
            QMessageBox msgBox(QMessageBox::Warning, tr("Un-accessible"),tr("Selected area is not accessible due to stage limits."), 0);
            msgBox.exec();
            return false;
        }

        //zero len @corners
        if(allIntersecPts[i][0] == allIntersecPts[i][1])
        {
            qDebug("specImageProcessor::calParsForStage() - Start/End Pt same - Deleted scanline %d", i);
            allIntersecPts.remove(i);
            i--;
        }
    }

    if(allIntersecPts.empty())
            return false;
    //6. Extract info for the stage interface
    //6-1 - Start point of scan.
    scanInfoPtr->startScanPos.setX(allIntersecPts[0][1].x);
    scanInfoPtr->startScanPos.setY(allIntersecPts[0][1].y);


    //6-2 - Length of each line and store into the global vector.
    scanInfoPtr->scanLinesASvec.clear();

    unsigned int accTrigCalc    = 0;
    unsigned int trigPerLine = 0;
    float scanIntervalLoc       = scanInfoPtr->scanInterval;
    float distance;
    float maxDistance = 0;
    float maxDistanceY = 0;
    float startPt;
    //allIntersecPts[i][0]; //->max first
    //allIntersecPts[i][1]; //->min second
    for (i = 0;i<allIntersecPts.size(); i++)
    {
        startPt         = (i%2 == 0) ? allIntersecPts[i][1].y: allIntersecPts[i][0].y;
        distance        = allIntersecPts[i][0].y - allIntersecPts[i][1].y;
        trigPerLine     = (distance/scanIntervalLoc+1); //stage generates 1 extra trigger
        accTrigCalc    += trigPerLine;
        scanInfoPtr->scanLinesASvec.append({startPt,distance,accTrigCalc,trigPerLine,0});

        //measure height based on pars being written to stage
        if (distance>maxDistance)
        {
            maxDistance  = distance;
            maxDistanceY = allIntersecPts[i][0].y;
        }
    }
    daqInfoPtr->ScanPoints = accTrigCalc;

    //compute the bounding box i.e min max x/y by self.
    float minXf,maxXf,curXf,minYf,maxYf,curYf;
    maxXf = 0;
    minXf = origCvImg.cols;
    curXf = 0;

    maxYf = 0;
    minYf = origCvImg.rows;
    curYf = 0;

    for (j=0;j<allIntersecPts.size();j++)
    {
        for (i=0;i<2;i++)
        {
            curXf = allIntersecPts[j][i].x;
            if (greaterOrEqual(curXf,maxXf))
                maxXf = curXf;
            if ((lessOrEqual(curXf,minXf)))
                minXf = curXf;

            curYf = allIntersecPts[j][i].y;
            if (greaterOrEqual(curYf,maxYf))
                maxYf = curYf;
            if ((lessOrEqual(curYf,minYf)))
                minYf = curYf;
        }
    }
    qDebug("specImageProcessor::calParsForStage() - Last Step - maxX = %f, minX = %f, maxY = %f, minY = %f" ,
           maxXf,minXf,maxYf,minYf);


    //6-3 - Width and height of the bounding rect - useful for the spectrogram
    scanInfoPtr->scanHeight = maxYf - minYf;
    scanInfoPtr->scanWidth = maxXf - minXf;
    scanInfoPtr->startRectPos.setX(minXf);
    scanInfoPtr->startRectPos.setY(maxYf);

    //6-4
    scanInfoPtr->preScanPos.setX(stageCont->getPosX());
    scanInfoPtr->preScanPos.setY(stageCont->getPosZ());

    return true;
}

/*
 * kinOrigin : always bottom-right
 * kinPoint follows Kinect axis notation
 * robPoint follows robot axis notation
 * Kinect(x,y,z)  -> Robot(y,z,x);
*/
inline void specImageProcessor::kinToRobUnits(Point3f kinPoint, structRobTarget &robPoint, float yaw,float pitch, bool enLog, originType outOriginType)
{
    structRobTarget Ps; // impinge point on the specimen in robot base coordinate system
    structRobTarget Pv; //Pv.Trans contains LDV beam vector from specimen point to the LDV
    int LSD = scanInfoPtr->ldvStandOffDistance;

    Ps.trans.y  =  kinPoint.x - calibData.kinCordOffset.x;
    Ps.trans.z  =  kinPoint.y - calibData.kinCordOffset.y;
    Ps.trans.x  =  kinPoint.z - calibData.kinCordOffset.z;

    //move the robot back LSD
    //if (scanInfoPtr->enableDepthControl)//There is a aprox 10mm overshoot , reasons un-known.
        robPoint.trans.x = Ps.trans.x - LSD;
    //else
    //    robPoint.trans.x = robotCont->homePos.trans.x;

    robPoint.trans.y = Ps.trans.y;
    robPoint.trans.z = Ps.trans.z;


    //incase orientation control is not enabled
    robPoint.transAfterAngle = robPoint.trans;
    robPoint.rot.q1 = 0.707106781;
    robPoint.rot.q2 = 0;
    robPoint.rot.q3 = 0.707106781;
    robPoint.rot.q4 = 0;

    if(scanInfoPtr->enableOrientControl)
    {
        //Pv.Trans contains LDV beam vector from specimen point to the LDV
        //i.e. Ps = {-scanInfoPtr->ldvStandOffDistance,0,0} --> robPoint = robPoint-Ps
        structRobTarget Pv;

        Pv.trans.x = -1*LSD;
        Pv.trans.y = 0;
        Pv.trans.z = 0;

        //Pr.Trans contains the result of the rotations applied to Pv.
        structRobTarget Pr;

        //rotate
        Pr.trans.x  =  -LSD * qCos(qDegreesToRadians(pitch)) * qCos(qDegreesToRadians(yaw));

        //yaw ->pitch
//        Pr.trans.y  =   LSD * qSin(qDegreesToRadians(yaw));
//        Pr.trans.z  =   LSD * qSin(qDegreesToRadians(pitch)) * qCos(qDegreesToRadians(yaw));
        //pitch->yaw
        Pr.trans.y  =   LSD * qSin(qDegreesToRadians(yaw)) * qCos(qDegreesToRadians(pitch));
        Pr.trans.z  =   LSD * qSin(qDegreesToRadians(pitch));

        //add the base to the vector so it is representated in the Robot BASE coordinates
        robPoint.transAfterAngle.x = Pr.trans.x + Ps.trans.x;
        robPoint.transAfterAngle.y = Pr.trans.y + Ps.trans.y;
        robPoint.transAfterAngle.z = Pr.trans.z + Ps.trans.z;

//        if (pitch>10)
//            robPoint.transAfterAngle.y +=1;

        eulerToQuaternion(yaw,pitch,robPoint.rot);
    }

    if(enLog)
        qDebug("kinToRobUnits - kinPoint(x,y,z) : (%f,%f,%f) , robPoint(y,z,x): (%f,%f,%f), yaw: %f , pitch: %f "
               "robPointAfterAngle(y,z,x): (%f,%f,%f)",
               kinPoint.x,kinPoint.y,kinPoint.z,robPoint.trans.y,robPoint.trans.z,robPoint.trans.x,
               yaw,pitch,
               robPoint.transAfterAngle.y,robPoint.transAfterAngle.z,robPoint.transAfterAngle.x);

}

void specImageProcessor::eulerToQuaternion(short yaw,short pitch, structOrient &robOrient )
{
    //theta       = 0;
    //short phi     = 0;
    short roll    = 0;

    //update orientation for negative angles
//    if (theta<0)
//    {
//       yaw     = theta;  //%rotation along Z-axis -- according to Theta
//       pitch   = -90+phi; //%rotation along Y-axis  -- according to Phi -- the starting condition = 90
//       roll    = 180;
//    }

    //pitch  *= -1;// inversion becuase the following code is written for y increasing to the right
    pitch  += 90;
    yaw    *= -1;

    float cy = qCos(qDegreesToRadians(yaw * 0.5));
    float sy = qSin(qDegreesToRadians(yaw * 0.5));
    float cp = qCos(qDegreesToRadians(pitch * 0.5));
    float sp = qSin(qDegreesToRadians(pitch * 0.5));
    float cr = qCos(qDegreesToRadians(roll * 0.5));
    float sr = qSin(qDegreesToRadians(roll * 0.5));

    robOrient.q1 = cy * cp * cr + sy * sp * sr;
    robOrient.q2 = cy * cp * sr - sy * sp * cr;
    robOrient.q3 = sy * cp * sr + cy * sp * cr;
    robOrient.q4 = sy * cp * cr - cy * sp * sr;
}

/*
 * kinOrigin : always bottom-left
 * kinPoint follows Kinect axis notation
 * robPoint follows robot axis notation
 * Kinect(x,y,z)  -> Robot(y,z,x);
*/
inline void specImageProcessor::robToKinUnits(Point2f robPoint,Point2f &kinPoint, originType outOriginType)
{
    //Observed change in y offset value with change in Z.
    // did 3 measurements and found that y offset value changes aproximately 11 mm with a changed of 430mm depth.

    //change the axis notation from Kinect to Robot
    if (outOriginType == originType::BOTTOM_RIGHT) //rob origin bottom-right - The first scan design
    {
        kinPoint.x = robPoint.x + calibData.kinCordOffset.x ;
        kinPoint.y = robPoint.y + calibData.kinCordOffset.y;
    }

    else if (outOriginType == originType::BOTTOM_LEFT) //rob origin bottom-left
    { //TODO

    }
    else if (outOriginType == originType::TOP_LEFT || outOriginType == originType::TOP_RIGHT)
    {// top-Left can never be unless robot is upsidedown
        qWarning("Rob Origin at top is not supported. Is this what you really want ??");
    }
}

/*changes origin of the input point from top-left to required one.
 * inOrigin : always top-left
*/
inline void specImageProcessor::convToStageUnits(Point2f inPoint,Point2f &outPoint, short outOriginType, bool getStageOffset)
{
    if(getStageOffset == true)
    {
        #ifdef ACTUALSYSTEM
        float tempfloat;
        tempfloat      = (float)stageCont->getPosX()/1000;
        tempfloat      = round(tempfloat*100)/100;
        stageOffset->x  = tempfloat;

        tempfloat      = (float)stageCont->getPosZ()/1000;
        tempfloat      = round(tempfloat*100)/100;
        stageOffset->y  = tempfloat;
        #endif
    }

    outPoint = inPoint;

    outPoint.x = outPoint.x - laserOffset->x;
    outPoint.y = outPoint.y - laserOffset->y;

    //change from pixel to mm
    outPoint.x = outPoint.x / pxPermm->x ;
    outPoint.y = outPoint.y / pxPermm->y ;

    // change origin and thus the direction of move vector
    if (outOriginType == 0)
    {

    }
    else if (outOriginType == 1) //bottom-left
    { //Just flip Y
        outPoint.y  *=-1;
    }
    else if (outOriginType == 2) //top-right
    { //Just flip X
        outPoint.x *=-1;
    }
    else if (outOriginType == 3) //bottom-right
    {  // Flip both X and Y.
        outPoint.y *=-1;
        outPoint.x *=-1;
    }

    //add stage offset
    outPoint.x = outPoint.x + stageOffset->x;
    outPoint.y = outPoint.y + stageOffset->y;

    // Stick all points to the GRID
    //To round or not to round?!?!?!
    //outPoint.x = ROUND(outPoint.x);
    //outPoint.y = ROUND(outPoint.y);

    outPoint.x = stickToGrid(outPoint.x,0.5);
    outPoint.y = stickToGrid(outPoint.y,0.5);
}

inline float specImageProcessor::stickToGrid(float input, float gridSpacingmm)
{
    float output;
    int tempInt = 0;
    float tempFloat = 0;
    float diffToGrid = 0;

    tempInt     = (int) input;
    tempFloat   = (float)input - (float)tempInt;

    diffToGrid  = gridSpacingmm - tempFloat;

    if (diffToGrid < 0.0)
        diffToGrid  = 2*gridSpacingmm - tempFloat;

    output  = input + diffToGrid;


    return output;
}

//! Check if the two numbers are equal (almost)
/*!
* The expression for determining if two real numbers are equal is:
* if (Abs(x - y) <= EPSILON * Max(1.0f, Abs(x), Abs(y))).
*
* @param number1 First number
* @param number2 Second number
*/
#define EPSILON 1E-5
inline bool specImageProcessor::almostEqual(float number1, float number2)
{
    return (std::abs(number1 - number2) <= (EPSILON * maximum(1.0, std::abs(number1), std::abs(number2))));
}

//! Check if the first number is greater than or equal to the second number
/*!
* @param number1 First number
* @param number2 Second number
*/
inline bool specImageProcessor::greaterOrEqual(float number1, float number2)
{
    return ((number1 > number2) || (almostEqual(number1, number2)));
}

//! Check if the first number is less than or equal to the second number
/*!
* @param number1 First number
* @param number2 Second number
*/
inline bool specImageProcessor::lessOrEqual(float number1, float number2)
{
    return ((number1 < number2) || (almostEqual(number1, number2)));
}

//! Return the maximum of the provided numbers
inline float specImageProcessor::maximum(float number1, float number2, float number3)
{
    return std::max(std::max(number1, number2), number3);
}

bool specImageProcessor::IsPointInBoundingBox(float x1, float y1, float x2, float y2, float px, float py)
{
    float left, top, right, bottom;
    // For Bounding Box
    if(x1 < x2)
    {
        left = x1;
        right = x2;
    }
    else
    {
        left = x2;
        right = x1;
    }
    if(y1 < y2)
    {
        top = y1;
        bottom = y2;
    }
    else
    {
        top = y2;
        bottom = y1;
    }
                    //if( (px+0.01) >= left && (px-0.01) <= right &&
                      //      (py+0.01) >= top && (py-0.01) <= bottom )

    if(greaterOrEqual(px,left) && lessOrEqual(px,right) &&
       greaterOrEqual(py,top) && lessOrEqual(py,bottom))
        return true;
    else
        return false;
}
//! Determine the intersection point of two lines, if this point exists
/*! Two lines intersect if they are not parallel (Parallel lines intersect at
* +/- infinity, but we do not consider this case here).
*
* The lines are specified by a pair of points each. If they intersect, then
* the function returns true, else it returns false.
*
* Lines can be specified in the following form:
*      A1x + B1x = C1
*      A2x + B2x = C2
*
* If det (= A1*B2 - A2*B1) == 0, then lines are parallel
*                                else they intersect
*
* If they intersect, then let us denote the intersection point with P(x, y) where:
*      x = (C1*B2 - C2*B1) / (det)
*      y = (C2*A1 - C1*A2) / (det)
*
* @param a1 First point for determining the first line
* @param b1 Second point for determining the first line
* @param a2 First point for determining the second line
* @param b2 Second point for determining the second line
* @param intersection The intersection point, if this point exists
*/
inline bool specImageProcessor::lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1,
                                                 const cv::Point2f &a2,const cv::Point2f &b2,
                                                 cv::Point2f &intersection)
{
    bool retVal = false;
    float A1 = b1.y - a1.y;
    float B1 = a1.x - b1.x;
    float C1 = (a1.x * A1) + (a1.y * B1);

    float A2 = b2.y - a2.y;
    float B2 = a2.x - b2.x;
    float C2 = (a2.x * A2) + (a2.y * B2);

    float det = (A1 * B2) - (A2 * B1);

    if (!almostEqual(det, 0))
    {
        intersection.x = static_cast<float>(((C1 * B2) - (C2 * B1)) / (det));
        intersection.y = static_cast<float>(((C2 * A1) - (C1 * A2)) / (det));

        if(IsPointInBoundingBox(a1.x,a1.y,b1.x,b1.y,intersection.x,intersection.y) &&
           IsPointInBoundingBox(a2.x,a2.y,b2.x,b2.y,intersection.x,intersection.y))
        {
            retVal = true;
#ifdef ENABLE_SPECPROC_DETAIL_LOGS
            qDebug("specImageProcessor::lineIntersection - Line1-> Pt1:(%.2f,%.2f) , Pt2:(%.2f,%.2f) - Line2-> Pt1:(%2f,%2f) , Pt2:(%2f,%2f) - isIntersecting: %d , PtInter:(%2f,%2f)  " ,
                   a1.x,a1.y,b1.x,b1.y,a2.x,a2.y,b2.x,b2.y,retVal,intersection.x,intersection.y);
#endif
        }
    }
    return retVal;
}

/*
//Store laser center
void specImageProcessor::measPxPermm()
{
    int currPosX = 0;
    int currPosY = 0;

    currPosX = stage->getPosX();
    currPosY = stage->getPosZ();

    get the
    //move the stage 10
}
*/

//these value will be set according to robot Home position and the camera robot relative positions.
//KINECT on the left of Robot
//#define LASERAPRXPOSATHOME_X    700
//#define LASERAPRXPOSATHOME_Y    180
//#define LASERSEARCHRANGE        100

/*
//KINECT on the right of Robot
#define LASERAPRXPOSATHOME_X    230
#define LASERAPRXPOSATHOME_Y    170
#define LASERSEARCHRANGE        200
#define LDVLENGTH               308
*/

//KINECT in front of robot (center)
//calibration @home position (480,25)
//calibration @(400,400) position (560,280)
#define LASERAPRXPOSATHOME_X    790
#define LASERAPRXPOSATHOME_Y    190
#define LASERSEARCHRANGE        25
#define LDVLENGTH               20 // the marker is placed at the front of LDV


void specImageProcessor::depthCalibrate()
{
    ASScanUpdateTimer->stop();
    Point laserPointPx,retLaserPointPx,retLdvMarkPx,ldvMarkPx;
    int colorDataIndex,numOfMeas = 10;
    structRobTarget robPos;
    int i = 0;
    float ldvBackDepthSensor,ldvFrontDepth;

   // KINEC-Robot, depth offset calc
     //   1-Move robot arm to position for distance calculation.
        robPos = robotCont->homePos;

        if (robPos.trans.x == -1)
        {
            //homePos not valid home to robot
            robotCont->home();
        }

        //KINECT on left side of the robot - reduced depth of specimen
        robPos.trans.x = 820;
        robPos.trans.y = 460;
        robPos.trans.z = 560;

//        //KINECT on left side of the robot
//        robPos.trans.x = 810;
//        robPos.trans.y = 500;
//        robPos.trans.z = 580;

        //KINECT on left side of the robot on 3 brackets
//        robPos.trans.x = 870;
//        robPos.trans.y = 420;
//        robPos.trans.z = 660;


        /*
        //KINECT on the right side of the robot
        robPos.trans.x = 950;
        robPos.trans.y = 40;
        robPos.trans.z = 320;
        */

        /*
        //KINECT in front center of Robot
        robPos.trans.x = 950;
        robPos.trans.y = 160;
        robPos.trans.z = 420;
        */

        robotCont->moveToTarget(robPos,false,true);
        QThread::msleep(500);
        robotCont->moveToTarget(robPos,false,true); //to move the robot to more accurate position
        QThread::msleep(5000);


        //read the current value since the robot may not reach the exact value with accuracy.
        robPos = robotCont->getPos();

        //2- detect the marker at back of LDV
        while (i++ < numOfMeas)
        {
            //edgeDetection();
            if ((retLdvMarkPx = yellowCircleTracker()).x == -1)  // search box center and range are in pixels
            {
                qDebug("specImageProcessor::yellowCircleTracker - trial counter: %d",i);
                i--;
            }
            else
            {
                qDebug("specImageProcessor::yellowCircleTracker - retLdvMarkPx: (%d,%d)",retLdvMarkPx.x,retLdvMarkPx.y);
                ldvMarkPx += retLdvMarkPx;
            }
            QCoreApplication::processEvents();
            QThread::msleep(100);
        }
        //3- Find the depth of marker in KINECT POV
        ldvMarkPx           = ldvMarkPx/numOfMeas; // the current position of laser in the color image in pixels
        colorDataIndex      = ptToDataIdxColor(ldvMarkPx.x,ldvMarkPx.y);
        ldvBackDepthSensor  = pColor2xyz[colorDataIndex].Z *1000; //everythings in mm

        while(ldvBackDepthSensor == -std::numeric_limits<float>::infinity())
        {
            ldvMarkPx.x = ldvMarkPx.x-1;
            qDebug(" ldvBackDepthSensor Infinity, check adjacent pixels: new ldvMarkPx(%d,%d))",ldvMarkPx.x,ldvMarkPx.y);
            //check adjacent pixels for correct values
        }

        ldvFrontDepth       = ldvBackDepthSensor + LDVLENGTH;

        //4-Find offset of robot depth from Kinect depth.
        calibData.kinCordOffset.z = ldvFrontDepth - robPos.trans.x;

        qDebug("specImageProcessor::depthCalibrate - ldvMarkPx:(%d,%d) - ldvBackDepthSensor: %f - ldvFrontDepth: %f - calibData.kinCordOffset.z: %f",
               ldvMarkPx.x,ldvMarkPx.y,ldvBackDepthSensor,ldvFrontDepth,calibData.kinCordOffset.z);

    ASScanUpdateTimer->start();
}

void specImageProcessor::xyCalibrate()
{
    structRobTarget robPos;
    int i = 0;
    int colorDataIndex,numOfMeas = 10;
    Point laserPointPx,retLaserPointPx,retLdvMarkPx,ldvMarkPx;

    ASScanUpdateTimer->stop();
//KINECT-robot coordinate sync
    //1- Calibration @ home
//        robotCont->home();
//        QThread::msleep(200);
//        robotCont->home();  //just to be more accurate
//        QThread::msleep(200);

//        //2-store the robotvalue at which calib is performed.
//        calibData.robPt.x = robotCont->homePos.trans.x;
//        calibData.robPt.y = robotCont->homePos.trans.y;
//        calibData.robPt.z = robotCont->homePos.trans.z;

        // //- Calibration @ custom point
        // //middle calibration for y offset dependence on depth
        robPos.trans.x = 630;
        robPos.trans.y = 380;
        robPos.trans.z = 480;

        robotCont->moveToTarget(robPos,false,true);
        QThread::msleep(500);
        robotCont->moveToTarget(robPos,false,true); //to move the robot to more accurate position
        QThread::msleep(5000);

        calibData.robPt.x = robPos.trans.x;
        calibData.robPt.y = robPos.trans.y;
        calibData.robPt.z = robPos.trans.z;


        i=0;
        //sync the xy values
        while (i++ < numOfMeas)
        {
            //edgeDetection();
            if ((retLaserPointPx = laserTrackerRobotArm(Point{LASERAPRXPOSATHOME_X,LASERAPRXPOSATHOME_Y},LASERSEARCHRANGE)).x == -1)  // search box center and range are in pixels
            {
                qDebug("specImageProcessor::measLaserCenter() - trial counter: %d",i);
                i--;
            }
            else
            {
                qDebug("specImageProcessor::measLaserCenter() - retLaserPoint: (%d,%d)",retLaserPointPx.x,retLaserPointPx.y);
                laserPointPx += retLaserPointPx ;
            }
            QCoreApplication::processEvents();
            QThread::msleep(100);
        }
        laserPointPx = laserPointPx/numOfMeas; // the current position of laser in the color image in pixels

        //for the laser point pixels read the XY values from the KINECT camera map
        colorDataIndex = ptToDataIdxColor(laserPointPx.x,laserPointPx.y);

        calibData.kinPt.x = pColor2xyz[colorDataIndex].X*1000;
        calibData.kinPt.y = pColor2xyz[colorDataIndex].Y*1000;
        calibData.kinPt.z = pColor2xyz[colorDataIndex].Z*1000;

        //caluculate the calib data sensor Pt if the robot was at (0,0,0).
        //actual movement of robot is not possible in current configuratin due to probable tool collision
        calibData.kinPtAtRobZero.x = calibData.kinPt.x - calibData.robPt.y;
        calibData.kinPtAtRobZero.y = calibData.kinPt.y - calibData.robPt.z;
        calibData.kinPtAtRobZero.z = calibData.kinPt.z; //will not use


        calibData.kinCordOffset.x = calibData.kinPtAtRobZero.x;
        calibData.kinCordOffset.y = calibData.kinPtAtRobZero.y;
        //calibData.kinCordOffset.z = calibData.kinPtAtRobZero.x;

        qDebug("specImageProcessor::xyCalibrate-laserPointPx:(%d,%d) - calibData.kinCordOffset.x: %f - calibData.kinCordOffset.y: %f - calibData.kinCordOffset.z: %f - calibData.kinPt.z: %f",
               laserPointPx.x,laserPointPx.y,calibData.kinCordOffset.x,calibData.kinCordOffset.y,calibData.kinCordOffset.z,calibData.kinPt.z);

        ASScanUpdateTimer->start();
}

inline int specImageProcessor::ptToDataIdxColor(int xColor,int yColor)
{
    int colorDataIndex;
    if (flipImage)
        xColor = (colorWidth - 1) - xColor;

    colorDataIndex = xColor+(yColor*colorWidth);
    return colorDataIndex;
}

inline int specImageProcessor::ptToDataIdxDepth(int xDepth,int yDepth)
{
    int depthDataIndex;
    if (flipImage)
        xDepth = (depthWidth - 1) - xDepth;

    depthDataIndex = xDepth+(yDepth*depthWidth);
    return depthDataIndex;
}
Point specImageProcessor::laserTrackerRobotArm(Point searchCenter, int searchRange)
{
        Point circleCentre;
        Mat startImg;
        bool success = false;
        int trials =0;

        if (useKinect)
        {
            while(success == false && trials<10)
            {
                success =   getImageFromKinect(&startImg,COL_DEP_IMAGE,0); // colour image
                trials++;
                QThread::msleep(500);
            }

            if (success == false)
            {
                qDebug("specImageProcessor::laserTracker - couldnt read the image");
                circleCentre.x = 0;
                circleCentre.y = 0;
                return circleCentre;
            }
        }
        else
        {
            //captured the zoomed in portion of the image
            if (camHndl.isOpened() == false)
                startImg = imread(ImgPath.toStdString());
            else
                camHndl.read(startImg);
        }

        //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        //Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);
        Rect roi(searchCenter - Point{searchRange/2,searchRange/2}, searchCenter + Point{searchRange/2,searchRange/2} );

        Mat gray_image = startImg(roi).clone();
        Mat orig_image = startImg(roi).clone();

        cvtColor(orig_image,gray_image,CV_BGRA2GRAY);

        //threshold(gray_image,binary_image,10,255,cv::THRESH_BINARY);
        //blur(gray_image, gray_image , Size(9, 9));

        // Use the Hough transform to detect circles in the combined threshold image
        vector<Vec3f> circles;
        HoughCircles(gray_image, circles, CV_HOUGH_GRADIENT, 1, gray_image.rows/8, 30, 10, 0, 0);

        if (circles.size() > 0)
        {
            circleCentre.x = round(circles[0][0]) + searchCenter.x - searchRange/2;
            circleCentre.y = round(circles[0][1]) + searchCenter.y - searchRange/2;
        }
        else
        {
            circleCentre.x = -1;
            circleCentre.y = -1;
        }

        // Loop over all detected circles and outline them on the original image
        //if(circles.size() == 0) return;
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
            center += searchCenter - Point{searchRange/2,searchRange/2};

            int radius = round(circles[current_circle][2]);
            circle(startImg, center, radius, Scalar(255*current_circle, 255,0,255), 2);
        }

        /*

        // Show images
        cv::namedWindow("gray_image", cv::WINDOW_AUTOSIZE);
        cv::imshow("gray_image", gray_image);
        //cv::namedWindow("binary_image", cv::WINDOW_AUTOSIZE);
        //cv::imshow("binary_image", binary_image);
        cv::namedWindow("Detected laser on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected laser on the input image", ctrImg);\
        cv::waitKey(0);
*/
        labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(startImg)));
        labelOrigImage->adjustSize();
        labelOrigImage->parentWidget()->adjustSize();
        labelOrigImage->updateGeometry();
        labelOrigImage->parentWidget()->updateGeometry();

        return circleCentre;
}

Point specImageProcessor::yellowCircleTracker()
{
        Point circleCentre;
        Mat startImg;
        bool success = false;
        int trials =0;

        if (useKinect)
        {
            while(success == false && trials<10)
            {
                success = getImageFromKinect(&startImg,COL_DEP_IMAGE,0); // colour image
                trials++;
                QThread::msleep(500);
            }

            if (success == false)
            {
                qDebug("specImageProcessor::laserTracker - couldnt read the image");
                circleCentre.x = 0;
                circleCentre.y = 0;
                return circleCentre;
            }
        }
        else
        {
            //captured the zoomed in portion of the image
            if (camHndl.isOpened() == false)
                startImg = imread(ImgPath.toStdString());
            else
                camHndl.read(startImg);
        }

        //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        //Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);

        Mat bgra_image = startImg;
        Mat bgr_image;
        Mat orig_image = bgra_image;
    // Convert input image to HSV
        Mat hsv_image;
        cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
        cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        // Threshold the HSV image, keep only the green pixels
        Mat yel_hue_image;

        inRange(hsv_image, cv::Scalar(15, 60, 100), cv::Scalar(35, 255, 255), yel_hue_image);

        cv::GaussianBlur(yel_hue_image, yel_hue_image, cv::Size(3, 3), 2, 2);
        //blur(grn_hue_image, grn_hue_image, Size(3, 3));

        // Use the Hough transform to detect circles in the combined threshold image
        vector<Vec3f> circles;
        HoughCircles(yel_hue_image, circles, CV_HOUGH_GRADIENT, 1, yel_hue_image.rows/8, 20, 10, 0, 0);

        // Loop over all detected circles and outline them on the original image
        //if(circles.size() == 0) return;
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
            int radius = round(circles[current_circle][2]);
            circle(orig_image, center, radius, Scalar(255*current_circle, 255,0,255), 2);
        }

        if (circles.size() > 0)
        {
            circleCentre.x = round(circles[0][0]);
            circleCentre.y = round(circles[0][1]);
        }
        else
        {
            circleCentre.x = -1;
            circleCentre.y = -1;
        }

        // Show images
/*
        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", yel_hue_image);
        cv::namedWindow("Detected yel circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected yel circles on the input image", orig_image);

        cv::waitKey(0);
*/

        labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(orig_image)));
        labelOrigImage->adjustSize();
        labelOrigImage->parentWidget()->adjustSize();
        labelOrigImage->updateGeometry();
        labelOrigImage->parentWidget()->updateGeometry();

        return circleCentre;
}
/*
Point specImageProcessor::greenLaserTracker()
{
        Point circleCentre;
        Mat startImg;
        bool success = false;
        int trials =0;

        int colorInHSpace = 60 ; //since opencv halves the colour values of H space
        int sensitivity = 20 ; //since opencv halves the colour values of H space

        if (useKinect)
        {
            while(success == false && trials<10)
            {
                success = getImageFromKinect(&startImg,COL_DEP_IMAGE,0); // colour image
                trials++;
                QThread::msleep(500);
            }

            if (success == false)
            {
                qDebug("specImageProcessor::laserTracker - couldnt read the image");
                circleCentre.x = 0;
                circleCentre.y = 0;
                return circleCentre;
            }
        }
        else
        {
            //captured the zoomed in portion of the image
            if (camHndl.isOpened() == false)
                startImg = imread(ImgPath.toStdString());
            else
                camHndl.read(startImg);
        }

        //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        //Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);

        Mat bgra_image = startImg;
        Mat bgr_image;
        Mat orig_image = bgra_image;
    // Convert input image to HSV
        Mat hsv_image;
        cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
        cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        // Threshold the HSV image, keep only the green pixels
        Mat yel_hue_image;

        inRange(hsv_image, cv::Scalar(colorInHSpace - sensitivity, 60, 100), cv::Scalar(colorInHSpace + sensitivity, 255, 255), yel_hue_image);

        cv::GaussianBlur(yel_hue_image, yel_hue_image, cv::Size(3, 3), 2, 2);
        //blur(grn_hue_image, grn_hue_image, Size(3, 3));

        // Use the Hough transform to detect circles in the combined threshold image
        vector<Vec3f> circles;
        HoughCircles(yel_hue_image, circles, CV_HOUGH_GRADIENT, 1, yel_hue_image.rows/8, 20, 10, 0, 0);

        // Loop over all detected circles and outline them on the original image
        //if(circles.size() == 0) return;
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
            int radius = round(circles[current_circle][2]);
            circle(orig_image, center, radius, Scalar(255*current_circle, 255,0,255), 2);
        }

        if (circles.size() > 0)
        {
            circleCentre.x = round(circles[0][0]);
            circleCentre.y = round(circles[0][1]);
        }
        else
        {
            circleCentre.x = -1;
            circleCentre.y = -1;
        }

        // Show images

        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", yel_hue_image);
        cv::namedWindow("Detected grn circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected grn circles on the input image", orig_image);

//        cv::waitKey(0);

//        labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(orig_image)));
//        labelOrigImage->adjustSize();
//        labelOrigImage->parentWidget()->adjustSize();
//        labelOrigImage->updateGeometry();
//        labelOrigImage->parentWidget()->updateGeometry();

        return circleCentre;
}

*/
Point specImageProcessor::greenLaserTrackerWithBgSubtraction(bool storeBgImage, int bgImgIndex)
{
        Point circleCentre;
        Mat startImg,camImg;
        bool success = false;
        int trials =0;

        int colorInHSpace = 60 ; //since opencv halves the colour values of H space
        int sensitivity = 20 ; //since opencv halves the colour values of H space

        if (useKinect || useAzure)
        {
            while(success == false && trials<10)
            {

                success = getImageFromKinect(&camImg,COL_IMAGE,0); // colour image
                //captured the zoomed in portion of the image
                if (success)
                {
                    Rect roi(0+zoomFactor*80, 0+zoomFactor*40, camImg.cols-zoomFactor*160, camImg.rows-zoomFactor*80);

                    if (storeBgImage == true)
                        bgImg[bgImgIndex]  = camImg(roi).clone();
                    else
                        startImg           = camImg(roi).clone();
                }

                trials++;
                QThread::msleep(500);
            }

            if (success == false)
            {
                qDebug("specImageProcessor::greenLaserTrackerWithBgSubtraction - couldnt read the image");
                circleCentre.x = -1;
                circleCentre.y = -1;
                return circleCentre;
            }

            if (storeBgImage == true)
            {
                qDebug("specImageProcessor::greenLaserTrackerWithBgSubtraction - Stored the BG Image");
                circleCentre.x = -1;
                circleCentre.y = -1;
                return circleCentre;
            }
        }
        else
        {
            //captured the zoomed in portion of the image
            if (camHndl.isOpened() == false)
                startImg = imread(ImgPath.toStdString());
            else
                camHndl.read(startImg);
        }

        //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        //Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);

        Mat orig_image = startImg;
        Mat bgSubImg = startImg - bgImg[bgImgIndex];
        Mat bgSubImgGrey;
        Mat bgSubImgBinary;


    // Convert input image to HSV
//        Mat bgra_image = startImg;
//        Mat bgr_image;
//        Mat orig_image = bgra_image;
//        Mat hsv_image;
//        cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
//        cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

//        // Threshold the HSV image, keep only the green pixels
//        Mat yel_hue_image;

//        inRange(hsv_image, cv::Scalar(colorInHSpace - sensitivity, 60, 100), cv::Scalar(colorInHSpace + sensitivity, 255, 255), yel_hue_image);

        cvtColor(bgSubImg,bgSubImgGrey,CV_BGRA2GRAY);
        threshold(bgSubImgGrey,bgSubImgBinary,100,255,cv::THRESH_BINARY_INV);
        //cv::GaussianBlur(bgSubImgGrey, bgSubImgGrey, cv::Size(3, 3), 2, 2);
        //blur(grn_hue_image, grn_hue_image, Size(3, 3));

//        cv::namedWindow("bgImg", cv::WINDOW_AUTOSIZE);
//        cv::imshow("bgImg", bgImg);

//        cv::namedWindow("bgSubImgGrey", cv::WINDOW_AUTOSIZE);
//        cv::imshow("bgSubImgGrey", bgSubImgGrey);

//        cv::namedWindow("bgSubImgBinary", cv::WINDOW_AUTOSIZE);
//        cv::imshow("bgSubImgBinary", bgSubImgBinary);

        // Use the Hough transform to detect circles in the combined threshold image
        vector<Vec3f> circles;
        HoughCircles(bgSubImgBinary, circles, CV_HOUGH_GRADIENT, 1, bgSubImgBinary.rows/8, 10, 5, 0, 20);

        // Loop over all detected circles and outline them on the original image
        //if(circles.size() == 0) return;
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
            int radius = round(circles[current_circle][2]);
            circle(orig_image, center, radius, Scalar(255*current_circle, 255,0,0), 2);
        }

        if (circles.size() > 0)
        {
            circleCentre.x = round(circles[0][0]);
            circleCentre.y = round(circles[0][1]);
        }
        else
        {
            circleCentre.x = -1;
            circleCentre.y = -1;
        }

        // Show images


        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", bgSubImgBinary);
        cv::namedWindow("Detected grn circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected grn circles on the input image", orig_image);


       // cv::waitKey(0);

//        labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(orig_image)));
//        labelOrigImage->adjustSize();
//        labelOrigImage->parentWidget()->adjustSize();
//        labelOrigImage->updateGeometry();
//        labelOrigImage->parentWidget()->updateGeometry();

        QCoreApplication::processEvents();
        return circleCentre;
}

Point specImageProcessor::redLaserTracker()
{
        Point circleCentre;
        Mat startImg;
        bool success = false;
        int trials =0;

        bool storeBgImage = false;

        int colorInHSpace = 60 ; //since opencv halves the colour values of H space
        int sensitivity = 20 ; //since opencv halves the colour values of H space

        if (useKinect || useAzure)
        {
            while(success == false && trials<10)
            {
                if (storeBgImage == true)
                    success = getImageFromKinect(&bgImg[0],COL_IMAGE,0); // colour image
                else
                    success = getImageFromKinect(&startImg,COL_IMAGE,0); // colour image
                trials++;
                QThread::msleep(500);
            }

            if (success == false)
            {
                qDebug("specImageProcessor::redLaserTracker - couldnt read the image");
                circleCentre.x = -1;
                circleCentre.y = -1;
                return circleCentre;
            }

            if (storeBgImage == true)
            {
                qDebug("specImageProcessor::redLaserTracker - Stored the BG Image");
                circleCentre.x = -1;
                circleCentre.y = -1;
                return circleCentre;
            }
        }
        else
        {
            //captured the zoomed in portion of the image
            if (camHndl.isOpened() == false)
                startImg = imread(ImgPath.toStdString());
            else
                camHndl.read(startImg);
        }

        //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        //Rect roi(0+zoomFactor*80, 0+zoomFactor*40, startImg.cols-zoomFactor*160, startImg.rows-zoomFactor*80);

        Mat bgSubImgBinary;


    // Convert input image to HSV
        Mat bgra_image = startImg;
        Mat bgr_image;
        Mat orig_image = bgra_image;
        Mat hsv_image;
        cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
        cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

//        // Threshold the HSV image, keep only the green pixels
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        //inRange(hsv_image, cv::Scalar(colorInHSpace - sensitivity, 60, 100), cv::Scalar(colorInHSpace + sensitivity, 255, 255), yel_hue_image);

        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        //red_hue_image = upper_red_hue_range;

        //blur(grn_hue_image, grn_hue_image, Size(3, 3));
        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);


        // Use the Hough transform to detect circles in the combined threshold image
        vector<Vec3f> circles;
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 10, 5, 0, 20);

        // Loop over all detected circles and outline them on the original image
        //if(circles.size() == 0) return;
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
            int radius = round(circles[current_circle][2]);
            circle(orig_image, center, radius, Scalar(255*current_circle, 255,0,0), 2);
        }

        if (circles.size() > 0)
        {
            circleCentre.x = round(circles[0][0]);
            circleCentre.y = round(circles[0][1]);
        }
        else
        {
            circleCentre.x = -1;
            circleCentre.y = -1;
        }

        // Show images


        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", red_hue_image);
        cv::namedWindow("Detected grn circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected grn circles on the input image", orig_image);


//        cv::waitKey(0);

//        labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(orig_image)));
//        labelOrigImage->adjustSize();
//        labelOrigImage->parentWidget()->adjustSize();
//        labelOrigImage->updateGeometry();
//        labelOrigImage->parentWidget()->updateGeometry();

        QCoreApplication::processEvents();
        return circleCentre;
}

void specImageProcessor::save()
{
    //Save the polygon pic
    bool result;
    QDateTime now = QDateTime::currentDateTime();
    QString dataPath = "D:\\"+now.toString("ddMMyy_hhmmss");

    QLabel *labelPtr = (QLabel *)this->widgetInteractive->childAt(20,20);
    const QPixmap *pixmapPtr = labelPtr->pixmap();
    result = pixmapPtr->save(dataPath+"MarkedArea.BMP",0,100);

    //save the actual Polygon
}

//checks if the the robot target is within the range of the robot
inline bool specImageProcessor::isRobPtWihinRange(structPos robTarget)
{
    bool retVal = true;

    structRobTarget home = robotCont->homePos;

    Point3f minRange;
    Point3f maxRange;

    minRange.x = home.trans.x-ROBOTRANGEXMIN;
    minRange.y = home.trans.y-ROBOTRANGEY;
    minRange.z = home.trans.z-ROBOTRANGEZ;

    maxRange.x =  home.trans.x+ROBOTRANGEXMAX;
    maxRange.y =  home.trans.y;
    maxRange.z =  home.trans.z;

    if (robTarget.x < minRange.x || robTarget.x > maxRange.x && retVal )
    {
        retVal = false;
        qWarning("specImageProcessor::isRobPtWihinRange - X val out of range, robTarget.x: %f, Range: %f ~ %f",
                 robTarget.x,minRange.x,maxRange.x);
    }


    if(robTarget.y< minRange.y || robTarget.y>maxRange.y && retVal)
    {
        retVal = false;
        qWarning("specImageProcessor::isRobPtWihinRange - Y val out of range, robTarget.y: %f, Range: %f ~ %f",
                 robTarget.y,minRange.y,maxRange.y);
    }

    if(robTarget.z < minRange.z || robTarget.z>maxRange.z && retVal)
    {
        retVal = false;
        qWarning("specImageProcessor::isRobPtWihinRange - Z val out of range, robTarget.z: %f, Range: %f ~ %f",
                 robTarget.z,minRange.z,maxRange.z);
    }

    return retVal;
}

void specImageProcessor::convHomeToKinCol()
{
    Point2f kinCsXy;
    Point2f robHome (robotCont->homePos.trans.y,robotCont->homePos.trans.z) ;

    robToKinUnits(robHome,kinCsXy);

    //search the colour image to find the closest point in X.
    robHomeclPx = Point2i(0,0);
    Point2i diff;
    _CameraSpacePoint searchPtTemp;

    while(robHomeclPx.x < colorWidth)
    {
        searchPtTemp = mTomm(pColor2xyz[ptToDataIdxColor(robHomeclPx.x,robHomeclPx.y)]);

        if (searchPtTemp.X != -std::numeric_limits<float>::infinity())
        {
            diff.x = fabs(kinCsXy.x - searchPtTemp.X);
            if (diff.x <5.0)
                break;
        }

        robHomeclPx.x++;
    }

    while(robHomeclPx.y < colorHeight)
    {
        searchPtTemp = mTomm(pColor2xyz[ptToDataIdxColor(robHomeclPx.x,robHomeclPx.y)]);

        if (searchPtTemp.Y != -std::numeric_limits<float>::infinity())
        {
            diff.y = fabs(kinCsXy.y - searchPtTemp.Y);

            if (diff.y <5.0)
                break;
        }

        robHomeclPx.y++;
    }

    kinCsXy.x = kinCsXy.x - ROBOTRANGEY;
    kinCsXy.y = kinCsXy.y - ROBOTRANGEZ;

    //search the colour image to find the closest point in X.
    robBotRightclPx = Point2i(0,0);
    diff = Point2i(0,0);

    while(robBotRightclPx.x < colorWidth)
    {
        searchPtTemp = mTomm(pColor2xyz[ptToDataIdxColor(robBotRightclPx.x,robBotRightclPx.y)]);

        if (searchPtTemp.X != -std::numeric_limits<float>::infinity())
        {
            diff.x = fabs(kinCsXy.x - searchPtTemp.X);
            if (diff.x <20.0)
                break;
        }

        robBotRightclPx.x++;
    }

    while(robBotRightclPx.y < colorHeight)
    {
        searchPtTemp = mTomm(pColor2xyz[ptToDataIdxColor(robBotRightclPx.x,robBotRightclPx.y)]);

        if (searchPtTemp.Y != -std::numeric_limits<float>::infinity())
        {
            diff.y = fabs(kinCsXy.y - searchPtTemp.Y);

            if (diff.y <20.0)
                break;
        }

        robBotRightclPx.y++;
    }

    qDebug(" specImageProcessor::convHomeToKinCol - Rob Home Position in Kinect Colour Image - robHomeclPx(x,y):(%d,%d), robBotRightclPx(x,y):(%d,%d)",
           robHomeclPx.x,robHomeclPx.y,robBotRightclPx.x,robBotRightclPx.y);
}

void specImageProcessor::save(QString dataPath)
{
    //Save the polygon pic
    bool result;

    QLabel *labelPtr = (QLabel *)this->widgetInteractive->childAt(20,20);
    const QPixmap *pixmapPtr = labelPtr->pixmap();
    result = pixmapPtr->save(dataPath+"MarkedArea.BMP",0,100);

    //save the actual Polygon
}

void specImageProcessor::load(QString dataPath)
{
    //stop the timer
    enASScanUpdateTimer(false);

    //load the polygon pic
    bool result;
    QLabel *labelPtr = (QLabel *)this->widgetInteractive->childAt(20,20);
    QPixmap pixmap;
    result = pixmap.load(dataPath+"\\MarkedArea.BMP",0);
    labelPtr->setPixmap(pixmap);

    //load the actual polygon
}

void specImageProcessor::saveKinectData(QString dataPath)
{
    bool result;
    QFile myfileout;
    //save only incase of COL_DEP
    if (imageType != COL_DEP_IMAGE)
        return;
    //stop the timer
    enASScanUpdateTimer(false);

    if (dataPath == NULL)
    {
        QDateTime now = QDateTime::currentDateTime();
        dataPath = "D:\\RA_UPI\\KINECT\\"+now.toString("ddMMyy_hhmmss")+"\\";

        if( QDir(dataPath).exists() == false)
            QDir().mkpath(dataPath);

    }
    std::string dataPathStd = dataPath.toLocal8Bit().constData();

    //save the color stream
    Mat colStreamMat;
    getImageFromKinect(&colStreamMat,COL_DEP_IMAGE,0);
    FileStorage fsC(dataPathStd+"colStreamMat.yml", FileStorage::WRITE );
    fsC<<"colStreamMat"<<colStreamMat;

    //save the depth stream
    ////////////////////////////////////////////////////////////////
    myfileout.setFileName(dataPath+"depthFile.bin");
    if(!myfileout.open(QIODevice::WriteOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open file for writing.");
        errBox.exec();
        return;
    }

    result=myfileout.write((char *)pDepthBuffer,2*512*424);
    myfileout.close();
    ///////////////////////////////////////////////////////////////
    //save the colour to cameraspace mapping
    myfileout.setFileName(dataPath+"color2xyzFile.bin");
    if(!myfileout.open(QIODevice::WriteOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open file for writing.");
        errBox.exec();
        return;
    }

    result=myfileout.write((char *)pColor2xyz,3*4*1920*1080); //each element has 3 floats
    myfileout.close();
    ///////////////////////////////////////////////////////////////
    //save the depth to xyz mapping pDepth2xyz
    myfileout.setFileName(dataPath+"depth2xyzFile.bin");
    if(!myfileout.open(QIODevice::WriteOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open file for writing.");
        errBox.exec();
        return;
    }

    result=myfileout.write((char *)pDepth2xyz,3*4*512*424); //each element has 3 floats
    myfileout.close();
    ///////////////////////////////////////////////////////////////
    //save the colour to depth mapping
    myfileout.setFileName(dataPath+"color2depthFile.bin");
    if(!myfileout.open(QIODevice::WriteOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open file for writing.");
        errBox.exec();
        return;
    }

    result=myfileout.write((char *)pColor2depth,2*4*1920*1080); //each element has 2 floats
    myfileout.close();
    ///////////////////////////////////////////////////////////////

    //save the average stream if averaging is done
    FileStorage fs(dataPathStd+"depImgAvgMat.yml", FileStorage::WRITE );
    fs<<"depImgAvgMat"<<depImgAvgMat;

    save(dataPath);
}

void specImageProcessor::loadKinectData(QString dataPath)
{
    bool result;
    QFile myfilein;
    enASScanUpdateTimer(false);

    if (dataPath == NULL)
    {
        QFileDialog *fd = new QFileDialog;
        //QTreeView *tree = fd->findChild <QTreeView*>();
        //tree->setRootIsDecorated(true);
        //tree->setItemsExpandable(true);
        fd->setFileMode(QFileDialog::Directory);
        fd->setOption(QFileDialog::ShowDirsOnly);
        fd->setViewMode(QFileDialog::Detail);
        fd->setDirectory("D:\\RA_UPI\\KINECT\\");

        if (fd->exec())
        {
            dataPath = fd->selectedFiles()[0];
            qDebug()<<dataPath;
        }
    }

    std::string dataPathStd = dataPath.toLocal8Bit().constData();

    //load the color stream
    Mat colStreamMat;
    FileStorage fsC(dataPathStd+"//colStreamMat.yml", FileStorage::READ );
    fsC["colStreamMat"] >> colStreamMat;

    //draw contours and markers
    labelOrigImage->setPixmap(QPixmap::fromImage(openCVtoQimg(colStreamMat,true)));
    //labelOrigImage->setPixmap(QPixmap::fromImage((openCVtoQimg(startImg))));
    labelOrigImage->adjustSize();
    labelOrigImage->parentWidget()->adjustSize();
    labelOrigImage->updateGeometry();
    labelOrigImage->parentWidget()->updateGeometry();


    //load the depth stream
    ////////////////////////////////////////////////////////////////
    myfilein.setFileName(dataPath+"//depthFile.bin");
    if(!myfilein.open(QIODevice::ReadOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open depthFile.bin for reading.");
        errBox.exec();
        return;
    }

    pDepthBuffer = new unsigned short[512*424];
    result=myfilein.read((char *)pDepthBuffer,2*512*424);
    myfilein.close();
    ///////////////////////////////////////////////////////////////
    //load the color to xyz stream
    myfilein.setFileName(dataPath+"//color2xyzFile.bin");
    if(!myfilein.open(QIODevice::ReadOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open color2xyzFile.bin for reading.");
        errBox.exec();
        return;
    }

//    if (pColor2xyz!=NULL)
//       {
//           delete pColor2xyz;
//           pColor2xyz=NULL;
//       }
//    pColor2xyz =  new CameraSpacePoint[1920*1080];

    result=myfilein.read((char *)pColor2xyz,3*4*1920*1080);
    myfilein.close();
    ///////////////////////////////////////////////////////////////
    //load the depth to xyz stream
    myfilein.setFileName(dataPath+"//depth2xyzFile.bin");
    if(!myfilein.open(QIODevice::ReadOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open color2xyzFile.bin for reading.");
        errBox.exec();
        return;
    }

    if (pDepth2xyz!=NULL)
       {
           delete pDepth2xyz;
           pDepth2xyz=NULL;
       }
    pDepth2xyz =  new CameraSpacePoint[512*424];

    result=myfilein.read((char *)pDepth2xyz,3*4*512*424);
    myfilein.close();
    ///////////////////////////////////////////////////////////////
    //load the color to depth stream
    myfilein.setFileName(dataPath+"//color2depthFile.bin");
    if(!myfilein.open(QIODevice::ReadOnly))
    {
        QMessageBox errBox;
        errBox.setText("Could not open color2depthFile.bin for reading.");
        errBox.exec();
        return;
    }

    if (pColor2depth!=NULL)
       {
           delete pColor2depth;
           pColor2depth=NULL;
       }
    pColor2depth =  new DepthSpacePoint[1920*1080];

    result=myfilein.read((char *)pColor2depth,2*4*1920*1080);
    myfilein.close();

    ///////////////////////////////////////////////////////////////
    //load averaged depth data
    FileStorage fs(dataPathStd+"//depImgAvgMat.yml", FileStorage::READ );
    depImgAvgMat = Scalar::all(0);
    fs["depImgAvgMat"] >> depImgAvgMat;

    depImgAvgMatFliped = Scalar::all(0);

#ifdef CALCNORMALS
    if (flipImage)
        flip(depImgAvgMat, depImgAvgMatFliped, 1);
    else
        depImgAvgMatFliped = depImgAvgMat;
#endif

    //this->calcNormals();
    //waitKey(0);
    this->calcNormalsPcl();
}

void specImageProcessor::setLoadKinFusCloud(bool loadKinFusCloud_arg)
{
    loadKinFusCloud =  loadKinFusCloud_arg;
    calcNormalsPcl();
}

//The one and only custom click-grabbing-label.
HLabel::HLabel(QWidget *parent, Qt::WindowFlags f):QLabel(parent,f){
}
HLabel::HLabel(const QString &text, QWidget *parent, Qt::WindowFlags f):QLabel(text,parent,f){}
HLabel::~HLabel(){
}

void HLabel::mousePressEvent(QMouseEvent *ev)
{
    emit clicked(ev);
}

void HLabel::mouseMoveEvent(QMouseEvent *ev)
{
    if(ev->type() == QEvent::MouseMove)
        emit moved(ev);
}

void HLabel::wheelEvent(QWheelEvent *ev)
{
    emit wheelMove(ev);
}


/*
 *
 * BACKUP - 24OCT2019
// * kinOrigin : always bottom-right
// * kinPoint follows Kinect axis notation
// * robPoint follows robot axis notation
// * Kinect(x,y,z)  -> Robot(y,z,x);
// * robPointWAngle is the robot trans point considering angle
// * normalAngle is the angle of the normal to the surface range 0~180 degreee.

inline void specImageProcessor::kinToRobUnits(Point3f kinPoint, structRobTarget &robPoint, float yaw,float pitch, bool enLog, originType outOriginType)
{
    //Observed change in y offset value with change in Z.
    // did 3 measurements and found that y offset value changes aproximately 11 mm with a changed of 430mm depth.
    float alpha = stickToGrid((calibData.kinPt.z - kinPoint.z) * ((float)20/(float)320), 0.5);
    float beta = stickToGrid((calibData.kinPt.z - kinPoint.z) * ((float)7/(float)320), 0.5);

    alpha =0;
    beta = 0;
    //change the axis notation from Kinect to Robot
    if (outOriginType == originType::BOTTOM_RIGHT) //rob origin bottom-right - The first scan design
    {
        robPoint.trans.y = kinPoint.x - (calibData.kinCordOffset.x + beta) ;
        robPoint.trans.z = kinPoint.y - (calibData.kinCordOffset.y + alpha);
    }
    else if (outOriginType == originType::BOTTOM_LEFT) //rob origin bottom-left
    {
        robPoint.trans.y = calibData.kinCordOffset.x + kinPoint.x + beta;
        robPoint.trans.z = calibData.kinCordOffset.y - (kinPoint.y+alpha);
    }
    else if (outOriginType == originType::TOP_LEFT || outOriginType == originType::TOP_RIGHT)
    {// top-Left can never be unless robot is upsidedown
        qWarning("Rob Origin at top is not supported. Is this what you really want ??");
    }
    if(enLog)
    {
        qDebug("kinToRobUnits - kinPoint(x,y,z) : (%f,%f,%f) , robPoint(y,z,x): (%f,%f,%f), alpha: %f, beta:%f",
               kinPoint.x,kinPoint.y,kinPoint.z,robPoint.trans.y,robPoint.trans.z,robPoint.trans.x,alpha,beta);
    }

    if (scanInfoPtr->enableDepthControl)
    {
        //There is a aprox 10mm overshoot , reasons un-known.
        robPoint.trans.x = (kinPoint.z - (scanInfoPtr->ldvStandOffDistance)) -  calibData.kinCordOffset.z;
    }
    else
    {
        robPoint.trans.x = robotCont->homePos.trans.x;
    }

    robPoint.transAfterAngle = robPoint.trans;

    robPoint.rot.q1 = 0.707106781;
    robPoint.rot.q2 = 0;
    robPoint.rot.q3 = 0.707106781;
    robPoint.rot.q4 = 0;
    //orientation control enabled ??

    if(scanInfoPtr->enableOrientControl)
    {
//        robPoint.transAfterAngle.y = robPoint.trans.y + scanInfoPtr->ldvStandOffDistance * (qCos(qDegreesToRadians(90-yaw)));
//        robPoint.transAfterAngle.x = robPoint.trans.x + (scanInfoPtr->ldvStandOffDistance - (scanInfoPtr->ldvStandOffDistance * (qSin(qDegreesToRadians(90-yaw)))));

          robPoint.transAfterAngle.y = robPoint.trans.y + scanInfoPtr->ldvStandOffDistance * (qSin(qDegreesToRadians(yaw)));
          robPoint.transAfterAngle.x = robPoint.trans.x + (scanInfoPtr->ldvStandOffDistance - (scanInfoPtr->ldvStandOffDistance * (qCos(qDegreesToRadians(yaw)))));

          robPoint.transAfterAngle.z = robPoint.trans.z + scanInfoPtr->ldvStandOffDistance * (qSin(qDegreesToRadians(pitch)));
          robPoint.transAfterAngle.x = robPoint.transAfterAngle.x + (scanInfoPtr->ldvStandOffDistance - (scanInfoPtr->ldvStandOffDistance * (qCos(qDegreesToRadians(pitch)))));


//update orientation for negative angle.
//        if (yaw<0)
//            robPoint.transAfterAngle.z -= 10;
        eulerToQuaternion(yaw,pitch,robPoint.rot);
    }
}*/
