#ifndef SPECIMAGEPROCESSOR_H
#define SPECIMAGEPROCESSOR_H

#define SAFE_RELEASE(p) { if ( (p) ) { (p)->Release(); (p) = 0; } }

#define CALCNORMALS
#define DEPTHIMGAVGCNT 50
//#define SHOWCLOUD

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QLabel>
#include <QToolTip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <QMessageBox>
#include "structDef.h"
#include "stageController.h"
#include "robotController.h"
#include "structDef.h"
#include "qnamespace.h"
#include <Kinect.h>
#include "lmsController.h"
#include <QFileDialog>
#include <k4a/k4a.h>
#ifdef CALCNORMALS
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>

    #ifdef SHOWCLOUD
    #include <pcl/io/vtk_lib_io.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <vtkAutoInit.h>
    #endif
#endif

using namespace cv;
using namespace std;

typedef struct _Point4f
{
    float X;
    float Y;
    float Z;
    float theta;
    float phi;
} 	Point5f;


enum imageType {
    COL_IMAGE       = 0,
    IR_IMAGE        = 1,
    DEP_IMAGE       = 2,
    COL_DEP_IMAGE   = 3,
    IR_DEP_IMAGE    = 4
};

enum originType {
    TOP_LEFT        = 0,
    BOTTOM_LEFT     = 1,
    TOP_RIGHT       = 2,
    BOTTOM_RIGHT    = 3,
};

struct lineData {
  Point2f pt1;
  Point2f pt2;
};

struct calibDataStruct
{
    Point3f robPt;
    Point3f kinPt;
    Point3f kinPtAtRobZero;
    Point3f kinCordOffset;
    bool isValid;
};

static void onMouseOpenCvWrapper( int event, int x, int y, int, void* userdata);

class HLabel : public QLabel
{
    Q_OBJECT
public:
    HLabel(QWidget *parent = 0, Qt::WindowFlags f=0);
    HLabel(const QString &text, QWidget *parent=0, Qt::WindowFlags f=0);
    ~HLabel();

    protected:
    void mousePressEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);
    void wheelEvent(QWheelEvent *ev);

    signals:
    void clicked(QMouseEvent *ev);
    void moved(QMouseEvent *ev);
    void wheelMove(QWheelEvent *ev);
};

class specImageProcessor : public QObject
{
    Q_OBJECT
public:
    specImageProcessor(structDaq *daqInfoPtrArg, structScan *scanInfoPtrArg, lmsController *lmsPtrArg,
                       QString ImgPath_arg, QWidget *widgetInteractive_arg, robotController *robContArg,
                       QLabel *labelProcessImageDebug_arg);
    ~specImageProcessor();

public slots:
        void edgeDetection();
        void setCannyThLo(int cannyThLo_arg);
        void setCannyThHi(int cannyThHi_arg);
        void setBlurKerSz(int blurKerSz_arg);
        void setCloseKerSz(int closeKerSz_arg);
        void setCloseKerItr(int closeKerItr_arg);
        void incZoom();
        void decZoom();
        void mousePressEvent(QMouseEvent *ev);
        void mouseMoveEvent(QMouseEvent *ev);
        void mouseWheelEvent(QWheelEvent *ev);

        bool calParsForStage(); //unused remove later
        bool calcScanGrid();
        Point laserTrackerRobotArm(Point searchCenter, int searchRange);
        Point yellowCircleTracker();
        void captureImage();
        void enASScanUpdateTimer(bool enable);
        inline void convToStageUnits(Point2f inPoint, Point2f &outPoint, short outOriginType=1, bool getStageOffset=true);
        inline void kinToRobUnits(Point3f kinPoint, structRobTarget &robPoint, float yaw = 0,float pitch = 0, bool enLog=false, originType outOriginType=BOTTOM_RIGHT);
        inline void robToKinUnits(Point2f robPoint,Point2f &kinPoint, originType outOriginType=BOTTOM_RIGHT);
        inline float stickToGrid(float input, float gridSpacingmm);
        void depthCalibrate();
        void xyCalibrate();
        void save();
        void save(QString dataPath);
        void load(QString dataPath);
        void changeImageType();
        void setRefreshInterval(QString refIntervalStr);
        #ifdef CALCNORMALS
            void onMouseOpenCv(int event, int x, int y);
            void setNormSmoothingSize(int normSmoothingSizeArg);
            void changeNormCalcMethod();
            void setMaxDepthChangeFactor(int maxDepthChangeFactorArg);
            void setEnableDepthDependentSmoothing(bool val);
            void setThetaMaxAngle(int thetaMaxAngleArg);
            void setPhiMaxAngle(int phiMaxAngleArg);
            void colouredDisplay(int x, int y, cv::ColormapTypes colBar, Mat inMat, String winName, int maxScaleVal = 90);
            void colouredDisplayNeg(int x,int y, cv::ColormapTypes colBar, Mat inMat, cv::String winName, int maxScaleVal=90);
			void genTestDepth();
        	void setSmoothSigmaColour(int smoothSigmaColourArg);
        	void setSmoothSigmaSpace(int smoothSigmaSpaceArg);
        	void setSmoothKernelSize(int smoothKernelSizeArg);
        #endif
        void saveKinectData(QString dataPath=NULL);
        void loadKinectData(QString dataPath=NULL);
        
        //Point greenLaserTracker();
        Point greenLaserTrackerWithBgSubtraction(bool storeBgImage=false, int bgImgIndex=0);
        Point redLaserTracker();
        void eulerToQuaternion(short yaw, short pitch, structOrient &robOrient);
        void loadMesh(bool showPtCloud = true);
        void setLoadKinFusCloud(bool loadKinFusCloud_arg);
#ifdef CALCNORMALS
    void calcNormals();
    void calcNormalsPcl(bool showCloud = false);
#endif

public:
    inline QImage openCVtoQimg(Mat opencvImg, bool convKinectImage=false);
    inline QImage drawContoursAndMarkers(Mat &contorImg,bool enLog = true, int selectedContOnly = -1, bool firstLastMarkerOnly = false);
    inline bool IsPointInBoundingBox(float x1, float y1, float x2, float y2, float px, float py);
    inline bool greaterOrEqual(float number1, float number2);
    inline bool lessOrEqual(float number1, float number2);
    inline bool almostEqual(float number1, float number2);
    inline float maximum(float number1, float number2, float number3);
    inline bool lineIntersection(const Point2f &a1, const Point2f &b1, const Point2f &a2,
                                              const Point2f &b2, Point2f &intersection);
    inline void genTestPolygon(int testNo = 0);
    bool contourTouchesImageBorder(std::vector<cv::Point>& contour, cv::Size& imageSize);
    bool getColourImageFromKinect(Mat *ptrToMat);
    bool getImageFromKinect(Mat *ptrToMat, int imageType, int depthRange );// pass a MAT pointer to be populated with the read data.
    bool initKinect();
    void grey16ToRgb8(unsigned short *src, uchar *dest, int width, int height, bool isDepthData);
    inline int ptToDataIdxColor(int xColor,int yColor);
    inline int ptToDataIdxDepth(int xDepth,int yDepth);
    inline bool isRobPtWihinRange(structPos robTarget);
    inline _CameraSpacePoint mTomm(_CameraSpacePoint InM, bool enStickToGrid=false, float gridSpacingmm = 0.5);
    void convHomeToKinCol();
    void smoothFilter();
    void pColorToAngle(int xPx, int yPx, int &phiAngle, int &thetaAngle);
	bool initAzure();



    structScan *scanInfoPtr;
    robotController* robotCont;
    stageController* stageCont;
    structDaq *daqInfoPtr;
    QString ImgPath;
    Mat origCvImg;
    Mat ctrImg;
    Mat globImg;
    Mat depImgSumMat;
    int depImgAvgCnt;
    Mat depImgAvgMat;
    Mat depImgAvgMatFliped;
    Mat depImgAvgDisMat;
    Mat depImgTemp;
    Mat normals,normalsByPcl_mat;
    Mat normalAngles;

    vector<vector<Point>> contours;
    int selectedContour;
    vector<Vec4i> hierarchy;
    vector<Vec4b> colorTable;
    int colorTableSize;

    QImage procQtImg;
    int cannyThLo,cannyThHi,blurKerSz,closeKerSz,closeKerItr;
    unsigned int zoomFactor;
    VideoCapture camHndl;
    HLabel *labelOrigImage;
    QWidget *widgetInteractive;
    QLabel *labelProcessImageDebug;
    Point2f *pxPermm;
    Point2f *stageOffset;
    Point2f *laserOffset;
    int numOfCountours;
    bool maxContOnly;
    int preSelectShapeIndex;
    Point preSelectShapePoint;
    bool useKinect;
	bool useAzure;
    calibDataStruct calibData;

    //just for debugging
    stageController *stage;
    QTimer *ASScanUpdateTimer;
    IKinectSensor *pMyKinect;
    IMultiSourceFrameReader *pMultiSourceFrameReader;
    IMultiSourceFrame* pMultiSourceFrame;
    IDepthFrame* pDepthFrame;
    IColorFrame* pColorFrame;
    ILongExposureInfraredFrame* pLongIRFrame;
    DepthSpacePoint* pColor2depth;
    CameraSpacePoint *pDepth2xyz;    // Maps depth pixels to 3d coordinates
    CameraSpacePoint *pColor2xyz;    // Maps color pixels to 3d coordinates
    CameraSpacePoint *pColor2xyzAvg;    // Maps color pixels to 3d coordinates
    Point3f clickedPt;
    ColorSpacePoint pColorSpaceOrigin;
    unsigned short* pDepthBuffer;
    uint8_t * aDepthTransBuffer;
    unsigned short* pDepthBufferAvg;
    int colorHeight,colorWidth,depthHeight,depthWidth;
    int imageType;
    Point2i robHomeclPx;
    Point2i robBotRightclPx;
    bool robRangeDrawn;
    int refreshInterval;
    lmsController *lmsPtr;
    bool allowDistanceAdjust;
    bool originDrawn;
#ifdef CALCNORMALS
    pcl::PointCloud<pcl::Normal>::Ptr normalsPcl;
    int normSmoothingSize;
    int maxDepthChangeFactor;
    int normCalcMethod;
    bool enableDepthDependentSmoothing;
    int thetaMaxAngle;
    int phiMaxAngle;
    Mat thetaMat,thetaPosMat,thetaPosDispMat,thetaFiltMat,thetaFiltPosMat,thetaFiltPosDispMat;
    Mat phiMat,phiPosMat,phiPosDispMat,phiFiltMat,phiFiltPosMat,phiFiltPosDispMat;
    Mat bgImg[2];

    //
    int smoothKernelSize;
    int smoothSigmaColour;
    int smoothSigmaSpace;
    bool loadKinFusCloud;
#endif


    Mat cameraMatrix, distCoeffs;
    k4a_device_t azureHandle;
    k4a_device_configuration_t AzureConfig;
    Mat pLocImageCloud;
    Mat undistMap1,undistMap2;
    k4a_transformation_t transformation_handle;
    bool flipImage;


signals:
    void originalImage(QPixmap origImage);
    void setStagePosX(uint,bool);
    void setStagePosZ(uint,bool);
    void plotGraph(short *, QString);
};

#endif // SPECIMAGEPROCESSOR_H
