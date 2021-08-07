r#-------------------------------------------------
#
# Project created by QtCreator 2016-07-11T12:11:57
#
#-------------------------------------------------

QMAKE_CXXFLAGS += -MP

QT       += \
            core gui \
            serialport\
            network widgets\
            winextras

#DEFINES += QT_NO_WARNING_OUTPUT QT_NO_DEBUG_OUTPUT

DEFINES += SAMPLESPERPOINT=2048\ #keep samples point in power of 2 to keep the DMA notification aligned
        += BURSTSIZE=16\
        += BYTESPERSAMPLE=2\
        += NUMOFBANDS=2\
        += SPECTBASESIZE=400\
        += MAXVTWAMRANGES=10\
        += ACTUALSYSTEM
#+= DAQ_DEBUG_LOGS\
#+= SIG_FIFO_LEN=512\
#+= VERTCAL_RANGE=3.49\



#CONFIG  += qwt
include(C:/qwt-6.1.3/features/qwt.prf)

LIBS += vfw32.lib Gdi32.lib User32.lib $$PWD/Spectrum/c_header/spcm_win64_msvcpp.lib

AZUREINCLUDE    = "C:\Program Files\Azure Kinect SDK v1.2.0\sdk\include"
AZURELIB        = "C:\Program Files\Azure Kinect SDK v1.2.0\sdk\windows-desktop\amd64\release\lib"

INCLUDEPATH += $$PWD/Spectrum/c_header  \
            += $$PWD/Spectrum/common    \
            += $$PWD/Spectrum/sb5_file  \
            += C:/qwt-6.1.3/include     \
            +=  $${AZUREINCLUDE}


DEPENDPATH += $$PWD/c_header \
           += $${AZUREINCLUDE}


LIBS += -L$${AZURELIB}  \
    k4a.lib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RA_UPI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    laserController.cpp \
    ldvController.cpp \
    Spectrum/common/spcm_lib_card.cpp \
    Spectrum/common/spcm_lib_data.cpp \
    Spectrum/common/ostools/spcm_ostools_win.cpp \
    daqControllerS.cpp \
    dataProcessor.cpp \
    plot.cpp \
    spectrogram.cpp \
    stageController.cpp \
    dialogenlarge.cpp \
    bandpasscontroller.cpp\
    specimageprocessor.cpp \
    robotController.cpp \
    lmsController.cpp


HEADERS  += mainwindow.h \
    laserController.h \
    structDef.h \
    ldvController.h \
    daqControllerS.h \
    dataProcessor.h \
    plot.h \
    spectrogram.h \
    stageController.h \
    dialogenlarge.h \
    bandpasscontroller.h\
    specimageprocessor.h \
    robotController.h \
    lmsController.h

FORMS    += mainwindow.ui \
    dialogenlarge.ui

RESOURCES += \
    MyRes.qrc

#OPENCV

#INCLUDEPATH += H:\OpenCV\Builds\install\include

#LIBS += -LH:/OpenCV/Builds/install/x64/vc12/lib/ \
#     -lopencv_world330d

INCLUDEPATH += D:\OpenCV\Builds\install\include
LIBS += -LD:/OpenCV/Builds/install/x64/vc12/lib/ \
    -lopencv_core330d       \
    -lopencv_calib3d330d    \
    -lopencv_dnn330d        \
    -lopencv_features2d330d \
    -lopencv_flann330d      \
    -lopencv_highgui330d    \
    -lopencv_imgcodecs330d  \
    -lopencv_imgproc330d    \
    -lopencv_ml330d         \
    -lopencv_objdetect330d  \
    -lopencv_photo330d      \
    -lopencv_shape330d      \
    -lopencv_stitching330d  \
    -lopencv_superres330d   \
    -lopencv_video330d      \
    -lopencv_videoio330d    \
    -lopencv_videostab330d

#KINECT

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/'../../../Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64/' -lKinect20
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/'../../../Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64/' -lKinect20

INCLUDEPATH += $$PWD/'../../../Program Files/Microsoft SDKs/Kinect/v2.0_1409/inc'
DEPENDPATH += $$PWD/'../../../Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64'

DISTFILES += \
    robotController.cpp.bak \
    robotController.h.bak

#LMS-------------------------------------------------

INCLUDEPATH += $$PWD/lms

DEPENDPATH  += $$PWD/lms
#-------------------------------------------------
#-------------------------------------------------
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/lms/ -lRTC5DLLx64
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/lms/ -lRTC5DLLx64
#-------------------------------------------------


#////////////////PCL+VTK///////////////////////////

PCLINCLUDE      = "C:/Program Files/PCL/include/pcl-1.8"
BOOSTINCLUDE    = "C:/Program Files/PCL/3rdParty/Boost/include/boost-1_61"
EIGENINCLUDE    = "C:/Program Files/PCL/3rdParty/Eigen/eigen3"
FLANNINCLUDE    = "C:/Program Files/PCL/3rdParty/FLANN/include"
VTKINCLUDE      = "C:/Program Files/PCL/3rdParty/VTK/include/vtk-7.1"
PCLLIB        = "C:/Program Files/PCL/lib/"
BOOSTLIB      = "C:\Program Files\PCL\3rdParty\Boost\lib"


#PCLINCLUDE      = "H:/PCL/include/pcl-1.8"
#BOOSTINCLUDE    = "H:/PCL/3rdParty/Boost/include/boost-1_61"
#EIGENINCLUDE    = "H:/PCL/3rdParty/Eigen/eigen3"
#FLANNINCLUDE    = "H:/PCL/3rdParty/FLANN/include"
#VTKINCLUDE      = "H:/PCL/3rdParty/VTK/include/vtk-7.1"
#PCLLIB          = "H:/PCL/lib/"
#BOOSTLIB        = "H:/PCL/3rdParty/Boost/lib"


INCLUDEPATH +=  $${PCLINCLUDE}\
                $${BOOSTINCLUDE}\
                $${EIGENINCLUDE}\
                $${FLANNINCLUDE}\
                $${VTKINCLUDE}


LIBS += -L$${BOOSTLIB} \
-llibboost_thread-vc120-mt-gd-1_61

LIBS += -L$${PCLLIB}  \
    -lpcl_common_debug              \
    -lpcl_features_debug            \
    -lpcl_filters_debug             \
    -lpcl_io_debug                  \
    -lpcl_io_ply_debug              \
    -lpcl_kdtree_debug              \
    -lpcl_keypoints_debug           \
    -lpcl_ml_debug                  \
    -lpcl_octree_debug              \
    -lpcl_outofcore_debug           \
    -lpcl_people_debug              \
    -lpcl_recognition_debug         \
    -lpcl_registration_debug        \
    -lpcl_sample_consensus_debug    \
    -lpcl_search_debug              \
    -lpcl_segmentation_debug        \
    -lpcl_stereo_debug              \
    -lpcl_surface_debug             \
    -lpcl_tracking_debug            \
    -lpcl_visualization_debug

VTKLIB      = "C:\Program Files\PCL\3rdParty\VTK\lib"

LIBS += -L$${VTKLIB} \
-lvtkalglib-7.1-gd\
-lvtkChartsCore-7.1-gd\
-lvtkCommonColor-7.1-gd\
-lvtkCommonComputationalGeometry-7.1-gd\
-lvtkCommonCore-7.1-gd\
-lvtkCommonDataModel-7.1-gd\
-lvtkCommonExecutionModel-7.1-gd\
-lvtkCommonMath-7.1-gd\
-lvtkCommonMisc-7.1-gd\
-lvtkCommonSystem-7.1-gd\
-lvtkCommonTransforms-7.1-gd\
-lvtkDICOMParser-7.1-gd\
-lvtkDomainsChemistry-7.1-gd\
-lvtkexoIIc-7.1-gd\
-lvtkexpat-7.1-gd\
-lvtkFiltersAMR-7.1-gd\
-lvtkFiltersCore-7.1-gd\
-lvtkFiltersExtraction-7.1-gd\
-lvtkFiltersFlowPaths-7.1-gd\
-lvtkFiltersGeneral-7.1-gd\
-lvtkFiltersGeneric-7.1-gd\
-lvtkFiltersGeometry-7.1-gd\
-lvtkFiltersHybrid-7.1-gd\
-lvtkFiltersHyperTree-7.1-gd\
-lvtkFiltersImaging-7.1-gd\
-lvtkFiltersModeling-7.1-gd\
-lvtkFiltersParallel-7.1-gd\
-lvtkFiltersParallelImaging-7.1-gd\
-lvtkFiltersPoints-7.1-gd\
-lvtkFiltersProgrammable-7.1-gd\
-lvtkFiltersSelection-7.1-gd\
-lvtkFiltersSMP-7.1-gd\
-lvtkFiltersSources-7.1-gd\
-lvtkFiltersStatistics-7.1-gd\
-lvtkFiltersTexture-7.1-gd\
-lvtkFiltersVerdict-7.1-gd\
-lvtkfreetype-7.1-gd\
-lvtkGeovisCore-7.1-gd\
-lvtkgl2ps-7.1-gd\
-lvtkGUISupportQt-7.1-gd\
-lvtkGUISupportQtOpenGL-7.1-gd\
-lvtkGUISupportQtSQL-7.1-gd\
-lvtkhdf5-7.1-gd\
-lvtkhdf5_hl-7.1-gd\
-lvtkImagingColor-7.1-gd\
-lvtkImagingCore-7.1-gd\
-lvtkImagingFourier-7.1-gd\
-lvtkImagingGeneral-7.1-gd\
-lvtkImagingHybrid-7.1-gd\
-lvtkImagingMath-7.1-gd\
-lvtkImagingMorphological-7.1-gd\
-lvtkImagingSources-7.1-gd\
-lvtkImagingStatistics-7.1-gd\
-lvtkImagingStencil-7.1-gd\
-lvtkInfovisCore-7.1-gd\
-lvtkInfovisLayout-7.1-gd\
-lvtkInteractionImage-7.1-gd\
-lvtkInteractionStyle-7.1-gd\
-lvtkInteractionWidgets-7.1-gd\
-lvtkIOAMR-7.1-gd\
-lvtkIOCore-7.1-gd\
-lvtkIOEnSight-7.1-gd\
-lvtkIOExodus-7.1-gd\
-lvtkIOExport-7.1-gd\
-lvtkIOGeometry-7.1-gd\
-lvtkIOImage-7.1-gd\
-lvtkIOImport-7.1-gd\
-lvtkIOInfovis-7.1-gd\
-lvtkIOLegacy-7.1-gd\
-lvtkIOLSDyna-7.1-gd\
-lvtkIOMINC-7.1-gd\
-lvtkIOMovie-7.1-gd\
-lvtkIONetCDF-7.1-gd\
-lvtkIOParallel-7.1-gd\
-lvtkIOParallelXML-7.1-gd\
-lvtkIOPLY-7.1-gd\
-lvtkIOSQL-7.1-gd\
-lvtkIOTecplotTable-7.1-gd\
-lvtkIOVideo-7.1-gd\
-lvtkIOXML-7.1-gd\
-lvtkIOXMLParser-7.1-gd\
-lvtkjpeg-7.1-gd\
-lvtkjsoncpp-7.1-gd\
-lvtklibxml2-7.1-gd\
-lvtkmetaio-7.1-gd\
-lvtkNetCDF-7.1-gd\
-lvtkNetCDF_cxx-7.1-gd\
-lvtkoggtheora-7.1-gd\
-lvtkParallelCore-7.1-gd\
-lvtkpng-7.1-gd\
-lvtkRenderingAnnotation-7.1-gd\
-lvtkRenderingContext2D-7.1-gd\
-lvtkRenderingContextOpenGL-7.1-gd\
-lvtkRenderingCore-7.1-gd\
-lvtkRenderingFreeType-7.1-gd\
-lvtkRenderingGL2PS-7.1-gd\
-lvtkRenderingImage-7.1-gd\
-lvtkRenderingLabel-7.1-gd\
-lvtkRenderingLIC-7.1-gd\
-lvtkRenderingLOD-7.1-gd\
-lvtkRenderingOpenGL-7.1-gd\
-lvtkRenderingQt-7.1-gd\
-lvtkRenderingVolume-7.1-gd\
-lvtkRenderingVolumeOpenGL-7.1-gd\
-lvtksqlite-7.1-gd\
-lvtksys-7.1-gd\
-lvtktiff-7.1-gd\
-lvtkverdict-7.1-gd\
-lvtkViewsContext2D-7.1-gd\
-lvtkViewsCore-7.1-gd\
-lvtkViewsInfovis-7.1-gd\
-lvtkViewsQt-7.1-gd\
-lvtkzlib-7.1-gd


#DEFINES += QWT_DLL

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../qwt-6.1.3/lib/ -lqwt
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../qwt-6.1.3/lib/ -lqwtd

#INCLUDEPATH += $$PWD/../../../qwt-6.1.3/include
#DEPENDPATH += $$PWD/../../../qwt-6.1.3/include

#win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../qwt-6.1.3/lib/libqwt.a
#else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../qwt-6.1.3/lib/libqwtd.a
#else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../qwt-6.1.3/lib/qwt.lib
#else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../qwt-6.1.3/lib/qwtd.lib

#include(C:/qwt-6.1.3/features/qwt.prf)
#QMAKE_CXXFLAGS_WARN_ON -= -w34100
#QMAKE_CXXFLAGS += -wd4100
#QMAKE_CXXFLAGS_WARN_ON -= -wd4189

QMAKE_CXXFLAGS_WARN_ON -= -w34100
QMAKE_CXXFLAGS_WARN_OFF += -wd4100
