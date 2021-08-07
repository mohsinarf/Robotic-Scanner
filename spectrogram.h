#ifndef SPECTROGRAM_H
#define SPECTROGRAM_H

#include <qwt_plot.h>
#include "qwt_plot_zoomer.h"
#include <qwt_plot_spectrogram.h>
#include <qwt_plot_curve.h>
#include <qwt_color_map.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_draw.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_renderer.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_engine.h>
#include "structDef.h"
#include <QElapsedTimer>
#include <qwt_text_label.h>
#include <QMessageBox>
//#include "aviUtil.h"


class ColorMapGrey: public QwtLinearColorMap
{
public:
    ColorMapGrey():
        QwtLinearColorMap( Qt::white, Qt::black )
    {
        //addColorStop( 0.8, Qt::white );
    }
};

class ColorMap: public QwtLinearColorMap
{
public:
    ColorMap():
        QwtLinearColorMap( Qt::blue, Qt::red )
    {
        addColorStop( 0.5, Qt::white );
    }
};

class VTWAMColorMap: public QwtLinearColorMap
{
public:
    VTWAMColorMap():
        QwtLinearColorMap(QColor(0,0,189), QColor(132,0,0))
    {
       addColorStop(1.0/12.0*1.0, QColor(0,0,255));
       addColorStop(1.0/12.0*2.0, QColor(0,66,255));
       addColorStop(1.0/12.0*3.0, QColor(0,132,255));
       addColorStop(1.0/12.0*4.0, QColor(0,189,255));
       addColorStop(1.0/12.0*5.0, QColor(0,255,255));
       addColorStop(1.0/12.0*6.0, QColor(66,255,189));
       addColorStop(1.0/12.0*7.0, QColor(132,255,132));
       addColorStop(1.0/12.0*8.0, QColor(189,255,66));
       addColorStop(1.0/12.0*9.0, QColor(255,255,0));
       addColorStop(1.0/12.0*10.0, QColor(255,189,0));
       addColorStop(1.0/12.0*11.0, QColor(255,66,0));
       addColorStop(1.0/12.0*12.0, QColor(189,0,0));
    }
};

class SpectZoomer: public QwtPlotZoomer
{
public:
    float scanInterval;
    short *shortframePointer; // just to display the current value for debugging
    int *intframePointer; // just to display the current value for debugging
    short y_length;
    bool isIntData;
    SpectZoomer( QWidget *canvas ):
        QwtPlotZoomer( canvas )
    {
        setTrackerMode( AlwaysOn );
        scanInterval = 0.5;
        y_length = 400;
        shortframePointer = NULL;
        intframePointer = NULL;
        isIntData = 0;
    }

    virtual QwtText trackerTextF( const QPointF &pos ) const
    {
        QColor bg( Qt::white );
        bg.setAlpha( 200 );
        QwtText text;
        text = QwtPlotZoomer::trackerTextF( pos * scanInterval );
        /*
        if ( (isIntData == 0 &&shortframePointer!=NULL ) || (isIntData == 1 &&intframePointer!=NULL ))
        {
            if (pos.x() >= 0 && pos.y() >= 0)
            {
                if (isIntData)
                    text = QString::number(intframePointer[((int)(pos.y())+(int)(pos.x()*y_length))]);
                    else
                    text = QString::number(shortframePointer[((int)(pos.y())+(int)(pos.x()*y_length))]);
            }
        }
        */
        text.setBackgroundBrush( QBrush( bg ) );
        return text;
    }

};

class GraphZoomer: public QwtPlotZoomer
{
public:
    GraphZoomer( QWidget *canvas ):
        QwtPlotZoomer( canvas )
    {
        setTrackerMode( AlwaysOn );
    }

    virtual QwtText trackerText( const QPoint &pos ) const
    {
        QColor bg( Qt::white );
        bg.setAlpha( 200 );
        QwtText text;

        //We have only one curve in the canvas so no need to iterate the whole list.
        QwtPlotCurve *Curve =   static_cast<QwtPlotCurve *> (*this->plot()->itemList().begin());
        if (Curve != NULL)
        {
            double d;
            int index = Curve->closestPoint(pos, &d);
            if (d < 5)
            {
               qDebug()<<Curve->sample(index);
               text = QwtPlotZoomer::trackerTextF(Curve->sample(index));
            }
        }

        text.setBackgroundBrush( QBrush( bg ) );

        return text;
    }
};

class mydata: public QwtRasterData
{
public:
    //keep em public for ease of access. only access from with spectrogram plot.
    int intensity;
    int x_length;
    int y_length;
    short *shortframePointer;
    int *intframePointer;
    QElapsedTimer timer;
    bool isIntData;

    //void updateDataAxis();
    mydata(int intensity = 0, int y_length = 50, int x_length = 50)
    {
        this->intensity     = intensity;
        this->x_length      = x_length;
        this->y_length      = y_length;
        this->shortframePointer  = NULL;
        this->intframePointer  = NULL;
        isIntData           = 0;
        updateDataAxis();
    }
    ~mydata()
    {

    }
    void updateDataAxis()
    {
        setInterval( Qt::XAxis, QwtInterval( 0, (x_length-0.01) ) );
        setInterval( Qt::YAxis, QwtInterval( 0, (y_length-0.01) ) );
        if (isIntData)
            setInterval( Qt::ZAxis, QwtInterval( 0 , intensity )) ;
        else
            setInterval( Qt::ZAxis, QwtInterval( -intensity, intensity )) ;
    }

    virtual double value( double x, double y ) const//shifted
    {
        int x_pos = static_cast<int>(x);
        int y_pos = static_cast<int>(y);
        //double ReturnValue = 0;
/*
        // this is just for bench marking and should be removed later
        static int paintedPixels;
        if (x == 0 && y == 0)
        {
            qDebug()<<"virtual double value - Started paint";
            const_cast <mydata *>(this)->timer.start();
            paintedPixels = 0;
        }
*/
        if ( (isIntData == 0 && shortframePointer!=NULL ) || (isIntData == 1 &&intframePointer!=NULL ))
        {
            if (isIntData)
                return((double)(intframePointer[((y_pos)+(x_pos*y_length))])); // change a straight frame to 2D frame
                else
                return((double)(shortframePointer[((y_pos)+(x_pos*y_length))])); // change a straight frame to 2D frame

            /*
            if (isIntData)
                return((double)(intframePointer[((x_pos)+(y_pos*x_length))])); // change a straight frame to 2D frame
                else
                return((double)(shortframePointer[((x_pos)+(y_pos*x_length))])); // change a straight frame to 2D frame
            */
        }
        else
        { // just make a pattern in case the data pointer in NULL
            if ( ((int)x%2) == 0 || ((int)y%2) == 0 )
                return(-10000);
            else
                return(10000);
        }
/*
        // this is just for bench marking and should be removed later
        if ( (paintedPixels++) == 100000)
            qDebug() << "virtual double value - painted the whole frame in (msec) : "<<timer.elapsed();
*/
        //return ReturnValue;
    }
/*
 * //Median Filter Coding

    //FiltType = 1: median filter
    //FiltType = 2: Spatial filter
    //KrnlSize = 3 5 7 -> //KrnlRadius = 1 2 3
    inline short imageFilterOpt(char FiltType, char krnlRadius, int x_pos, int y_pos) const
    {
        int top,bottom,left,right,yKernel,xKernel,vectorIndex;
        short kernelData[50];

        if ( (x_pos < krnlRadius) ||
             (y_pos < krnlRadius) ||
             (y_pos >= (y_length-krnlRadius)) ||
             (x_pos >= (x_length-krnlRadius)))
        {
            return framePointer[((x_pos)+(y_pos*x_length))]; // no filtering at the borders
        }

        top     = y_pos-krnlRadius;
        bottom  = y_pos+krnlRadius;
        left    = x_pos-krnlRadius;
        right   = x_pos+krnlRadius;

        vectorIndex = 0;

        for (yKernel = top; yKernel<=bottom; yKernel++)
        {
            for (xKernel = left; xKernel<=right; xKernel++)
            {
                kernelData[vectorIndex++] = framePointer[xKernel+yKernel*x_length];
            }
        }

        if (krnlRadius == 1)
            return (optMed9(&kernelData[0]));
        if (krnlRadius == 2)
            return (optMed25(&kernelData[0]));
        if (krnlRadius == 3)
            return (optMed49(&kernelData[0]));
    }



    //FiltType = 1: median filter
    //FiltType = 2: Spatial filter
    inline short imageFilter(char FiltType, char krnlSz, int x_pos, int y_pos) const
    {
        short radius = ((krnlSz-1)/2); //size 3 radius -> 1
        int top,bottom,left,right,yKernel,xKernel,vectorIndex,medianIndex = (krnlSz*krnlSz)/2;
        QVector<short> kernelVector;
        kernelVector.reserve(krnlSz*krnlSz);

        if ( (x_pos < radius) ||
             (y_pos < radius) ||
             (y_pos >= (y_length-radius)) ||
             (x_pos >= (x_length-radius)))
        {
            return framePointer[((x_pos)+(y_pos*x_length))]; // no filtering at the borders
        }

        top     = max(y_pos-radius,0);
        bottom  = min(y_pos+radius,y_length-1);

        left    = max(x_pos-radius,0);
        right   = min(x_pos+radius,x_length);

        vectorIndex = 0;

        for (yKernel = top; yKernel<=bottom; yKernel++)
        {
            for (xKernel = left; xKernel<=right; xKernel++)
            {
                kernelVector.insert(vectorIndex++,framePointer[xKernel+yKernel*x_length]);
            }
        }
        if(kernelVector.size() < (krnlSz*krnlSz))
        {
            qWarning("imageFilter - Kernel not fully overlapped - FilterType: %d, krnlSz: %d",FiltType,krnlSz);
        }
        qSort(kernelVector);
        return (kernelVector[medianIndex]);
    }
    */
};

class MyScaleDraw: public QwtScaleDraw
{
public:
    float scanInterval;
    MyScaleDraw()
    {

        //setTickLength( QwtScaleDiv::MajorTick, 10 );
        //setTickLength( QwtScaleDiv::MinorTick, 2 );
        //setTickLength( QwtScaleDiv::MediumTick, 5 );

        setLabelRotation( 0 );
        setLabelAlignment( Qt::AlignLeft | Qt::AlignVCenter );
        setSpacing( 10 );

    }

    virtual QwtText label( double value ) const
    {
        QwtText h=QwtText(QString::number(value*scanInterval));
        return h;
    }

    void invalidateCacheLoc()
    {
        invalidateCache();
    }
};

class spectrogram: public QwtPlot
{
    Q_OBJECT
    int fnum;
    int dial;
    int g_x;
    int g_y;
    float g_int;
    int stop;
    int movie_type;

public:
    spectrogram(QWidget * = NULL, structScan * scanInfoPtr = NULL, short leftPosArg=0, short topPosArg=0,
                 short widthArg=SPECTBASESIZE+150, short heightArg=SPECTBASESIZE, QString titleArg=NULL ,
                int noOfSlavesArg=1, int canHeightArg = 600, int canWidthArg = 1200);
    ~spectrogram();
    bool enlargeEnabled;

public Q_SLOTS:
    void selectPoint(QPointF Pos);
    void updateData(void *framePointer, QString title=NULL, bool isIntData=0);
    void updateAxisXY(int enlargeFactor = 0, bool calcSize = true, bool makePotrait = false);
    void setIntensity( int );
    void savePlot(QString outfolderpath = 0);
    void SaveMovie(QString outfolderpath = 0);
    QPixmap getPlotPixmap(int frameNum);
    void setMovieImageSize();
    void placeSlaveslot(int leftPos, int topPos, int widthSetByMaster, int heightSetByMaster, bool makePotrait);
    void toggleUWPIGreyScale(bool isGreyForUWPI);
    float polyArea();
    void selectPointForPolygon( QPointF Pos );
    void clearPolyGon( QPointF Pos );


private:
    QwtPlotSpectrogram *d_spectrogram;
    SpectZoomer* zoomer;
    MyScaleDraw *scaleDrawXaxis, *scaleDrawYaxis;

    structScan *scanInfoPtr;
    short framePointer;
    mydata *data;
    short leftPos,topPos,width,height;
    QString title;
    QImage *imageForMovie;
    QPainter *painterForMovie;
    QRect imageRect;
    QwtPlotRenderer renderer;
    QPixmap *pixMapCurptr;
    QWidget *container;
    int noOfSlaves;
    bool isIntData;
    bool isGreyForUWPI;
    int imgHeight,imgWidth;
    QPolygonF pointList;
    QwtPlotCurve *boundaryCurve;
    int canHeight;
    int canWidth;
    bool makePotraitMem;



    void showSpectrogram( bool on );
    //void setAlpha( int );

    void setcolorbar();
    void go_annimation();
    void stopper( int stopper );

    signals:
    void pointToPlot(int SelectedImpingePoint);
    void placeSlave(int leftPos,int topPos, int width, int height,bool);

};
#endif
