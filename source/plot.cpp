#include "plot.h"
#include <qwt_plot_grid.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_directpainter.h>
#include <qwt_curve_fitter.h>
#include <qwt_painter.h>
#include <qevent.h>
#include <QTimer.h>

Plot::Plot(QWidget *parent , bool vMarkEnArg, QString title,structDaq *daqInfoPtrArg):
    QwtPlot( parent )
{
    //d_directPainter = new QwtPlotDirectPainter();
    daqInfoPtr = daqInfoPtrArg;
    setTitle(title);
    setAutoReplot( false );
    setCanvas( new QwtPlotCanvas() );

    plotLayout()->setAlignCanvasToScales( true );

    //setAxisTitle( QwtPlot::xBottom, "Samples" );
    //setAxisScale( QwtPlot::xBottom, 1, SAMPLESPERPOINT);

    setAxisTitle( QwtPlot::xBottom, "Time (\xC2\xB5s)" );
    setAxisScale( QwtPlot::xBottom, 0, (double)((SAMPLESPERPOINT-1))/(double)(daqInfoPtr->SamplingFreq));

    setAxisTitle( QwtPlot::yLeft, "Amplitude (bit)" );
    setAxisScale( QwtPlot::yLeft,-32768,32767 ); // 32752 -> 3.49 & -32768 -> -3.49
    //setAxisScale( QwtPlot::yLeft,-10000,10000 ); // 32752 -> 3.49 & -32768 -> -3.49

    //change the yaxis display
    QwtPlotGrid *grid = new QwtPlotGrid();
    grid->setPen( Qt::gray, 0.0, Qt::DotLine );
    grid->enableX( true );
    grid->enableXMin( true );
    grid->enableY( true );
    grid->enableYMin( false );
    grid->attach( this );
/*
    d_origin = new QwtPlotMarker();
    d_origin->setLineStyle( QwtPlotMarker::Cross );
    //d_origin->setValue( d_interval.minValue() + d_interval.width() / 2.0, 0.0 );
    d_origin->setLinePen( Qt::gray, 0.0, Qt::DashLine );
    d_origin->attach( this );
*/
    d_curve = new QwtPlotCurve();
    d_curve->setStyle( QwtPlotCurve::Lines );
    d_curve->setPen( canvas()->palette().color( QPalette::WindowText ) );
    d_curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );
    d_curve->setPaintAttribute( QwtPlotCurve::ClipPolygons, false );
    //d_curve->setData( new CurveData() );
    //d_curve->setData
    d_curve->attach( this );

    if (this->vMarkEn = vMarkEnArg)
    {
        vertMarker = new QwtPlotMarker();
        //marker->setTitle( QString( "Event %1" ).arg( i + 1 ) );
        vertMarker->setLineStyle( QwtPlotMarker::VLine );
        vertMarker->setLinePen( Qt::blue, 2, Qt::SolidLine );
        vertMarker->attach( this );
        vertMarker->setVisible(false);
    }
    else
        vertMarker=NULL;

    x_pt_d_loc = new double[SAMPLESPERPOINT]();
    y_pt_d_loc = new double[SAMPLESPERPOINT]();
/*
    for (int i=0;i<SAMPLESPERPOINT;i++)
        x_pt_d_loc[i]=(double)((i))/(double)(daqInfoPtr->SamplingFreq); // initiate once
*/
    //marker->setValue( QwtDate::toDouble( dt ), 0.0 );
    //marker->setItemAttribute( QwtPlotItem::Legend, true );

    zoomer = new QwtPlotZoomer( canvas() );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect2,
        Qt::RightButton, Qt::ControlModifier );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect3,
        Qt::RightButton );

    QwtPlotPanner *panner = new QwtPlotPanner( canvas() );
    panner->setAxisEnabled( QwtPlot::yRight, false );
    panner->setMouseButton( Qt::MidButton );

    zoomer->zoom(QRectF::QRectF(0,-90,450,180));
    zoomer->setZoomBase(QRectF::QRectF(0,-90,450,180));

}

Plot::~Plot()
{
   // delete d_curve;
   // if (vertMarker!=NULL)
    //    delete vertMarker;

    delete x_pt_d_loc;
    delete y_pt_d_loc;
}

void Plot::updateAxisScale(int xPosRange, int yRange )
{
    setAxisTitle( QwtPlot::xBottom, "Time (\xC2\xB5s)" );
    setAxisScale( QwtPlot::xBottom, 0, (double)((SAMPLESPERPOINT))/(double)(daqInfoPtr->SamplingFreq));
    //setAxisScale( QwtPlot::xBottom, 0, (double)((xPosRange)));

    setAxisTitle( QwtPlot::yLeft, "Amplitude (bit)" );
    //setAxisScale( QwtPlot::yLeft,-yRange,yRange ); // 32752 -> 3.49 & -32768 -> -3.49
    setAxisScale( QwtPlot::yLeft,-32000,32000 ); // 32752 -> 3.49 & -32768 -> -3.49
    //setAxisScale( QwtPlot::yLeft,-10000,10000 );
    updateAxes();

    for (int i=0;i<SAMPLESPERPOINT;i++)
        x_pt_d_loc[i] = (double)((i))/(double)(daqInfoPtr->SamplingFreq); // initiate once

//    for (int i=0;i<SAMPLESPERPOINT;i++)
//            x_pt_d_loc[i] = (double)((i)); // initiate once
}

void Plot::UpdateCurve(short *y_pt, QString Title)
{
    //double debugVal;
   //for (int i=0;i<SAMPLESPERPOINT;i++)
       //debugVal = y_pt_d_loc[i] = y_pt[i]; //just copy through pointer

   for (int i=0;i<SAMPLESPERPOINT;i++)
        y_pt_d_loc[i] = y_pt[i]; //just copy through pointer

   this->d_curve->setSamples(x_pt_d_loc,y_pt_d_loc,SAMPLESPERPOINT);

   if(this->vMarkEn)
       vertMarker->setVisible(true);
   if (Title!=NULL)
       setTitle(Title);

   replot();
}

void Plot::UpdateCurveOsciDouble(double *y_pt)
{
    setAxisScale( QwtPlot::yLeft,-3.5,3.5); // 32752 -> 3.49 & -32768 -> -3.49

  //debug
    /*
    double y_pt_loc[SAMPLESPERPOINT];
    for (int i =0; i<SAMPLESPERPOINT;i++)
        y_pt_loc[i] = y_pt[i];

    qDebug()<<"UpdateCurveOsci-DebugTime"<<y_pt<<y_pt_loc[100]<<y_pt_loc[101]<<y_pt_loc[102]<<y_pt_loc[103]<<y_pt_loc[104];
*/
    this->d_curve->setSamples(x_pt_d_loc,y_pt,SAMPLESPERPOINT);
    replot();
}

void Plot::initPlot()
{
    setTitle("");
    for (int i=0;i<SAMPLESPERPOINT;i++)
         y_pt_d_loc[i] = 50000; //outside of the y-axis range

    this->d_curve->setSamples(x_pt_d_loc,y_pt_d_loc,SAMPLESPERPOINT);
    replot();
}

void Plot::updateVertMarker(int markerPos)
{
    vertMarker->setValue( ((double)(double)((markerPos))/(double)(daqInfoPtr->SamplingFreq)) ,0.0);
    replot();
    //qDebug()<<"Plot::verticleMarker"<<markerPos;
}

/*
void Plot::refreshPlot()
{
     replotRequested = false;
     replot();
}
*/
//dont need to re-define the virtual function
/*
void Plot::replot()
{
    CurveData *data = static_cast<CurveData *>( d_curve->data() );
    data->values().lock();

    QwtPlot::replot();
    d_paintedPoints = data->size();

    data->values().unlock();
}
*/

