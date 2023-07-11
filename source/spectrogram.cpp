#include "spectrogram.h"
#include "qwt_symbol.h"

#define WIDTHOFYAXISANDCOLOURBAR            150
#define WIDTHOFYAXISANDCOLOURBARNOTITLE     124
#define HEIGHTOFBOTTOMXAXIS                 35


#define DSTBWSPECT  90
#define DSTBWSPECTPOTRAIT  25

spectrogram::spectrogram(QWidget *parent, structScan * scanInfoPtr_arg ,
                         short leftPosArg, short topPosArg, short widthArg, short heightArg,
                         QString titleArg, int noOfSlavesArg,int canHeightArg,int canWidthArg):
    QwtPlot( parent )
{

    //this->axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted,true);
    //this->axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Inverted,true);

    this->scanInfoPtr = scanInfoPtr_arg;
    canHeight   = canHeightArg;
    canWidth    = canWidthArg;

    int y_length = (scanInfoPtr->scanHeight/scanInfoPtr->scanInterval);
    int x_length = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval);
    x_length = y_length = 51;
    this->enableAxis(QwtPlot::yLeft, true);

    //this->enableAxis(QwtPlot::xTop, true);
    //this->enableAxis(QwtPlot::xBottom, false);
    scaleDrawXaxis = new MyScaleDraw;
    scaleDrawYaxis = new MyScaleDraw;


    this->setAxisScaleDraw(QwtPlot::xBottom,scaleDrawXaxis);
    this->setAxisScaleDraw(QwtPlot::yLeft,scaleDrawYaxis);

    QwtPlotPicker* picker=new QwtPlotPicker(canvas());
    picker->setStateMachine(new QwtPickerClickPointMachine);
    //picker->setRubberBand( QwtPicker::CrossRubberBand );
    picker->setMousePattern(QwtPicker::MouseSelect1,Qt::MiddleButton);
    connect(picker,SIGNAL(selected(QPointF)),SLOT(selectPoint(QPointF)));

    d_spectrogram = new QwtPlotSpectrogram();
    d_spectrogram->setRenderThreadCount( 5 ); // use system specific thread count

    d_spectrogram->setColorMap( new ColorMap() );
    d_spectrogram->setCachePolicy( QwtPlotRasterItem::NoCache );

    data = new mydata(500,y_length,x_length);
    d_spectrogram->setData(data);
    d_spectrogram->attach( this );
/*
    QList<double> contourLevels;
    for ( double level = 0.5; level < 10.0; level += 1.0 )
        contourLevels += level;
    d_spectrogram->setContourLevels( contourLevels );
*/
    setcolorbar();

    plotLayout()->setAlignCanvasToScales( true );

    // LeftButton for the zooming
    // MidButton for the panning
    // RightButton: zoom out by 1
    // Ctrl+RighButton: zoom out to full size

    zoomer = new SpectZoomer( canvas() );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect2,
        Qt::RightButton, Qt::ControlModifier );
    zoomer->setMousePattern( QwtEventPattern::MouseSelect3,
        Qt::RightButton );

    QwtPlotPanner *panner = new QwtPlotPanner( canvas() );
    panner->setAxisEnabled( QwtPlot::yRight, false );
    panner->setMouseButton( Qt::MidButton );


    // Avoid jumping when labels with more/less digits
    // appear/disappear when scrolling vertically
/*
    const QFontMetrics fm( axisWidget( QwtPlot::yLeft )->font() );
    QwtScaleDraw *sd = axisScaleDraw( QwtPlot::yLeft );
    sd->setMinimumExtent( fm.width( "100.00" ) );
*/
    const QColor c( Qt::darkBlue );
    zoomer->setRubberBandPen( c );
    zoomer->setTrackerPen( c );


    zoomer->zoom(QRectF::QRectF(0,0,x_length,y_length));
    zoomer->setZoomBase(QRectF::QRectF(0,0,x_length,y_length));

    qDebug()<<"spectrogram::spectrogram - x_length"<<x_length<<"y_length"<<y_length
           <<"canvasWidth"<<canvas()->width()
           <<"canvasHeight"<<canvas()->height()
           <<"canHeight"<<canHeight;

    leftPos = leftPosArg;
    topPos  = topPosArg;

    //width   = widthArg;
    //height  = heightArg;

    width           = canHeight + WIDTHOFYAXISANDCOLOURBARNOTITLE;
    height          = canHeight + HEIGHTOFBOTTOMXAXIS;
    makePotraitMem  = false;

    this->imgHeight = height;
    this->imgWidth  = width;

    this->setGeometry(QRect(leftPos,topPos,width,height));
    this->updateGeometry();

    title = titleArg;
    this->setTitle(title);
    replot();

    imageForMovie = NULL;
    painterForMovie = NULL;

    container        = parent;
    noOfSlaves       = noOfSlavesArg;
    isIntData        = false;
    isGreyForUWPI    = false;
    enlargeEnabled   = false;
    //polygon based area calculations.
    boundaryCurve = new QwtPlotCurve();
    boundaryCurve->setStyle( QwtPlotCurve::Lines );
    boundaryCurve->setPen( canvas()->palette().color( QPalette::WindowText ) );
    boundaryCurve->setRenderHint( QwtPlotItem::RenderAntialiased, true );
    boundaryCurve->setPaintAttribute( QwtPlotCurve::ClipPolygons, false );
    boundaryCurve->setPaintAttribute( QwtPlotCurve::FilterPoints, true );
    boundaryCurve->setSymbol( new QwtSymbol( QwtSymbol::Diamond,Qt::gray, c, QSize( 8, 8 ) ) );
    //d_curve->setData( new CurveData() );
    //d_curve->setData
    boundaryCurve->attach( this );

    QwtPlotPicker* pickerPoly=new QwtPlotPicker(canvas());
    pickerPoly->setStateMachine(new QwtPickerDragPointMachine);
    pickerPoly->setMousePattern(QwtPicker::MouseSelect1,Qt::LeftButton,Qt::ControlModifier);
    connect(pickerPoly,SIGNAL(selected(QPointF)),SLOT(selectPointForPolygon(QPointF)));
    connect(pickerPoly,SIGNAL(moved(QPointF)),SLOT(selectPointForPolygon(QPointF)));

    QwtPlotPicker* pickerPolyClear=new QwtPlotPicker(canvas());
    pickerPolyClear->setStateMachine(new QwtPickerClickPointMachine);
    pickerPolyClear->setMousePattern(QwtPicker::MouseSelect1,Qt::RightButton,Qt::ControlModifier);
    connect(pickerPolyClear,SIGNAL(selected(QPointF)),SLOT(clearPolyGon(QPointF)));
}

void spectrogram::clearPolyGon( QPointF Pos )
{
    pointList.clear();
    boundaryCurve->setSamples(pointList);

    int indexOfArea = this->title.indexOf("  -  s",0);
    if ( indexOfArea != -1) // appending the area string for the first time
        this->title.truncate(indexOfArea);
    setTitle(this->title);

    replot();
}

void spectrogram::selectPointForPolygon( QPointF Pos )
{
    int PosXInt,PosYInt;
    QPointF intPt;
    float area;

    PosXInt = Pos.x();
    PosYInt = Pos.y();

    intPt.setX( PosXInt/*+ scanInfoPtr->scanInterval/2*/ );
    intPt.setY( PosYInt/* + scanInfoPtr->scanInterval/2*/);

    //copy the first point twice to ensure closed polygon
    if (pointList.isEmpty())
    {
        pointList.append(Pos);
        pointList.append(Pos);
    }
    else
    {
        pointList.insert(pointList.size()-1,Pos);
    }

    //pointList.append(Pos);

    qDebug() << "selectPoint - Float("<<Pos.x() <<","<< Pos.y()<<")"
             <<"Int("<< PosXInt << PosYInt <<")"
            <<"Pos.toPoint()"<<Pos
    <<"isClosed ?" <<pointList.isClosed()
    <<"pointListSize"<<pointList.size();

    //populate the curve
    boundaryCurve->setSamples(pointList);
    replot();

    //calculate area
    if (pointList.isClosed())
    {
        area = polyArea();

        QString areaString = "  -  selected area = "+QString::number(area,'f',2) + " mm" + QString(QChar(178));
        int indexOfArea = this->title.indexOf("  -  s",0);
        if ( indexOfArea != -1) // appending the area string for the first time
            this->title.truncate(indexOfArea);
        this->title = this->title + areaString;

        this->setTitle(this->title);
    }
}

float spectrogram::polyArea()
{
    float area = 0;

    for (int i = 0;i<(pointList.size()-1);i++) //last point is repeation of first point.
        area += (pointList[i].x()*pointList[i+1].y()) - (pointList[i+1].x()*pointList[i].y());

   area = scanInfoPtr->scanInterval*scanInfoPtr->scanInterval*area/2;

   return (area<0 ? -1*area : area);
}

void spectrogram::selectPoint( QPointF Pos )
{
    int PosXInt,PosYInt,SelectedImpingePoint;

    int y_length = (scanInfoPtr->scanHeight/scanInfoPtr->scanInterval);
    int x_length = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval);
    QPointF intPt;

   // Pos = Pos*info_Inspection.scanInterval.toFloat();
    PosXInt = Pos.x();
    PosYInt = Pos.y();

    //SelectedImpingePoint = (x_length) * ((y_length - 1)  - PosYInt)  + (PosXInt);
    //SelectedImpingePoint = (ScanWidth-1) * ((ScanHeight-2*(info_Inspection.scanInterval.toFloat())) - PosYInt)  + (PosXInt+1);
    //SelectedImpingePoint = (x_length) * (PosYInt)  + (PosXInt);
    SelectedImpingePoint = (PosYInt)  + (y_length)*(PosXInt);

    if ( PosXInt >= 0 && PosYInt >= 0 && PosXInt < x_length && PosYInt < y_length)
    {
        //emit signal to plot for plotting
        emit pointToPlot(SelectedImpingePoint);
    }

    return;
}

spectrogram::~spectrogram()
{
    delete d_spectrogram;
    //delete data;
    delete zoomer;
    if (imageForMovie!=NULL)
        delete imageForMovie;
}

void spectrogram::updateAxisXY(int enlargeFactor,bool calcSize, bool makePotrait)
{
    container = this->parentWidget();

    int y_length = (scanInfoPtr->scanHeight/scanInfoPtr->scanInterval);
    int x_length = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval);

    data->x_length = x_length;
    data->y_length = y_length;
    data->updateDataAxis();

    //this->setAxisScale(QwtPlot::yLeft,(y_length-0.01),0);
    this->setAxisAutoScale(QwtPlot::yLeft);
    this->axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted,true);

    zoomer->zoom(QRectF::QRectF(0,0,x_length,y_length));
    zoomer->setZoomBase(QRectF::QRectF(0,0,x_length,y_length));
    zoomer->scanInterval = scanInfoPtr->scanInterval;
    zoomer->y_length    =   y_length; // just for debug

    //re-draw scale
    scaleDrawXaxis->scanInterval = scanInfoPtr->scanInterval;
    scaleDrawYaxis->scanInterval = scanInfoPtr->scanInterval;

    scaleDrawYaxis->invalidateCacheLoc();
    scaleDrawXaxis->invalidateCacheLoc();
    axisWidget(yLeft)->update();
    axisWidget(xBottom)->update();

    float HtoWratio = (float)y_length/(float)x_length; //YtoXRatio
    float WtoHratio = (float)x_length/(float)y_length;
    int newWidth    =  width,newHeight=height;
    makePotraitMem  = makePotrait;

    if (enlargeEnabled)
    {
        double enlargeMultiplier = 1.0;
/*
        if      (enlargeFactor   >   0)
            enlargeMultiplier = enlargeFactor+1;
        else if (enlargeFactor  <   0)
            enlargeMultiplier = -1*((double)1/(double)(enlargeFactor-1));

        double enlargeMultiplier = pow(2,enlargeFactor);
*/
        enlargeMultiplier = pow(2,enlargeFactor);
        int inchWidth = (x_length/25.4)*(logicalDpiX()*1.41);
        int inchHeight = (y_length/25.4)*(logicalDpiY()*1.41);

        newWidth    = inchWidth;
        newHeight   = inchHeight;

        newWidth    *=enlargeMultiplier;
        newHeight   *=enlargeMultiplier;

        newWidth    += WIDTHOFYAXISANDCOLOURBARNOTITLE;
        newHeight   += HEIGHTOFBOTTOMXAXIS;

        if (newWidth<6000 && newHeight<6000)
        {
            container->setGeometry(QRect(leftPos,topPos, newWidth, newHeight ));
            container->updateGeometry();
            setGeometry(QRect(leftPos,topPos, newWidth, newHeight));
        }
        else
        {
            QMessageBox msgBox(QMessageBox::Information, tr("Size Exceeds Limits"),tr("This is not supported.Please choose a smaller size."), 0, this);
            msgBox.exec();
        }
    }
    else if (calcSize)
    {
        if (x_length == y_length)
        {
            newHeight = canHeight/(this->noOfSlaves);
            newWidth = newHeight*(WtoHratio) + (WIDTHOFYAXISANDCOLOURBARNOTITLE);
        }
        else
        {
            if (y_length<x_length && HtoWratio < 0.5)
                makePotraitMem = makePotrait = true;
            else
                makePotraitMem = makePotrait = false;

            if (makePotrait)//(y_length<x_length) // one on top of other
            {
                //newHeight = canHeight/(this->noOfSlaves+1) - 2*HEIGHTOFBOTTOMXAXIS; //2*HEIGHTOFBOTTOMXAXIS for title and x axis
                //newWidth = newHeight*(WtoHratio) + (WIDTHOFYAXISANDCOLOURBARNOTITLE);
                newWidth = canWidth-WIDTHOFYAXISANDCOLOURBARNOTITLE;
                newHeight = (newWidth)*(HtoWratio) + (2*HEIGHTOFBOTTOMXAXIS);

                if ((newHeight + DSTBWSPECTPOTRAIT) > (canHeight/(noOfSlaves+1)))
                {
                    newHeight  = canHeight/(this->noOfSlaves+1) - DSTBWSPECTPOTRAIT;
                    newWidth = (newHeight-2*HEIGHTOFBOTTOMXAXIS)*(WtoHratio) + WIDTHOFYAXISANDCOLOURBARNOTITLE;
                }

            }
            else //if (x_length<y_length)
            {
                newHeight = canHeight;
                newWidth = (newHeight-2*(HEIGHTOFBOTTOMXAXIS))*(WtoHratio) + (WIDTHOFYAXISANDCOLOURBARNOTITLE);

                if ((newWidth + DSTBWSPECT) > (canWidth/(noOfSlaves+1)))
                {
                    newWidth  = canWidth/(this->noOfSlaves+1) - DSTBWSPECT;
                    newHeight = (newWidth-WIDTHOFYAXISANDCOLOURBARNOTITLE)*(HtoWratio) + 2*(HEIGHTOFBOTTOMXAXIS);
                }
            }
        }

    }

    setGeometry(QRect(leftPos,topPos, newWidth, newHeight));
    if (noOfSlaves > 0 && !enlargeEnabled)//master sets the container below itself
    {

        if(makePotraitMem)//(newWidth > (canHeight + WIDTHOFYAXISANDCOLOURBARNOTITLE))//slave below the master
        {
            emit placeSlave(leftPos, topPos+newHeight+DSTBWSPECTPOTRAIT,newWidth,newHeight,makePotraitMem);

            container->setGeometry(QRect(leftPos,topPos, newWidth, (noOfSlaves+1)*(newHeight+DSTBWSPECTPOTRAIT)-DSTBWSPECTPOTRAIT ));
            container->updateGeometry();
        }
        else //slave at side of master
        {
            emit placeSlave(leftPos+newWidth+DSTBWSPECT, topPos,newWidth,newHeight,makePotraitMem);

            container->setGeometry(QRect(leftPos,topPos, (noOfSlaves+1)*(newWidth+DSTBWSPECT)-DSTBWSPECT, newHeight ));
            container->updateGeometry();
        }
    }

    this->imgHeight = newHeight;
    this->imgWidth  = newWidth;

    setcolorbar();
    this->updateGeometry();
    this->setTitle(title);
    replot();
    repaint();

    qDebug()<<this->objectName()<<"noOfSlavesArg"<<noOfSlaves<<"spectrogram::updateAxisXY - x_length"<<x_length<<"y_length"<<y_length
           <<"leftPos"<<leftPos
           <<"topPos"<<topPos
           <<"canvasWidth"<<canvas()->width()
           <<"canvasHeight"<<canvas()->height()
          <<"newWidth"<<newWidth
          <<"newHeight"<<newHeight;

}

void spectrogram::placeSlaveslot(int leftPos, int topPos,int widthSetByMaster, int heightSetByMaster,bool makePotrait)
{

    qDebug()<<"spectrogram::placeSlaveslot -"<<this->objectName()<<"leftPos"<<leftPos<<"topPos"<<topPos;
    this->leftPos   = leftPos;
    this->topPos    = topPos;
    this->width     = widthSetByMaster;
    this->height    = heightSetByMaster;

    this->updateAxisXY(false,false,makePotrait);
}


void spectrogram::showSpectrogram( bool on )
{
    d_spectrogram->setDisplayMode( QwtPlotSpectrogram::ImageMode, on );
    d_spectrogram->setDefaultContourPen(on ? QPen( Qt::black, 0 ) : QPen( Qt::NoPen ) );
    replot();
}

/*
void Plot::setAlpha( int alpha )
{
    fnum=alpha;
    d_spectrogram->setData( new mydata(movie_type,fnum,dial) );
    d_spectrogram->attach( this );
    //QTest::qWait(1);
    replot();
}
*/

void spectrogram::setIntensity( int alpha )
{
    data->intensity = alpha;
    data->updateDataAxis();
    setcolorbar();
    plotLayout()->setAlignCanvasToScales( true );
    //d_spectrogram->attach( this ); // not needed
    replot();
}

/*
void Plot::stopper( int stopper )
{
    stop=stopper;
}
*/

// has to come from data processor
/*
void Plot::go_annimation()
{
//    for(fnum =0; fnum < 500 ; fnum++)
//    {
//        char file[35];
//        QwtPlotRenderer renderer;
//        QSizeF t3(13, 10);
//        t3.scale(130, 130, Qt::KeepAspectRatioByExpanding);

//        setAlpha(fnum);
//        QTest::qWait(1);
//        qDebug()<<fnum;
//        sprintf(file, "c:\\mydata\\uwpi%d.bmp", fnum);
//        renderer.renderDocument(this,file,t3,85);
//     }
//    SaveMovie();
    for(fnum =0; fnum < 500 ; fnum++)
    {
        setAlpha(fnum);
        QTest::qWait(1);
    qDebug()<<fnum;
        if (stop==1)
        {stop =0;
            break;}
    }
    fnum =84;
    setAlpha(fnum);
}
*/
void spectrogram::toggleUWPIGreyScale(bool isGreyForUWPI)
{
    this->isGreyForUWPI = isGreyForUWPI;
    setcolorbar();
    replot();
    repaint();
}

void spectrogram::setcolorbar()
{
    const QwtInterval zInterval = d_spectrogram->data()->interval( Qt::ZAxis );
    // A color bar on the right axis
    QwtScaleWidget *rightAxis = axisWidget( QwtPlot::yRight );
    //rightAxis->setTitle( "Intensity Map" );
    rightAxis->setColorBarEnabled( true );

    if (this->isIntData == false && !this->title.contains("Subband - 2"))
    {
        if(isGreyForUWPI)
        {
            rightAxis->setColorMap( zInterval, new ColorMapGrey() );
            d_spectrogram->setColorMap(new ColorMapGrey());
        }
        else
        {
            rightAxis->setColorMap( zInterval, new ColorMap()  );
            d_spectrogram->setColorMap( new ColorMap() );
            //data
        }
    }
    else
    {
        if(isGreyForUWPI)
        {
            rightAxis->setColorMap( zInterval, new ColorMapGrey() );
            d_spectrogram->setColorMap(new ColorMapGrey());
        }
        else
        {

            rightAxis->setColorMap( zInterval, new VTWAMColorMap() );
            d_spectrogram->setColorMap( new VTWAMColorMap());
        }
    }

    setAxisScale( QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue() );
    enableAxis( QwtPlot::yRight );
}


void spectrogram::savePlot(QString outfolderpath)
{
    QFont tempFont;
    float origFontSize[4];
    float origFontSizeTitle;
    QwtText tempText;
    short scaleWindowSize=200;
    float Ratio,fontReductionFactor,wReductionRatio,hAfterScaled,hReductionRatio,wAfterScaled;

    //how much reduction will be done while fitting?
    if (this->imgWidth>this->imgHeight)
    {
        Ratio = this->imgWidth/this->imgHeight;
        wReductionRatio = this->imgWidth/scaleWindowSize;
        hAfterScaled = this->imgHeight/wReductionRatio;

        if (hAfterScaled < 30)
        {
            wReductionRatio = this->imgHeight/30;
            scaleWindowSize = this->imgWidth/wReductionRatio;
        }

        //fontReductionRatio = Ratio*(float)(200.0/(float)scaleWindowSize);

        fontReductionFactor = 1;
        if (Ratio>2 && Ratio < 4)
            fontReductionFactor = 1.5;
        else if (Ratio>4)
            fontReductionFactor = 1.75;

    }
    else
    {
        Ratio = this->imgHeight/this->imgWidth;

        hReductionRatio = this->imgHeight/scaleWindowSize;
        wAfterScaled = this->imgWidth/hReductionRatio;

        if (wAfterScaled < 30)
        {
            hReductionRatio = this->imgWidth/30;
            scaleWindowSize = this->imgHeight/hReductionRatio;
        }

        //fontReductionRatio = Ratio*(float)(200.0/(float)scaleWindowSize);

        fontReductionFactor = 1;
        if (Ratio>2 && Ratio < 4)
            fontReductionFactor = 1.5;
        else if (Ratio>4)
            fontReductionFactor = 1.75;
    }

    /*
    origFontSizeTitle = this->titleLabel()->text().font().pointSizeF();
    tempFont.setPointSize(origFontSizeTitle/reductionRatio+2);

    tempText = this->titleLabel()->text();
    tempText.setFont(tempFont);
    this->titleLabel()->setText(tempText);
*/
    for (int i = 0;i<3;i++)
    {
        origFontSize[i] = this->axisFont(i).pointSizeF();
        tempFont.setPointSize(origFontSize[i]/fontReductionFactor);
        this->setAxisFont(i,tempFont);
    }

    qDebug()<<"spectrogram::savePlot"<<
              "this->imgWidth"<<this->imgWidth<<
              "this->imgHeight"<<this->imgHeight<<
              "origFontSize"<<origFontSize[0]<<
              "reductionRatio"<<Ratio<<
              "New Font Size"<<origFontSize[0]/Ratio+2<<
              "origFontSizeTitle"<<origFontSizeTitle;



    QSizeF t3(this->imgWidth,this->imgHeight);
    t3.scale(scaleWindowSize, scaleWindowSize,Qt::KeepAspectRatio);

    renderer.renderDocument(this,outfolderpath+".jpeg","jpeg",t3,500);
    //renderer.renderDocument(this,outfolderpath+".bmp","bmp",t3,500); // bigger file but better results

    /*
    tempFont.setPointSize(origFontSizeTitle);
    //tempText = this->titleLabel()->text();
    tempText.setFont(tempFont);
    this->titleLabel()->setText(tempText);
   */

    for (int i = 0;i<3;i++)
    {
        tempFont.setPointSize(origFontSize[i]);
        this->setAxisFont(i,tempFont);
    }
}

void spectrogram::setMovieImageSize()
{
    QFont tempFont;
    float origFontSize[4];
    short scaleWindowSize=100;
    float Ratio,fontReductionFactor,wReductionRatio,hAfterScaled,hReductionRatio,wAfterScaled;

    const double mmToInch = 1.0 / 25.4*1.41;
    const int resolution = 100;
    if (this->scanInfoPtr->scanHeight > 599 ||
        this->scanInfoPtr->scanWidth  > 599)
      scaleWindowSize=200;

    qDebug()<<"spectrogram::setMovieImageSize"<<
              "this->imgWidth"<<this->imgWidth<<
              "this->imgHeight"<<this->imgHeight;

    //how much reduction will be done while fitting?
    if (this->imgWidth>this->imgHeight)
    {
        Ratio = this->imgWidth/this->imgHeight;
        wReductionRatio = this->imgWidth/scaleWindowSize;
        hAfterScaled = this->imgHeight/wReductionRatio;

        if (hAfterScaled < 30)
        {
            wReductionRatio = this->imgHeight/30;
            scaleWindowSize = this->imgWidth/wReductionRatio;
        }

        //fontReductionRatio = Ratio*(float)(200.0/(float)scaleWindowSize);

        fontReductionFactor = 1;
        if (Ratio>2 && Ratio < 4)
            fontReductionFactor = 1.05;
        else if (Ratio>4)
            fontReductionFactor = 1.15;
    }
    else
    {
        Ratio = this->imgHeight/this->imgWidth;

        hReductionRatio = this->imgHeight/scaleWindowSize;
        wAfterScaled = this->imgWidth/hReductionRatio;

        if (wAfterScaled < 30)
        {
            hReductionRatio = this->imgWidth/30;
            scaleWindowSize = this->imgHeight/hReductionRatio;
        }

        //fontReductionRatio = Ratio*(float)(200.0/(float)scaleWindowSize);

        fontReductionFactor = 1;
        if (Ratio>2 && Ratio < 4)
            fontReductionFactor = 1.05;
        else if (Ratio>4)
            fontReductionFactor = 1.15;
    }

    for (int i = 0;i<3;i++)
    {
        origFontSize[i] = this->axisFont(i).pointSizeF();
        tempFont.setPointSize(origFontSize[i]/fontReductionFactor);
        this->setAxisFont(i,tempFont);
    }

    QwtPlotRenderer renderer;
    QSizeF sizeMM(this->imgWidth,this->imgHeight);
    sizeMM.scale(scaleWindowSize, scaleWindowSize,Qt::KeepAspectRatio);

    const QSizeF size = sizeMM * mmToInch * resolution;
    const QRectF documentRect( 0.0, 0.0, size.width(), size.height() );

    imageRect = documentRect.toRect();
    const int dotsPerMeter = qRound( resolution * mmToInch * 1000.0 );

    if (imageForMovie!=NULL)
    {
        delete imageForMovie;
        imageForMovie = NULL;
    }

    imageForMovie = new QImage( imageRect.size(), QImage::Format_ARGB32 );
    imageForMovie->setDotsPerMeterX( dotsPerMeter );
    imageForMovie->setDotsPerMeterY( dotsPerMeter );
    imageForMovie->fill( QColor( Qt::white ).rgb() );

    //QPixmap pixMapCur(imageRect.size());
    pixMapCurptr = new QPixmap (imageRect.size());
    //scaled(imageRect.width(),imageRect.height(),Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
/*
    if (painterForMovie!=NULL)
    {
        delete painterForMovie;
        painterForMovie = NULL;
    }
*/
    //painterForMovie = new QPainter( imageForMovie );

    painterForMovie = new QPainter( pixMapCurptr );

}

QPixmap spectrogram::getPlotPixmap(int frameNum)
{
    renderer.render(this,painterForMovie,imageRect);

    if(frameNum == (SAMPLESPERPOINT-1))
    {
        QFont tempFont;
        for (int i = 0;i<3;i++)
        {
            tempFont.setPointSize(10);
            this->setAxisFont(i,tempFont);
        }
    }

    return(QPixmap::fromImage(pixMapCurptr->toImage().mirrored()));
}

void spectrogram::SaveMovie(QString outfolderpath)
{
    /*
    QString file("demo.avi");
    QByteArray fileArr = file.toUtf8();
    HBITMAP hbm;
    //
    HAVI avi = CreateAvi(fileArr.toStdString().c_str(),100,NULL);
    QList<QPixmap> images;
    for (int frame=0; frame < images.size(); frame++) {
        hbm = qt_pixmapToWinHBITMAP(images.at(frame));
        AddAviFrame(avi,hbm);
    }
    CloseAvi(avi);
    */
}

void spectrogram::updateData(void *framePointer, QString title, bool isIntData)
{

    if (title != NULL)
    {
        QString areaString = NULL;
        int indexOfArea = this->title.indexOf("  -  s");
        if ( indexOfArea != -1) // appending the area string for the first time
            areaString = this->title.remove(0,indexOfArea);

        this->title = title + areaString;
        this->setTitle(this->title);
    }
    else
    {
        this->setTitle(NULL);
    }
    //this->setTitle(this->title);

    if (this->data->isIntData            = isIntData)
        this->data->intframePointer      = (int*)framePointer;
    else
        this->data->shortframePointer    = (short*)framePointer;

    if (this->zoomer->isIntData          = isIntData)
        this->zoomer->intframePointer    = (int*)framePointer;
    else

        this->zoomer->shortframePointer  = (short*)framePointer;

    this->isIntData = isIntData;

    setcolorbar(); //isIntData means VTWAM result

    replot();
    //repaint();
}

/*
 * x_length 101 y_length 101 canvasWidth 400 396 canvasHeight 360 356 //colourBar
x_length 101 y_length 101 canvasWidth 426 canvasHeight 360 // no titleforColourBar
x_length 101 y_length 101 canvasWidth 510 506 canvasHeight 360 356 //noColourBar
*/
