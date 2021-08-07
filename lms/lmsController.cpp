#include "lmsController.h"

#define TRIG_DURATION 10 // ms

lmsController::lmsController(structLms *lmsInfoPtrArg, structScan *scanInfoPtrArg, long **posBundleArg)
{
//    scanThread      = new lmsThread(this,(structLms *)&lmsInfoPtrArg,(structScan *)&scanInfoPtrArg);
    lmsInfoPtr      = lmsInfoPtrArg;
    scanInfoPtr     = scanInfoPtrArg;
    posBundle       = posBundleArg;
}

void lmsController::lmsInitialize()
{
    short ErrorCode;
    ErrorCode = init_rtc5_dll();
    stop_execution();                                    // - Stopping any list running on RTC®5 board
    ErrorCode = load_program_file(0);                     // - Calling load_program_file for resetting the board, loading the program file,
    ErrorCode = load_correction_file("Cor_1to1.ct5", 1, 2);
    config_list(-1, 1);                          // Configures list memory to list 1 =
    set_matrix(0,1,0,0,1,0); //SMART HANGAR
    set_angle(0,90,0);         //SMART HANGAR

    QMessageBox errMsg;

    if (!ErrorCode)
    {
        qDebug()<<"LMS initialized";
        set_laser_mode(1);              // YAG mode selected
        select_cor_table(1,0);          // Scan head A = ON with cor_table #1
        set_standby(0, 0);
        set_scanner_delays(25, 10, 5);
        set_jump_speed(10000.0);
        set_mark_speed(250.0);
        set_laser_control(0);
    }
    else
    {
        QString err = "Correction file loading error " + QString::number(ErrorCode);
        errMsg.setText(err);
        qDebug()<<err;
    }
    UpdateLmsParStruct();
}

void lmsController::UpdateLmsParStruct()
{
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());//1137282
//    double Height = 1048576/bitFactor;  // 1048576 is maximum length of jump.
//    double Width  = 1048576/bitFactor;

//    lmsInfoPtr->currentX = (int) Height/2;
//    lmsInfoPtr->currentY = (int) Width/2;
//    lmsInfoPtr->currentXabs = -524286 + (lmsInfoPtr->currentX)*bitFactor;
//    lmsInfoPtr->currentYabs = -524286 + (lmsInfoPtr->currentY)*bitFactor;
//    updatePos();
//    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);


    lmsInfoPtr->currentX = 0;
    lmsInfoPtr->currentY = 0;
    lmsInfoPtr->currentXabs = (lmsInfoPtr->currentX)*bitFactor;
    lmsInfoPtr->currentYabs = (lmsInfoPtr->currentY)*bitFactor;
    updatePos();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "origin x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::updatePos()
{
    QString returnValString;
    returnValString = QString::number(lmsInfoPtr->currentX);
    emit Xpos(returnValString);
    returnValString = QString::number(lmsInfoPtr->currentY);
    emit Ypos(returnValString);
}

void lmsController::lmsSetPos(int xLoc, int yLoc)
{
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentY = yLoc;
    lmsInfoPtr->currentX = xLoc;
    lmsInfoPtr->currentXabs = (lmsInfoPtr->currentX)*bitFactor;
    lmsInfoPtr->currentYabs = (lmsInfoPtr->currentY)*bitFactor;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);

        qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
}

void lmsController::lmsMoveyp()
{
    double bitFactorY = 1197282*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentY += lmsInfoPtr->laserStep;
    lmsInfoPtr->currentYabs += lmsInfoPtr->laserStep * bitFactorY;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMoveyn()
{
    double bitFactorY = 1197282*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentY -= lmsInfoPtr->laserStep;
    lmsInfoPtr->currentYabs -= lmsInfoPtr->laserStep * bitFactorY;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMovexp()
{
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentX += lmsInfoPtr->laserStep;
    lmsInfoPtr->currentXabs += lmsInfoPtr->laserStep * bitFactor;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMovexn()
{
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentX -= lmsInfoPtr->laserStep;
    lmsInfoPtr->currentXabs -= lmsInfoPtr->laserStep * bitFactor;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsDisplayArea()
{
    double bitFactorX = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    double bitFactorY = 1197282*atan(1/lmsInfoPtr->SOD.toDouble());

    double xAbs = lmsInfoPtr->currentXabs;
    double xxAbs = 0;
    double yAbs = lmsInfoPtr->currentYabs;
    double yyAbs = 0;
    double bit_scanIntervalX = scanInfoPtr->scanInterval*bitFactorX;
    double bit_scanIntervalY = scanInfoPtr->scanInterval*bitFactorY;
    int scan_Point_H = (scanInfoPtr->scanHeight/scanInfoPtr->scanInterval)+1;
    int scan_Point_W = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval)+1;
    double TempX = lmsInfoPtr->currentX;
    double TempY = lmsInfoPtr->currentY;

    do{
        for(int i = 0; i < scan_Point_W/2; i++)
        {
            lmsJumpAbs(xAbs + i*(bit_scanIntervalX*2),yAbs);
            lmsInfoPtr->currentX += 2;
            xxAbs = xAbs + i*(bit_scanIntervalX*2);
            updatePos();
        }
        for(int j = 0; j < (scan_Point_H)/2; j++)
        {
            lmsJumpAbs(xxAbs,yAbs - j*(bit_scanIntervalY*2));
            yyAbs = yAbs - j*(bit_scanIntervalY*2);
            lmsInfoPtr->currentY -= 2;
            updatePos();
        }

        for(int k = 0; k < scan_Point_W/2; k++)
        {
            lmsJumpAbs(xxAbs - k*(bit_scanIntervalX*2), yyAbs);
            lmsInfoPtr->currentX -= 2 ;
            updatePos();
        }
        for(int l = 0; l < (scan_Point_H)/2; l++)
        {
            lmsJumpAbs(xAbs, yyAbs + l*(bit_scanIntervalY*2));
            lmsInfoPtr->currentY += 2 ;
            updatePos();
        }

        QCoreApplication::processEvents();
       lmsInfoPtr->currentX = TempX;
       lmsInfoPtr->currentY = TempY;
//qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
    }while(lmsInfoPtr->diplayArea);
}

void lmsController::lmsAreaLimitCheck()
{
    //This checking method is based on the bottom-left corner point of scanner area.
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    double limit_xRight = 8388607;
    double limit_xLeft = -(limit_xRight - 1);
    double limit_yTop = 8388607;
    double limit_yBottom = -(limit_xRight - 1);

    double bit_scanHeight = bitFactor*scanInfoPtr->scanHeight;
    double bit_scanWidth = bitFactor*scanInfoPtr->scanWidth;

    double current_xRight = lmsInfoPtr->currentXabs + bit_scanWidth;
    double current_xLeft = lmsInfoPtr->currentXabs;
    double current_yTop = lmsInfoPtr->currentYabs;
    double current_yBottom = lmsInfoPtr->currentYabs - bit_scanHeight;

    if (limit_yTop < current_yTop)
    {
        QString errTxt = "positive y";
        lmsScanAreaError(errTxt);
    }
    else if (limit_yBottom > current_yBottom)
    {
        QString errTxt = "negative y";
        lmsScanAreaError(errTxt);
    }

    if (limit_xLeft > current_xLeft)
    {
        QString errTxt = "negative x";
        lmsScanAreaError(errTxt);
    }
    else if (limit_xRight < current_xRight)
    {
        QString errTxt = "positive x";
        lmsScanAreaError(errTxt);
    }
}

void lmsController::lmsScanAreaError(QString direction)
{
    QString errMsg = "Over the limit of the " + direction +"-axis!";
    QMessageBox errorMsg;
    errorMsg.critical(0,"Error", errMsg);
    errorMsg.setFixedSize(500,200);
}

void lmsController::lmsJumpAbs(long xAbs, long yAbs)
{
    set_start_list(1);      // Open the list memory #1
    set_jump_speed(500000);
    jump_abs(xAbs,yAbs);
    set_end_of_list();
    execute_list(1);
    updatePos();
}

void lmsController::lmsMarkAbs(long xAbs, long yAbs)
{
//    lmsMarkAbs(xAbs + (bit_scanInterval*scan_Point_W),yAbs);
//    QThread::msleep(100);
//    xxAbs = xAbs + (bit_scanInterval*scan_Point_W);

//    QCoreApplication::processEvents();
//    lmsMarkAbs(xxAbs,yAbs - (bit_scanInterval*scan_Point_H));
//    QThread::msleep(100);
//    yyAbs = yAbs - (bit_scanInterval*scan_Point_H);

//    QCoreApplication::processEvents();
//    lmsMarkAbs(xxAbs - (bit_scanInterval*scan_Point_W), yyAbs);
//    QThread::msleep(100);
//    xxAbs = xAbs - (bit_scanInterval*scan_Point_W);

//    QCoreApplication::processEvents();
//    lmsMarkAbs(xAbs, yyAbs + (bit_scanInterval*scan_Point_H));
//    QThread::msleep(100);
//    yyAbs = yAbs + (bit_scanInterval*scan_Point_H);

    qDebug() << "x = " << xAbs<< " y = " << yAbs;
    set_start_list(1);      // Open the list memory #1
    set_mark_speed(5000);
    mark_abs(xAbs,yAbs);
    set_end_of_list();
    execute_list(1);
    updatePos();
}

void lmsController::lmsLaserTiming()
{
    const UINT   LaserHalfPeriod      =        10*64;   //  100 us [1/8 us] must be at least 13
    const UINT   LaserPulseWidth      =        5*64;   //   50 us [1/8 us]
    const UINT   JumpDelay            =       0;   //  250 us [10 us]
    const UINT   MarkDelay            =       100/10;   //  100 us [10 us]
    const UINT   PolygonDelay         =        50/10;   //   50 us [10 us]
    const long   LaserOnDelay         =        1*1;   //  100 us [1 us]
    const UINT   LaserOffDelay        =        1*1;   //  100 us [1 us]
    const double MarkSpeed            =        250.0;   //  [16 Bits/ms]
    const double JumpSpeed            =       799999.0;   //  [16 Bits/ms]
    config_list(-1, 100000);
    set_control_mode(15);
    set_laser_control(0);
    set_default_pixel( 0 );

    qDebug("LMS LASER timing");
    load_list( 1, 0);
    set_laser_pulses( LaserHalfPeriod ,    // half of laser signal period
                     LaserPulseWidth);    // pulse widths of signal LASER1 (Q-switch)

    set_scanner_delays(  JumpDelay,  // jump delay in 25us
                       MarkDelay,  // mark delay in 10us
                       PolygonDelay);  // polygon delay 5us
    set_laser_delays( LaserOnDelay,     // laser on delay in us
                     LaserOffDelay);    // laser off delay in us
    set_jump_speed( JumpSpeed); // jump speed in bits per ms
    set_mark_speed( MarkSpeed);  // mark speed in bits per ms
    set_end_of_list();
    execute_list(1);
    simulate_ext_start_ctrl();
    qDebug()<<"lmsLaserTiming";
}

void lmsController::lmsAlgorithm(long xLoc, long yLoc, unsigned int PIXELS, unsigned int LINES, double DotDist, unsigned int DotFreq)
{
        strucLmsImage graystairs = {
            xLoc, yLoc,
            DotDist,
            DotFreq,
            PIXELS,
            LINES,
        };
        scanInfoPtr->quitScan= false;
        while (!lmsImagePrint(&graystairs, scanInfoPtr->quitScan) )//CHK
        {

        };

        return;
}

int lmsController::lmsImagePrint(strucLmsImage *picture, bool quitScan)
{
    double DRAGDELAY = 120;//360;//120;
    double AXELPERIOD;
    AXELPERIOD = (3*DRAGDELAY);

    static unsigned line = 0;
    static unsigned long pixel_period;
    static long xCounterBalance;

    unsigned i;
    UINT busy;
    UINT position;

    if(quitScan)
    {
        line = 0;

        return(1);
    }

    if(!line)
    {
        if(picture->dotFrequency < 2)
        {
            pixel_period = 0xfffff;
        }
        else
        {
            pixel_period = (const UINT)((500000*64)/picture->dotFrequency);
        }
        xCounterBalance = (long)((double)((AXELPERIOD - DRAGDELAY)/
                          (double)pixel_period/10) * picture->dotDistance);
    }

    get_status(&busy, &position);
    if(busy)
        if(line & 1)
        {
            if(position >= 500000) return(0);
        }
        else
        {
            if(position < 500000) return(0);
        }

    if(line&1)
    {
        set_start_list((unsigned short)((line & 1) + 1));

        jump_abs((long)((picture->xLocus - xCounterBalance)+(picture->dotDistance)*(picture->ppl)),
                 (long)(picture->yLocus - (double)line*picture->dotDistance));

        set_pixel_line(1, pixel_period, -(picture->dotDistance), 0.0);

        //qDebug()<<" pixel period is "<<pixel_period<<" Dotdistance is "<<picture->dotDistance;

        for(i = 0; i < picture->ppl; i++)
        {
            set_pixel(400,0);//### change here for duty cycle
        }

        set_end_of_list();
    }
    else
    {
        set_start_list((unsigned short)((line & 1) + 1));

        jump_abs((long)(picture->xLocus - xCounterBalance),
                (long)(picture->yLocus - (double)line*picture->dotDistance));

        set_pixel_line(1, pixel_period, picture->dotDistance, 0.0);

        for(i = 0; i < picture->ppl; i++)
        {
            set_pixel(400,0);//### change here for duty cycle
        }

        set_end_of_list();
    }

    line?auto_change():execute_list(1);
    line++;
    if(line == picture->lpi)
    {
        line = 0;

        return(1);
    }
    return(0);
}

void lmsController::lmsScan()
{
    qDebug()<<"lmsScan";
    double bitFactor = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    double x = lmsInfoPtr->currentXabs;
    double y = lmsInfoPtr->currentYabs;


    long xLoc = (long)x;
    long yLoc = (long)y;
    double dotDistance = bitFactor*scanInfoPtr->scanInterval;

    int LINES = (int)((scanInfoPtr->scanHeight/scanInfoPtr->scanInterval)+1);
    int PIXELS = (int)((scanInfoPtr->scanWidth/scanInfoPtr->scanInterval)+1);

    int dotFrequency = (int)(1000*scanInfoPtr->PRF.toDouble());
    lmsLaserTiming();

    lmsAlgorithm(xLoc, yLoc, PIXELS, LINES, dotDistance , dotFrequency);

    emit finished();
}

void lmsController::extMode()
{

    double bitFactorX = 1137282*atan(1/lmsInfoPtr->SOD.toDouble());
    double bitFactorY = 1197282*atan(1/lmsInfoPtr->SOD.toDouble());

    int scLnIdx,ptIdx,lmsPtIdx,trigIdx;
    float curXf,curYf,curZf,nextYf,nextScLnStYf,nextScLnStXf,nextScLnStZf;
    float lmsCurY;

    int listEndPos;
    int totalPoints = 0;
    int pointerPos;

    double dotDistanceY = bitFactorY*scanInfoPtr->scanInterval;
    double dotDistanceVector;

    float dotPeriod =  (TRIG_DURATION/(float)lmsInfoPtr->trigMulFac);
    int pixel_period = (const UINT)((500*64)*dotPeriod);

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length()-1;scLnIdx++)
    {
        totalPoints =totalPoints+((scanInfoPtr->scanGrid[scLnIdx][0].trans.z-scanInfoPtr->scanGrid[scLnIdx][scanInfoPtr->scLnPtCntArr[scLnIdx]-1].trans.z)/scanInfoPtr->scanInterval);
    }
    listEndPos = totalPoints + ((totalPoints/lmsInfoPtr->trigMulFac)*4) + 2 + scanInfoPtr->scLnPtCntArr.length()-1;

    lmsLaserTiming();
    set_start_list(1); // #list start command
    set_extstartpos_list(3); //#list command
    list_jump_pos(listEndPos); // #list command

    qDebug()<<get_list_space()<<"see input pointer inside of list mark 1 = "<<get_input_pointer()<<"listEndPos"<<listEndPos<<"totalPoints"<<totalPoints; // address is zero at this point
    pointerPos = 2;

//    scLnIdx=0;
    for (scLnIdx=0;scLnIdx<scanInfoPtr->scLnPtCntArr.length()-1;scLnIdx++) //upto total scan lines
    {
        nextScLnStXf = scanInfoPtr->scanGrid[scLnIdx+1][0].trans.y; //horizontal jump location after the end of scan line

        if (scLnIdx&1)  //odd scan line
        {
        nextScLnStYf = scanInfoPtr->scanGrid[scLnIdx+1][0].trans.z;
        nextScLnStZf = scanInfoPtr->scanGrid[scLnIdx+1][0].trans.x;
        }
        else      //even scan line
        {
        nextScLnStYf = scanInfoPtr->scanGrid[scLnIdx+1][scanInfoPtr->scLnPtCntArr[scLnIdx]-1].trans.z; //vertical jump location after the end of scan line
        nextScLnStZf = scanInfoPtr->scanGrid[scLnIdx+1][scanInfoPtr->scLnPtCntArr[scLnIdx]-1].trans.x; //Z direction jump location after the end of scan line
        }

//        qDebug()<<"see input pointer inside of list mark 3 = "<<get_input_pointer()<<"scLnIdx"<<scLnIdx<<"pointerPos"<<pointerPos;

        for(ptIdx  = 0;ptIdx<scanInfoPtr->scLnPtCntArr[scLnIdx]-1;ptIdx++) //upto total points in one scan line
        {
            curXf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // trans.y = x axis in LMS
            curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z; // trans.z = y axis in LMS
            curZf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.x; // trans x = z axis in LMS
            nextYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx+1].trans.z;

            lmsCurY = curYf;
            lmsPtIdx = 0;

            bitFactorY = 1197282*atan(1/curZf); //correcting bitfactor for new Z distance
            dotDistanceY = bitFactorY*scanInfoPtr->scanInterval; //changing dotDistance for new Z distance

            if (scLnIdx&1)
            dotDistanceVector = -dotDistanceY; //downward scan
            else
            dotDistanceVector =  dotDistanceY; //upward scan

            while (lmsCurY > nextYf+scanInfoPtr->scanInterval) // loop until number of points per path length achieved
            {
                for (trigIdx  = 0;trigIdx<lmsInfoPtr->trigMulFac;trigIdx++) // loop until number of triggers per path length achieved
                {
                    lmsCurY = curYf-(lmsPtIdx*scanInfoPtr->scanInterval);
                    lmsPtIdx++;
                    if (trigIdx == 0)
                    set_pixel_line(1, pixel_period,0, -dotDistanceVector);// #list command

                    if (trigIdx < lmsInfoPtr->trigMulFac-1)
                    set_pixel(400,0);//### change here for duty cycle  // #list command
                    else
                    {
                    set_pixel_line(1, pixel_period/4,0, -dotDistanceVector);// #list command
                    set_pixel(400,0);//### change here for duty cycle  // #list command
                    }
    //                qDebug()<<"scanGrid = ["<<ptIdx<<"]["<< scLnIdx<<"] = "<<curXf<<","<<curYf<<"nextYf"<<nextYf
    //                       <<"trigIdx"<<trigIdx<<"lmsCurY"<<lmsCurY<<"lmsPtIdx"<<lmsPtIdx;

                }

                pointerPos = pointerPos+(lmsInfoPtr->trigMulFac+2+2);
                set_extstartpos_list(pointerPos);// #list command
                list_jump_pos(listEndPos);// #list command

//                qDebug()<<"see input pointer inside of list mark 2 = "<<get_input_pointer()<<"pointerPos"<<pointerPos<<"scLnIdx"<<scLnIdx;

            };
        }
        pointerPos++;
        bitFactorX = 1137282*atan(1/nextScLnStZf);
        bitFactorY = 1197282*atan(1/nextScLnStZf);
        jump_abs(nextScLnStXf*bitFactorX,nextScLnStYf*bitFactorY);
//        qDebug()<<"see input pointer inside of list mark 3 = "<<get_input_pointer()<<"scLnIdx"<<scLnIdx;
    }

    set_end_of_list(); //#list end command
    execute_list(1); //#list execute command

    qDebug()<<get_list_space()<<"set input pointer outside of list "<<get_input_pointer()<<"pointerPos"<<pointerPos;
    updatePos();
}

void lmsController::extTriggerCounter()
{
    int scLnIdx,ptIdx;
    float curXf,curYf;
    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
        for(ptIdx  = 0;ptIdx<scanInfoPtr->scLnPtCntArr[scLnIdx];ptIdx++)
        {
            curXf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // x axis in LMS
            curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z; // y axis in LMS
            qDebug()<<"scanGrid = ["<<scLnIdx<<"]["<< ptIdx<<"] = "<<curXf<<","<<curYf;
        }
    }

}

