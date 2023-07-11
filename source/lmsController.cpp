
#include "lmsController.h"

#define BIT_TO_MM_CONV_CONST_X 1180000
#define BIT_TO_MM_CONV_CONST_Y 1090000

#define ROB_TO_LMS_OFFSET_X 522
//Increasing --> Right - Increasing ROB_TO_LMS_OFFSET_X moves the LMS laser to the right.  -
#define ROB_TO_LMS_OFFSET_Y 317 //Increasing --> Down - Increasing ROB_TO_LMS_OFFSET_Y moves the LMS laser downward.
//#define ROB_TO_LMS_OFFSET_Y 357  //Increasing --> Down - Increasing ROB_TO_LMS_OFFSET_Y moves the LMS laser downward.


#define ROB_TO_LMS_OFFSET_Z 175// //ROB_TO_LMS_OFFSET_Z = robPos.trans.x – lmsLdvDistance , lmsLdvDistance: from center of aperture to LDV front (the very front)
#define Comp_Factor_For_Speed 0 //33 was good
#define Offset_For_XDirection 0
#define Origin_for_3D_Correction_Factor 763   // original value 1048

lmsController::lmsController(structLms *lmsInfoPtrArg, structScan *scanInfoPtrArg)
{
//    scanThread      = new lmsThread(this,(structLms *)&lmsInfoPtrArg,(structScan *)&scanInfoPtrArg);
    lmsInfoPtr      = lmsInfoPtrArg;
    scanInfoPtr     = scanInfoPtrArg;

    lmsInitialize();
}

void lmsController::lmsInitialize()
{
    short ErrorCode;
    double HeadPara[16];
    ErrorCode = init_rtc5_dll();
    stop_execution();                                    // - Stopping any list running on RTC®5 board
    ErrorCode = load_program_file(0);                     // - Calling load_program_file for resetting the board, loading the program file,

//        ErrorCode = load_correction_file("D:/Hasan/LMS/correXionPro104/Cor1data_1Mar_7.ct5", 1, 2);

    if (lmsInfoPtr->loadCorrectionFile==true)
//        ErrorCode = load_correction_file("D:/Hasan/LMS/correXionPro104/June19CorFile/13X13CorrectionFile.ct5", 1, 2);
        ErrorCode = load_correction_file("D:/Mohsin_Data/Pincushion_removal_camera_based_second_study/corFiles/13X13CorrectionFile.ct5", 1, 2);
    else
       ErrorCode = load_correction_file("Cor_1to1.ct5", 1, 2);

    qDebug()<<"PRF = "<<scanInfoPtr->PRF;
    QMessageBox errMsg;

    if (!ErrorCode)
    {
        qDebug()<<"LMS initialized";
        select_cor_table(1,0);          // Scan head A = ON with cor_table #1

        for (int i = 0; i<16; i++)
        {
            HeadPara[i] = get_head_para( 1, i );
            qDebug()<<"HeadPara["<<i<<"]=" <<HeadPara[i];
        }
        calFactorK =  HeadPara[1];
        set_laser_mode(1);              // YAG mode selected
        config_list(-1, 1);             // Configures list memory to list 1 =
        set_matrix(0,1,0,0,1,0);
        set_angle(0,90,0);
        set_standby(0, 0);
        set_scanner_delays(25, 10, 5);
        set_jump_speed(10000.0);
        set_mark_speed(250.0);
        set_laser_control(0);
        qDebug()<<"Get RTC Version"<<get_rtc_version();
    }
    else
    {
        QString err = "Correction file loading error " + QString::number(ErrorCode);
        errMsg.setText(err);
        qDebug()<<err;
    }
    UpdateLmsParStruct();

//    lmsJumpAbs(-371280,263120);
}

void lmsController::UpdateLmsParStruct()
{
    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentX = 0;
    lmsInfoPtr->currentY = 0;
    lmsInfoPtr->currentZ = lmsInfoPtr->SOD.toDouble();
    lmsInfoPtr->currentXabs = (lmsInfoPtr->currentX)*bitFactorX;
    lmsInfoPtr->currentYabs = (lmsInfoPtr->currentY)*bitFactorY;
    updatePos();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "origin x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep<<"SOD"<<lmsInfoPtr->SOD.toDouble();
}

void lmsController::updatePos()
{
    QString returnValString;
    returnValString = QString::number(lmsInfoPtr->currentXabs);
    emit Xpos(returnValString);
    returnValString = QString::number(lmsInfoPtr->currentYabs);
    emit Ypos(returnValString);
    returnValString = QString::number(lmsInfoPtr->currentZ);
    emit Zpos(returnValString);
}

void lmsController::lmsSetPos(float xLoc, float yLoc, float zLoc)
{
    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/zLoc);
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/zLoc);
    int    ModCalFactorK = (-1.2*lmsInfoPtr->SOD.toDouble())+2284.8;
    lmsInfoPtr->currentY = yLoc;
    lmsInfoPtr->currentX = xLoc;
    lmsInfoPtr->currentXabs = xLoc;
    lmsInfoPtr->currentYabs = yLoc;
//    lmsInfoPtr->currentXabs = xLoc * ModCalFactorK;
//    lmsInfoPtr->currentYabs = yLoc * ModCalFactorK;
    lmsAreaLimitCheck();
//lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
        lmsJumpAbs(xLoc,yLoc);
    qDebug() << "xLoc " << xLoc << " y = " << yLoc<<" z = " <<zLoc;
      qDebug() << "xLoc absolute " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<" z = " <<ModCalFactorK;
}

void lmsController::lmsClickToPointPos(structRobTarget robTarget)
{
    float xLoc, yLoc, zLoc;
    double bitFactorX,bitFactorY;
    int Start_point_depth=0;
    xLoc = ROB_TO_LMS_OFFSET_X-robTarget.trans.y;//x
    yLoc = robTarget.trans.z-ROB_TO_LMS_OFFSET_Y;//y
    zLoc = robTarget.trans.x + scanInfoPtr->ldvStandOffDistance;//Z

//    Start_point_depth = scanInfoPtr->scanGrid[0][0].trans.x + scanInfoPtr->ldvStandOffDistance;
//    qDebug()<<"lmsClickToPointPos : Current Point Depth  "<<zLoc<<" Start Point Depth "<<Start_point_depth<<
//              "depth Difference"<<Start_point_depth - zLoc<<
//              " X Mul Factor "<<lastPixelPeriodInfo<<" Y Mul Factor "<<trigCompressInfo;
//  zLoc=1051;
    if(lmsInfoPtr->loadCorrectionFile==true)
    {

//        bitFactorX =((Start_point_depth - zLoc)*1.2 + calFactorK);
//        bitFactorY =((Start_point_depth - zLoc)*1.2 + calFactorK);
//        bitFactorX =((Start_point_depth - zLoc)*lastPixelPeriodInfo + calFactorK);
//        bitFactorY =((Start_point_depth - zLoc)*trigCompressInfo + calFactorK);

        bitFactorX = bitFactorY = calFactorK;
    }
    else
    {
        bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/zLoc);
        bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/zLoc);
    }

    lmsInfoPtr->currentY = yLoc;
    lmsInfoPtr->currentX = xLoc;
    lmsInfoPtr->currentZ = zLoc;

    lmsInfoPtr->currentXabs = (lmsInfoPtr->currentX)*bitFactorX;
    lmsInfoPtr->currentYabs = (lmsInfoPtr->currentY)*bitFactorY;
//    lmsAreaLimitCheck();

    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);

    qDebug() << "lmsClickToPointPos: xLoc " << lmsInfoPtr->currentX << " yLoc = " << lmsInfoPtr->currentY<<" zLoc = " << lmsInfoPtr->currentZ<<"bitFactorX"<<bitFactorX<<
                "bitFactorY"<<bitFactorY;
}

void lmsController::lmsMoveyp()
{
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentY += lmsInfoPtr->laserStep;

    if(lmsInfoPtr->loadCorrectionFile==true)
       lmsInfoPtr->currentYabs += lmsInfoPtr->laserStep * calFactorK;

    else
        lmsInfoPtr->currentYabs += lmsInfoPtr->laserStep;
    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMoveyn()
{
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentY -= lmsInfoPtr->laserStep;

    if(lmsInfoPtr->loadCorrectionFile==true)
       lmsInfoPtr->currentYabs -= lmsInfoPtr->laserStep * calFactorK;
    else
        lmsInfoPtr->currentYabs -= lmsInfoPtr->laserStep;

    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMovexp()
{
    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentX += lmsInfoPtr->laserStep;

    if(lmsInfoPtr->loadCorrectionFile==true)
       lmsInfoPtr->currentXabs += lmsInfoPtr->laserStep * calFactorK;

    else
    lmsInfoPtr->currentXabs += lmsInfoPtr->laserStep;

    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsMovexn()
{
    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
    lmsInfoPtr->currentX -= lmsInfoPtr->laserStep;

    if(lmsInfoPtr->loadCorrectionFile==true)
       lmsInfoPtr->currentXabs -= lmsInfoPtr->laserStep * calFactorK;

    else
    lmsInfoPtr->currentXabs -= lmsInfoPtr->laserStep;

    lmsAreaLimitCheck();
    lmsJumpAbs(lmsInfoPtr->currentXabs,lmsInfoPtr->currentYabs);
    qDebug() << "x = " << lmsInfoPtr->currentXabs << " y = " << lmsInfoPtr->currentYabs<<"step"<<lmsInfoPtr->laserStep;
}

void lmsController::lmsDisplayArea()
{
    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/lmsInfoPtr->SOD.toDouble());

    double xAbs = lmsInfoPtr->currentXabs;
    double xxAbs = 0;
    double yAbs = lmsInfoPtr->currentYabs;
    double yyAbs = 0;

    double bit_scanIntervalX;
    double bit_scanIntervalY;

//    int    ModCalFactorK = (calFactorK + 16) + (lmsInfoPtr->SOD.toDouble() - 904 )  ;
//int    ModCalFactorK = (-1.2*lmsInfoPtr->SOD.toDouble())+2284.8;

    int    ModCalFactorK = calFactorK;

    if(lmsInfoPtr->loadCorrectionFile==true)
    {
         bit_scanIntervalX = scanInfoPtr->scanInterval*ModCalFactorK;
         bit_scanIntervalY = scanInfoPtr->scanInterval*ModCalFactorK;
    }
    else
    {
        bit_scanIntervalX = scanInfoPtr->scanInterval*bitFactorX;
        bit_scanIntervalY = scanInfoPtr->scanInterval*bitFactorY;
    }

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
//            if (i == (scan_Point_W/2) -1)
//               qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
        }
        for(int j = 0; j < (scan_Point_H)/2; j++)
        {
            lmsJumpAbs(xxAbs,yAbs - j*(bit_scanIntervalY*2));
            yyAbs = yAbs - j*(bit_scanIntervalY*2);
            lmsInfoPtr->currentY -= 2;
            updatePos();
//            if (j == (scan_Point_H/2) -1)
////                qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
        }

        for(int k = 0; k < scan_Point_W/2; k++)
        {
            lmsJumpAbs(xxAbs - k*(bit_scanIntervalX*2), yyAbs);
            lmsInfoPtr->currentX -= 2 ;
            updatePos();
//            if (k == (scan_Point_W/2) -1)
//                qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
        }
        for(int l = 0; l < (scan_Point_H)/2; l++)
        {
            lmsJumpAbs(xAbs, yyAbs + l*(bit_scanIntervalY*2));
            lmsInfoPtr->currentY += 2 ;
            updatePos();

//            if (l == (scan_Point_H/2) -1)
//                qDebug() << "x = " << lmsInfoPtr->currentX << " y = " << lmsInfoPtr->currentY;
        }

       QCoreApplication::processEvents();
       lmsInfoPtr->currentX = TempX;
       lmsInfoPtr->currentY = TempY;

    }while(lmsInfoPtr->diplayArea);

    qDebug() << "calfactor"<<calFactorK;
    qDebug() << "Modified calfactor"<<ModCalFactorK;
}

void lmsController::lmsAreaLimitCheck()
{
//    //This checking method is based on the bottom-left corner point of scanner area.
//    double bitFactor = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
//    double limit_xRight = 524287;
//    double limit_xLeft = -(limit_xRight - 1);
//    double limit_yTop = 524287;
//    double limit_yBottom = -(limit_xRight - 1);

//    double bit_scanHeight = bitFactor*scanInfoPtr->scanHeight;
//    double bit_scanWidth = bitFactor*scanInfoPtr->scanWidth;

//    double current_xRight = lmsInfoPtr->currentXabs + bit_scanWidth;
//    double current_xLeft = lmsInfoPtr->currentXabs;
//    double current_yTop = lmsInfoPtr->currentYabs;
//    double current_yBottom = lmsInfoPtr->currentYabs - bit_scanHeight;

//    if (limit_yTop < current_yTop)
//    {
//        QString errTxt = "positive y";
//        lmsScanAreaError(errTxt);
//    }
//    else if (limit_yBottom > current_yBottom)
//    {
//        QString errTxt = "negative y";
//        lmsScanAreaError(errTxt);
//    }

//    if (limit_xLeft > current_xLeft)
//    {
//        QString errTxt = "negative x";
//        lmsScanAreaError(errTxt);
//    }
//    else if (limit_xRight < current_xRight)
//    {
//        QString errTxt = "positive x";
//        lmsScanAreaError(errTxt);
//    }
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
    config_list(1000000, 1000);
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


void lmsController::lmsScan()
{

    double bitFactorX = BIT_TO_MM_CONV_CONST_X*atan(1/lmsInfoPtr->SOD.toDouble());
    double bitFactorY = BIT_TO_MM_CONV_CONST_Y*atan(1/lmsInfoPtr->SOD.toDouble());

    if(lmsInfoPtr->loadCorrectionFile   ==  true)
        bitFactorX = bitFactorY = calFactorK;

    int scLnIdx,ptIdx,lmsPtIdx,trigIdx;
    float curXf,curYf,curZf,nextYf,nextScLnStYf,nextScLnStXf,nextScLnStZf;
    float lmsCurY;
    qDebug()<<" bitFactorX ="<<bitFactorX<<" bitFactorY"<<bitFactorY;
    int TempOddXBits = 0;
    int TempOddYBits = 0;
    int TempEvenXBits = 0;
    int TempEvenYBits = 0;

    int listEndPos;
    int totalPoints = 0;
    int pointsPerLine = 0;
    int pointerPos;

    double dotDistanceY = bitFactorY*scanInfoPtr->scanInterval;
    double dotDistanceVector;
    //int Start_point_depth = scanInfoPtr->scanGrid[0][0].trans.x + scanInfoPtr->ldvStandOffDistance;
    float trigDuration = 1000*this->scanInfoPtr->scanSpeedArr[this->scanInfoPtr->scanSpeedIndex].trigInterval/this->scanInfoPtr->scanSpeedArr[this->scanInfoPtr->scanSpeedIndex].speed;

    float dotPeriod =  ((trigDuration-0.1)/(float)lmsInfoPtr->trigMulFac);//in milliseconds
    // 1000*64*dotPeriod/2 --> dotPeriod is timeperiod in ms & 1 bit equals 1/64us;
    // 32 * dotPeriod(in micro seconds);
    int pixel_period = (const UINT)((500*64)*dotPeriod);
    qDebug()<<"trigDuration"<<trigDuration<<"dotPeriod"<<dotPeriod<<"lmsInfoPtr->trigMulFac"<<lmsInfoPtr->trigMulFac<<"Pixel Period"<<pixel_period;
    lmsClickToPointPos(scanInfoPtr->scanGrid[0][0]);

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
        pointsPerLine=((scanInfoPtr->scanGrid[scLnIdx][0].trans.z-scanInfoPtr->scanGrid[scLnIdx][scanInfoPtr->scLnPtCntArr[scLnIdx]-1].trans.z)/scanInfoPtr->scanInterval);
//      qDebug()<<" Points per Line Mark 1"<<pointsPerLine;
        totalPoints =totalPoints+pointsPerLine;
    }

    listEndPos = totalPoints + ((totalPoints/lmsInfoPtr->trigMulFac)*5) + scanInfoPtr->scLnPtCntArr.length()+2;//+2
    lmsLaserTiming();
    set_start_list(1); // #list start command - list writing has been started
    set_extstartpos_list(2); //#list command
    list_jump_pos(listEndPos); // #list command

    qDebug()<<get_list_space()<<"see input pointer inside of list mark 1 = "<<get_input_pointer()<<"listEndPos"<<listEndPos<<"totalPoints"<<totalPoints; // address is zero at this point
    pointerPos = 2;


    for (scLnIdx=0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++) //upto total scan lines
    {
        if (scLnIdx<scanInfoPtr->scLnPtCntArr.length()-1)
            nextScLnStXf = ROB_TO_LMS_OFFSET_X-scanInfoPtr->scanGrid[scLnIdx+1][0].trans.y; //horizontal jump location after the end of scan line

            if (scLnIdx & 0x1)
            {
               int oddCount_bits = 0;

                if (scLnIdx<scanInfoPtr->scLnPtCntArr.length()-1)
                {
                    nextScLnStYf = (scanInfoPtr->scanGrid[scLnIdx+1][0].trans.z-ROB_TO_LMS_OFFSET_Y);
//                    nextScLnStYf = scanInfoPtr->scanGrid[scLnIdx+1][0].trans.z-ROB_TO_LMS_OFFSET_Y;
                    nextScLnStZf = scanInfoPtr->scanGrid[scLnIdx+1][0].trans.x+scanInfoPtr->ldvStandOffDistance;
                }
                //odd scan line
                for(ptIdx  = scanInfoPtr->scLnPtCntArr[scLnIdx]-1;ptIdx>0;ptIdx--) //upto total points in one scan line
                {
                    curXf = ROB_TO_LMS_OFFSET_X- scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // trans.y = x axis in LMS
                    curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z-ROB_TO_LMS_OFFSET_Y; // trans.z = y axis in LMS
                    curZf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.x + scanInfoPtr->ldvStandOffDistance; // trans x = z axis in LMS
                    nextYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx-1].trans.z-ROB_TO_LMS_OFFSET_Y;

//                    qDebug()<<"Odd Scan Line No "<<scLnIdx<<"Point number is "<<ptIdx<<" Value of Depth "<<curZf;

                    lmsCurY = curYf;
                    lmsPtIdx = 0;

//                    curZf=nextScLnStZf=1051;
                    //bitFactorX =((Start_point_depth - curZf)*lastPixelPeriodInfo + calFactorK);
                    //bitFactorY = ((Start_point_depth - curZf)*trigCompressInfo + calFactorK); //correcting bitfactor for new Z distance
//                    bitFactorX = bitFactorY = calFactorK; // MAC 4-28-2020
                    dotDistanceY = bitFactorY * scanInfoPtr->scanInterval;
                    dotDistanceVector = dotDistanceY; //upward scan
                    jump_abs(curXf*bitFactorX,curYf*bitFactorY);

                    while (lmsCurY < nextYf-scanInfoPtr->scanInterval) // loop until number of points per path length achieved
                    {
                        for (trigIdx  = 0;trigIdx<lmsInfoPtr->trigMulFac;trigIdx++) // loop until number of triggers per path length achieved
                        {
                            lmsCurY = curYf-(lmsPtIdx*scanInfoPtr->scanInterval);
                            lmsPtIdx--;
                            if (trigIdx == 0)
                                set_pixel_line(1, pixel_period,0, dotDistanceVector);// #list command
//                                qDebug()<<" Pixel Period is Marker"<<pixel_period;
                            if (trigIdx < lmsInfoPtr->trigMulFac-1)
                            {
                                set_pixel(400,0);//### change here for duty cycle  // #list command
                                oddCount_bits = oddCount_bits +  dotDistanceY;
                            }
                            else
                            {
                                set_pixel_line(1, pixel_period/8,0, dotDistanceVector);// #list command 64 bits = 1us, 104 minimum allowed value
                                set_pixel(400,0);//### change here for duty cycle  // #list command
                                oddCount_bits = oddCount_bits +  dotDistanceY;
                            }
//                            qDebug()<<"scanGrid = ["<<ptIdx<<"]["<< scLnIdx<<"] = "<<curXf<<","<<curYf<<"nextYf"<<nextYf
//                                   <<"trigIdx"<<trigIdx<<"lmsCurY"<<lmsCurY<<"lmsPtIdx"<<lmsPtIdx;
                        }

                        pointerPos = pointerPos+(lmsInfoPtr->trigMulFac+2+2+1);
//                        bitFactorX =((Start_point_depth - nextScLnStZf)*lastPixelPeriodInfo + calFactorK);
//                        bitFactorY = ((Start_point_depth - curZf)*trigCompressInfo + calFactorK);
//                        bitFactorX = bitFactorY = calFactorK; // MAC 4-28-2020
//                        jump_abs(curXf*bitFactorX,nextYf*bitFactorY);
                        if (ptIdx==1 && lmsCurY >= nextYf-scanInfoPtr->scanInterval)
                        {
                            TempEvenXBits = nextScLnStXf*bitFactorX;
                            TempEvenYBits = nextScLnStYf*bitFactorY;
//
                            qDebug()<<" Scan Line ID: "<<scLnIdx<<  " End bit   = "<<" X BitFactor "<<TempOddXBits<<" Y Bit Factor "<<TempOddYBits + oddCount_bits<<" Odd Line"<<" BitFactorX"<<bitFactorX<<" BitFactorY"<<bitFactorY;
                            qDebug()<<" Scan Line ID: "<<scLnIdx+1<<" Start bit = "<<" X BitFactor "<<nextScLnStXf*bitFactorX<<" Y Bit Factor "<<nextScLnStYf*bitFactorY<<" Even Line";
                            pointerPos++;
                            jump_abs(nextScLnStXf*bitFactorX,nextScLnStYf*bitFactorY);
                        }

                        set_extstartpos_list(pointerPos);// #list command
                        list_jump_pos(listEndPos);// #list command
        //                qDebug()<<"see input pointer inside of list mark 2 = "<<get_input_pointer()<<"pointerPos"<<pointerPos<<"scLnIdx"<<scLnIdx;
                    }
                }
//             qDebug()<<"Odd line number = "<<scLnIdx<<"Total number of bits travelled"<<oddCount_bits;
            }
            else
            {
               int evenCount_bits=0;

                if (scLnIdx<scanInfoPtr->scLnPtCntArr.length()-1)
                {
                    nextScLnStYf = (scanInfoPtr->scanGrid[scLnIdx+1][scanInfoPtr->scLnPtCntArr[scLnIdx+1]-1].trans.z-ROB_TO_LMS_OFFSET_Y); //vertical jump location after the end of scan line
                    nextScLnStZf = scanInfoPtr->scanGrid[scLnIdx+1][scanInfoPtr->scLnPtCntArr[scLnIdx+1]-1].trans.x+scanInfoPtr->ldvStandOffDistance; //Z direction jump location after the end of scan line
                }
                //even scan line
                for(ptIdx  = 0;ptIdx<scanInfoPtr->scLnPtCntArr[scLnIdx]-1;ptIdx++) //upto total points in one scan line
                {
                    curXf = ROB_TO_LMS_OFFSET_X- scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // trans.y = x axis in LMS
                    curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z-ROB_TO_LMS_OFFSET_Y; // trans.z = y axis in LMS
                    curZf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.x + scanInfoPtr->ldvStandOffDistance; // trans x = z axis in LMS
                    nextYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx+1].trans.z-ROB_TO_LMS_OFFSET_Y;

                    lmsCurY = curYf;
                    lmsPtIdx = 0;

//                  qDebug()<<"Even Scan Line No "<<scLnIdx<<"Point number is "<<ptIdx<<" Value of Depth "<<curZf;
//                    curZf=nextScLnStZf=1051;
//                    bitFactorX =((Start_point_depth - curZf)*lastPixelPeriodInfo + calFactorK);
//                    bitFactorY = ((Start_point_depth - curZf)*trigCompressInfo + calFactorK); //correcting bitfactor for new Z distance
//                    bitFactorX = bitFactorY = calFactorK;  // MAC 4-28-2020
                    dotDistanceY = bitFactorY * scanInfoPtr->scanInterval;
                    dotDistanceVector =  -dotDistanceY; //downward scan

                    jump_abs(curXf*bitFactorX,curYf*bitFactorY);

                    while (lmsCurY > nextYf+scanInfoPtr->scanInterval) // loop until number of points per path length achieved
                    {
                        for (trigIdx  = 0;trigIdx<lmsInfoPtr->trigMulFac;trigIdx++) //loop until number of triggers per path length achieved
                        {
                            lmsCurY = curYf-(lmsPtIdx*scanInfoPtr->scanInterval);
                            lmsPtIdx++;
                            if (trigIdx == 0)
                                set_pixel_line(1, pixel_period,0, dotDistanceVector);// #list command

                            if (trigIdx < lmsInfoPtr->trigMulFac-1)
                            {
                                set_pixel(400,0);//### change here for duty cycle  // #list command
                                evenCount_bits = evenCount_bits + dotDistanceY;
                            }
                            else
                            {
                                set_pixel_line(1, pixel_period/8,0, dotDistanceVector);// #list command
                                set_pixel(400,0);//### change here for duty cycle  // #list command
                                evenCount_bits = evenCount_bits + dotDistanceY;
                            }
//                            qDebug()<<"scanGrid = ["<<ptIdx<<"]["<< scLnIdx<<"] = "<<curXf<<","<<curYf<<"nextYf"<<nextYf
//                                   <<"trigIdx"<<trigIdx<<"lmsCurY"<<lmsCurY<<"lmsPtIdx"<<lmsPtIdx;
                        }
                        pointerPos = pointerPos+(lmsInfoPtr->trigMulFac+2+2+1);


//                          bitFactorX = ((Start_point_depth - nextScLnStZf)*lastPixelPeriodInfo + calFactorK);
//                          bitFactorY = ((Start_point_depth - curZf)*trigCompressInfo + calFactorK);
//                            bitFactorX = bitFactorY = calFactorK; // MAC 4-28-2020
//                          jump_abs(curXf*bitFactorX,nextYf*bitFactorY);
                        if (ptIdx == scanInfoPtr->scLnPtCntArr[scLnIdx]-2 && lmsCurY == nextYf+scanInfoPtr->scanInterval)
                        {
                            TempOddXBits = nextScLnStXf*bitFactorX;
                            TempOddYBits = nextScLnStYf*bitFactorY;
//                            qDebug()<<"End of Even Line = "<<" X BitFactor"<<TempOddXBits<<" Y Bit Factor "<<TempOddYBits - evenCount_bits;
                            qDebug()<<" Scan Line ID: "<<scLnIdx<<  " End bit   = "<<" X BitFactor "<<TempEvenXBits<<" Y Bit Factor "<<TempEvenYBits - evenCount_bits<<" Even Line";
                            qDebug()<<" Scan Line ID: "<<scLnIdx+1<<" Start bit = "<<" X BitFactor "<<nextScLnStXf*bitFactorX<<" Y Bit Factor "<<nextScLnStYf*bitFactorY<<" Odd Line";

                            pointerPos++;
                            jump_abs(nextScLnStXf*bitFactorX,nextScLnStYf*bitFactorY);
                        }
                        set_extstartpos_list(pointerPos);// #list command
                        list_jump_pos(listEndPos);// #list command
        //                qDebug()<<"see input pointer inside of list mark 2 = "<<get_input_pointer()<<"pointerPos"<<pointerPos<<"scLnIdx"<<scLnIdx;

                    };
                }
//                         qDebug()<<"Even line number = "<<scLnIdx<<"Total number of bits travelled"<<evenCount_bits;
            }
//            qDebug()<<"nextScLnStXf = "<<nextScLnStXf<<" nextScLnStYf = "<< nextScLnStYf;
        }
//        qDebug()<<"see input pointer inside of list mark 3 = "<<get_input_pointer()<<"scLnIdx"<<scLnIdx<<"pointerPos"<<pointerPos;

    set_end_of_list(); //#list end command
    execute_list(1); //#list execute command

    qDebug()<<get_list_space()<<"set input pointer outside of list "<<get_input_pointer()<<"pointerPos"<<pointerPos;
    qDebug()<<"lmsController::lmsInfo.trigMulFac "<<lmsInfoPtr->trigMulFac ;
    updatePos();
}

void lmsController::printLmsScanGrid()
{
    int scLnIdx,ptIdx;
    float curXf,curYf,curZf;

    int totalPoints = 0;
    int pointsPerLine = 0;

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
//        for(ptIdx  = 0;ptIdx<scanInfoPtr->scLnPtCntArr[scLnIdx];ptIdx++)
        ptIdx  = 0;
        {
            curXf = ROB_TO_LMS_OFFSET_X - scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // x axis in LMS
            curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z-ROB_TO_LMS_OFFSET_Y; // y axis in LMS
            curZf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.x-ROB_TO_LMS_OFFSET_Z+scanInfoPtr->ldvStandOffDistance; // z axis in LMS
            qDebug()<<"scanGrid = ["<<scLnIdx<<"]["<< ptIdx<<"] = "<<curXf<<","<<curYf<<","<<curZf;
        }
        ptIdx=scanInfoPtr->scLnPtCntArr[scLnIdx]-1;
        {
            curXf = ROB_TO_LMS_OFFSET_X - scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.y; // x axis in LMS
            curYf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.z-ROB_TO_LMS_OFFSET_Y; // y axis in LMS
            curZf = scanInfoPtr->scanGrid[scLnIdx][ptIdx].trans.x-ROB_TO_LMS_OFFSET_Z+scanInfoPtr->ldvStandOffDistance; // z axis in LMS
            qDebug()<<"scanGrid = ["<<scLnIdx<<"]["<< ptIdx<<"] = "<<curXf<<","<<curYf<<","<<curZf;
        }
    }

    for(scLnIdx = 0;scLnIdx<scanInfoPtr->scLnPtCntArr.length();scLnIdx++)
    {
        pointsPerLine=((scanInfoPtr->scanGrid[scLnIdx][0].trans.z-scanInfoPtr->scanGrid[scLnIdx][scanInfoPtr->scLnPtCntArr[scLnIdx]-1].trans.z)/scanInfoPtr->scanInterval);
        totalPoints =totalPoints+pointsPerLine;
        qDebug()<<"total LMS triggers at line = ["<<scLnIdx<<"] ="<< pointsPerLine<< "TrigPerLineExp"<<scanInfoPtr->scanLinesASvec[scLnIdx].trigPerLineExp
                  <<"accNumOfTrigs"<<scanInfoPtr->scanLinesASvec[scLnIdx].accNumOfTrigs;

    }
    qDebug()<<"total LMS scan lines = "<<scanInfoPtr->scLnPtCntArr.length();
    qDebug()<<"total LMS triggers in grids = "<<totalPoints<< "accNumOfTrigs"<<scanInfoPtr->scanLinesASvec[scLnIdx-1].accNumOfTrigs;
}

void lmsController::genTrig()
{
//    lmsLaserTiming();
//    set_start_list(1); // #list start command
//    set_pixel_line(1, 640,0, 0);// #list command
//    set_pixel(400,0);//### change here for duty cycle  // #list command
//    set_end_of_list(); //#list end command
//    execute_list(1); //#list execute command
}

void lmsController::getTrigCount()
{
    int count = get_counts();
    qDebug()<<"TrigIn count = "<<count;
}

void lmsController::extStart()
{
   simulate_ext_start_ctrl();
}
void lmsController::LmsDebug()
{
    qDebug()<<" LMS Debug is pressed";

    int dotDist = 10400;
    qDebug()<<" Dot Distance is "<<dotDist;
    set_start_list(1);      // Open the list memory #1
//    jump_abs(-371280,263120);
    set_pixel_line(1, 200000,0, dotDist);

    for(int i=0;i<10;i++)
    set_pixel(400,0);

    set_end_of_list();
    execute_list(1);

//    robotCont->moveToTarget(scanInfoPtr->scanGrid[scLnIdx][ptIdx],false,true);

}

void lmsController::LmssetPixelPeriod(float PixelPeriodArg)
{
    lastPixelPeriodInfo= PixelPeriodArg;
    qDebug()<<" pixel Period INfO"<<lastPixelPeriodInfo;
}
void lmsController::LmssetTrigCompress(float TrigCompressArg)
{
    trigCompressInfo=TrigCompressArg;
    qDebug()<<" Trig Compress INfO"<<trigCompressInfo;
}
void lmsController::initCorFileParameters(int corXaxisRightLimitArg,int corXaxisLeftLimitArg,int corYaxisUpLimitArg,int corXaxisDownLimitArg,int corFileResolutionArg)
{
    xAxisRightLimit = corXaxisRightLimitArg;
    xAxisLeftLimit = corXaxisLeftLimitArg;
    yAxisUpLimit = corYaxisUpLimitArg;
    xAxisDownLimit = corXaxisDownLimitArg;
    CorFileResolution = corFileResolutionArg;

}

void lmsController::GenerateCorFile()
{


}


