#include "robotController.h"

#define DEBUGLOGS_ROBOT
#define JOG_STEP        10
#define HOSTPORT        6000

const QString HOSTIPVC      = "127.0.0.1";
const QString HOSTIPRC      = "192.168.125.1";

robotController::robotController(structScan * scanInfoPtr_arg)
{
    jogTimer = new QTimer(this);
    connect(jogTimer, SIGNAL(timeout()), this, SLOT(jogSlot()));

    /*
    tcpSocketIrc = new QTcpSocket();
    tcpSocketIrc->connectToHost(HOSTIP, HOSTPORT);
    bool connected = tcpSocketIrc->waitForConnected(3000);
    // return connteced
    // -------- bool connect() function part end ------

    // connected has value that it is connected
    qDebug()<<"robotController::robotController - "<<tcpSocketIrc->state();
    if (!connected)
    {
        qWarning() << "Connection to robot failed";
    }
    else
    {
        sendStrCmd((char *)"10,$");
    }
    */

    //scanStartPos = [0.0];

    homePos.trans.x = -1;

    useVcCont = false;
    robContIp = HOSTIPRC;

    scanInfoPtr = scanInfoPtr_arg;
    posForJogPtr = &posForJog.trans.x;


    if (sendStrCmd((char *)"10,$"))
    {
        homePos = getPos(true);
    }
    else
    {//robot not ON just initilize the homeposition in software.
        homePos.trans.x = 630;
        homePos.trans.y = 500;
        homePos.trans.z = 700;

        homePos.rot.q1 = 0.707106781;
        homePos.rot.q2 = 0;
        homePos.rot.q3 = 0.707106781;
        homePos.rot.q4 = 0;
    }
    home();
    //QTimer::singleShot(500,this,SLOT(uploadScanGrid()));
}

robotController::~robotController()
{
    tcpSocketIrc->close();
    delete jogTimer;
}

// the function faces the LDV upwards so in essence the ldv laser is removed from the image
void robotController::ldvLaserTurnOn(bool turnOn)
{
    static structRobTarget robPosLdvOn;
    structRobTarget robPos;

    if (turnOn == false)
    {
        robPosLdvOn = getPos(false);

         robPos.trans.x = 324;
         robPos.trans.y = 433;
         robPos.trans.z = 430;

         robPos.rot.q1 = 0.68952;
         robPos.rot.q2 = -0.34228;
         robPos.rot.q3 = 0.60409;
         robPos.rot.q4 = 0.20612;

//        robPos.trans.x = 375;
//        robPos.trans.y = 284;
//        robPos.trans.z = 620;

//        robPos.rot.q1 = 0.96289;
//        robPos.rot.q2 = -0.00016;
//        robPos.rot.q3 = 0.26991;
//        robPos.rot.q4 = 0;

//        robPos.rot.q1 = 0.51196;
//        robPos.rot.q2 = 0.45981;
//        robPos.rot.q3 = 0.47695;
//        robPos.rot.q4 = -0.54680;
    }
    else
    {
        robPos = robPosLdvOn;
    }

    robPos.transAfterAngle = robPos.trans;
    moveToTarget(robPos,true,true);
}

bool robotController::setIsVcCont(bool isVcContSlected)
{
    useVcCont = isVcContSlected;
    if(useVcCont)
        robContIp = HOSTIPVC;
    else
        robContIp = HOSTIPRC;

    if (sendStrCmd((char *)"10,$"))
    {
        homePos = getPos(true);
    }
    else
    {//robot not ON just initilize the homeposition in software.
        homePos.trans.x = 570;
        homePos.trans.y = 500;
        homePos.trans.z = 700;

        homePos.rot.q1 = 0.707106781;
        homePos.rot.q2 = 0;
        homePos.rot.q3 = 0.707106781;
        homePos.rot.q4 = 0;
    }

    return true;
}

//same values as in the robot code
#define MAX_SCAN_LINES_PERBUF 500
#define MAX_LINE_LEN 100

bool robotController::sendStrCmd(char *sendString, QByteArray* rspArray)
{
    QByteArray array;
    int firstComa,secComa;
    QByteArray subRspResStr;
    int subRspRes;
    int cnt = 0;
    bool success = false;
    int i =0;

    QElapsedTimer t;
    t.start();
    QTcpSocket *tcpSocketIrcLoc;

    tcpSocketIrcLoc = new QTcpSocket();
    tcpSocketIrcLoc->connectToHost(robContIp, HOSTPORT);
    tcpSocketIrcLoc->waitForConnected(100);

    if (tcpSocketIrcLoc->state() == QAbstractSocket::ConnectedState)
    {
        // Socket Write
        tcpSocketIrcLoc->write(sendString);

        if (sendString[0] == '5' || sendString[1] == '2')
            return true;

        // Sockt Read
        while (!array.contains("$") && cnt < 50)
        {
            tcpSocketIrcLoc->waitForReadyRead(100);
            array += tcpSocketIrcLoc->readAll();
            cnt += 1;
        }

#ifdef DEBUGLOGS_ROBOT
        // Debug Log
        qDebug()<<"";
        qDebug() << "Cmd:" + QString(sendString);
        qDebug() << "Rsp: " + array;
#endif

        //Check if the command was successful
        firstComa   = array.indexOf(",",0);
        secComa     = array.indexOf(",",firstComa+1);
        subRspResStr= array.mid(firstComa+1,1);
        subRspRes   = subRspResStr.toInt();

        if (subRspRes == 0) //error
            success =  false;

        else if (subRspRes == 1)
            success =  true;

        if (rspArray != NULL && success)
        {
            rspArray->clear();
            for (int i=secComa+1;i<array.length()-2;i++)
                rspArray->append(array.at(i));
        }
    }

    //for data upload command. now upload the data
    if (sendString[0] == '4' && success)
    {
        //pack scanLineLen
        //each packet is 1024 byte
        char packet[1024];
        char *data = (char*)scanInfoPtr->scLnPtCntArr.data();

        int pIndex;
        int dLength;
        int dIndex;
        int packetCntDebug = 0;
        int scanLineIndex;
        int scPtIndex;

        dIndex  = 0;
        dLength = scanInfoPtr->scLnPtCntArr.length();
        while (dIndex < dLength)
        {
            pIndex = dIndex%1024;
            packet[pIndex] = data[dIndex];

            if (pIndex == 1023 || dIndex == dLength-1)
            {
                tcpSocketIrcLoc->write(packet,1024);
                packetCntDebug++;
            }

            //qDebug()<<pIndex;

            dIndex++;
        }

        qDebug("UploadScanGrid - scLnPtCntArr transfer done - scanInfoPtr->scLnPtCntArr.length():%d,  packets:%d ",
               scanInfoPtr->scLnPtCntArr.length(),packetCntDebug);

//        i = 0;
//        while  (i< 4000000)
//        {
//            i++;
//            QCoreApplication::processEvents();
//        }

        //Now tranfer the actual scan grid
        //targetPt -> buff#(uchar)+ linenum(ushort),ptnum(uchar)+ pos data (3 floats) + rot data (4 floats)
        //target pt ->32 bytes
        //1 packet -> 32 targetPts -> 1024 bytes

        int packetsToSend = 0;
        int numOfTargetPts = 0;
        int byteIndex = 0;
        char packetData[1024];
        float *debugPointer;
        char buffNum;
        short buffLineNum;
        int validPacketCount;
        packetCntDebug = 0;
        pIndex = 0;

        for (int i =0; i< scanInfoPtr->scLnPtCntArr.length();i++)
            numOfTargetPts += scanInfoPtr->scLnPtCntArr[i];

        //1 packet -> 32 targetPts -> 1024 bytes
        packetsToSend = numOfTargetPts / 32 ;
        packetsToSend += ((numOfTargetPts % 32) == 0)? 0 : 1; // the last not completely filled packet

        for (scanLineIndex = 0;scanLineIndex <scanInfoPtr->scLnPtCntArr.length();scanLineIndex++)
        {
            if (scanInfoPtr->scLnPtCntArr[scanLineIndex]+1 > MAX_LINE_LEN)
            {
                qWarning("Line length:%d greater than robot buffer size: %d",scanInfoPtr->scLnPtCntArr[scanLineIndex],MAX_LINE_LEN);
                return 0 ;
            }

            for (scPtIndex = 0;scPtIndex < scanInfoPtr->scLnPtCntArr[scanLineIndex];scPtIndex++)
            {
                //just for debugging
                //debugPointer = &scanInfoPtr->scanGrid[scanLineIndex][scPtIndex].trans.x;
                //qDebug()<<*debugPointer;

                // RAPID maintains 4 buffers each of MAX_SCAN_LINES_PERBUF line capacity
                //RAPID indexing starts from 1
                buffNum     = (scanLineIndex/MAX_SCAN_LINES_PERBUF)+1;
                buffLineNum = (scanLineIndex%MAX_SCAN_LINES_PERBUF)+1;

                packetData[pIndex++] = buffNum;
                packetData[pIndex++] = (char)buffLineNum&0xff;
                packetData[pIndex++] = (char)(buffLineNum>>8)&0xff;
                packetData[pIndex++] = scPtIndex+1; //should be 1~80

                data = (char*) &scanInfoPtr->scanGrid[scanLineIndex][scPtIndex].transAfterAngle.x;
                //copy a single target pt to the packet , 28 bytes per target pt
                /*
                for (byteIndex = 0; byteIndex < 28;byteIndex++)
                    packetData[pIndex++] = data[byteIndex];
                */
                //need to send transafterAngle from now on.
                //12 bytes of transAfterAngle
                for (byteIndex = 0; byteIndex < 12;byteIndex++)
                    packetData[pIndex++] = data[byteIndex];

                data = (char*) &scanInfoPtr->scanGrid[scanLineIndex][scPtIndex].rot.q1;
                for (byteIndex = 0; byteIndex < 16;byteIndex++)
                    packetData[pIndex++] = data[byteIndex];


                if (pIndex == 1024)//send packet when full
                {
                    tcpSocketIrcLoc->write(packetData,1024);
                    packetCntDebug++;
                    //qDebug()<<packetCntDebug;
                    pIndex = 0;
//                    i=0;
//                    while  (i< 4000000)
//                    {
//                        i++;
//                        QCoreApplication::processEvents();
//                    }

                }
            }
        }

        if (pIndex != 0) // a partial packet exists that needs to be sent
        {
            qDebug("pIndex : %d, spaceForTargets : %d",pIndex, (1024-pIndex)/32);
            //fill the whole packet with values to be written to a place which will not be accessed.
            //just to avoid counting the data members in RAPID code
            while(pIndex<1024)
            {
                buffNum     = 4;
                buffLineNum = MAX_SCAN_LINES_PERBUF+1;

                packetData[pIndex++] = buffNum;
                packetData[pIndex++] = (char)buffLineNum&0xff;
                packetData[pIndex++] = (char)(buffLineNum>>8)&0xff;
                packetData[pIndex++] = 81;
                //copy a single target pt to the packet , 28 bytes per target pt
                for (byteIndex = 0; byteIndex < 28;byteIndex++)
                    packetData[pIndex++] = 1;

            }
            tcpSocketIrcLoc->write(packetData,1024);
            packetCntDebug++;

            pIndex = 0;
        }

        qDebug("UploadScanGrid - ScanGrid trasferred, Total TargetPts: %d, Required Packets :%d, Sent Packets: %d ",
               numOfTargetPts,packetsToSend,packetCntDebug);


//        while  (i< 4000000)
//        {
//            i++;
//            QCoreApplication::processEvents();
//        }
    }

    tcpSocketIrcLoc->waitForDisconnected(30000);
    tcpSocketIrcLoc->close();

    qDebug()<<"The command and response took (ms):"<<t.elapsed();
    return success;
}

bool robotController::uploadScanGrid()
{
    char cmdStr[80];
    structRobTarget currentRobPos;
    bool success;
    int totScLns;

    /*
    //###########just for testing should be set from appropriate places################
    scanInfoPtr->scanSpeedIndex = 0;
    totScLns = 5;

    scanInfoPtr->scLnPtCntArr.clear();
    for (int i=0;i<totScLns;i++)
    {
        scanInfoPtr->scLnPtCntArr.append(20+i);
    }
    //##########################################################################
*/
    if (scanInfoPtr->scLnPtCntArr.length() == 0)
        genTestPattern(0);

    totScLns = scanInfoPtr->scLnPtCntArr.length();
    sprintf(cmdStr, "4,%d,%d,$",totScLns,scanInfoPtr->scanSpeedIndex+1); // IRC5 array starting index is 1.

    success = sendStrCmd((char *)cmdStr);

    return true;
}

bool robotController::startScan()
{
    uploadScanGrid();
//    int i = 0;
//    while  (i< 4000000)
//    {
//        i++;
//        QCoreApplication::processEvents();
//    }
//    sendStrCmd((char *)"5,$");
    //sendStrCmd((char *)"5,$");
    return true;
}


void robotController::turnOffPos()
{
    sendStrCmd((char *)"6,$");
}

void robotController::loadMountPos()
{
    sendStrCmd((char *)"7,$");
}

void robotController::bracketAttachPos()
{
    sendStrCmd((char *)"8,$");
}

bool robotController::home()
{
    sendStrCmd((char *)"0,$");
    return true;
}

void robotController::stop()
{
    sendStrCmd((char *)"9,$");
}

void robotController::Jogxp()
{
       jogStart(0,1);
}

void robotController::Jogxn()
{
   jogStart(0,-1);
}

void robotController::Jogyp()
{
      jogStart(1,1);
}

void robotController::Jogyn()
{
   jogStart(1,-1);
}

void robotController::Jogzp()
{
    jogStart(2,1);
}

void robotController::Jogzn()
{
    jogStart(2,-1);
}

// axis --> 0=x,1=y,2=z,direction --> +1/-1
void robotController::jogStart( int axis, int direction)
{
    posForJog       = getPos();
    jogAxis         = axis;
    jogDirection    = direction;

    jogTimer->start(100);

    posForJog.transAfterAngle = posForJog.trans;
}

void robotController::jogSlot()
{
    *(posForJogPtr+jogAxis) += jogDirection*JOG_STEP;
    moveToTarget(posForJog,false);
}

void robotController::jogStop()
{
    jogTimer->stop();
}

void robotController::genTrig()
{
    char cmdStr[80];

    int numOfTrigs = scanInfoPtr->NumOfGenTrig;
    sprintf(cmdStr,"12,%d,$",numOfTrigs);
    sendStrCmd((char *)cmdStr);
}

void robotController::moveToTarget(structRobTarget target, bool updateToolRotation, bool useMoveJ)
{
    //QString cmdStr;
    char cmdStr[80];
    char cmd;
    bool success;

    cmd = useMoveJ ? '2':'1';

    if (updateToolRotation)
    {
        sprintf(cmdStr, "%c,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,$",
                cmd,target.transAfterAngle.x,target.transAfterAngle.y,target.transAfterAngle.z,
                target.rot.q1,target.rot.q2,target.rot.q3,target.rot.q4);
    }
    else
    {
        sprintf(cmdStr, "%c,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,$",
                cmd,target.trans.x,target.trans.y,target.trans.z,
                homePos.rot.q1,homePos.rot.q2,homePos.rot.q3,homePos.rot.q4);
    }

    success = sendStrCmd((char *)cmdStr);
}

structRobTarget robotController::getPos(bool getHomePos)
{
    QByteArray response;
    structRobTarget robPos;
    bool success;
    int firstComa,parCnt;
    float *floatPtr = &robPos.trans.x;

    if (getHomePos)
        success = sendStrCmd((char *)"11,$",&response);
    else
        success = sendStrCmd((char *)"3,$",&response);

    parCnt = 0;
    while (parCnt < 7)
    {
        firstComa         = response.indexOf(",",0);
        //rspVal.trans.x  = response.left(firstComa-1).toFloat();
        *(floatPtr+parCnt)= response.left(firstComa).toFloat();
        response          = response.right(response.length()-firstComa-1);
        parCnt++;
    }

    emit Xpos(QString::number(robPos.trans.x));
    emit Ypos(QString::number(robPos.trans.y));
    emit Zpos(QString::number(robPos.trans.z));

    return robPos;
}

bool robotController::markScanArea()
{

    return true;
}

void robotController::gotoScanPrePos()
{
    /*
    //move stage to original position beacuase the stage doesnot take care of this on its own.
    if(scanInfoPtr->preScanPos.x() != getPosX() || scanInfoPtr->preScanPos.y() != getPosZ())
    {
        setPosX(scanInfoPtr->preScanPos.x());
        setPosZ(scanInfoPtr->preScanPos.y());
        QTimer::singleShot(3000,this,SLOT(gotoScanPrePos())); //try again after 3 secs to set the position
    }
*/
}

void robotController::gotoScanStart()
{
    /*
    setPosX(this->scanStartPosX);
    setPosZ(this->scanStartPosZ);
    */
}

inline structRobTarget robotController::offs(structRobTarget inPos, float xOffset, float yOffset, float zOffset)
{
    structRobTarget outPos;

    outPos.trans.x = inPos.trans.x + xOffset;
    outPos.trans.y = inPos.trans.y + yOffset;
    outPos.trans.z = inPos.trans.z + zOffset;

    return outPos;
}

void robotController::genTestPattern(short testPatternNum)
{
    qDebug("robotController::genTestPattern - testPatternNum:%d ",testPatternNum);
    int i,j,targetIdx;
    int x_length = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval)+1;
    float actualInterval = scanInfoPtr->scanInterval;
    float height = scanInfoPtr->scanHeight;
    structRobTarget currentPos,Home;
    float pathLength;
    int totScLns;
    int sLnLen;

    double offset;
    double len;
    int change = 0;
    int sLnNumIdx;

    QVector<structRobTarget> tempScanLineVector;

    scanInfoPtr->scanLinesASvec.clear();

    scanInfoPtr->scanGrid.clear();
    scanInfoPtr->scLnPtCntArr.clear();

    switch (testPatternNum)
    {
    case 0: // makes a dummy buffer for transmission verification.

        totScLns = 22;
        sLnLen = 15;

        totScLns = scanInfoPtr->scanWidth;
        sLnLen = scanInfoPtr->scanHeight;

        sLnNumIdx = 0;
        while(sLnNumIdx<totScLns)
        {
            //evenline
            tempScanLineVector.clear();

            for (targetIdx = 0; targetIdx<sLnLen; targetIdx++)
            {
                currentPos= {targetIdx,10,20,0.5,0.6,0.7,0.8};
                tempScanLineVector.append((currentPos));
            }
            //Add the scan line to the main grid
            scanInfoPtr->scLnPtCntArr.append((tempScanLineVector.length()));
            scanInfoPtr->scanGrid.append(tempScanLineVector);

            sLnNumIdx++;

            //OddLine
            tempScanLineVector.clear();

            for (targetIdx = 0; targetIdx<sLnLen; targetIdx++)
            {
                currentPos= {targetIdx,10.5,20.5,0.9,1.0,1.1,1.2};
                tempScanLineVector.append((currentPos));
            }

            //Add the scan line to the main grid
            scanInfoPtr->scLnPtCntArr.append((tempScanLineVector.length()));
            scanInfoPtr->scanGrid.append(tempScanLineVector);
            sLnNumIdx++;
        }

        break;
    case 1: //a rectangle from height and width;
        pathLength  = scanInfoPtr->scanSpeedArr[scanInfoPtr->scanSpeedIndex].pathLen;
        totScLns    = (scanInfoPtr->scanWidth/scanInfoPtr->scanInterval)+1;
        sLnLen      = scanInfoPtr->scanHeight/pathLength+1;


        Home = currentPos = getPos();

        //scanStartPosY = getPosZ()/1000;
        //scanInfoPtr->startScanPos.setX(getPosX()/1000);
        //scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        sLnNumIdx = 0;
        while(sLnNumIdx<totScLns)
        {
            //evenline
            currentPos  = offs(Home,0,-(sLnNumIdx)*actualInterval,0);
            tempScanLineVector.clear();

            for (targetIdx = 0; targetIdx<sLnLen; targetIdx++)
            {
                tempScanLineVector.append(offs(currentPos,0,0,-targetIdx*pathLength));
                //scanGrid{sLnNumIdx,targetIdx} := offs(currentPos,0,0,-targetIdx*pathLength);
            }

            //Add the scan line to the main grid
            scanInfoPtr->scLnPtCntArr.append((tempScanLineVector.length()));
            scanInfoPtr->scanGrid.append(tempScanLineVector);

            sLnNumIdx++;
            //OddLine
            currentPos = offs(Home,0,-(sLnNumIdx)*actualInterval,-height);
            tempScanLineVector.clear();

            for (targetIdx = 0; targetIdx<sLnLen; targetIdx++)
            {
                tempScanLineVector.append(offs(currentPos,0,0,targetIdx*pathLength));
                //scanGrid{sLnNumIdx,targetIdx} := offs(currentPos,0,0,targetIdx*pathLength);
            }

            //Add the scan line to the main grid
            scanInfoPtr->scLnPtCntArr.append((tempScanLineVector.length()));
            scanInfoPtr->scanGrid.append(tempScanLineVector);

            sLnNumIdx++;
        }

        break;
 /*
    case 2: //50mm offset
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY,scanInfoPtr->scanHeight});
            else
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+scanInfoPtr->scanHeight,scanInfoPtr->scanHeight});
        }
        break;

    case 3: //50mm offset
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        len = scanInfoPtr->scanHeight;

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY,len});
            else
            {
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+len,len});
                len = len-20;
                if (len < 50)
                    len = 50;
            }
        }
        break;

    case 4: //50mm offset
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        len = scanInfoPtr->scanHeight;

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY,len});
            else
            {
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+len,len});
                len = len+20;
                if (len > 200)
                    len = 200;
            }
        }
        break;

    case 5: //50mm offset
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        len = scanInfoPtr->scanHeight;

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY,len});
            else
            {
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+len,len});
                if (len > 50)
                {
                    len = len - 40;
                    scanStartPosY =  scanStartPosY+ 20;
                }
            }
        }
        break;

    case 6: //50mm offset
        j=0;
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        len = scanInfoPtr->scanHeight;

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY,len});
            else
            {
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+len,len});

                if (j%2 == 0)
                    len = len/2;
                else
                    len = len*2;
                j++;
            }
        }
        break;

    case 7: //50mm offset
        j=0;
        offset = 50;
        scanStartPosY = getPosZ()/1000;
        scanInfoPtr->startScanPos.setX(getPosX()/1000+offset);
        scanInfoPtr->startScanPos.setY(getPosZ()/1000);

        len = scanInfoPtr->scanHeight;

        for(i = 0;i < x_length;i++)
        {
            if (i%2==0)
                scanInfoPtr->scanLinesASvec.append({scanStartPosY-change,len+(2*change)});
            else
            {
                scanInfoPtr->scanLinesASvec.append({scanStartPosY+(len-change),(len-(2*change))});
                change = 15;
            }
        }
        break;
    */
    default:
        break;
    }

}

