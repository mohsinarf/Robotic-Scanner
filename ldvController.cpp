#include "ldvController.h"

#define ECHO_WAIT 500
//make sure to match the baudrate of the LDV controller when trying to communicate



ldvController::ldvController(structDaq *ldvInfoPtrArg, QString portName)
{
    serial          = new QSerialPort;
    ldvInfoPtr      = ldvInfoPtrArg;
    baudRate        = 115200;
    veloDecNumAux   = 0;
    veloDecNum      = 1;
    LdvComPort      = "COM6";

    sigLevelTimer = new QTimer(this);
    connect(sigLevelTimer, SIGNAL(timeout()),SLOT(getSignalLevel()));
}

ldvController::~ldvController()
{
    //LockOut local display
    //QString LLOOnCmd = "Set,Controller,0,Remote,Local\r\n";
    //rwSerialPort(LdvComPort,LLOOnCmd.toLocal8Bit(),ECHO_WAIT,NULL);

    //Turn-off acknowledgement
    QString AckOnCmd = "Set,Interface,0,Acknowledge,Off\r\n";
    rwSerialPort(LdvComPort,AckOnCmd.toLocal8Bit(),ECHO_WAIT,NULL);

    serial->close();
}

void ldvController::initSerialPort()
{
    serial->setBaudRate(this->baudRate);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
}

bool ldvController::rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString* strRsp)
{
    bool SerialPortOpen;
    qint64 WrittenBytes,numRead;
    char response[200] ={'\0'} ;
    int ReadIndex = 0;

    serial->setPortName(portName);
    initSerialPort();
    if(SerialPortOpen = serial->open(QIODevice::ReadWrite))
    {
        WrittenBytes = serial->write(data);

        QTime myTimer;
        myTimer.start();
        //read response
        ReadIndex = 0;
        for (int j = 0; j<20; j ++) response[j] ='\0';
        do
        {
            serial->waitForReadyRead(waitForEcho);
            numRead  = serial->read(&response[ReadIndex], 20);
            ReadIndex += numRead;
        }
        while((waitForEcho!=0) && (response[(ReadIndex-1)]!='\n') && numRead!=0 );

        data.chop(2);

        if( (waitForEcho!=0) && (ReadIndex == 0) )
        {
          // no response from port somethings wrong, quit further commands and try manual command exec
          qDebug().nospace()<<"Command: "<<data<<
                    "       "<<
                    "No Echo from Serial Port" << portName;
          serial->close();
          return false;
        }
        else
        {
            if( (waitForEcho!=0) )
            {
               response[ReadIndex-1] = '\0';
            }
            if(data != "Get,SignalLevel,0,Value")
                qDebug().nospace()<<"Command: "<<data<<"       "<<"\t\t-Response: "<<response<<"\t-Response Time(ms): "<<myTimer.elapsed();
        }
    }
    else
    {
        qDebug()<<"Could not open PortNumber:"<<portName<<"SerialPortOpen: "<<SerialPortOpen;
        serial->close();
        return false;
    }
    serial->close();
    if (strRsp != NULL)
    {
        strRsp->append(response);
    }
    return true;
}

bool ldvController::connectLdv(QString LdvComPortArg)
{
    bool result = false;
    int i = 0;
    //int baudRateArray[8] = {1200,2400,4800,9600,19200,38400,57600,115200};
    QString strRsp;
    QByteArray contNameCmd = "GetDevInfo,controller,0,Name\r\n";

    this->LdvComPort = LdvComPortArg;

    //serial->setBaudRate(baudRateArray[i]);
    /*
     // try different baud rates
    while ( ((result = rwSerialPort(LdvComPort,contNameCmd,ECHO_WAIT)) == false) && (i<8) )
    {
        qDebug()<<"Failed Baudrate:"<<this->baudRate;
        this->baudRate = baudRateArray[i++];
        qDebug()<<"Try with Baudrate:"<<this->baudRate;
    }
    */

    result = rwSerialPort(LdvComPort,contNameCmd,ECHO_WAIT);

    if (result==true)
    {
        //Turn-On acknowledgement
        QString AckOnCmd = "Set,Interface,0,Acknowledge,On\r\n";
        rwSerialPort(LdvComPort,AckOnCmd.toLocal8Bit(),ECHO_WAIT,&strRsp);

        //LockOut local display
       // QString LLOOnCmd = "Set,Controller,0,Remote,LLO\r\n";
       // rwSerialPort(LdvComPort,LLOOnCmd.toLocal8Bit(),ECHO_WAIT,&strRsp);

        //Set power-up option
        QString pwrUpOptCmd = "Set,Controller,0,PowerUp,Default\r\n";
        rwSerialPort(LdvComPort,pwrUpOptCmd.toLocal8Bit(),ECHO_WAIT,&strRsp);

        //Get page options
        QString pageOptCmd = "GetDevInfo,Display,0,Page\r\n";
        rwSerialPort(LdvComPort,pageOptCmd.toLocal8Bit(),ECHO_WAIT,&strRsp);

        //set settings page
        QString setMainPage = "Set,Display,0,Page,Settings\r\n";
        rwSerialPort(LdvComPort,setMainPage.toLocal8Bit(),1000,&strRsp);

        //start the call back for reading the signal strength
        getSignalLevel();
        sigLevelTimer->start(2000);

        for (i = 0;i<2;i++)
        {
            QString VeloDecCmdString = "Get,VeloDec,"+QString::number(i)+",Name\r\n";
            QByteArray VeloDecCmd = VeloDecCmdString.toLocal8Bit();
            strRsp.clear();
            rwSerialPort(LdvComPort,VeloDecCmd,ECHO_WAIT,&strRsp);

            if (strRsp == "VD-09") //The velocity decoder
                this->veloDecNum = i;
            else if (strRsp == "VD-05 (Aux)")
                this->veloDecNumAux = i;
        }
        //get the velocity decoders and thier respective numbers in the LDV
    }
    return result;
}

bool ldvController::setAutoFocus()
{
    if (!getAutoFocusResults()) // to make sure of the ldv connection
        return false;
    QByteArray autoFocusCmd = "Set,SensorHead,0,AutoFocus,Search\r\n";
    return(rwSerialPort(LdvComPort,autoFocusCmd,ECHO_WAIT));
}
bool ldvController::getAutoFocusResults()
{
    QByteArray autoFocusCmd = "Get,SensorHead,0,AutoFocus\r\n";   
    return(rwSerialPort(LdvComPort,autoFocusCmd,ECHO_WAIT));
}


bool ldvController::getSignalLevel()
{
    bool success = true;
    QString strRsp;
    QByteArray sigLevelCmd = "Get,SignalLevel,0,Value\r\n";
    success = rwSerialPort(LdvComPort,sigLevelCmd,ECHO_WAIT,&strRsp);
    emit ldvSignalLevel(strRsp.toInt());
    return(success);
}

bool ldvController::getControllerRemoteStatus()
{
    QByteArray remStatusCmd = "GetDevInfo,controller,0,Remote\r\n";
    return(rwSerialPort(LdvComPort,remStatusCmd,ECHO_WAIT));
}

bool ldvController::setMainPage()
{
    QString setMainPage = "Set,Display,0,Page,Settings\r\n";
    return(rwSerialPort(LdvComPort,setMainPage.toLocal8Bit(),ECHO_WAIT));
}

bool ldvController::setRange()
{
    bool result;
    QString setRangeCmdS = "Set,VeloDec,"+QString::number(this->veloDecNum)+",Range,"+QString::number(ldvInfoPtr->Range)+"mm/s/V\r\n";
    QString setVelocityPage = "Set,Display,0,Page,VelocityOutput\r\n";

    result = rwSerialPort(LdvComPort,setRangeCmdS.toLocal8Bit(),ECHO_WAIT);
    //rwSerialPort(LdvComPort,setMainPage.toLocal8Bit(),0);
    rwSerialPort(LdvComPort,setVelocityPage.toLocal8Bit(),ECHO_WAIT);

    QTimer::singleShot(6000,this,SLOT(setMainPage()));

    return result;
}

bool ldvController::setRangeAux()
{
    bool result;
    QString setRangeCmdS = "Set,VeloDec,"+QString::number(this->veloDecNumAux)+",Range,"+QString::number(100)+"mm/s/V\r\n";
    QString setVelocityPageAux = "Set,Display,0,Page,AuxiliaryOutput\r\n";

    result = rwSerialPort(LdvComPort,setRangeCmdS.toLocal8Bit(),ECHO_WAIT);
    //rwSerialPort(LdvComPort,setMainPage.toLocal8Bit(),0);
    rwSerialPort(LdvComPort,setVelocityPageAux.toLocal8Bit(),ECHO_WAIT);

    QTimer::singleShot(6000,this,SLOT(setMainPage()));

    return result;
}
