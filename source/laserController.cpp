#include "laserController.h"

#define ECHO_WAIT 110
#define ECHO_WAIT_FOR_CURRENT 6000


laserController::laserController(structScan *laserInfoPtrArg, QString portName)
{
    serial = new QSerialPort;
    initLaserControllerDone = false;

    laserInfoPtr = laserInfoPtrArg;
    LaserComPort= "COM9";

    //if (portName !=NULL)
       //initLaserController();
}

laserController::~laserController()
{
    serial->close();
}


bool laserController::initLaserController()
{
    QByteArray InitComArray[9];
    QString StatusTip;
    int i;
    int waitForEcho = 150;
    qDebug()<<"initLaserController()";
    //Populate Init  Command Array
    InitComArray[0] = "MCH=1\r\n";
    InitComArray[1] = "MCH\r\n";
    InitComArray[2] = "QSW=1\r\n";
    InitComArray[3] = "SHT=0\r\n";
    InitComArray[4] = "EXT=0\r\n";
    InitComArray[5] = "PFL=10\r\n";
    {
        //QString laserPRFCmd = ("PRF="+(QString::number(laserInfoPtr->PRF.toDouble()))+"\r\n");
        QString laserPRFCmd = ("PRF=50\r\n");//always set the PRF to 50, later on the trigger from the stage will take care of PRF
        InitComArray[6] = laserPRFCmd.toLocal8Bit();
    }

    {
        QString laserCurrentCmd = ("IDI="+(QString::number(laserInfoPtr->Current.toDouble()*10))+"\r\n");
        InitComArray[7] = laserCurrentCmd.toLocal8Bit();
    }
    InitComArray[8] = "DIO=1\r\n";

    for (i = 0; i<9; i++)
    {
        waitForEcho =  (i!=5) ? ECHO_WAIT : ECHO_WAIT_FOR_CURRENT;

        if( rwSerialPort(LaserComPort, InitComArray[i],waitForEcho ) == false)
        {
            initLaserControllerDone = false;
            emit updateProgressBar_laserControllerSignal(100);
            return false;
        }
        emit updateProgressBar_laserControllerSignal((i*100)/6);
    }
    serial->close();

    initLaserControllerDone = true;
    return true;
}

void laserController::initSerialPort()
{
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
}

bool laserController::rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString* qsRsp)
{
    bool SerialPortOpen;
    qint64 WrittenBytes,numRead;
    char response[20] ={'\0'} ;
    int ReadIndex = 0;

    serial->setPortName(portName);
    if(SerialPortOpen = serial->open(QIODevice::ReadWrite))
    {
        initSerialPort();
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
        while( (response[(ReadIndex-1)]!='\n') && numRead!=0 );

        if(ReadIndex == 0)
        {
          // no response from port somethings wrong, quit further commands and try manual command exec
          qDebug()<<"No Echo from Serial Port" << portName;
          serial->close();
          return false;
        }
        else
        {
            data.chop(2);
            response[ReadIndex-1] = response[ReadIndex-2] = '\0';
            qDebug().nospace()<<"Command: "<<data<<"      "<<"\t\t-Response: "<<response<<"\t-Response Time(ms): "<<myTimer.elapsed();
            if (qsRsp != NULL)
            {
                qsRsp->clear();
                int i = 0;
                while(response[i]!='\0')
                    qsRsp->append(response[i++]);
            }
        }
    }
    else
    {
        qDebug()<<"Could not open PortNumber:"<<portName<<"SerialPortOpen: "<<SerialPortOpen;
        serial->close();
        return false;
    }
    serial->close();
    return true;
}
bool laserController::srchPortandConnect(QString comPortHint, QString* laserComPortSel)
{
    bool portAccessResult = false;
    this->LaserComPort = comPortHint;

    QByteArray data("?DIO\r\n"); //enquire diode status to see if this is the actual laser controller
    portAccessResult = rwSerialPort(LaserComPort,data,ECHO_WAIT);

    if (portAccessResult == true)
    {
        laserComPortSel->append(this->LaserComPort);
        qDebug()<<"LaserController port found successfully:"<<*laserComPortSel;
        return portAccessResult;
    }
    else
    { //check other ports
        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            if (info.portName() != "COM1" && info.portName() != comPortHint )
            {
                this->LaserComPort = info.portName();
                portAccessResult = rwSerialPort(LaserComPort,data,ECHO_WAIT);
                if (portAccessResult == true)
                {
                    laserComPortSel->append(this->LaserComPort);
                    qDebug()<<"LaserController port found successfully: "<<*laserComPortSel;
                    return portAccessResult;
                }
            }
        }
    }

    qDebug()<<"Laser Controller Port not found";
    return portAccessResult;
}

bool laserController::onEXT()
{
    QByteArray data("EXT=1\r\n");
    return(rwSerialPort(LaserComPort,data,ECHO_WAIT));
}

bool laserController::offEXT()
{
    bool success = false;
    QString response;
    QByteArray data("EXT=0\r\n");
    do{
          success = rwSerialPort(LaserComPort,data,ECHO_WAIT,&response);
      }
      while(response[0] == 'B');
      return(success);
}

bool laserController::onSHT()
{
    bool success = false;
    QString response;
    QByteArray data("SHT=1\r\n");
    do{
        Sleep(50);
        success = rwSerialPort(LaserComPort,data,ECHO_WAIT,&response);
      }
      while(response[0] == 'B');
      return(success);
}

bool laserController::offSHT()
{
    bool success = false;
    QString response;
    QByteArray data("SHT=0\r\n");
    do{
        Sleep(50);
        success = rwSerialPort(LaserComPort,data,ECHO_WAIT,&response);
      }
      while(response[0] == 'B');
      return(success);
}

bool laserController::offDIO()
{
    bool success = false;
    QString response;
    QByteArray data("DIO=0\r\n");
    do{
        Sleep(50);
          success = rwSerialPort(LaserComPort,data,ECHO_WAIT,&response);
      }
      while(response[0] == 'B');

      return(success);
}
bool laserController::onDIO()
{
    QByteArray data("DIO=1\r\n");
    return(rwSerialPort(LaserComPort,data,ECHO_WAIT));
}

bool laserController::offQS()
{
    bool success = false;
    QString response;
    QByteArray data("QSW=0\r\n");

      do{
        Sleep(50);
            success = rwSerialPort(LaserComPort,data,ECHO_WAIT,&response);
        }
        while(response[0] == 'B');

    return(success);
}
bool laserController::onQS()
{
    QByteArray data("QSW=1\r\n");
    return(rwSerialPort(LaserComPort,data,ECHO_WAIT));
}

bool laserController::setPRF()
{
    updateStatusBar_laserControllerSignal("Please wait while setting laser PRF.");
    double d_laserPRF = laserInfoPtr->PRF.toDouble();
    QString PRF = QString::number(d_laserPRF);
    QString laser_PRF = "PRF="+PRF+"\r\n";
//    QString laser_PRF = "PRF=1000\r\n"; //always set to 50 later-on the scan system will dictate the prf.
    QByteArray data = laser_PRF.toLocal8Bit();
    return(rwSerialPort(LaserComPort,data,ECHO_WAIT));
}

bool laserController::setCurrent()
{
    updateStatusBar_laserControllerSignal("Please wait while setting laser current.");
    double d_lasercurrent = laserInfoPtr->Current.toDouble()*10;
    QString current = QString::number(d_lasercurrent);
    QString laser_current = "IDI="+current+"\r\n";
    QByteArray data = laser_current.toLocal8Bit();
    return(rwSerialPort(LaserComPort,data,ECHO_WAIT_FOR_CURRENT));
}

bool laserController::getCurrent(QString *sCurrent)
{
    bool success = false;
    QByteArray data("?IDS\r\n");
    success = rwSerialPort(LaserComPort,data,ECHO_WAIT,sCurrent);
    return(success);
}
/*
void SerialPort_dialog::on_pushBtn_ComSelected_clicked()
{
    QString portName = ui->combo_nameComPort->currentText();

    //portName = this->PortName;

    laser_setRemote(portName);
    laser_onQswitch(portName);
    laser_offEXT(portName);
    laser_setPRF(portName);
    laser_setCurrent(portName);
    laser_onCurrent(portName);

    info_Laser.setPort = portName;

    this->close();
}
void SerialPort_dialog::laser_setRemote(QString portName)
{
    QByteArray data("MCH\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_setLocal(QString portName)
{
    QByteArray data("MCH=1\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_onEXT(QString portName)
{
    QByteArray data("EXT=1\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_offEXT(QString portName)
{
    QByteArray data("EXT=0\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_environmentalTemp(QString portName)
{
    QByteArray data("?TEV\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_diodeTemp(QString portName)
{
    QByteArray data("?TDI\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_laserHeadTemp(QString portName)
{
    QByteArray data("?TLA\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_onCurrent(QString portName)
{
    QByteArray data("DIO=1\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_offCurrent(QString portName)
{
    QByteArray data("DIO=0\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}



void SerialPort_dialog::laser_setZeroCurrent(QString portName)
{
    QString laser_current = "IDI=0\r\n";
    QByteArray data = laser_current.toLocal8Bit();
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_onQswitch(QString portName)
{
    QByteArray data("QSW=1\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}

void SerialPort_dialog::laser_offQswitch(QString portName)
{
    QByteArray data("QSW=0\r\n");
    rw_SerialPort(portName,data);
    serial->close();
}




*/
