#include "bandpasscontroller.h"

#define ECHO_WAIT 110

bandpassController::bandpassController()
{
    serial = new QSerialPort;
    filterComPort = "COM10";
}

bandpassController::~bandpassController()
{
    serial->close();
}

bool bandpassController::config(short ch,short gain, short highpass, short lowpass)
{

    bool success = false;
#if 1
    QString zeroStrng = "0";
    QString gainStr = zeroStrng.repeated(2-QString::number(gain).length())+QString::number(gain);
    QString highpassStr = zeroStrng.repeated(4-QString::number(highpass).length())+QString::number(highpass);
    QString lowpassStr = zeroStrng.repeated(4-QString::number(lowpass).length())+QString::number(lowpass);

    QString cmd = "!"+QString::number(ch)+"%"+highpassStr+lowpassStr+gainStr+"*";
    QByteArray data = cmd.toUtf8();

    QString response;
    QString responceChk = QString::number(ch)+"%"+highpassStr+lowpassStr+gainStr;

    qDebug()<<data;

    do{
          success = rwSerialPort(filterComPort,data,ECHO_WAIT,&response);
      }
      while(response[0] == 'B');
          qDebug()<<"responceChk "<<responceChk<<"   responce = "<<response;

      if(responceChk.compare(response)!=0)
          success = false;
#endif
      return(success);

}


void bandpassController::initSerialPort()
{
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
}


bool bandpassController::rwSerialPort(QString portName, QByteArray data, int waitForEcho, QString *qsRsp)
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
            numRead  = serial->read(&response[ReadIndex], 100);
            ReadIndex += numRead;
        }
        while(numRead!=0 );

        if(ReadIndex == 0)
        {
          // no response from port somethings wrong, quit further commands and try manual command exec
          qDebug()<<"No Echo from Serial Port" << portName;
          serial->close();
          return false;
        }
        else
        {
            //data.chop(2);
            //response[ReadIndex-1] = response[ReadIndex-2] = '\0';
//            qDebug().nospace()<<"Command: "<<data<<"      "<<"\t\t-Response: "<<response<<"\t-Response Time(ms): "<<myTimer.elapsed();
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
