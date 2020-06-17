#include "serial.h"

Serial::Serial()
{
    flag=false;
}

bool Serial::getFlag() const
{
    return flag;
}

void Serial::setFlag(bool value)
{
    flag = value;
}

QList<QSerialPortInfo> Serial::getDevices() const
{
    return devices;
}
void Serial::searchDevices()
{
    devices = QSerialPortInfo::availablePorts();
}

void Serial::connecting(QString portName)
{
    setPortName(portName);

    if(this->open(QSerialPort::ReadWrite))
    {
        this->setBaudRate(QSerialPort::Baud9600);
        this->setDataBits(QSerialPort::Data8);
        this->setParity(QSerialPort::EvenParity);
        this->setStopBits(QSerialPort::OneStop);
        this->setFlowControl(QSerialPort::NoFlowControl);

        //Connect
        connect(this, SIGNAL(readyRead()), this, SLOT(readFromPort()));
        flag = true;
    }
    else
    {
        flag =false;
    }

}

void Serial::readFromPort()
{
    while(this->canReadLine())
    {
    char buff[20];
        QString line = this->readLine();
        QString terminator = "#";
        qDebug() << line;
        int pos = line.lastIndexOf(terminator);
        memcpy( &buff, line.left(pos).toStdString().c_str() ,line.left(pos).size());
        for(int i =0; i<line.left(pos).size();i++){
            H_sum +=buff[i];
        }
        H_sum = H_sum %37;
        if(buff[0]=='X'  && H_sum==line.split("#")[1].toInt())
            {
            emit dane(line.split(" ")[1]+ " cm");
            }
        else qDebug() << "Błędna CRC!";
        H_sum=0;
    }
}


