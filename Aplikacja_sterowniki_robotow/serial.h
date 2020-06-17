#ifndef SERIAL_H
#define SERIAL_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>

class Serial : public QSerialPort
{
    Q_OBJECT

public:
    explicit Serial();

    QList<QSerialPortInfo> getDevices() const;

    bool getFlag() const;
    void setFlag(bool value);

private slots:
    void searchDevices();
    void connecting(QString portName);
    void readFromPort();

signals:
     void dane(QString dystans);

private:
    QList<QSerialPortInfo> devices;
    bool flag;
    int H_sum=0;
};

#endif // SERIAL_H
