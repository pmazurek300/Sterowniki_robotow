#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QList>
#include <QSerialPortInfo>
#include <QSerialPort>

#include <serial.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
   void searchingSignal();
   void connectingSignal(QString portName);

private slots:
   void on_szukaj_clicked();

   void on_polacz_clicked();

   void wyswietl_odleglosc(QString odleglosc);
   void on_przod_p_pressed();

   void on_przod_p_released();

   void on_lewo_p_pressed();

   void on_lewo_p_released();

   void on_prawo_p_pressed();

   void on_prawo_p_released();

   void on_tyl_p_pressed();

   void on_tyl_p_released();

   void sendMessageToDevice(QString message);
   void on_lewo_m_pressed();

   void on_lewo_m_released();

   void on_prawo_m_pressed();

   void on_prawo_m_released();

   void on_podnies_1_pressed();

   void on_podnies_1_released();

   void on_podnies_2_pressed();

   void on_podnies_2_released();

   void on_opusc_2_pressed();

   void on_opusc_2_released();

   void on_zamknij_pressed();

   void on_zamknij_released();

   void on_otworz_pressed();

   void on_otworz_released();

   void on_opusc_1_pressed();

   void on_opusc_1_released();

   void on_zeruj_pressed();

   void on_zeruj_released();

   void on_wyslij_fab_pressed();

   void on_wyslij_fab_released();

private:
    Ui::MainWindow *ui;
    Serial device;
};
#endif // MAINWINDOW_H
