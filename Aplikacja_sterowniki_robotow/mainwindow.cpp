#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Aplikacja sterująca manipulatorem mobilnym");
    connect(this, SIGNAL(searchingSignal()), &device, SLOT(searchDevices()));
    connect(this, SIGNAL(connectingSignal(QString)), &device, SLOT(connecting(QString)));
    connect(&device, SIGNAL(dane(QString)), this, SLOT(wyswietl_odleglosc(QString)));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_szukaj_clicked()
{
    emit searchingSignal();

    ui->lista_portow->clear();
    for(int i = 0; i < device.getDevices().length(); i++){
        ui->lista_portow->addItem(device.getDevices().at(i).portName() + "\t" + device.getDevices().at(i).description());
    }
}

void MainWindow::on_polacz_clicked()
{
    if(ui->lista_portow->count() == 0)
    {
        return;
    }
    else
    {
        QString portName = ui->lista_portow->currentText().split("\t").first();
        emit connectingSignal(portName);

        if(device.isOpen() && device.getFlag())
        {
            ui->polacz->setText(tr("Rozłącz"));
            ui->status_polaczenia->setValue(100);
        }
        else if(!device.isOpen() || !device.getFlag())
        {
            ui->polacz->setText(tr("Połącz"));
            ui->status_polaczenia->setValue(0);
            device.close();
        }
    }
}

void MainWindow::sendMessageToDevice(QString message)
{
    if(device.isOpen()) {
        device.write(message.toStdString().c_str());
    }
}

void MainWindow::wyswietl_odleglosc(QString odleglosc){
    ui->odleglosc->setText(odleglosc);
}


void MainWindow::on_przod_p_pressed()
{
    this->sendMessageToDevice("0 xxxxxxxxxxxxx");
}

void MainWindow::on_przod_p_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_lewo_p_pressed()
{
    this->sendMessageToDevice("1 xxxxxxxxxxxxx");
}

void MainWindow::on_lewo_p_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_prawo_p_pressed()
{
    this->sendMessageToDevice("2 xxxxxxxxxxxxx");
}

void MainWindow::on_prawo_p_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_tyl_p_pressed()
{
    this->sendMessageToDevice("3 xxxxxxxxxxxxx");
}

void MainWindow::on_tyl_p_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_lewo_m_pressed()
{
    this->sendMessageToDevice("4 xxxxxxxxxxxxx");
}

void MainWindow::on_lewo_m_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_prawo_m_pressed()
{
    this->sendMessageToDevice("5 xxxxxxxxxxxxx");
}

void MainWindow::on_prawo_m_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_podnies_1_pressed()
{
    this->sendMessageToDevice("6 xxxxxxxxxxxxx");
}

void MainWindow::on_podnies_1_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_podnies_2_pressed()
{
    this->sendMessageToDevice("7 xxxxxxxxxxxxx");
}

void MainWindow::on_podnies_2_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_opusc_2_pressed()
{
    this->sendMessageToDevice("8 xxxxxxxxxxxxx");
}

void MainWindow::on_opusc_2_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_zamknij_pressed()
{
    this->sendMessageToDevice("9 xxxxxxxxxxxxx");
}

void MainWindow::on_zamknij_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_otworz_pressed()
{
    this->sendMessageToDevice("10 xxxxxxxxxxxx");
}

void MainWindow::on_otworz_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

//void MainWindow::on_wyslij_fab_clicked()
//{
//    if(ui->x_fab->text()!=nullptr && ui->y_fab->text()!=nullptr && ui->z_fab->text()!=nullptr){
//        QString temp= "13 "+ui->x_fab->text()+" "+ui->y_fab->text()+" "+ui->z_fab->text()+" ";
//        for(int i = temp.size(); i<15;i++){
//            temp.append("x");
//        }
//        this->sendMessageToDevice(temp);
//        qDebug() << temp;
//    }
//    qDebug() << "ok";
//}

void MainWindow::on_opusc_1_pressed()
{
    this->sendMessageToDevice("11 xxxxxxxxxxxx");
}

void MainWindow::on_opusc_1_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_zeruj_pressed()
{
    this->sendMessageToDevice("12 xxxxxxxxxxxx");
}

void MainWindow::on_zeruj_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}

void MainWindow::on_wyslij_fab_pressed()
{
    if(ui->x_fab->text()!=nullptr && ui->y_fab->text()!=nullptr && ui->z_fab->text()!=nullptr){
        QString temp= "13 "+ui->x_fab->text()+" "+ui->y_fab->text()+" "+ui->z_fab->text()+" ";
        for(int i = temp.size(); i<15;i++){
            temp.append("x");
        }
        this->sendMessageToDevice(temp);
        qDebug() << temp;
    }
    qDebug() << "ok";
}

void MainWindow::on_wyslij_fab_released()
{
    this->sendMessageToDevice("40 xxxxxxxxxxxx");
}
