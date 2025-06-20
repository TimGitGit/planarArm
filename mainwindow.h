#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include <QSerialPort>
#include "Smartbus.h"
#include "bus_serport.h"
using namespace std;
using Smartbus_space::Smartbus;
static bus_serport* serport_with_mainBord;
static Smartbus* bus;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void initUI();
    void initCon();
signals:
public slots:


private:
    Ui::MainWindow *ui;
    Smartbus::RouteList_t route = { 0,0, 0, 0, 0, 0, 0, 0 };

};
#endif // MAINWINDOW_H
