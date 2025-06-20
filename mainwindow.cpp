#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initUI();
    initCon();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    setWindowIcon(QIcon(":/images/image/logo.png"));

    bus = new Smartbus(1);
    serport_with_mainBord = new bus_serport();

    uint8_t adder[] = { 2,3,5,6,7,8,10 };
    for (auto const id : adder)
    {
        uint8_t quo = id / 32;
        uint32_t* start = &route[quo];
        uint8_t mod = id % 32;
        uint32_t mask = (0x80000000 >> mod);
        *start |= mask;
    }
    ui->buttonPortDis->setEnabled(false);
    ui->buttonPing->setEnabled(false);

    ui->buttonXLMove->setEnabled(false);
    ui->buttonXLReset->setEnabled(false);
    ui->buttonXRMove->setEnabled(false);
    ui->buttonXRReset->setEnabled(false);
    ui->buttonZLMove->setEnabled(false);
    ui->buttonZLReset->setEnabled(false);
    ui->buttonZRMove->setEnabled(false);
    ui->buttonZRReset->setEnabled(false);
    ui->buttonXLStop->setEnabled(false);
    ui->buttonXRStop->setEnabled(false);
    ui->buttonZLStop->setEnabled(false);
    ui->buttonZRStop->setEnabled(false);

    ui->buttonLAll->setEnabled(false);
    ui->buttonRAll->setEnabled(false);
    ui->buttonAll->setEnabled(false);

    ui->comboBox->clear();
    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        if (serial.open(QIODevice::ReadWrite))
        {
            ui->comboBox->addItem(serial.portName());
            serial.close();
        }
    }
}

void MainWindow::initCon()
{
    //连接
    QObject::connect(ui->buttonPortConn, &QPushButton::clicked, [=]() {
        if (ui->comboBox->currentText() != "") {
            serport_with_mainBord->open_new_serport_(ui->comboBox->currentText(), 115200);
            bus->register_interface(serport_with_mainBord, route);

            Smartbus::Bus_transpond_info t;
            t.messenger_id = 2;

            t.chans.push_back(0);
            bus->register_transpond(t);
            bus->open();
            ui->buttonPortConn->setEnabled(false);
            ui->buttonPortDis->setEnabled(true);
            ui->buttonPing->setEnabled(true);
            ui->statusbar->showMessage(QStringLiteral("Serial port connection successful"));

        }
        else {
            ui->statusbar->showMessage(QStringLiteral("Serial port is empty"));
        }
    });
    //断开连接
    QObject::connect(ui->buttonPortDis, &QPushButton::clicked, [=]() {
        bus->close();
        ui->buttonPortConn->setEnabled(true);
        ui->buttonPortDis->setEnabled(false);
        ui->buttonPing->setEnabled(false);

        ui->buttonXLMove->setEnabled(false);
        ui->buttonXLReset->setEnabled(false);
        ui->buttonXRMove->setEnabled(false);
        ui->buttonXRReset->setEnabled(false);
        ui->buttonZLMove->setEnabled(false);
        ui->buttonZLReset->setEnabled(false);
        ui->buttonZRMove->setEnabled(false);
        ui->buttonZRReset->setEnabled(false);
        ui->buttonXLStop->setEnabled(false);
        ui->buttonXRStop->setEnabled(false);
        ui->buttonZLStop->setEnabled(false);
        ui->buttonZRStop->setEnabled(false);
        ui->buttonLAll->setEnabled(false);
        ui->buttonRAll->setEnabled(false);
        ui->buttonAll->setEnabled(false);
        ui->statusbar->showMessage("The equipment has been disconnected");
    });
    //ping
    QObject::connect(ui->buttonPing, &QPushButton::clicked, [=]() {
        QString subID = ui->lineEdit->text();
        bool ok;
        int num = subID.toInt(&ok);
        if(ok){
            //std::thread t([=]() {
                Smartbus::BusReturn bus_rt=	bus->send_request(num,0x00,0x01,nullptr,0,1000);
                if (bus_rt.errorCode == Smartbus::BusErrorCode::BusErrorCode_OK)
                {
                    ui->buttonPortConn->setEnabled(false);
                    ui->buttonPortDis->setEnabled(true);
                    ui->buttonPing->setEnabled(true);

                    ui->buttonXLMove->setEnabled(true);
                    ui->buttonXLReset->setEnabled(true);
                    ui->buttonXRMove->setEnabled(true);
                    ui->buttonXRReset->setEnabled(true);
                    ui->buttonZLMove->setEnabled(true);
                    ui->buttonZLReset->setEnabled(true);
                    ui->buttonZRMove->setEnabled(true);
                    ui->buttonZRReset->setEnabled(true);
                    ui->buttonXLStop->setEnabled(true);
                    ui->buttonXRStop->setEnabled(true);
                    ui->buttonZLStop->setEnabled(true);
                    ui->buttonZRStop->setEnabled(true);
                    ui->buttonLAll->setEnabled(true);
                    ui->buttonRAll->setEnabled(true);
                    ui->buttonAll->setEnabled(true);
                    ui->statusbar->showMessage("Serial port connection successful");
                }else{

                    ui->statusbar->showMessage("Connection communication failed");

                }
            //});
            //t.detach();
        }else{
            ui->statusbar->showMessage("Parameter is incorrect");
        }

    });
    //XL移动
    QObject::connect(ui->buttonXLMove, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 0;
            float position = ui->lineEditXLDistance->text().toFloat();
            float speed = ui->lineEditXLSpeed->text().toFloat();
            float acc = ui->lineEditXLAcc->text().toFloat();
            float dcc = ui->lineEditXLDec->text().toFloat();
            int offset = 0;
            std::vector<uint8_t> sendBuf(17);
            sendBuf[offset] = motor_id;
            offset += 1;
            qToLittleEndian<float>(position, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc, sendBuf.data() + offset);
            offset += sizeof(float);
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8002, sendBuf.data(), static_cast<uint16_t>(sendBuf.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis move failed");
            }
            else {
                ui->statusbar->showMessage("Left X-axis move successful");
            }
            });
        t.detach();
    });
    //XL复位
    QObject::connect(ui->buttonXLReset, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 0;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                 ui->statusbar->showMessage("Left X-axis reset failed");
            }else{
                ui->statusbar->showMessage("Left X-axis reset successful");
            }
        });
        t.detach();
    });
    //XL停止
    QObject::connect(ui->buttonXLStop, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 0;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8020, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis stop failed");
            }
            else {
                ui->statusbar->showMessage("Left X-axis stop successful");
            }
            });
        t.detach();
        });
    //XR移动
    QObject::connect(ui->buttonXRMove, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 2;
            float position = ui->lineEditXRDistance->text().toFloat();
            float speed = ui->lineEditXRSpeed->text().toFloat();
            float acc = ui->lineEditXRAcc->text().toFloat();
            float dcc = ui->lineEditXRDec->text().toFloat();
            int offset = 0;
            std::vector<uint8_t> sendBuf(17);
            sendBuf[offset] = motor_id;
            offset += 1;
            qToLittleEndian<float>(position, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc, sendBuf.data() + offset);
            offset += sizeof(float);
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8002, sendBuf.data(), static_cast<uint16_t>(sendBuf.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis move failed");
            }
            else {
                ui->statusbar->showMessage("Right X-axis move successful");
            }
            });
        t.detach();
    });
    //XR复位
    QObject::connect(ui->buttonXRReset, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 2;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis reset failed");
            }
            else {
                ui->statusbar->showMessage("Right X-axis reset successful");
            }
            });
        t.detach();
    });
    //XR停止
    QObject::connect(ui->buttonXRStop, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 2;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8020, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis stop failed");
            }
            else {
                ui->statusbar->showMessage("Right X-axis stop successful");
            }
            });
        t.detach();
        });
    //ZL移动
    QObject::connect(ui->buttonZLMove, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 1;
            float position = ui->lineEditZLDistance->text().toFloat();
            float speed = ui->lineEditZLSpeed->text().toFloat();
            float acc = ui->lineEditZLAcc->text().toFloat();
            float dcc = ui->lineEditZLDec->text().toFloat();
            int offset = 0;
            std::vector<uint8_t> sendBuf(17);
            sendBuf[offset] = motor_id;
            offset += 1;
            qToLittleEndian<float>(position, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc, sendBuf.data() + offset);
            offset += sizeof(float);
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8002, sendBuf.data(), static_cast<uint16_t>(sendBuf.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis move failed");
            }
            else {
                ui->statusbar->showMessage("Left Z-axis move successful");
            }
            });
        t.detach();
    });
    //ZL复位
    QObject::connect(ui->buttonZLReset, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 1;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis reset failed");
            }
            else {
                ui->statusbar->showMessage("Left Z-axis reset successful");
            }
            });
        t.detach();
        });
    //ZL停止
    QObject::connect(ui->buttonZLStop, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 1;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8020, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis stop failed");
            }
            else {
                ui->statusbar->showMessage("Left Z-axis stop successful");
            }
            });
        t.detach();
        });
    //ZR移动
    QObject::connect(ui->buttonZRMove, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 3;
            float position = ui->lineEditZRDistance->text().toFloat();
            float speed = ui->lineEditZRSpeed->text().toFloat();
            float acc = ui->lineEditZRAcc->text().toFloat();
            float dcc = ui->lineEditZRDec->text().toFloat();
            int offset = 0;
            std::vector<uint8_t> sendBuf(17);
            sendBuf[offset] = motor_id;
            offset += 1;
            qToLittleEndian<float>(position, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc, sendBuf.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc, sendBuf.data() + offset);
            offset += sizeof(float);
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8002, sendBuf.data(), static_cast<uint16_t>(sendBuf.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis move failed");
            }
            else {
                ui->statusbar->showMessage("Right Z-axis move successful");
            }
            });
        t.detach();
    });
    //ZR复位
    QObject::connect(ui->buttonZRReset, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 3;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis reset failed");
            }
            else {
                ui->statusbar->showMessage("Right Z-axis reset successful");
            }
            });
        t.detach();
    });
    //ZR停止
    QObject::connect(ui->buttonZRStop, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            uint8_t motor_id = 3;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0, 8020, &motor_id, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK || bus_rt.data_len != 1 || bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis stop failed");
            }
            else {
                ui->statusbar->showMessage("Right Z-axis stop successful");
            }
            });
        t.detach();
        });
    //示例
    //左臂组示例
    QObject::connect(ui->buttonLAll, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            // 1. Z_Left复位
            uint8_t motor_id_z = 1;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_z, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Left Z-axis reset successful");

            // 2. X_Left复位
            uint8_t motor_id_x = 0;
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_x, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Left X-axis reset successful");


            // 3. X_Left走
            float position_x = 25.0f;
            float speed_x = 100.0f;
            float acc_x = 100.0f;
            float dcc_x = 100.0f;

            std::vector<uint8_t> sendBuf_x(17);
            int offset = 0;
            sendBuf_x[offset] = motor_id_x;
            offset += 1;
            qToLittleEndian<float>(position_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_x, sendBuf_x.data() + offset);

            bus_rt = bus->send_request(2, 0, 8002, sendBuf_x.data(), static_cast<uint16_t>(sendBuf_x.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis move failed");
                return;
            }
            ui->statusbar->showMessage("Left X-axis moved to 40");

            // 4. Z_Left走
            float position_z = 60.0f;
            float speed_z = 300.0f;
            float acc_z = 500.0f;
            float dcc_z = 500.0f;

            std::vector<uint8_t> sendBuf_z(17);
            offset = 0;
            sendBuf_z[offset] = motor_id_z;
            offset += 1;
            qToLittleEndian<float>(position_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_z, sendBuf_z.data() + offset);

            bus_rt = bus->send_request(2, 0, 8002, sendBuf_z.data(), static_cast<uint16_t>(sendBuf_z.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis move failed");
                return;
            }
            ui->statusbar->showMessage("Left Z-axis moved to 50");
            QThread::sleep(5); // 暂停5秒

            // 5. Z_Left复位
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_z, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Left Z-axis reset successful");

            // 6. X_Left复位
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_x, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Left X-axis reset successful");
            ui->statusbar->showMessage("Left arm sequence completed successfully");
            });
        t.detach();
        });
    //右臂组示例
    QObject::connect(ui->buttonRAll, &QPushButton::clicked, [=]() {
        std::thread t([=]() {
            // 1. Z_Right复位
            uint8_t motor_id_z = 3;
            Smartbus::BusReturn bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_z, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Right Z-axis reset successful");

            // 2. X_Right复位
            uint8_t motor_id_x = 2;
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_x, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Right X-axis reset successful");


            // 3. X_Right走
            float position_x = 32.5f;
            float speed_x = 100.0f;
            float acc_x = 100.0f;
            float dcc_x = 100.0f;

            std::vector<uint8_t> sendBuf_x(17);
            int offset = 0;
            sendBuf_x[offset] = motor_id_x;
            offset += 1;
            qToLittleEndian<float>(position_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_x, sendBuf_x.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_x, sendBuf_x.data() + offset);

            bus_rt = bus->send_request(2, 0, 8002, sendBuf_x.data(), static_cast<uint16_t>(sendBuf_x.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis move failed");
                return;
            }
            ui->statusbar->showMessage("Right X-axis moved to 40");

            // 4. Z_Right走
            float position_z = 60.0f;
            float speed_z = 300.0f;
            float acc_z = 500.0f;
            float dcc_z = 500.0f;

            std::vector<uint8_t> sendBuf_z(17);
            offset = 0;
            sendBuf_z[offset] = motor_id_z;
            offset += 1;
            qToLittleEndian<float>(position_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_z, sendBuf_z.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_z, sendBuf_z.data() + offset);

            bus_rt = bus->send_request(2, 0, 8002, sendBuf_z.data(), static_cast<uint16_t>(sendBuf_z.size()), 10000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis move failed");
                return;
            }
            ui->statusbar->showMessage("Right Z-axis moved to 50");
            QThread::sleep(5); // 暂停5秒

            // 5. Z_Right复位
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_z, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Right Z-axis reset successful");

            // 6. X_Right复位
            bus_rt = bus->send_request(2, 0xffff, 20000, &motor_id_x, 1, 20000);
            if (bus_rt.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt.data_len != 1 ||
                bus_rt.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis reset failed");
                return;
            }
            ui->statusbar->showMessage("Right X-axis reset successful");
            ui->statusbar->showMessage("Right arm sequence completed successfully");
            });
        t.detach();
        });
    //共同示例
    QObject::connect(ui->buttonAll, &QPushButton::clicked, [=]() {
        // 启动左臂线程
        std::thread leftThread([=]() {
            // 左臂复位
            uint8_t motor_id_xl = 0;
            uint8_t motor_id_zl = 1;
            Smartbus::BusReturn bus_rt_zl = bus->send_request(2, 0xffff, 20000, &motor_id_zl, 1, 20000);
            Smartbus::BusReturn bus_rt_xl = bus->send_request(2, 0xffff, 20000, &motor_id_xl, 1, 20000);

            if (bus_rt_xl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xl.data_len != 1 ||
                bus_rt_xl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis reset failed");
                return;
            }

            if (bus_rt_zl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zl.data_len != 1 ||
                bus_rt_zl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis reset failed");
                return;
            }

            ui->statusbar->showMessage("Left arm reset successful");

            // 左臂运动
            float position_xl = 25.0f;
            float speed_xl = 100.0f;
            float acc_xl = 100.0f;
            float dcc_xl = 100.0f;

            std::vector<uint8_t> sendBuf_xl(17);
            int offset = 0;
            sendBuf_xl[offset] = motor_id_xl;
            offset += 1;
            qToLittleEndian<float>(position_xl, sendBuf_xl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_xl, sendBuf_xl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_xl, sendBuf_xl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_xl, sendBuf_xl.data() + offset);

            bus_rt_xl = bus->send_request(2, 0, 8002, sendBuf_xl.data(), static_cast<uint16_t>(sendBuf_xl.size()), 10000);
            if (bus_rt_xl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xl.data_len != 1 ||
                bus_rt_xl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis move failed");
                return;
            }

            float position_zl = 60.0f;
            float speed_zl = 300.0f;
            float acc_zl = 500.0f;
            float dcc_zl = 500.0f;

            std::vector<uint8_t> sendBuf_zl(17);
            offset = 0;
            sendBuf_zl[offset] = motor_id_zl;
            offset += 1;
            qToLittleEndian<float>(position_zl, sendBuf_zl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_zl, sendBuf_zl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_zl, sendBuf_zl.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_zl, sendBuf_zl.data() + offset);

            bus_rt_zl = bus->send_request(2, 0, 8002, sendBuf_zl.data(), static_cast<uint16_t>(sendBuf_zl.size()), 10000);
            if (bus_rt_zl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zl.data_len != 1 ||
                bus_rt_zl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis move failed");
                return;
            }

            ui->statusbar->showMessage("Left arm movement completed");
            QThread::sleep(5);

            // 左臂复位
            bus_rt_zl = bus->send_request(2, 0xffff, 20000, &motor_id_zl, 1, 20000);
            bus_rt_xl = bus->send_request(2, 0xffff, 20000, &motor_id_xl, 1, 20000);

            if (bus_rt_zl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zl.data_len != 1 ||
                bus_rt_zl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left Z-axis reset failed");
            }

            if (bus_rt_xl.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xl.data_len != 1 ||
                bus_rt_xl.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Left X-axis reset failed");
            }

            ui->statusbar->showMessage("Left arm reset completed");
            });

        // 启动右臂线程
        std::thread rightThread([=]() {
            // 右臂复位
            uint8_t motor_id_xr = 2;
            uint8_t motor_id_zr = 3;

            Smartbus::BusReturn bus_rt_zr = bus->send_request(2, 0xffff, 20000, &motor_id_zr, 1, 20000);
            Smartbus::BusReturn bus_rt_xr = bus->send_request(2, 0xffff, 20000, &motor_id_xr, 1, 20000);

            if (bus_rt_xr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xr.data_len != 1 ||
                bus_rt_xr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis reset failed");
                return;
            }

            if (bus_rt_zr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zr.data_len != 1 ||
                bus_rt_zr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis reset failed");
                return;
            }

            ui->statusbar->showMessage("Right arm reset successful");

            // 右臂运动
            float position_xr = 32.5f;
            float speed_xr = 100.0f;
            float acc_xr = 100.0f;
            float dcc_xr = 100.0f;
            std::vector<uint8_t> sendBuf_xr(17);
            int offset = 0;
            sendBuf_xr[offset] = motor_id_xr;
            offset += 1;
            qToLittleEndian<float>(position_xr, sendBuf_xr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_xr, sendBuf_xr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_xr, sendBuf_xr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_xr, sendBuf_xr.data() + offset);

            bus_rt_xr = bus->send_request(2, 0, 8002, sendBuf_xr.data(), static_cast<uint16_t>(sendBuf_xr.size()), 10000);
            if (bus_rt_xr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xr.data_len != 1 ||
                bus_rt_xr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis move failed");
                return;
            }

            float position_zr = 60.0f;
            float speed_zr = 300.0f;
            float acc_zr = 500.0f;
            float dcc_zr = 500.0f;

            std::vector<uint8_t> sendBuf_zr(17);
            offset = 0;
            sendBuf_zr[offset] = motor_id_zr;
            offset += 1;
            qToLittleEndian<float>(position_zr, sendBuf_zr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(speed_zr, sendBuf_zr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(acc_zr, sendBuf_zr.data() + offset);
            offset += sizeof(float);
            qToLittleEndian<float>(dcc_zr, sendBuf_zr.data() + offset);

            bus_rt_zr = bus->send_request(2, 0, 8002, sendBuf_zr.data(), static_cast<uint16_t>(sendBuf_zr.size()), 10000);
            if (bus_rt_zr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zr.data_len != 1 ||
                bus_rt_zr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis move failed");
                return;
            }

            ui->statusbar->showMessage("Right arm movement completed");
            QThread::sleep(5);

            // 右臂复位
            bus_rt_zr = bus->send_request(2, 0xffff, 20000, &motor_id_zr, 1, 20000);
            bus_rt_xr = bus->send_request(2, 0xffff, 20000, &motor_id_xr, 1, 20000);

            if (bus_rt_zr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_zr.data_len != 1 ||
                bus_rt_zr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right Z-axis reset failed");
            }

            if (bus_rt_xr.errorCode != Smartbus::BusErrorCode::BusErrorCode_OK ||
                bus_rt_xr.data_len != 1 ||
                bus_rt_xr.data.get()[0] != 0)
            {
                ui->statusbar->showMessage("Right X-axis reset failed");
            }

            ui->statusbar->showMessage("Right arm reset completed");
            });

        // 启动两个线程
        leftThread.detach();
        rightThread.detach();

        ui->statusbar->showMessage("Both arms started moving simultaneously");
        });
}
