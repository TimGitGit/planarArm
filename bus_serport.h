#pragma once

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QThread>
#include "Smartbus.h"



class bus_serport :public QObject ,public Smartbus_space::Smartbus::Bus_Interface
{
	Q_OBJECT
public:
	bus_serport();
	~bus_serport();

	void open_new_serport_(QString ser, qint32 Baud);
	void close_serport_();

	enum serport_errcode
	{
		serport_errcode_ok,
		serport_errcode_err
	};


private:

	QThread self_thread;
	QSerialPort serial;
	void output_data_fram(uint8_t* dataBuf, uint32_t len) override;

public slots:
	void ReadData_slots();
	void sendData_slots(QByteArray data);
	serport_errcode open_new_serport(QString ser, qint32 Baud);
	serport_errcode close_serport();
signals:

};
