#include "bus_serport.h"


bus_serport::bus_serport()
{
	qRegisterMetaType<serport_errcode>("serport_errcode");

	serial.moveToThread(&this->self_thread);
	this->moveToThread(&this->self_thread);

	connect(&serial, &QSerialPort::readyRead, this, &bus_serport::ReadData_slots);
	self_thread.start();
}


bus_serport::~bus_serport()
{
	if(serial.isOpen() == true)
	{
		serial.close();
	}
}

void bus_serport::ReadData_slots()
{
	QByteArray data = serial.readAll();

	

	if (!data.isEmpty() && data.length() > 0)
	{
		//qDebug() << "BusInterface_SerialPort read:" << data.length();
		//qDebug() << "BusInterface_SerialPort read:" << data;
		
		this->input_data_frame((uint8_t*)data.data(),data.length());
	}
}
void bus_serport:: sendData_slots(QByteArray data)
{
	if (serial.isOpen() == true)
	{
		//qDebug() << "BusInterface_SerialPort send:" << data.length();

		serial.write(data.data(), data.size());
		serial.flush();
	}

}

void bus_serport::open_new_serport_(QString ser, qint32 Baud)
{
	serport_errcode errcode = serport_errcode_ok;

	QMetaObject::invokeMethod(this, "open_new_serport", Qt::BlockingQueuedConnection,
		Q_RETURN_ARG(serport_errcode, errcode),
		Q_ARG(QString, ser),
		Q_ARG(qint32, Baud)
	);

	if (errcode != serport_errcode_ok)
	{
		
	}
}

void bus_serport::close_serport_()
{
	serport_errcode errcode = serport_errcode_ok;

	QMetaObject::invokeMethod(this, "close_serport", Qt::BlockingQueuedConnection,
		Q_RETURN_ARG(serport_errcode, errcode)
	);

	if (errcode != serport_errcode_ok)
	{

	}
}

bus_serport::serport_errcode bus_serport::open_new_serport(QString ser, qint32 Baud)
{
	if (serial.isOpen() == true)
		serial.close();


    serial.setPortName(ser);
    serial.setBaudRate((QSerialPort::BaudRate)Baud);
    serial.setParity(QSerialPort::NoParity);
    serial.setDataBits(QSerialPort::Data8);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.setReadBufferSize(10240);

	if (serial.open(QIODevice::ReadWrite)!=true)
	{
		return serport_errcode::serport_errcode_err;
	}
	return serport_errcode::serport_errcode_ok;
}

bus_serport::serport_errcode bus_serport::close_serport()
{
	if (serial.isOpen() == true)
		serial.close();

	return serport_errcode::serport_errcode_ok;
}

void  bus_serport::output_data_fram (uint8_t * dataBuf, uint32_t len) 
{
	QByteArray sendData;

	sendData.append((char*)dataBuf, len);

	QMetaObject::invokeMethod(this, "sendData_slots", Qt::BlockingQueuedConnection,
		Q_ARG(QByteArray, sendData)
	);
}