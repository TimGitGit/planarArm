#include <iostream>
#include "Smartbus.h"

using namespace std;
using namespace Smartbus_space;

 

Smartbus::Smartbus(uint8_t id) : local_id(id)
{
	state.store(BusState_Close);

	task_exit_flag.store(false);
	cureent_connectId.store(0);
	
	send_thread = nullptr;
	receive_thread = nullptr;
	connect_pool_thread = nullptr;
	datacach_thread = nullptr;

	//根据配置 初始化 线程池
	threadPool = new ThreadPool(cfg.bus_thread_pool_max);

	app_ping = new BusApp_ping();
	register_app(app_ping, 0, 1, BusApp_request_support);

	app_tranpond = new BusApp_transpond(this);
	register_app(app_tranpond,0,2001, BusApp_simple_support);

}

Smartbus::Smartbus(uint8_t id ,BusCfg c) : local_id(id), cfg(c)
{
	task_exit_flag.store(false);
	
	cureent_connectId.store(0);
	state.store(BusState_Close);

	send_thread = nullptr;
	receive_thread = nullptr;
	connect_pool_thread = nullptr;
	datacach_thread = nullptr;

	// 根据配置 初始化 线程池
	threadPool = new ThreadPool(cfg.bus_thread_pool_max);

	app_ping = new BusApp_ping();

	register_app(app_ping,0,1, BusApp_request_support);

}


Smartbus::~Smartbus()
{
	BusState state_current = state.load();

	if (state_current == BusState_Open)
	{
		close();

		delete threadPool;

		for (Bus_Interface_info* face : interface_list)
		{
			face->inface = nullptr;
		
			delete face;
		}
		interface_list.clear();

		for (Bus_App_info* app_info : app_list)
		{
			app_info->app = nullptr;

			delete app_info;
		}

		delete app_ping;

	}

}


void Smartbus::open()
{
	BusState state_current = state.load();

	if (state_current == BusState_Open)
	{
		throw std::logic_error("bus is already open!");
	}
	else
	{
		task_exit_flag.store(false);


		send_thread = new thread([this]() { this->send_Task();});
		receive_thread = new thread([this]() { this->receive_Task(); });
		connect_pool_thread = new thread([this]() { this->connect_pool_Task();});

		datacach_thread = new thread([this]() { this->datacach_Task(); });


		send_thread->detach();
		receive_thread->detach();
		connect_pool_thread->detach();
		datacach_thread->detach();

		state.store(BusState_Open);
	}
}

void Smartbus::close()
{
	BusState state_current = state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is already Close!");
	}
	else
	{
		task_exit_flag.store(true);


		sendQueue.push({});
		receiveQueue.push({});

		connect_thread_wait_end.lock();
		connect_thread_wait_end.unlock();
		send_thread_wait_end.lock();
		send_thread_wait_end.unlock();
		receive_thread_wait_end.lock();
		receive_thread_wait_end.unlock();
		datacach_thread_wait_end.lock();
		datacach_thread_wait_end.unlock();

		delete send_thread;
		delete receive_thread;
		delete connect_pool_thread;
		delete datacach_thread;

		//消费完所有队列
		while (sendQueue.empty()==false)
		{
			BusDataSrc pack;
			sendQueue.try_pop(pack);
		}
		while (receiveQueue.empty() == false)
		{
			BusDataSrc pack;
			receiveQueue.try_pop(pack);
		}

		while (datacachQueue.empty() == false)
		{
			datacachQueue.wait_and_pop();
		}

		//清除所有连接池

		connect_pool_mutex.lock();
		connect_pool.clear();
		connect_pool_mutex.unlock();

		state.store(BusState_Close);
		
	}
}


void Smartbus::send_ack(BusPacketHeaderKey key, Bus_ack_pool_state state)
{
	BusState state_current = this->state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is not open!");
	}

	BusDataSrc sendPack;
	sendPack.packet.type = BusPacketType_ack;
	sendPack.isTransmit = false;

	sendPack.packet.key._key.desId = key._key.srcId;
	sendPack.packet.key._key.srcId = local_id;
	sendPack.packet.key._key.connectId = key._key.connectId;

	sendPack.packet.dataLen = 1;
	sendPack.packet.data = (uint8_t*)malloc(1);
	sendPack.packet.data[0] = (uint8_t)state;

	sendQueue.push(sendPack);
}

void Smartbus::send_beat(BusPacketHeaderKey key)
{
	BusState state_current = this->state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is not open!");
	}

	BusDataSrc sendPack;
	sendPack.packet.type = BusPacketType_beat;

	sendPack.isTransmit = false;

	sendPack.packet.key._key.desId = key._key.srcId;
	sendPack.packet.key._key.srcId = local_id;
	sendPack.packet.key._key.connectId = key._key.connectId;

	sendPack.packet.dataLen = 0;
	sendPack.packet.data = nullptr;

	sendQueue.push(sendPack);

}


Smartbus::BusReturn Smartbus::send_request(uint8_t desId, uint16_t funSpace, uint16_t funCode, uint8_t const* data, uint16_t  len, uint32_t timeOut)
{
	BusState state_current = this->state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is not open!");
	}

	BusReturn rt;
	rt.data = nullptr;
	rt.data_len = 0;
	//先把数据 进行封包
	BusDataSrc sendPack;
	sendPack.isTransmit = false;
	sendPack.packet.dataLen = len + 4 + 4;
	sendPack.packet.data = (uint8_t*)malloc(sendPack.packet.dataLen);

	if (sendPack.packet.data == nullptr)
	{
		throw std::logic_error("malloc is err!");
	}

	*(uint32_t*)(&sendPack.packet.data[0]) = timeOut;
	*(uint16_t*)(&sendPack.packet.data[4]) = funSpace;
	*(uint16_t*)(&sendPack.packet.data[6]) = funCode;

	if (len > 0)
	{
		memcpy(&sendPack.packet.data[8], data, len);
	}

	sendPack.packet.type = BusPacketType_request;
	sendPack.packet.key._key.desId = desId;
	sendPack.packet.key._key.srcId = local_id ;
	sendPack.packet.key._key.connectId = connectId_get();


	//建立连接池
	ConnectPoolType c_pool;
	c_pool.semaphore = new Semaphore(0);
	c_pool.state = Bus_connect_pool_state_waitConnect;
	c_pool.isActiveFlag = true;
	c_pool.maxTimeCount = 0;
	c_pool.beatOutTimeCount = 0;
	connect_pool_mutex.lock();
	connect_pool.insert(make_pair(sendPack.packet.key.key, c_pool));
	connect_pool_mutex.unlock();


	//建立 应答池
	AckPoolType a_pool;
	a_pool.semaphore = new Semaphore(0);


	ack_pool_mutex.lock();
	ack_pool.insert(make_pair(sendPack.packet.key.key, a_pool));
	ack_pool_mutex.unlock();

	//进行发送和等待应答
	bool reust;
	bool is_wait_ack = false;
	int reopen_count = 0;
	for (int i = 0; i < cfg.busAckreopenCount; i++)
	{
		sendQueue.push(sendPack);
		bool mutex_rt = a_pool.semaphore->TryAcquireFor (chrono::milliseconds(cfg.busAckTimeOut));

		if (mutex_rt == true)
		{
			is_wait_ack = true;
			break;
		}
		reopen_count++;
	}


    //等到应答后 清理刚才创建的应答
	ack_pool_mutex.lock();

		if (ack_pool.find((sendPack.packet.key.key)) != ack_pool.end())
		{
		
			a_pool = ack_pool[sendPack.packet.key.key];
		}
		else
		{
			a_pool.state = Bus_ack_pool_state_error_unnecessary;
		}

		ack_pool.erase(sendPack.packet.key.key);
		delete a_pool.semaphore;

	ack_pool_mutex.unlock();


	if (sendPack.packet.data != nullptr)
		free(sendPack.packet.data);
	

	rt.errorCode = BusErrorCode_OK;
	//根据应答的状态 进行操作
	if (is_wait_ack == false)
	{
		rt.errorCode = BusErrorCode_AckErr;
		goto clearnANdRteurn;
	}
	else if (is_wait_ack == true)
	{
		if (a_pool.state != Bus_ack_pool_state_finish)
		{
			if (reopen_count > 0 && a_pool.state == Bus_ack_pool_state_error_unnecessary)
			{
				//Debug.WriteLine("reopen,but first not ack");
			}
			else
			{
	
				if (a_pool.state == Bus_ack_pool_state_error_unnecessary)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error_supportedApp)
					rt.errorCode = BusErrorCode_NotFindConnet;
				else if (a_pool.state == Bus_ack_pool_state_error_max_connection)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error)
					rt.errorCode = BusErrorCode_ConnetErr;

				goto clearnANdRteurn;
			}
		}
	}


	// 阻塞至接收到 回复包 或超时
	reust = c_pool.semaphore->TryAcquireFor(chrono::milliseconds(timeOut));

	if (reust == true)
	{
		//从连接池中 更新下 cpool的信息
		connect_pool_mutex.lock();

			if ( (connect_pool.find((sendPack.packet.key.key)) != connect_pool.end()))
			{
				c_pool = connect_pool[sendPack.packet.key.key];
			}
			else
			{
				c_pool.state = Bus_connect_pool_state_finish;
			}
		
		connect_pool_mutex.unlock();


		if (c_pool.state == Bus_connect_pool_state_finish)
		{
			rt.errorCode = BusErrorCode_OK;
		}
		else if (c_pool.state == Bus_connect_pool_state_error_beatTimeOut)
		{
			rt.errorCode = BusErrorCode_BeatErr;
		}
		else
		{
			rt.errorCode = BusErrorCode_ConnetErr;
		}
	}
	else
	{
		rt.errorCode = BusErrorCode_TimeOutErr;
	}

clearnANdRteurn:


	if (c_pool.recoverPack.data != nullptr && c_pool.recoverPack.dataLen > 0)
	{
		rt.data = std::move(unique_ptr<uint8_t>(c_pool.recoverPack.data));
		rt.data_len = c_pool.recoverPack.dataLen;
	}
	else
	{
		rt.data = nullptr;
	}

	//从连接池中 移除这个连接
	connect_pool_mutex.lock();
	connect_pool.erase(sendPack.packet.key.key);
	delete c_pool.semaphore;
	connect_pool_mutex.unlock();


	return rt;

}

Smartbus::BusReturn Smartbus::send_simple(uint8_t desId, uint16_t funSpace, uint16_t funCode, uint8_t const* data, uint16_t  len)
{
	BusState state_current = this->state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is not open!");
	}

	BusReturn rt;
	rt.data = nullptr;
	rt.data_len = 0;
	//先把数据 进行封包
	BusDataSrc sendPack;
	sendPack.isTransmit = false;

	sendPack.packet.type = BusPacketType_simple;
	sendPack.packet.key._key.desId = desId;
	sendPack.packet.key._key.srcId = local_id;
	sendPack.packet.key._key.connectId = connectId_get();

	sendPack.packet.dataLen = len + 4;
	sendPack.packet.data = (uint8_t*)malloc(sendPack.packet.dataLen);

	if (sendPack.packet.data == nullptr)
	{
		throw std::logic_error("malloc is err!");
	}

	*(uint16_t*)(&sendPack.packet.data[0]) = funSpace;
	*(uint16_t*)(&sendPack.packet.data[2]) = funCode;

	if (len > 0)
	{
		memcpy(&sendPack.packet.data[4], data, len);
	}



	//建立 应答池
	AckPoolType a_pool;
	a_pool.semaphore = new Semaphore(0);
	ack_pool_mutex.lock();
	ack_pool.insert(make_pair(sendPack.packet.key.key, a_pool));
	ack_pool_mutex.unlock();

	//进行发送和等待应答
	bool is_wait_ack = false;
	int reopen_count = 0;

	for (int i = 0; i < cfg.busAckreopenCount; i++)
	{
		sendQueue.push(sendPack);
		bool mutex_rt = a_pool.semaphore->TryAcquireFor(chrono::milliseconds(cfg.busAckTimeOut));

		if (mutex_rt == true)
		{
			is_wait_ack = true;
			break;
		}
		reopen_count++;
	}

	//等到应答后 清理刚才创建的应答
	ack_pool_mutex.lock();
	a_pool = ack_pool[sendPack.packet.key.key];
	ack_pool.erase(sendPack.packet.key.key);
	delete a_pool.semaphore;
	ack_pool_mutex.unlock();

	if (sendPack.packet.data != nullptr)
		free(sendPack.packet.data);


	rt.errorCode = BusErrorCode_OK;

	//根据应答的状态 进行操作
	if (is_wait_ack == false)
	{
		rt.errorCode = BusErrorCode_AckErr;
	}
	else if (is_wait_ack == true)
	{
		if (a_pool.state != Bus_ack_pool_state_finish)
		{
			if (reopen_count > 0 && a_pool.state == Bus_ack_pool_state_error_unnecessary)
			{
				//Debug.WriteLine("reopen,but first not ack");
			}
			else
			{

				if (a_pool.state == Bus_ack_pool_state_error_unnecessary)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error_supportedApp)
					rt.errorCode = BusErrorCode_NotFindConnet;
				else if (a_pool.state == Bus_ack_pool_state_error_max_connection)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error)
					rt.errorCode = BusErrorCode_ConnetErr;
			}
		}
	}

	rt.data = nullptr;
	return rt;
}

Smartbus::BusReturn Smartbus::send_recover(BusPacketHeaderKey key, uint8_t const* data, uint16_t  len)
{
	BusState state_current = this->state.load();

	if (state_current == BusState_Close)
	{
		throw std::logic_error("bus is not open!");
	}

	BusReturn rt;
	rt.data = nullptr;
	rt.data_len = 0;

	ConnectPoolType c_pool;
	c_pool.isActiveFlag = true;
	connect_pool_mutex.lock();

	if (connect_pool.find(key.key) != connect_pool.end())
	{	
		//暂停心跳发送
		connect_pool[key.key].state = Bus_connect_pool_state_finish;
		c_pool = connect_pool[key.key];
	}
	
	connect_pool_mutex.unlock();

	if (c_pool.isActiveFlag == true)
	{
		rt.errorCode = BusErrorCode_NotFindConnet;
		return rt;
	}

	BusDataSrc sendPack;

	sendPack.isTransmit = false;
	sendPack.packet.type = BusPacketType_recover;
	sendPack.packet.key._key.desId = key._key.srcId;
	sendPack.packet.key._key.srcId = local_id;
	sendPack.packet.key._key.connectId = key._key.connectId;

	if (data != nullptr && len > 0)
	{
		sendPack.packet.dataLen = len;
		sendPack.packet.data = (uint8_t*)malloc(len);
		if (sendPack.packet.data == nullptr)
		{
			throw std::logic_error("malloc is err!");
		}
		memcpy(sendPack.packet.data,data,len);
	}
	else
	{
		sendPack.packet.dataLen = 0;
		sendPack.packet.data = nullptr;
	}
	
	
	//建立 应答池
	AckPoolType a_pool;
	a_pool.semaphore = new Semaphore(0);
	ack_pool_mutex.lock();
	ack_pool.insert(make_pair(sendPack.packet.key.key, a_pool));
	ack_pool_mutex.unlock();

	//进行发送和等待应答
	bool is_wait_ack = false;
	int reopen_count = 0;
	for (int i = 0; i < cfg.busAckreopenCount; i++)
	{
		sendQueue.push(sendPack);
		bool mutex_rt = a_pool.semaphore->TryAcquireFor(chrono::milliseconds(cfg.busAckTimeOut));

		if (mutex_rt == true)
		{
			is_wait_ack = true;
			break;
		}
		reopen_count++;
	}

	//等到应答后 清理刚才创建的应答
	ack_pool_mutex.lock();
	a_pool = ack_pool[sendPack.packet.key.key];
	ack_pool.erase(sendPack.packet.key.key);
	delete a_pool.semaphore;
	ack_pool_mutex.unlock();

	if (sendPack.packet.data != nullptr)
		free(sendPack.packet.data);


	rt.errorCode = BusErrorCode_OK;

	//根据应答的状态 进行操作
	if (is_wait_ack == false)
	{
		rt.errorCode = BusErrorCode_AckErr;
	}
	else if (is_wait_ack == true)
	{
		if (a_pool.state != Bus_ack_pool_state_finish)
		{
			if (reopen_count > 0 && a_pool.state == Bus_ack_pool_state_error_unnecessary)
			{
				//Debug.WriteLine("reopen,but first not ack");
			}
			else
			{

				if (a_pool.state == Bus_ack_pool_state_error_unnecessary)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error_supportedApp)
					rt.errorCode = BusErrorCode_NotFindConnet;
				else if (a_pool.state == Bus_ack_pool_state_error_max_connection)
					rt.errorCode = BusErrorCode_ConnetErr;
				else if (a_pool.state == Bus_ack_pool_state_error)
					rt.errorCode = BusErrorCode_ConnetErr;
			}
		}
	}


	//从连接池中 移除这个连接
	connect_pool_mutex.lock();
	connect_pool.erase(key.key);
	delete c_pool.semaphore;

	connect_pool_mutex.unlock();

	return rt;
}


void Smartbus::register_interface(Bus_Interface* interface , RouteList_t route)
{
	BusState state_current = state.load();

	if (state_current == BusState_Open)
	{
		throw std::logic_error("bus is already open!");
	}

	if (interface == nullptr)
	{
		throw std::invalid_argument("interface is NULL!");
		
	}
	for (const Bus_Interface_info * face : interface_list)
	{
		if (face->inface  == interface)
		{
			throw std::logic_error("interface is already register!");
		}
	}

	interface->bus = this;
	Bus_Interface_info* add_info = new Bus_Interface_info(interface, route);

	if ( add_info->inface== nullptr)
	{
		throw std::logic_error("interface dataBuf is  err!");	
	}

	interface_list.push_back(add_info);
}

void Smartbus::register_app(Bus_app* app, uint16_t cmd_space, uint16_t cmd, Bus_App_support_t support)
{
	BusState state_current = state.load();

	if (state_current == BusState_Open)
	{
		throw std::logic_error("bus is already open!");
	}

	if (app == nullptr)
	{
		throw std::invalid_argument("interface is NULL!");

	}

	for (const Bus_App_info* a : app_list)
	{
		if (a->cmd == cmd && a->cmd_space == cmd_space)
		{
			throw std::logic_error("app is already register!");
		}
	}

	app->bus = this;
	Bus_App_info* app_info = new Bus_App_info(app, cmd_space,cmd, support);

	app_list.push_back(app_info);

}


void Smartbus::register_transpond(Bus_transpond_info tran)
{

	for (uint8_t c : tran.chans)
	{
		uint16_t key = ((uint16_t)tran.messenger_id << 8) | c;

		if (this->bus_transpond_list.find(key) == this->bus_transpond_list.end())
		{
			bus_transpond_list.insert(make_pair(key, new Bus_transpond_obj() ));
		}
	}

}


//消费者模型
Smartbus::BusReturn Smartbus::sr_transpond(uint8_t desId, uint8_t chan, uint8_t const* data, uint16_t  len, uint32_t timeOut,
	
	bool (*check)(vector<uint8_t> &)
)
{
	if (timeOut == 0)
		timeOut = UINT32_MAX;

	BusReturn rt;
	uint16_t  key = ((uint16_t)desId << 8) | chan;
	//没有找到转发信道
	if (this->bus_transpond_list.find(key) == this->bus_transpond_list.end())
	{
		rt.errorCode = BusErrorCode_NotFindChan;
		return rt;
	}

	//有数据才发送 否则直接等待接收
	if (len > 0 && data != nullptr)
	{
		uint8_t* sendData = (uint8_t*)malloc(len + 3);

		if (sendData == nullptr)
		{
			rt.errorCode = BusErrorCode_MemErr;
			return rt;
		}

		//指定信道
		sendData[0] = chan;
		//设定数据长度
		*(uint16_t*)&sendData[1] = len;
		memcpy(&sendData[3], data, len);

		BusReturn simple_rt =  this->send_request (desId, 0x00, 2000, sendData, len + 3,10000);

		free(sendData);

		if (simple_rt.errorCode != BusErrorCode::BusErrorCode_OK)
		{

			return simple_rt;
		}

	}

	//这里进入接收阶段
	
	uint32_t time_acc = 0;
	vector<uint8_t> rt_data;
	rt_data.clear();

	
	Bus_transpond_obj* obj = bus_transpond_list[key];
	std::unique_lock<std::mutex> lock(obj->mtx);
	obj->data.clear();
	while (1)
	{
		clock_t start_time;
		clock_t end_time;
		int elapsed_mm = 0;

		start_time = clock();
		obj->data.clear();

		bool status = obj->cv.wait_for(lock, std::chrono::milliseconds(timeOut- time_acc),
			[obj]()
			{
				cout << "cv wait = " << obj->data.size() <<"\r\n";
				if (obj->data.size() > 0)
					return true;
				else
					return false;
			}
		
		);

		cout << "sr_transpond len = " << obj->data.size() << "\r\n";
		for (int i = 0; i < obj->data.size(); i++)
		{
		cout << (int)obj->data[i] << " ";
		}
		cout << "\r\n";


		end_time = clock();
		elapsed_mm = int(end_time - start_time);//计算出 间隔的时间

		rt_data.insert(rt_data.end(), obj->data.begin(), obj->data.end());

		//超时
		if (status == false)
		{
			rt.errorCode = BusErrorCode_TimeOutErr;
			return rt;
		}

		if (check(rt_data) == true)
		{
			break;
		}
	
	
		time_acc += elapsed_mm;

	}
	
	

	uint8_t* rtdata = (uint8_t*)malloc(rt_data.size());

	if (rtdata == nullptr)
	{
			rt.errorCode = BusErrorCode_MemErr;
			return rt;
	}
	
	rt.errorCode = BusErrorCode_OK;

	memcpy(rtdata, rt_data.data(), rt_data.size() );
	rt.data = std::move(unique_ptr<uint8_t>(rtdata));
	rt.data_len = rt_data.size();

	return rt;

}



uint16_t Smartbus:: connectId_get()
{
	uint16_t id =cureent_connectId.fetch_add(1);
	return id;
}

Smartbus::Bus_Interface* Smartbus::find_route(uint8_t des)
{
	if (des == 0)
	{
		return nullptr;
	}

	for (Bus_Interface_info* face : interface_list)
	{
		uint8_t quo = des / 32;
		uint32_t const* start = &face->routeList[quo];
		uint8_t mod = des % 32;
		uint32_t mask = (0x80000000 >> mod);
		if (*start & mask)
		{
			return face->inface;
		}
	}
	return nullptr;

}

void Smartbus::send_Task()
{
	uint8_t* send_buf = (uint8_t*)malloc(this->cfg.bus_packet_max_len * 2);
	uint8_t* send_pack_buf = (uint8_t*)malloc(this->cfg.bus_packet_max_len);

	send_thread_wait_end.lock();

	BusDataSrc send_pack;

	std::cout << "send_Task start\n";
	for (;;)
	{
		send_pack = this->sendQueue.wait_and_pop();

		bool is_exit = this->task_exit_flag.load();

		if (is_exit == true)
		{
			break;
		}

		uint16_t offset = 0;
		uint32_t crc = 0;
		uint32_t sendByteLen = 0;//	

		Bus_Interface* des_face = find_route(send_pack.packet.key._key.desId);
		if (des_face == nullptr)
		{
			goto CleanMem;
		}
		//把packet转换为Byte数组
		{
			offset = 0;
			memcpy(&send_pack_buf[offset], &send_pack.packet.key.key, sizeof(uint32_t));
			offset += sizeof(uint32_t);
			send_pack_buf[offset] = (uint8_t)(send_pack.packet.type);
			offset += sizeof(uint8_t);
			memcpy(&send_pack_buf[offset], &send_pack.packet.dataLen, sizeof(uint16_t));
			offset += sizeof(uint16_t);

			if (send_pack.packet.dataLen != 0)
			{
				memcpy(&send_pack_buf[offset], send_pack.packet.data, (send_pack.packet.dataLen));
				offset += (send_pack.packet.dataLen);
			}
				
			crc = CRC32(send_pack_buf, offset);
			memcpy(&send_pack_buf[offset], &crc, sizeof(uint32_t));


			//增加转义符
			sendByteLen = this->cfg.packStructBaseByteSize + send_pack.packet.dataLen;
			{
				send_buf[0] = this->cfg.bus_Packet_Start;
				uint32_t i = 0, k = 1;
				for (i = 0; i < sendByteLen; i++, k++)
				{
					if (send_pack_buf[i] == this->cfg.bus_Packet_Start ||
						send_pack_buf[i] == this->cfg.bus_Packet_End ||
						send_pack_buf[i] == this->cfg.bus_Packet_Escape)

					{
						send_buf[k] = this->cfg.bus_Packet_Escape;
						k++;
						send_buf[k] = send_pack_buf[i];
					}
					else
					{
						send_buf[k] = send_pack_buf[i];
					}
				}
				send_buf[k] = this->cfg.bus_Packet_End;
				k++;
				sendByteLen = k;
			}

			des_face->output_data_fram(send_buf, sendByteLen);

		CleanMem:
			//转发包 发送完毕后无论如何也要直接清理内存 
			//不是转发包 就由发送者自行清理
			if (send_pack.isTransmit == true)
			{
				if(send_pack.packet.data != NULL)
					free(send_pack.packet.data);
			}
			else
			{
				if (send_pack.packet.type == BusPacketType_ack)
				{
					if (send_pack.packet.data != NULL)
						free(send_pack.packet.data);
				}
			}

		}


		

	}
	std::cout << "send_Task end\n";

	if (send_buf != nullptr)
		free(send_buf);

	if (send_pack_buf != nullptr)
		free(send_pack_buf);


	send_thread_wait_end.unlock();
}

void Smartbus::receive_Task()
{

	BusDataSrc receive_pack;

	receive_thread_wait_end.lock();

	std::cout << "receive_Task start\n";
	for (;;)
	{
		receive_pack = this->receiveQueue.wait_and_pop();

		bool is_exit = this->task_exit_flag.load();

		if (is_exit == true)
		{
			if (receive_pack.packet.data != nullptr)
			{
				free(receive_pack.packet.data);
			}
			break;
		}

		//std::cout << "receive_type"<< receive_pack.packet.type<<"\n";
		//std::cout << receive_pack.packet.type << "\n";

		switch (receive_pack.packet.type)
		{
		case BusPacketType_request:
		{
			ConnectPoolType makeConnect;
			ConnectPoolType find_cpool;


			connect_pool_mutex.lock();

			if (connect_pool.find(receive_pack.packet.key.key) == connect_pool.end())
			{
				Bus_App_info const* findapp = nullptr;

				findapp = app_find(receive_pack);

				if (findapp == nullptr)
				{
					send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_supportedApp);
				}
				else
				{
					send_ack(receive_pack.packet.key, Bus_ack_pool_state_finish);


					makeConnect.semaphore = nullptr;
					makeConnect.state = Bus_connect_pool_state_waitRecover;
					makeConnect.isActiveFlag = false;
					makeConnect.beatOutTimeCount = 0;
					makeConnect.maxTime = *(uint32_t*)&receive_pack.packet.data[0];
					makeConnect.maxTimeCount = 0;
					if (makeConnect.maxTime < 5000)
						makeConnect.maxTime = 5000;

					connect_pool.insert(make_pair(receive_pack.packet.key.key, makeConnect));

				
					app_run(findapp, receive_pack);
				}

			}

			else
			{
				send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_unnecessary);
			}

			
			connect_pool_mutex.unlock();

			break;
		}
		case BusPacketType_simple:
		{
			Bus_App_info const* findapp = nullptr;

			findapp = app_find(receive_pack);

			if (findapp == nullptr)
			{
				send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_supportedApp);
			}
			else
			{
				send_ack(receive_pack.packet.key, Bus_ack_pool_state_finish);

				//cout << "find " << findapp->cmd<<"\r\n";
				app_run(findapp, receive_pack);
			}
			break;
		}
		
		case BusPacketType_recover:
		{
			BusPacketHeaderKey findkey;

			findkey._key.connectId = receive_pack.packet.key._key.connectId;
			findkey._key.desId = receive_pack.packet.key._key.srcId;
			findkey._key.srcId = receive_pack.packet.key._key.desId;

			connect_pool_mutex.lock();

			if (connect_pool.find(findkey.key) != connect_pool.end())
			{
				send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_supportedApp);

				ConnectPoolType find_cpool = connect_pool[findkey.key];

				//收到回复包 但是没收到应答包
				if (find_cpool.state == Bus_connect_pool_state_waitConnect)
				{
					find_cpool.recoverPack = receive_pack.packet;

					if (receive_pack.packet.data != nullptr && receive_pack.packet.dataLen != 0)
					{
						//创建一份数据副本 放入连接池的回复 源数据立刻清除 副本数据在 建立连接函数内通过智能指针清除
						uint8_t* r_data = (uint8_t*)malloc(receive_pack.packet.dataLen);
						if (r_data != nullptr)
							memcpy(r_data, receive_pack.packet.data, receive_pack.packet.dataLen);

						find_cpool.recoverPack.data = r_data;
					}
					find_cpool.state = Bus_connect_pool_state_finish;

					connect_pool[findkey.key] = find_cpool;

					if (find_cpool.semaphore != nullptr)
					{
						find_cpool.semaphore->Release();
					}
					else
					{
						if (find_cpool.recoverPack.data != nullptr)
							free(find_cpool.recoverPack.data);

						connect_pool.erase(findkey.key);

						cout << "cp1 free\r\n";
					}
						
					ack_pool_mutex.lock();
					if (ack_pool.find((findkey.key)) != ack_pool.end())
					{
						AckPoolType findAck = ack_pool[findkey.key];
						findAck.state = Bus_ack_pool_state_finish;
						ack_pool[findkey.key] = findAck;
						if (findAck.semaphore != nullptr)
						{
							findAck.semaphore->Release();
						}
						
					}
					ack_pool_mutex.unlock();

				}
				//收到回复包 也已经收到应答包
				else if (find_cpool.state == Bus_connect_pool_state_waitRecover)
				{
					find_cpool.recoverPack = receive_pack.packet;

					if (receive_pack.packet.data != nullptr && receive_pack.packet.dataLen != 0)
					{
						uint8_t* r_data = (uint8_t*)malloc(receive_pack.packet.dataLen);

						if(r_data!=nullptr)
							memcpy(r_data, receive_pack.packet.data, receive_pack.packet.dataLen);

						find_cpool.recoverPack.data = r_data;
					}
					else
					{
						find_cpool.recoverPack.data = nullptr;
					}

					find_cpool.state = Bus_connect_pool_state_finish;

					connect_pool[findkey.key] = find_cpool;


					if (find_cpool.semaphore != nullptr)
					{
						find_cpool.semaphore->Release();
					}
					else
					{
						if (find_cpool.recoverPack.data != nullptr)
							free(find_cpool.recoverPack.data);

						connect_pool.erase(findkey.key);

						cout << "cp 2free\r\n";
					}
				}
				else
				{
					send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_unnecessary);
				}

			}
			else
			{
				send_ack(receive_pack.packet.key, Bus_ack_pool_state_error_unnecessary);
			}

			connect_pool_mutex.unlock();
			break;
		}

		case BusPacketType_ack:
		{
			BusPacketHeaderKey findkey;
			findkey._key.connectId = receive_pack.packet.key._key.connectId;
			findkey._key.desId = receive_pack.packet.key._key.srcId;
			findkey._key.srcId = receive_pack.packet.key._key.desId;

			ack_pool_mutex.lock();

			if (ack_pool.find((findkey.key)) != ack_pool.end())
			{
				//先把连接状态修改 可能的话
				connect_pool_mutex.lock();

					if (connect_pool.find(findkey.key) != connect_pool.end())
					{
						ConnectPoolType findConnect;
						findConnect = connect_pool[findkey.key];
						findConnect.state = Bus_connect_pool_state_waitRecover;
						connect_pool[findkey.key] = findConnect;
						//cout << "ack c is find\r\n";
					}
					else
					{
						//cout << "ack c not find\r\n";
					}

				connect_pool_mutex.unlock();


				AckPoolType findAck = ack_pool[findkey.key];
				findAck.state = (Bus_ack_pool_state)receive_pack.packet.data[0];
				ack_pool[findkey.key] = findAck;

				if (findAck.semaphore != nullptr)
				{
					findAck.semaphore->Release();
				}
				else
				{
					ack_pool.erase(findkey.key);
					cout << "ap free\r\n";
				}
					
			}
			else
			{
				cout << "from " << (int)receive_pack.packet.key._key.srcId << " to " << (int)receive_pack.packet.key._key.desId << "id" << receive_pack.packet.key._key.connectId;
				cout << "ack not find\r\n";
				//应答不在应答池
			}

			ack_pool_mutex.unlock();
			break;

		}

		case BusPacketType_beat:
		{
			BusPacketHeaderKey findkey;

			findkey._key.connectId = receive_pack.packet.key._key.connectId;
			findkey._key.desId = receive_pack.packet.key._key.srcId;
			findkey._key.srcId = receive_pack.packet.key._key.desId;

			connect_pool_mutex.lock();

			if (connect_pool.find(findkey.key) != connect_pool.end())
			{
				ConnectPoolType findConnect;
				findConnect = connect_pool[findkey.key];
				if (findConnect.isActiveFlag == true && findConnect.state == Bus_connect_pool_state_waitRecover)
				{
					findConnect.beatOutTimeCount = 0;
					connect_pool[findkey.key] = findConnect;
				}
			}
			else
			{
				cout << "beat not find c\r\n";
				//心跳包 没有找到对应的 连接池 
			}

			connect_pool_mutex.unlock();
			break;
		}

		default:
			break;
		}

		if (receive_pack.packet.data != nullptr)
		{
			free(receive_pack.packet.data);
		}
	}


	std::cout << "receive_Task end\n";
	receive_thread_wait_end.unlock();

}

void Smartbus::connect_pool_Task()
{
	connect_thread_wait_end.lock();

	std::cout << "connect_pool_Task start\n";

	for (;;)
	{
		this_thread::sleep_for(chrono::milliseconds(this->cfg.busBeatSpanTime));

		bool is_exit = this->task_exit_flag.load();

		if (is_exit == true)
		{
			
			break;
		}

		vector<uint32_t> keys_to_delete;

		
		connect_pool_mutex.lock();

		for (auto & [k,v] : connect_pool)
		{
			if (v.state == Bus_connect_pool_state_waitRecover)
			{
				if (v.isActiveFlag == true)
				{

					if (v.beatOutTimeCount >= cfg.busBeatTimeOut)
					{
						v.beatOutTimeCount = 0;
						v.state = Bus_connect_pool_state_error_beatTimeOut;
						if (v.semaphore != nullptr)
						{
							v.semaphore->Release();
						}
						else
						{

						}
						
					}
				}
				else
				{

					if (v.maxTimeCount >= v.maxTime)
					{
			
						v.maxTimeCount = 0;
						keys_to_delete.push_back(k);

					}
					else if (v.beatOutTimeCount >= cfg.busBeatSpanTime)
					{
						v.beatOutTimeCount = 0;
						//发送心跳包
						BusPacketHeaderKey kkk;
						kkk.key = k;
						send_beat(kkk);
					}

				}
				v.beatOutTimeCount += 1000;
				v.maxTimeCount += 1000;
				
			}
		}

		if (keys_to_delete.empty() != true)
		{
			for (uint32_t key : keys_to_delete)
			{
				connect_pool.erase(key);
			}
			keys_to_delete.clear();
			
		}
		connect_pool_mutex.unlock();
	}
	std::cout << "connect_pool_Task end\n";
	connect_thread_wait_end.unlock();
}




void Smartbus::datacach_Task()
{
	
	datacach_thread_wait_end.lock();
	for (;;)
	{
		Bus_interface_data_queue queue = this->datacachQueue.wait_and_pop();

		bool is_exit = this->task_exit_flag.load();

		if (is_exit == true)
		{
			break;
		}

		if (queue.inface == nullptr || queue.data == nullptr || queue.len == 0)
		{
			if (queue.data != nullptr)
			{	
				free(queue.data);
				continue;
			}
		}

		for (Bus_Interface_info* face : interface_list)
		{
			if (face->inface == queue.inface)
			{
				for (uint32_t i = 0; i < queue.len; i++)
				{
					uint8_t data_one = queue.data[i];

					switch (face->makePack_state)
					{

					case BusMod_pack_state_WaitStart:
						if (data_one == this->cfg.bus_Packet_Start)
						{
							face->dataBuf.clear();
							face->makePack_state = BusMod_pack_state_Receiving;//开始接收
						}
						else
						{
							face->dataBuf.clear();
							face->makePack_state = BusMod_pack_state_WaitStart;//代表出错 恢复初始状态
						}
					break;

					case BusMod_pack_state_Receiving:
						if (data_one == this->cfg.bus_Packet_Start)
						{
							face->dataBuf.clear();
							face->makePack_state = BusMod_pack_state_Receiving;//接收数据中接收到了开始符号 直接重新接收
						}
						
						else if (data_one == this->cfg.bus_Packet_End)//接收数据中 接收到了接收符号 代表接收完成
						{
							BusDataSrc data_src;
				

							data_src.packet.crc = *(uint32_t*)&face->dataBuf.data()[face->dataBuf.size() - 4];

							uint32_t crc_count = Smartbus::CRC32(face->dataBuf.data(), (uint32_t)(face->dataBuf.size()-4));


							if (crc_count == data_src.packet.crc)
							{
								uint16_t offset = 0;
								memcpy(&data_src.packet.key, &face->dataBuf[offset], sizeof(BusPacketHeaderKey));
								offset += sizeof(BusPacketHeaderKey);

								data_src.packet.type = (BusPacketType)face->dataBuf[offset];
								offset += sizeof(uint8_t);

								memcpy(&data_src.packet.dataLen, &face->dataBuf[offset], sizeof(uint16_t));
								offset += sizeof(uint16_t);


								if (data_src.packet.dataLen == (face->dataBuf.size() - this->cfg.packStructBaseByteSize))
								{

									if (data_src.packet.dataLen != 0)
									{

										data_src.packet.data = (uint8_t*)malloc(data_src.packet.dataLen);//这里对包的数据进行深拷贝
										if (data_src.packet.data != NULL)
											memcpy(data_src.packet.data, &face->dataBuf.data()[offset], data_src.packet.dataLen);
									}
									else
									{
										data_src.packet.data = NULL;
									}

									//这里进行判断 是自己的数据包进行处理 不是自己的直接进行转发
									if (data_src.packet.key._key.desId != this->local_id)
									{
										data_src.isTransmit = true;
										this->sendQueue.push(data_src);
									}
									else
									{
										data_src.isTransmit = false;
										this->receiveQueue.push(data_src);
									}
								}
							}
							else
							{
		
							}


							face->dataBuf.clear();
							face->makePack_state = BusMod_pack_state_WaitStart;

						}
						
						else if (data_one == cfg.bus_Packet_Escape)//收到 转义符号 开始转义
						{
							face->makePack_state = BusMod_pack_state_Escapeing;
						}
						
						else //其他情况都是数据
						{

							face->dataBuf.push_back(data_one);							
						}
						
					break;

					case BusMod_pack_state_Escapeing:
						
							face->dataBuf.push_back(data_one);						
							face->makePack_state = BusMod_pack_state_Receiving;//开始接收
					
						break;
					}
				}
				
				break;
			}
		}

		if (queue.data != nullptr)
		{
			free(queue.data);
			continue;
		}

	}

	datacach_thread_wait_end.unlock();
}



Smartbus::Bus_Interface::Bus_Interface()
{


	bus = nullptr;

}

Smartbus::Bus_Interface::~Bus_Interface()
{

	this->bus = nullptr;

}


void Smartbus::Bus_Interface::input_data_frame(uint8_t* dataBuf, uint32_t len)
{
	if (this->bus == nullptr)
	{
		return ;
	}
	if (dataBuf == nullptr|| len==0)
	{
		return ;
	}

	BusState state_current = bus->state.load();

	if (state_current == BusState_Close)
	{
		return;
	}

	uint8_t* data = (uint8_t*)malloc(len);
	if (data != nullptr)
		memcpy(data, dataBuf, len);
	else
		return;

	Bus_interface_data_queue queue = {this,data ,len};
	this->bus->datacachQueue.push(queue);
	
}
		


Smartbus::Bus_App_info const* Smartbus::app_find(BusDataSrc const& datasrc)
{
	uint16_t cmd_space = 0;
	uint16_t cmd = 0;
	Bus_App_support_t cmd_support = BusApp_simple_unsupport;
	if (datasrc.packet.type == BusPacketType_request)
	{
		cmd_space = *(uint16_t*)(&datasrc.packet.data[4]);
		cmd = *(uint16_t*)(&datasrc.packet.data[6]);
		cmd_support = BusApp_request_support;
	}
	else if (datasrc.packet.type == BusPacketType_simple)
	{
		cmd_space = *(uint16_t*)(&datasrc.packet.data[0]);
		cmd = *(uint16_t*)(&datasrc.packet.data[2]);
		cmd_support = BusApp_simple_support;
	}

	Bus_App_info const* find_app  = nullptr;

	for (Bus_App_info* const a : app_list)
	{
		if (a->cmd == cmd && a->cmd_space == cmd_space &&a->support == cmd_support)
		{
			find_app = a;
		}
	}

	return find_app;
}

void  Smartbus::app_run(Bus_App_info const* findApp_info, BusDataSrc const& datasrc)
{
	if (this->threadPool == nullptr)
	{
		throw std::logic_error("threadPool is unstart!");
	}

	
	uint8_t* data = nullptr;
	uint32_t pack_data_len = 0;

	if (datasrc.packet.type == BusPacketType_request)
	{
		pack_data_len = datasrc.packet.dataLen - 4 -4;//减去 cmd和cmdspace 和maxtime

		if (pack_data_len != 0)
		{
			data = (uint8_t*)malloc(pack_data_len);
			if (data != nullptr)
				memcpy(data, &datasrc.packet.data[8], pack_data_len);
		}
	}

	else if (datasrc.packet.type == BusPacketType_simple)
	{
		pack_data_len = datasrc.packet.dataLen - 4;  //减去 cmd和cmdspace
		if (pack_data_len != 0)
		{
			data = (uint8_t*)malloc(pack_data_len);
			if(data!=nullptr)
				memcpy(data, &datasrc.packet.data[4], pack_data_len);
		}
	}

	threadPool->enqueue(
		[findApp_info,datasrc, data, pack_data_len]
		{

			findApp_info->app->Task_call_back(datasrc.packet.key,data, pack_data_len);
			if (data != nullptr)
				free(data);
		}
	);


}


const uint32_t Smartbus::crc_table[256] = {
	0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
	0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
	0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
	0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
	0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
	0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
	0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
	0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
	0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
	0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
	0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
	0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
	0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
	0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
	0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
	0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
	0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
	0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
	0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
	0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
	0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
	0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
	0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
	0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
	0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
	0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
	0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
	0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
	0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
	0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
	0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
	0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4,
};

uint32_t Smartbus::CRC32(uint8_t* const& di, uint32_t crcLen)
{
	////X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+1 total 32 effective bits without X^32.

	//uint32_t crc_poly = 0x04C11DB7;
	uint32_t data_t = 0xffffffff;
	for (uint32_t i = 0; i < crcLen; i++)
	{
		data_t = (data_t << 8) ^ crc_table[(data_t >> 24 ^ di[i]) & 0xff];
	}
	return (data_t);
}


Smartbus::BusApp_ping::BusApp_ping()
{

}

Smartbus::BusApp_ping::~BusApp_ping()
{

}


void Smartbus::BusApp_ping::Task_call_back(BusPacketHeaderKey src_key, uint8_t* dataBuf, uint32_t len)
{
	bus->send_recover(src_key,nullptr,0);
}


Smartbus::BusApp_transpond::BusApp_transpond(Smartbus* b) :bus(b)
{

}

Smartbus::BusApp_transpond::~BusApp_transpond()
{

}


//消息转发 生产者模型
void Smartbus::BusApp_transpond::Task_call_back(BusPacketHeaderKey src_key, uint8_t* dataBuf, uint32_t len)
{
	if (len <= 3 || dataBuf==nullptr)
	{
		return;
	}

	uint8_t des = src_key._key.srcId;
	uint8_t chan = dataBuf[0];
	uint16_t key = (uint16_t)des << 8 | chan;

	cout << "des = " <<(int) des <<  "chan = " << (int)chan << "\r\n";
	//没有找到转发信道
	if (bus->bus_transpond_list.find(key) == bus->bus_transpond_list.end())
	{	
		cout << "BusApp_transpond not find "<< key;
		return;
	}


	Bus_transpond_obj* obj = bus->bus_transpond_list[key];

	std::lock_guard<std::mutex> lock(obj->mtx);

	//obj->data.clear();

	for (uint32_t i=3;i<len;i++)
	{
		obj->data.push_back(dataBuf[i]);
	}



    cout << "tr len = " << obj->data.size() << "\r\n";
	for (int i = 0; i < obj->data.size(); i++)
	{
		cout << (int)obj->data[i] << " ";
	}
	cout << "\r\n";

	obj->cv.notify_all();
	
}



 Smartbus::Semaphore::Semaphore(int count) : count_(count)
{

}


void Smartbus::Semaphore::Acquire()
{
	std::unique_lock<std::mutex> lock(mutex_);
	cv_.wait(lock, [this] { return count_ > 0; });
	--count_;
}

bool Smartbus::Semaphore::TryAcquireFor(std::chrono::milliseconds timeout)
{
	std::unique_lock<std::mutex> lock(mutex_);

	if (cv_.wait_for(lock, timeout, [this] { return count_ > 0; })) {
		--count_;
		return true;
	}
	return false;

}

void Smartbus::Semaphore::Release()
{
	std::unique_lock<std::mutex> lock(mutex_);
	++count_;
	cv_.notify_one();
}



Smartbus::ThreadPool::ThreadPool(size_t numThreads) : stop(false)
{
	for (size_t i = 0; i < numThreads; ++i)
	{
		workers.emplace_back(&ThreadPool::workerThread, this);
	}
}

Smartbus::ThreadPool::~ThreadPool()
{
	{
		std::unique_lock<std::mutex> lock(queueMutex);
		stop = true;
	}

	condition.notify_all();
	for (std::thread& worker : workers) 
	{
		worker.join();
	}
}

void Smartbus::ThreadPool::enqueue(std::function<void()> task)
{
	{
		std::unique_lock<std::mutex> lock(queueMutex);
		tasks.push(std::move(task));
	}
	condition.notify_one();
}

void Smartbus::ThreadPool::workerThread()
{
	while (true)
	{
		std::function<void()> task;

		{
			std::unique_lock<std::mutex> lock(queueMutex);

			condition.wait(lock, 
								[this] { return stop || !tasks.empty(); }
						  );

			if (stop && tasks.empty()) {
				return;
			}
			task = std::move(tasks.front());
			tasks.pop();
		}
		task();
	}
}




template<typename T>
void  threadsafe_queue<T>::push(const T& new_value)
{
	std::lock_guard<std::mutex> lk(m_data_mutex);
	m_data_queue.push(std::move(new_value));
	m_data_cond.notify_one();
}


template<typename T>
 T threadsafe_queue<T>::wait_and_pop()
{
	std::unique_lock<std::mutex> lk(m_data_mutex);
	m_data_cond.wait(lk, [this] {return !this->m_data_queue.empty(); });
	auto value = std::move(m_data_queue.front());
	m_data_queue.pop();
	return value;
}


template<typename T>
bool threadsafe_queue<T>::try_pop(T& value)
{
	std::lock_guard<std::mutex> lk(m_data_mutex);
	if (m_data_queue.empty())
	{
		return false;
	}

	value = std::move(m_data_queue.front());
	m_data_queue.pop();
	return true;
}


template<typename T>
auto threadsafe_queue<T>::empty() const -> decltype(m_data_queue.empty())
{
	std::lock_guard<std::mutex> lk(m_data_mutex);
	return m_data_queue.empty();
}


template<typename T>
auto threadsafe_queue<T>::size() const -> decltype(m_data_queue.size())
{
	std::lock_guard<std::mutex> lk(m_data_mutex);
	return m_data_queue.size();
}

