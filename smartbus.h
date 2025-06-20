#pragma once


#include <queue>
#include <map>
#include <unordered_map>
#include <vector>
#include <list>
#include <cstring>

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <exception>



namespace Smartbus_space
{
    using namespace std;

	template<typename T>
	class threadsafe_queue 
	{
	private:
		mutable std::mutex					m_data_mutex;
		mutable std::condition_variable		m_data_cond;
	
        queue<T>							m_data_queue;

	public:


		threadsafe_queue() = default;


		/*
		* 将元素加入队列
		**/
		void push(const T& new_value);

		/*
		* 从队列中弹出一个元素，如果队列为空就阻塞
		*/
        T wait_and_pop();

		/*
		* 从队列中弹出一个元素，如果队列为空就返回false
		*/
		bool try_pop(T& value);

		/*
		* 返回队列是否为空
		*/
		auto empty() const -> decltype(m_data_queue.empty());

		/*
		* 返回队列中元素个数
		*/
		auto size() const -> decltype(m_data_queue.size());
	}; 

	class Smartbus
	{

#pragma region  exception

#pragma endregion

#pragma region  TYPE

    public:

    private:
        enum BusPacketType
        {
            BusPacketType_unkonw =0,
            BusPacketType_ack = 1,//应答 
            BusPacketType_beat = 2,//心跳 
            BusPacketType_request = 10,//请求  
            BusPacketType_recover = 20,//回复  
            BusPacketType_simple = 30// 简易包
        };

        //应答池 应答状态
        enum Bus_ack_pool_state {

            Bus_ack_pool_state_finish = 0,//应答完成
            Bus_ack_pool_state_error_unnecessary = 1,//多余的请求或回复
            Bus_ack_pool_state_error_supportedApp = 2,//不支持的APP
            Bus_ack_pool_state_error_max_connection = 3,//超出最大连接
            Bus_ack_pool_state_error = 4,//
        };

        //连接 连接状态
        enum Bus_connect_pool_state {
            Bus_connect_pool_state_waitConnect = 0,//等待建立连接
            Bus_connect_pool_state_waitRecover = 1,//等待回复
            Bus_connect_pool_state_finish = 2,//连接已完成
            Bus_connect_pool_state_error_unknow = 3,
            Bus_connect_pool_state_error_beatTimeOut = 4//连接过程中心跳超时
        };

        //做包状态
        enum BusMod_pack_state
        {
            BusMod_pack_state_WaitStart = 0,//等待开始
            BusMod_pack_state_Receiving = 1,//接收中
            BusMod_pack_state_Escapeing = 2,//转义中
        };

        //数据包头 数据结构
        union  BusPacketHeaderKey
        {
            struct {
                uint8_t srcId;
                uint8_t desId;
                uint16_t connectId;
            }_key;

            uint32_t key;

            BusPacketHeaderKey() 
            {
                key = 0;
            };
            BusPacketHeaderKey(uint32_t k)
            {
                key = k;
            };
            
            BusPacketHeaderKey(uint8_t src, uint8_t des, uint16_t id)
            {
                _key.srcId = src;
                _key.desId = des;
                _key.connectId = id;
            };
        };

        //数据包 数据结构
        struct BusPacket
        {
            BusPacketHeaderKey  key; // 四个字节
            BusPacketType type = BusPacketType_unkonw;//包种类 //一个字节
            uint16_t dataLen=0;//数据长度（字节） 两个字节
            uint8_t* data =nullptr;//数据 dataLen个字节
            uint32_t crc = 0;//crc校验 四个字节
        };

        struct BusDataSrc
        {
            BusPacket packet;
            bool isTransmit;//标识是否转发包
        };

        //一个简单的信号量
        class Semaphore
        {
        public:
  
            explicit Semaphore(int count);
            //~Semaphore();
            void Acquire();
            bool TryAcquireFor(std::chrono::milliseconds timeout);
            void Release();

        private:
            int count_;
            std::mutex mutex_;
            std::condition_variable cv_;
        };

        //连接池的元素定义
        struct ConnectPoolType {
            Bus_connect_pool_state state;
            Semaphore* semaphore;//     
            bool isActiveFlag;//为true时代表本次连接是主动连接
            uint16_t beatOutTimeCount;//心跳超时计数
            BusPacket recoverPack;//深拷贝储存 连接的返回数据 便于接收

            uint32_t maxTime;//连接允许存在的最大时间
            uint32_t maxTimeCount;

            //ConnectPoolType& operator=(const ConnectPoolType& other)
            //{
            //    if (this != &other)
            //    {
            //        state = other.state;
            //        semaphore = other.semaphore;
            //        isActiveFlag = other.isActiveFlag;
            //        beatOutTimeCount = other.beatOutTimeCount;
            //        recoverPack = other.recoverPack;
            //        maxTime = other.maxTime;
            //        maxTimeCount = other.maxTimeCount;
            //    }
            //    return *this;
            //}
        };
        //应答池的元素定义
        struct AckPoolType {
            Bus_ack_pool_state state;

            Semaphore* semaphore;

            //AckPoolType& operator=(const AckPoolType& other)
            //{
            //    if (this != &other)
            //    {
            //        state = other.state;
            //        semaphore = other.semaphore;
            //    }
            //    return *this;

            //}
        };

    public:

        
        enum BusErrorCode {

            BusErrorCode_OK = 0,//无错误
            BusErrorCode_AckErr = 1,//应答错误
            BusErrorCode_BeatErr = 2,//心跳错误
            BusErrorCode_ConnetErr = 3,//连接错误
            BusErrorCode_TimeOutErr = 4,//超时
            BusErrorCode_NotFindConnet = 5,//没有找到连接
            BusErrorCode_MemErr = 6,//内存错误
            BusErrorCode_NotFindChan = 7
        };
        //连接返回
        struct BusReturn
        {
            BusErrorCode errorCode;//通信错误码
            unique_ptr<uint8_t>data;
   
            uint32_t data_len;
        };

        struct BusCfg
        {
            uint16_t packStructBaseByteSize = (4 + 1 + 2 + 4);//数据包结构基础字节大小
            uint16_t busAckTimeOut = 1000;//总线应答重发等待时间ms
            uint16_t busAckreopenCount = 5;//总线应答重发次数
            uint16_t busBeatTimeOut = 10000;//心跳超时时间
            uint16_t busBeatSpanTime = 2000;//心跳间隔

            uint8_t  bus_Packet_Start = 0xAA;//
            uint8_t  bus_Packet_End = 0xEE;
            uint8_t  bus_Packet_Escape = '\\';

            uint32_t bus_packet_max_len = 60 * 1024;
            uint16_t bus_thread_pool_max = 64;
        };
        enum BusState
        {
            BusState_Open,
            BusState_Close
        };
        struct Bus_transpond_info
        {
            uint8_t  messenger_id;
            vector<uint8_t> chans;
        };

        typedef uint32_t RouteList_t[8];
   
        class Bus_Interface
        {
        public:
            friend class Smartbus;

        protected:
            Smartbus* bus;
            void input_data_frame(uint8_t* dataBuf, uint32_t len);

        public:
            Bus_Interface();
            ~Bus_Interface();

            virtual void output_data_fram(uint8_t* dataBuf, uint32_t len) = 0;

        };


        class Bus_app
        {
        public:
            friend class Smartbus;
        protected:
            Smartbus* bus;
     
        public:
            virtual void Task_call_back(BusPacketHeaderKey src_key,uint8_t* dataBuf, uint32_t len)=0;
        };

  
    private:

        class BusApp_ping :public Smartbus::Bus_app
        {
        public:
            BusApp_ping();
            ~BusApp_ping();

            void Task_call_back(BusPacketHeaderKey src_key, uint8_t* dataBuf, uint32_t len) override;
        private:
            
        };
        
        class BusApp_transpond :public Smartbus::Bus_app
        {
        public:
            friend class Smartbus;
            BusApp_transpond(Smartbus *b);
            ~BusApp_transpond();
       
            void Task_call_back(BusPacketHeaderKey src_key, uint8_t* dataBuf, uint32_t len) override;
        protected:
            Smartbus * bus;

        private:

        };

        struct Bus_Interface_info
        {
            Bus_Interface* inface;
            RouteList_t routeList;//接口的路由表

            vector<uint8_t> dataBuf;
            BusMod_pack_state makePack_state;

        public:

            Bus_Interface_info(Bus_Interface* face, RouteList_t rou)
            {
                inface = face;
        
                makePack_state = BusMod_pack_state_WaitStart;

                memcpy(routeList, rou, sizeof(RouteList_t));
            }
        };




        struct  Bus_interface_data_queue
        {
           const Bus_Interface* inface;
           uint8_t* const data;
           const uint32_t len;

        public:
            Bus_interface_data_queue(Bus_Interface* f, uint8_t* d, uint32_t l):inface(f), data(d),len(l)
            {
      
            }
        };




        enum Bus_App_support_t
        {
            BusApp_simple_unsupport =0,
            BusApp_simple_support = 1,
            BusApp_request_support = 2,
            //BusApp_simple_support_and_request = 4,
        };

        struct Bus_App_info
        {
            Bus_app* app;

            uint16_t cmd_space;
            uint16_t cmd;
            Bus_App_support_t support;
        public:

            Bus_App_info(Bus_app* a, uint16_t cs, uint16_t c, Bus_App_support_t sup)
            {
                app = a;
                cmd_space = cs;
                cmd = c;
                support = sup;
            }
        };



        typedef uint16_t Bus_transpond_key;

        struct Bus_transpond_obj
        {
            std::mutex mtx;
            std::condition_variable cv;
            vector<uint8_t> data;
        };
        //一个简易的线程池
        class ThreadPool {

        public:
            ThreadPool(size_t numThreads);
            ~ThreadPool();

            void enqueue(std::function<void()> task);

        private:
            std::vector<std::thread> workers;

            std::queue<std::function<void()>> tasks;

            std::mutex queueMutex;
            std::condition_variable condition;
            bool stop;

            void workerThread();
        };



        static const uint32_t crc_table[256];
        static uint32_t CRC32(uint8_t* const& di, uint32_t crcLen);

#pragma endregion

    public:
        Smartbus(uint8_t id);
        Smartbus(uint8_t id, BusCfg c);
        ~Smartbus();

        void open();
        void close();

        const BusCfg cfg;
    private:

        //内部默认的APP
        BusApp_ping* app_ping;
        BusApp_transpond * app_tranpond;


        const uint8_t local_id;
        atomic<BusState> state;
        atomic<uint16_t> cureent_connectId;//当前连接id
        uint16_t connectId_get();
        atomic<bool> task_exit_flag;

        // 发送 -- 接受 ---超时处理 相关
        thread  *send_thread;//发送任务线程
        thread  *receive_thread;//接收任务线程
        thread  *connect_pool_thread;//连接池处理任务线程
       
        mutable mutex send_thread_wait_end;
        mutable mutex receive_thread_wait_end;
        mutable mutex connect_thread_wait_end;

        threadsafe_queue<BusDataSrc> receiveQueue;
        threadsafe_queue<BusDataSrc> sendQueue;

        unordered_map<uint32_t, ConnectPoolType> connect_pool;
        unordered_map<uint32_t, AckPoolType> ack_pool;

        mutable mutex connect_pool_mutex;
        mutable mutex ack_pool_mutex;

        void send_Task();
        void receive_Task();
        void connect_pool_Task();

        //硬件接口 数据缓存相关
        thread* datacach_thread;//硬件接口的数据 缓存线程
        mutable mutex datacach_thread_wait_end;
        threadsafe_queue<Bus_interface_data_queue> datacachQueue;
        void datacach_Task();


        //硬件接口注册相关
        list<Bus_Interface_info*> interface_list;
        Bus_Interface* find_route(uint8_t des);

        //线程池 相关
        ThreadPool* threadPool;

        //App接口注册相关
        list<Bus_App_info*> app_list;

        Bus_App_info const *  app_find(BusDataSrc const & datasrc);
        void  app_run(Bus_App_info const* findApp_info,BusDataSrc const& datasrc);

        void send_ack(BusPacketHeaderKey key,  Bus_ack_pool_state state);
        void send_beat(BusPacketHeaderKey key);

        //Bus转发功能 
        unordered_map<Bus_transpond_key, Bus_transpond_obj * > bus_transpond_list;

    public:
        void register_interface(Bus_Interface *interface, RouteList_t route);
        void register_app(Bus_app* app, uint16_t cmd_space, uint16_t cmd, Bus_App_support_t support);

        void register_transpond(Bus_transpond_info tran);

        BusReturn send_request(uint8_t desId, uint16_t funSpace, uint16_t funCode, uint8_t const* data, uint16_t  len, uint32_t timeOut);
        BusReturn send_simple(uint8_t desId, uint16_t funSpace, uint16_t funCode, uint8_t const* data, uint16_t  len);
        BusReturn send_recover(BusPacketHeaderKey key, uint8_t const* data, uint16_t  len);


        //BusReturn send_transpond_only(uint8_t desId,uint8_t chan, uint8_t const* data, uint16_t  len);
        //BusReturn read_transpond_only(uint8_t desId, uint8_t chan,uint32_t timeOut);

        //转发
        BusReturn sr_transpond(uint8_t desId, uint8_t chan, uint8_t const* data, uint16_t  len, uint32_t timeOut, bool (*check)(vector<uint8_t>  &));


        //BusReturn send_broadcast(uint16_t funSpace, uint16_t funCode, uint8_t const* data, uint16_t  len);
	};  
    



}


