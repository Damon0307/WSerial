#ifndef __WSERIALPORT_H__
#define __WSERIALPORT_H__

//串口类 具备发送和接收功能
#include <unistd.h>
#include <functional>
#include <mutex>   
//typedef std::function<void(const char *,int)> recv_callback_type;
using recv_callback_type = std::function<void(const char *,int)>;

class WSerialPort
{
private:
    int serial_fd;
    recv_callback_type recv_callback;
   // std::mutex recv_mutex; 外部调用
    std::mutex send_mutex;

public:
   explicit WSerialPort(const char *port_name,int baudrate,int parity,int data_bits,int stop_bits);
    ~WSerialPort();
   inline void SetRecvCallback(recv_callback_type callback)
    {
        recv_callback = callback;
    }
    void StartRecv();

    int SendData(const char *data,int len);
};
 

 
#endif // __WSERIALPORT_H__