#include "WSerialPort.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <getopt.h>
#include <string.h>
#include <time.h>
#include <sys/select.h>
#include <thread>
#include <chrono>

#include <atomic>

#define MAX_RECV_SIZE 1024

WSerialPort::WSerialPort(const char *port_name, int baudrate, int data_bits, int parity, int stop_bits)
{
    // 判断一下是否是合法的波特率，否则默认为9600
    if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400 && baudrate != 57600 && baudrate != 115200)
    {
        baudrate = 9600;
    }
    // 判断一下是否是合法的奇偶校验位，否则默认为无校验
    if (parity != 'o' && parity != 'e' && parity != 'n')
    {
        parity = 'n';
    }
    // 判断一下是否是合法的数据位，否则默认为8位
    if (data_bits != 5 && data_bits != 6 && data_bits != 7 && data_bits != 8)
    {
        data_bits = 8;
    }
    // 判断一下是否是合法的停止位，否则默认为1位
    if (stop_bits != 1 && stop_bits != 2)
    {
        stop_bits = 1;
    }
    // serial_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    serial_fd = ::open(port_name, O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        printf("open %s failed\n", port_name);
        ::close(serial_fd);
        exit(-1);
    }
    else
    {

        if (fcntl(serial_fd, F_SETFL, 0) < 0)
        {
            printf("WSerial  fcntl F_SETFL, %s\n", strerror(errno));
               ::close(serial_fd);
        exit(-1);
        }

        if (isatty(serial_fd) == 0)
        {
            printf(" WSerial open failed with standard input is not a terminal device!\n");
              ::close(serial_fd);
        exit(-1);
        }

        struct termios newtio, oldtio;
        if (tcgetattr(serial_fd, &oldtio) != 0)
        {
            printf("WSerial open failed with tcgetattr failed!\n");
              ::close(serial_fd);
        exit(-1);
        }

        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        /* databit init */
        switch (data_bits)
        {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        }
        /* paritybit init */
        switch (parity)
        {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        }
        /* stopbit init */
        switch (stop_bits)
        {
        case 1:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        }
        /* baudrate init */
        switch (baudrate)
        {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 19200:
            cfsetispeed(&newtio, B19200);
            cfsetospeed(&newtio, B19200);
            break;
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
            break;
        case 57600:
            cfsetispeed(&newtio, B57600);
            cfsetospeed(&newtio, B57600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        }

        newtio.c_cc[VTIME] = 1;
        newtio.c_cc[VMIN] = 1;

        tcflush(serial_fd, TCIFLUSH);
        if (tcsetattr(serial_fd, TCSANOW, &newtio) != 0)
        {
            printf(" tcsetattr open failed with tcgetattr failed!\n");
               ::close(serial_fd);
        exit(-1);
        }

        tcflush(serial_fd, TCIOFLUSH);
        fcntl(serial_fd, F_SETFL, 0);
        printf("Open Serial Port   %s Success! \n", port_name);
    }
 
}

WSerialPort::~WSerialPort()
{
    close(serial_fd);
}

void WSerialPort::StartRecv()
{
    std::thread([&]
                {

                    while(1)
                    {
                           unsigned char receiveData[MAX_RECV_SIZE]={0};

        int recv_len = read(serial_fd,receiveData,MAX_RECV_SIZE);
        if (recv_len > 0)
        {
            //receiveData转化为const char *类型
            const char *recv_data = reinterpret_cast<const char *>(receiveData);
            recv_callback(recv_data,recv_len);
        }else if (recv_len < 0)
        {
            printf("read error!\n");
        } 
                    }

     
        
        })
        .detach();
}

int WSerialPort::SendData(const char *data, int len)
{
    std::lock_guard<std::mutex> lock(send_mutex);

    if (serial_fd < 0)
    {
        return -1;
    }
    else
    {
        return write(serial_fd, data, len);
    }
}
