#include <iostream>
#include "WSerialPort.h"
#include <memory>

void RecvData(const char * buf, int len) {
   //16进制打印数据
    for(int i=0;i<len;i++)
        printf("%02X ",buf[i]);
    printf("\n");   

}   

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::unique_ptr<WSerialPort> uart2(new WSerialPort("/dev/ttyS3",115200, 'n',8,1));

    uart2.get()->SetRecvCallback(RecvData);    

    uart2.get()->StartRecv();
    //每秒发送hi

    while (1) {
        uart2.get()->SendData("hi\n",3);
        sleep(1);
    }   
  
 
    return 0;

}