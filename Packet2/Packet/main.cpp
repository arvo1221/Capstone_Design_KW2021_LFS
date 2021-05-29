#include <iostream>
#include <string>
#include <math.h>
#include <conio.h>
#include "Comm.h"
#include "dataType.h"

#define pi 3.1415926535

//using namespace std;

int main() {

	//std::cout << "hello world" << "\n";
	CComm serial;
	const char* port = "COM6";
    Packet_t _sendPacket;
    Packet_t _packet;
    unsigned char _recvBuf[4096];
    unsigned char _writeBuf[4096];
	int16_t degree = 0;
	int a = 1;
	double pos, vel, shoot;

//    int8_t i = 0;
//    int8_t j = 0;

    static int mode, readSize = 0, checkSize;

	std::cout << "Connecting Serial" << "\n";
	serial.Open(port, 115200);

    if (!serial.isOpen()) {//시리얼 통신 유무 파악
        std::cout << "Serial Connecting Failed" << "\n";
        std::cout << "Retry Connecting Serial" << "\n";
        serial.Open(port, 115200);//1회차 실패시 다시 한번 시도
        if (!serial.isOpen()) {
            std::cout << "Serial Connecting Failed" << "\n";
            return -1;//재시도 실패시 프로그램 종료
        }
        else   std::cout << "Serial Connecting Success" << "\n";
    }
    else   std::cout << "Serial Connecting Success" << "\n";

    while (1) {
		if (degree > 120)	a = -1;
		else if (degree < -120) a = 1;
		degree += a;
		_sendPacket.data.header[0] = _sendPacket.data.header[1] = _sendPacket.data.header[2] = _sendPacket.data.header[3] = 0xFE;
		_sendPacket.data.id = 1;
		_sendPacket.data.mode = 2;
		_sendPacket.data.check = 0;
		_sendPacket.data.size = sizeof(Packet_t);
		_sendPacket.data.pos_P = degree*1000;
		_sendPacket.data.vel_P = 3*1000;
		_sendPacket.data.shoot = 0;
		for (int i = 8; i < sizeof(Packet_t); i++)
			_sendPacket.data.check += _sendPacket.buffer[i];

		serial.Write((char*)_sendPacket.buffer, sizeof(Packet_t));
		readSize = serial.Read((char*)_recvBuf, 4096);
		//printf("Read : %d\n", readSize);

		for (int i = 0; i < readSize; i++) {

			switch (mode) {

			case 0:
				if (_recvBuf[i] == 0xFE) {
					checkSize++;
					if (checkSize == 4) {
						mode = 1;
					}
				}
				else {
					checkSize = 0;
				}
				break;

			case 1:

				_packet.buffer[checkSize++] = _recvBuf[i];

				if (checkSize == 8) {
					mode = 2;
				}
				break;

			case 2:

				_packet.buffer[checkSize++] = _recvBuf[i];

				if (checkSize == _packet.data.size) {
					pos = _packet.data.pos_P / 1000.;
					vel = _packet.data.vel_P / 1000.;
					printf("pos : %lf, vel : %lf, shoot : %d \n",
						pos,
						vel,
						_packet.data.shoot);
						
					//memset(_recvBuf, 0, readSize);
					mode = 0;
					checkSize = 0;
				}

			}
		}
        
    }



	return 0;
}