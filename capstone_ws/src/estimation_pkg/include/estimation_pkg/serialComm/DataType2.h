#pragma once

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958


typedef struct _controlData2 {

  double position_P;
  double velocity_P;
  int shooting;

}ControlData2_t;


typedef struct _packet_data2 {

	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;

  int pos_P;
  int velo_P;
  int shoot;

}Packet_data2_t;


typedef union _packet2 {

	Packet_data2_t data;
	unsigned char buffer[sizeof(Packet_data2_t)];

}Packet2_t;
