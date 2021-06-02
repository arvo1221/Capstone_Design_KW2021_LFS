#ifndef CSERIAL2_H
#define CSERIAL2_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include "serial/serial.h"
#include <string.h>
#include "cstdlib"
#include "math.h"
#include "stdio.h"
#include "../include/estimation_pkg/serialComm/DataType2.h"
#include "../include/estimation_pkg/serialComm/t_serial.h"


class cserial2 {
  // Define ////////////////////////////////////////////////////////
  public:

  protected:

    // Method ////////////////////////////////////////////////////////
  public:
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: CComm
    // Input	: None
    // Output	: None
    // Summury	: Standard constructor
    ////////////////////////////////////////////////////////////////////////////////////////////
    cserial2();


    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: ~CComm
    // Input	: None
    // Output	: None
    // Summury	: Standard destructor
    ////////////////////////////////////////////////////////////////////////////////////////////
    ~cserial2();



    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: Open
    // Input	: port name(std::stirng), baudrate(int)
    // Output	: Result(bool)
    // Summury	: Open port handler.
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool Open(std::string port, int baudrate);



    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: Close
    // Input	: None
    // Output	: Result(bool)
    // Summury	: Close port handler.
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool Close();

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Method	: _execute()
    // Input	: None
    // Output	: None
    // Summury	: Execute Serial Communication work.
    ////////////////////////////////////////////////////////////////////////////////////////////
    void Execute();

  protected:

  private:


  // Member ////////////////////////////////////////////////////////
  public:
    bool m_Open;
    unsigned char m_recvBuf[256];
    unsigned char m_writeBuf[4096];
    
    //Camera motor & shooting 
    Packet2_t m_sendPacket;
    Packet2_t m_packet;

    ControlData2_t m_target, m_current;   

  protected:

  private:
    //serial::Serial m_ser;
    t_serial m_ser;

};



#endif // CSERIAL_H
