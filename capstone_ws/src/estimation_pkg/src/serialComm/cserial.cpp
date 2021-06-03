#include "../include/estimation_pkg/serialComm/cserial.h"

cserial::cserial(){
  m_target.position_P = 0;
  m_target.velocity_P = 30;
  m_target.position_Y = 0;
  m_target.velocity_Y = 30;
  m_sendPacket.data.header[0] = m_sendPacket.data.header[1] = m_sendPacket.data.header[2] = m_sendPacket.data.header[3] = 0xFE;
    m_sendPacket.data.id = 1;
    m_sendPacket.data.mode = 2;
    m_sendPacket.data.size = sizeof(Packet_t);
}

cserial::~cserial(){

}

// bool cserial::Open(std::string port, int baudrate){
//   m_ser.setPort(port);
//   m_ser.setBaudrate(baudrate);
//   serial::Timeout to = serial::Timeout::simpleTimeout(100);
//   m_ser.setTimeout(to);
//   m_ser.open();
//   m_Open = m_ser.isOpen();
//   return m_Open;
// }
bool cserial::Open(std::string port, int baudrate){
  m_ser.Open(const_cast<char*>(port.c_str()), baudrate);
  m_Open = m_ser.isOpen();
  return m_Open;
}


bool cserial::Close() {
    m_ser.Close();
    m_Open = m_ser.isOpen();
    return m_Open;
}

void cserial::Execute() {
  static int mode, readSize = 0, checkSize;
    static unsigned char check;

    if (m_ser.isOpen()) {
      std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

      m_sendPacket.data.check = 0;
      m_sendPacket.data.pos_Y = m_target.position_Y*1000;
      m_sendPacket.data.velo_Y = m_target.velocity_Y*1000;
      m_sendPacket.data.pos_P = m_target.position_P*1000;
      m_sendPacket.data.velo_P = m_target.velocity_P*1000;

      //checkbit ����
      for (int i = 8; i < sizeof(Packet_t); i++)
        m_sendPacket.data.check += m_sendPacket.buffer[i];
      //packet �߼�
      m_ser.Write(m_sendPacket.buffer, sizeof(Packet_t));

      //receive packet
      readSize = m_ser.Read(m_recvBuf, 4096);
      
     // std::cout << "Read : " << readSize << std::endl;
     // std::cout << "rev : " << m_recvBuf[0] << std::endl;


      for (int i = 0; i < readSize; i++) {

        switch (mode) {

        case 0:
      //    std::cout << "Case 0 " << std::endl;

          if (m_recvBuf[i] == 0xFE) {//������Ŷ 4�� �������� Ȯ���� mode1�� ����
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
          //std::cout << "Case 1 " << std::endl;

          //��Ŷ ������ buffer�� ����
          m_packet.buffer[checkSize++] = m_recvBuf[i];
          //8bit���۹޾Ҵ��� Ȯ���ϰ� mode2������ ����
          if (checkSize == 8) {
            mode = 2;
          }
          break;

        case 2:
         // std::cout << "Case 2 " << std::endl;

          //��Ŷ pos,vel,cur�� ������ ����
          m_packet.buffer[checkSize++] = m_recvBuf[i];
          check += m_recvBuf[i];	// check sum

          if (checkSize == m_packet.data.size) {
           // std::cout << "size ok" << std::endl;

            if (check == m_packet.data.check) {			// check bit Ȯ��


                m_current.position_Y = m_packet.data.pos_Y / 1000.;		//get Motor Pos
                //m_current.velocity_Y = m_packet.data.velo_Y / 1000.;		//get Motor Vel
                m_current.position_P = m_packet.data.pos_P / 1000.;		//get Motor Pos
                //m_current.velocity_P = m_packet.data.velo_P / 1000.;		//get Motor Vel

              //  std::cout << "Yaw deg : " << m_current.position_Y << std::endl;
               // std::cout << "veloY" << m_current.velocity_Y << std::endl;
              //  std::cout << "Pitch deg : " << m_current.position_P << std::endl;
               // std::cout << "veloP" << m_current.velocity_P << std::endl;


              }

            check = 0;
            mode = 0;
            checkSize = 0;
            }
           


          }

        }
        std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;
     // std::cout << "sec_serial : " << sec.count() << std::endl;
      }
    else {
      std::cout << " Serial is not Open "<< std::endl;
    }

}

