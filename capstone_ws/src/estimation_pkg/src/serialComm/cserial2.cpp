#include "../include/estimation_pkg/serialComm/cserial2.h"

cserial2::cserial2(){
  m_target.position_P = 0;
  m_target.velocity_P = 30;
  m_target.shooting = 0;
  m_sendPacket.data.header[0] = m_sendPacket.data.header[1] = m_sendPacket.data.header[2] = m_sendPacket.data.header[3] = 0xFE;
    m_sendPacket.data.id = 1;
    m_sendPacket.data.mode = 2;
    m_sendPacket.data.size = sizeof(Packet2_t);
}

cserial2::~cserial2(){

}

// bool cserial2::Open(std::string port, int baudrate){
//   m_ser.setPort(port);
//   m_ser.setBaudrate(baudrate);
//   serial::Timeout to = serial::Timeout::simpleTimeout(100);
//   m_ser.setTimeout(to);
//   m_ser.open();
//   m_Open = m_ser.isOpen();
//   return m_Open;
// }
bool cserial2::Open(std::string port, int baudrate){
  m_ser.Open(const_cast<char*>(port.c_str()), baudrate);
  m_Open = m_ser.isOpen();
  return m_Open;
}

bool cserial2::Close() {
    m_ser.Close();
    m_Open = m_ser.isOpen();
    return m_Open;
}

void cserial2::Execute() {
  static int mode, readSize = 0, checkSize;
    static unsigned char check;

    if (m_ser.isOpen()) {

      m_sendPacket.data.check = 0;
      m_sendPacket.data.pos_P = m_target.position_P*1000;
      m_sendPacket.data.velo_P = m_target.velocity_P*1000;
      m_sendPacket.data.shoot = m_target.shooting;
      //checkbit ����
      for (int i = 8; i < sizeof(Packet2_t); i++)
        m_sendPacket.data.check += m_sendPacket.buffer[i];
      //packet �߼�
      m_ser.Write(m_sendPacket.buffer, sizeof(Packet2_t));

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
              

                m_current.position_P = m_packet.data.pos_P / 1000.;		//get Motor Pos
                m_current.velocity_P = m_packet.data.velo_P / 1000.;		//get Motor Vel
                m_current.shooting = m_packet.data.shoot;
               // std::cout << "posY" << m_packet.data.pos_P/1000 << std::endl;
              //  std::cout << "Shoot state : " << m_packet.data.shoot << std::endl;

              }

            check = 0;
            mode = 0;
            checkSize = 0;
            }
           


          }

        }

        
      }

}

