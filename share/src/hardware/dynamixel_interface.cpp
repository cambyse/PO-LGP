#ifdef DYNAMIXEL
#include "dynamixel_interface.h"
#include <dynamixel.h>
#include <iostream>

struct sDynamixel {
  int deviceIndex;
  int baudnum;

  void send16Packet(int id, int command, int value) {
    dxl_write_word(id, command, value);
    error_handling();
  }

  void send8Packet(int id, int command, int value) {
    dxl_write_byte(id, command, value);
    error_handling();
  }
  int read16Packet(int id, int command) {
    int value = dxl_read_word(id, command);
    error_handling();
    return value;
  }
  int read8Packet(int id, int command) {
    int value = dxl_read_byte(id, command);
    error_handling();
    return value;
  }
  void error_handling() {
    int 	CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
    {
      if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        std::cout << "Input voltage error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        std::cout << "Angle limit error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        std::cout << "Overheat error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        std::cout << "Out of range error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        std::cout << "Checksum error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        std::cout << "Overload error!" << std::endl;

      if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        std::cout << "Instruction code error!" << std::endl;
    }
    else
    {
      switch(CommStatus)
      {
      case COMM_TXFAIL:
        std::cout << "COMM_TXFAIL: Failed transmit instruction packet!" << std::endl;
        break;

      case COMM_TXERROR:
        std::cout << "COMM_TXERROR: Incorrect instruction packet!" << std::endl;
        break;

      case COMM_RXFAIL:
        std::cout << "COMM_RXFAIL: Failed get status packet from device!" << std::endl;
        break;

      case COMM_RXWAITING:
        std::cout << "COMM_RXWAITING: Now recieving status packet!" << std::endl;
        break;

      case COMM_RXTIMEOUT:
        std::cout << "COMM_RXTIMEOUT: There is no status packet!" << std::endl;
        break;

      case COMM_RXCORRUPT:
        std::cout << "COMM_RXCORRUPT: Incorrect status packet!" << std::endl;
        break;

      default:
        std::cout << "This is unknown error code!" << std::endl;
        break;
      }
    }
  }
};

Dynamixel::Dynamixel() {
  s = new sDynamixel;
  s->baudnum = 1;
  s->deviceIndex = 0;
}

void Dynamixel::openBus() {
  if( dxl_initialize(s->deviceIndex, s->baudnum) == 0 )
    std::cerr << "Failed to open USB2Dynamixel!" << std::endl;
	else
		std::cout << "Succeed to open USB2Dynamixel!" << std::endl;
}


void Dynamixel::closeBus() {
  dxl_terminate();  
}

void Dynamixel::setBusBaudrate(int baudnum) {
  s->baudnum = baudnum;  
}
void Dynamixel::setDeviceNumber(int dev) {
  s->deviceIndex = dev;  
}

void Dynamixel::setMotorBaudrate(int id, int baudnum) {
    s->send8Packet(id, DXL_BAUDRATE, baudnum);
}
void Dynamixel::setID(int old_id, int new_id) {
  s->send8Packet(old_id, DXL_ID, new_id);  
}

void Dynamixel::setGoalPosition(int id, int position) {
  s->send16Packet(id, DXL_GOAL_POSITION_L, position);  
}

void Dynamixel::setGoalAngle(int id, double angle) {
  int pos = ang2pos(angle);  
  s->send16Packet(id, DXL_GOAL_POSITION_L, pos);
}
int Dynamixel::getPosition(int id) {
  return s->read16Packet(id, DXL_POSITION);
}
double Dynamixel::getAngle(int id) {
  int pos = s->read16Packet(id, DXL_POSITION);
  return pos2ang(pos);
}

//void Dynamixel::setSpeed(int id, int speed);
//int Dynamixel::getSpeed(int id);

//void Dynamixel::setLED(int id, bool on);
//bool Dynamixel::getLED(int id);
#endif
