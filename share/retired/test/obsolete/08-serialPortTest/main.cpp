#define MT_IMPLEMENTATION

#include <MT/serial.h>
#include <Core/util.h>

void sendCommand(byte address,byte command,byte speed){
  PutSerialChar(address);
  PutSerialChar(command);
  PutSerialChar(speed);
  PutSerialChar((address + command + speed) & 0x7f);
}

int main(int argc,char **argv){

  int r=OpenPort(5,9600,0,NULL);
  cout <<"success?" <<r <<endl;
  std::cout <<"serial port open?: " <<SerialPortIsOpen() <<std::endl;
  MT::wait();
  PutSerialChar(170);
  sendCommand(128,0,0);
  cout <<"initialized and 0. go 1?" <<endl;
  MT::wait();
  sendCommand(128,0,10);
  cout <<"stop?" <<endl;
  MT::wait();
  sendCommand(128,0,0);
  MT::wait();

  cout <<"initialized and 0. go -1?" <<endl;
  MT::wait();
  sendCommand(128,1,10);
  cout <<"stop?" <<endl;
  MT::wait();
  sendCommand(128,1,0);
  MT::wait();

  cout <<"initialized and 0. go -1?" <<endl;
  MT::wait();
  sendCommand(128,4,10);
  cout <<"stop?" <<endl;
  MT::wait();
  sendCommand(128,4,0);
  MT::wait();

  cout <<"initialized and 0. go -1?" <<endl;
  MT::wait();
  sendCommand(128,5,10);
  cout <<"stop?" <<endl;
  MT::wait();
  sendCommand(128,5,0);
  MT::wait();

  CloseSerialPort();

  return 0;
}
