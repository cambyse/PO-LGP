#include "Connection.h"



Connection* Connection::getConnection() {

  static Connection connection;
 
  return &connection;

}

Connection::Connection() {

  if (!openConnection(DEVICE)) {

    std::cout << "Program will be closed\n" << std::cout;
    exit(0);

  }

}


bool Connection::openConnection(const char* device) {

  bool success = true;

  _fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK );
  
  if (_fd == -1) {
    
    perror("Failed to open device");
    success = false;

  }
  else {

    if (tcgetattr(_fd, &defaults) < 0) {

      perror("tcgetattr") ;
      success = false;
    }
  
    cfmakeraw(&config);

    if (tcsetattr(_fd, TCSANOW, &config) < 0) {

      perror("tcsetattr config");
      success = false;
    }

  }
  
  return success; 

}

bool Connection::isDeviceConnected(unsigned char address) {

  unsigned char buffer[2];
  buffer[0] = 0x58;
  buffer[1] = address;
  

  if (write(_fd, buffer, 2) < 0) {

    perror("isDeviceConnected write");    
    tcflush(_fd, TCIOFLUSH);
    return false;
  }


  if (tcdrain(_fd) < 0) {

    perror("isDeviceConnected tcdrain");
    tcflush(_fd, TCIOFLUSH);
    return false;
  }

  usleep(5000);
  if (read(_fd, buffer, 1) < 0) {

    perror("isDeviceConnected read");
    tcflush(_fd, TCIOFLUSH);
    return false;
  }

  return true;
}


bool Connection::registerInteraction(unsigned char buffer[], std::string registerDescription, int bytesToWrite, int bytesToRead) {

  tcflush(_fd, TCIFLUSH);

  if (write(_fd, buffer, bytesToWrite) < 0) {

    perror(registerDescription.c_str());
    tcflush(_fd, TCIOFLUSH);
    return false;
  }


  if (tcdrain(_fd) < 0) {

    perror(registerDescription.c_str());
    tcflush(_fd, TCIOFLUSH);
    return false;
  }


  usleep(5000);
  if (read(_fd, buffer, bytesToRead) < 0) {
 
    perror(registerDescription.c_str());
    tcflush(_fd, TCIOFLUSH);
    return false;
  }

  return true;
}


void Connection::closeConnection() {

  if (tcsetattr(_fd, TCSANOW, &defaults) < 0) {

    perror("tcsetattr default");

  }

  close(_fd); 

}
