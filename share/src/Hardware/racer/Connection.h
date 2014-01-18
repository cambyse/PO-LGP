#ifndef CONNECTION_H
#define CONNECTION_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <string>

#include "Constants.h"


class Connection {

  public:   

    /*------------------------------------------------------------------------------------
     * Name:
     *   getConnection
     *
     * Description:
     *   If there is no instance of the class Connection existing, the method creates
     *   an instance and returns the reference, else it only returns the reference.
     *   (Singleton)
     *
     * Return:
     *   returns the reference to an instance of the Connection class
     *-----------------------------------------------------------------------------------*/
    static Connection* getConnection();
    



    /*------------------------------------------------------------------------------------
     * Name:
     *   isDeviceConnected
     *
     * Description:
     *   Checks if a device with the given address is connected. Works only with version 5
     *   or later firmeware of the USB-ISS
     *
     * Parameter:
     *   - unsigned char address: the given address of the device
     *
     * Return: returns true if a device with the given address is connected, else false
     *-----------------------------------------------------------------------------------*/
    bool isDeviceConnected(unsigned char address);




    /*------------------------------------------------------------------------------------
     * Name:
     *   registerInteraction
     *
     * Description:
     *   Interacts with a device. The commands, information and data are stored in the buffer.
     *   The mothod sends bytes in the data stream, waits until the data is transmitted and
     *   reads back the data. This method can be used to write data to a register, read out
     *   of a register or do both.
     *
     * Parameter:
     *   - unsigned char buffer[]: all information, data and the read or write command are
     *                             stored in the buffer
     *
     *   - const std::string registerDescription: a string to describe the register or action
     *                                            Only used if an error occured during the
     *                                            interaction. It is needed to identify the
     *                                            error
     *
     *   - const int bytesToWrite: the count of bytes to write
     *
     *   - const int bytesToRead: the count of bytes to read
     *
     * Return:
     *   returns true if no error occured during the interaction, else false
     *-----------------------------------------------------------------------------------*/
    bool registerInteraction(unsigned char buffer[],
                              std::string registerDescription,
                              int bytesToWrite,
                              int bytesToRead
                             );

    /*------------------------------------------------------------------------------------
     * Name:
     *   closeConnection
     *
     * Description:
     *   Closes the Connection and sets the configuration of the termios back to default
     *   settings
     *-----------------------------------------------------------------------------------*/
    void closeConnection();


  private:

    int _fd;
    struct termios defaults;
    struct termios config;
 

    /*------------------------------------------------------------------------------------
     * Name:
     *   Connection
     *
     * Description:
     *   The constructor tries to open a connection. If it fails,
     *   the programm will be closed.
     *-----------------------------------------------------------------------------------*/
    Connection();

  

    /*------------------------------------------------------------------------------------
     * Name:
     *   openConnection
     *
     * Description:
     *   Opens a connection to a serial port and sets the termios configuration to raw modus
     *
     * Parameter:
     *   - const char* device: the path to the serial port
     *
     * Return:
     *   returns true if it was successful, else false
     *-----------------------------------------------------------------------------------*/
    bool openConnection(const char* device);

    
    

   

  
};

#endif // CONNECTION_H
