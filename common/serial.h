/*==================================================================================

    Filename:  serial.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Cross-platform structure used for reading and writing to 
    a serial, (RS232), port. 
    -------------------------------------------------------------------------------

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

==================================================================================*/
#ifndef _SERIAL_H
#define _SERIAL_H

#ifdef __cplusplus

#if defined(WIN32)
#include <winsock.h>
#include <windows.h>
#else
#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <string.h>
#include "packet.h"
#include "zstring.h"


////////////////////////////////////////////////////////////////////////////////////
///
///   \class Serial
///   \brief Wrapper class for serial communication in Windows and Linux.
///
///   Serial class contains function for connecting, reading and writing to
///   serial ports in both Windows and Linux. 
///
///   This class is part of the zebbase library.  zbase is the base library
///   for the Zebulon software libraries and code bank.  It contains code for
///   doing cross platform tasks such as threading, network communication, serial
///   communication, and handling some HI devices.
///
///   Required Libraries:
///   <ul>
///     <li><b>Windows</b>
///         <ol>
///         <li>ws2_32.lib or winsock.lib</li>
///         </ol>
///     </li>
///     <li><b>Linux</b>
///         <ol>
///         <li>None</li>
///         </ol>
///     </li>
///   </ul>
///
////////////////////////////////////////////////////////////////////////////////////
class Serial
{
public:
    Serial();
    ~Serial();
    int  connect(const char *port, 
                 const unsigned int baud = 9600, 
                 const unsigned int bits = 8, 
                 const unsigned int partiy = 0, 
                 const unsigned int stop = 1); 
    int disconnect(bool purgeSettings = true);                                        //  Disconnect
    int  reconnect();                                         //  Attempts to reconnect to current connection
    unsigned int  send(const char *buff, 
                       const int size) const;                 //  Sends data in buff
    unsigned int  send(const unsigned char *buff, 
                       const int size) const;                 //  Sends data in buff
    unsigned int  send(const Packet &p) const;                //  Send packet data.
    unsigned int  send(const String &string) const;           //  Send a string.
    unsigned int  recv(char *buff, int size) const;           //  Puts received data in buff
    unsigned int  recv(unsigned char *buff, int size) const;  //  Puts received data in buff
    unsigned int  recv(Packet &p, const int bytes = -1) const;//  Receives to a packet structure.
    unsigned int  recv(String &s, const int bytes = -1) const;//  Receives to a string structure.
    unsigned int  getBaudRate() const;                        //  Returns baud rate
    void getPortName(char *buff, int size);                   //  Gets the port name
    bool connected() const;
    int setTimeouts(const int ri = 100,
                    const int rt = 1,
                    const int rtm = 10,
                    const int wt = 0,
                    const int wtm = 0);
protected:
#if defined(WIN32)
    ///  Windows handle
    HANDLE hcom;
    ///  Windows COM structure for port settings
    DCB    DCBcomport;
#else
    ///  Structure for backing up port settings in Linux
    struct termios oldtio;
    ///  Structure for desried port settings in Linux
    struct termios newtio;
    ///  File handle
    int    com;
#endif
    char mPort[200];             ///< Port name
    char parity;                ///< Parity
    unsigned int  baudRate;     ///< Current baudrate
    unsigned int  stopBits;     ///< Stop bits
    unsigned int  dataBits;     ///< Data bits
    bool   connection;          ///< Active connection?
};

#endif
#endif
/*  End of File */
