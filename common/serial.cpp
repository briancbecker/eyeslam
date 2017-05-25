/*==================================================================================

    Filename:  serial.cpp

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
#include "serial.h"


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Default constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Serial::Serial()
{
    connection = false;
    baudRate = 9600;
    dataBits = 8;
    stopBits = 1;
    parity = 'N';
    memset(mPort, 0, 200);
#if defined(WIN32)
    hcom = NULL;
#else
    com = 0;
#endif

}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Destructor.
///
///  Disconnects from any active connection.
///
////////////////////////////////////////////////////////////////////////////////////
Serial::~Serial()
{
    if(connection)
        disconnect();
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Connects to a serial port.
///
///  Settings are configured to set the port for 8 data bits, 1 stop bit and no
///  parity check.  Connects to the port in a non-blocking mode.
///
///  \param port The port to connect to
///  \param baud The baud rate to connect at
///  \param bits The number of data bits (default = 8)
///  \param parity The type of parity (0 = none, 1 = even, 2 = odd)
///  \param stop The number of stop bits
///
///  \return 1 on success, 0 on failure
///
////////////////////////////////////////////////////////////////////////////////////
int Serial::connect(const char *port, 
                    const unsigned int baud, 
                    const unsigned int bits, 
                    const unsigned int parity, 
                    const unsigned int stop)
{
    if(this->connected())
        disconnect();

    baudRate = baud;
    this->parity = parity;
    this->stopBits = stop;
    this->dataBits = bits;
#if defined (WIN32)
    if(hcom == NULL)
    {
        char temp[200];
        //  Timeouts for file reading
        //  and writing
        COMMTIMEOUTS timeouts;

        //  First lets check and set
        //  the port name
        if(port == NULL)
        {
            return 0;
        }
        
        strcpy(temp, "\\\\.\\");
        strcat(temp, port);
    
        hcom = CreateFileA(  temp, 
                            GENERIC_READ | GENERIC_WRITE,
                            0,
                            0,
                            OPEN_EXISTING, 
                            0, 
                            0);
        if(hcom == NULL)
        {
            printf("ERROR:  Could not establish connection\n");
            printf("Compass::connect(char *port, int baud)\n");
            return 0;
        }
    
        ZeroMemory(&DCBcomport, sizeof(DCB));
        //  Allocate memory
        DCBcomport.DCBlength = sizeof(DCB);
        //  Get the current comm state
        GetCommState(hcom, &DCBcomport);
  
        DCBcomport.BaudRate = baud;
        DCBcomport.ByteSize = bits;
        DCBcomport.fOutxCtsFlow = FALSE;
        DCBcomport.fOutX = FALSE;
        DCBcomport.fInX = FALSE;
        DCBcomport.ByteSize = bits;
        DCBcomport.fRtsControl = RTS_CONTROL_DISABLE;
        DCBcomport.fDtrControl = DTR_CONTROL_DISABLE;

        switch(stop)
        {
        case 1:
            DCBcomport.StopBits = ONESTOPBIT;
            break;
        case 2:
            DCBcomport.StopBits = ONE5STOPBITS;
            break;
        case 3:
            DCBcomport.StopBits = TWOSTOPBITS;
            break;
        default:
            DCBcomport.StopBits = ONESTOPBIT;
            break;
        };

        DCBcomport.fParity = TRUE;
        switch(parity)
        {
        case 0:
            DCBcomport.Parity = NOPARITY;
            break;
        case 1:
            DCBcomport.Parity = EVENPARITY;
            break;
        case 2:
            DCBcomport.Parity = ODDPARITY;
            break;
        default:
            DCBcomport.Parity = NOPARITY;
            break;
        };
        
        SetCommState(hcom, &DCBcomport);

        
        //  Get the timeouts
        GetCommTimeouts(hcom, &timeouts);

        //  Change them
        timeouts.ReadIntervalTimeout = 100;
        timeouts.ReadTotalTimeoutConstant = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        //timeouts.ReadIntervalTimeout = 4294967295;
        //timeouts.ReadTotalTimeoutConstant = 4294967295;
        //timeouts.ReadTotalTimeoutMultiplier = 500;
        timeouts.WriteTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;

        //  Set the timeouts
        SetCommTimeouts(hcom, &timeouts);
      
        strcpy(this->mPort, port);
        connection = true;

        return 1;

    }

    return 0;

#else
    if(com == 0)
    {
        char temp[50];

        strcpy(temp, port);

        //  Open the com port device
        com = open(port, O_RDWR | O_NOCTTY);
        if (com < 0)
        {
            printf("ERROR:  Could not make connection\n");
            printf("Compass::connect(char *port, int baud)\n");
        return 0;
        }
    
        //  Save the current serial port settings
        tcgetattr(com, &oldtio);
        //  Clear structure for new information
        bzero(&newtio, sizeof(newtio));

        //  Set the BAUD, BITS, CONNECTION ETC.
        //newtio.c_cflag = baud | CRTSCTS | CS8 | CLOCAL | CREAD;
        //  New method here includes setting stop bits, parity, and data bits
        //  dynamically
        newtio.c_cflag = baud | CRTSCTS | CLOCAL | CREAD;
        switch(bits)
        {
        case CS8:
            newtio.c_cflag |= CS8;
            break;
        case CS7:
            newtio.c_cflag |= CS7;
            break;
        case CS6:
            newtio.c_cflag |= CS6;
            break;
        case CS5:
            newtio.c_cflag |= CS5;
            break;
        default:
            newtio.c_cflag |= CS8;
            break;
        }
        
        if(parity)
        {
            if(parity == 1)
            {
                newtio.c_cflag |= PARENB;  //  Enable even parity
            }
            else
            {
                newtio.c_cflag |= PARENB;  //  Enable odd parity
                newtio.c_cflag |= PARODD;
            }
        }

        if(stop == 2)
            newtio.c_cflag |= CSTOPB;

        //  Set input flags
        newtio.c_iflag = IGNPAR | ICRNL;
        //  Raw output
        newtio.c_oflag = 0;
        // Set canonical input
        newtio.c_lflag = ICANON;
        //  Using default characters so no chaning of
        //  newtio.c_cc settings

        //  Now clean the modem line
        tcflush(com, TCIFLUSH);
        tcsetattr(com, TCSANOW, &newtio);
        
        connection = true;
        strcpy(this->mPort, temp);

        return 1;
    }

    return 0;
#endif

}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Disconnects from the connect.
///
////////////////////////////////////////////////////////////////////////////////////
int Serial::disconnect( bool purgeSettings /*= true*/ )
{
#if defined(WIN32)
    PurgeComm(hcom, PURGE_RXABORT);
    CloseHandle(hcom);
    connection = false;
    hcom = NULL;
    if (purgeSettings)
    {
        this->baudRate = 0;
        strcpy(this->mPort, "");
    }
    return 1;
#else
    tcsetattr(com,TCSANOW,&oldtio);
    connection = false;
    com = 0;
    this->baudRate = 0;
    strcpy(this->mPort, "");
    return 1;
#endif
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Reconnects to the current port.
///
////////////////////////////////////////////////////////////////////////////////////
int Serial::reconnect()
{
    int baud, stop, bits;
    char parity, bport[200];

    //  Backup previous settings
    strcpy(bport, mPort);
    baud = this->baudRate;
    stop = this->stopBits;
    parity = this->parity;
    bits = this->dataBits;

    disconnect();
    return this->connect(bport, baud, bits, parity, stop);
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Writes to the serial port.
///
///  \param buff Buffer containing data to write.
///  \param size Number of bytes/size of buffer to write.
///
///  \return Number of bytes sent.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::send(const char *buff, const int size) const
{
#if defined(WIN32)
    unsigned long sum;

    if(connection)
    {
        WriteFile(hcom, (void *)buff, size, &sum, 0);
        return (int)sum;
    }
    else
    {
        return 0;
    }


#else
    
    if(connection)
    {
        int sum;

        sum = write(com, buff, (unsigned int)size);
        return sum;
    }
    else
    {
        return 0;
    }

#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Writes to the serial port.
///
///  \param buff Buffer containing data to write.
///  \param size Number of bytes/size of buffer to write.
///
///  \return Number of bytes sent.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::send(const unsigned char *buff, const int size) const
{
#if defined(WIN32)
    unsigned long sum;

    if(connection)
    {
        WriteFile(hcom, (void *)buff, size, &sum, 0);
        return (int)sum;
    }
    else
    {
        return 0;
    }


#else
    
    if(connection)
    {
        int sum;

        sum = write(com, (char *)buff, (unsigned int)size);
        return sum;
    }
    else
    {
        return 0;
    }

#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Writes to the serial port.
///
///  \param p Data to send.
///
///  \return Number of bytes sent.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::send(const Packet &p) const
{
    return send(p.mPacket, p.mLength);
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Writes to the serial port.
///
///  \param s Data to send.
///
///  \return Number of bytes sent.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::send(const String &s) const
{
    return send(s.mString, s.mLength);
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Reads from the serial port.
///
///  Attempts to receive up to the number of bytes reserved in the packet if
///  size = -1, otherwise attempts to receive "size" number of  bytes.
///
///  \param p Packet to receive data into.
///  \param size If <= 0 than the reserve size of packet is used for
///              receiving, otherwise size number of bytes are attempted.
///
///  \return Number of bytes received.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::recv(Packet &p, const int size) const
{
    unsigned int total;
    p.clear();
    if(size >= 0 )
        p.reserve(size);
    total = this->recv(p.mPacket, p.mReserved);
    p.mLength = total;
    p.mDecodePos = 0;
    p.mEncodePos = p.mLength;
    return total;
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Reads from the serial port.
///
///  Attempts to receive up to the number of bytes reserved in the packet if
///  size = -1, otherwise attempts to receive "size" number of  bytes.
///
///  \param s String to receive data into.
///  \param size If <= 0 than the reserve size of packet is used for
///              receiving, otherwise size number of bytes are attempted.
///
///  \return Number of bytes received.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::recv(String &s, const int size) const
{
    unsigned int total;
    s.clear();
    if(size >= 0 )
        s.reserve(size);
    total = this->recv(s.mString, s.mReserved);
    s.mLength = total;
    return total;
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Reads from the serial port.
///
///  Function returns the number of bytes actually read.
///
///  \param buff Buffer for storing read data.
///  \param size The size of the buffer for storing read data.
///
///  \return Number of bytes received.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::recv(char *buff, int size) const
{
#if defined(WIN32)
    unsigned long sum;

    if(connection)
    {
        ReadFile(hcom, buff, size, &sum, 0);
		if( sum > (unsigned int)size || sum < 0)
			return 0;
        return (int)sum;
    }
    else
    {
        return 0;
    }


#else
    
    if(connection)
    {
        int sum;

        sum = read(com, buff, (unsigned int)size);

        return sum;
    }
    else
    {
        return 0;
    }

#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Reads from the serial port.
///
///  Function returns the number of bytes actually read.
///
///  \param buff Buffer for storing read data.
///  \param size The size of the buffer for storing read data.
///
///  \return Number of bytes received.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::recv(unsigned char *buff, int size) const
{
#if defined(WIN32)
    unsigned long sum;

    if(connection)
    {
        ReadFile(hcom, (void *)buff, size, &sum, 0);
    
        return (unsigned int)sum;
    }
    else
    {
        return 0;
    }


#else
    
    if(connection)
    {
        unsigned int sum;

        sum = read(com, (char *)buff, (unsigned int)size);

        return sum;
    }
    else
    {
        return 0;
    }

#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Returns the connected baud rate.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int Serial::getBaudRate() const
{
    return baudRate;
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Access function for retriving the port name.
///
///  The port name is max 200 characters.
///
///  \param buff Pointer to buffer to write name to
///  \param size The size of the buffer
///
////////////////////////////////////////////////////////////////////////////////////
void Serial::getPortName(char *buff, int size)
{
    if(size <= (int)strlen(mPort))
        return;

    strcpy(buff, mPort);
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Access function for retriving the port name.
///
///  The port name is max 200 characters.
///
///  \param ri Maximum acceptable time, in milliseconds, to elapse 
///            between the arrival of two characters on the communication line.
///  \param rt Specifies the multiplier, in milliseconds, used to calculate the 
///            total timeout period for read operations. For each read operation, 
///            this value is multiplied by the requested number of bytes to be read. 
///  \param rtm Specifies the constant, in milliseconds, used to calculate 
///             the total timeout period for read operations.
///  \param wt Specifies the multiplier, in milliseconds, used to 
///            calculate the total timeout period for write operations.
///  \param wtm Specifies the constant, in milliseconds, used to calculate 
///             the total timeout period for write operations.
///
///  \return 1 on ok, 0 on error.
///
////////////////////////////////////////////////////////////////////////////////////
int Serial::setTimeouts(const int ri,
                        const int rt,
                        const int rtm,
                        const int wt,
                        const int wtm)
{
#ifdef WIN32

    if(connected())
    {
        COMMTIMEOUTS timeouts;
        //  Get the timeouts
        GetCommTimeouts(hcom, &timeouts);

        //  Change them
        timeouts.ReadIntervalTimeout = ri;
        timeouts.ReadTotalTimeoutConstant = rt;
        timeouts.ReadTotalTimeoutMultiplier = rtm;
        timeouts.WriteTotalTimeoutConstant = wt;
        timeouts.WriteTotalTimeoutMultiplier = wtm;

        //  Set the timeouts
        SetCommTimeouts(hcom, &timeouts);
        return 1;
    }

#else
#endif

    return 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief  Returns true if there is an active connection.
///
////////////////////////////////////////////////////////////////////////////////////
bool Serial::connected() const
{
    return this->connection;
}

/*  End of File */
