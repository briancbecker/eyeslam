/*==================================================================================

    Filename:  packet.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Structure encoding and decoding data into/from a buffer.  Very useful for
    quickly creating network packets and decoding.  
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
#ifndef _PACKET_H
#define _PACKET_H

#ifdef __cplusplus

#include <limits.h>

#define PACKET_LITTLE_ENDIAN      0                 ///<  Little endian byte order.  (Default).
#define PACKET_BIG_ENDIAN         1                 ///<  Big endian byte order.  (Network Byte Order).
#define PACKET_NETWORK_BYTE_ORDER PACKET_BIG_ENDIAN ///<  Network byte order (Big Endian)
#define PACKET_BLOCK_SIZE         64                ///<  Block size (bytes) for reserving memory automatically.
#define SIZE_OF_INT64             8                 ///<  Size of 64 bit integer in bytes.
#define SIZE_OF_INT32             4                 ///<  Size of 32 bit integer in bytes.
#define SIZE_OF_INT16             2                 ///<  Size of 16 bit integer in bytes.

typedef long long int int64;
typedef unsigned long long int uint64;

inline void convertIntegerToReal(int scaled, double upper, double lower, double &real);
inline void convertIntegerToReal(unsigned int scaled, double upper, double lower, double &real);
inline void convertIntegerToReal(short scaled, double upper, double lower, double &real);
inline void convertIntegerToReal(unsigned short scaled, double upper, double lower, double &real);
inline void convertIntegerToReal(int64 scaled, double upper, double lower, double &real);
inline void convertIntegerToReal(uint64 scaled, double upper, double lower, double &real);


////////////////////////////////////////////////////////////////////////////////////
///
///   \class Packet
///   \brief Data structure for storing and creating buffered packet data
///   and decoding it quickly.
///
///   This structure contains many different utilties for encoding and decoding
///   data into a buffer with automatic byte order conversions.  It automatically
///   keeps track of encoding and decoding positions within a buffer make while
///   still allowing decoding and encoding from any point in the buffer through
///   other functions.
///
////////////////////////////////////////////////////////////////////////////////////
class Packet
{
    friend class Network;
    friend class Serial;
public:
    Packet();
    Packet(const Packet &another);                      ///<  Copy constructor.
    Packet(const unsigned char *buff,                   
           const unsigned int len,
           const bool wrapper = false);                 ///<  Create packet with copy of data.
    ~Packet();
    int encode(const Packet &another);                  ///<  Encode packet data at end of packet.
    int encode(const int val);                          ///<  Encode a number at end of packet.
    int encode(const char val);                         ///<  Encode a number at end of packet.
    int encode(const short val);                        ///<  Encode a number at end of packet.
    int encode(const float val);                        ///<  Encode a number at end of packet.
    int encode(const double val);                       ///<  Encode a number at end of packet.
    int encode(const unsigned int val);                 ///<  Encode a number at end of packet.
    int encode(const unsigned char val);                ///<  Encode a number at end of packet.
    int encode(const unsigned short val);               ///<  Encode a number at end of packet.
    int encode(const int64 val);                        ///<  Encode a number at end of packet.
    int encode(const uint64 val);                       ///<  Encode a number at end of packet.
    int encode(const unsigned char *p, 
               const unsigned int size);                ///<  Copy entire buffer at end of packet.
    int fill(const unsigned char v, 
             const unsigned int len);                   ///<  Fill up the rest of the packet with v until len reached.
    int decode(int &val);                               ///<  Decode a number from packet.
    int decode(short &val);                             ///<  Decode a number from packet.
    int decode(float &val);                             ///<  Decode a number from packet.
    int decode(double &val);                            ///<  Decode a number from packet.
    int decode(char &val);
    int decode(unsigned int &val);                      ///<  Decode a number from packet.
    int decode(unsigned char &val);                     ///<  Decode a number from packet.
    int decode(unsigned short &val);                    ///<  Decode a number from packet.
    int decode(uint64 &val);                            ///<  Decode a number from packet.
    int decode(int64 &val);                             ///<  Decode a number from packet.
    int decode(unsigned char *p, 
               const unsigned int size);                ///<  Extract copy of data from packet.
    int encode(const unsigned int pos,
               int val);                                ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const short val);                        ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const float val);                        ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const double val);                       ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const unsigned int val);                 ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const unsigned char val);                ///<  Encode a number atspecific point in packet.
    int encode(const unsigned int pos,
               const unsigned short val);               ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const int64 val);                        ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const uint64 val);                       ///<  Encode a number at specific point in packet.
    int encode(const unsigned int pos,
               const unsigned char *p, 
               const unsigned int size);                ///<  Copy entire buffer at specific point in packet.
    int decode(const int pos, int &val) const;
    int decode(const int pos, float &val) const;
    int decode(const int pos, double &val) const;
    int decode(const int pos, short &val) const;
    int decode(const int pos, unsigned int &val) const;
    int decode(const int pos, unsigned char &val) const;
    int decode(const int pos, unsigned short &val) const;
    int decode(const int pos, int64 &val) const;
    int decode(const int pos, uint64 &val) const;
    
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      int &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      short &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      float &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      double &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      unsigned int &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      int64 &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      uint64 &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      unsigned char &val,
                      const int border = 0);            
    static int decode(const unsigned char *buff, 
                      const unsigned int size,
                      const unsigned int pos,
                      unsigned short &val,
                      const int border = 0);                  ///<  Decode a number from packet.
    static int machineEndianness();                           ///<  Get machine endianness.
    int getByteOrder() const { return mByteOrder; }           ///<  Get byte order being used for encoding/decoding.
    void destroy();                                           ///<  Delete all content.
    void clear();                                             ///<  Clears packet length and sets encode/decode positions to 0.
    bool removeRange(unsigned int start, int end = -1);       ///<  Deletes a portion of the packet.
    void reserve(const unsigned int size);                    ///<  Pre-allocate additional memory.
    void setByteOrder(const int endianness);                  ///<  Set byte order for encoding/decoding.
    unsigned int getDecodePos() const { return mDecodePos; }  ///<  Get the current position in packet for decoding.
    int setDecodePos(const unsigned int pos = 0);             ///<  Set position in packet for decoding.
    unsigned int getEncodePos() const { return mEncodePos; }  ///<  Get the current position in packet for decoding.
    int setEncodePos(const unsigned int pos = UINT_MAX);      ///<  Set position in packet for decoding.
    unsigned int reserved() const { return mReserved; }       ///<  Amount of memory reserved by packet for encoding.
    unsigned int length() const { return mLength; }           ///<  Size/length of packet in bytes.
    unsigned int size() const { return mLength; }             ///<  Size/length of packet in bytes.
    const unsigned char *ptr() const { return mPacket; }      ///<  Pointer to raw data.
    operator const unsigned char *() const {return mPacket;}  ///<  Implicit operator for raw data.
    Packet &operator=(const Packet &another);                 ///<  Set equal to another packet.
    inline static int convertByteOrder(const int a);
    inline static short convertByteOrder(const short a);
    inline static unsigned int convertByteOrder(const unsigned int a);
    inline static unsigned short convertByteOrder(const unsigned short a); 
    inline static int64 convertByteOrder(const int64 a);
    inline static uint64 convertByteOrder(const uint64 a);
protected:    
    void growPacket(unsigned int size);
    int mByteOrder;              ///<  What byte order to use for encoding.
    bool mWrapperPacket;         ///<  Is this a wrapper packet?
    static int mEndianness;      ///<  Endiannes of the system.
    unsigned char *mPacket;      ///<  Packet data.
    unsigned int mLength;        ///<  Length of packet.
    unsigned int mDecodePos;     ///<  Current position in packet for decoding.
    unsigned int mEncodePos;     ///<  Current position in packet for encoding.
    unsigned int mReserved;      ///<  Reserved memory space.
    
    /// Writes a value to buffer in packet.
    template <class T>
    inline int writePacket(Packet *p, T val)
    {
        if (!p)
            return 0;

        T data;
        unsigned int bytes = sizeof(data);
        if(p->mByteOrder != p->mEndianness && bytes > 1)
        {
            switch(bytes)
            {
            case SIZE_OF_INT16:
                data = (T)convertByteOrder((short)(val));
                break;
            case SIZE_OF_INT32:
                data = (T)convertByteOrder((int)(val));
                break;
            case SIZE_OF_INT64:
                data = (T)convertByteOrder((int64)(val));
                break;
            default:
                data = val;  //  Don't do anything for a single byte.
                break;
            };
        }
        else
            data = val;
        if(p->mEncodePos + bytes + 1 >= p->mReserved)
            if(mWrapperPacket)
                return 0;
            else
                p->growPacket(p->mEncodePos + bytes + 1);
        memcpy(&p->mPacket[p->mEncodePos], &data, bytes);
        p->mEncodePos += bytes;
        if (p->mEncodePos > p->mLength)
        {
            p->mLength = p->mEncodePos;
            p->mPacket[p->mLength] = '\0';
        }

        return bytes;
    }


    //  Read and decode value from packet.
    template <class T>
    inline int readPacket(Packet *p, T &val)
    {
        if(!p->mPacket)
            return 0;

        unsigned int bytes = sizeof(val);
        if(p->mDecodePos + bytes <= p->mLength)
        {
            memcpy(&val, &p->mPacket[p->mDecodePos], bytes); 
            if(p->mByteOrder != p->mEndianness && bytes > 1)
            {
                switch(bytes)
                {
                case SIZE_OF_INT16:
                    val = (T)convertByteOrder((short)(val));
                    break;
                case SIZE_OF_INT32:
                    val = (T)convertByteOrder((int)(val));
                    break;
                case SIZE_OF_INT64:
                    val = (T)convertByteOrder((int64)(val));
                    break;
                default:
                    val = val;  //  Don't do anything for a single byte.
                    break;
                };
            }
            p->mDecodePos += bytes;
            return bytes;
        }
        else
            return 0;
    }


    //  Read and decode from a specified position in packet.
    template <class T>
    inline int readPacketAtPos(const Packet * const p, unsigned int pos, T &val) const
    {
        if(!p->mPacket || pos < 0 && pos >= p->mLength)
            return 0;

        unsigned int bytes = sizeof(val);
        if(pos + bytes <= p->mLength)
        {
            memcpy(&val, &p->mPacket[pos], bytes); 
            if(p->mByteOrder != p->mEndianness && bytes > 1)
            {
                switch(bytes)
                {
                case SIZE_OF_INT16:
                    val = (T)convertByteOrder((short)(val));
                    break;
                case SIZE_OF_INT32:
                    val = (T)convertByteOrder((int)(val));
                    break;
                case SIZE_OF_INT64:
                    val = (T)convertByteOrder((int64)(val));
                    break;
                default:
                    val = val;  //  Don't do anything for a single byte.
                    break;
                };
            }
            return bytes;
        }
        else
            return 0;
    }

    template <class T>
    inline int writePacketAtPos(Packet *p, unsigned int pos, T val)
    {
        if (!p || pos < 0)
            return 0;

        T data;
        unsigned int bytes = sizeof(data);
        if(p->mByteOrder != p->mEndianness && bytes > 1)
        {
            switch(bytes)
            {
            case SIZE_OF_INT16:
                data = (T)convertByteOrder((short)(val));
                break;
            case SIZE_OF_INT32:
                data = (T)convertByteOrder((int)(val));
                break;
            case SIZE_OF_INT64:
                data = (T)convertByteOrder((int64)(val));
                break;
            default:
                data = (T)val;  //  Don't do anything for a single byte.
                break;
            };
        }
        else
            data = val;
        if(pos + bytes + 1 >= p->mReserved)
            if(p->mWrapperPacket)
                return 0;
            else
                p->growPacket(pos + bytes + 1);
        memcpy(&p->mPacket[pos], &data, bytes);
        if (pos > p->mLength)
        {
            p->mLength = pos + bytes;
            p->mPacket[p->mLength] = '\0';
        }
        return bytes;
    }
};

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline short Packet::convertByteOrder(const short a)
{
    return ((a & 0xFF00) >> 8) | ((a & 0x00ff) << 8);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline unsigned short Packet::convertByteOrder(const unsigned short a)
{
    return ((a & 0xFF00) >> 8) | ((a & 0x00ff) << 8);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline int Packet::convertByteOrder(const int a)
{
    return ((a & 0xFF000000) >> 24)|
           ((a & 0x00FF0000) >> 8) |
           ((a & 0x0000FF00) << 8) |
           ((a & 0x000000FF) << 24);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline unsigned int Packet::convertByteOrder(const unsigned int a)
{
    return ((a & 0xFF000000) >> 24)|
           ((a & 0x00FF0000) >> 8) |
           ((a & 0x0000FF00) << 8) |
           ((a & 0x000000FF) << 24);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline uint64 Packet::convertByteOrder(const uint64 a)
{
    return  ( (a & 0xFF00000000000000ULL) >> 56) |
            ( (a & 0x00FF000000000000ULL) >> 40) |
            ( (a & 0x0000FF0000000000ULL) >> 24) |
            ( (a & 0x000000FF00000000ULL) >> 8 ) |
            ( (a & 0x00000000FF000000ULL) << 8 ) |
            ( (a & 0x0000000000FF0000ULL) << 24) |
            ( (a & 0x000000000000FF00ULL) << 40) |
            ( (a & 0x00000000000000FFULL) << 56);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the endianness of the data.
///
///   \return The value with it's byte order reversed.
///
////////////////////////////////////////////////////////////////////////////////////
inline int64 Packet::convertByteOrder(const int64 a)
{
    return  ( (a & 0xFF00000000000000ULL) >> 56) |
            ( (a & 0x00FF000000000000ULL) >> 40) |
            ( (a & 0x0000FF0000000000ULL) >> 24) |
            ( (a & 0x000000FF00000000ULL) >> 8 ) |
            ( (a & 0x00000000FF000000ULL) << 8 ) |
            ( (a & 0x0000000000FF0000ULL) << 24) |
            ( (a & 0x000000000000FF00ULL) << 40) |
            ( (a & 0x00000000000000FFULL) << 56);
}

#define ULONGLONG_RANGE (double) 1.8446744073709552e19      ///<  Range of values for an uint64.
#define LONGLONG_RANGE  9223372036854775800.0               ///<  Range of values for a int64.
#define UINTEGER_RANGE  4294967295.0                        ///<  Range of values for an unsigned int.
#define INTEGER_RANGE   4294967294.0                        ///<  Range of values for an integer.
#define USHORT_RANGE    65535.0                             ///<  Range of values for an unsigned short.
#define SHORT_RANGE     65534.0                             ///<  Range of values for a short.
#define SCALED_EPSILON  .00000000000000000000001            ///<  Used to prevent divide by 0.


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, int &scaled)
{
    double sf = (upper - lower)/INTEGER_RANGE;
    double bias = (lower + upper)/2.0;
    scaled = (int)((real - bias)/(sf + SCALED_EPSILON) + .5);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, unsigned int &scaled)
{
    double sf = (upper - lower)/UINTEGER_RANGE;
    double bias = lower;
    scaled = (unsigned int)((real - bias)/(sf + SCALED_EPSILON) + .5);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, short &scaled)
{
    double sf = (upper - lower)/SHORT_RANGE;
    double bias = (lower + upper)/2.0;
    scaled = (short)((real - bias)/(sf + SCALED_EPSILON) + .5);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, unsigned short &scaled)
{
    double sf = (upper - lower)/USHORT_RANGE;
    double bias = lower;
    scaled = (unsigned short)((real - bias)/(sf + SCALED_EPSILON) + .5);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, int64 &scaled)
{
    scaled = (int64)((real - (upper + lower)/2.0)*2*((LONGLONG_RANGE)/(upper - lower + SCALED_EPSILON)) + .5);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(int scaled, double upper, double lower, double &real)
{
    double sf = (upper - lower)/INTEGER_RANGE;
    double bias = (lower + upper)/2.0;
    real = scaled*sf + bias;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(unsigned int scaled, double upper, double lower, double &real)
{
    double sf = (upper - lower)/UINTEGER_RANGE;
    double bias = lower;
    real = scaled*sf + bias;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(short scaled, double upper, double lower, double &real)
{
    double sf = (upper - lower)/SHORT_RANGE;
    double bias = (lower + upper)/2.0;
    real = scaled*sf + bias;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(unsigned short scaled, double upper, double lower, double &real)
{
    double sf = (upper - lower)/USHORT_RANGE;
    double bias = lower;
    real = scaled*sf + bias;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(int64 scaled, double upper, double lower, double &real)
{
    real = scaled/2.0*((upper - lower)/LONGLONG_RANGE) + (upper + lower)/2.0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a real number to a scaled integer value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertRealToInteger(double real, double upper, double lower, uint64 &scaled)
{
    double factor = (upper-lower)/ULONGLONG_RANGE;
    double bias = lower;
                
    // Don't pass the upper and lower bounds.
    if(real < lower) 
        real = lower;
    if(real > upper) 
        real = upper;
                       
    scaled = (uint64)((real - bias)/factor);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts a scaled integer number to a real value.
///
///   This format is used in JAUS and other libraries to scale real numbers.
///
///   \param real The real number to convert
///   \param upper The upper limit of the real number
///   \param lower The lower limit of the real number
///   \param scaled The scaled integer
///
////////////////////////////////////////////////////////////////////////////////////
inline void convertIntegerToReal(uint64 scaled, double upper, double lower, double &real)
{
    double factor = (upper - lower)/ULONGLONG_RANGE;
    real = scaled*factor + lower;
}


#endif
#endif
/*  End of File */
