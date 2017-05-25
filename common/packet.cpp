/*==================================================================================

    Filename:  packet.cpp

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
#include "packet.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <limits.h>

//  Save some copy paste time...
#define CALC_CHECK_AND_CONVERT_ENDIANNESS          \
        if(Packet::mEndianness == -1)              \
            mEndianness = machineEndianness();     \
        if(border != Packet::mEndianness)          \
            val = convertByteOrder(val);          \


int Packet::mEndianness = -1;   ///<  System Endianness

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Packet::Packet()
{
    mLength = 0;
    mEncodePos = 0;
    mDecodePos = 0;
    mReserved = 0;
    mByteOrder = PACKET_LITTLE_ENDIAN;
    mWrapperPacket = false;
    mPacket = NULL;
    //  If the machine endiannes has not
    //  been determined yet, set it.
    if(mEndianness == -1)
    {
        mEndianness = machineEndianness();
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Packet::Packet(const Packet &another)
{
    mLength = 0;
    mEncodePos = 0;
    mDecodePos = 0;
    mReserved = 0;
    mByteOrder = PACKET_LITTLE_ENDIAN;
    mWrapperPacket = false;
    mPacket = NULL;
    //  If the machine endiannes has not
    //  been determined yet, set it.
    if(mEndianness == -1)
    {
        mEndianness = machineEndianness();
    }
    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Build packet from existing data.
///
///   If the wrapper flag is set to true, then the packet will use
///   the existing memory allocation pointed to by buff for encoding and
///   decoding of the packet.  However it will not resize/grow or delete
///   the memory at all.
///
///   If the wrapper flag is false, then a copy of the data is made.
///
///   \param buff Data to use for packet.
///   \param len Length of buff.
///   \param wrapper Create a wrapper packet.
///
////////////////////////////////////////////////////////////////////////////////////
Packet::Packet(const unsigned char *buff,
               const unsigned int len,
               const bool wrapper)
{
    mLength = 0;
    mEncodePos = 0;
    mDecodePos = 0;
    mReserved = 0;
    mByteOrder = PACKET_LITTLE_ENDIAN;
    mWrapperPacket = wrapper;
    mPacket = NULL;
    //  If the machine endiannes has not
    //  been determined yet, set it.
    if(mEndianness == -1)
    {
        mEndianness = machineEndianness();
    }
    assert(buff && len);
    if(!mWrapperPacket)
    {
        mPacket = new unsigned char[len];
        assert(mPacket);
        memcpy(mPacket, buff, len*sizeof(unsigned char));
        mLength = len;
        mReserved = len;
        mEncodePos = mLength;
    }
    else
    {
        mPacket = ((unsigned char *)(buff));
        mReserved = len;
        mLength = len;
        mEncodePos = 0;
    }
    
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Packet::~Packet()
{
    destroy();
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data in another packet to the end of the packet.
///
///   \param another Packet to add to end.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const Packet &another)
{
    if(mEncodePos + another.mLength + 1 >= mReserved)
        growPacket(mEncodePos + another.mLength + 1);
    memcpy(&mPacket[mEncodePos], another.mPacket, sizeof(unsigned char)*another.mLength);
    mEncodePos += another.mLength;
    if (mEncodePos >= mLength)
        mLength = mEncodePos;
    mPacket[mLength] = '\0';
    return another.mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const int val)
{
    return writePacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const short val)
{
    return writePacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function <b>DOES NOT</b> convert or do any byte order changes
///   for floating point numbers.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const float val)
{
    int temp;
    //  Must copy the memory to a buffer
    //  first.
    memcpy(&temp, &val, sizeof(val));
    return writePacket(this, temp);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function <b>DOES NOT</b> convert or do any byte order changes
///   for floating point numbers.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const double val)
{
    int64 temp;
    //  Must copy the memory to a buffer
    //  first.
    memcpy(&temp, &val, sizeof(val));
    return writePacket(this, temp);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int val)
{
    return writePacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const char val)
{
    return writePacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned char val)
{
    if(this->mEncodePos + sizeof(unsigned char) + 1 > mReserved)
    {
        if(mWrapperPacket)
            return false;
        else
            growPacket(this->mReserved + sizeof(unsigned char) + 1);
    }
    mPacket[mEncodePos] = val;
    mPacket[mEncodePos + 1] = '\0';
    mEncodePos++;
    if(mEncodePos > mLength)
        mLength++;

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned short val)
{
    return writePacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const int64 val)
{
    return writePacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const uint64 val)
{
    return writePacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Encode the data to the end of the packet.
///
///   \param p The data to add to end of packet.
///   \param size The size of the data in bytes.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned char *p,
                   const unsigned int size)
{
    assert(size >= 0);
    if(mEncodePos + size + 1 >= mReserved)
        growPacket(mEncodePos + size + 1);
    memcpy(&mPacket[mLength], p, sizeof(unsigned char)*size);
    mEncodePos += size;
    if (mEncodePos > mLength)
        mLength = mEncodePos;
    mPacket[mLength] = '\0';
    return size;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Fills the packet up to the value of len with the value of v.  This 
///   can be done when creating fixed sized packets used, and extra filler
///   is needed to reach the fixed size.
///
///   \param v The value to fill the packet memory with.
///   \param len Position after the end of the packet to fill to.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::fill(const unsigned char v, const unsigned int len)
{
    if(len < mLength)
        return 0;
    else
    {
        if(len + 1 > mReserved)
            growPacket(len + 1);
        memset(&mPacket[mLength], v, sizeof(unsigned char)*(len - mLength));
        mLength += len - mLength;
        mPacket[mLength] = '\0';
        return (len - mLength);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(int &val)
{
    return readPacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(int64 &val)
{
    return readPacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(uint64 &val)
{
    return readPacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(short &val)
{
    return readPacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function <b>DOES NOT</b> convert any byte order information for
///   floating point numbers.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(float &val)
{
    int temp;
    int result = 0;
    if( (result = readPacket(this, temp)) > 0 )
    {
        memcpy(&val, &temp, sizeof(val));
        return result;
    }
    return result;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function <b>DOES NOT</b> convert any byte order information for
///   floating point numbers.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(double &val)
{
    int64 temp;
    int result = 0;
    if( (result = readPacket(this, temp)) > 0 )
    {
        memcpy(&val, &temp, sizeof(val));
        return result;
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(unsigned int &val)
{
    return readPacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(unsigned char &val)
{
    return readPacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(char &val)
{
    return readPacket(this, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param val The data decoded.
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(unsigned short &val)
{
    return readPacket(this, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from the current decode position in the packet.
///
///   The decode position is also updated so that it is after the newly
///   decoded data.  (Position is advanced by the number of bytes decoded). To
///   change the decode position use the setPos function.
///
///   \param p Buffer to copy data to.
///   \param size The number of bytes to extract from packet.
///
///   \return Number of bytes actually decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(unsigned char *p,
                   const unsigned int size)
{
    if(!mPacket)
        return 0;

    unsigned int bytes = size;
    assert(p && size > 0);
    if(mDecodePos + bytes >= mLength)
        bytes = mLength - mDecodePos;
    if(bytes)
    {
        memcpy(p, &mPacket[mDecodePos], bytes); 
        mDecodePos += bytes;
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const int64 val)
{
    return writePacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const int val)
{
    return writePacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const uint64 val)
{
    return writePacketAtPos(this, pos, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const short val)
{
    return writePacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const float val)
{
    int temp;
    memcpy(&temp, &val, sizeof(float));
    return writePacketAtPos(this, pos, temp);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const double val)
{
    int64 temp;
    memcpy(&temp, &val, sizeof(double));
    return writePacketAtPos(this, pos, temp);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const unsigned int val)
{
    return writePacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const unsigned char val)
{
    return writePacketAtPos(this, pos, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param val The data to encode.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const unsigned short val)
{
    return writePacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Re-encode the data to a previously encoded part of the packet.
///
///   This is useful if you only need to change specific parts of the packet
///   after encoding the rest of your data.  However, you can only encode
///   to sections of the packet that already exist (ie. cannot encode after
///   end of packet).
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param pos Position in the packet to encode.
///   \param p The buffer to add to packet.
///   \param size of buffer.
///
///   \return Number of bytes encoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::encode(const unsigned int pos,
                   const unsigned char *p,
                   const unsigned int size)
{
    int encoded = 0;
    int oldPos = getEncodePos();
    setEncodePos(pos);
    encoded = encode(p, size);
    setEncodePos(oldPos);
    return encoded;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, int64 &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, uint64 &val) const
{
    return readPacketAtPos(this, pos, val);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, int &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, short &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, unsigned int &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, float &val) const
{
    int temp;
    int result = 0;
    if( (result = readPacketAtPos(this, pos, temp)) > 0)
    {
        memcpy(&val, &temp, sizeof(val));
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, double &val) const
{
    int64 temp;
    int result = 0;
    if( (result = readPacketAtPos(this, pos, temp)) > 0)
    {
        memcpy(&val, &temp, sizeof(val));
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, unsigned char &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the number at byte position within packet.
///
///   \param pos Starting byte position of the number to decode.
///   \param val The size and type of integer to decode.
///   
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const int pos, unsigned short &val) const
{
    return readPacketAtPos(this, pos, val);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   int &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   unsigned char &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   short &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   int64 &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   uint64 &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param flt The float data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   float &flt,
                   const int border)
{
    if(!buff)
        return 0;
    int val;
    unsigned int bytes = sizeof(flt);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        memcpy(&flt, &val, bytes);
        return bytes;
    }
    else
        return 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param dbl The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   double &dbl,
                   const int border)
{
    if(!buff)
        return 0;
    int64 val;
    unsigned int bytes = sizeof(dbl);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        memcpy(&dbl, &val, bytes);
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   unsigned int &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Decodes the data from a position in a buffer of data.
///
///   This function also converts the byte order if necessary based on
///   the setByteOrder function.
///
///   \param buff The buffer to decode data from.
///   \param len The length of the buffer.
///   \param pos The position in the buffer to decode from.
///   \param val The data decoded.
///   \param border The byte order of the encoded data. (PACKET_LITTLE_ENDIAN, 
///                 PACKET_BIG_ENDIAN/PACKET_NETWORK_BYTE_ORDER). 
///
///   \return Number of bytes decoded.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::decode(const unsigned char *buff,
                   const unsigned int len,
                   const unsigned int pos,
                   unsigned short &val,
                   const int border)
{
    if(!buff)
        return 0;

    unsigned int bytes = sizeof(val);
    if(pos + bytes <= len)
    {
        memcpy(&val, &buff[pos], bytes); 
        CALC_CHECK_AND_CONVERT_ENDIANNESS
        return bytes;
    }
    else
        return 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return PACKET_LITTLE_ENDIAN if the current machine's byte order is
///    little endian, otherwise PACKET_BIG_ENDIAN.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::machineEndianness()
{
   int i = 1;
   char *p = (char *) &i;
   if (p[0] == 1) // Lowest address contains the least significant byte
      return PACKET_LITTLE_ENDIAN;
   else
      return PACKET_BIG_ENDIAN;
}


//////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set what position to start decoding data from the packet.
///
///   \param pos The position to start decoding data from [0, packet size).
///
///   \return 0 on error, 1 if set properly.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::setDecodePos(const unsigned int pos)
{
    if(pos >= 0 && pos <= mLength)
    {
        mDecodePos = pos;
        return 1;
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set what position to start encoding data to the packet.
///
///   \param pos The position to start encoding data to [0, packet size), if
///   pos is set to UINT_MAX than position is set to end of packet (this is
///   default).
///
///   \return 0 on error, 1 if set properly.
///
////////////////////////////////////////////////////////////////////////////////////
int Packet::setEncodePos(const unsigned int pos)
{
    if ((unsigned int)pos >= 0 && (unsigned int)pos < mLength)
    {
        mEncodePos = pos;
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes all data allocated in the packet.
///
////////////////////////////////////////////////////////////////////////////////////
void Packet::destroy()
{
    if(mPacket && !mWrapperPacket)
    {
        delete[] mPacket;
        mPacket = NULL;
    }

    mLength = mEncodePos = mDecodePos = mReserved = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears the contents of the packet.
///
////////////////////////////////////////////////////////////////////////////////////
void Packet::clear()
{
    mLength = mDecodePos = mEncodePos = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Reserve memory for storing packet data.  This speeds up
///   use of packet because memory allocation happens less.
///
///   \param size Number of bytes to reserve.
///
////////////////////////////////////////////////////////////////////////////////////
void Packet::reserve(const unsigned int size)
{
    if(mWrapperPacket)
        return;
    assert(size > 0);
    if(mReserved == size)
        return;

    unsigned char *newPtr = new unsigned char[size];
    assert(newPtr);
    //  Clear the memory.
    memset(newPtr, 0, sizeof(unsigned char)*size);
    if(mLength > 0)
    {
        //  Try save data if possible.
        if(mLength < size && mPacket)
        {
            memcpy(newPtr, mPacket, sizeof(unsigned char)*mLength);
        }
        else
        {
            mLength = mDecodePos = mEncodePos = 0;
        }
    }

    mReserved = size;
    if(mPacket)
    {
        delete[] mPacket;
        mPacket = NULL;
    }
    mPacket = newPtr;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the byte order to encode integer data as.
///
///   \param endianness PACKET_LITTLE_ENDIAN, or PACKET_BIG_ENDIAN.
///
////////////////////////////////////////////////////////////////////////////////////
void Packet::setByteOrder(const int endianness)
{
    if( endianness == PACKET_LITTLE_ENDIAN ||
        endianness == PACKET_BIG_ENDIAN )
    {
        mByteOrder = endianness;
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
Packet &Packet::operator=(const Packet &another)
{
    if(this != &another && !mWrapperPacket)
    {
        if(mLength < another.mLength && another.mLength > 0)
        {
            reserve(another.mLength + 1);
        }
        if(another.mLength > 0 && another.mPacket)
        {
            //  Just in case.
            if(!mPacket)
            {
                reserve(another.mLength + 1);
            }
            memcpy(mPacket, another.mPacket, sizeof(unsigned char)*another.mLength);
        }
        mLength = another.mLength;
        mDecodePos = another.mDecodePos;
        mEncodePos = another.mEncodePos;
        mByteOrder = another.mByteOrder;
    }
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Grows the packet size to fit the size desired.
///
///   \param size The size to grow to fit.
///
////////////////////////////////////////////////////////////////////////////////////
void Packet::growPacket(const unsigned int size)
{
    assert(!mWrapperPacket);
    unsigned int newSize = mLength;
    while(newSize < size && newSize < UINT_MAX - PACKET_BLOCK_SIZE)
        newSize += PACKET_BLOCK_SIZE;
    reserve(newSize);
}


///////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes part of a packet, shifting remaining packet data left
///
///   \param start Starting byte to remove bytes
///   \param end Ending byte to remove bytes
///   \return True on success, false otherwise
///
///////////////////////////////////////////////////////////////////////////////
bool Packet::removeRange(unsigned int start, int end /*= -1*/)
{
    if (end == -1)
        end = mLength;

    if (start >= 0 && start < mLength && (unsigned int)end <= mLength && start < (unsigned int)end)
    {
        memcpy(mPacket+start, mPacket+end, mLength - (end - start));
        mLength -= end - start;
        return true;
    }
    return false;
}


/*  End of File */
