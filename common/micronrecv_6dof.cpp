////////////////////////////////////////////////////////////////////////////////
///
///   Filename: micronrecv.cpp
/// 
///   Copyright (C) 2008-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Medical Instrumentation Lab       Micron Project
///
///   Description: Socket/Packet code for receiving data from Micron over UDP
///   --------------------------------------------------------------------------
///	  Contains classes for incoming communication with Micron. The socket class
///	  receives packet classes in a background thread. Packets are queued
///	  chronologically as they are received and made available through the
///	  interface in a non-blocking manner. Packets contain information about the
///	  status of Micron (positions, pose, stacks, flags, etc). See the Network
///	  cluster on the front panel of micron_ui.vi to configure Micron to send
///	  status updates.
///
////////////////////////////////////////////////////////////////////////////////

//#include "micronrecv.h"
#include "micronrecv_6dof.h"


#define WRITE_DATA(fp,x) fwrite(&x, sizeof(x), 1, fp)
#define READ_DATA(fp,x) fread(&x, sizeof(x), 1, fp)

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Initialize the micron receiving socket, thread, and fps counter
///
////////////////////////////////////////////////////////////////////////////////
MicronRecvSocket::MicronRecvSocket() : Thread("MicronRecvSocket"), mFPS(4000)
{
	init();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destroy the socket (wait a bit for clean exit of thread)
///
////////////////////////////////////////////////////////////////////////////////
MicronRecvSocket::~MicronRecvSocket()
{
	destroy();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Initialize member variables to their default
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::zero()
{
	mPort = LIN_PORT;
	mBlocking = LIN_BLOCKING;
	mRecvNumber = 0;
	mFrequency = 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Initialize member variables
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::init()
{
	zero();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Stops the thread (waits a bit for clean exit), then clears object
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::destroy()
{
	this->stopThread(LIN_WAIT_KILL_TIME);

	mMutex.enter();
	mPackets.clear();
	mMutex.leave();
	
	init();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Threaded function that runs in the background at 2kHz receiving
///          status updates from Micron and copying it into the receive queue
///
///   This function runs in the background at a high priority listening for
///   packets the Micron realtime target has sent over. Packets should be
///   arriving at ~1 kHz. Once a packet is received, it is parsed into a
///   MicronRecvPacket structure with all the information from the Micron
///   realtime target (tip position, etc, etc) and copied into the received
///   packets queue, which can then be removed with the MicronRecvSocket::recv
///   function.
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::execute()
{
	WSADATA wsaData;
	sockaddr_in recvAddr;
	sockaddr_in senderAddr;
	int addrSize = sizeof(senderAddr);
	int length = 0;
	char buff[LIN_BUFF_LEN];
	MicronRecvPacket p;
	int flags;
	unsigned long blocking = mBlocking;
	int err;
	float arr[LIN_BUFF_LEN];

	// Raise the priority of this thread some so we get better performance
#if defined(WIN32) || defined(WINDOWS)
	Thread::setPriority(5);
	
	// Initialize Winsock (Windows specific)
	WSAStartup(MAKEWORD(2,2), &wsaData);
#endif

	// Create a receiver socket to receive datagrams
	mSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// Set this to a non-blocking socket if desired
	// fcntl(mSock, F_SETFL,O_NONBLOCK|FASYNC) under UNIX
	ioctlsocket(mSock, FIONBIO, &blocking);

	// Bind the socket to any address and the specified port.
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(mPort);
	recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(mSock, (SOCKADDR *) &recvAddr, sizeof(recvAddr));

	while (!this->quitFlag())
	{
		// This is a blocking call so that's why we are in a separate thread
		length = recvfrom(mSock, buff, LIN_BUFF_LEN, 0, (SOCKADDR *)&senderAddr, &addrSize);

		// If we are in non-blocking mode, no data will result in a WSAEWOULDBLOCK, 
		// so wait a bit and try again
		err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK)
		{
			Sleep(LIN_SLEEP_TIME);
			continue;
		}

		// We got something!
		if (length > 0)
		{
			// Null terminate (important if sending strings!)
			buff[length] = 0;

			// Ignore the first two set of 4 bytes because they are LabVIEW headers (EDIT: Removed LabVIEW headers!)
			const int offset = 0;
			for (int i = offset; i < length/4; i++)
			{
				// Adjust from network byte order to host order
				int *dst = (int*)arr+i-offset;
				int *src = (int*)buff+i;
				*dst = ntohl(*src);
			}

			// Parse information from giant array into packet
			// NOTE: If the LabVIEW code changes, this must also change!
			// First three numbers: magic value, version number, triplet count
			// seq num, trace index, reserved

			memset(&p, 0, sizeof(MicronRecvPacket) - MICRON_RECV_PACKET_PADDING);
			if (arr[0] == micron_trace_data_magic && arr[1] == micron_trace_data_version && (int)arr[2] == micron_trace_length)
			{
				memcpy(&p, &arr[3], micron_trace_length*3*sizeof(float));
			}

			// Last two floats are sequence_num and trace_index
			p.sequence_num = *((unsigned int*)&arr[3 + micron_trace_length*3 + 0]);
			p.trace_index = *((int*)&arr[3 + micron_trace_length*3 + 1]);
			
			// Trace data encodes special bit flags about the status of Micron
			flags = (int)p.status_flags[0]; // status flags moved to p.statusFlags
			p.cancellation = !!(flags & 1<<LIN_BIT_CANCELLATION); ///// TODO: Check these!
			p.scaling = !!(flags & 1<<LIN_BIT_SCALING);
			p.saturated = !!(flags & 1<<LIN_BIT_SATURATED);
			p.valid_pos = !!(flags & 1<<micron_valid_position_flag);
			p.validPos = p.valid_pos;
			for (int i = 0; i < 32; i++)
			{
				p.flags[i] = !!(flags & 1<<i);
			}

			// Add to the queue of retrieved packets
			mMutex.enter();
			mPackets.enqueue(p);
			mMutex.leave();

			// Update number of received packets and frequency information
			mRecvNumber++;
			mFrequency = mFPS.click();
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Close the socket, we are done
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::cleanup()
{
	// We probably won't get here if the socket is blocking
	closesocket(mSock);
	WSACleanup();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Starts the background thread listening for Micron data
///
///   \param port Port that Micron is sending data on (default is LIN_PORT)
///   \param blocking Whether or not to use a blocking thread (default is false)
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvSocket::start(int port /*= LIN_PORT*/, bool blocking /*= false*/)
{
	if (!isActive())
	{
		mPort = port;
		mBlocking = blocking;
		this->createThread();
	}
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Non-blocking transfer of a packet from the queue of received Micron
///          packets to input variable
///
///   If queue of received packets is non-empty, dequeue chronologically and
///   copy to input parameter. If the queue is empty, return immediately.
///
///   \param packet Packet to fill with data if queue is not empty
///   \result True if queue was non-empty and a packet was copied, false
///           otherwise
///
////////////////////////////////////////////////////////////////////////////////
int MicronRecvSocket::recv(MicronRecvPacket &packet)
{
	bool result = false;

	mMutex.enter();
	if (mPackets.size())
	{
		packet = mPackets.dequeue();
		result = true;
	}
	mMutex.leave();

	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies one recieved Micron packet to another
///
////////////////////////////////////////////////////////////////////////////////
MicronRecvPacket &MicronRecvPacket::operator=(const MicronRecvPacket &rhs)
{
	if (this != &rhs)
	{
		memcpy(this, &rhs, sizeof(MicronRecvPacket)  - MICRON_RECV_PACKET_PADDING);
	}

	return *this;
}

void MicronRecvPacket::write(FILE *fp)
{
	if (!fp)
		return;

	fwrite(this, sizeof(MicronRecvPacket) - MICRON_RECV_PACKET_PADDING, 1, fp);
}

void MicronRecvPacket::read(FILE *fp)
{
	if (!fp)
		return;
	
	fread(this, sizeof(MicronRecvPacket) - MICRON_RECV_PACKET_PADDING, 1, fp);
}

MicronRecvPacket::MicronRecvPacket()
{
	clear(); 
	if (MICRON_BACKWARDS_COMPATABILITY)
	{
		tipPos = this->position_tip;
		centralTipPos = this->null_pos_tip;
		analogData = this->analog_data_0;
	}
}

void MicronRecvPacket::clear()
{
	memset(this, 0, sizeof(MicronRecvPacket) - MICRON_RECV_PACKET_PADDING);
}
