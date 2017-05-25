////////////////////////////////////////////////////////////////////////////////
///
///   Filename: micronsend.cpp
/// 
///   Copyright (C) 2008-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Medical Instrumentation Lab       Micron Project
///
///   Description: Socket/Packet code for sending data to Micron over UDP
///   --------------------------------------------------------------------------
///	  Contains classes for outgoing communication with Micron. The socket class
///	  sends packet classes chronologically in a background thread to control the
///	  position of Micron. See the Network cluster on the front panel of
///	  micron_ui.vi to configure Micron to receive commands.
///
////////////////////////////////////////////////////////////////////////////////

#include "micronsend.h"

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates the Micron send socket and thread
///
////////////////////////////////////////////////////////////////////////////////
MicronSendSocket::MicronSendSocket() : Thread("MicronSendSocket"), mFPS(2000)
{
	init();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Disconnects from Micron and stops the thread
///
////////////////////////////////////////////////////////////////////////////////
MicronSendSocket::~MicronSendSocket()
{
	destroy();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets member variables to their default
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::zero()
{
	mPort = LOUT_PORT;
	mBlocking = LOUT_BLOCKING;
	strcpy(mAddress, "");
	mSendNumber = 0;
	mFrequency = 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Initializes member variables
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::init()
{
	zero();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Stop the send thread and clear packets
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::destroy()
{
	this->stopThread(LOUT_WAIT_KILL_TIME);

	mMutex.enter();
	mPackets.clear();
	mMutex.leave();
	
	init();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Threaded function that runs in the background at 1 kHz sending
///          commands to the Micron realtime target
///
///   When turned on via the Network settings on the LabVIEW front panel of
///   Micron's user interface VI, Micron can be commanded remotely by listening
///   to packets on a host port. This function runs in the background, waiting
///   for packets to be copied into the send queue by the function
///   MicronSendSocket::send. As soon as packets are copied into the send queue,
///   they are dispatched chronologically to the Micron realtime target.
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::execute()
{
	WSADATA wsaData;
	sockaddr_in sendAddr;
	int addrSize = sizeof(sendAddr);
	int length = 0;
	unsigned long blocking = mBlocking;
	MicronSendPacket p;
	bool sendPacket = false;
	int err = 0;
	float arr[LOUT_BUFF_LEN];

	// Raise the priority of this thread some so we get better performance
#if defined(WIN32) || defined(WINDOWS)
	Thread::setPriority(5);
#endif
	
	// Initialize Winsock (Windows specific)
	WSAStartup(MAKEWORD(2,2), &wsaData);

	// Create a receiver socket to receive datagrams
	mSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// Set this to a non-blocking socket if desired
	// fcntl(mSock, F_SETFL,O_NONBLOCK|FASYNC) under UNIX
	ioctlsocket(mSock, FIONBIO, &blocking);

	// Bind the socket to any address and the specified port.
	sendAddr.sin_family = AF_INET;
	sendAddr.sin_port = htons(mPort);
	sendAddr.sin_addr.s_addr = inet_addr(mAddress);

	while (!this->quitFlag())
	{
		sendPacket = false;
		mMutex.enter();
		if (mPackets.size())
		{
			mPackets.dequeue(p);
			sendPacket = true;
		}
		mMutex.leave();

		if (sendPacket)
		{
			// Encode goalPos, useGoalPos, and extraInfo into packet using network-byte order
			//int *dst = (int*)arr;
			//int *src = (int*)p.goalPos;
			//for (int i = 0; i < 3; i++)
			//	*dst++ = htonl(*src++);
			//src = (int*)&p.useGoalPos;
			//*dst++ = htonl(*src);
			//src = (int*)p.extraInfo;
			//for (int i = 0; i < 12; i++)
			//	*dst++ = htonl(*src++);


			int *dst = (int*)arr;
			int *src = (int*)p.goalPos;
			for (int i = 0; i < 3; i++)
				*dst++ = htonl(*src++);

			src = (int*)&p.useGoalPos;
			*dst++ = htonl(*src);

			
			src = (int*)p.goalPosRCM;
			for (int i = 0; i < 3; i++)
				*dst++ = htonl(*src++);


			src = (int*)&p.fireLaser;
			*dst++ = htonl(*src);

			src = (int*)p.extraInfo;
			for (int i = 0; i < 8; i++)
				*dst++ = htonl(*src++);


			//// Send packet with goalPos, whether to command Micron to the goalPos, and extra info
			err = sendto(mSock, (const char*)arr, (int)(sizeof(float)*((float*)dst - arr)), 0, (SOCKADDR *)&sendAddr, addrSize);

			// Update number of sent packets and frequency information
			mSendNumber++;
			mFrequency = mFPS.click();
		}
		else
		{
			Sleep(LOUT_SLEEP_TIME);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Cleans up thread by closing socket connections
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::cleanup()
{
	// We probably won't get here if the socket is blocking
	closesocket(mSock);
	WSACleanup();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Starts Micron thread sending in the background
///
///   \param address Address of the Micron realtime target machine
///   \param port Port to send data over (default LOUT_PORT)
///   \param blocking Create blocking socket (default false)
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendSocket::start(const char *address, int port /*= LOUT_PORT*/, bool blocking /*= false*/)
{
	if (!isActive())
	{
		mPort = port;
		mBlocking = blocking;
		strcpy(mAddress, address);
		this->createThread();
	}
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Enqueue a packet to be sent to the realtime target
///
///   \param packet Micron send packet to send
///
////////////////////////////////////////////////////////////////////////////////
int MicronSendSocket::send(MicronSendPacket &packet)
{
	mMutex.enter();
	mPackets.enqueue(packet);
	mMutex.leave();

	return true;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clear Micron send packet
///
////////////////////////////////////////////////////////////////////////////////
void MicronSendPacket::clear()
{
	//for (int i = 0; i < 3; i++)
	//	goalPos[i] = 0;
	//for (int i = 0; i < 12; i++)
	//	extraInfo[i] = 0;
	//useGoalPos = 0;

	for (int i = 0; i < 3; i++)
	{
		goalPos[i] = 0;
		goalPosRCM[i] = 0;
	}
	for (int i = 0; i < 8; i++)
		extraInfo[i] = 0;

	useGoalPos = 0;
	fireLaser = 0;


}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies one Micron send packet to another
///
////////////////////////////////////////////////////////////////////////////////
MicronSendPacket & MicronSendPacket::operator=( const MicronSendPacket &rhs )
{
	//if (this != &rhs)
	//{
	//	for (int i = 0; i < 3; i++)
	//		goalPos[i] = rhs.goalPos[i];
	//	for (int i = 0; i < 12; i++)
	//		extraInfo[i] = rhs.extraInfo[i];
	//	useGoalPos = rhs.useGoalPos;
	//}

	if (this != &rhs)
	{
		for (int i = 0; i < 3; i++)
		{
			goalPos[i] = rhs.goalPos[i];
			goalPosRCM[i] = rhs.goalPosRCM[i];
		
		}
		for (int i = 0; i < 8; i++)
			extraInfo[i] = rhs.extraInfo[i];

		useGoalPos = rhs.useGoalPos;
		fireLaser = rhs.fireLaser;
	}

	return *this;
}