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

#include "micronrecv.h"

#if USE_MICRON_6DOF
#include "micronrecv_6dof.cpp"
#else

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
///   arriving at ~2 kHz. Once a packet is received, it is parsed into a
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
#endif
	
	// Initialize Winsock (Windows specific)
	WSAStartup(MAKEWORD(2,2), &wsaData);

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

			// Ignore the first two set of 4 bytes because they are LabVIEW headers
			const int offset = 2;
			for (int i = offset; i < length/4; i++)
			{
				// Adjust from network byte order to host order
				int *dst = (int*)arr+i-offset;
				int *src = (int*)buff+i;
				*dst = ntohl(*src);
			}

			// Parse information from giant array into packet
			// NOTE: If the LabVIEW code changes, this must also change!
			int ctr = 0;
			p.samples = 1; // The way it's currently set up, there will only be one sample at a time
			for (int i = 0; i < 3; i++) { p.manipPose[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.vLimiter[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.tipGoal[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.tipErrorManip[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.inverseKin[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.controller[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.stackPos[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.stackFeedback[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.stackDrive[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.stackReadback[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.stimulus[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.statusFlags[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.analogData[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.traceData[i] = arr[ctr+i]; } ctr += 3;

			for (int j = 0; j < 4; j++) { for (int i = 0; i < 3; i++) { p.light[j][i] = arr[ctr+i]; } ctr += 3; }
			for (int j = 0; j < 3; j++) { for (int i = 0; i < 3; i++) { p.rotation[j][i] = arr[ctr+i]; } ctr += 3; }

			for (int i = 0; i < 3; i++) { p.zeroingOffset[i] = arr[ctr+i]; } ctr += 3;

#ifndef MRECV_OLD
			for (int j = 0; j < 8; j++) { for (int i = 0; i < 3; i++) { p.psdValues[j][i] = arr[ctr+i]; } ctr += 3; }
#endif

			for (int i = 0; i < 3; i++) { p.tipPos[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.centralTipPos[i] = arr[ctr+i]; } ctr += 3;
#ifndef MRECV_OLD
			for (int i = 0; i < 3; i++) { p.handlePos[i] = arr[ctr+i]; } ctr += 3;
			for (int j = 0; j < 3; j++) { for (int i = 0; i < 3; i++) { p.handleRotation[j][i] = arr[ctr+i]; } ctr += 3; }

			for (int j = 0; j < 5; j++) { for (int i = 0; i < 3; i++) { p.tipState[j][i] = arr[ctr+i]; } ctr += 3; }
			for (int j = 0; j < 5; j++) { for (int i = 0; i < 3; i++) { p.handleState[j][i] = arr[ctr+i]; } ctr += 3; }

			for (int i = 0; i < 3; i++) { p.tipPosKF[i] = arr[ctr+i]; } ctr += 3;
			for (int i = 0; i < 3; i++) { p.centralTipPosKF[i] = arr[ctr+i]; } ctr += 3;
#endif

			p.counter = arr[ctr];
			
			// Trace data encodes special bit flags about the status of Micron
			//flags = (int)p.traceData[0];
			flags = (int)p.statusFlags[0]; // status flags moved to p.statusFlags
			p.cancellation = !!(flags & 1<<LIN_BIT_CANCELLATION);
			p.scaling = !!(flags & 1<<LIN_BIT_SCALING);
			p.saturated = !!(flags & 1<<LIN_BIT_SATURATED);
			p.validPos = !!(flags & 1<<LIN_BIT_VALIDPOS);

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
///   \brief Clears data in a Micron receive packet
///
////////////////////////////////////////////////////////////////////////////////
void MicronRecvPacket::clear()
{
	for (int i = 0; i < 3; i++)
	{
		manipPose[i] = 0;
		vLimiter[i] = 0;
		tipGoal[i] = 0;
		tipErrorManip[i] = 0;
		inverseKin[i] = 0;
		controller[i] = 0;
		stackPos[i] = 0;
		stackFeedback[i] = 0;
		stackDrive[i] = 0;
		stackReadback[i] = 0;
		stimulus[i] = 0;
		statusFlags[i] = 0;
		analogData[i] = 0;
		traceData[i] = 0;
		zeroingOffset[i] = 0;
		tipPos[i] = 0;
		centralTipPos[i] = 0;
		handlePos[i] = 0;

		for (int j = 0; j < 4; j++)
			light[j][i] = 0;

		for (int j = 0; j < 3; j++)
			rotation[j][i] = 0;

		for (int j = 0; j < 8; j++)
			psdValues[j][i] = 0;

		for (int j = 0; j < 3; j++)
			handleRotation[j][i] = 0;

		for (int j = 0; j < 5; j++)
			tipState[j][i] = 0;

		for (int j = 0; j < 5; j++)
			handleState[j][i] = 0;

		tipPosKF[i] = 0;
		centralTipPosKF[i] = 0;
	}
	counter = 0;
	scaling = 0;
	samples = 0;
	cancellation = 0;
	validPos = 0;
	saturated = 0;
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
		for (int i = 0; i < 3; i++)
		{
			manipPose[i] = rhs.manipPose[i];
			vLimiter[i] = rhs.vLimiter[i];
			tipGoal[i] = rhs.tipGoal[i];
			tipErrorManip[i] = rhs.tipErrorManip[i];
			inverseKin[i] = rhs.inverseKin[i];
			controller[i] = rhs.controller[i];
			stackPos[i] = rhs.stackPos[i];
			stackFeedback[i] = rhs.stackFeedback[i];
			stackDrive[i] = rhs.stackDrive[i];
			stackReadback[i] = rhs.stackReadback[i];
			stimulus[i] = rhs.stimulus[i];
			statusFlags[i] = rhs.statusFlags[i];
			analogData[i] = rhs.analogData[i];
			traceData[i] = rhs.traceData[i];
			zeroingOffset[i] = rhs.zeroingOffset[i];
			tipPos[i] = rhs.tipPos[i];
			centralTipPos[i] = rhs.centralTipPos[i];
			handlePos[i] = rhs.handlePos[i];

			for (int j = 0; j < 4; j++)
				light[j][i] = rhs.light[j][i];

			for (int j = 0; j < 3; j++)
				rotation[j][i] = rhs.rotation[j][i];

			for (int j = 0; j < 8; j++)
				psdValues[j][i] = rhs.psdValues[j][i];

			for (int j = 0; j < 3; j++)
				handleRotation[j][i] = rhs.handleRotation[j][i];

			for (int j = 0; j < 5; j++)
				tipState[j][i] = rhs.tipState[j][i];

			for (int j = 0; j < 5; j++)
				handleState[j][i] = rhs.handleState[j][i];

			tipPosKF[i] = rhs.tipPosKF[i];
			centralTipPosKF[i] = rhs.centralTipPosKF[i];
		}
		counter = rhs.counter;
		scaling = rhs.scaling;
		samples = rhs.samples;
		cancellation = rhs.cancellation;
		validPos = rhs.validPos;
		saturated = rhs.saturated;
	}

	return *this;
}

void MicronRecvPacket::write(FILE *fp)
{
	if (!fp)
		return;

	for (int i = 0; i < 3; i++)
	{
		WRITE_DATA(fp, manipPose[i]);
		WRITE_DATA(fp, vLimiter[i]);
		WRITE_DATA(fp, tipGoal[i]);
		WRITE_DATA(fp, tipErrorManip[i]);
		WRITE_DATA(fp, inverseKin[i]);
		WRITE_DATA(fp, controller[i]);
		WRITE_DATA(fp, stackPos[i]);
		WRITE_DATA(fp, stackFeedback[i]);
		WRITE_DATA(fp, stackDrive[i]);
		WRITE_DATA(fp, stackReadback[i]);
		WRITE_DATA(fp, stimulus[i]);
		WRITE_DATA(fp, statusFlags[i]);
		WRITE_DATA(fp, analogData[i]);
		WRITE_DATA(fp, traceData[i]);
		WRITE_DATA(fp, zeroingOffset[i]);
		WRITE_DATA(fp, tipPos[i]);
		WRITE_DATA(fp, centralTipPos[i]);
		WRITE_DATA(fp, handlePos[i]);

		for (int j = 0; j < 4; j++)
			WRITE_DATA(fp, light[j][i]);

		for (int j = 0; j < 3; j++)
			WRITE_DATA(fp, rotation[j][i]);

		for (int j = 0; j < 8; j++)
			WRITE_DATA(fp, psdValues[j][i]);

		for (int j = 0; j < 3; j++)
			WRITE_DATA(fp, handleRotation[j][i]);

		for (int j = 0; j < 5; j++)
			WRITE_DATA(fp, tipState[j][i]);
		
		for (int j = 0; j < 5; j++)
			WRITE_DATA(fp, handleState[j][i]);

		WRITE_DATA(fp, tipPosKF[i]);
		WRITE_DATA(fp, centralTipPosKF[i]);
	}
	WRITE_DATA(fp, counter);
	WRITE_DATA(fp, scaling);
	WRITE_DATA(fp, samples);
	WRITE_DATA(fp, cancellation);
	WRITE_DATA(fp, validPos);
	WRITE_DATA(fp, saturated);
}

void MicronRecvPacket::read(FILE *fp)
{
	if (!fp)
		return;

	for (int i = 0; i < 3; i++)
	{
		READ_DATA(fp, manipPose[i]);
		READ_DATA(fp, vLimiter[i]);
		READ_DATA(fp, tipGoal[i]);
		READ_DATA(fp, tipErrorManip[i]);
		READ_DATA(fp, inverseKin[i]);
		READ_DATA(fp, controller[i]);
		READ_DATA(fp, stackPos[i]);
		READ_DATA(fp, stackFeedback[i]);
		READ_DATA(fp, stackDrive[i]);
		READ_DATA(fp, stackReadback[i]);
		READ_DATA(fp, stimulus[i]);
		READ_DATA(fp, statusFlags[i]);
		READ_DATA(fp, analogData[i]);
		READ_DATA(fp, traceData[i]);
		READ_DATA(fp, zeroingOffset[i]);
		READ_DATA(fp, tipPos[i]);
		READ_DATA(fp, centralTipPos[i]);
		READ_DATA(fp, handlePos[i]);

		for (int j = 0; j < 4; j++)
			READ_DATA(fp, light[j][i]);

		for (int j = 0; j < 3; j++)
			READ_DATA(fp, rotation[j][i]);

		for (int j = 0; j < 8; j++)
			READ_DATA(fp, psdValues[j][i]);

		for (int j = 0; j < 3; j++)
			READ_DATA(fp, handleRotation[j][i]);

		for (int j = 0; j < 5; j++)
			READ_DATA(fp, tipState[j][i]);

		for (int j = 0; j < 5; j++)
			READ_DATA(fp, handleState[j][i]);

		READ_DATA(fp, tipPosKF[i]);
		READ_DATA(fp, centralTipPosKF[i]);
	}
	READ_DATA(fp, counter);
	READ_DATA(fp, scaling);
	READ_DATA(fp, samples);
	READ_DATA(fp, cancellation);
	READ_DATA(fp, validPos);
	READ_DATA(fp, saturated);
}

#endif // USE_MICRON_6DOF