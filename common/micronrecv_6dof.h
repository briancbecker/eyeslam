////////////////////////////////////////////////////////////////////////////////
///
///   Filename: micronrecv.h
///
///   Copyright (C) 2008-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Surgical Mechatronics Lab       Micron Project
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

#ifndef MICRONRECV_6DOF_H
#define MICRONRECV_6DOF_H

#ifndef WIN32
//#include <WinSock2.h>
#include <windows.h>
#endif


#include "thread.h"
#include "array.h"
#include "mutex.h"
#include "queue.h"
#include "fpstracker.h"



// For the purposes of backwards compatibility and using MicronRecvPacket::tipPos variables, you can set this to true
#define MICRON_BACKWARDS_COMPATABILITY  1
#define MICRON_RECV_PACKET_PADDING      (!!MICRON_BACKWARDS_COMPATABILITY*3*sizeof(float)*3)


#define USE_MICRON_6DOF 1
// This define is kept around for compatibility reasons
//#define MRECV_OLD

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_PORT
///   \brief Default port to listen to (the port Micron will be sending data from)
////////////////////////////////////////////////////////////////////////////////
#define LIN_PORT                        61557

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BUFF_LEN
///   \brief Default buffer length to receive packet 
////////////////////////////////////////////////////////////////////////////////
#define LIN_BUFF_LEN                    (1024*10)

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_SLEEP_TIME
///   \brief Milliseconds 
////////////////////////////////////////////////////////////////////////////////
#define LIN_SLEEP_TIME                  0

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_WAIT_KILL_TIME
///   \brief Milliseconds to wait for thread to stop, then just terminate it
////////////////////////////////////////////////////////////////////////////////
#define LIN_WAIT_KILL_TIME              (100)

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BLOCKING
///   \brief Should the socket act as a blocking socket by default? This parameter
///          doesn't matter too much and can be changed later
////////////////////////////////////////////////////////////////////////////////
#define LIN_BLOCKING                    1

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BIT_CANCELLATION
///   \brief Bit position in the status flag that indicates whether cancellation
///          mode is on
////////////////////////////////////////////////////////////////////////////////
#define LIN_BIT_CANCELLATION            0

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BIT_SCALING
///   \brief Bit position in the status flag that indicates whether scaling mode
///          is on
////////////////////////////////////////////////////////////////////////////////
#define LIN_BIT_SCALING                 1

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BIT_SATURATED
///   \brief Bit position in the status flag that indicates whether or not the
///          manipulator actuator are saturated (have reached their limits)
////////////////////////////////////////////////////////////////////////////////
#define LIN_BIT_SATURATED               3

////////////////////////////////////////////////////////////////////////////////
///   \def LIN_BIT_VALIDPOS
///   \brief Bit position in the status flag that indicates whether or not
///          Micron is in the ASAP workspace (and thus whether or not we are
///          getting correct positioning measurements from ASAP)
////////////////////////////////////////////////////////////////////////////////
#define LIN_BIT_VALIDPOS                4

////////////////////////////////////////////////////////////////////////////////
///
///   \class MicronRecvPacket
///   \brief Packet structure to hold various data received from Micron
///
///   This public class houses data received from the Micron realtime target at
///   each sensor measurement cycle. Check the wiki for details on what each
///   variable is.
/// 
////////////////////////////////////////////////////////////////////////////////
#pragma pack(push,4)

// probe_points.h defines the MicronRecvPacket class and all of the information we are getting from LabVIEW. 
#include "probe_points.h"
	MicronRecvPacket();
	void clear();

	float samples;                         ///< Number of measurements since last update
	int cancellation;                      ///< Is cancellation mode on?
	int valid_pos;                         ///< Is Micron inside the workspace?
	int validPos;                          ///< Same as valid_pos
	int saturated;                         ///< Is Micron saturated (motors at their limits)
	int scaling;                           ///< Is Micron scaling mode on?
	char flags[32];                         ///< Flags
	
	MicronRecvPacket &operator=(const MicronRecvPacket &rhs);

	void write(FILE *fp);
	void read(FILE *fp);

	// These are the commonly used data points you would use from the earlier versions of this class, so link them up for backwards compatibility
#if MICRON_BACKWARDS_COMPATABILITY 
	float *tipPos;                         ///< Pointer to position_tip
	float *centralTipPos;                  ///< Pointer to null_pos_tip
	float *analogData;                     ///< Pointer to analog_data
#endif
};
#pragma pack(pop)


////////////////////////////////////////////////////////////////////////////////
///
///   \class MicronRecvSocket
///   \brief Incoming communication with Micron realtime target
///
///   This is a threaded class that connects to the Micron real-time target and
///   receives status packets at 2 kHz. The packets are queued in the background
///   and made available through the interface in a mutex-protected manner. You
///   can specify the port number to listen to and whether or not to use a
///   blocking socket. The update frequency is recorded as packets are received
///   and can be accessed.
/// 
////////////////////////////////////////////////////////////////////////////////
class MicronRecvSocket : protected Thread
{
public:
	MicronRecvSocket();
	~MicronRecvSocket();

	void start(int port = LIN_PORT, bool blocking = true);
	int recv(MicronRecvPacket &packet);
	/// Get the number of packets received
	int getNumPackets() { return mRecvNumber; }
	/// Get the frequency at which packets are arriving
	double getFrequency() { return mFrequency; }

private:
	virtual void execute();
	virtual void cleanup();

	void init();
	void zero();
	void destroy();

	int mPort;                          ///< Port to listen for Micron data
	bool mBlocking;                     ///< Use a blocking socket?
	Mutex mMutex;                       ///< Mutex for thread protection
	int mRecvNumber;                    ///< Number of received packets

	UINT_PTR mSock;						///< Windows socket handle....05/06/2014..it has been changed because to compile _encode_vid..
	//SOCKET mSock;                     ///< Windows socket handle....05/06/2014..it has been changed because to compile _encode_vid..
										

	FPSTracker mFPS;                    ///< For tracking the update frequency
	double mFrequency;                  ///< What is the update frequency?

	Queue<MicronRecvPacket> mPackets;   ///< Queue of incoming packets
};

#endif // MICRONRECV_6DOF_H