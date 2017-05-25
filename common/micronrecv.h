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

#ifndef MICRONRECV_H
#define MICRONRECV_H

#define USE_MICRON_6DOF 1

#if USE_MICRON_6DOF
#include "micronrecv_6dof.h"
#else

#ifndef WIN32
////#include <WinSock2.h>
#include <windows.h>
#endif

#include "thread.h"
#include "array.h"
#include "mutex.h"
#include "queue.h"
#include "fpstracker.h"

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
class MicronRecvPacket
{
public:
	MicronRecvPacket() { clear(); }
	void clear();

	/// The "core" position of Micron. It is attached to the output, with the origin at top center of the tool holder.  Note that the rotation part of the manipulator pose is elsewhere in the trace (rotation 0 1 2), and that you need to pad with a 0 0 0 1 row at the bottom to reconstruct the 4x4 pose matrix. 
	float manipPose[3];
	/// Something to do with the velocity limiting (?)
	float vLimiter[3];
	/// 3D position where Micron is trying to get to
	float tipGoal[3];
	/// Tip position error in manipulator coordinates
	float tipErrorManip[3];
	/// Tip error after conversion to stack coordinates (i.e. tip error expressed as stack motion.)
	float inverseKin[3];
	/// Output from the controller (stack coordinates)
	float controller[3];
	/// Position of the each of the 3 stacks (-400, 400)
	float stackPos[3];
	/// Voltage of the stacks
	float stackFeedback[3];
	/// Current to the stack
	float stackDrive[3];
	/// Sensor on the current stack position/charge (?)
	float stackReadback[3];
	/// Signal injected by the front panel generator (step, sine, etc). Should be zero when off (it is normally off)
	float stimulus[3];
	/// Status flags
	float statusFlags[3];
	/// Data coming in through analog connection (camera strobe, etc)
	float analogData[3];
	/// Extra info
	float traceData[3];
	/// 3D positions of the 4 light sensors
	float light[4][3];
	/// 3x3 rotation matrix representing the pose of Micron
	float rotation[3][3];
	/// Offset used to the central tip position (CTP) to make sure CTP is at stack position (0, 0, 0)
	float zeroingOffset[3];
//#ifndef MRECV_OLD
	/// Raw PSD values
	float psdValues[8][3];
//#endif
	/// 3D position of the tip of Micron
	float tipPos[3];
	/// 3D position of where the tip of Micron could be if un-actuated
	float centralTipPos[3];
//#ifndef MRECV_OLD
	/// 3D Handle position
	float handlePos[3];
	/// 3D Handle rotation
	float handleRotation[3][3];
	/// State of the tip from Kalman Filter
	float tipState[5][3];
	/// State of the handle from the Kalman Filter
	float handleState[5][3];
	/// Tip position (Kalman Filtered)
	float tipPosKF[3];
	/// Central tip position (Kalman Filtered)
	float centralTipPosKF[3];
//#endif
	
	float counter;                         ///< Number of packets sent from Micron

	float samples;                         ///< Number of measurements since last update
	int cancellation;                      ///< Is cancellation mode on?
	int validPos;                          ///< Is Micron inside the workspace?
	int saturated;                         ///< Is Micron saturated (motors at their limits)
	int scaling;                           ///< Is Micron scaling mode on?
	
	MicronRecvPacket &operator=(const MicronRecvPacket &rhs);

	void write(FILE *fp);
	void read(FILE *fp);
};


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
	SOCKET mSock;                       ///< Windows socket handle
	FPSTracker mFPS;                    ///< For tracking the update frequency
	double mFrequency;                  ///< What is the update frequency?

	Queue<MicronRecvPacket> mPackets;   ///< Queue of incoming packets
};

#endif // USE_MICRON_6DOF

#endif // MICRONRECV_H