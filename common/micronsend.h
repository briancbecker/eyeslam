////////////////////////////////////////////////////////////////////////////////
///
///   Filename: micronsend.h
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

#ifndef MICRONSEND_H
#define MICRONSEND_H

#ifndef WIN32
//#include <WinSock2.h>
#include <windows.h>
#endif



#include "thread.h"
#include "array.h"
#include "mutex.h"
#include "queue.h"
#include "string.h"
#include "fpstracker.h"

////////////////////////////////////////////////////////////////////////////////
///   \def LOUT_PORT
///   \brief Port which Micron will be listening on to get sent commands
////////////////////////////////////////////////////////////////////////////////
#define LOUT_PORT                       61558

////////////////////////////////////////////////////////////////////////////////
///   \def LOUT_SLEEP_TIME
///   \brief Millisecond delay between checking for packets to send
////////////////////////////////////////////////////////////////////////////////
#define LOUT_SLEEP_TIME                 1

////////////////////////////////////////////////////////////////////////////////
///   \def LOUT_BLOCKING
///   \brief Should we use a blocking socket to send commands? (default)
////////////////////////////////////////////////////////////////////////////////
#define LOUT_BLOCKING                   1

////////////////////////////////////////////////////////////////////////////////
///   \def LOUT_WAIT_KILL_TIME
///   \brief Milliseconds to wait for thread to stop, then just terminate it
////////////////////////////////////////////////////////////////////////////////
#define LOUT_WAIT_KILL_TIME             (100)

////////////////////////////////////////////////////////////////////////////////
///   \def LOUT_BUFF_LEN
///   \brief Buffer length to create packets
////////////////////////////////////////////////////////////////////////////////
#define LOUT_BUFF_LEN                   1024


////////////////////////////////////////////////////////////////////////////////
///
///   \class MicronSendPacket
///   \brief Public packet structure to hold commands sent to Micron
///
///   We can send commands to Micron, particularly the 3D goal position, by
///   which we can control the tip position of Micron. To enable control,
///   useGoalPos must be set to a non-zero value and the "Recv Commands" front
///   panel button of Micron's LabVIEW VI must be enabled.
///
///   For future expansion, there are 12 extra floats that are sent over the
///   network (can be configured to control a laser or other Micron processes).
/// 
////////////////////////////////////////////////////////////////////////////////
class MicronSendPacket
{
public:
	MicronSendPacket() { clear(); }
	void clear();

	//float goalPos[3];                      ///< 3D position Micron should go to
	//float useGoalPos;                      ///< If 0, turns normal operation on
	//float extraInfo[12];                   ///< Reserved for future use

	//0-2: goal_tip_pos
	//3: useGoal Pos: either 1 or 3 (using RCM goal)
	//4: extraInfo[0], fire laser..
//////////////////////////////////////////////////////////////////////////////////////////////////
	float goalPos[3];                      ///< 3D position Micron should go to
	float useGoalPos;                      ///< If 0, turns normal operation on

	float goalPosRCM[3];                   ///< 3D RCM position Micron should go to
	float fireLaser;

	float extraInfo[8];                   ///< Reserved for future use
	//0-2: goal_pos_tip
	//3: useGoal Pos: either 1 or 3 (for using RCM goal)
	//4-6: goal_pos_rcm
	//7: fire laser

	MicronSendPacket &operator=(const MicronSendPacket &rhs);
};


////////////////////////////////////////////////////////////////////////////////
///
///   \class MicronSendSocket
///   \brief Outgoing communication with Micron realtime target
///
///   This threaded class runs in the background and sends controls to the
///   Micron realtime target. Control primarily consists of commanding 3D
///   positions for Micron to reach, although other information can be passed
///   along to the micron realtime target. Currently, control is run at 1 kHz,
///   so sending control updates at a higher rate will be ignored.
/// 
////////////////////////////////////////////////////////////////////////////////
class MicronSendSocket : protected Thread
{
public:
	MicronSendSocket();
	~MicronSendSocket();

	void start(const char *address, int port = LOUT_PORT, bool blocking = true);
	int send(MicronSendPacket &packet);
	/// Returns the number of packets sent to the Micron realtime target
	int getNumPackets() { return mSendNumber; }
	/// Returns the frequency at which packets are sent to Micron (in Hz)
	double getFrequency() { return mFrequency; }

private:
	virtual void execute();
	virtual void cleanup();

	void init();
	void zero();
	void destroy();

	int mPort;                          ///< Port to send data over
	bool mBlocking;                     ///< Use a blocking socket?
	Mutex mMutex;                       ///< Mutex for thread protection
	char mAddress[256];                 ///< IP address of Micron machine
	int mSendNumber;                    ///< Number of packets sent
	SOCKET mSock;                       ///< Windows socket handle
	FPSTracker mFPS;                    ///< For tracking frequency
	double mFrequency;                  ///< Actual frequency of sending packets

	Queue<MicronSendPacket> mPackets;   ///< Queue of packets to send
};

#endif // MICRONSEND_H