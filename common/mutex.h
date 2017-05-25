/*==================================================================================

    Filename:  mutex.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Cross-platform mutex structures.  
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
#ifndef _MUTEX_H
#define _MUTEX_H


//  Include pthread for LINUX
#ifdef WIN32
//#include <WinSock2.h>
#include <windows.h>
#else
#include <pthread.h>
#endif

#ifndef INFINITE
#define 0xFFFFFFFF
#endif

////////////////////////////////////////////////////////////////////////////////////
///
///   \class Mutex
///   \brief Cross platform class for creating mutexes.
///
///   The Mutex class is used when you need to have read write protection
///   on data being accessed from multiple threads and processes.  This class
///   is a wrapper to create a mutex on both a Windows and Linux system.
///
///   This class is part of the zebbase library.  zbase is the base library
///   for the Zebulon software libraries and code bank.  It contains code for
///   doing cross platform tasks such as threading, network communication, serial
///   comunication, and handling some HI devices.
///
///   As of this writing the rLock, wLock, and rwUnlock have only been 
///   tested under Windows, not Linux. However the enter and leave
///   functions have been tested and do work in both.  7/12/2005 - Daniel Barber
///
///   Required Libraries:
///   <ul>
///     <li><b>Windows</b>
///         <ol>
///             <li>None</li>
///         </ol>
///     </li>
///     <li><b>Linux</b>
///         <ol>
///             <li>pthread.lib</li>
///         </ol>
///     </li>
///   </ul>
///
////////////////////////////////////////////////////////////////////////////////////
class Mutex
{
public:
    Mutex();
    ~Mutex();
    void enter(int wait = INFINITE) const;
    void leave() const;
private:
#ifdef WIN32
    HANDLE mMutex;                   ///<  Mutex handle.    
#else
    pthread_mutex_t mMutex;          ///< Standard pthread mutex.
#endif    
    static unsigned int mCount;      ///<  The mutex number
};

////////////////////////////////////////////////////////////////////////////////////
///
///   \class ReadWriteMutex
///   \brief Cross platform class for creating mutexes.
///
///   The ReadWriteMutex structure uses critical sections in memory to specify a
///   how to enter a critical section.  For example, if you only need to read
///   from a critical area than your priority is different than if writing. This
///   could be used to increase the speed of a program, however as of this
///   writing no significant speed increase has been found.
///
///   This class is part of the zebbase library.  zbase is the base library
///   for the Zebulon software libraries and code bank.  It contains code for
///   doing cross platform tasks such as threading, network communication, serial
///   comunication, and handling some HI devices.
///
///   As of this writing the rLock, wLock, and rwUnlock have only been 
///   tested under Windows, not Linux. 7/12/2005 - Daniel Barber
///
///   Required Libraries:
///   <ul>
///     <li><b>Windows</b>
///         <ol>
///             <li>None</li>
///         </ol>
///     </li>
///     <li><b>Linux</b>
///         <ol>
///             <li>pthread.lib</li>
///         </ol>
///     </li>
///   </ul>
///
////////////////////////////////////////////////////////////////////////////////////
class ReadWriteMutex
{
public:
    ReadWriteMutex();
    ~ReadWriteMutex();
    void rLock();       ///<  Lock mutex for reading only.
    void wLock();       ///<  Lock mutex for reading/writing.
    void unlock();      ///<  Unlock mutex from reading or writing.
private:
#ifdef WIN32
    //  This is for read/write locks
    HANDLE              mReading;           ///< Event object.
    HANDLE              mFinished;          ///< Event object.
    int                 mReadCount;         ///< Number of readers.
    int                 mWriteCount;        ///< Number of writers.
    CRITICAL_SECTION    mCriticalSection;   ///< Critical section.
#else
    //  For read/write locks
    pthread_mutex_t mMutex;                 ///< Mutex structure.
    pthread_cond_t mReadCond;               ///< Read condition of mutex.
    pthread_cond_t mWriteCond;              ///< Write condition of mutex.
    int mLocks;                             ///< Locks in place.
    int mWriteCount;                        ///< Number of writers.
    int mReadCount;                         ///< Number of readers.
#endif    
};


#endif
