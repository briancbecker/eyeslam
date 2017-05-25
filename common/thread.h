/*==================================================================================

    Filename:  thread.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Cross-platform interface for creation of threads.
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
#ifndef _THREAD_H
#define _THREAD_H

#ifdef WIN32
////#include <WinSock2.h>
#  include <windows.h>
#else
#  include <stdlib.h>
#  include <unistd.h>
#  include <signal.h>
#  include <pthread.h>
#endif


////////////////////////////////////////////////////////////////////////////////////
///
///  \def INFINITE
///  \brief Defines the value for INFINITE if it has not already been defined.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef INFINITE
#  define INFINITE 0xFFFFFFFF
#endif

////////////////////////////////////////////////////////////////////////////////////
///
///   \class Thread
///   \brief Cross-platform class for creating threads from functions or adding
///   threads to a class through inheritance.
///
///   Thread is used to create a threaded function within a class on both
///   Windows and Linux based systems.  This is extremely useful and simple to use
///   and makes it easy to have threading on multiple platforms.
///
///   To make use of the Thread class just use inheritance and define the virtual
///   functions.  The execute and cleanup functions are where you place code to
///   be performed in the spawned thread.  Functions for checking the status of
///   thread and killing it exist.
///
///   The thread created will perform the execute and cleanup functions and then
///   exit.  If it is desired to have the execute function loop, then that can
///   be done execute virtual function.  It is up to the programmer to handle 
///   creating flags to stop the loops within the execute function in the
///   child class.
///
///   This class is part of the zebbase library.  zbase is the base library
///   for the Zebulon software libraries and code bank.  It contains code for
///   doing cross platform tasks such as threading, network communication, serial
///   communication, and handling some HI devices.
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
class Thread
{
public:
    Thread();                                   ///<  Constructor.
    Thread(const char *name);                   ///<  Constructor with optional name for VC++ debugger.
    virtual ~Thread();                          ///<  Destructor.
    bool createThread(const bool loop = false); ///<  Creates the thread .
    bool createThread(void (*func)(void *), 
                      void *args);              ///<  Creates a thread based on a function pointer.
    bool stopThread(const int ms = 500);        ///<  Stop a looping thread.
    bool isActive() const;                      ///<  Is the thread still active?
    bool setPriority(const int priority);       ///<  Call after creating the thread
    bool wait(const int ms = INFINITE);         ///<  How long to wait for thread to exit..
    bool kill();                                ///<  Kills the thread forcefully.
    bool loopFlag() const;                      ///<  If true, thread should keep looping.
    bool quitFlag() const;                      ///<  If true thread should exit.
    void setName(const char *name);             ///<  Sets the name of the thread in the VC++ debugger.
    const char *getName(const char *name);      ///<  Get the name of the thread.
#ifdef WIN32
    DWORD getId() const;                        ///<  Gets the thread id.
#else
    pthread_t getId() const;                    ///<  Get the thread id.
#endif

protected:    
    bool mLoopThread;                           ///<  Keep looping thread.
    bool mActive;                               ///<  Is the thread active?
    bool mQuitThreadFlag;
    virtual void execute();                     ///<  The actual thread function.
    virtual void cleanup();                     ///<  Function for any cleanup.
private:
    void init();                                ///<  Common constructor.
    void (*mThreadFunc)(void *);                ///<  Function pointer for thread objects.
    void *mThreadArgs;                          ///<  Arguments for function pointer.
    char *mName;                                ///< Name of the thread.
#ifdef WIN32    
    static unsigned int mThreadNum;             ///< Thread number.
    HANDLE mThread;                             ///< File descriptor information for thread.
    DWORD  mThreadId;                           ///< The id number of the thread.
    friend DWORD WINAPI _threadproc(LPVOID arg); 
#else
    pthread_t mThread;                          ///< File descriptor for thread .
    friend void * _threadproc(void *arg);
#endif

};

#endif
/*  End of File */
