/*==================================================================================

    Filename:  thread.cpp

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
#include <stdio.h>
#include "thread.h"
#include <string.h>

#ifdef WIN32

#include <time.h>


unsigned int Thread::mThreadNum = 0;

//
// Usage: SetThreadName (-1, "MainThread");
//
typedef struct tagTHREADNAME_INFO
{
    DWORD dwType;     ///< must be 0x1000
    LPCSTR szName;    ///< pointer to name (in user addr space)
    DWORD dwThreadID; ///< thread ID (-1=caller thread)
    DWORD dwFlags;    ///< reserved for future use, must be zero
} THREADNAME_INFO;

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the thread name for Visual Studio C++ 6 and above
///
///   \param dwThreadID ID of the thrad
///   \param szThreadName Name of the thread
///
////////////////////////////////////////////////////////////////////////////////
void SetThreadName(DWORD dwThreadID, LPCSTR szThreadName)
{
#ifdef WIN32

    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = szThreadName;
    info.dwThreadID = dwThreadID;
    info.dwFlags = 0;

    __try
    {
#ifdef _WIN64
		RaiseException(0x406D1388, 0, sizeof(info)/sizeof(DWORD), (ULONG_PTR*)&info);
#else
		RaiseException(0x406D1388, 0, sizeof(info)/sizeof(DWORD), (DWORD*)&info);
#endif
    }
    __except(EXCEPTION_CONTINUE_EXECUTION)
    {
    }

#endif
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief The actual function that gets called by the new thread within
///  the wrapper class.
///
///  This function uses a void pointer to a class that is based off of the
///  Thread class and calls the virtual functions execute and cleanup
///  that are in the Thread class.  It also sets the active status flag.
///
////////////////////////////////////////////////////////////////////////////////////
DWORD WINAPI _threadproc(LPVOID arg)
{
    Thread *pThread = (Thread*)arg;
    pThread->mActive = true;

    if (pThread->mName)
    {
        SetThreadName(GetCurrentThreadId(), pThread->mName);
    }

    if(pThread->mLoopThread)
    {
        while(pThread->mLoopThread && !pThread->mQuitThreadFlag)
        {
            pThread->execute();
            Sleep(1);
        }
        pThread->cleanup();
    }
    else
    {
        pThread->execute();
    }
    pThread->cleanup();
    pThread->mActive = false;
    return 0;
}
#else
void *_threadproc(void *arg)
{
    //  Typecast
    Thread *thread = (Thread*)arg;
    thread->mActive = true;
    if(thread->mLoopThread)
    {
        while(thread->mLoopThread)
        {
            thread->execute();
            usleep(1000);
        }
    }
    else
    {
        thread->execute();
    }
    thread->cleanup();
    pthread_join(thread->mThread, NULL);
    thread->mActive = false;
    return 0;
}
#endif



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Constructor, sets defaults.
///
////////////////////////////////////////////////////////////////////////////////////
Thread::Thread() : mThreadFunc(0)
{
    init();
#ifdef WIN32
    //  Generate a unique name for the thread for
    //  tracking in Visual Studio
    char name[256];
    sprintf(name, "ZebulonThread_%d_%d", (unsigned int)(time(NULL)), ++mThreadNum);
    setName(name);
#endif

}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates the thread object and then sets the name
///
///   \param name Name of the thread to display in the VC++ debugger
///
////////////////////////////////////////////////////////////////////////////////
Thread::Thread(const char *name)
{
    init();
    setName(name);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the name of the thread in the VC++ debugger window
///
///   \param name Name of the thread (9 char limitaiton in VC++ 6)
///
////////////////////////////////////////////////////////////////////////////////
void Thread::setName(const char *name)
{
    if (name)
    {
        if(mName)
            delete[] mName;
        mName = new char[(unsigned int)(strlen(name) + 1)];
        strcpy(mName, name);

#ifdef WIN32
        if (isActive())
        {
            SetThreadName(mThreadId, mName);
        }
#endif
    }
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the defaults on all the thread members 
///
////////////////////////////////////////////////////////////////////////////////
void Thread::init()
{
    mActive = false;
    mLoopThread = false;
    mQuitThreadFlag = true;
    this->mThread = 0;
    this->mThreadArgs = NULL;
#ifdef WIN32
    this->mThreadId = 0;
    this->mName = 0;
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Destructor, terminates any running threads by force.
///
////////////////////////////////////////////////////////////////////////////////////
Thread::~Thread()
{
    this->stopThread();
    kill();
#ifdef WIN32
    if(mThread)
    {
        CloseHandle(mThread);
        mThread = NULL;
    }
    if (mName)
    {
        delete[] mName;
        mName = 0;
    }
#endif
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Creates the thread.
///
///  Uses either windows or pthread function calls to spawn a thread that uses
///  the _threadproc function.
///
///  Overload the execute and cleanup functions which are actually called in
///  the newly created thread.
///
///  \param loop If true, the thread functioned will be continuously called.  If 
///              false, than the function is called once, and the thread exits.
///
///  Returns true if successful, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::createThread(const bool loop)
{
    //  Close any previous open threads
    kill();
    mLoopThread = loop;
#ifdef WIN32
    mThread = CreateThread(NULL,
                          0,
                          (LPTHREAD_START_ROUTINE)_threadproc,
                          (LPVOID) this,
                          0,
                          (LPDWORD)&mThreadId);

    if(mThread == NULL)
        return false;
       
    mActive = true;
    mQuitThreadFlag = false;
    //Sleep(1);

    return true;
#else
    int rc;
	mActive = true;
	mQuitThreadFlag = false;
    rc = pthread_create(&mThread, NULL, _threadproc, (void *)this);
    
    if(rc)
    {
        printf("ERROR:Failed to create thread, error code %d\n", rc);
        printf("Thread::start()\n");
		mActive = false;
		mQuitThreadFlag = true;
        return false;
    }
    else
    {
        //usleep(1000);
        return true;
    }
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Creates a thread based on the function and it's arguments.
///
///  Uses either windows or pthread function calls to spawn a thread that uses
///  the function passed.
///
///  \param func Function pointer for thread operation.
///  \param args Arguments for the thread.
///
///  Returns true if successful, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::createThread(void (*func)(void *), void *args)
{
    if(!func)
        return false;

    this->mThreadFunc = func;
    this->mThreadArgs = args;
    return this->createThread();
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief If the thread is in a looping state, this function will stop it.  If
///  the thread takes longer than ms milliseconds to exit, then the thread is
///  killed.
///
///  \param ms How long to wait for looping thread to stop.
///
///  \return True if thread stopped.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::stopThread(const int ms)
{
    if(!isActive())
        return false;

    mLoopThread = false;
    mQuitThreadFlag = true;
    for(int i = 0; i < ms; i++)
    {
        if(!isActive())
            return true;
#ifdef WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
    }

    if(isActive())
        return kill();

    return true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Returns true if a thread is active, false otherwise.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::isActive() const
{
    return mActive;
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Waits for the thread to exit.
///
///  \param ms How many milliseconds to wait for thread to exit.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::wait(const int ms)
{
    if(ms <= 0)
        return false;

#ifdef WIN32
    return (WAIT_OBJECT_0 == WaitForSingleObject(mThread, ms));
#else
    if(pthread_join(mThread, (void **)ms))
    {
        return false;
    }
    else
    {
        return true;
    }
#endif
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Kills the thread, returns true on success.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::kill()
{
    mQuitThreadFlag = true;
#ifdef WIN32
    int  i;
    bool x;
    if(mActive)
    {
        i = TerminateThread(mThread, 1);
        if(i == 0)
            x = false;
        else
        {
            CloseHandle(mThread);
            mThread = NULL;
            mActive = false;
            x = true;
        }
    }
    else
    {
        if(mThread != NULL)
        {
            CloseHandle(mThread);
            mThread = NULL;
        }
        x = true;
    }

    
    return x;
#else
    if(mActive)
    {
        if(pthread_kill(mThread, 1))
        {
            return false;
        }
        else
        {
            mThread = 0;
            mActive = false;
            return true;
        }
    }
    else
    {
        mThread = 0;
        return true;
    }
#endif
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Returns the identifier of the thread.
///
////////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
DWORD Thread::getId() const
{
    return mThreadId;
}
#else
pthread_t Thread::getId() const
{
    return mThread;
}
#endif

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Sets the priority of the thread
///
///  Must be called after the thread has been created.
///  In Windows 0 is default, up to 15 is high, don't go above 8 or you will mess
///  with the OS.  -15 is the lowest.
///
///  \param priority Amount of priority, see Windows API reference for SetThreadPriority
///  \return True if successfull, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::setPriority(const int priority)
{
#if WIN32
    if (isActive())
    {
        if(SetThreadPriority(mThread, priority))
            return true;
        else
            return false;
    }
#else
    if(isActive())
    {
        int ret;
        struct sched_param param;
        param.sched_priority = priority;
        // Scheduling parameters of target thread 
        ret = pthread_setschedparam(mThread, SCHED_OTHER, &param);
        if(ret == 0)
            return true;
    }
#endif
    return false;
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \return True if the the thread has been told to quit, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::loopFlag() const { return mLoopThread; } 

////////////////////////////////////////////////////////////////////////////////////
///
///  \return True if the the thread has been told to quit, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool Thread::quitFlag() const { return mQuitThreadFlag; } 


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief The main execution function called within the thread.
///
///  This virtual function is what gets called within the thread created by
///  the createThread function. This is an abstract function, meaning if you
///  inherit Thread, you must implement this function.
///
////////////////////////////////////////////////////////////////////////////////////
void Thread::execute()
{
    if(this->mThreadFunc)
        mThreadFunc(mThreadArgs);
}



////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Function for doing any thread cleanup. This is an abstract function, 
///  meaning if you inherit Thread, you must implement this function.
///
////////////////////////////////////////////////////////////////////////////////////
void Thread::cleanup()
{
    // BCB: Umm...do we really need a printf statement saying this? I don't think so...
    //printf("\nCleaning up thread\n");
}
