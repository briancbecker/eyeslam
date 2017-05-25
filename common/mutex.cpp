/*==================================================================================

    Filename:  mutex.cpp

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
#include "mutex.h"
#include "macros.h"
#include <time.h>
#include <assert.h>
#include <stdio.h>
#include <cstdlib>

unsigned int Mutex::mCount = 0;

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Constructor.  This initializes the mutex file descripters for use.
///
////////////////////////////////////////////////////////////////////////////////////
Mutex::Mutex()
{
#if defined(WIN32)
    char name[100];
    mMutex = 0;

	mCount++;
    sprintf(name, "MUTEX%d%d", mCount, GET_TIME_MS());
    mMutex = CreateMutexA(NULL, TRUE, name);
    assert(mMutex);
    ReleaseMutex(mMutex);
#else
    pthread_mutex_init(&mMutex, NULL);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Destructor.  This closes any mutex that may be open.
///
////////////////////////////////////////////////////////////////////////////////////
Mutex::~Mutex() 
{
#ifdef WIN32
    if (mMutex)
    {
        ReleaseMutex(mMutex);
        CloseHandle(mMutex);
        mMutex = 0;
    }
#else
    //  Close the mutex
    pthread_mutex_destroy(&mMutex);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Call this function to enter the critical area that the mutex is in.
///
///  \param wait Wait for the mutex to be released 
///             (default time is wait infinitely) and is in milliseconds.  This
///              variable only has an impact in Windows.
///
////////////////////////////////////////////////////////////////////////////////////
void Mutex::enter(int wait /*= INFINITE*/) const
{
#ifdef WIN32
    if (mMutex)
    {
      WaitForSingleObject(mMutex, wait);  
    }   
#else
    pthread_mutex_t *mut = (pthread_mutex_t *)(&mMutex);
    pthread_mutex_lock(mut);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Call this function to leave the critical area that the mutex is in.
///
////////////////////////////////////////////////////////////////////////////////////
void Mutex::leave() const
{
#ifdef WIN32
    if(mMutex)
        ReleaseMutex(mMutex);
#else
    pthread_mutex_t *mut = (pthread_mutex_t *)(&mMutex);
    pthread_mutex_unlock(mut);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Constructor.  This initializes the mutex file descripters for use.
///
////////////////////////////////////////////////////////////////////////////////////
ReadWriteMutex::ReadWriteMutex()
{
#if defined(WIN32)  
    //For RWLock functionality
    mReadCount = -1;
    mWriteCount = 0;
    mReading = CreateEvent(0, true, false, 0);
    mFinished = CreateEvent(0, false, true, 0);
    if (mReading == 0 || mFinished == 0)
        exit(1);
    InitializeCriticalSection(&mCriticalSection);
#else
    //For rwLock functionality
    mLocks = mWriteCount = mReadCount = 0;
    pthread_mutex_init(&mMutex, 0);
    pthread_cond_init(&mReadCond, 0);
    pthread_cond_init(&mWriteCond, 0);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Destructor.  This closes any mutex that may be open.
///
////////////////////////////////////////////////////////////////////////////////////
ReadWriteMutex::~ReadWriteMutex() 
{
#ifdef WIN32
    //  For rwLock
    CloseHandle(mReading);
    CloseHandle(mFinished);
#else
    //  For read write lock
    pthread_cond_destroy(&mWriteCond);
    pthread_cond_destroy(&mReadCond);
    pthread_mutex_destroy(&mMutex);
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Read lock is a special lock where it the critical area when you want
///  to do reading.  Using this makes mutex protection faster at runtime, because
///  it signals to the system you are only reading, not writing.
///
////////////////////////////////////////////////////////////////////////////////////
void ReadWriteMutex::rLock()
{
#ifdef WIN32
    if (++mReadCount == 0)
    {
        WaitForSingleObject(mFinished, INFINITE);
        SetEvent(mReading);    
    }
    WaitForSingleObject(mReading, INFINITE);
#else
    pthread_mutex_lock(&mMutex);
    mReadCount++;
    while (mLocks < 0)
        pthread_cond_wait(&mReadCond, &mMutex);
    mReadCount--;
    mLocks++;
    pthread_mutex_unlock(&mMutex);
#endif    
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief This function is used when you are only entering the critical area
///  to write to some memory.  Using this makes mutex protection faster at runtime
///  because it tells the system specifically that you are writing.
///
////////////////////////////////////////////////////////////////////////////////////
void ReadWriteMutex::wLock()
{
#ifdef WIN32
    EnterCriticalSection(&mCriticalSection);
    WaitForSingleObject(mFinished, INFINITE);
    mWriteCount++;
#else
    pthread_mutex_lock(&mMutex);
    mWriteCount++;
    while (mLocks != 0)
        pthread_cond_wait(&mWriteCond, &mMutex);
    mLocks = -1;
    mWriteCount--;
    pthread_mutex_unlock(&mMutex);
#endif    
}


////////////////////////////////////////////////////////////////////////////////////
///
///  \brief If you are using the rLock and wLock functions then use this function
///  to unlock/leave the mutex.
///
////////////////////////////////////////////////////////////////////////////////////
void ReadWriteMutex::unlock()
{
#ifdef WIN32
    if (mWriteCount != 0) 
    {
        mWriteCount--;
        SetEvent(mFinished);
        LeaveCriticalSection(&mCriticalSection);
    } 
    else if (--mReadCount < 0) 
    {
        ResetEvent(mReading);
        SetEvent(mFinished);
    } 
#else
    pthread_mutex_lock(&mMutex);
    if (mLocks > 0)
    {
        mLocks--;
        if (mLocks == 0)
            pthread_cond_signal(&mWriteCond);
    }
    else
    {
        mLocks = 0;
        if (mReadCount != 0)
            pthread_cond_broadcast(&mReadCond);
        else
            pthread_cond_signal(&mWriteCond);
    }
    pthread_mutex_unlock(&mMutex);
#endif    
}

/*  End of File */
