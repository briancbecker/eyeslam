/*==================================================================================

    Filename:  queue.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Template for creating queues without using a linked list structure.  By not 
    using a linked list, memory access time is reduced.  For most applications
    the speedup is meaningless, but in robots, any speed increase adds up in
    the long run.  This queue template has built in automatic resizing if
    desired, and is on by default. 
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
#ifndef _QUEUE_H
#define _QUEUE_H

#include <assert.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \class Queue
///   \brief Template for creating queues without using a Linked List.  By not 
///    using a linked list, memory access time is reduced.  For most applications
///    the speedup is meaningless, but in robots, any speed increase adds up in
///    the long run.  This queue template has built in automatic resizing if
///    desired, and is on by default.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
class Queue
{
public:
    Queue(const int resv = 10, const bool autoResize = true);
    Queue(const Queue<T> &rhs);
    ~Queue();
    int  enqueue(const T &rhs); ///<  Enqueue the data
    int  dequeue(T &rhs);       ///<  Dequeue data and copy it to rhs
    T &dequeue();               ///<  Dequeue data without copying it
    void clear();
    void destroy();    
    bool get(const int i, T &rhs) const;
    bool get(const int i, T **rhs) const;
    bool replace(const int i, const T &rhs);
    int size() const;
    int create(const int size, const bool autoResize = true);
    int resize(const int size);
    Queue<T> &operator=(const Queue<T> &rhs);
protected:
    T *mQueue;              ///<  Actual queue data
    bool mResize;           ///<  Automatic resizing of queue?
    int mReserved; ///<  Maximum reserved memory size
    int mElements; ///<  Number of elements in queue
    int mFront;    ///<  Front of queue
    int mBack;     ///<  Back of the queue
};


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param resv Amount of memory to reserve for queue
///   \param autoResize Allow automatic resizing if memory is filled up.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
Queue<T>::Queue(const int resv, const bool autoResize)
{
    mQueue = NULL;
    mResize = true;
    mReserved = mElements = mFront = mBack = 0;
    create(resv, autoResize);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param rhs The queue to be equal to.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
Queue<T>::Queue(const Queue<T> &rhs)
{
    mQueue = NULL;
    mResize = true;
    mReserved = mElements = mFront = mBack = 0;
    *this = rhs;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
Queue<T>::~Queue()
{
    destroy();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds the data to the back of the queue.
///
///   \param rhs Item to add to queue.
///
///   \return Size of the queue.  If <= 0, then queue is full and resizing is off.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
int Queue<T>::enqueue(const T &rhs)
{
    if(!mQueue && mResize)
    {
        resize(5);
    }
    else if(!mQueue)
    {
        return -1;
    }
    if(mElements == mReserved)
    {
        if(mResize)
        {
            //  Make the queue bigger to
            //  fit the new data
            resize(mReserved*2);
        }
        else
        {
            //  The queue is full!
            return -1;
        }
    }

    if(mElements == 0)
    {
        mQueue[mFront] = rhs;
        mBack = mFront;
        mBack++;
        if(mBack == mReserved)
        {
            mBack = 0;
        }
        mElements++;
        return mElements;
    }

    mQueue[mBack] = rhs;
    mBack++;
    mElements++;
    //  Hit the end of reserved memory
    //  wrap around to front of memory array
    if(mBack  == mReserved)
    {
        mBack = 0;
    }

    return (int)mElements;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data from the front of the queue.
///
///   \param rhs Item removed from queue.
///
///   \return Size of the queue.  If < 0, then queue is empty.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
int Queue<T>::dequeue(T &rhs)
{
    if(!mQueue || mElements == 0)
        return -1;

    rhs = mQueue[mFront];
    mFront++;
    mElements--;
    //  If we hit the end of the reserved memory
    //  then wrap around to beginning of array
    if(mFront == mReserved)
    {
        mFront = 0;
    }

    return (int)mElements;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data from the front of the queue.
///
///   \return Size of the queue.  If < 0, then queue is empty.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
T &Queue<T>::dequeue()
{
    int index = mFront;
    assert(mQueue && mElements > 0);

    mFront++;
    mElements--;
    //  If we hit the end of the reserved memory
    //  then wrap around to beginning of array
    if(mFront == mReserved)
    {
        mFront = 0;
    }

    return mQueue[index];
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \return The size of the queue.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
int Queue<T>::size() const
{
    return mElements;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates a queue, deletes all old data.
///
///   \param size Amount of memory to reserve for queue
///   \param autoResize Allow automatic resizing if memory is filled up.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
int Queue<T>::create(int size, const bool autoResize)
{
    destroy();
    if(size == 0)
        return 0;

    mResize = autoResize;
    mQueue = new T[size];
    assert(mQueue);
    mReserved = size;

    return mReserved;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the queue, saves data.  If queue is made smaller, then data
///   at the end of the queue is lost.
///
///   \return Size of the queue.
///
///   \param size The new size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
int Queue<T>::resize(const int size)
{
    if(size == 0)
    {    
        destroy();
        return 0;
    }
    
    T *ptr;
    int i, c;
    ptr = new T[size];
    assert(ptr);
    //  Copy the data from the previous array from front
    //  to back
    for(i = mFront, c = 0; c < size && c < mElements; c++)
    {
        ptr[c] = mQueue[i];
        i++;
        if(i == mReserved)
        {
            i = 0;
        }
    }

    //  The front of the queue is now 0
    mFront = 0;
    mReserved = size;
    mBack = c;

    if(mQueue)
    {
        delete[] mQueue;
        mQueue = NULL;
    }

    mQueue = ptr;

    //  If we made the new queue smaller, then
    //  we lost some data
    if(c < mElements)
    {
        mElements = c;
    }

    return mElements;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes all allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Queue<T>::destroy()
{
    if(mQueue)
    {
        delete[] mQueue;
        mQueue = NULL;
    }
    mResize = true;
    mReserved = mElements = mFront = mBack = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets data in the queue at the index.
///
///   \param i Index in queue (0 is front).
///   \param rhs Data at index in the queue.
///
///   \return True if get happened.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
bool Queue<T>::get(const int i, T &rhs) const
{
    if(i < 0 || i >= mElements)
        return false;

    int pos = mFront + i;

    if(pos >= mReserved)
    {
        pos -= mReserved;
    }

    rhs = mQueue[pos];

    return true;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets a pointer to data in the queue at the index.
///
///   \param i Index in queue (0 is front).
///   \param rhs Pointer at index in the queue.
///
///   \return True if get happened.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
bool Queue<T>::get(const int i, T **rhs) const
{
    if(i < 0 || i >= mElements)
        return false;

    int pos = mFront + i;

    if(pos >= mReserved)
    {
        pos -= mReserved;
    }

    rhs = &mQueue[pos];

    return true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Replaces a data in the queue at the index.
///
///   \param i Index in queue (0 is front).
///   \param rhs Data to copy into queue.
///
///   \return True if replace happened.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
bool Queue<T>::replace(const int i, const T &rhs)
{
    if(i < 0 || i >= mElements)
        return false;

    int pos = mFront + i;

    if(pos >= mReserved)
    {
        pos -= mReserved;
    }

    mQueue[pos] = rhs;

    return true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears the number of elements in the queue, but does not delete
///   allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Queue<T>::clear()
{
    mFront = mBack = 0;
    mElements = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to the queue.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
Queue<T> &Queue<T>::operator=(const Queue<T> &rhs)
{
    if(this != &rhs)
    {
        if(mReserved != rhs.mReserved)
        {
            create(rhs.mReserved);
        }
        for(int i = rhs.mFront, e = 0; e < rhs.mElements; e++)
        {
            mQueue[i] = rhs.mQueue[i];
            i++;
            if(i == mReserved)
                i = 0;
        }
        mFront = rhs.mFront;
        mBack = rhs.mBack;
        mElements = rhs.mElements;
        mResize = rhs.mResize;
    }

    return *this;
}

#endif
