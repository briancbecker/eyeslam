/*==================================================================================

    Filename:  circulararray.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Template for creating circular arrays.  These arrays are of a limited size
    and do not automatically resize.  When the end of the array is reached than
    the oldest data is replaced with the latest.  This is useful when you need
    a maximum number of entries that may be time related.
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
#ifndef _CIRCULAR_ARRAY_H
#define _CIRCULAR_ARRAY_H

#ifdef __cplusplus

#include <stdlib.h>
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \def CIRCULAR_ARRAY_RECENT_FIRST
///   \brief Have the newest data at the front of the circular array (starting
///          at index 0), and oldest in the back.
///
////////////////////////////////////////////////////////////////////////////////////
#define CIRCULAR_ARRAY_RECENT_FIRST true
////////////////////////////////////////////////////////////////////////////////////
///
///   \def CIRCULAR_ARRAY_OLDEST_FIRST
///   \brief Have the oldest data at the front of the circular array (starting
///          at index 0), and newest in the back.
///
////////////////////////////////////////////////////////////////////////////////////
#define CIRCULAR_ARRAY_OLDEST_FIRST false

////////////////////////////////////////////////////////////////////////////////////
///
///   \class CircularArray
///   \brief Template for creating circular arrays.  
///
///   These arrays are of a limited size and do not automatically resize.  
///   When the end of the array is reached than the oldest data is replaced 
///   with the latest.
///
///   Depending on what direction is set, when indexing either the most
///   recent data is at index 0, or the oldest is at index 0.  By default
///   the most recent data is at the front of the array (index 0).
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
class CircularArray
{
public:
    CircularArray(const bool direction = CIRCULAR_ARRAY_RECENT_FIRST, 
                  const int size = 10);
    CircularArray(const CircularArray<T> &another);
    ~CircularArray();
    void clear();
    void destroy();
    void setDirection(const bool direction = CIRCULAR_ARRAY_RECENT_FIRST) const;
    int reserve(const int size);
    int resize(const int size);
    int add(const T &data);
    int push(const T &data);
    int get(const int index,
                     T &data);
    int get(const int index,
                     T **data);
    int reserved() const;
    int size() const;
    int elements() const;
    T &operator[](const int index) const;
    CircularArray<T> &operator=(const CircularArray<T> &another);
protected:
    bool mDirection;        ///<  Direction for accessing data.
    int mElements; ///<  Number of elements in the array.
    int mReserved; ///<  Amount of memory reserved.
    int mPosition; ///<  Position in the array.
    T *mArray;              ///<  The circular array.
};

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param direction What direction to access data form.  If 
///   CIRCULAR_ARRAY_RECENT_FIRST than the most recent data is accessed starting
///   at index 0, if CIRCULAR_ARRAY_OLDEST_FIRST than the oldest data begins
///   at index 0.
///   \param size Default size of array.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
CircularArray<T>::CircularArray(const bool direction,
                                const int size) : mElements(0),
                                                           mReserved(0),
                                                           mPosition(0),
                                                           mArray(0),
                                                           mDirection(direction)
{
    reserve(size);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
CircularArray<T>::CircularArray(const CircularArray<T> &another) : mElements(0),
                                                                   mReserved(0),
                                                                   mPosition(0),
                                                                   mArray(0),
                                                                   mDirection(true)
{
    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
CircularArray<T>::~CircularArray()
{
    destroy();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears contents, but does not delete memory.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void CircularArray<T>::clear()
{
    mPosition = mElements = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the direction for data access.
///
///   \param direction What direction to access data form.  If 
///   CIRCULAR_ARRAY_RECENT_FIRST than the most recent data is accessed starting
///   at index 0, if CIRCULAR_ARRAY_OLDEST_FIRST than the oldest data begins
///   at index 0.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void CircularArray<T>::setDirection(const bool direction) const
{
    bool *dir = (bool *)(&mDirection);
    *dir = direction;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes all allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void CircularArray<T>::destroy()
{
    if(mArray)
        delete[] mArray;

    mArray = NULL;
    mElements = mReserved = mPosition = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Reserve how large the array is.
///
///   \param size How large the array is.
///
///   \return 1 on ok, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::reserve(const int size)
{
    T *ptr = NULL;
    //  Delete old data
    destroy();

    ptr = new T[size];
    assert(ptr);

    mArray = ptr;
    mReserved = size;
    mPosition = mElements = 0;

    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the array and preserves any existing data when possible.
///
///   \param size The new size of the array.
///
///   \return 1 on ok, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::resize(const int size)
{
    if(size == mReserved)
        return 1;
    
    T *ptr = new T[size];
    assert(ptr);
    
    int limit = mElements;
    if(mElements > size)
        limit = size;

    //  Copy the data backwards from most recent to oldest
    for(int i = mPosition, j = 0, r = 0; r < limit; i--, j--, r++)
    {
        ptr[j] = mArray[i];
        if(i == 0)
        {
            i = mReserved;
        }
        if(j == 0)
        {
            j = size;
        }
    }

    if(mArray)
        delete[] mArray;

    mArray = ptr;
    mElements = limit;
    mReserved = size;
    mPosition = 0;

    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Get data at an index.
///
///   \param index Index in the array.  Depending on direction the most
///                recent data is at front, or oldest.  (front = index 0).
///   \param data Data copied.
///
///   \return 1 on ok, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::get(const int index, T &data)
{
    if(index >= mElements)
        return 0;
    
    if(mDirection)
    {
        int pos;
        if(mPosition < index)
        {
            pos = (mPosition + mReserved) - index;
        }
        else
        {
            pos = mPosition - index;
        }
        
        data = mArray[pos];
    }
    else
    {
        int pos;
        pos = mPosition + index + 1;
        if(pos >= mReserved)
            pos -= mReserved;

        data = mArray[pos];
    }

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Get pointer to data at an index.
///
///   \param index Index in the array.  Depending on direction the most
///                recent data is at front, or oldest.  (front = index 0).
///   \param data Data copied.
///
///   \return 1 on ok, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::get(const int index, T **data)
{
    if(index >= mElements)
        return 0;
    
    if(mDirection)
    {
        int pos;
        if(mPosition < index)
        {
            pos = (mPosition + mReserved) - index;
        }
        else
        {
            pos = mPosition - index;
        }
        
        *data = &mArray[pos];
    }
    else
    {
        int pos;
        pos = mPosition + index + 1;
        if(pos >= mReserved)
            pos -= mReserved;

        *data = &mArray[pos];
    }

    return 1;
}



////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Add data to the array.  
///
///   \param data Data to be added.
///
///   \return Number of elements in array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::add(const T &data)
{
    if(!mArray)
        return 0;

    mPosition++;
    if(mPosition >= mReserved)
        mPosition = 0;

    mArray[mPosition] = data;

    if(mElements < mReserved)
        mElements++;

    return mElements;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Add data to the array.  
///
///   \param data Data to be added.
///
///   \return Number of elements in array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::push(const T &data)
{
    if(!mArray)
        return 0;

    mPosition++;
    if(mPosition >= mReserved)
        mPosition = 0;

    mArray[mPosition] = data;

    if(mElements < mReserved)
        mElements++;

    return mElements;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return The maximum size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::reserved() const { return mReserved; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return The number of elements in the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::size() const { return mElements; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return The number of elements in the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int CircularArray<T>::elements() const { return mElements; }


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets data the index.  Order data comes is based on direction. If 
///   CIRCULAR_ARRAY_RECENT_FIRST than the most recent data is accessed starting
///   at index 0, if CIRCULAR_ARRAY_OLDEST_FIRST than the oldest data begins
///   at index 0.
///
///   \param index Index in array to get data from [0, size()/elements()).
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
T &CircularArray<T>::operator[](const int index) const
{
    assert(index < mElements && index < mReserved);
    
    if(mDirection)
    {
        int pos;
        if(mPosition < index)
        {
            pos = (mPosition + mReserved) - index;
        }
        else
        {
            pos = mPosition - index;
        }
        
        return mArray[pos];
    }
    else
    {
        int pos;
        pos = mPosition + index + 1;
        if(pos >= mReserved)
            pos -= mReserved;

        return mArray[pos];
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies data.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
CircularArray<T> &CircularArray<T>::operator=(const CircularArray<T> &another)
{
    if(this != &another)
    {
        if(mReserved != another.mReserved)
        {
            reserve(another.mReserved);
        }
        clear();
        for(int i = 0; i < another.mElements; i++)
        {
            add(another[i]);
        }
    }

    return *this;
}

#endif
#endif

/* End of File */
