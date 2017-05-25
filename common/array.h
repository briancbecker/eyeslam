/*==================================================================================

    Filename:  array.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Template for creating arrays that can be sorted, resized, etc.  This is
    very similar to the vector template in the STL, but is simpler and easier
    to debug when things go wrong.
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
#ifndef _ARRAY_H
#define _ARRAY_H

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \class Array
///   \brief Template class for creating arrays.
///
///   This template is almost similar to the vector template in the STL, but
///   has less code to debug if something else is wrong with code.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
class Array
{
public:
    Array();                                        ///<  Constructor
    Array(const unsigned int size);                 ///<  Create with default reserved size
    Array(const Array<T> &rhs);                     ///<  Array to be equal to
    ~Array();                                       ///<  Destructor
    void swap(Array<T> &rhs);                       ///<  Swap contents
    void destroy();                                 ///<  Destroys all data in array
    void clear();                                   ///<  Clear array without deleting allocated memory
    void sort();                                    ///<  Sorts data in the array based on compare function
    int add(const T &rhs);                          ///<  Resize the array + 1 and adds to back
    int push(const T &rhs);                         ///<  Resize the array + 1 and adds to back
    int push_back(const T &rhs){return push(rhs);}  ///<  Resize the array + 1 and adds to back
    int pop(T &rhs);                                ///<  Remove element from end of array and resizes
    T &pop();                                       ///<  Deletes data at end of stack
    int remove();                                   ///<  Deletes data from end of stack
    int remove(const unsigned int i);               ///<  Remove data at index
    unsigned int memcpy(const Array<T> &rhs);       ///<  Performs a memory copy of the contents of rhs (BE CAREFUL)
    int size() const;                      ///<  Size of array
    unsigned int reserved();                        ///<  Total reserved array size 
    unsigned int create(const unsigned int size);   ///<  Deletes existing data and creates a new sized array
    unsigned int resize(const unsigned int size);   ///<  Resize the array (saves existing data that fits)
    unsigned int reserve(const unsigned int size);  ///<  Reserve a specific amount of memory to reduce memory allocations
    T *ptr() const;                                 ///<  BE CAREFUL!  Gets direct pointer to data (for speed purposes)
    T &operator[](const unsigned int index) const;  ///<  Get data at index
    Array<T> &operator=(const Array<T> &rhs);       ///<  Set equal to
    Array<T> &operator+=(const Array<T> &rhs);      ///<  Add the array onto the end of this array
    Array<T> operator+(const Array<T> &rhs);        ///<  Add two arrays together (doesn't add data)
    void setCompareFunction(bool (*compareFunc)(const T&, const T &)); ///<  Set comparison function for sorting
    T &get(const unsigned int index) const;         ///<  Get data at index
    bool contains(T &rhs, bool binSearch = false);  ///<  See if array contains an item

	static void swap(T &left, T &right);
protected:
    void mergesort(T *&data, int length, int reserve = -1);
    void mergesortSimple(T *&data, int length, int reserve = -1);
    void mergesortInnerSimple(T *ord, T *beg1, T *beg2, T *end1, T *end2, int length);
    void mergesortInner(T *ord, T *beg1, T *beg2, T *end1, T *end2, int length);
    bool (*mCompare)(const T &, const T &);         ///<  Compare function
    T *mData;                                       ///<  Actual data
    unsigned int mLength;                           ///<  Length of array
    unsigned int mReserved;                         ///<  Reserved length
};

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T>::Array()
{
    mLength = 0;
    mReserved = 0;
    mData = NULL;
    mCompare = NULL;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param size The size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T>::Array(unsigned int size)
{    
    mLength = 0;
    mReserved = 0;
    mData = NULL;
    mCompare = NULL;
    reserve(size);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.  Also copies data in array passed to it.
///
///   \param rhs The array to be equal to.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T>::Array(const Array<T> &rhs)
{    
    mLength = 0;
    mReserved = 0;
    mData = NULL;
    mCompare = NULL;
    *this = rhs;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T>::~Array()
{
    destroy();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes the entire array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::destroy()
{
    if(mData)
    {
        delete[] mData;
    }
    mLength = 0;
    mReserved = 0;
    mData = NULL;    
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the size of the array to 0, but does not change the amount of
///   reserved memory.  To delete memory, call destroy.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::clear()
{
    mLength = 0;  
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Swaps the contents of the arrays.
///
///   \param rhs The array to swap data with.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::swap(Array<T> &rhs)
{
    T *tPtr;
    unsigned int sTemp;

    //  Swap pointers
    tPtr = mData;
    mData = rhs.mData;
    rhs.mData = tPtr;

    //  Swap sizes
    sTemp = mLength;
    mLength = rhs.mLength;
    rhs.mLength = sTemp;

    //  Swap function pointers
    bool (*compareT)(const T &, const T &);
    compareT = this->mCompare;
    this->mCompare = rhs.mCompare;
    rhs.mCompare = compareT;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sorts the data in the array based on the compare function provided.
///
///   Uses quicksort or mergesort to sort data.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::sort()
{
    if(!mCompare)
        return;

    if(mLength >= 2)
    {
        // Choose the sorting algorithm here
        mergesort(mData, mLength, mReserved);
        //quicksort(mData, 0, mLength - 1);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sorts the data in the array based on the compare function provided.
///
///   Uses quicksort or mergesort to sort data.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline void Array<int>::sort()
{
    if(mLength >= 2)
    {
        // Choose the sorting algorithm here
        mergesortSimple(mData, mLength, mReserved);
        //quicksort(mData, 0, mLength - 1);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sorts the data in the array based on the compare function provided.
///
///   Uses quicksort or mergesort to sort data.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline void Array<float>::sort()
{
    if(mLength >= 2)
    {
        // Choose the sorting algorithm here
        mergesortSimple(mData, mLength, mReserved);
        //quicksort(mData, 0, mLength - 1);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sorts the data in the array based on the compare function provided.
///
///   Uses quicksort or mergesort to sort data.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline void Array<double>::sort()
{
    if(mLength >= 2)
    {
        // Choose the sorting algorithm here
        mergesortSimple(mData, mLength, mReserved);
        //quicksort(mData, 0, mLength - 1);
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds new data to the end of the array.  Resizes the array to do this.
///
///   \param rhs Data to add to end of array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::add(const T &rhs)
{
    return push(rhs);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds new data to the end of the array.  Resizes the array to do this.
///
///   \param rhs Data to add to end of array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::push(const T &rhs)
{
    if(mLength == 0 && mReserved == 0)
    {
        reserve(4);
    }
    else if(mLength + 1 == mReserved)
    {
        reserve(mReserved*2);
    }
    mData[mLength] = rhs;
    mLength++;
    return mLength;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            for(unsigned int pos = i; pos < mLength - 1; pos++)
                mData[pos] = mData[pos + 1];
            //::memcpy(&mData[i], &mData[i + 1], sizeof(T)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<int>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(int)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<int *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(int *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<short>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(short)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<short *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(short *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<unsigned int>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(unsigned int)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<unsigned int *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(unsigned int *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<float>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(float)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<float *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(float *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<double>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(double)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<double *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(double *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<unsigned char>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(unsigned char)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<unsigned char *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(unsigned char *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<char>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(char)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data at the index.
///
///   \param i Index in array to remove data from.
///
///   \return New size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline int Array<char *>::remove(const unsigned int i)
{
    if(i >= 0 && i < mLength)
    {
        if(i == mLength - 1)
            mLength--;
        else
        {
            ::memcpy(&mData[i], &mData[i + 1], sizeof(char *)*(mLength - 1 - i));
            mLength--;
        }
    }

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data to the end of the array.  Resizes the array to do this.
///
///   \param rhs Where to save the data removed from the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::pop(T &rhs)
{
    if(!mData || mLength == 0)
        return -1;

    rhs = mData[mLength - 1];
    mLength--;

    if(mLength < (unsigned int)(mReserved*.5) && mLength > 0)
        resize(mLength);

    return mLength;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data to the end of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::remove()
{
    if(!mData || mLength == 0)
        return -1;

    mLength--;

    if(mLength < (unsigned int)(mReserved*.5) && mLength > 0)
        resize(mLength);

    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Removes data to the end of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
T &Array<T>::pop()
{
    assert(mData && mLength > 0);
    if(mLength < (unsigned int)(mReserved*.5) && mLength > 0)
        resize(mLength);
    mLength--;
    return mData[mLength];
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies the memory data in the parameter to internal data members.  
///
///   This is different than using operator= because during a call to
///   operator= each element in the array is copied using the types own
///   = function.  This is done to make sure the data is copied, and not just
///   pointers.  However this can be a slow operation when you don't have 
///   data structures with pointers to memory (like array of int).  So you can
///   use this memoryCopy function which does copy the entire array to
///   internal data members.
///
///   \param rhs Array to memcpy data from.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
unsigned int Array<T>::memcpy(const Array<T> &rhs)
{ 
    if(rhs.mLength == 0)
    {
        mLength = 0;
        return 0;
    }
    if(mReserved < rhs.mReserved)
    {
        reserve(rhs.mRserved);
    }
    ::memcpy(mData, rhs.mData, sizeof(T)*rhs.mLength);
    mLength = rhs.mLength;
    return mLength;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return The size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
int Array<T>::size() const { return mLength; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief The reserved amount of memory for storing the array.  This amount
///   is equal to size() + .5size().  This is done to make push and pop functions 
///   faster.
///
///   \return The reserved amount of memory for storing the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
unsigned int Array<T>::reserved() { return mReserved; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates an array of the size <size>, deletes any existing data.
///
///   If you wish to resize the array only, so you can save existing data use
///   the resize function.
///
///   \param size The new array size.
///
///   \return The size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
unsigned int Array<T>::create(unsigned int size)
{
    destroy();

    if(size == 0)
        return 0;

    mData = new T[size + size/2];
    assert(mData);
    memset(mData, 0, sizeof(T)*(size + size/2));
    mLength = size;
    mReserved = size + size/2;

    return mLength;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the array without deleting existing data.  If the new size
///   is smaller than the current size, then data up to the new size will be
///   saved, everything after deleted.
///
///   \param size The new array size.
///
///   \return The size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
unsigned int Array<T>::resize(const unsigned int size)
{
    if(size == 0)  
    {
        mLength = 0;            //  Do a clear
        return 0;
    }

    //  If we are really big, than we need
    //  to reduce the amount of memory already 
    //  allocated
    if(mData && mLength > size + 500)
        return reserve(size + size/2);

    if(size < mLength && mData) //  Already have reserved memory larger
    {
        mLength = size;
        return mLength;
    }

    /*
    if(size > mLength && size < mReserved && mData)
    {
        mLength = size;
        return mLength;
    }
    */
    if(size == mLength && mData) //  Already the correct size
        return mLength;

    T *newPtr = NULL;
    newPtr = new T[size + size/2];
    assert(newPtr);
    mReserved = size + size/2;

    if(mData)
    {
        for(unsigned int i = 0; i < mLength && i < size; i++)
        {
            newPtr[i] = mData[i];
        }

        delete[] mData;
        mData = NULL;
    }

    mData = newPtr;
    mLength = size;

    return mLength;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the reserved array size.  This is not equal to the size
///   of the number of elements in the array, but the amount of items reserved
///   maximum for the array to fill using push/add type functions.
///
///   \param size The new array reserved size.
///
///   \return The reserved size of the array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
unsigned int Array<T>::reserve(const unsigned int size)
{
    if(size == 0)
    {
        destroy();
        return 0;
    }
    else if(size == mReserved)
        return mReserved;

    T *newPtr = NULL;
    newPtr = new T[size];
    assert(newPtr);

    for(int i = 0; i < (int)size && i < (int)mLength; i++)
    {
        newPtr[i] = mData[i];
    }

    if(mData)
    {
        delete[] mData;
        mData = NULL;
    }

    //  Only change size if we have reserved
    //  less memory
    if(mLength > size)
        mLength = size;

    //  Copy the pointer
    mData = newPtr;
    mReserved = size;
    
    return mReserved;
}



////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Indexing through array.
///
///   \return Element at index.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
T &Array<T>::operator[](const unsigned int index) const
{
    assert(index < mLength);
    return mData[index];
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T> &Array<T>::operator=(const Array<T> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        for(int i = 0; i < (int)rhs.mLength; i++)
        {
            mData[i] = rhs.mData[i];
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<int> &Array<int>::operator=(const Array<int> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(int)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<unsigned int> &Array<unsigned int>::operator=(const Array<unsigned int> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(unsigned int)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<unsigned short> &Array<unsigned short>::operator=(const Array<unsigned short> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(unsigned short)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<short> &Array<short>::operator=(const Array<short> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(short)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<char> &Array<char>::operator=(const Array<char> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(char)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<unsigned char> &Array<unsigned char>::operator=(const Array<unsigned char> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(unsigned char)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<float> &Array<float>::operator=(const Array<float> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(float)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<double> &Array<double>::operator=(const Array<double> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(double)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<long> &Array<long>::operator=(const Array<long> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(long)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets array equal to other array.
///
////////////////////////////////////////////////////////////////////////////////////
template<>
inline Array<unsigned long> &Array<unsigned long>::operator=(const Array<unsigned long> &rhs)
{
    if(this != &rhs)
    {
        if(this->mLength != rhs.mLength)
            create(rhs.mLength);
        
        if(rhs.mLength > 0)
        {
            ::memcpy(mData, rhs.mData, sizeof(unsigned long)*rhs.mLength);
        }

        mCompare = rhs.mCompare;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Add the data in <rhs> onto the end of this array.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T> &Array<T>::operator+=(const Array<T> &rhs)
{
    int k = 0;
    int pos = mLength;
    resize(mLength + rhs.mLength);
    for(int i = pos; i < (int)mLength && k < (int)rhs.mLength; i++)
    {
        mData[i] = rhs.mData[k];
        k++;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Add the data in <rhs> onto the end of this array and return a copy.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
Array<T> Array<T>::operator+(const Array<T> &rhs)
{
    Array<T> sum(*this);   
    sum += rhs;
    return sum;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets what function to use for comparing data during sorting.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::setCompareFunction(bool (*compareFunc)(const T &, const T &))
{
    mCompare = compareFunc;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Swaps elements.
///
///   \param left Left element to switch with right
///   \param right Right element to switch with left
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
void Array<T>::swap(T &left, T &right)
{
    T temp;
    temp = left;
    left = right;
    right = temp;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sometimes speed is importantant when processing data.  Use this
///   function very carefully.
///
///   \return Direct pointer to data <b>BE CAREFUL!</b>
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
T *Array<T>::ptr() const
{
    return mData;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Inner loop of the mergesort
///
///   Sorts two buffers (beg1 and beg2) into an third buffer (ord)
///
///   \param ord Buffer to be filled with the ordered numbers
///   \param beg1 Beginning of buffer 1
///   \param beg2 Beginning of buffer 2
///   \param end1 End of buffer 1
///   \param end2 End of buffer 2
///   \param length Length of buffer1 + buffer2
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
void Array<T>::mergesortInner(T *ord, T *beg1, T *beg2, T *end1, T *end2, int length)
{
    int ctr = length;
    while (ctr--)
    {
        if (mCompare(*beg1, *beg2))
        {
            *ord++ = *beg1++;
            if (beg1 == end1)
            {
                for (T *d = beg2; d < end2; d++)
                {
                    *ord++ = *d;
                }
                break;
            }
        }
        else
        {
            *ord++ = *beg2++;
            if (beg2 == end2)
            {
                for (T *d = beg1; d < end1; d++)
                {
                    *ord++ = *d;
                }
                break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Inner loop of the mergesort
///
///   Sorts two buffers (beg1 and beg2) into an third buffer (ord) 
///   [optimized for simple data types]
///
///   \param ord Buffer to be filled with the ordered numbers
///   \param beg1 Beginning of buffer 1
///   \param beg2 Beginning of buffer 2
///   \param end1 End of buffer 1
///   \param end2 End of buffer 2
///   \param length Length of buffer1 + buffer2
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
void Array<T>::mergesortInnerSimple(T *ord, T *beg1, T *beg2, T *end1, T *end2, int length)
{
    int ctr = length;
    while (ctr--)
    {
        if (mCompare ? mCompare(*beg1, *beg2) : *beg1 < *beg2)
        {
            *ord++ = *beg1++;
            if (beg1 == end1)
            {
                for (T *d = beg2; d < end2; d++)
                {
                    *ord++ = *d;
                }
                break;
            }
        }
        else
        {
            *ord++ = *beg2++;
            if (beg2 == end2)
            {
                for (T *d = beg1; d < end1; d++)
                {
                    *ord++ = *d;
                }
                break;
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Mergesorts a buffer of data types
///
///   Sorts a buffer of data types of a certain length using mergesort
///   Uses O(n) memory, which is created and destroyed. Incoming data pointer
///   may be modified (swapped out)
///
///   \param data Pointer to buffer to sort.
///   \param length Length of the buffer to sort.
///   \param reserve Reserve size needed for buffer.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
void Array<T>::mergesort(T *&data, int length, int reserve /*= -1*/)
{
    int steps = 0;
    int step = 1;
    int twoStep;
    T *beg1, *beg2, *end1, *end2, *ord;
    T *tmp = 0;

    if (reserve == -1)
        reserve = length;

    if (length > 1)
    {
        tmp = new T[reserve];
        while (step < length)
        {
            assert(tmp);
            steps = length / step / 2;
            twoStep = step << 1;
            //if (steps <= 0)
            //    steps = 1;

            ord = tmp;
            beg1 = data;
            beg2 = beg1 + step;
            end1 = beg1 + step;
            end2 = beg2 + step;

            for (int i = steps; i > 0; i--)
            {
                mergesortInner(ord, beg1, beg2, end1, end2, twoStep);
                ord += twoStep;
                beg1 += twoStep;
                beg2 += twoStep;
                end1 += twoStep;
                end2 += twoStep;
            }

            end2 = data + length;
            if (beg2 < end2)
            {
                mergesortInner(ord, beg1, beg2, end1, end2, step + length % step);
            }
            else
            {
                for (; beg1 < data + length; beg1++)
                    *ord++ = *beg1;
            }

            //end2 = data + length;
            //mergesortInner(ord, beg1, beg2, end1, end2);

            // Swap pointers
            ord = tmp;
            tmp = data;
            data = ord;

            step <<= 1;
        }

        delete[] tmp;
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Mergesorts a buffer of data types
///
///   Sorts a buffer of data types of a certain length using mergesort
///   Uses O(n) memory, which is created and destroyed. Incoming data pointer
///   may be modified (swapped out)
///   [optimized for simple data types]
///
///   \param data Pointer to buffer to sort.
///   \param length Length of the buffer to sort.
///   \param reserve Reserve size needed for buffer.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
void Array<T>::mergesortSimple(T *&data, int length, int reserve /*= -1*/)
{
    int steps = 0;
    int step = 1;
    int twoStep;
    T *beg1, *beg2, *end1, *end2, *ord;
    T *tmp = 0;

    if (reserve == -1)
        reserve = length;

    if (length > 1)
    {
        tmp = new T[reserve];
        while (step < length)
        {
            assert(tmp);
            steps = length / step / 2;
            twoStep = step << 1;
            //if (steps <= 0)
            //    steps = 1;

            ord = tmp;
            beg1 = data;
            beg2 = beg1 + step;
            end1 = beg1 + step;
            end2 = beg2 + step;

            for (int i = steps; i > 0; i--)
            {
                mergesortInnerSimple(ord, beg1, beg2, end1, end2, twoStep);
                ord += twoStep;
                beg1 += twoStep;
                beg2 += twoStep;
                end1 += twoStep;
                end2 += twoStep;
            }

            end2 = data + length;
            if (beg2 < end2)
            {
                mergesortInnerSimple(ord, beg1, beg2, end1, end2, step + length % step);
            }
            else
            {
                for (; beg1 < data + length; beg1++)
                    *ord++ = *beg1;
            }

            //end2 = data + length;
            //mergesortInner(ord, beg1, beg2, end1, end2);

            // Swap pointers
            ord = tmp;
            tmp = data;
            data = ord;

            step <<= 1;
        }

        delete[] tmp;
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Indexing through array.
///
///   \return Element at index.
///
////////////////////////////////////////////////////////////////////////////////////
template<class T>
T &Array<T>::get(const unsigned int index) const
{
    assert(index < mLength);
    return mData[index];
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Check to see if this array contains a specific element
///
///   \param rhs The element to check for
///   \param binSearch Perform a binary search (array must be sorted) [todo]
///   \return True if element is already in array, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
bool Array<T>::contains(T &rhs, bool binSearch)
{
    if (!binSearch)
    {
        for (unsigned int i = 0; i < this->mLength; i++)
        {
            if (mData[i] == rhs)
            {
                return true;
            }
        }
        return false;
    }

    return false;
}

#endif
/* End of file */
