/*==================================================================================

    Filename:  macros.h

    Copyright 2007 Brian C. Becker
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    File containing useful macros, inline functions, and defines.  Examples
    include:
    PI
    HALF_PI
    DEG2RAD
    GET_TIME_MS
    SLEEP

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
#ifndef _MACROS_H
#define _MACROS_H

#include <stdio.h>
#include <time.h>

#ifdef WIN32
	//#include <WinSock2.h>
    #include <windows.h>
#else
    #include <unistd.h>
    #include <sys/time.h>
    #include <string.h>
    #include <locale>
#endif


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Resizes an array, moving the elements in the old array to the new one
///
///  \param mem Pointer to memory to resize
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void NEWMEM(T *&mem)
{
    if (!mem)
        mem = new T;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Copies one array to another
///
///  \param mem Pointer to memory to copy to
///  \param memOrig Pointer to memory to copy from
///  \param size Size of the original array
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void COPY_MEM(T *&mem, const T *memOrig, int size)
{
    if (size > 0 && memOrig)
    {
        DELMEMS(mem);
        mem = new T[size];
        for (int i = 0; i < size; i++)
        {
            mem[i] = memOrig[i];
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Copies one array to another
///
///  \param mem Pointer to memory to copy to
///  \param memOrig Pointer to memory to copy from
///  \param delSize How much to delete from the old array
///  \param size Size of the original array
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void COPY_MEM(T *&mem, const T *memOrig, int &delSize, int size)
{
    if (delSize > 0 && size > 0 && memOrig)
    {
        DELMEMS(mem, delSize);
        mem = new T[delSize = size];
        for (int i = 0; i < size; i++)
        {
            mem[i] = memOrig[i];
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Resizes an array, moving the elements in the old array to the new one
///
///  \param mem Pointer to memory to resize
///  \param oldsize Existing size of the array
///  \param newsize New size of the array (-1 indicates increase to 2*oldsize)
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void RESIZE_MEM(T *&mem, int &oldsize, int newsize = -1)
{
    if (oldsize == 0 || mem == 0)
    {
        DELMEMS(mem);
        if (newsize <= 0) newsize = 1;
        mem = new T[newsize];
        return;
    }
    if (newsize == -1) newsize = oldsize*2;

    if (newsize <= 0) newsize = 1;
    T *nmem = new T[newsize];
    for (int i = 0; i < oldsize; i++)
    {
        nmem[i] = mem[i];
    }
    DELMEMS(mem);
    mem = nmem;
    // If we want, we can reset the size
    //    oldsize = newsize;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Deletes a memory spot and sets it to zero
///
///  \param mem Pointer to memory to delete. To delete an array, call DELMEMS
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void DELMEM(T *&mem)
{
    if (mem)
        delete mem;
    mem = 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Deletes a memory spot array and sets it to zero
///
///  \param mem Pointer to straight array to delete. 
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void DELMEMS(T *&mem)
{
    if (mem)
        delete[] mem;
    mem = 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Copies one string to another, allocating memory for the first string
///
///  \param mem Pointer to string to copy to
///  \param memOrig Pointer to string to copy from
///  \return The string that was made from the copy
///
////////////////////////////////////////////////////////////////////////////////////
inline char* COPY_STR(char *&mem, const char *memOrig)
{
    DELMEMS(mem);
    if (memOrig)
    {
        mem = new char[strlen(memOrig) + 1];
        strcpy(mem, memOrig);
    }

    return mem;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Copies one string and returns a pointer, allocating memory
///
///  \param memOrig Pointer to string to copy from
///  \return Pointer to the newly allocated string
///
////////////////////////////////////////////////////////////////////////////////////
inline char* COPY_STR(const char *memOrig)
{
    char *mem = 0;
    if (memOrig)
    {
        mem = new char[strlen(memOrig) + 1];
        strcpy(mem, memOrig);
    }
    else
    {
        mem = new char[1];
        strcpy(mem, "");
    }
    return mem;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Deletes contents of a memory, memory array, and sets it to zero
///
///  \param mems Pointer to memory array to delete.
///  \param count Number of elements in the array
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline void DELMEMS(T **&mems, int count)
{
    if (mems)
    {
        for (int i = 0; i < count; i++)
        {
            if (mems[i])
                delete mems[i];
        }
    
        delete[] mems;
    }
    mems = 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Cross platform system sleep call in ms.
///
///  \param ms Time to sleep in milliseconds
///
////////////////////////////////////////////////////////////////////////////////////
inline void SLEEP(int ms)
{
    #if defined(WIN32)
        Sleep((ms));
    #else
        usleep((ms*1000));
    #endif
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Gets the current system time in milliseconds
///
///  \return Current time in milliseconds
///
////////////////////////////////////////////////////////////////////////////////////
inline unsigned int GET_TIME_MS(void)
{
    unsigned int ms;

#ifdef WIN32
    SYSTEMTIME t;
    GetSystemTime(&t);
    //  Add up and convert to milliseconds
    ms = t.wHour*3600000 + t.wMinute*60000 + t.wSecond*1000 + t.wMilliseconds;
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ms = (unsigned int)(tv.tv_sec*1000 + tv.tv_usec*1000);
#endif

    return ms;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \def PI
///  \brief The value of pi.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef PI
#define  PI 3.14159265358979324
#endif

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \def HALF_PI
///  \brief Equal to PI/2.0, useful for steering angles.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef HALF_PI
#define  HALF_PI 1.57079632679489662
#endif

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Returns the minimum of two values.
///
///  \param a First element in comparison.
///  \param b Second element in comparison.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef MIN
template <class T>
inline T MIN(T a, T b)
{
    return ( ( (a) < (b) ) ? (a) : (b) );
}

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Returns the minimum of three values.
///
///  \param a First element in comparison.
///  \param b Second element in comparison.
///  \param c Third element in comparison.
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline T MIN(T a, T b, T c)
{
    return MIN(MIN(a, b), c);
}

#endif

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \def MAX
///  \brief Returns the maximum of two values.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef MAX
#define MAX(a,b) ( ( (a) > (b) ) ? (a) : (b) )
#endif


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \def DEG2RAD
///  \brief Converts from degrees to radians.
///  \return Value converted to radians, (double).
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef DEG2RAD
#define DEG2RAD(deg) ( (deg*0.0174532925) )
#endif

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \def RAD2DEG
///  \brief Converts from radians to degrees.
///  \return Value converted to degrees, (double).
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef RAD2DEG
#define RAD2DEG(rad) ( (rad*57.2957795) )
#endif

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Checks if two strings are equal
///  \param s1 First string to compare
///  \param s2 Second string to compare
///  \return True if strings are identical, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
inline bool EQ(const char *s1, const char *s2)
{
    if (!s1 || !s2)
        return false;
    return !strcmp(s1, s2);
}

#if !defined(RGB)
inline int RGB(unsigned char r, unsigned char g, unsigned char b)
{
    return (int)r | (int)g << 8 | (int) b << 16;
}
#endif

////////////////////////////////////////////////////////////////////////////////////
///
///  \brief Extract the RGB triplet from interleved data.
///
///  \param rgb Interleved rgb data
///  \param r Red value extracted.
///  \param g Green value extracted.
///  \param b Blue value extracted
///
////////////////////////////////////////////////////////////////////////////////////
inline void GETRGB(int rgb, unsigned char &r, unsigned char &g, unsigned char &b)
{
    r = (unsigned char)rgb;
    g = (unsigned char)(rgb >> 8);
    b = (unsigned char)(rgb >> 16);
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Converts a string to uppercase and returns it
///
///  \param in String to convert (modifies string pointed to)
///  \return The same string that was just modified
///
////////////////////////////////////////////////////////////////////////////////////
inline char *strToUpper(char *in)
{
    int size = (int)strlen(in);
    for (int i = 0; i < size; i++)
    {
        in[i] = (char)toupper(in[i]);
    }
    return in;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Caps to a specified value
///
///  \param t Value to cap
///  \param cap The cap
///  \return Capped value
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline T CAP(T t, const T cap)
{
    if (t > cap)
        t = cap;
    return t;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Caps between two values
///
///  \param t Value to cap
///  \param low The cap on the low end
///  \param high The cap on the high end
///  \return Capped value
///
////////////////////////////////////////////////////////////////////////////////////
template <class T>
inline T CAP(T t, const T low, const T high)
{
    if (t < low)
        t = low;
    else if (t > high)
        t = high;

    return t;
}
////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns the current date and time in a string format
///
///   \return Const pointer to a static string (not thread safe)
///
////////////////////////////////////////////////////////////////////////////////
inline const char* GET_DATETIME_STR()
{
	static char str[256];
	time_t rawtime;
	tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	sprintf(str, "%s", asctime(timeinfo));
	str[strlen(str)-1] = 0; // remove trailing \n
	return str;


	time_t now = time(0);
   //cout << "Number of sec since January 1,1970:" << now << endl;

   tm *ltm = localtime(&now);

   // print various components of tm structure.
   //cout << "Year: "<< 1900 + ltm->tm_year << endl;
   //cout << "Month: "<< 1 + ltm->tm_mon<< endl;
   //cout << "Day: "<<  ltm->tm_mday << endl;
   //cout << "Time: "<< 1 + ltm->tm_hour << ":";
   //cout << 1 + ltm->tm_min << ":";
   //cout << 1 + ltm->tm_sec << endl;

}
inline const char* GET_DATETIME_STR2()
{
	static char str[256];

	time_t now = time(0);
	tm *ltm = localtime(&now);

   // print various components of tm structure.
	int year = 1900 + ltm->tm_year;
	int month = 1 + ltm->tm_mon;
	int day = ltm->tm_mday;
	int hour =  ltm->tm_hour;
	int min = ltm->tm_min;
	int sec =  ltm->tm_sec;

	sprintf(str,"%04d_%02d_%02d_%02d_%02d_%02d", year, month, day, hour, min, sec); 
   	return str;

}
#endif

/* End of file */
