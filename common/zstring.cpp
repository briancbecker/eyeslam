/*==================================================================================

    Filename:  string.cpp

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    String class as an alternative to the STL string.  Contains additional string
    manipulation functions not available with the STL string.  Additional 
    functions exisit so that it is still compatible with STL string through 
    implicit and overloaded operators.
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

#include "zstring.h"
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

using namespace std;

#define STRING_MIN_RESERVED 16


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////
String::String()
{
    mReserved = mLength = 0;
    mString = NULL;
    mString = new char[1];
    mString[0] = '\0';
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////
String::String(const String &rhs)
{
    mReserved = mLength = 0;
    mString = NULL;
    *this = rhs;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////
String::String(const char *string)
{
    mReserved = mLength = 0;
    mString = NULL;
    *this = string;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////
String::~String()
{
    //  Don't call the destroy function because
    //  it doesn't delete all memory (leaves 1 byte)
    //  so that you can have an empty string of "".
    if(mString)
        delete[] mString;
    mString = NULL;
    mReserved = mLength = 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out what kind of string it is.
///
///   This is useful to determine if the string is a mixed string or a single
///   integer or double value, so it can be converted.
///
///   \return STRING_TYPE_NULL (0) if the string is NULL, STRING_TYPE_INT (2) if 
////  the string is an integer, STRING_TYPE_DOUBLE (3) if it is a floating point 
////  value, otherwise returns STRING_TYPE_STRING (1).
///
////////////////////////////////////////////////////////////////////////////////
int String::type() const
{
    //  check for undefined string type
    if(mString == NULL || mLength == 0)
        return STRING_TYPE_NULL;

    int result = STRING_TYPE_INT;
    char *ptr = mString;
    for(unsigned int i = 0; i < mLength; i++, ptr++)
    {
        if(*ptr < 48 || *ptr > 57)
        {
            //  If it is not a digit character than it
            //  is a string, so return so
            if(*ptr != '-' && *ptr != '.')
                return STRING_TYPE_STRING;
            else if(*ptr == '.')
            {
                //  If there is a decimal, than it is a
                //  floating point number, so assign as so
                result = STRING_TYPE_DOUBLE;
            }
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the string represents an integer value, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////
bool String::isInteger() const
{
    if(mString == NULL || mLength == 0)
        return false;
    char *ptr = mString;
    for(unsigned int i = 0; i < mLength; i++, ptr++)
    {
        if(*ptr < 48 || *ptr > 57)
        {
            if(*ptr != '-')
                return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the string is a number, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////
bool String::isNumber() const
{
    if(mString == NULL || mLength == 0)
        return false;
    char *ptr = mString;
    for(unsigned int i = 0; i < mLength; i++, ptr++)
    {
        if(*ptr < 48 || *ptr > 57)
        {
            if(*ptr != '-' && *ptr != '.')
                return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief True if the string is a double value, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////
bool String::isDouble() const
{
    if(mString == NULL || mLength == 0)
        return false;
    char *ptr = mString;
    bool foundDecimal = false;
    for(unsigned int i = 0; i < mLength; i++, ptr++)
    {
        if(*ptr < 48 || *ptr > 57)
        {
            if(*ptr != '-' && *ptr != '.')
                return false;
            else if(*ptr == '.')
                foundDecimal = true;
        }
    }

    return foundDecimal;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to an integer.
///
///   \return The integer representation.  If an error occurs 0 will be returned, 
///   but this won't tell you much.  
///
////////////////////////////////////////////////////////////////////////////////
int String::toInt() const
{
    int result;

    if(mLength == 0 || !mString)
        return 0;
    
    result = atoi(mString);
    return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to a long value.
///
///   \param lnum Where to save the converted string.
///
///   \return True ok, if the string contained a long number. 
///
////////////////////////////////////////////////////////////////////////////////
bool String::toLong(long *lnum) const
{
    if(isInteger())
    {
        *lnum = atol(mString);
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to a integer value.
///
///   \param inum Where to save the converted string.
///
///   \return True on success, false if number wasn't integer.
///
////////////////////////////////////////////////////////////////////////////////
bool String::toInt(int *inum) const
{
    if(isInteger())
    {
        *inum = atoi(mString);
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to a double value.
///
///   \param dnum Where to save the converted string.
///
///   \return True on success, false if number wasn't double.
///
////////////////////////////////////////////////////////////////////////////////
bool String::toDouble(double *dnum) const
{
    if(isNumber())
    {
        *dnum = atof(mString);
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to a long value.
///
///   \return The long representation.  If an error occurs 0 will be returned, 
///   but this won't tell you much.  
///
////////////////////////////////////////////////////////////////////////////////
long String::toLong() const
{
    long result;

    if(mLength == 0 || !mString)
        return 0;
    
    result = atol(mString);
    return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to an double.
///
///   \return The double representation.  If an error occurs 0 will be returned, 
///   but this won't tell you much.  
///
////////////////////////////////////////////////////////////////////////////////
double String::toDouble() const
{
    double result;

    if(mLength == 0 || !mString)
        return 0.0;
    
    result = atof(mString);
    return result;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets what the string should be equal to.  Uses a method similar to
///   printf and sprintf where it is possible to created a formatted string.
///
///   The format and usage used is identical to printf.
///
///   Example:  set("My name is %s, and I am %d\n", "Daniel Barber", 26);
///   Result:  String will be equal to "My name is Daniel Barber, and I am 26\n"
///
///   \return The number of bytes encoded in the string. 
///
////////////////////////////////////////////////////////////////////////////////
String & String::set(const char *format, ...)
{
    va_list argptr;
    int nump = 0;
    char buffer[128];
    char fformat[16];
    
    clear();

    if(format == NULL)
        return *this;

    nump = (int)(occurrences("%", format));
    va_start(argptr, format);
    for(unsigned long i = 0; format[i] != '\0' || format[i] != 0; )
    {
        if(format[i] != '%')
        {
            append(format[i]);
            i++;
        }
        else
        {
            i++;
            if(format[i] == 's' || format[i] == 'S')
            {
                *this += va_arg(argptr, char *);
                i++;
            }
            else
            {
                
                int pos = 0;
                fformat[pos] = '%';
                pos++;
                while(pos < 16)
                {
                    if(format[i] == 'c' ||
                       format[i] == 'd' ||
                       format[i] == 'i' ||
                       format[i] == 'e' ||
                       format[i] == 'E' ||
                       format[i] == 'f' ||
                       format[i] == 'g' ||
                       format[i] == 'G' ||
                       format[i] == 'o' ||
                       format[i] == 'u' ||
                       format[i] == 'x' ||
                       format[i] == 'X' )
                    {
                        break;
                    }
                    else
                    {
                        fformat[pos] = format[i];
                        i++;
                        pos++;
                    }
                }
                fformat[pos] = format[i];
                i++;
                fformat[pos + 1] = '\0';
                if(fformat[pos] == 'c')
                {
                    ::sprintf(buffer, fformat, (char)va_arg(argptr, int));
                }
                else if(fformat[pos] == 'd' || fformat[pos] == 'i' || fformat[pos] == 'x' || fformat[pos] == 'X')
                {
                    int dec = va_arg(argptr, int);
                    ::sprintf(buffer, fformat, dec);
                }
                else if(fformat[pos] == 'f' || 
                        fformat[pos] == 'e' || 
                        fformat[pos] == 'E' ||
                        fformat[pos] == 'g' || 
                        fformat[pos] == 'G')
                {
                    double dbl = va_arg(argptr, double);
                    if(fformat[pos - 1] == 'l')
                    {
                        ::sprintf(buffer, fformat, dbl);
                    }
                    else
                    {
                        ::sprintf(buffer, fformat, (float)dbl);
                    }
                }
                else
                {
                    ::sprintf(buffer, fformat, va_arg(argptr, int));
                }

                append(buffer);
            }
        }
    }
    
    va_end(argptr);

    return *this;
}



////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets what the string should be equal to.  Uses a method similar to
///   printf and sprintf where it is possible to created a formatted string.
///
///   The format and usage used is identical to printf.
///
///   Example:  set("My name is %s, and I am %d\n", "Daniel Barber", 26);
///   Result:  String will be equal to "My name is Daniel Barber, and I am 26\n"
///
///   \return The number of bytes encoded in the string. 
///
////////////////////////////////////////////////////////////////////////////////
String & String::sprintf(const char *format, ...) 
{
    va_list argptr;
    
    clear();

    if(format == NULL)
        return *this;

    va_start(argptr, format);

	vsprintf(format, argptr);
    
    va_end(argptr);

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets what the string should be equal to.  Uses a method similar to
///   printf and sprintf where it is possible to created a formatted string.
///
///   This version directly takes a variable argument pointer so it can be
///   easily used from functions that want sprintf-functionality with other
///   arguments.
///
///   The format and usage used is identical to printf.
///
///   Example:  set("My name is %s, and I am %d\n", "Daniel Barber", 26);
///   Result:  String will be equal to "My name is Daniel Barber, and I am 26\n"
///
///   \return The number of bytes encoded in the string. 
///
////////////////////////////////////////////////////////////////////////////////
String & String::vsprintf(const char *format, va_list argptr)
{
	int nump = 0;
	char buffer[128];
	char fformat[16];

    nump = (int)(occurrences("%", format));

	for(unsigned long i = 0; format[i] != '\0' || format[i] != 0; )
	{
		if(format[i] != '%')
		{
			append(format[i]);
			i++;
		}
		else
		{
			i++;
			if(format[i] == 's' || format[i] == 'S')
			{
				*this += va_arg(argptr, char *);
				i++;
			}
			else
			{

				int pos = 0;
				fformat[pos] = '%';
				pos++;
				while(pos < 16)
				{
					if(format[i] == 'c' ||
						format[i] == 'd' ||
						format[i] == 'i' ||
						format[i] == 'e' ||
						format[i] == 'E' ||
						format[i] == 'f' ||
						format[i] == 'g' ||
						format[i] == 'G' ||
						format[i] == 'o' ||
						format[i] == 'u' ||
						format[i] == 'x' ||
						format[i] == 'X' )
					{
						break;
					}
					else
					{
						fformat[pos] = format[i];
						i++;
						pos++;
					}
				}
				fformat[pos] = format[i];
				i++;
				fformat[pos + 1] = '\0';
				if(fformat[pos] == 'c')
				{
					::sprintf(buffer, fformat, (char)va_arg(argptr, int));
				}
				else if(fformat[pos] == 'd' || fformat[pos] == 'i' || fformat[pos] == 'x' || fformat[pos] == 'X')
				{
					int dec = va_arg(argptr, int);
					::sprintf(buffer, fformat, dec);
				}
				else if(fformat[pos] == 'f' || 
					fformat[pos] == 'e' || 
					fformat[pos] == 'E' ||
					fformat[pos] == 'g' || 
					fformat[pos] == 'G')
				{
					double dbl = va_arg(argptr, double);
					if(fformat[pos - 1] == 'l')
					{
						::sprintf(buffer, fformat, dbl);
					}
					else
					{
						::sprintf(buffer, fformat, (float)dbl);
					}
				}
				else
				{
					::sprintf(buffer, fformat, va_arg(argptr, int));
				}

				append(buffer);
			}
		}
	}

	return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends a formatted string to the end of the current string.  Usage
///   is similar to the set function.
///
///   The format and usage used is identical to printf.
///
///   Example:  set("My name is %s, and I am %d\n", "Daniel Barber", 26);
///   Result:  String will append "My name is Daniel Barber, and I am 26\n"
///
///   \return The number of bytes encoded in the string. 
///
////////////////////////////////////////////////////////////////////////////////
String &String::appendf(const char *format, ...)
{
    va_list argptr;
    int nump = 0;
    char buffer[128];
    char fformat[16];
    String append;
    
    if(format == NULL)
        return *this;

    clear();
    nump = (int)(occurrences("%", format));
    va_start(argptr, format);
    for(unsigned long i = 0; format[i] != '\0' || format[i] != 0; )
    {
        if(format[i] != '%')
        {
            append.append(format[i]);
            i++;
        }
        else
        {
            i++;
            if(format[i] == 's' || format[i] == 'S')
            {
                *this += va_arg(argptr, char *);
                i++;
            }
            else
            {
                
                int pos = 0;
                fformat[pos] = '%';
                pos++;
                while(pos < 16)
                {
                    if(format[i] == 'c' ||
                       format[i] == 'd' ||
                       format[i] == 'i' ||
                       format[i] == 'e' ||
                       format[i] == 'E' ||
                       format[i] == 'f' ||
                       format[i] == 'g' ||
                       format[i] == 'G' ||
                       format[i] == 'o' ||
                       format[i] == 'u' ||
                       format[i] == 'x' ||
                       format[i] == 'X' )
                    {
                        break;
                    }
                    else
                    {
                        fformat[pos] = format[i];
                        i++;
                        pos++;
                    }
                }
                fformat[pos] = format[i];
                i++;
                fformat[pos + 1] = '\0';
                if(fformat[pos] == 'c')
                {
                    sprintf(buffer, fformat, (char)va_arg(argptr, int));
                }
                else if(fformat[pos] == 'd' || fformat[pos] == 'i' || fformat[pos] == 'x' || fformat[pos] == 'X')
                {
                    int dec = va_arg(argptr, int);
                    sprintf(buffer, fformat, dec);
                }
                else if(fformat[pos] == 'f' || 
                        fformat[pos] == 'e' || 
                        fformat[pos] == 'E' ||
                        fformat[pos] == 'g' || 
                        fformat[pos] == 'G')
                {
                    double dbl = va_arg(argptr, double);
                    if(fformat[pos - 1] == 'l')
                    {
                        sprintf(buffer, fformat, dbl);
                    }
                    else
                    {
                        sprintf(buffer, fformat, (float)dbl);
                    }
                }
                else
                {
                    sprintf(buffer, fformat, va_arg(argptr, int));
                }

                append.append(buffer);
            }
        }
    }
    
    va_end(argptr);

    *this += append;

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return Number of bytes in the string, excluding the end of string character.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::length() const
{
    return mLength;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return Number of bytes of memory pre-allocated for the string to take up.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::reserved() const
{
    return mReserved;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \return Number of bytes of memory pre-allocated for the string to take up.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::capacity() const
{
    return mReserved;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Inserts a string at the position.
///
///   \param pos The position in the string to insert.
///   \param str The string to insert.
///
///   \return True on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////
String & String::insert(const unsigned long pos, const char *str)
{
    String temp;
    
    if(pos >= mLength || !str)
        return *this;

    char *ptr = mString;
    for(unsigned long i = 0; i < pos; i++)
    {
        temp.append(*ptr);
        ptr++;
    }
    temp.append(str);
    temp.append(ptr);

    swap(*this, temp);

    return *this;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Inserts a string at the position.
///
///   \param pos The position in the string to insert.
///   \param str The string to insert.
///
///   \return True on success, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////
String & String::insert(const unsigned long pos, const String &str)
{
    String temp;
    
    if(pos >= mLength)
        return *this;

    char *ptr = mString;
    for(unsigned long i = 0; i < pos; i++)
    {
        temp.append(*ptr);
        ptr++;
    }
    temp.append(str);
    temp.append(ptr);

    swap(*this, temp);

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Compares the two strings to see if they are equal.
///
///   \param left The first string to check.
///   \param right The second string to check.
///   \param nocase If true than not case-sensitive.
///
///   \return True if the strings are equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::compare(const String &left, const String &right, const bool nocase)
{
    if(!nocase)
        return left == right;

    if(left.mLength != right.mLength)
        return false;
    else if(left.mLength == 0)
        return true;

    char *lPtr, *rPtr;
    lPtr = left.c_str();
    rPtr = right.c_str();
    for(unsigned long i = 0; i < left.mLength; i++)
    {
        if(tolower(*lPtr) != tolower(*rPtr))
            return false;
    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Replaces the character with another character throughout the string.
///
///   This function is case sensitive.
///
///   \param oc The character to replace.
///   \param nc The caracter to substitute.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::replace(const char oc, const char nc)
{
    unsigned long total = 0;
    if(mLength == 0)
        return *this;

    char *ptr = mString;
    char *sub = ptr;
    unsigned long totalBytes = 0;
    while((sub = strchr(ptr, oc)) != NULL)
    {
        total++;
        *sub = nc;
        ptr = sub + 1;
        totalBytes++;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Erases the character from the string.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::erase(const char chrtr)
{
    char buff[2];
    buff[0] = chrtr;
    buff[1] = 0;
    return replace(buff, "");
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Erases data at a starting index for a specified number of bytes.
///
///   \param start The starting index to erase from.
///   \param num The number of bytes to erase.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::erase(const unsigned long start, const unsigned long num)
{
	int finalnum = start + num;

    if(start >= mLength || num > mLength)
        return *this;

	if(start + num >= mLength)
		finalnum = mLength - start;

	mLength -= num;
	memmove(mString+start, mString+start+num, mLength-start);
	mString[mLength] = 0;
   
    return *this;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Erases the substring from the string.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::erase(const char *substr)
{
    String final;

    unsigned long total = 0;
    if(mLength == 0 || !substr)
        return *this;

    char *ptr = mString;
    char *prevPtr = mString;
    char *sub = ptr;
    unsigned long subSize = (unsigned long)strlen(substr);
    unsigned long totalBytes = 0;
    while((sub = strstr(ptr, substr)) != NULL)
    {
        total++;
        //  Add up to the location of the substring
        while(prevPtr != sub)
        {
            final.append(*prevPtr);
            prevPtr++;
        }
        //  Skip past the substring
        ptr = sub + subSize;
        //  Set the previous to the current position after
        //  the substring found
        prevPtr = ptr;
        //  Make sure we do not go out of memory ranges
        totalBytes += subSize;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    final += ptr;

    swap(*this, final);

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Erases up to and including the character in the string.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::eraseUpTo(const char c, const unsigned long start)
{
    long epos;
    char str[2];
    str[0] = c;
    str[1] = '\0';
    if((epos = this->findLast(str, start)) >= 0)
    {
        this->erase(start, epos + 1);
    }
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Replaces the substring with another substring throughout the string.
///
///   This function is case sensitive.
///
///   \param substr The substring to replace.
///   \param repstr The replacement string.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::replace(const char *substr, const char *repstr)
{
    String final;

    unsigned long total = 0;
    if(mLength == 0 || !substr || !repstr)
        return *this;

    char *ptr = mString;
    char *prevPtr = mString;
    char *sub = ptr;
    unsigned long subSize = (unsigned long)strlen(substr);
    unsigned long totalBytes = 0;
    while((sub = strstr(ptr, substr)) != NULL)
    {
        total++;
        //  Add up to the location of the substring
        while(prevPtr != sub)
        {
            final.append(*prevPtr);
            prevPtr++;
        }
        //  Add the replacement string to final
        final.append(repstr);
        //  Skip past the substring
        ptr = sub + subSize;
        //  Set the previous to the current position after
        //  the substring found
        prevPtr = ptr;
        //  Make sure we do not go out of memory ranges
        totalBytes += subSize;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    final += ptr;

    swap(*this, final);

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Replaces the substring with another substring throughout the string.
///
///   This function is case sensitive.
///
///   \param substr The substring to replace.
///   \param repstr The replacement string.
///
///   \return The new string.
///
////////////////////////////////////////////////////////////////////////////////
String & String::replace(const String &substr, const String &repstr)
{
    return replace(substr.mString, repstr.mString);
}
    
////////////////////////////////////////////////////////////////////////////////
///
///   \brief Swaps the contents of one string with another.  Good for avoiding
///   additional copying of memory when not needed.
///
///   \param left One of the strings to swap with.
///   \param right The other string to swap with.
///
////////////////////////////////////////////////////////////////////////////////
void String::swap(String &left, String &right)
{
    char *tmp;
    unsigned long stmp;

    tmp = left.mString;
    left.mString = right.mString;
    right.mString = tmp;

    stmp = left.mLength;
    left.mLength = right.mLength;
    right.mLength = stmp;

    stmp = left.mReserved;
    left.mReserved = right.mReserved;
    right.mReserved = stmp;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out how many times the substring occurs within the string.
///
///   This function is case sensitive.
///
///   \param substr The substring to search for.
///
///   \return Number of times the string occurs withing the string.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::occurrences(const char *substr) const
{
    unsigned long total = 0;
    if(mLength == 0 || !substr)
        return 0;

    char *ptr = mString;
    char *sub = ptr;
    unsigned long subSize = (unsigned long)strlen(substr);
    unsigned long totalBytes = 0;
    while((sub = strstr(ptr, substr)) != NULL)
    {
        total++;
        ptr = sub + subSize;
        totalBytes += subSize;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    return total;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out how many times the substring occurs within the string.
///
///   This function is case sensitive.
///
///   \param substr The substring to search for.
///
///   \return Number of times the string occurs withing the string.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::occurrences(const String &substr) const
{
    unsigned long total = 0;
    if(mLength == 0 || substr.mLength == 0)
        return 0;

    char *ptr = mString;
    char *sub = ptr;
    unsigned long subSize = substr.mLength;
    unsigned long totalBytes = 0;
    while((sub = strstr(ptr, substr.mString)) != NULL)
    {
        total++;
        ptr = sub + subSize;
        totalBytes += subSize;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    return total;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out how many times the substring occurs within the string.
///
///   This function is case sensitive.
///
///   \param substr The substring to search for.
///   \param str The string to search.
///
///   \return Number of times the string occurs withing the string.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::occurrences(const char *substr, const char *str) const
{
    unsigned long total = 0;
    assert(substr && str);
    char *ptr = (char *)(str + 0); //  Ha ha, fooled you compiler!
    char *sub = ptr;
    unsigned long subSize = (unsigned long)strlen(substr);
    unsigned long strSize = (unsigned long)strlen(str);
    unsigned long totalBytes = 0;

    if(subSize > strSize)
        return 0;

    while((sub = strstr(ptr, substr)) != NULL)
    {
        total++;
        ptr = sub + subSize;
        totalBytes += subSize;
        if(ptr == '\0' || totalBytes > strSize)
            break;
    }

    return total;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out how many times the character occurs within the string.
///
///   This function is case sensitive.
///
///   \param chtr The character to search for.
///
///   \return Number of times the character occurs withing the string.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::occurrences(const char chtr) const
{
    unsigned long total = 0;
    if(mLength == 0)
        return 0;

    char *ptr = mString;
    char *sub = ptr;
    unsigned long totalBytes = 0;
    while((sub = strchr(ptr, chtr)) != NULL)
    {
        total++;
        ptr = sub + 1;
        totalBytes++;
        if(ptr == '\0' || totalBytes > mLength)
            break;
    }

    return total;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Prints the contents of the string to a console.
///
////////////////////////////////////////////////////////////////////////////////
void String::print()
{
    printf("%s", mString);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to all uppercase letters.
///
////////////////////////////////////////////////////////////////////////////////
String &String::toUpper()
{
    char *ptr = mString;
    for(unsigned long i = 0; i < mLength; i++, ptr++)
    {
        *ptr = toupper(*ptr);
    }
	return *this;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts the string to all lowercase letters.
///
////////////////////////////////////////////////////////////////////////////////
String &String::toLower()
{
    char *ptr = mString;
    for(unsigned long i = 0; i < mLength; i++, ptr++)
    {
        *ptr = tolower(*ptr);
    }
	return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears the contents of the string, but does not erase or deallocate
///   memory.  Use destroy for deletion.
///
////////////////////////////////////////////////////////////////////////////////
void String::clear()
{
    mLength = 0;
    if(mString)
        *mString = '\0';
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears the contents of the string, but does not erase or deallocate
///   memory.  Use destroy for deletion.
///
////////////////////////////////////////////////////////////////////////////////
void String::erase()
{
    clear();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the character to the end of the string.
///
///   \param c The character to append.
///
///   \return The new string size.
///
////////////////////////////////////////////////////////////////////////////////
String &String::append(const char c)
{
    if(mReserved == 0 || mString == NULL)
        reserve(STRING_MIN_RESERVED);
    else if(mLength >= mReserved - 1)
    {
        reserve(mReserved*2);
    }
    
    mString[mLength] = c;
    mLength++;
    mString[mLength] = '\0';

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the string to the end of the string.
///
///   \param rhs The string to append.
///
///   \return The new string size.
///
////////////////////////////////////////////////////////////////////////////////
String &String::append(const String &rhs)
{
    *this += rhs;

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the string to the end of the string.
///
///   \param str The string to append.
///
///   \return The new string size.
///
////////////////////////////////////////////////////////////////////////////////
String &String::append(const char *str)
{
    *this += str;

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes the string (all allocated memory).
///
////////////////////////////////////////////////////////////////////////////////
void String::destroy()
{
    if(mString)
    {
        delete[] mString;
        mString = NULL;
    }

    mReserved = mLength = 0;
    mString = new char[1];
    *mString = '\0';
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds out how much to increase the reserved amount of memory to
///   contain the size required, and resizes to it if needed.  All memory
///   is reserved in powers of 2, so that they line up nicely in memory.
///
///   \param size The size to expand to fit.
///
////////////////////////////////////////////////////////////////////////////////
void String::expand(const unsigned long size)
{
    unsigned long newReserved = mReserved;
    if(newReserved == 0)
    {
        newReserved = STRING_MIN_RESERVED;
    }
    while(newReserved < size + 1)
    {
        //  Reserve as a power of two
        newReserved *= 2;
    }
    reserve(newReserved);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Reduces the size of memory by a factor of two, but in a way that
///   the size required will still fit.  This helps prevent the sucking up
///   of a lot of memory by the String class throughout its use.
///
///   \param size The size to reduce to fit.
///
////////////////////////////////////////////////////////////////////////////////
void String::retract(const unsigned long size)
{
    unsigned long newReserved = mReserved;
    //  Reduce in size by a factor of two
    while(newReserved > size + 1)
    {
        newReserved /= 2;
        if(newReserved < STRING_MIN_RESERVED)
            break;
    }
    if(newReserved < STRING_MIN_RESERVED)
        reserve(STRING_MIN_RESERVED);
    else
        reserve(newReserved*2);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the reserved memory to size.
///
///   \param size The size to resize to.
///
////////////////////////////////////////////////////////////////////////////////
void String::reserve(const unsigned long size)
{
    if(size == 0)
    {
        destroy();
        return;
    }

    //  Don't resize if already there
    if(size == mReserved)
        return;

    unsigned int rsize = (size > 2) ? size : 4;
    char *ptr = new char[rsize];
    assert(ptr);

    //  If we are still large enough
    //  copy the previous string information 
    if(size > mLength && mLength > 0 && mReserved > 0)
    {
        //  Copy the old data, but don't change the 
        //  string length
        strcpy(ptr, mString);
    }
    else 
    {
        //  Must lose the old data so clear the string length
        mLength = 0;
    }
    if(mString)
    {
        delete[] mString;
        mString = NULL;
    }
    mString = ptr;
    mReserved = rsize;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Resizes the reserved memory to a specific size, and sets the values
///   of the characters to the parameter passed.
///
///   \param size The size to resize to.
///   \param c The value to give all data in the buffer.
///
////////////////////////////////////////////////////////////////////////////////
void String::resize(const unsigned long size, const char c)
{
    if(size == 0)
        destroy();

    if(size != mLength)
    {
        unsigned long newReserved = mReserved;
        if(size > mLength)
        {
            if(newReserved == 0)
                newReserved = STRING_MIN_RESERVED;

            while(newReserved < size + 1)
                newReserved *= 2;
        }
        else if(size > mLength)
        {
            //  Reduce in size by a factor of two
            while(newReserved > size + 1)
            {
                newReserved /= 2;
                if(newReserved < STRING_MIN_RESERVED)
                    break;
            }
            if(newReserved < STRING_MIN_RESERVED)
                newReserved = STRING_MIN_RESERVED;
            else
                newReserved *= 2;
        }

        char *newBuff = new char[newReserved];
        assert(newBuff);

        if(mString)
        {
            delete[] mString;
            mString = NULL;
        }
        mString = newBuff;
        mLength = size;
        mReserved = newReserved;
    }

    memset(mString, c, sizeof(char)*mLength);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Trims off data after and including index pos.
///
///   \param pos Position to cut off at.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::trim(const unsigned long pos)
{
    if(pos >= mLength || mLength == 0 || pos > mReserved)
        return mLength;
    else
    {
        mString[pos] = '\0';
        mLength = pos == 0 ? 0 : pos - 1;
        return mLength;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// 
///   \brief Trims whitespace off the beginning and end of the string
///
///   \return A string with no beginning/ending whitespace
///
////////////////////////////////////////////////////////////////////////////////
String &String::trim()
{
	unsigned long i;
	int pos = 0;
	char *ptr;
	
	// Look for whitespace at the beginning of the string
	ptr = mString;
	for(i = 0; i < mLength; i++, ptr++)
	{
		if (!isspace(*ptr))
			break;
	}

	// Shift the string if necessary (yeah, not terribly efficient, but then again...)
	if (i)
	{
		memmove(mString, ptr, mLength - i);
		mLength -= i;
		mString[mLength] = 0;
	}

	if (mLength > 0)
	{
		// Look for whitespace at the end of the string
		ptr = mString + mLength - 1;
		for(i = mLength - 1; i >= 0; i--, ptr--)
		{
			if (!isspace(*ptr))
				break;
		}

		// Truncate the string if any trailing whitespace is found
		if (i < mLength - 1)
		{
			mLength = i + 1;
			mString[mLength] = 0; // Null terminate
		}
	}



	return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator=(const char *string)
{
    unsigned long size;
    if(string == NULL)
    {
        destroy();
        return *this;
    }

    size = (unsigned long)strlen(string);

    if(size == 0)
    {
        destroy();
        return *this;
    }
    if(mReserved < size)
    {
        expand(size);
    }
    if(mReserved > size*10)
    {
        retract(size);
    }

    mLength = (unsigned long)size;
    memcpy(mString, string, sizeof(char)*mLength);
    mString[mLength] = '\0';  //  Ensure NULL character at the end

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator=(const char c)
{
    clear();
    //  Make sure we have allocated memory
    if(mReserved || mString == NULL)
    {
        reserve(STRING_MIN_RESERVED);
    }
    mLength = 1;
    mString[0] = c;
    mString[1] = '\0';

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns a pointer to the character array that is NULL terminated.
///
////////////////////////////////////////////////////////////////////////////////
char *String::ascii() const
{
    return c_str();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns a pointer to the character array that is NULL terminated.
///
////////////////////////////////////////////////////////////////////////////////
char *String::c_str() const
{
    static char *emptyString = "";
    return mString ? mString : emptyString;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns the last character in the string.
///
////////////////////////////////////////////////////////////////////////////////
char String::end() const
{
    if(mLength > 0 && mString)
    {
        return mString[mLength - 1];
    }
    else
        return '\0';
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the strings are equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator==(const String &rhs) const
{
    if(mLength != rhs.mLength)
        return false;
    if(mLength == 0 && rhs.mLength == 0)
        return true;
    
    if(strcmp(mString, rhs.mString) == 0)
        return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the strings are not equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator!=(const char *rhs) const
{
    return !this->operator ==(rhs);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the strings are not equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator!=(const String &rhs) const
{
    return !this->operator ==(rhs);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return True if the strings are equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator==(const char *rhs) const
{
    if(mString && strcmp(mString, rhs) == 0)
        return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Compares strings for sorting alphabetically.
///
///   \param rhs The string to compare against.
///
///   \return True if the first non-matched character in current string is less
///           than the character in the second.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator<(const String &rhs) const
{
    return strcmp(mString, rhs.mString) < 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Compares strings for sorting alphabetically.
///
///   \param rhs The string to compare against.
///
///   \return True if the first non-matched character in current string is larger
///           than the character in the second.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator>(const String &rhs) const
{
    return strcmp(mString, rhs.mString) > 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Compares strings for sorting alphabetically.
///
///   \param rhs The string to compare against.
///
///   \return True if the first non-matched character in current string is larger
///           than the character in the second, or if string is equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator>=(const String &rhs) const
{
    return strcmp(mString, rhs.mString) >= 0;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Compares strings for sorting alphabetically.
///
///   \param rhs The string to compare against.
///
///   \return True if the first non-matched character in current string is less
///           than the character in the second, or if string is equal.
///
////////////////////////////////////////////////////////////////////////////////
bool String::operator<=(const String &rhs) const
{
    return strcmp(mString, rhs.mString) <= 0;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator=(const String &rhs)
{
    if(&rhs != this)
    {
        if(rhs.mReserved == 0 || !rhs.mString)
        {
            mLength = 0;
        }
        else
        {
            if(mReserved < rhs.mReserved)
            {
                reserve(rhs.mReserved);
            }

            if(rhs.mLength > 0)
                memcpy(mString, rhs.mString, sizeof(char)*(rhs.mLength + 1));

            mLength = rhs.mLength;
        }
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the string to the end of the current string.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator+=(const char *string)
{
    unsigned long size;
    assert(string);
    size = (unsigned long)strlen(string);
    
    if(size == 0)
        return *this;

    if(((mLength + size) >= mReserved) || mReserved == 0)
    {
        expand(mLength + size);
    }
    
    memcpy(&mString[mLength], string, sizeof(char)*size);
    mLength += size;
    mString[mLength] = '\0';

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the string to the end of the current string.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator+=(const String &rhs)
{   
    if(rhs.mLength == 0 || rhs.mString == NULL)
        return *this;

    if(((mLength + rhs.mLength) >= mReserved) || mReserved == 0)
    {
        expand(mLength + rhs.mLength);
    }
    
    memcpy(&mString[mLength], rhs.mString, sizeof(char)*rhs.mLength);
    mLength += rhs.mLength;
    mString[mLength] = '\0';

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Appends the character to the end of the current string.
///
////////////////////////////////////////////////////////////////////////////////
String &String::operator+=(const char c)
{   
    this->append(c);

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds to strings together and returns the result.
///
////////////////////////////////////////////////////////////////////////////////
String String::operator+(const String &rhs)
{
    String t(*this);
    t += rhs;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds to strings together and returns the result.
///
////////////////////////////////////////////////////////////////////////////////
String String::operator+(const char *string)
{
    String t(*this);
    t += string;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds to strings together and returns the result.
///
////////////////////////////////////////////////////////////////////////////////
String String::operator+(const char c)
{
    String t(*this);
    t += c;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for casting.  
///
///   \return Pointer to character array.
///
////////////////////////////////////////////////////////////////////////////////
String::operator char *() const
{
    return mString;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for casting.  
///
///   \return Pointer to character array.
///
////////////////////////////////////////////////////////////////////////////////
String::operator const char *()
{
    if (!mString) return "";
    return mString;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Allows for indexing within the character array.  Pretty slow way to
///   do it when you could just get a copy of the pointer to the data (HINT!).
///
///   \param i The index within the string to get.
///
////////////////////////////////////////////////////////////////////////////////
char &String::at(const unsigned long i)
{
    assert(i < mLength);
    return mString[i];
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Allows for indexing within the character array.  Pretty slow way to
///   do it when you could just get a copy of the pointer to the data (HINT!).
///
////////////////////////////////////////////////////////////////////////////////
char &String::operator[](const unsigned long i)
{
    assert(i < mLength);
    return mString[i];
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Allows for indexing within the character array.  Pretty slow way to
///   do it when you could just get a copy of the pointer to the data (HINT!).
///
////////////////////////////////////////////////////////////////////////////////
char &String::charAt(const unsigned long i)
{
    assert(i < mLength);
    return mString[i];
}

////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of the string.  Same as lenght() function.
///
////////////////////////////////////////////////////////////////////////////////
unsigned long String::size() const { return mLength; }

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds the position of a string starting at a point within the
///   string.
///
///   \param str The substring to find.
///   \param pos Index to start in the array.
///
///   \return Index of the substring.
///
////////////////////////////////////////////////////////////////////////////////
long String::find(const String &str, const unsigned long pos) const
{
    if(pos >= mLength || !mString)
        return -1;

    char *ptr = mString + pos;
    char *delm = str.mString;
    char *final = strstr(ptr, delm);
    long fpos = pos;
    if(final == 0)
        return -1;

    while(ptr != final)
    {
        fpos++;
        ptr++;
    }

    return fpos;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds the last position of a string starting at a point within the
///   string.
///
///   \param str The substring to find.
///   \param pos Index to start in the array.
///
///   \return Index of the substring.
///
////////////////////////////////////////////////////////////////////////////////
long String::findLast(const char *str, const unsigned long pos) const
{
    if(pos >= mLength || !mString || mLength == 0|| !str)
        return -1;

    unsigned long length = (unsigned long)strlen(str);
    if(length <= 0 || length >= mLength)
        return -1;

    long end = mLength - 1 - length;
    char *ptr = &mString[end];

    for(long i = end; i >= (long)pos; i --)
    {
        char backup;
        backup = *(ptr + length);
        *(ptr + length) = '\0';
        if(strcmp(ptr, str) == 0)
        {
            *(ptr + length) = backup;
            return i;
        }
        *(ptr + length) = backup;
        ptr --;
    }

    return -1;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds the last position of a string starting at a point within the
///   string.
///
///   \param str The substring to find.
///   \param pos Index to start in the array.
///
///   \return Index of the substring.
///
////////////////////////////////////////////////////////////////////////////////
long String::findLast(const String &str, const unsigned long pos) const
{
    if(pos >= mLength || !mString || mLength == 0)
        return -1;

    unsigned long length = str.length();
    if(length <= 0 || length >= mLength)
        return -1;

    long end = mLength - 1 - length;
    char *ptr = &mString[end];

    for(long i = end; i >= (long)pos; i --)
    {
        char backup;
        backup = *(ptr + length);
        *(ptr + length) = '\0';
        if(strcmp(ptr, str.mString) == 0)
        {
            *(ptr + length) = backup;
            return i;
        }
        *(ptr + length) = backup;
        ptr --;
    }

    return -1;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Finds the position of a string starting at a point within the
///   string.
///
///   \param str The substring to find.
///   \param pos Index to start in the array.
///
///   \return Index of the substring, -1 if not found.
///
////////////////////////////////////////////////////////////////////////////////
long String::find(const char *str, const unsigned long pos) const
{
    if(pos >= mLength || !mString || !str)
        return -1;

    char *ptr = mString + pos;
    char *final = strstr(ptr, str);
    long fpos = pos;
    if(final == 0)
        return -1;

    while(ptr != final)
    {
        fpos++;
        ptr++;
    }

    return fpos;
}


////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns a substring at the starting index of a certain length.
///
///   \param start Starting index to start looking.
///   \param length Length of the substring to get. If larger than the
///   length of the string, than this value is truncated.
///
///   \return Substring of the current string of size length, or up to the length
///   of the string.
///
////////////////////////////////////////////////////////////////////////////////
String String::substr(const unsigned long start, const unsigned long length) const
{
    String sub;
    unsigned long size;

    if(start > mLength || mLength == 0 || !mString)
        return sub;

    if((start + length) > mLength)
        size = mLength - start;
    else
        size = length;

    sub.expand(size);
    memcpy(sub.mString, &mString[start], sizeof(char)*size);
    sub.mLength = size;
    sub.mString[size] = '\0';
    
    return sub;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns a substring at the starting index to end of the string.
///
///   \param start Starting index to start looking.
///
///   \return Substring starting from the start index.
///
////////////////////////////////////////////////////////////////////////////////
String String::substr(const unsigned long start) const
{
    String sub;
    unsigned long size;

    if(start > mLength || mLength == 0 || !mString)
        return sub;

    size = mLength - start;

    sub.expand(size);
    memcpy(sub.mString, &mString[start], sizeof(char)*size);
    sub.mLength = size;
    sub.mString[size] = '\0';
    
    return sub;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets a substring at the starting index to end of the string.
///
///   \param start Starting index to start looking.
///   \param length How long the substring is.  If longer than the actual string
///   than this value is truncated.
///   \param result Where to save the substring.
///
///   \return -1 if an error, otherwise size of substring.
///
////////////////////////////////////////////////////////////////////////////////
int String::substr(const unsigned long start, const unsigned long length, String &result) const
{
    result.clear();
    unsigned long size;

    if(start > mLength || mLength == 0 || !mString)
        return - 1;

    if((start + length) > mLength)
        size = mLength - start;
    else
        size = length;

    if(result.mReserved >= 10*size)
        result.retract(size);
    else if(result.mReserved < size)
        result.expand(size);

    memcpy(result.mString, &mString[start], sizeof(char)*size);
    result.mLength = size;
    result.mString[size] = '\0';
    
    return size;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets a substring at the starting index to end of the string.
///
///   \param start Starting index to start looking.
///   \param result Where to save the substring.
///
///   \return -1 if an error, otherwise size of substring.
///
////////////////////////////////////////////////////////////////////////////////
int String::substr(const unsigned long start, String &result) const
{
    result.clear();
    unsigned long size;

    if(start > mLength || mLength == 0 || !mString)
        return - 1;

    size = mLength - start;

    if(result.mReserved >= 10*size)
        result.retract(size);
    else if(result.mReserved < size)
        result.expand(size);

    memcpy(result.mString, &mString[start], sizeof(char)*size);
    result.mLength = size;
    result.mString[size] = '\0';
    
    return size;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts a substring of data from the String between and including 
///   a start and end character (unless trimSides is set to true).  For example 
///   if you wanted to extract data between a set of quotes in a string, you could.
///
///   Example:  String str = "void setInt(int a)";
///             String params; str.extract('(', ')', params);
///   Result:
///             str == "void setInt" params == "(int a)"
/// 
///             or params == "int a" if trimSides == true
///
///   \param sC The start character sequence of the substring to find
///   \param eC The end character sequence of the substring to find
///   \param result Where to save the substring if found (cleared by default).
///   \param _erase Erase the found substring from the current string.
///   \param trimSides Whether or not to trim off the characters on either side
///
///   \return -1 if an error, otherwise size of substring.
///
////////////////////////////////////////////////////////////////////////////////
String &String::extract(const char sC, const char eC, String &result, bool _erase, bool trimSides)
{
    result.clear();

    if(!mString)
        return *this;;

    //  First lets try and find the starting character and ending characters
    char *sPtr, *ePtr, *ptr;

    //  Find them in the string
    if((sPtr = strchr(mString, sC)) == NULL)
        return *this;; //  Start character not in the string

    ptr = sPtr + 1;    //  Start after the first character
    //  Try find the ending character  within the string
    if(*ptr == '\0' || (ePtr = strchr(ptr, eC)) == NULL)
        return *this;;
    
    //  We found the sequence, lets add it to the result!
	if (!trimSides)
	{
		ptr = sPtr;
		while(ptr <= ePtr)
		{
			result.append(*ptr);
			ptr++;
		}
		sPtr++;
	}
	else
	{
		while(ptr < ePtr)
		{
			result.append(*ptr);
			ptr++;
		}
		ePtr++;
	}
    
    if(_erase)
    {
		this->erase((int)(sPtr - mString), (int)(ePtr - sPtr));
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// 
///   \brief Extracts the front part of a string based on the a character
///
///   Everything up until the extraction character will be transfered to the
///   result string, and, if _erase is set, will be removed from original string
///
///   \param c Character marking the extraction point
///   \param result Store the part up until the extraction character
///   \param _erase Erase the extracted string from the original?
///
///   \return The string (erased or un-erased, depending on the parameters)
///
////////////////////////////////////////////////////////////////////////////////
String & String::extract(const char c, String &result, bool _erase /*= true*/)
{
	int length;
	//  First lets try and find the starting character and ending characters
	char *ptr;

	result.clear();

	//  Find them in the string
	if((ptr = strchr(mString, c)) == NULL)
		return *this; //  Start character not in the string

	ptr++;            //  Start after the first character

	// Assign the result
	length = (int)(ptr - mString);
	result = this->substr(0, length - 1);

	if (_erase)
	{
		// Adjust string length
		mLength -= length ;
		memmove(mString, ptr, mLength);
		mString[mLength] = 0;
	}

	return *this;

}
//////////////////////////////////////////////////////////////////////////////
///
///   \brief Splits a single string into an array using a delimiter
///
///   For instance, you can use this to split CSV up in a spreadsheet
///   Run,Method,Accuracy,Etc,Etc
/// 
///   Adapted from http://www.codeproject.com/string/stringsplit.asp
///   (Note the original version was buggy and is hopefully fixed now)
///
///   \param input String to split
///   \param delimiter Substring to split the string with
///   \param results Vector to contain array of split substrings
///   \param includeEmpties Include empty strings in the array (default: true)
///   \return Number of delimiters found
///
//////////////////////////////////////////////////////////////////////////////
int String::split(const String& input, const char *delimiter, Array<String>& results, bool includeEmpties)
{
    long iPos = 0;
    long newPos = -1;
    assert(delimiter);
    long sizeS2 = (long)(strlen(delimiter));
    long isize = input.size();

	results.clear();

    if(!isize || !sizeS2)
    {
        return 0;
    }

    vector<unsigned long> positions;

    newPos = (int)input.find (delimiter, 0);

    if( newPos < 0 )
    { 
        return 0; 
    }

    int numFound = 0;

    while( newPos >= iPos )
    {
        numFound++;
        positions.push_back(newPos);
        iPos = newPos;
        newPos = (int)input.find (delimiter, iPos+sizeS2);
    }

    if( numFound == 0 )
    {
        return 0;
    }

    for( int i=0; i <= (int)positions.size(); ++i )
    {
        String s;
        if( i == 0 ) 
        { 
            input.substr( i, positions[i], s ); 
        }
        else
        {
            int offset = positions[i-1] + sizeS2;
            if( offset < isize )
            {
                if( i == (int)positions.size() )
                {
                    input.substr(offset, s);
                }
                else if( i > 0 )
                {
                    input.substr( positions[i-1] + sizeS2, 
                          positions[i] - positions[i-1] - sizeS2, s );
                }
            }
        }
        if( includeEmpties || ( s.size() > 0 ) )
        {
            results.push_back(s);
        }
    }
    
    return numFound;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Splits a single string into an array using a delimiter
///
///   For instance, you can use this to split CSV up in a spreadsheet
///   Run,Method,Accuracy,Etc,Etc
/// 
///   Adapted from http://www.codeproject.com/string/stringsplit.asp
///   (Note the original version was buggy and is hopefully fixed now)
///
///   \param input String to split
///   \param delimiter Substring to split the string with
///   \param results Vector to contain array of split substrings
///   \param includeEmpties Include empty strings in the array (default: true)
///   \return Number of delimiters found
///
//////////////////////////////////////////////////////////////////////////////
int String::split(const String& input, const String& delimiter, Array<String>& results, bool includeEmpties)
{
	return split(input, delimiter.c_str(), results, includeEmpties);
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Splits up the string into an array using a delimiter
///
///   For instance, you can use this to split CSV up in a spreadsheet
///   Run,Method,Accuracy,Etc,Etc
/// 
///   Adapted from http://www.codeproject.com/string/stringsplit.asp
///   (Note the original version was buggy and is hopefully fixed now)
///
///   \param delimiter Substring to split the string with
///   \param results Array containing the individual split up strings
///   \param includeEmpties Include empty strings in the array (default: true)
///   \return Number of delimeters found
///
//////////////////////////////////////////////////////////////////////////////
int String::split(const char *delimiter, Array<String> &results, bool includeEmpties /*= true*/) const
{
	return String::split(*this, delimiter, results, includeEmpties); 
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Splits up the string into an array using a delimiter
///
///   For instance, you can use this to split CSV up in a spreadsheet
///   Run,Method,Accuracy,Etc,Etc
/// 
///   Adapted from http://www.codeproject.com/string/stringsplit.asp
///   (Note the original version was buggy and is hopefully fixed now)
///
///   \param delimiter Substring to split the string with
///   \param results Array containing the individual split up strings
///   \param includeEmpties Include empty strings in the array (default: true)
///   \return Number of delimeters found
///
//////////////////////////////////////////////////////////////////////////////
int String::split(const String &delimiter, Array<String> &results, bool includeEmpties /*= true*/) const 
{
	return String::split(*this, delimiter, results, includeEmpties); 
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Makes a copy of the string to the buffer.
///
///   Automatically resizes memory pointer points to.
///
///   \param buff Where to save the copy to.
///
///   \return The size of buff.
///
//////////////////////////////////////////////////////////////////////////////
unsigned long String::copy(char *&buff)
{
    if(buff)
    {
        delete[] buff;
        buff = NULL;
    }
    
    if(mLength == 0)
    {
        buff = NULL;
        return 0;
    }

    buff = new char[mLength + 1];
    memcpy(buff, mString, sizeof(char)*mLength);
    buff[mLength] = '\0';

    return mLength;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Makes a copy of the string to the buffer.
///
///   \param buff Where to save the copy to.
///   \param size How large the buffer is.
///
///   \return The size of buff.
///
//////////////////////////////////////////////////////////////////////////////
unsigned long String::copy(char *buff, const unsigned long size)
{
    if(mLength == 0 || !mString)
    {
        if(buff && size > 0)
            buff[0] = '\0';
        return 0;
    }
    if(size >= mLength + 1)
    {
        memcpy(buff, mString, sizeof(char)*mLength);
        buff[mLength] = '\0';
        return mLength;
    }
    else
    {
        memcpy(buff, mString, sizeof(char)*size);
        return size;
    }
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return True if empty string.  Otherwise false.
///
//////////////////////////////////////////////////////////////////////////////
bool String::empty() const 
{
    return ((mString == 0) || (mLength == 0)) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return True if empty string.  Otherwise false.
///
//////////////////////////////////////////////////////////////////////////////
bool String::null() const 
{
    return ((mString == 0) || (mLength == 0)) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return True if empty string.  Otherwise false.
///
//////////////////////////////////////////////////////////////////////////////
bool String::isNull() const 
{
	return null();
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief To maintain compatibility with the standard STL string.
///
//////////////////////////////////////////////////////////////////////////////
String::operator string() const
{
    string a;

    a = mString;

    return a;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief To maintain compatibility with the standard STL string.
///
//////////////////////////////////////////////////////////////////////////////
String &String::operator=(const std::string &str)
{
    this->reserve((unsigned long)str.size());
    memcpy(mString, str.c_str(), sizeof(char)*mLength);
    return *this;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief To maintain compatibility with iostream like the STL string 
///   can.
///
///   \param os The outstream in used.
///   \param s The string to output with it.
///
///   \return The output stream.
///
//////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& os, const String& s)
{
    os << s.c_str();

    return os;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief To maintain compatibility with iostream like the STL string 
///   can.
///
///   \param is The input stream.
///   \param str The string to input to.
///
///   \return The input stream.
///
//////////////////////////////////////////////////////////////////////////////
istream& operator>>(istream& is, String& str)
{
#ifdef WIN32
    int w = is.width(0);
    if ( is.ipfx(0) ) 
    {
        streambuf *sb = is.rdbuf();
        str.erase();
        while ( true ) 
        {
            int ch = sb->sbumpc ();
            if ( ch == EOF ) 
            {
                is.setstate(ios::eofbit);
                break;
            }
            else if ( isspace(ch) ) 
            {
                sb->sungetc();
                break;
            }

            str += ch;
            if ( --w == 1 )
            break;
        }
    }

    // Non ANSI standard, only a placeholder for future expansion
    // is.isfx();
    if ( str.length() == 0 )
    is.setstate(ios::failbit);

#endif
    
    return is;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief To maintain compatibility with iostream like the STL string 
///   can.
///
///   The delimeter is read but not stored in the string.
///
///   \param is The input stream.
///   \param str The string to input to.
///   \param delimiter When to stop.
///
///   \return The input stream.
///
//////////////////////////////////////////////////////////////////////////////
istream& getline(istream& is, String& str, char delimiter)
{    
#ifdef WIN32
    int w = is.width(0);
    if ( is.ipfx(0) ) 
    {
        streambuf *sb = is.rdbuf();
        str.erase();
        while ( true ) 
        {
            int ch = sb->sbumpc ();
            if ( ch == EOF ) 
            {
                is.setstate(ios::eofbit);
                break;
            }
            else if ( ch == delimiter) 
            {
                sb->sungetc();
                break;
            }

            str += ch;
            if ( --w == 1 )
            break;
        }
    }

    // Non-standard ANSI code, only a placeholder for future expansion
    // is.isfx();
    if ( str.length() == 0 )
    is.setstate(ios::failbit);

#endif
    
    return is;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Reads in the number of bytes specified from a file pointer.
///
///   \param fp Open file pointer to read from using fread.
///   \param bytes Number of bytes to read in.
///
///   \return Total number of bytes read.
///
//////////////////////////////////////////////////////////////////////////////
unsigned long String::fread(FILE *fp, const unsigned int bytes)
{
    if(fp && bytes)
    {
        this->clear();
        this->reserve(bytes);
        unsigned int total = 0;
        char *raw = mString;
        while(total != bytes && !feof(fp))
        {
            unsigned int r = ((unsigned int)(::fread(raw, sizeof(unsigned char), bytes - total, fp)));
            total += r;
            raw += r;
        }
        mLength = total;
        return total;
    }

    return 0;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Writes to an open file pointer.
///
///   \param fp File pointer to write to using fwrite.
///
///   \return Number of bytes written.
///
//////////////////////////////////////////////////////////////////////////////
unsigned long String::fwrite(FILE *fp) const
{
    if(fp && mString && mLength)
    {
        unsigned int total = 0;
        char *raw = mString;
        while(total != mLength)
        {
            unsigned int w = ((unsigned int)(::fwrite(raw, sizeof(unsigned char), mLength - total, fp)));
            total += w;
            raw += w;
        }
        return total;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Returns a string containing a timestamp in string format
///
///   \param format See strftime for details of the format (default is %c)
///
///   \return String containing the specified timestamp
///
////////////////////////////////////////////////////////////////////////////////
String String::timestamp(const char *format /*= "%c"*/)
{
	String result;
	const int size = 256;
	char tmpStr[size];
	time_t rawtime;
	tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(tmpStr, size, format, timeinfo);
	result = tmpStr;

	return result;
}

/*  End of File */
