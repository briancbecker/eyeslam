/*==================================================================================

    Filename:  string.h

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
#ifndef _ZEBSTRING_H
#define _ZEBSTRING_H

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <assert.h>
#include <vector>
#include "array.h"

#define STRING_TYPE_NULL   0
#define STRING_TYPE_STRING 1
#define STRING_TYPE_INT    2
#define STRING_TYPE_DOUBLE 3


////////////////////////////////////////////////////////////////////////////////
///
///   \class String
///   \brief An alternative to the STL string which has additional functions for
///   string manipulation not available with the STL string.  
///
///   Additional functions liek simple creation of formatted strings similar to 
///   sprintf, upper and lower case conversions, replacement of characters
///   or substrings without needing to know the position, conversion to numbers,
///   type checking (is it a number, string, etc.), and functions for splitting
///   a string up into multiple strings based on a delimiter. 
///
///   No additional libraries are required.
///
////////////////////////////////////////////////////////////////////////////////
class String
{
    friend class Network;
    friend class Serial;
public:
    String();
    String(const String &rhs);
    String(const char *string);
    ~String();
    int type() const;
    int toInt() const;
    int substr(const unsigned long start, String &result) const;
    int substr(const unsigned long start, const unsigned long length, String &result) const;
    void print();
    void clear();
    void erase();
    String &toUpper();
    String &toLower();  
    void destroy();
    void resize(const unsigned long size, const char c);
    void reserve(const unsigned long size);
    bool toInt(int *inum) const;
    bool toLong(long *lnum) const;    
    bool toDouble(double *dnum) const;
    long toLong() const;
    char *ascii() const;
    char *c_str() const;
    char end() const;
    char &at(const unsigned long pos);
    long find(const String &str, const unsigned long pos = 0) const;
    long find(const char *str, const unsigned long pos = 0) const;
    long findLast(const char *str, const unsigned long pos = 0) const;
    long findLast(const String &str, const unsigned long pos = 0) const;
    bool empty() const;
	bool isEmpty() const { return empty(); }
    bool null() const;
	bool isNull() const;
    bool isNumber() const;
    bool isInteger() const;
    bool isDouble() const;    
    bool compare(const String &left, const String &right, const bool nocase = false);
    double toDouble() const;
    unsigned long copy(char *&buff);
    unsigned long copy(char *buff, const unsigned long size);
    unsigned long occurrences(const char chrtr) const;
    unsigned long occurrences(const char *substr) const;
    unsigned long occurrences(const String &substr) const;
    unsigned long occurrences(const char *substr, const char *str2) const;
    unsigned long length() const;
    unsigned long size() const;
    unsigned long reserved() const;   
    unsigned long capacity() const;
    unsigned long trim(const unsigned long pos);
	String &trim();
    unsigned long fread(FILE *fp, const unsigned int bytes);
    unsigned long fwrite(FILE *fp) const;
    static int split(const String &input, const char *delimeter, Array<String> &results, bool includeEmpties = true);
    static int split(const String &input, const String &delimeter, Array<String> &results, bool includeEmpties = true);
	int split(const char *delimeter, Array<String> &results, bool includeEmpties = true) const;
	int split(const String &delimiter, Array<String> &results, bool includeEmpties = true) const;
    static void swap(String &left, String &right);
    operator char *() const;
    operator const char *();
    bool operator==(const String &rhs) const;
    bool operator==(const char *rhs) const;
    bool operator!=(const String &rhs) const;
    bool operator!=(const char *rhs) const;
    bool operator<(const String &rhs) const;
    bool operator>(const String &rhs) const;
    bool operator>=(const String &rhs) const;
    bool operator<=(const String &rhs) const;
    char &operator[](const unsigned long i);
    char &charAt(const unsigned long i);
    String & erase(const char chrtr);
    String & erase(const char *substr);
    String & erase(const unsigned long start, const unsigned long num);
    String & eraseUpTo(const char c, const unsigned long start = 0);
    String & extract(const char sC, const char eC, String &result, bool _erase = true, bool trimSides = false);
	String & extract(const char c, String &result, bool _erase = true);
    String & replace(const char oc, const char nc);
    String & replace(const char *substr, const char *repstr);
    String & replace(const String &substr, const String &repstr);
    String & insert(const unsigned long pos, const char *str);
    String & insert(const unsigned long pos, const String &str);
    String & set(const char *format, ...);
    String & sprintf(const char *format, ...);
	String & vsprintf(const char *format, va_list argptr);
    String & append(const char c);                                       
    String & append(const char *str);
    String & append(const String &rhs);
    String & appendf(const char *format, ...);
    String substr(const unsigned long start) const;
    String substr(const unsigned long start, const unsigned long length) const;
    String &operator=(const char *string);
    String &operator=(const std::string &str);
    String &operator=(const char c);
    String &operator=(const String &rhs);
    String  operator+(const String &rhs);
    String  operator+(const char *string);
    String  operator+(const char c);
    String &operator+=(const String &rhs);
    String &operator+=(const char *string);
    String &operator+=(const char c);
    operator std::string() const;
	static String timestamp(const char *format = "%c");
private:
    void expand(const unsigned long size);
    void retract(const unsigned long size);
    char *mString;              ///<  Memory allocated for string
    unsigned long mLength;      ///<  Actual length of string (not including NULL character)
    unsigned long mReserved;    ///<  Amount of reserved memory for strings
};

std::ostream& operator<<(std::ostream& os, const String& s);
std::istream& operator>>(std::istream& is, String& s);
std::istream& getline(std::istream& is, String& s, char delimiter = '\n');

///  Read in a line of text from a file
inline unsigned int getLine(FILE *fp, String *s)
{
    if(fp && s)
    {
        char ch = 0;
        s->clear();
        while(!feof(fp))
        {
            ch = fgetc(fp);
            (*s) += ch;
            if(ch == '\n')
                break;
        }
        return s->length();
    }
    else
        return 0;
}


#endif   // _ZEBSTRING_H
