/*==================================================================================

    Filename:  fileio.h

    Copyright 2007 Brian C. Becker
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Description:
    -------------------------------------------------------------------------------
    Contains static functions for performing cross-platform file I/O operations
    such as creating and deleting a directory and loading in lists of file
    names from folders, etc.

    License
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

#ifndef _FILE_IO_H
#define _FILE_IO_H

#include <stdio.h>
#ifdef WIN32
/*
#ifdef UNICODE
#undef UNICODE
#endif
#ifdef _UNICODE
#undef _UNICODE
#endif
*/
//#include <WinSock2.h>
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>    // NON-STANDARD ANSI C library, but common
#include <fnmatch.h>
#define MAX_PATH 256
#endif

#include "zstring.h"
#include "array.h"



////////////////////////////////////////////////////////////////////////////////////
///
///   \class FileIO
///   \brief A static class that helps common file manipulations easier
///
///   Includes methods for testing whether a file exists, reading an entire file
///   into memory, breaking appart filenames and paths, getting the size of a file,
///   and obtaining a list of all files in a directory that meet a filename
///   pattern match
///
////////////////////////////////////////////////////////////////////////////////////
class FileIO
{
public:
    /// Create a directory
    static int createDir(const char *fname);
	 static int createDir(LPCWSTR fname);
    /// Delete a directory
    static int deleteDir(const char *fname);
    /// Delete a file
    static int deleteFile(const char *fname);
    /// Delete an array of files
    static int deleteFiles(const Array<String> &fnames);

    /// Set the current directory
    static int setCurDir(const char *dir);

    /// Rename a directory or file
    static int rename(const char *fnameOld, const char *fnameNew);

    /// Scan a directory with a file pattern (such as *.*) for a list of files
    static int scanDirForFiles(const String &filePattern, Array<String> &files, const String dir = "", const bool extractPath = false);
	static int scanDirForDirs(const String &filePattern, Array<String> &files, const String dir = "", const bool extractPath = false);
    
    /// Checks for the existance of a file
    static int fileExists(const char * filename);

    /// Extracts the path from a full filename
    static String extractPath(const String &filename);

    /// Extracts only the filename from a full filename + path
    static String extractFilename(const String &fullFilename);

    /// Reads an entire file into memory (returned memory is allocated using new)
    static String readFileContents(const char * filename);

    /// Returns the size of a file
    static int getFileSize(const char *filename);
private:
    static int extractPath(const char * filename, char * path);
    static int extractFilename(const char * fullFilename, char * filename);
};

#endif // _FILE_IO_H
