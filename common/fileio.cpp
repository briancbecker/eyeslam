/*==================================================================================

    Filename:  fileio.cpp

    Copyright 2007 Brian C. Becker
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Contains static functions for performing cross-platform file I/O operations
    such as creating and deleting a directory and loading in lists of file
    names from folders, etc.
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
#include "fileio.h"

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Scans an existing directory to get a list of files within.
///
///   \param dir The directory to scan.
///   \param filePattern The file pattern to search for (*.*)
///   \param files Where to store the file information.
///   \param extractPath If true, the path is extracted.
///
///   \return Number of files found.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::scanDirForFiles(const String &filePattern, Array<String> &files, const String dir, const bool extractPath)
{
    int result = 0;
    String listItem;
    String findPattern;
    String wholeDir;
    char c;

    files.clear();
   
    wholeDir = dir;

    if(dir.length() == 0)
    {
        findPattern = filePattern;
    }
    else
    {
        wholeDir = dir;
        c = dir[dir.length() - 1];
        if (dir.length() > 0 && c != '/' && c != '\\')
        {
            wholeDir += '/';
        }

        findPattern = wholeDir + filePattern;
    }

#if defined(WIN32)
    WIN32_FIND_DATA foundFiles;
    HANDLE hFind;
	
    //hFind = FindFirstFileA(findPattern.ascii(), &foundFiles);
	hFind = FindFirstFileA(findPattern.ascii(), reinterpret_cast<LPWIN32_FIND_DATAA>(&foundFiles));//..modifed by Yang..

    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if (!(foundFiles.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            {
                listItem.clear();

                //  If we are not extracing the path, keep it in the file name (string)
                if(!extractPath)
                    listItem = wholeDir;
				
				//listItem += foundFiles.cFileName;
                listItem += (const char*)foundFiles.cFileName; //modifed by Yang..

                files.push_back(listItem);
                result++;
            }
        } while (FindNextFile(hFind, &foundFiles));

        FindClose(hFind);
    }
    else
    {
        //result = -1;
    }
#else
    DIR *directory = opendir(dir.length() == 0 ? "." : dir);
    if (directory)
    {
        dirent *d = readdir(directory);
        while (d)
        {
            if (!fnmatch(filePattern, d->d_name, 0))
            {
                files.push_back(d->d_name);
                result++;
            }
            d = readdir(directory);
        }
        closedir(directory);
    }
    else
    {
        result = -1;
    }
#endif

    //printf("ending\n");

    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Scans an existing directory to get a list of files within.
///
///   \param dir The directory to scan.
///   \param filePattern The file pattern to search for (*.*)
///   \param files Where to store the file information.
///   \param extractPath If true, the path is extracted.
///
///   \return Number of files found.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::scanDirForDirs(const String &filePattern, Array<String> &files, const String dir, const bool extractPath)
{
	int result = 0;
	String listItem;
	String findPattern;
	String wholeDir;
	char c;

	files.clear();

	wholeDir = dir;

	if(dir.length() == 0)
	{
		findPattern = filePattern;
	}
	else
	{
		wholeDir = dir;
		c = dir[dir.length() - 1];
		if (dir.length() > 0 && c != '/' && c != '\\')
		{
			wholeDir += '/';
		}

		findPattern = wholeDir + filePattern;
	}

#if defined(WIN32)
	WIN32_FIND_DATA foundFiles;
	HANDLE hFind;

	//hFind = FindFirstFileA(findPattern.ascii(), &foundFiles);
	hFind = FindFirstFileA(findPattern.ascii(), reinterpret_cast<LPWIN32_FIND_DATAA>(&foundFiles)); //..modified by Yang..
	
	if (hFind != INVALID_HANDLE_VALUE)
	{
		do
		{
			if ((foundFiles.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
				listItem.clear();

				//  If we are not extracing the path, keep it in the file name (string)
				if(!extractPath)
					listItem = wholeDir;

				//listItem += foundFiles.cFileName;
				listItem += (const char*)foundFiles.cFileName; //..modifeid by Yang

				files.push_back(listItem);
				result++;
			}
		} while (FindNextFile(hFind, &foundFiles));

		FindClose(hFind);
	}
	else
	{
		//result = -1;
	}
#else
	DIR *directory = opendir(dir.length() == 0 ? "." : dir);
	if (directory)
	{
		dirent *d = readdir(directory);
		while (d)
		{
			if (!fnmatch(filePattern, d->d_name, 0))
			{
				files.push_back(d->d_name);
				result++;
			}
			d = readdir(directory);
		}
		closedir(directory);
	}
	else
	{
		result = -1;
	}
#endif

	//printf("ending\n");

	return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \param filename The file to check for.
///
///   \return True 1 if the file exists, otherwise 0.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::fileExists(const char * filename)
{
    int result = false;

#ifdef WIN32
    if (GetFileAttributesA(filename) != /*INVALID_FILE_ATTRIBUTES*/ -1)
    {
        result = true;
    }
#else
    FILE * file = fopen(filename, "rb");
    if (file)
    {
        fclose(file);
        result = true;
    }
#endif

    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts the path information from the filename.
///
///   \param filename The file to extract the path from.
///   \param path The path information.
///
///   \return 1 if the path was extracted, otherwise 0.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::extractPath(const char * filename, char * path)
{
    int length = (int)(strlen(filename));
    int i, found = -1;

    if (path)
    {
        strcpy(path, "");

        for (i = length - 1; i >= 0; i--)
        {
            if (found >= 0)
            {
                path[i] = filename[i];
            }
            else
            {
                if (filename[i] == '/' || filename[i] == '\\')
                {
                    path[i+1] = 0;
                    //path[i] = filename[i];
                    found = i;
                }
            }
        }
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts the path information from the filename.
///
///   \param filename The file to extract the path from.
///
///   \return The path of the filename
///
////////////////////////////////////////////////////////////////////////////////////
String FileIO::extractPath(const String &filename)
{
    String path = "";

    long length = filename.length();
    long i, found = 0;
    
    path = "";
    
    if (length > 0)
    {
        for(i = length - 1; i >= 0; i--)
        {
            if(filename[i] == '/' || filename[i] == '\\')
            {
                found = i;
                break;
            }
        }

        path = filename.substr(0, found);
    }

    return path;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts the filename from the full file name (removes path info.)
///
///   \param fullFilename The full filename with the path.
///   \param filename Filename without path information.
///
///   \return 1 if the path was extracted, otherwise 0.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::extractFilename(const char * fullFilename, char * filename)
{
    int length = (int)(strlen(fullFilename));
    int i, found = -1;

    if (filename)
    {
        for (i = length - 1; i >= 0; i--)
        {
            if (fullFilename[i] == '/' || fullFilename[i] == '\\')
            {
                found = i;
                break;
            }
        }

        if (found < 0)
        {
            strcpy(filename, fullFilename);
        }
        else
        {
            found++;
            for (i = found; i < length; i++)
            {
                filename[i - found] = fullFilename[i];
            }

            filename[i - found] = 0;
        }
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts the filename from the full file name (removes path info.)
///
///   \param fullFilename The full filename with the path.
///
///   \return Filename without the path information
///
////////////////////////////////////////////////////////////////////////////////////
String FileIO::extractFilename(const String &fullFilename)
{
    String filename = "";

    long length = fullFilename.length();
    int i, found = -1;

    if (length > 0)
    {
        for (i = length - 1; i >= 0; i--)
        {
            if (fullFilename[i] == '/' || fullFilename[i] == '\\')
            {
                found = i;
                break;
            }
        }

        if (found < 0)
        {
            strcpy(filename, fullFilename);
        }
        else
        {
            found++;
            filename = fullFilename.substr(found);
        }
    }

    return filename;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \param filename File to get size of.
///
///   \return Size of the file if it exists, otherwise -1.
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::getFileSize(const char *filename)
{
    int size = -1;

    FILE *fp = fopen(filename, "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        rewind(fp);
        size = size - ftell(fp);
    }

    return size;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Reads a whole file into memory.  Memory is allocated using new, and
///   a pointer to the newly allocated memory is returned.
///
///   \param filename File to read.
///
///   \return String object containing the contents of the entire file
///
////////////////////////////////////////////////////////////////////////////////////
String FileIO::readFileContents(const char *filename)
{
    String result = "";
    char *tmp = 0;
    FILE *fp;
    int size;

    fp = fopen(filename, "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        rewind(fp);
        size = size - ftell(fp);

        tmp = new char[size+1];
        tmp[size] = 0;

        fread(tmp, size, 1, fp);
        fclose(fp);
        result = tmp;
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates a directory
///
///   \param fname The name of the directory to create
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::createDir(const char *fname)
{
#ifdef WIN32
	return CreateDirectoryA(fname, 0);
    //return CreateDirectory((LPCWSTR)fname, 0);
#else
    // By default the permissions on the directory is 755
    return !mkdir(fname, 0755);
#endif
}

int FileIO::createDir(LPCWSTR fname)
{
#ifdef WIN32
	//return CreateDirectory(fname, 0);
    return CreateDirectoryW(fname, 0);
#else
    // By default the permissions on the directory is 755
    return !mkdir(fname, 0755);
#endif
}
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes a directory
///
///   \param fname The name of the directory to delete
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::deleteDir(const char *fname)
{
#ifdef WIN32
#ifndef UNICODE
    return RemoveDirectory(fname); //original..
#else 
	return RemoveDirectory((LPCWSTR)fname); //curretnly working for main..
#endif

	//return RemoveDirectoryW((LPCWSTR)fname);
	
#else
    return !rmdir(fname);
#endif
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes a file
///
///   \param fname The name of the file to delete
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::deleteFile(const char *fname)
{
    return !remove(fname);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Delete's an array of files
///
///   \param fnames An array of files to delete
///
///   \result Numbe of files sucessfully deleted
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::deleteFiles(const Array<String> &fnames)
{
    int result = 0;

    for (int i = 0; i < fnames.size(); i++)
    {
        result += deleteFile(fnames[i]);
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Renames a file or directory
///
///   \param fnameOld The original name of the file/directory
///   \param fnameNew The new desired name of the file/directory
///
///   \result True on success, false on failure
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::rename(const char *fnameOld, const char *fnameNew)
{
    return !::rename(fnameOld, fnameNew);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the current working directory
///
///   \param dir The directory to become the new current directory
///
///   \result True on success, false on failure
///
////////////////////////////////////////////////////////////////////////////////////
int FileIO::setCurDir(const char *dir)
{
#ifdef WIN32

#ifndef UNICODE
    return SetCurrentDirectory(dir); //original..
#else 
	return SetCurrentDirectory((LPCWSTR)dir); //curretnly working for main..
#endif
   
	
	//return SetCurrentDirectoryW((LPCWSTR)dir);


#else
    return !chdir(dir);
#endif
}

/* End of file */
