//=============================================================================
// Copyright © 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: PGRAviFile.h,v 1.20 2007/06/11 20:54:15 soowei Exp $
//=============================================================================
#ifndef __PGRAVIFILE_H__
#define __PGRAVIFILE_H__

//=============================================================================
// System Includes
//=============================================================================
//#define WIN32_LEAN_AND_MEAN
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

//#include <WinSock2.h>
#include "windows.h"

#pragma warning ( disable: 4201 ) // nonstandard extension used : nameless struct/union
#include <vfw.h>

//=============================================================================
// PGR Includes
//=============================================================================

//=============================================================================
// Project Includes
//=============================================================================

#define AVI_FILE_SPLIT_SIZE ( (__int64)( 1024 * 1024 * 1024 ) )


/**
 * A simple wrapper for the VFW .AVI file interface.
 *
 * @note Only supports 24 bit BGR, 32 bit BGRU, and 8 bit greyscale.
 *
 * @bug  8-bit .avis do not display properly for some reason.
 */
class PGRAviFile  
{
public:
   /** Default constructor. */
   PGRAviFile();

   /** Default destructor. */
   virtual ~PGRAviFile();

   /**
    * Open an .avi for writing. The size of the avi file will be limited to SPLIT_SIZE bytes.
    * The file is splited automatically.
    *
    * @param   iCols       Width, in pixels, of each frame.
    * @param   iRows       Hight, in pixels, of each frame.
    * @param   ibpp	       Bits per pixel -- 24 (BGR), 32 (BGRU). or 8 bit greyscale.
    * @param   iFramerate  Framerate that the .avi will play back in.
    */
   bool openSizeLimitedAVI( 
      const char* pszFilename, 
      int	  iCols, 
      int	  iRows, 
      int	  ibpp, 
      double  iFramerate,
	  const char *codec = 0);

   /** Get the the bytes written */
   long int bytesWritten();

   /**
    * Load a bitmap from a file and append it to the current open .avi.
    * Must be in the correct format.
    */
   bool appendBMP( const char* pszFilename );

   /**
    * Add a frame (in the specified format) to the open .avi
    */
   bool	appendFrame( const unsigned char* pBuffer, bool bInvert = true );

   /** Close the .avi file.  This is also done by the destructor. */
   bool	 close();

protected:

	/** 
	* Read the next frame from the avi stream.  File must have been opened for 
	* reading.
	* 
	* @return false if read error, or last frame, true on success.
	* @bug Need a better return value.
	*/
	bool readNextFrame( unsigned char* pBuffer, bool bInvert = true );
	/***
	* Enumerates all available codecs using iRows, iCols, and iBPP.
	*
	* Call this function twice:
	* 1) Set picinfo as NULL and use the return value to setup
	*    an array of ICINFO structures.
	* 2) Pass in the array and it will be filled accordingly.
	*
	* @param   iCols       Width, in pixels, of each frame.
	* @param   iRows       Hight, in pixels, of each frame.
	* @param   iBPP	   Bits per pixel -- 24 (BGR), 32 (BGRU). or 8 bit greyscale.
	* @param   picinfo     Array of compressor info structures.
	* @param   iNumICInfo  Number of compressor info structures passed in.
	*
	* @return  The number of compressors available.
	*/
	int  enumerateCompressors( int iRows, int iCols, int iBPP, 
		ICINFO* picinfo, int iNumICInfo );

	/*
	* Sets the compressor type.  Call this before opening an AVI.
	*
	* @param   compvars  Compressor options.
	*/
	void setCompressor( COMPVARS* compvars );

	/*
	* Converts the error to a string a posts a message.
	*
	* @param   hrErr      The resulting error.
	*/
	void errorToString( HRESULT hrErr );

	/**
	* Open an AVI for writing.
	*
	* @param   iCols       Width, in pixels, of each frame.
	* @param   iRows       Hight, in pixels, of each frame.
	* @param   ibpp	   Bits per pixel -- 24 (BGR), 32 (BGRU). or 8 bit greyscale.
	* @param   dFramerate  Framerate that the .avi will play back in.
	*/
	bool	 open( 
		const char* pszFilename, 
		int	  iCols, 
		int	  iRows, 
		int	  ibpp, 
		double	  dFramerate);

	/**
	* Open an AVI for writing.  Deprecated.
	*
	* @param   iCols       Width, in pixels, of each frame.
	* @param   iRows       Hight, in pixels, of each frame.
	* @param   ibpp	   Bits per pixel -- 24 (BGR), 32 (BGRU). or 8 bit greyscale.
	* @param   iFramerate  Framerate that the .avi will play back in.
	*/
	bool	 open( 
		const char* pszFilename, 
		int	  iCols, 
		int	  iRows, 
		int	  ibpp, 
		int	  iFramerate );

	/** Open an .avi file for reading. */
	bool open( char* pszFilename );

   PAVIFILE    m_pavifile;
   PAVISTREAM  m_pstream;
   PAVISTREAM  m_pstreamcompressed;

   /** Height, in pixels, of each frame in the .avi. */
   int	 m_iRows;

   /** Width, in pixels, of each frame in the .avi. */
   int	 m_iCols;

   /** Bits per pixel of the .avi. */
   int	 m_iBPP;

   /** Row increment, in bytes. */
   int	 m_iRowInc;

   /** Image size in bytes. */
   int	 m_iSize;

   /** Frame rate */
   double m_frameRate;

   /** Time index for current frame. */
   int	 m_iTimeIndex;

   /** Temporary image buffer. */
   unsigned char* m_pTempBuffer;

   /** Temporary buffer for saving .bmps. */
   unsigned char* m_pTempBMPBuffer;

   /** Our bitmapinfo structure */
   BITMAPINFO* m_pBitmapInfo;

   /** Avi file counter. */
   int m_iSplitFile;

   /** Flag indicating if the size of the avi file is limited to AVI_FILE_SPLIT_SIZE bytes */
   bool m_bSizeLimited;

   /** Total bytes written. */
   long int m_liBytesWritten;

   /** avi file name */
   char m_szAVIDestFile[ _MAX_PATH ];

   /** Compressor options. */
   COMPVARS m_compvars;

private:
   
   /** Read and verify the Video For Windows version. */
   bool checkVFWVersion();

};


#endif // #ifndef __PGRAVIFILE_H__
