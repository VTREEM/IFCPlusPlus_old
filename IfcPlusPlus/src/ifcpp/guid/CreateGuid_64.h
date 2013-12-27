#pragma once

#include <string>

#ifdef WIN32
#include	<Windows.h>
std::string createGUID32();
#endif
// For non windows platforms you probably will need
// the following definitions instead of <Windows.h>
// to use this module:

#ifndef WIN32
#include <time.h>
#include <uuid/uuid.h>
#include <cstdlib>

typedef struct _GUID {		  // size is 16
	unsigned long Data1;
	unsigned short  Data2;
	unsigned short  Data3;
	unsigned char Data4[8];
} GUID;

typedef long BOOL;


#define FALSE	0
#define TRUE	 1

const GUID GUID_NULL = { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };
static long CoCreateGuid( GUID   *pGuid ) {
	//TODO: Check if this is actually compatible with the CoCreateGuid implementation in Windows.
	uuid_t uuid;
	uuid_generate_random ( uuid );

	//Append bytes from generated uuid_t to make a long and two short values for use in _GUID struct
	unsigned long data1 = (unsigned long)uuid[0];
	data1=data1<<24;
	data1=data1|((unsigned long)uuid[1])<<16|((unsigned long)uuid[2])<<8|(unsigned long)uuid[3];

	unsigned short data2 = (unsigned short) uuid[4];
	data2=(data2<<8)|(unsigned short)uuid[5];

	unsigned short data3 = (unsigned short) uuid[6];
	data3=(data3<<8)|(unsigned short)uuid[7];
	GUID guid = { data1, data2, data3, { uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15] } };
	*pGuid = guid;
	return 0;
};

#endif

std::string CreateCompressedGuidString22();

//#if defined (__cplusplus)
//extern "C"{
//#endif
//
// For all the following routines len contains the usable length of buf
// These routines will return NULL on failure and &buf on success.
// Upon successful completion buf will hold the resulting zero terminated strings.
//

char* CreateCompressedGuidString( char * buf, int len );					   // len >= 23
char* String64_To_HexaGuidString( const char *string64, char * buf, int len ); // len >= 39
char* String64_To_String85( const char *string64, char * buf, int len );	   // len >= 21
char* String85_To_String64( const char *string85, char * buf, int len );	   // len >= 23
char* getString16FromGuid( const GUID   *pGuid, char * buf, int len );
char* getString64FromGuid( const GUID *pGuid, char * buf, int len );
bool getGuidFromString64( const char *string, GUID *pGuid );

bool getGuidFromString85( const char	*string, GUID *pGuid );
char* getString85FromGuid( const GUID *pGuid, char * buf, int len );
