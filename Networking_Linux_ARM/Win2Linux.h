#pragma once
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
//typedef unsigned long DWORD;
//typedef unsigned long ULONG;
// in windows, long is 4 bytes, but on Linux ARM is resolves to 8 bytes, so
typedef unsigned int DWORD;
typedef unsigned int ULONG;

//typedef long long LONGLONG;
typedef long LONGLONG;
typedef int LONG;
typedef int HRESULT;

typedef void* HANDLE;
typedef void *PVOID;
typedef void* LPVOID;

typedef int BOOL;
typedef unsigned int UINT;
typedef unsigned long long ULONG_PTR;

typedef unsigned short USHORT;
typedef USHORT ADDRESS_FAMILY;
typedef unsigned short UINT16;
typedef unsigned char BYTE;
typedef char CHAR;

typedef union _LARGE_INTEGER {
    struct {    
        DWORD LowPart;
        LONG HighPart;
    } DUMMYSTRUCTNAME;
    struct {
        DWORD LowPart;
        LONG HighPart;
    } u;
    LONGLONG QuadPart;
} LARGE_INTEGER;

typedef struct _GUID {
    unsigned int  Data1;
    unsigned short Data2;
    unsigned short Data3;
    unsigned char  Data4[ 8 ];
} GUID;

typedef struct _OVERLAPPED {
    ULONG_PTR Internal;
    ULONG_PTR InternalHigh;
    union {
        struct {
            DWORD Offset;
            DWORD OffsetHigh;
        } DUMMYSTRUCTNAME;
        PVOID Pointer;
    } DUMMYUNIONNAME;

    HANDLE  hEvent;
} OVERLAPPED, *LPOVERLAPPED;
typedef struct _OVERLAPPED *    LPWSAOVERLAPPED;

typedef struct _WSABUF {
    ULONG len;     /* the length of the buffer */
    CHAR *buf; /* the pointer to the buffer */
} WSABUF, *LPWSABUF;

#define _SS_MAXSIZE 128                 // Maximum size
#define _SS_ALIGNSIZE (sizeof(LONGLONG)) // Desired alignment
#define _SS_PAD1SIZE (_SS_ALIGNSIZE - sizeof(USHORT))
#define _SS_PAD2SIZE (_SS_MAXSIZE - (sizeof(USHORT) + _SS_PAD1SIZE + _SS_ALIGNSIZE))

// STDMETHODCALLTYPE resolves to __stdcall in Win32, and shouldn't be needed at all in Linux
#define STDMETHODCALLTYPE  
#define WINAPI
#define interface struct
#ifndef PURE
    #define PURE = 0
#endif
#define INFINITE 0xFFFFFFFF

#define S_OK 0
#define FALSE 0
#define SUCCEEDED(hr) (((HRESULT)(hr)) >= 0)
#define FAILED(hr) (((HRESULT)(hr)) < 0)

#define WAIT_OBJECT_0       ((STATUS_WAIT_0 ) + 0 )
#define WSAID_ACCEPTEX \
        {0xb5367df1,0xcbac,0x11cf,{0x95,0xca,0x00,0x80,0x5f,0x48,0xa1,0x92}}
#define WSAID_GETACCEPTEXSOCKADDRS \
        {0xb5367df2,0xcbac,0x11cf,{0x95,0xca,0x00,0x80,0x5f,0x48,0xa1,0x92}}


//HRESULT CODES from winerror.h
#define E_INVALIDARG    0x80070057L
#define E_FAIL          0x80004005L
#define E_POINTER       0x80004003L
#define E_ABORT         0x80004004L
#define E_OUTOFMEMORY   0x8007000EL

#include <string.h>
#define memcpy_s(dest, destSize, src, count) memcpy(dest, src, count)
#define sscanf_s sscanf
