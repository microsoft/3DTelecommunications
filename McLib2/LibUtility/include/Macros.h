// MSR CCS Copyright 2011
//----------------------------------------------

#pragma once

#include <iostream>

#ifndef SAFE_ARRAY_DELETE
#define SAFE_ARRAY_DELETE(ptrMem)   if(ptrMem){ delete[] ptrMem; ptrMem = NULL;}
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(p)       { if (p) { delete (p);     (p)=NULL; } }
#endif   

#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if (p) { (p)->Release(); (p)=NULL; } }
#endif

#ifndef MAX
#define MAX(a,b)   ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b)   ((a) < (b) ? (a) : (b))
#endif

#ifndef MC_ERROR
#define MC_ERROR( message ) do { \
    std::cout << "ERROR! " << message << "\n" \
        << "  " << __FILE__ << ":" << __LINE__ << "\n"; \
    exit(0); \
} while(false)
#endif

#ifndef MC_VERIFY
#define MC_VERIFY( condition, message ) do { \
    if(!(condition)) \
        MC_ERROR(message); \
} while(false)
#endif
