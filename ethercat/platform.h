#ifndef __PLATFORM__
#define __PLATFORM__

#ifdef WIN32

#define EXPORT __declspec(dllexport)

#else
#define EXPORT
#define POSIX
#endif


#endif
