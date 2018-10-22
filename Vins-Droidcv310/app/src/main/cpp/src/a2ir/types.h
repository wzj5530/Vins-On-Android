//
// Created by win7 on 2017/12/20.
//

#ifndef ARDEMO_TYPES_H
#define ARDEMO_TYPES_H


//-----------------------------------------------------------------------------------
// ****** Operating System
//
// Type definitions exist for the following operating systems: (PVR_OS_x)
//
//    WIN32    - Win32 (Windows 95/98/ME and Windows NT/2000/XP)
//    DARWIN   - Darwin OS (Mac OS X)
//    LINUX    - Linux
//    ANDROID  - Android
//    IPHONE   - iPhone

#if (defined(__APPLE__) && (defined(__GNUC__) ||\
     defined(__xlC__) || defined(__xlc__))) || defined(__MACOS__)
#  if (defined(__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__) || defined(__IPHONE_OS_VERSION_MIN_REQUIRED))
#    define PVR_OS_IPHONE
#  else
#    define PVR_OS_DARWIN
#    define PVR_OS_MAC
#  endif
#elif (defined(WIN64) || defined(_WIN64) || defined(__WIN64__))
#  define PVR_OS_WIN32
#elif (defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__))
#  define PVR_OS_WIN32
#elif defined(__linux__) || defined(__linux)
#  define PVR_OS_LINUX
#else
#  define PVR_OS_OTHER
#endif

#if defined(ANDROID)
#  define PVR_OS_ANDROID
#endif


//-----------------------------------------------------------------------------------
// ***** CPU Architecture
//
// The following CPUs are defined: (PVR_CPU_x)
//
//    X86        - x86 (IA-32)
//    X86_64     - x86_64 (amd64)
//    PPC        - PowerPC
//    PPC64      - PowerPC64
//    MIPS       - MIPS
//    OTHER      - CPU for which no special support is present or needed


#if defined(__x86_64__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
#  define PVR_CPU_X86_64
#  define PVR_64BIT_POINTERS
#elif defined(__i386__) || defined(PVR_OS_WIN32)
#  define PVR_CPU_X86
#elif defined(__powerpc64__)
#  define PVR_CPU_PPC64
#elif defined(__ppc__)
#  define PVR_CPU_PPC
#elif defined(__mips__) || defined(__MIPSEL__)
#  define PVR_CPU_MIPS
#elif defined(__arm__)
#  define PVR_CPU_ARM
#else
#  define PVR_CPU_OTHER
#endif

//-----------------------------------------------------------------------------------
// ***** Co-Processor Architecture
//
// The following co-processors are defined: (PVR_CPU_x)
//
//    SSE        - Available on all modern x86 processors.
//    Altivec    - Available on all modern ppc processors.
//    Neon       - Available on some armv7+ processors.

#if defined(__SSE__) || defined(PVR_OS_WIN32)
#  define  PVR_CPU_SSE
#endif // __SSE__

#if defined( __ALTIVEC__ )
#  define PVR_CPU_ALTIVEC
#endif // __ALTIVEC__

#if defined(__ARM_NEON__)
#  define PVR_CPU_ARM_NEON
#endif // __ARM_NEON__


//-----------------------------------------------------------------------------------
// ***** Compiler
//
//  The following compilers are defined: (PVR_CC_x)
//
//     MSVC     - Microsoft Visual C/C++
//     INTEL    - Intel C++ for Linux / Windows
//     GNU      - GNU C++
//     ARM      - ARM C/C++

#if defined(__INTEL_COMPILER)
// Intel 4.0                    = 400
// Intel 5.0                    = 500
// Intel 6.0                    = 600
// Intel 8.0                    = 800
// Intel 9.0                    = 900
#  define PVR_CC_INTEL       __INTEL_COMPILER

#elif defined(_MSC_VER)
// MSVC 5.0                     = 1100
// MSVC 6.0                     = 1200
// MSVC 7.0 (VC2002)            = 1300
// MSVC 7.1 (VC2003)            = 1310
// MSVC 8.0 (VC2005)            = 1400
// MSVC 9.0 (VC2008)            = 1500
// MSVC 10.0 (VC2010)           = 1600
#  define PVR_CC_MSVC        _MSC_VER

#elif defined(__GNUC__)
#  define PVR_CC_GNU

#elif defined(__CC_ARM)
#  define PVR_CC_ARM

#else
#  error "Pico does not support this Compiler"
#endif


//-----------------------------------------------------------------------------------
// ***** Compiler Warnings

// Disable MSVC warnings
#if defined(PVR_CC_MSVC)
#  pragma warning(disable : 4127)    // Inconsistent dll linkage
#  pragma warning(disable : 4530)    // Exception handling
#  if (PVR_CC_MSVC<1300)
#    pragma warning(disable : 4514)  // Unreferenced inline function has been removed
#    pragma warning(disable : 4710)  // Function not inlined
#    pragma warning(disable : 4714)  // _force_inline not inlined
#    pragma warning(disable : 4786)  // Debug variable name longer than 255 chars
#  endif // (PVR_CC_MSVC<1300)
#endif // (PVR_CC_MSVC)



// *** Linux Unicode - must come before Standard Includes

#ifdef PVR_OS_LINUX
// Use glibc unicode functions on linux.
#  ifndef  _GNU_SOURCE
#    define _GNU_SOURCE
#  endif
#endif

//-----------------------------------------------------------------------------------
// ***** Standard Includes
//
#include    <stddef.h>
#include    <limits.h>
#include    <float.h>


// MSVC Based Memory Leak checking - for now
#if defined(PVR_CC_MSVC) && defined(PVR_BUILD_DEBUG)
#  define _CRTDBG_MAP_ALLOC
#  include <stdlib.h>
#  include <crtdbg.h>

// Uncomment this to help debug memory leaks under Visual Studio in PVR apps only.
// This shouldn't be defined in customer releases.
#  ifndef PVR_DEFINE_NEW
#    define PVR_DEFINE_NEW new(__FILE__, __LINE__)
#    define new PVR_DEFINE_NEW
#  endif

#endif


//-----------------------------------------------------------------------------------
// ***** Type definitions for Common Systems

namespace PVR {

    typedef char            Char;

// Pointer-sized integer
    typedef size_t          UPInt;
    typedef ptrdiff_t       SPInt;


#if defined(PVR_OS_WIN32)

    typedef char            SByte;  // 8 bit Integer (Byte)
typedef unsigned char   UByte;
typedef short           SInt16; // 16 bit Integer (Word)
typedef unsigned short  UInt16;
typedef long            SInt32; // 32 bit Integer
typedef unsigned long   UInt32;
typedef __int64         SInt64; // 64 bit Integer (QWord)
typedef unsigned __int64 UInt64;


#elif defined(PVR_OS_MAC) || defined(PVR_OS_IPHONE) || defined(PVR_CC_GNU)

    typedef int             SByte  __attribute__((__mode__ (__QI__)));
    typedef unsigned int    UByte  __attribute__((__mode__ (__QI__)));
    typedef int             SInt16 __attribute__((__mode__ (__HI__)));
    typedef unsigned int    UInt16 __attribute__((__mode__ (__HI__)));
    typedef int             SInt32 __attribute__((__mode__ (__SI__)));
    typedef unsigned int    UInt32 __attribute__((__mode__ (__SI__)));
    typedef int             SInt64 __attribute__((__mode__ (__DI__)));
    typedef unsigned int    UInt64 __attribute__((__mode__ (__DI__)));

#else

    #include <sys/types.h>
typedef int8_t          SByte;
typedef uint8_t         UByte;
typedef int16_t         SInt16;
typedef uint16_t        UInt16;
typedef int32_t         SInt32;
typedef uint32_t        UInt32;
typedef int64_t         SInt64;
typedef uint64_t        UInt64;

#endif


// ***** BaseTypes Namespace

// BaseTypes namespace is explicitly declared to allow base types to be used
// by customers directly without other contents of PVR namespace.
//
// Its is expected that GFx samples will declare 'using namespace PVR::BaseTypes'
// to allow using these directly without polluting the target scope with other
// PVR declarations, such as Ptr<>, String or Mutex.
    namespace BaseTypes
    {
        using PVR::UPInt;
        using PVR::SPInt;
        using PVR::UByte;
        using PVR::SByte;
        using PVR::UInt16;
        using PVR::SInt16;
        using PVR::UInt32;
        using PVR::SInt32;
        using PVR::UInt64;
        using PVR::SInt64;
    } // PVR::BaseTypes

} // PVR


//-----------------------------------------------------------------------------------
// ***** Macro Definitions
//
// We define the following:
//
//  PVR_BYTE_ORDER      - Defined to either PVR_LITTLE_ENDIAN or PVR_BIG_ENDIAN
//  PVR_FORCE_INLINE    - Forces inline expansion of function
//  PVR_ASM             - Assembly language prefix
//  PVR_STR             - Prefixes string with L"" if building unicode
//
//  PVR_STDCALL         - Use stdcall calling convention (Pascal arg order)
//  PVR_CDECL           - Use cdecl calling convention (C argument order)
//  PVR_FASTCALL        - Use fastcall calling convention (registers)
//

// Byte order constants, PVR_BYTE_ORDER is defined to be one of these.
#define PVR_LITTLE_ENDIAN       1
#define PVR_BIG_ENDIAN          2


// Force inline substitute - goes before function declaration
#if defined(PVR_CC_MSVC)
#  define PVR_FORCE_INLINE  __forceinline
#elif defined(PVR_CC_GNU)
#  define PVR_FORCE_INLINE  inline __attribute__((always_inline))
#else
#  define PVR_FORCE_INLINE  inline
#endif  // PVR_CC_MSVC


#if defined(PVR_OS_WIN32)

// ***** Win32

    // Byte order
    #define PVR_BYTE_ORDER    PVR_LITTLE_ENDIAN

    // Calling convention - goes after function return type but before function name
    #ifdef __cplusplus_cli
    #  define PVR_FASTCALL      __stdcall
    #else
    #  define PVR_FASTCALL      __fastcall
    #endif

    #define PVR_STDCALL         __stdcall
    #define PVR_CDECL           __cdecl


    // Assembly macros
    #if defined(PVR_CC_MSVC)
    #  define PVR_ASM           _asm
    #else
    #  define PVR_ASM           asm
    #endif // (PVR_CC_MSVC)

    #ifdef UNICODE
    #  define PVR_STR(str)      L##str
    #else
    #  define PVR_STR(str)      str
    #endif // UNICODE

#else

// **** Standard systems

#if (defined(BYTE_ORDER) && (BYTE_ORDER == BIG_ENDIAN))|| \
        (defined(_BYTE_ORDER) && (_BYTE_ORDER == _BIG_ENDIAN))
#  define PVR_BYTE_ORDER    PVR_BIG_ENDIAN
#elif (defined(__ARMEB__) || defined(PVR_CPU_PPC) || defined(PVR_CPU_PPC64))
#  define PVR_BYTE_ORDER    PVR_BIG_ENDIAN
#else
#  define PVR_BYTE_ORDER    PVR_LITTLE_ENDIAN
#endif

// Assembly macros
#define PVR_ASM                  __asm__
#define PVR_ASM_PROC(procname)   PVR_ASM
#define PVR_ASM_END              PVR_ASM

// Calling convention - goes after function return type but before function name
#define PVR_FASTCALL
#define PVR_STDCALL
#define PVR_CDECL

#endif // defined(PVR_OS_WIN32)



//-----------------------------------------------------------------------------------
// ***** PVR_DEBUG_BREAK, PVR_ASSERT
//
// If not in debug build, macros do nothing
#ifndef PVR_BUILD_DEBUG

#  define PVR_DEBUG_BREAK  ((void)0)
#  define PVR_ASSERT(p)    ((void)0)

#else

// Microsoft Win32 specific debugging support
#if defined(PVR_OS_WIN32)
#  ifdef PVR_CPU_X86
#    if defined(__cplusplus_cli)
#      define PVR_DEBUG_BREAK   do { __debugbreak(); } while(0)
#    elif defined(PVR_CC_GNU)
#      define PVR_DEBUG_BREAK   do { PVR_ASM("int $3\n\t"); } while(0)
#    else
#      define PVR_DEBUG_BREAK   do { PVR_ASM int 3 } while (0)
#    endif
#  else
#    define PVR_DEBUG_BREAK     do { __debugbreak(); } while(0)
#  endif
// Android specific debugging support
#elif defined(PVR_OS_ANDROID)
#  define PVR_DEBUG_BREAK       do { __builtin_trap(); } while(0)
// Unix specific debugging support
#elif defined(PVR_CPU_X86) || defined(PVR_CPU_X86_64)
#  define PVR_DEBUG_BREAK       do { PVR_ASM("int $3\n\t"); } while(0)
#else
#  define PVR_DEBUG_BREAK       do { *((int *) 0) = 1; } while(0)
#endif

// This will cause compiler breakpoint
#define PVR_ASSERT(p)           do { if (!(p))  { PVR_DEBUG_BREAK; } } while(0)

#endif // PVR_BUILD_DEBUG


// Compile-time assert; produces compiler error if condition is false
#define PVR_COMPILER_ASSERT(x)  { int zero = 0; switch(zero) {case 0: case x:;} }



//-----------------------------------------------------------------------------------
// ***** PVR_UNUSED - Unused Argument handling

// Macro to quiet compiler warnings about unused parameters/variables.
#if defined(PVR_CC_GNU)
#  define   PVR_UNUSED(a)   do {__typeof__ (&a) __attribute__ ((unused)) __tmp = &a; } while(0)
#else
#  define   PVR_UNUSED(a)   (a)
#endif

#define     PVR_UNUSED1(a1) PVR_UNUSED(a1)
#define     PVR_UNUSED2(a1,a2) PVR_UNUSED(a1); PVR_UNUSED(a2)
#define     PVR_UNUSED3(a1,a2,a3) PVR_UNUSED2(a1,a2); PVR_UNUSED(a3)
#define     PVR_UNUSED4(a1,a2,a3,a4) PVR_UNUSED3(a1,a2,a3); PVR_UNUSED(a4)
#define     PVR_UNUSED5(a1,a2,a3,a4,a5) PVR_UNUSED4(a1,a2,a3,a4); PVR_UNUSED(a5)
#define     PVR_UNUSED6(a1,a2,a3,a4,a5,a6) PVR_UNUSED4(a1,a2,a3,a4); PVR_UNUSED2(a5,a6)
#define     PVR_UNUSED7(a1,a2,a3,a4,a5,a6,a7) PVR_UNUSED4(a1,a2,a3,a4); PVR_UNUSED3(a5,a6,a7)
#define     PVR_UNUSED8(a1,a2,a3,a4,a5,a6,a7,a8) PVR_UNUSED4(a1,a2,a3,a4); PVR_UNUSED4(a5,a6,a7,a8)
#define     PVR_UNUSED9(a1,a2,a3,a4,a5,a6,a7,a8,a9) PVR_UNUSED4(a1,a2,a3,a4); PVR_UNUSED5(a5,a6,a7,a8,a9)


//-----------------------------------------------------------------------------------
// ***** Configuration Macros

// SF Build type
#ifdef PVR_BUILD_DEBUG
#  define PVR_BUILD_STRING  "Debug"
#else
#  define PVR_BUILD_STRING  "Release"
#endif


//// Enables SF Debugging information
//# define PVR_BUILD_DEBUG

// PVR_DEBUG_STATEMENT injects a statement only in debug builds.
// PVR_DEBUG_SELECT injects first argument in debug builds, second argument otherwise.
#ifdef PVR_BUILD_DEBUG
#define PVR_DEBUG_STATEMENT(s)   s
#define PVR_DEBUG_SELECT(d, nd)  d
#else
#define PVR_DEBUG_STATEMENT(s)
#define PVR_DEBUG_SELECT(d, nd)  nd
#endif


#define PVR_ENABLE_THREADS
//
// Prevents PVR from defining new within
// type macros, so developers can override
// new using the #define new new(...) trick
// - used with PVR_DEFINE_NEW macro
//# define PVR_BUILD_DEFINE_NEW
//

#endif //ARDEMO_TYPES_H
