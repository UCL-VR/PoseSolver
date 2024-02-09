// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"

#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "Export.h"


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

EXPORT float getVersion()
{
    return 0.01f;
}

EXPORT void HelloWorld()
{
}