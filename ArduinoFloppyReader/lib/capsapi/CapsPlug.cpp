// Simple caps plug-in support on Win32, should fit most users
//
// You may want to link with the libray instead, if your product fully complies
// with the license.
// If using the library directly without the plug-in, define CAPS_USER, and include CapsLib.h
// functions are the same with the plug-in, but their name start with "CAPS", not "Caps"
// CAPSInit does not have any parameters, CapsInit gets the library name
//
// Comment out stdafx.h if your project does not use the MSVC precompiled headers
//
// www.caps-project.org
//
// Modified for basic cross-platform support

#include "Comtype.h"
#include "CapsAPI.h"
#include "CapsPlug.h"


#ifdef _WIN32
#include "windows.h"

#define CALLING_CONVENTION WINAPI
#define GETFUNC GetProcAddress

HMODULE capi = NULL;


#else
#include <dlfcn.h>

#define CALLING_CONVENTION
#define GETFUNC dlsym

void* capi = NULL;
#endif



CapsProc cpr[]= {
	"CAPSInit", NULL,
	"CAPSExit", NULL,
	"CAPSAddImage", NULL,
	"CAPSRemImage", NULL,
	"CAPSLockImage", NULL,
	"CAPSUnlockImage", NULL,
	"CAPSLoadImage", NULL,
	"CAPSGetImageInfo", NULL,
	"CAPSLockTrack", NULL,
	"CAPSUnlockTrack", NULL,
	"CAPSUnlockAllTracks", NULL,
	"CAPSGetPlatformName", NULL,
	"CAPSLockImageMemory", NULL,
	"CAPSGetInfo", NULL,
	NULL, NULL
};



// start caps image support
SDWORD CapsInit()
{
	if (capi)
		return imgeOk;

#ifdef WIN32
	capi = LoadLibrary(L"CAPSImg.dll");
#else
	capi = dlopen("libcapsimage.so.5", RTLD_NOW);
	if (!capi) capi = dlopen("libcapsimage.so.5", RTLD_NOW);
	if (!capi) capi = dlopen("libcapsimage.so.5.1", RTLD_NOW);
	if (!capi) capi = dlopen("libcapsimage.so.4", RTLD_NOW);
	if (!capi) capi = dlopen("libcapsimage.so.4.2", RTLD_NOW);
	if (!capi) capi = dlopen("libcapsimage.so", RTLD_NOW);
#endif
	if (!capi)
		return imgeUnsupported;

	for (int prc=0; cpr[prc].name; prc++)
		cpr[prc].proc= GETFUNC(capi, cpr[prc].name);

	SDWORD res=cpr[0].proc ? CAPSHOOKN(cpr[0].proc)() : imgeUnsupported;

	return res;
}

// stop caps image support
SDWORD CapsExit()
{
	SDWORD res=cpr[1].proc ? CAPSHOOKN(cpr[1].proc)() : imgeUnsupported;

	if (capi) {
#ifdef _WIN32
		FreeLibrary(capi);
#else
		dlclose(capi);
#endif
		capi=NULL;
	}

	for (int prc=0; cpr[prc].name; prc++)
		cpr[prc].proc=NULL;

	return res;
}

// add image container
SDWORD CapsAddImage()
{
	SDWORD res=cpr[2].proc ? CAPSHOOKN(cpr[2].proc)() : -1;

	return res;
}

// delete image container
SDWORD CapsRemImage(SDWORD id)
{
	SDWORD res=cpr[3].proc ? CAPSHOOKN(cpr[3].proc)(id) : -1;

	return res;
}

// lock image
SDWORD CapsLockImage(SDWORD id, PCHAR name)
{
	SDWORD res=cpr[4].proc ? CAPSHOOKN(cpr[4].proc)(id, name) : imgeUnsupported;

	return res;
}

// unlock image
SDWORD CapsUnlockImage(SDWORD id)
{
	SDWORD res=cpr[5].proc ? CAPSHOOKN(cpr[5].proc)(id) : imgeUnsupported;

	return res;
}

// load and decode complete image
SDWORD CapsLoadImage(SDWORD id, UDWORD flag)
{
	SDWORD res=cpr[6].proc ? CAPSHOOKN(cpr[6].proc)(id, flag) : imgeUnsupported;

	return res;
}

// get image information
SDWORD CapsGetImageInfo(PCAPSIMAGEINFO pi, SDWORD id)
{
	SDWORD res=cpr[7].proc ? CAPSHOOKN(cpr[7].proc)(pi, id) : imgeUnsupported;

	return res;
}

SDWORD CapsGetInfo(void* si, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD inftype, UDWORD infid) {
	SDWORD res = cpr[13].proc ? CAPSHOOKN(cpr[13].proc)(si, id, cylinder, head, inftype, infid) : imgeUnsupported;

	return res;
}

// load and decode track, or return with the cache
SDWORD CapsLockTrack(PCAPSTRACKINFO pi, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD flag)
{
	SDWORD res=cpr[8].proc ? CAPSHOOKN(cpr[8].proc)(pi, id, cylinder, head, flag) : imgeUnsupported;

	return res;
}

// remove track from cache
SDWORD CapsUnlockTrack(SDWORD id, UDWORD cylinder, UDWORD head)
{
	SDWORD res=cpr[9].proc ? CAPSHOOKN(cpr[9].proc)(id, cylinder, head) : imgeUnsupported;

	return res;
}

// remove all tracks from cache
SDWORD CapsUnlockAllTracks(SDWORD id)
{
	SDWORD res=cpr[10].proc ? CAPSHOOKN(cpr[10].proc)(id) : imgeUnsupported;

	return res;
}

// get platform name
PCHAR CapsGetPlatformName(UDWORD pid)
{
	PCHAR res=cpr[11].proc ? CAPSHOOKS(cpr[11].proc)(pid) : NULL;

	return res;
}

// lock memory mapped image
SDWORD CapsLockImageMemory(SDWORD id, PUBYTE buffer, UDWORD length, UDWORD flag)
{
	SDWORD res=cpr[12].proc ? CAPSHOOKN(cpr[12].proc)(id, buffer, length, flag) : imgeUnsupported;

	return res;
}

