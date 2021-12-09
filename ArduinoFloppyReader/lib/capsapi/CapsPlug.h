#ifndef CAPSPLUG_H
#define CAPSPLUG_H

// library function pointers
struct CapsProc {
	char* name;
#ifndef _WIN32
	FARPROC proc;
#else
	void* proc;
#endif
};

typedef SDWORD (__cdecl *CAPSHOOKN)(...);
typedef PCHAR  (__cdecl *CAPSHOOKS)(...);

extern "C" {

SDWORD CapsInit();
SDWORD CapsExit();
SDWORD CapsAddImage();
SDWORD CapsRemImage(SDWORD id);
SDWORD CapsLockImage(SDWORD id, PCHAR name);
SDWORD CapsLockImageMemory(SDWORD id, PUBYTE buffer, UDWORD length, UDWORD flag);
SDWORD CapsUnlockImage(SDWORD id);
SDWORD CapsLoadImage(SDWORD id, UDWORD flag);
SDWORD CapsGetImageInfo(PCAPSIMAGEINFO pi, SDWORD id);
SDWORD CapsLockTrack(PCAPSTRACKINFO pi, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD flag);
SDWORD CapsUnlockTrack(SDWORD id, UDWORD cylinder, UDWORD head);
SDWORD CapsUnlockAllTracks(SDWORD id);
SDWORD CapsGetInfo(void* si, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD inftype, UDWORD infid);
PCHAR  CapsGetPlatformName(UDWORD pid);

}

#endif
