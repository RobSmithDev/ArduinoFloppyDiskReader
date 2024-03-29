#ifndef CAPSLIB_H
#define CAPSLIB_H

#ifdef _WIN32
#define CAPS_CALLING __cdecl
#else
#define CAPS_CALLING
#endif

SDWORD CAPS_CALLING CAPSInit();
SDWORD CAPS_CALLING CAPSExit();
SDWORD CAPS_CALLING CAPSAddImage();
SDWORD CAPS_CALLING CAPSRemImage(SDWORD id);
SDWORD CAPS_CALLING CAPSLockImage(SDWORD id, PCHAR name);
SDWORD CAPS_CALLING CAPSLockImageMemory(SDWORD id, PUBYTE buffer, UDWORD length, UDWORD flag);
SDWORD CAPS_CALLING CAPSUnlockImage(SDWORD id);
SDWORD CAPS_CALLING CAPSLoadImage(SDWORD id, UDWORD flag);
SDWORD CAPS_CALLING CAPSGetImageInfo(PCAPSIMAGEINFO pi, SDWORD id);
SDWORD CAPS_CALLING CAPSLockTrack(PCAPSTRACKINFO pi, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD flag);
SDWORD CAPS_CALLING CAPSUnlockTrack(SDWORD id, UDWORD cylinder, UDWORD head);
SDWORD CAPS_CALLING CAPSUnlockAllTracks(SDWORD id);
PCHAR  CAPS_CALLING CAPSGetPlatformName(UDWORD pid);
SDWORD CAPS_CALLING CAPSGetVersionInfo(PCAPSVERSIONINFO pi, UDWORD flag);
/*
SDWORD CAPS_CALLING CAPSInit();
SDWORD CAPS_CALLING CAPSExit();
SDWORD CAPS_CALLING CAPSAddImage();
SDWORD CAPS_CALLING CAPSRemImage(SDWORD id);
SDWORD CAPS_CALLING CAPSLockImage(SDWORD id, PCHAR name);
SDWORD CAPS_CALLING CAPSLockImageMemory(SDWORD id, PUBYTE buffer, UDWORD length, UDWORD flag);
SDWORD CAPS_CALLING CAPSUnlockImage(SDWORD id);
SDWORD CAPS_CALLING CAPSLoadImage(SDWORD id, UDWORD flag);
SDWORD CAPS_CALLING CAPSGetImageInfo(PCAPSIMAGEINFO pi, SDWORD id);
SDWORD CAPS_CALLING CAPSLockTrack(PVOID ptrackinfo, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD flag);
SDWORD CAPS_CALLING CAPSUnlockTrack(SDWORD id, UDWORD cylinder, UDWORD head);
SDWORD CAPS_CALLING CAPSUnlockAllTracks(SDWORD id);
PCHAR  CAPS_CALLING CAPSGetPlatformName(UDWORD pid);
SDWORD CAPS_CALLING CAPSGetVersionInfo(PVOID pversioninfo, UDWORD flag);
UDWORD CAPS_CALLING CAPSFdcGetInfo(SDWORD iid, PCAPSFDC pc, SDWORD ext);
SDWORD CAPS_CALLING CAPSFdcInit(PCAPSFDC pc);
void   CAPS_CALLING CAPSFdcReset(PCAPSFDC pc);
void   CAPS_CALLING CAPSFdcEmulate(PCAPSFDC pc, UDWORD cyclecnt);
UDWORD CAPS_CALLING CAPSFdcRead(PCAPSFDC pc, UDWORD address);
void   CAPS_CALLING CAPSFdcWrite(PCAPSFDC pc, UDWORD address, UDWORD data);
SDWORD CAPS_CALLING CAPSFdcInvalidateTrack(PCAPSFDC pc, SDWORD drive);
SDWORD CAPS_CALLING CAPSFormatDataToMFM(PVOID pformattrack, UDWORD flag);
SDWORD CAPS_CALLING CAPSGetInfo(PVOID pinfo, SDWORD id, UDWORD cylinder, UDWORD head, UDWORD inftype, UDWORD infid);
SDWORD CAPS_CALLING CAPSSetRevolution(SDWORD id, UDWORD value);
SDWORD CAPS_CALLING CAPSGetImageType(PCHAR name);
SDWORD CAPS_CALLING CAPSGetImageTypeMemory(PUBYTE buffer, UDWORD length);
SDWORD CAPS_CALLING CAPSGetDebugRequest();
*/
#endif
