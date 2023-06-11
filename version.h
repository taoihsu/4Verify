// ----------------------------------------------------------------------------
// --- written by Joseph Braun [12-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#ifndef VERSION_H
#define VERSION_H

#include "..\Library\AppCtrl\Include\AppCtrlVersion.h"

#define VERSION_MAJOR               1
#define VERSION_MINOR               0
#define VERSION_REVISION            0
#define VERSION_BUILD               0
#define VERSION_VARIANCE            0

#define VER_DATE_STR	    	    "$Date: Mar-29-21 1:43:34 PM $"
#define VER_FILE_VERSION            VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION, VERSION_BUILD
#define VER_FILE_VERSION_COMPACT    (VERSION_MAJOR<<24|VERSION_MINOR<<16|VERSION_REVISION<<8|VERSION_BUILD)
#define VER_FILE_VERSION_STR        STR2(VERSION_MAJOR) STR2(VERSION_MINOR) STR2(VERSION_REVISION) STR2(VERSION_BUILD)

#define VER_PRODUCT_VERSION         VER_FILE_VERSION
#define VER_PRODUCT_VERSION_STR     VER_APPCTRL_VERSION_PREFIX_STR "." VER_FILE_VERSION_STR "." STR2(VERSION_VARIANCE)
#define VER_PRODUCTNAME_STR         "OC.DLL"
#define VER_ORIGINAL_FILENAME_STR   VER_APPCTRL_BRANCH_STR ".AppCtrl." VER_PRODUCTNAME_STR
#define VER_INTERNAL_NAME_STR       "AppCtrl - Online Calibration "


#endif // !VERSION_H
