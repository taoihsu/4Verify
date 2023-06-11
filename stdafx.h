// ----------------------------------------------------------------------------
// --- written by Rathi G. R. [28-JUN-2012]
// --- Copyright (c) Magna Vectrics (MEVC) 2012
// ----------------------------------------------------------------------------
#ifndef STDAFX_H
#define STDAFX_H

#include "targetver.h"

// --- Exclude rarely-used stuff from Windows headers
#define WIN32_LEAN_AND_MEAN
// --- some CString constructors will be explicit
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS


// --- Exclude rarely-used stuff from Windows headers
#ifndef VC_EXTRALEAN
    #define VC_EXTRALEAN
#endif

#include <afx.h>
// --- MFC core and standard components
#include <afxwin.h>
// --- MFC extensions
#include <afxext.h>

// --- MFC support for Internet Explorer 4 Common Controls
#ifndef _AFX_NO_OLE_SUPPORT
    #include <afxdtctl.h>
#endif

// --- MFC support for Windows Common Controls
#ifndef _AFX_NO_AFXCMN_SUPPORT
    #include <afxcmn.h>
#endif

// --- windows
#include <iostream>
#include <windows.h>


// --- some CString constructors will be explicit
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS


// --- ATL
#include <atlbase.h>
#include <atlstr.h>
// ----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
#define _IPP_SEQUENTIAL_STATIC
#include "ipp.h"
//extern Ipp64u g__ipp_cpuid;

#pragma comment (lib, "IPPCORE_L")
#pragma comment (lib, "IPPS_L")
#pragma comment (lib, "IPPI_L")
#pragma comment (lib, "IPPCC_L")
#pragma comment (lib, "IPPCV_L")
#pragma comment( lib, "ippm" )

// -----------------------------------------------------------------------------
#include "FGWIN.H"
#pragma comment (lib, "FGWVC32")
#define MARKUP_STL
#include "MARKUP.H"
// ----------------------------------------------------------------------------
#include "VIDFileEx.h"
#include "TracerDB.h"
#include "T_DataLock.h"
#include "T_Bitmap.h"
#include "AppCtrlAPI.h"
#include "AppCtrlUtility.h"
// ----------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ml/ml.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/video.hpp"

// ----------------------------------------------------------------------------
#include <VideoImageIntf.h>
#include <CANTranslationIntf.h>
#include <KinematicModelIntf.h>
#include <integratorDataBlock.h>
#include <version.h>
#include <common.h>
#include "OCDef.h"

#ifdef _DEBUG
    #pragma comment(lib, "opencv_world340d.lib" )
#else
    #pragma comment(lib, "opencv_world340.lib" )
#endif

// ----------------------------------------------------------------------------
#endif // !STDAFX_H

