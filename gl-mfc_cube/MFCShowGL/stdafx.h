
// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 从 Windows 头中排除极少使用的资料
#endif

#include "targetver.h"

#define POINTER_64 __ptr64
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 某些 CString 构造函数将是显式的

// 关闭 MFC 对某些常见但经常可放心忽略的警告消息的隐藏
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 核心组件和标准组件
#include <afxext.h>         // MFC 扩展


#include <afxdisp.h>        // MFC 自动化类



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC 对 Internet Explorer 4 公共控件的支持
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // MFC 对 Windows 公共控件的支持
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // 功能区和控件条的 MFC 支持

// TODO: 在此处引用程序需要的其他头文件
#include "../glutdlls37beta/glew.h"		// 包含最新的gl.h,glu.h库
#include "../glutdlls37beta/glut.h"
#include "../glutdlls37beta/glaux.h"
/*
1>glaux.lib(tk.obj) : error LNK2019: 无法解析的外部符号 _sscanf，该符号在函数 _GetRegistrySysColors@8 中被引用
1>glaux.lib(tk.obj) : error LNK2019: 无法解析的外部符号 _vsprintf，该符号在函数 _PrintMessage 中被引用
VS2015编译会出现这个问题，解决办法在项目属性->链接器->输入->附加依赖项中添加依赖项 legacy_stdio_definitions.lib;
*/
#pragma comment (lib, "legacy_stdio_definitions.lib")
#pragma comment (lib, "../glutdlls37beta/freeglut.lib")
#pragma comment (lib, "../glutdlls37beta/GLu32.lib")
#pragma comment (lib, "../glutdlls37beta/GLut.lib")
#pragma comment (lib, "../glutdlls37beta/GLut32.lib")
#pragma comment (lib, "../glutdlls37beta/GLaux.lib")
#pragma comment (lib, "../glutdlls37beta/OpenGL32.lib")



#ifdef _DEBUG
#include "debug.h"
#else
#include "release.h"
#endif

using namespace std;
using namespace cv;







#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif


