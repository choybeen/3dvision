
// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // �� Windows ͷ���ų�����ʹ�õ�����
#endif

#include "targetver.h"

#define POINTER_64 __ptr64
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // ĳЩ CString ���캯��������ʽ��

// �ر� MFC ��ĳЩ�����������ɷ��ĺ��Եľ�����Ϣ������
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC ��������ͱ�׼���
#include <afxext.h>         // MFC ��չ


#include <afxdisp.h>        // MFC �Զ�����



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC �� Internet Explorer 4 �����ؼ���֧��
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // MFC �� Windows �����ؼ���֧��
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // �������Ϳؼ����� MFC ֧��

// TODO: �ڴ˴����ó�����Ҫ������ͷ�ļ�
#include "../glutdlls37beta/glew.h"		// �������µ�gl.h,glu.h��
#include "../glutdlls37beta/glut.h"
#include "../glutdlls37beta/glaux.h"
/*
1>glaux.lib(tk.obj) : error LNK2019: �޷��������ⲿ���� _sscanf���÷����ں��� _GetRegistrySysColors@8 �б�����
1>glaux.lib(tk.obj) : error LNK2019: �޷��������ⲿ���� _vsprintf���÷����ں��� _PrintMessage �б�����
VS2015��������������⣬����취����Ŀ����->������->����->��������������������� legacy_stdio_definitions.lib;
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


