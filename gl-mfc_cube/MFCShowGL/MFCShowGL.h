
// MFCShowGL.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CMFCShowGLApp:
// �йش����ʵ�֣������ MFCShowGL.cpp
//

class CMFCShowGLApp : public CWinApp
{
public:
	CMFCShowGLApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CMFCShowGLApp theApp;