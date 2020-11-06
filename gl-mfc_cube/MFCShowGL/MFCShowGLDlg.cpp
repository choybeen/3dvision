
// MFCShowGLDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "MFCShowGL.h"
#include "MFCShowGLDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



// CMFCShowGLDlg �Ի���



CMFCShowGLDlg::CMFCShowGLDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CMFCShowGLDlg::IDD, pParent)
	, m_bLighting(TRUE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_xRotate = 0;
	m_yRotate = 0;
	m_xacc = 0.3;
	m_yacc = 0.1;
	m_xPos = -2.0f;
	m_yPos = -2.0f;
	m_zPos = -15.0f;  
	ptpre = CPoint(0, 0);
	ptcnt = CPoint(0, 0);
	bbtndown = false;  //left button pressed
}

void CMFCShowGLDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SLIDER1, m_SliderY);
	DDX_Control(pDX, IDC_SLIDER2, m_SliderX);
	DDX_Check(pDX, IDC_CHECK1, m_bLighting);
}

BEGIN_MESSAGE_MAP(CMFCShowGLDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_MOUSEMOVE()
	ON_NOTIFY(NM_RELEASEDCAPTURE, IDC_SLIDER1, &CMFCShowGLDlg::OnReleasedcaptureSlider1)
	ON_NOTIFY(NM_RELEASEDCAPTURE, IDC_SLIDER2, &CMFCShowGLDlg::OnReleasedcaptureSlider2)
	ON_BN_CLICKED(IDC_CHECK1, &CMFCShowGLDlg::OnBnClickedCheck1)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
END_MESSAGE_MAP()


BOOL CMFCShowGLDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	SetIcon(m_hIcon, TRUE);
	SetIcon(m_hIcon, FALSE);

	///////////////////////OPENGL INIT///////////////////////// 
	CWnd *wnd = GetDlgItem(IDC_RENDER);
	hrenderDC = ::GetDC(wnd->m_hWnd);
	if (SetWindowPixelFormat(hrenderDC) == FALSE)
		return 0;
	CRect rtwin;
	wnd->GetClientRect(&rtwin);
	wwin = rtwin.Width();
	hwin = rtwin.Height();

	if (CreateViewGLContext(hrenderDC) == FALSE)
		return 0;
	imgbg = cv::imread("bgimage.jpg");

	m_SliderX.SetRange(0, 30);
	m_SliderY.SetRange(0, 30);
	m_SliderX.SetPos(3);
	m_SliderY.SetPos(1);

	//glPolygonMode(GL_FRONT,GL_FILL); 
   // glPolygonMode(GL_BACK,GL_FILL); 
	/////////////////////////////////////////// 
	width = 1280;
	height = 1024;
	glEnable(GL_TEXTURE_2D);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, 1, 0.1, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glShadeModel(GL_SMOOTH);       // Enable Smooth Shading 
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);    // Black Background 
	glClearDepth(1.0f);         // Depth Buffer Setup 
	glEnable(GL_DEPTH_TEST);       // Enables Depth Testing 
	glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do 
	///////////////////////////////////////////////////////////////////////// 
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	SetTimer(1, 10, 0);

	////////////////////////////////////////////////////////////////
	return TRUE;
}
void CMFCShowGLDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	CDialogEx::OnSysCommand(nID, lParam);
}
void CMFCShowGLDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this);
		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
		CDialogEx::OnPaint();
}
HCURSOR CMFCShowGLDlg::OnQueryDragIcon(){
	return static_cast<HCURSOR>(m_hIcon);
}

void CMFCShowGLDlg::OnTimer(UINT_PTR nIDEvent)
{
    RenderScene(); 
	m_xRotate += m_xacc;
	m_yRotate += m_yacc;
	CDialogEx::OnTimer(nIDEvent);
}
BOOL CMFCShowGLDlg::CreateViewGLContext(HDC hDC) 
{ 
	hrenderRC = wglCreateContext(hDC); 
	if(hrenderRC==NULL) 
	   return FALSE; 
	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) 
	   return FALSE;   
	return TRUE; 
}
BOOL CMFCShowGLDlg::SetWindowPixelFormat(HDC hDC) 
{ 
	PIXELFORMATDESCRIPTOR pixelDesc; 
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
	pixelDesc.nVersion = 1; 
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW |  
		 PFD_SUPPORT_OPENGL | 
		 PFD_DOUBLEBUFFER | 
		 PFD_TYPE_RGBA; 
	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
	pixelDesc.cColorBits = 32; 
	pixelDesc.cRedBits = 0; 
	pixelDesc.cRedShift = 0; 
	pixelDesc.cGreenBits = 0; 
	pixelDesc.cGreenShift = 0; 
	pixelDesc.cBlueBits = 0; 
	pixelDesc.cBlueShift = 0; 
	pixelDesc.cAlphaBits = 0; 
	pixelDesc.cAlphaShift = 0; 
	pixelDesc.cAccumBits = 0; 
	pixelDesc.cAccumRedBits = 0; 
	pixelDesc.cAccumGreenBits = 0; 
	pixelDesc.cAccumBlueBits = 0; 
	pixelDesc.cAccumAlphaBits = 0; 
	pixelDesc.cDepthBits = 0; 
	pixelDesc.cStencilBits = 1; 
	pixelDesc.cAuxBuffers = 0; 
	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
	pixelDesc.bReserved = 0; 
	pixelDesc.dwLayerMask = 0; 
	pixelDesc.dwVisibleMask = 0; 
	pixelDesc.dwDamageMask = 0; 

	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc); 
	if(PixelFormat==0) // Choose default 
	{ 
	    PixelFormat = 1; 
	    if(DescribePixelFormat(hDC,PixelFormat,  sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0) 
	        return FALSE; 
	} 

	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc)==FALSE) 
	{  
	   return FALSE; 
	} 
	glEnable(GL_TEXTURE_2D);						// ��������ӳ��
	glShadeModel(GL_SMOOTH);						// ������Ӱƽ��
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);					// ��ɫ����
	glClearDepth(1.0f);							// ������Ȼ���
	glEnable(GL_DEPTH_TEST);						// ������Ȳ���
	glDepthFunc(GL_LEQUAL);							// ������Ȳ��Ե�����
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);			// ������ϸ��͸������

	GLfloat LightAmbient[] = { 0.8f, 0.8f, 0.8f, 1.0f }; 				// ���������
	GLfloat LightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };				 // ��������
	GLfloat LightPosition[] = { 0.0f, 10.0f, 2.0f, 1.0f };				 // ��Դλ��
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);				// ���û�����
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);				// ���������
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);			// ���ù�Դλ��
	glEnable(GL_LIGHT1);							// ����һ�Ź�Դ

	glBlendFunc(GL_SRC_ALPHA, GL_ONE);		// ����Դ����alphaͨ��ֵ�İ�͸����Ϻ���
	glEnable(GL_BLEND);					// ���û�ɫ

	glMatrixMode(GL_PROJECTION);						// ѡ��ͶӰ����
	gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);						// ѡ��ģ�͹۲����

	return TRUE; 
}
void CMFCShowGLDlg::RenderScene()    
{ 
 ///////////////////////////////////////////////// 
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
  
 glLoadIdentity(); 
 glTranslatef(m_xPos, m_yPos,-15.0f);      // Move Left 1.5 Units And Into The Screen 6.0 
 glEnable(GL_TEXTURE_2D);						// ��������ӳ��
 glColor4f(1.0f, 1.0f, 1.0f, 1.0f);			// ȫ���ȣ� 50% Alpha ���
 glBlendFunc(GL_SRC_ALPHA, GL_ONE);		// ����Դ����alphaͨ��ֵ�İ�͸����Ϻ���
 glEnable(GL_BLEND);					// ���û�ɫ
 glRotated(m_xRotate, 1.0, 0.0, 0.0);
 glRotated(m_yRotate, 0.0, 1.0, 0.0);

 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// �����˲�
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	// �����˲�
 glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgbg.cols, imgbg.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, imgbg.data);
 glBegin(GL_TRIANGLES); // Drawing Using Triangles 
 glTexCoord2f(0.0f, 0.0f);  glVertex3f( 0.0f, 1.0f, 0.0f);     // Top 
 glTexCoord2f(1.0f, 0.0f);  glVertex3f(-1.0f,-1.0f, 0.0f);     // Bottom Left 
 glTexCoord2f(1.0f, 1.0f);  glVertex3f( 1.0f,-1.0f, 0.0f);     // Bottom Right 
 glEnd();						// ���������ƽ���

 glColor4f(1.0f, 1.0f, 1.0f, 1.0f);		// ������ɫ	// ȫ���ȣ� 50% Alpha ���

 glBegin(GL_QUADS);							// �����ı���
  //	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, imgbg.cols, imgbg.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, imgbg.data);
 // ǰ����
	glNormal3f(0.0f, 0.0f, 1.0f);					// ����ָ��۲���
	glColor3f(1.0f, 0.5f, 0.0f);			// ��ɫ�ĳɳ�ɫ
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 1.0f);			// ��ɫ�ĳ���ɫ
	glVertex3f(1.0f, -1.0f, 1.0f);
	glColor3f(0.0f, 1.0f, 0.0f);			// ��ɫ�ĳ���ɫ
	glVertex3f(1.0f, 1.0f, 1.0f);
	glColor3f(0.0f, 1.0f, 1.0f);			// ��ɫ�ĳ�����ɫ
	glVertex3f(-1.0f, 1.0f, 1.0f);
	// �����
	glNormal3f(0.0f, 0.0f, -1.0f);					// ���߱���۲���
	glColor3f(0.0f, 0.0f, 1.0f);			// ��ɫ�ĳ���ɫ
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	// ����
	glNormal3f(0.0f, 1.0f, 0.0f);					// ��������
	glColor3f(1.0f, 0.0f, 0.0f);			// ��ɫ�ĳɺ�ɫ
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	// ����
	glNormal3f(0.0f, -1.0f, 0.0f);					// ���߳���
	glColor3f(0.0f, 1.0f, 0.0f);			// ��ɫ�ĳ���ɫ
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	// �Ҳ���
	glNormal3f(1.0f, 0.0f, 0.0f);					// ���߳���
	glColor3f(1.0f, 1.0f, 0.0f);			// ��ɫ�ĳɻ�ɫ
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	// �����
	glColor3f(1.0f, 1.0f, 1.0f);			// ������ɫ
	glNormal3f(-1.0f, 0.0f, 0.0f);					// ���߳���
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// ������ı��ε�����
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, 1.0f);	// ������ı��ε�����
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, 1.0f, 1.0f);	// ������ı��ε�����
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, 1.0f, -1.0f);	// ������ı��ε�����
	glEnd();           // Finished Drawing The Triangle 

 SwapBuffers(hrenderDC); 
}


void CMFCShowGLDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	if (!bbtndown)  //only process when left button pressed
		return;
	if (point.x<1 || point.x>wwin - 1 || point.y<1 || point.y>hwin - 1) {
		bbtndown = false;
		return;
	}
	int diffx = point.x - wwin/2;
	int diffy = point.y - hwin/2;
	m_xPos = -2.0f + diffx/100.0;
	m_yPos = -2.0f - diffy/100.0;
	ptpre = point;
	CDialogEx::OnMouseMove(nFlags, point);
}

void CMFCShowGLDlg::OnReleasedcaptureSlider1(NMHDR *pNMHDR, LRESULT *pResult)
{
	int value = m_SliderY.GetPos();
	m_yacc = value / 10.0;
	*pResult = 0;
}


void CMFCShowGLDlg::OnReleasedcaptureSlider2(NMHDR *pNMHDR, LRESULT *pResult)
{
	int value = m_SliderX.GetPos();
	m_xacc = value / 10.0;
	*pResult = 0;
}


void CMFCShowGLDlg::OnBnClickedCheck1()
{
	if(m_bLighting)
	    glEnable(GL_LIGHTING);		// ���ù�Դ
	else
	    glDisable(GL_LIGHTING);		// ���ù�Դ
	UpdateData();
}


void CMFCShowGLDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	bbtndown = true;
	CDialogEx::OnLButtonDown(nFlags, point);
}


void CMFCShowGLDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	bbtndown = false;
	CDialogEx::OnLButtonUp(nFlags, point);
}
