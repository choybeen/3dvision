
// MFCShowGLDlg.h : 头文件
//

#pragma once
#pragma comment (lib, "freeglut.lib")
//#pragma comment (lib, "gltools.lib")
#pragma comment (lib, "opengl32.lib")
#pragma comment (lib, "glu32.lib")
#pragma comment (lib, "glut.lib")
#pragma comment (lib, "glaux.lib")

// CMFCShowGLDlg 对话框
class CMFCShowGLDlg : public CDialogEx
{
// 构造
public:
	CMFCShowGLDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_MFCSHOWGL_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持

    HDC hrenderDC;  //设备上下文
    HGLRC hrenderRC;  //渲染上下文
	float m_xPos, m_yPos, m_zPos;
	float m_xRotate, m_xacc;  //转速 加速
	float m_yRotate, m_yacc;  //转速 加速
    int PixelFormat;    //像素格式

	BOOL SetWindowPixelFormat(HDC hDC);  //设定像素格式
    BOOL CreateViewGLContext(HDC hDC);   //view GL Context
    void RenderScene();  //绘制场景
protected:
	HICON m_hIcon;
	int width, height;
	int wwin, hwin;
	Mat imgbg;
	CPoint ptpre, ptcnt;
	bool  bbtndown;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	CSliderCtrl m_SliderY;
	CSliderCtrl m_SliderX;
	afx_msg void OnReleasedcaptureSlider1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnReleasedcaptureSlider2(NMHDR *pNMHDR, LRESULT *pResult);
	BOOL m_bLighting;
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
};
