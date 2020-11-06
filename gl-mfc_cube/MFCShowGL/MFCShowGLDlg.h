
// MFCShowGLDlg.h : ͷ�ļ�
//

#pragma once
#pragma comment (lib, "freeglut.lib")
//#pragma comment (lib, "gltools.lib")
#pragma comment (lib, "opengl32.lib")
#pragma comment (lib, "glu32.lib")
#pragma comment (lib, "glut.lib")
#pragma comment (lib, "glaux.lib")

// CMFCShowGLDlg �Ի���
class CMFCShowGLDlg : public CDialogEx
{
// ����
public:
	CMFCShowGLDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_MFCSHOWGL_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��

    HDC hrenderDC;  //�豸������
    HGLRC hrenderRC;  //��Ⱦ������
	float m_xPos, m_yPos, m_zPos;
	float m_xRotate, m_xacc;  //ת�� ����
	float m_yRotate, m_yacc;  //ת�� ����
    int PixelFormat;    //���ظ�ʽ

	BOOL SetWindowPixelFormat(HDC hDC);  //�趨���ظ�ʽ
    BOOL CreateViewGLContext(HDC hDC);   //view GL Context
    void RenderScene();  //���Ƴ���
protected:
	HICON m_hIcon;
	int width, height;
	int wwin, hwin;
	Mat imgbg;
	CPoint ptpre, ptcnt;
	bool  bbtndown;

	// ���ɵ���Ϣӳ�亯��
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
