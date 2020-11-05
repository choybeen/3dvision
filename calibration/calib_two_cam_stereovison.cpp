#include "stdafx.h"
#include "funcdef.h"
#include "cameraApi.h"
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <thread>         // std::thread

using namespace cv;
using namespace std;
#define idscam
Mat fakeColor(Mat input)  //change depth image to fack color
{
	struct ParamColorMap {
		int iColormap = 2;
		Mat watershed_img;
	};
	Mat dst;
	applyColorMap(input, dst, 15);
	return dst;
}
void viz3dshow() 
{

}
void computer_match(Mat img1, Mat img2, Mat Q) {
	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
	int alg = STEREO_SGBM;
	int SADWindowSize = 31;
	int numberOfDisparities = 16;
	bool no_display = false;
	alg = STEREO_SGBM; // STEREO_HH; // STEREO_VAR: STEREO_3WAY;  //STEREO_BM;  // 
	Mat disp, disp8;
	Rect roi1, roi2;

	int64 t = getTickCount();
	float disparity_multiplier = 1.0f;
	if (alg == STEREO_BM)
	{
		Ptr<StereoBM> bm = StereoBM::create(16, 9);
		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterCap(31);
		bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		bm->setMinDisparity(0);
		bm->setNumDisparities(numberOfDisparities);
		bm->setTextureThreshold(10);
		bm->setUniquenessRatio(15);
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(1);
		bm->compute(img1, img2, disp);
		if (disp.type() == CV_16S)
			disparity_multiplier = 16.0f;
	}
	else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
	{
		Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
		sgbm->setPreFilterCap(63);
		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
		sgbm->setBlockSize(sgbmWinSize);
		int cn = img1.channels();
		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);
		if (alg == STEREO_HH)
			sgbm->setMode(StereoSGBM::MODE_HH);
		else if (alg == STEREO_SGBM)
			sgbm->setMode(StereoSGBM::MODE_SGBM);
		else if (alg == STEREO_3WAY)
			sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
		sgbm->compute(img1, img2, disp);
		if (disp.type() == CV_16S)
			disparity_multiplier = 16.0f;
		if (alg != STEREO_VAR)
			disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
		else
			disp.convertTo(disp8, CV_8U);
	}
	if (alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);
	Mat depthcolor = fakeColor(disp8);
	imshow("disparity", depthcolor);

	//transform depth image to 3D construction
	Mat xyz;
	Mat floatDisp;
	disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
	reprojectImageTo3D(floatDisp, xyz, Q, true);
	double minVal, maxVal;
	minMaxLoc(xyz, &minVal, &maxVal); //find minimum and maximum intensities
	Mat mask_map = xyz > minVal;
	Mat output_points, outcolor;
	xyz.copyTo(output_points, mask_map);
	depthcolor.copyTo(outcolor, mask_map);
}
void calibed_stereodepth_rectifyimage()
{
	// read intrinsic parameters
	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Mat cameraMatrix[2], distCoeffs[2];
	Mat rmap[2][2];
	Mat R1, R2, P1, P2, Q;   //computer distoration parameters, radial distoration k1/k2/k3 , tangential distortation: p1/p2
	Mat R, T, E, F;  // rotate, translate, Essential, Fundamental
	FileStorage fs("calib_stereo_allparams.yml", FileStorage::READ);
	if (fs.isOpened()) {
		fs["M1"] >> cameraMatrix[0]; 
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else
		return;

#ifdef idscam
	IdsCamera ids[2];
	ids[0].Ids_OpenCamera(0);
	ids[1].Ids_OpenCamera(1);
	int w = ids[0].m_dwWidth, h = ids[0].m_dwHeight;
#else
	VideoCapture cam[2];
	cam[0].open(0);
	cam[1].open(1);
	Mat img;
	cam[0] >> img;
	int w = img.cols, h = img.rows;
#endif
	int count = 0;

	const float maxScale = 0.5;
	Size imageSize = Size(w, h);
	Size boardSize = Size(9, 6);
	Mat imggray[2], imgcolor[2];
	Mat canvas, canvas2;
	canvas.create(h, w * 2, CV_8UC1);
	canvas2.create(h, w * 2, CV_8UC1);

	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	while (count < 10000) {
#ifdef idscam
		ids[0].Ids_GrabImage(imggray[0], imgcolor[0]);
		ids[1].Ids_GrabImage(imggray[1], imgcolor[1]);
#else
		cam[0] >> imgcolor[0];
		cam[1] >> imgcolor[1];
		cvtColor(imgcolor[0], imggray[0], COLOR_RGB2GRAY);
		cvtColor(imgcolor[1], imggray[1], COLOR_RGB2GRAY);
#endif
		Mat rimg1, rimg2;
		remap(imggray[0], rimg1, rmap[0][0], rmap[0][1], INTER_LINEAR);	//rectified image
		remap(imggray[1], rimg2, rmap[1][0], rmap[1][1], INTER_LINEAR);	//rectified image

		Mat canvasPart = canvas(Rect(0, 0, w, h));
		resize(rimg1, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		canvasPart = canvas(Rect(w, 0, w, h));
		resize(rimg2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		cv::imshow("1", canvas);

		Mat canvasPart2 = canvas2(Rect(0, 0, w, h));
		resize(imggray[0], canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		canvasPart2 = canvas2(Rect(w, 0, w, h));
		resize(imggray[1], canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		cv::imshow("2", canvas2);

		computer_match(rimg1, rimg2, Q);

		char key = (char)waitKey(30);
		if (key == 27 || key == 'q' || key == 'Q') // 'ESC'
			break;
	}
#ifdef idscam
	ids[0].Ids_CloseCamera();
	ids[1].Ids_CloseCamera();
#endif
}

//calibration for on cameras
void calib_stereodepth_calibration()  //2 cameras calibration
{
#ifdef idscam
	IdsCamera ids[2];
	ids[0].Ids_OpenCamera(0);
	ids[1].Ids_OpenCamera(1);
	int w = ids[0].m_dwWidth, h = ids[0].m_dwHeight;
#else
	VideoCapture cam[2];
	cam[0].open(0);
	cam[1].open(1);
	Mat img;
	cam[0] >> img;
	int w = img.cols, h = img.rows;
#endif
	int count = 0;

	const float maxScale = 0.5;
	vector<vector<Point2f> > imagePoints[2];
	Size imageSize = Size(w, h);
	Size boardSize = Size(9, 6);
	int i, j, k=0, nimages = 30;
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);

//step1  find corners on chessboard
	Mat imggray[2], imgcolor[2];
	Mat canvas;
	canvas.create(h, w * 2, CV_8UC3);
	while (count < nimages) {
#ifdef idscam
		ids[0].Ids_GrabImage(imggray[0], imgcolor[0]);
		ids[1].Ids_GrabImage(imggray[1], imgcolor[1]);
#else
		cam[0] >> imgcolor[0];
		cam[1] >> imgcolor[1];
		cvtColor(imgcolor[0], imggray[0], COLOR_RGB2GRAY);
		cvtColor(imgcolor[1], imggray[1], COLOR_RGB2GRAY);
#endif
		Mat timg1, timg2;
		resize(imggray[0], timg1, Size(), maxScale, maxScale, INTER_LINEAR_EXACT);
		resize(imggray[1], timg2, Size(), maxScale, maxScale, INTER_LINEAR_EXACT);

		vector<Point2f>& corners1 = imagePoints[0][count];
		vector<Point2f>& corners2 = imagePoints[1][count];
		bool found1 = findChessboardCorners(timg1, boardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		bool found2 = findChessboardCorners(timg2, boardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (found1 && found2)
		{
			Mat cornersMat1(corners1);			cornersMat1 *= 1. / maxScale;
			Mat cornersMat2(corners2);			cornersMat2 *= 1. / maxScale;
			cornerSubPix(imggray[0], corners1, Size(11, 11), Size(-1, -1),TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,30, 0.01));
			cornerSubPix(imggray[1], corners2, Size(11, 11), Size(-1, -1),TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,30, 0.01));
			drawChessboardCorners(imgcolor[0], boardSize, corners1, found1);
			drawChessboardCorners(imgcolor[1], boardSize, corners2, found2);
			count++;
			waitKey(0);
		}
		if (k++ > 1000) break;
		Mat canvasPart =canvas(Rect(0, 0, w, h));
		resize(imgcolor[0], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		canvasPart = canvas(Rect(w, 0, w, h));
		resize(imgcolor[1], canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		cv::imshow("1", canvas);
		waitKey(30);
	}
#ifdef idscam
	ids[0].Ids_CloseCamera();
	ids[1].Ids_CloseCamera();
#endif
	if (count < 10)	return;
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);

	vector<vector<Point3f> > objectPoints;
	objectPoints.resize(nimages);

//step2 calibrate stereo cameras
	float squareSize = 40.0f;
	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}
	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
	Mat R, T, E, F;  // rotate, translate, Essential, Fundamental

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], cameraMatrix[0], distCoeffs[0],cameraMatrix[1], distCoeffs[1],imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_USE_INTRINSIC_GUESS + CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;
	// CALIBRATION QUALITY CHECK // we can check the quality of calibration using the epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

//step3  rectify the intrinsic parameter matries
	Mat R1, R2, P1, P2, Q;   //computer distoration parameters, radial distoration k1/k2/k3 , tangential distortation: p1/p2
	Rect validRoi[2];             //matrix containing k1, k2, p1, p2, and k3 (in that order),  R1,R2:k1,k1   P1,P2:p1,p2    Q:k3
	stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

//step4   find fundamental matrix between 2 images by corresponding feature points
	vector<Point2f> allimgpt[2];
	for (k = 0; k < 2; k++)
	{
		for (i = 0; i < nimages; i++)
			std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
	}
	F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0); 
	Mat H1, H2;
	stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
	R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
	R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
	P1 = cameraMatrix[0];
	P2 = cameraMatrix[1];

//step5  Precompute maps for cv::remap()
	Mat rmap[2][2];
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

//step6  last step save all parameters
// save intrinsic parameters
	FileStorage fs("calib_stereo_allparams.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs << "R" << R << "T" << T;
		fs << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs << "mapx0" << rmap[0][0] << "mapy0" << rmap[0][1] << "mapx1" << rmap[1][0] << "mapy1" << rmap[1][1];
		fs.release();
	}
	else
		cout << "Error: can not save the stereoRectiry parameters\n";

	cout << "calibration finished" << endl;
}

void calibrate_two_cams()  //2 cameras calibration
{
	//1)  calib_stereodepth_calibration : for calibration
	//std::thread thread_calibrating(calib_stereodepth_calibration);  
	//2)  calibed_stereodepth_rectifyimage  :  for result showing
	std::thread thread_calibrating(calibed_stereodepth_rectifyimage);    
	thread_calibrating.join();                // start thread
}