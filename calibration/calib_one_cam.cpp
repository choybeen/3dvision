#include "stdafx.h"
#include "funcdef.h"
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>       // std::cout
#include <thread>         // std::thread

using namespace cv;
using namespace std;

bool bCalibrated = false;
int m_bCamCalibChecked = true;
inline int drawCorssMark(IplImage *dst, CvPoint pt);
void InitCorners3D(vector<vector<Point3f>> &obj_orners, Size ChessBoardSize, int NImages, float SquareSize);
int myFindChessboardCorners(const void* image, CvSize pattern_size, CvPoint2D32f* corners, int* corner_count, int flags);
void Read_camera_params(const string& in_filename, int &image_count, Size &img_size, Size &board_size, float &square_size, float &aspect_ratio, int &flags,
		cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& transMatrix, vector<Mat>& rotation_vectors, vector<Mat>& translation_vectors, vector<float>& reprojErrs, vector<vector<Point2f> >& imagePoints, double& totalAvgErr);
static void saveCameraParams(const string& filename, int image_count, Size imageSize, Size boardSize, float squareSize, float aspectRatio, int flags, const Mat& cameraMatrix,
	const Mat& distCoeffs, const Mat& transMatrix, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints, double totalAvgErr);
static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors);

inline int drawCorssMark(IplImage *dst, CvPoint pt)
{
	const int cross_len = 4;
	CvPoint pt1, pt2, pt3, pt4;
	pt1.x = pt.x;
	pt1.y = pt.y - cross_len;
	pt2.x = pt.x;
	pt2.y = pt.y + cross_len;
	pt3.x = pt.x - cross_len;
	pt3.y = pt.y;
	pt4.x = pt.x + cross_len;
	pt4.y = pt.y;
	cvLine(dst, pt1, pt2, cvScalar(0, 255, 0), 2, CV_AA, 0);
	cvLine(dst, pt3, pt4, cvScalar(0, 255, 0), 2, CV_AA, 0);
	return 0;
}
int myFindChessboardCorners(const void* image, CvSize pattern_size,
	CvPoint2D32f* corners, int* corner_count, int flags)
{
	IplImage* eig = cvCreateImage(cvGetSize(image), 32, 1);
	IplImage* temp = cvCreateImage(cvGetSize(image), 32, 1);
	double quality = 0.01;
	double min_distance = 5;
	int win_size = 10;
	int count = pattern_size.width * pattern_size.height;
	cvGoodFeaturesToTrack(image, eig, temp, corners, &count, quality, min_distance, 0, 3, 0, 0.04);
	cvFindCornerSubPix(image, corners, count, cvSize(win_size, win_size), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
	cvReleaseImage(&eig);
	cvReleaseImage(&temp);
	return 1;
}
void InitCorners3D(vector<vector<Point3f>> &obj_orners, Size ChessBoardSize, int NImages, float SquareSize)
{
	int CurrentImage = 0;
	int CurrentRow = 0;
	int CurrentColumn = 0;
	int NPoints = ChessBoardSize.height*ChessBoardSize.width;
	// for now, assuming we're row-scanning
	vector<Point3f> Corners3D;
	for (CurrentRow = 0; CurrentRow < ChessBoardSize.height; CurrentRow++)
	{
		for (CurrentColumn = 0; CurrentColumn < ChessBoardSize.width; CurrentColumn++)
		{
			Point3f temppoints((float)CurrentRow*SquareSize, (float)CurrentColumn*SquareSize, 0.f);
			Corners3D.push_back(temppoints);
		}
	}
	for (CurrentImage = 0; CurrentImage < NImages; CurrentImage++)
	{
		obj_orners.push_back(Corners3D);
	}
}

void Read_camera_params(const string& in_filename, int &image_count, Size &img_size, Size &board_size, float &square_size, float &aspect_ratio, int &flags,
	cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& transMatrix, vector<Mat>& rotation_vectors,vector<Mat>& translation_vectors, vector<float>& reprojErrs, vector<vector<Point2f> >& imagePoints, double& totalAvgErr)
{
	double avg_reproj_err;
	FileStorage fs;
	if (fs.open(in_filename, FileStorage::READ) == false) return;
	fs["image_count"] >> image_count;
	fs["image_width"] >> img_size.width;
	fs["image_height"] >> img_size.height;
	fs["board_width"] >> board_size.width;
	fs["board_height"] >> board_size.height;
	fs["square_size"] >> square_size;
	fs["aspectRatio"] >> aspect_ratio;
	fs["flags"] >> flags;

	fs["camera_matrix"] >> camera_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;
	fs["translation_coefficients"] >> transMatrix;
	fs["totalAvgErr"] >> totalAvgErr;
	if (!fs["per_view_reprojection_errors"].empty()) {
//		fs["per_view_reprojection_errors"] >> reprojErrs;
	}
	if (!fs["extrinsic_parameters"].empty())	{
		Mat bigmat;
		fs["extrinsic_parameters"] >> bigmat;
		rotation_vectors.push_back(bigmat.colRange(0,3));
		translation_vectors.push_back(bigmat.colRange(3,6));
 	}
	bCalibrated = true;
}
static void saveCameraParams(const string& filename, int image_count, Size imageSize, Size boardSize,float squareSize, float aspectRatio, int flags,const Mat& cameraMatrix,
	const Mat& distCoeffs, const Mat& transMatrix, const vector<Mat>& rvecs, const vector<Mat>& tvecs,const vector<float>& reprojErrs,const vector<vector<Point2f> >& imagePoints,double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);
	fs << "calibration_time" << buf;

	fs << "image_count" << image_count;
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;
	fs << "aspectRatio" << aspectRatio;

	if (flags != 0){
		std::sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;
	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "translation_coefficients" << transMatrix;
	fs << "totalAvgErr" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));
			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}


static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

//1) bshowcalib=false, for calibrate    
//2)bshowcalib=false,  for show result, read map matrix from file
int thread_calibrating_run()   //calibration with chessboard(9x6) for on camera, result is intrinsic matrix and distortion
{
	const char* calibor_filename = "calibdata_1cam.yml";
	bool bshowcalib = true;    // show rectify frames from previous calibration result data
	const int chessw = 9;
	const int chessh = 6;
	Size  ChessBoardSize = cvSize(chessw, chessh);
	float SquareWidth = 160.0f; //投影实际距离 毫米单位  200
	float aspect_ratio = 1.0f;
	int flags = CV_CALIB_CB_ADAPTIVE_THRESH;
	const   int NPoints = chessw*chessh;
	const   int NImages = 30; //Number of images to collect
	Size imageSize(0,0);
	//CvPoint2D32f corners[NPoints*NImages];
	vector<int> corner_count;
	vector<Point2f> corners;
	vector<vector<Point2f> > image_points;
	int write_extrinsics = 0, write_points = 0;
	int framecount = 0;
	int find_corners_result = 0;
	Mat frame, image, gray, maskFound;
	Mat current_frame_hsv;
	Mat camera_intrinsics = Mat::eye(3, 3, CV_64F);
	camera_intrinsics.at<double>(0, 0) = aspect_ratio;   //fx		
	Mat distortion_coeffs = Mat::zeros(8, 1, CV_64F);
	Mat translation_vectors_better;
	vector<Mat> rotation_vectors, translation_vectors;
	vector<float> reprojErrs;
	double totalAvgErr = 0;
	vector<vector<Point3f> > object_points;
	InitCorners3D(object_points, ChessBoardSize, NImages, SquareWidth);  // Function to fill in the real-world points of the checkerboard

	Mat rectifymap[2];

	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);
	cap.open(0);
	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}
	cap >> frame;
	imageSize=frame.size();;
	maskFound = Mat::zeros(imageSize, image.type());
//	rectifymap[0] = Mat::zeros(imageSize, CV_16SC1);
//	rectifymap[1] = Mat::zeros(imageSize, CV_16SC1);
	if (bshowcalib) {   //show result, read camera parameters from file
		Size calibSize(0, 0);
		Read_camera_params(calibor_filename, framecount, calibSize, ChessBoardSize, SquareWidth, aspect_ratio, flags, camera_intrinsics, distortion_coeffs,
			translation_vectors_better, rotation_vectors, translation_vectors, reprojErrs, image_points, totalAvgErr);
		Mat rvec = rotation_vectors[0];
		//computer the map matrix fro image rectify in advance
		initUndistortRectifyMap(camera_intrinsics, distortion_coeffs, Mat(), translation_vectors_better, imageSize, CV_16SC2, rectifymap[0], rectifymap[1]);
	}
	/*
	camera_intrinsics.at<double>(0, 0) = 3161.7478262f;   //fx		
	camera_intrinsics.at<double>(0, 2) = 390.2826538f;   //cx
	camera_intrinsics.at<double>(1, 1) = 3265.8450139f;   //fy
	camera_intrinsics.at<double>(1, 2) = 295.6264572f;   //cy

	distortion_coeffs.at<double>(0, 0) = -0.193740f;  //k1
	distortion_coeffs.at<double>(1, 0) = -0.378588f;  //k2
	distortion_coeffs.at<double>(2, 0) = 0.028980f;   //p1
	distortion_coeffs.at<double>(3, 0) = 0.008136f;   //p2
	*/
	long ltstart, ltend;
	ltstart = cvGetTickCount();
	int i, j, k, m;
	while (m_bCamCalibChecked)
	{
		cap >> frame;
		if (frame.empty())
		{
			ltend = cvGetTickCount() - ltstart;
			waitKey(500);
			continue;
		}
		frame.copyTo(image);
		if(bshowcalib || bCalibrated){
			//cvUndistort2(pImgColor, pColorCpy, intrinsics, distortion_coeff);
			Mat temp = image.clone();
			//undistort(temp, image, camera_intrinsics, distortion_coeffs);
			remap(temp, image, rectifymap[0], rectifymap[1], INTER_LINEAR);
			imshow("rectified", image);
		}
	    else // if( framecount < NImages)
		{
			//	cvNamedWindow("2",0); cvShowImage("2",pImgColor); cvWaitKey(10);
			IplImage img = cvIplImage(image);
			//find_corners_result = cvFindChessboardCorners(pImgColor, ChessBoardSize, &corners[framecount*NPoints], &corner_count[framecount], 0);
			find_corners_result = findChessboardCorners(image, ChessBoardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			//cvDrawChessboardCorners(pImgColor, ChessBoardSize, &corners[framecount*NPoints], NPoints, find_corners_result);
			if (find_corners_result && corners.size() > 0)
			{
				corner_count.push_back(corners.size()*1.0f);
				image_points.push_back(corners);
				drawChessboardCorners(image, ChessBoardSize, Mat(corners), find_corners_result);
				for (i = 0; i < NPoints; i++)
				{
					circle(maskFound, corners[i], chessw, Scalar(255, 255, 255), 0);	//	cvCircle(pMaskFound, cvPoint(corners[framecount*NPoints + i].x, corners[framecount*NPoints + i].y), chessw, cvScalar(255,255,255), 0);
					//image.setTo(Scalar(0, 255, 0), maskFound);//cvSet(pImgColor, cvScalar(0,255,0), pMaskFound);
				}
				if (find_corners_result == 1)
				{
					framecount++;
					waitKey(450);
				}
			}
			imshow("calibrating", image);
			imshow("mask", maskFound);
			ltend = (cvGetTickCount() - ltstart)/1000;
		    waitKey(50);
			ltstart = cvGetTickCount();
			find_corners_result = 0;
		} //第二个while结束   
		if(!m_bCamCalibChecked)
			break;
		if (bCalibrated == false) {
			if (framecount == NImages) {
				cout << "calibrating...." << endl;
				//cvCalibrateCamera2(object_points, image_points, point_counts, sizeImage, intrinsics,distortion_coeff, rotation_vectors, translation_vectors, CV_CALIB_FIX_PRINCIPAL_POINT /*&& CV_CALIB_ZERO_TANGENT_DIST &&*/ /*CV_CALIB_USE_INTRINSIC_GUESS*/);
				double rms = calibrateCamera(object_points, image_points, imageSize, camera_intrinsics, distortion_coeffs, rotation_vectors, translation_vectors, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
				totalAvgErr = computeReprojectionErrors(object_points, image_points, rotation_vectors, translation_vectors, camera_intrinsics, distortion_coeffs, reprojErrs);
				bCalibrated = true;
				translation_vectors_better = getOptimalNewCameraMatrix(camera_intrinsics, distortion_coeffs, imageSize, 1, imageSize, 0);
				cout << "calibrated, saving data...." << endl;
				saveCameraParams(calibor_filename, framecount, imageSize, ChessBoardSize, SquareWidth, aspect_ratio, flags, camera_intrinsics, distortion_coeffs,
					translation_vectors_better, rotation_vectors, translation_vectors, reprojErrs, image_points, totalAvgErr);
				//Precompute maps for cv::remap()
				initUndistortRectifyMap(camera_intrinsics, distortion_coeffs, Mat(), translation_vectors_better, imageSize, CV_16SC2, rectifymap[0], rectifymap[1]);
			}
		}
		char key = (char)waitKey(30);
		if (key == 27 || key == 'q' || key == 'Q') // 'ESC'
			break;
	}
	return 1;
}
//calibration for on camera
void calibrate_one_cam()  //1 camera
{
	std::thread thread_calibrating(thread_calibrating_run);     // spawn new thread that calls calibration()
	thread_calibrating.join();                // pauses until first finishes
//	thread_calibrating_run();
}