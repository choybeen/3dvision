#include <stdio.h>
#include "stdafx.h"
#include <iostream>
#include "funcdef.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/ximgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

int cube3dAR_video_stable()
{
	const int bestmatch_max_num = 30;
	// 1, Read the camera calibration parameters
	cv::Mat cameraMatrix;
	cv::Mat cameraDistCoeffs;
	cv::FileStorage fs("calibration_intrinsics.yml", cv::FileStorage::READ);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> cameraDistCoeffs;
	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
	std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
	std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl << std::endl;
	cv::Matx33d cMatrix(cameraMatrix);

   // 2  init image and points
	std::string pathname1 = "images/bookfacemodel.jpg"; //"E:/MyProjects/OpenCV/data/win1.bmp"
	std::string pathname2 = "images/bookface2.JPG"; //E:/MyProjects/OpenCV/data/win3.bmp
	cv::Mat imagegray1 = cv::imread(pathname1, 0);
	cv::Mat imagegray2 = cv::imread(pathname2, 0);
	cv::Mat imagecolor1 = cv::imread(pathname1);
	cv::Mat imagecolor2 = cv::imread(pathname2);
	if (!imagegray1.data || !imagegray2.data)
		return 0;
	VideoCapture cap;
	cap.open(0);
	//cap.open("E:/MyProjects/testAV/bookface.mp4");

	// Input image points
	std::vector<cv::Point2f>  points2d;
	std::vector<cv::Point3f>  points3d;
	// Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
	points2d.resize(4);
	points3d.resize(4);

	// Image dimensions
	const float w = imagegray1.cols, h = imagegray1.rows;

	// Normalized dimensions:
	const float maxSize = std::max(w, h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;

	points2d[0] = cv::Point2f(w / 4, h / 4);
	points2d[1] = cv::Point2f(w / 2, h / 4);
	points2d[2] = cv::Point2f(w / 2, h / 2);
	points2d[3] = cv::Point2f(w / 4, h / 2);

	points3d[0] = cv::Point3f(-unitW, unitH, 0);
	points3d[1] = cv::Point3f(unitW, unitH, 0);
	points3d[2] = cv::Point3f(unitW, -unitH, 0);
	points3d[3] = cv::Point3f(-unitW, -unitH, 0);

	//3  detect features and matching
	// vector of keypoints and descriptors
	std::vector<cv::KeyPoint> trainKeypoints;
	std::vector<cv::KeyPoint> queryKeypoints;
	cv::Mat descriptors1, descriptors2;

	// Construction of the SIFT feature detector 
//	cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SIFT::create(500);
	cv::Ptr<cv::Feature2D> ptrFeature2D = BRISK::create();  // cv::ORB::create();
	ptrFeature2D->detectAndCompute(imagegray1, cv::noArray(), trainKeypoints, descriptors1);
		std::vector<cv::Mat> descriptors(1);
		cv::Ptr<cv::BFMatcher>   m_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
		descriptors[0] = descriptors1.clone();
		m_matcher->add(descriptors);
		// After adding train data perform actual train:
		m_matcher->train();

	Mat m_warpedImg;
	for (;;)
	{
		const char key = (char)cv::waitKey(10);
		if (key == 27 || key == 'q') // ESC
			break;
		// prepare input frame
		cap >> imagecolor2;
		cv::cvtColor(imagecolor2, imagegray2, cv::COLOR_RGB2GRAY);
		Mat cubearimg = imagecolor2.clone();
		cv::Size szimg(imagecolor2.size());
		//cv::imshow("arv", imagecolor2);
		// Detection of the SIFT/ORB features and associated descriptors
		ptrFeature2D->detectAndCompute(imagegray2, cv::noArray(), queryKeypoints, descriptors2);

		// Match the two image descriptors
		// Construction of the matcher with crosscheck 
		std::vector<cv::DMatch>   matches_ret;
		m_matcher->match(descriptors2, matches_ret);
		//drawMatches(imagecolor1, trainKeypoints, imagecolor2, queryKeypoints, matches_ret, cubearimg);
		//cv::imshow("matching", cubearimg);

		const int minNumberMatchesAllowed = 8;
		if (matches_ret.size() < minNumberMatchesAllowed) {
			cv::imshow("ar", cubearimg);
			continue;
		}
		// Prepare data for cv::findHomography
		std::vector<cv::Point2f> srcPoints(matches_ret.size());
		std::vector<cv::Point2f> dstPoints(matches_ret.size());
		for (size_t i = 0; i < matches_ret.size(); i++)
		{
			srcPoints[i] = trainKeypoints[matches_ret[i].trainIdx].pt;
			dstPoints[i] = queryKeypoints[matches_ret[i].queryIdx].pt;
		}

		// Find homography transformation and detect good matches
		int reprojectionThreshold = 3;
		std::vector<cv::Point2f>  points2dwarped;
		points2dwarped.resize(4);
		std::vector<unsigned char> inliersMask(srcPoints.size());
		cv::Mat homography, m_roughHomography, m_refinedHomography;
		std::vector<cv::DMatch> refinedMatches;
		std::vector<cv::KeyPoint> warpedKeypoints;
		m_roughHomography = cv::findHomography(srcPoints, dstPoints,
			                                                   cv::FM_RANSAC, reprojectionThreshold, inliersMask);
		std::vector<cv::DMatch> inliers1;
		for (size_t i = 0; i < inliersMask.size(); i++)
		{
			if (inliersMask[i])
				inliers1.push_back(matches_ret[i]);
		}
		matches_ret.swap(inliers1);
		bool homographyFound = (matches_ret.size() > minNumberMatchesAllowed);
		if (homographyFound) {
			// Warp image using found homography
			cv::warpPerspective(imagegray2, m_warpedImg, m_roughHomography, szimg, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
			// Get refined matches:
			// Detect features on warped image
			ptrFeature2D->detectAndCompute(m_warpedImg, cv::noArray(), warpedKeypoints, descriptors2);
			// Match with pattern
			m_matcher->match(descriptors2, refinedMatches);
			std::vector<cv::Point2f> srcPoints(refinedMatches.size());
			std::vector<cv::Point2f> dstPoints(refinedMatches.size());
			for (size_t i = 0; i < refinedMatches.size(); i++)
			{
				srcPoints[i] = trainKeypoints[refinedMatches[i].trainIdx].pt;
				dstPoints[i] = warpedKeypoints[refinedMatches[i].queryIdx].pt;
			}
			// Estimate new refinement homography
			m_refinedHomography = cv::findHomography(srcPoints, dstPoints,
				                                                      cv::FM_RANSAC, reprojectionThreshold, inliersMask);
			std::vector<cv::DMatch> inliers2;
			for (size_t i = 0; i < inliersMask.size(); i++)
			{
				if (inliersMask[i])
					inliers2.push_back(refinedMatches[i]);
			}
			refinedMatches.swap(inliers2);
			homographyFound = (refinedMatches.size() > minNumberMatchesAllowed);
			homography = m_roughHomography * m_refinedHomography;
			// Transform contour with precise homography
			cv::perspectiveTransform(points2d, points2dwarped, homography);
		}
		else {
			homography = m_roughHomography;
			// Transform contour with rough homography
			cv::perspectiveTransform(points2d, points2dwarped, homography);
		}

		//prepare for show matching points with lines
		if (homographyFound == false) {
			waitKey(90);
			continue;
		}

		//5  get rotate and transplate matrix
		cv::Mat Rvec;
		cv::Mat_<float> Tvec;
		cv::Mat rvec, tvec;
		cv::solvePnP(points3d, points2dwarped, cameraMatrix, cameraDistCoeffs, rvec, tvec, false);
		cv::Mat rotation;
		cv::Rodrigues(rvec, rotation);// 转换成三维旋转矩阵

		// Copy to transformation matrix
		//cv::Affine3d pose(rotMat, tvec);
		vector<Point3f> axis_cube = { {0, 0, 2}, {0, 2, 2}, {2, 2, 2}, {2, 0, 2}, {0, 0, -2}, {0, 2, -2}, {2, 2, -2}, {2, 0, -2}, {0, 0, -2} };
		vector<Point2f> projectedpoints;
		cv::projectPoints(axis_cube, rotation, tvec, cameraMatrix, cameraDistCoeffs, projectedpoints);

		for (size_t i = 0; i < 4; i++) {
			if (i < 3)
				cv::line(cubearimg, projectedpoints[i], projectedpoints[(i + 1) % projectedpoints.size()], cv::Scalar(0, 255, 0), 2);
			else
				cv::line(cubearimg, projectedpoints[i], projectedpoints[0], cv::Scalar(0, 255, 0), 2);
			cv::line(cubearimg, projectedpoints[i], projectedpoints[(i + 4) % projectedpoints.size()], cv::Scalar(0, 255, 0), 2);
			cv::line(cubearimg, projectedpoints[i + 4], projectedpoints[(i + 5) % projectedpoints.size()], cv::Scalar(0, 255, 255), 5);
		}
		cv::imshow("ar", cubearimg);
	}
	cv::waitKey(0);
	cv::destroyAllWindows();
	return 0;
}