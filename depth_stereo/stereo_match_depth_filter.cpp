#include "stdafx.h"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include "opencv2/viz.hpp"
#include <opencv2/viz/widget_accessor.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/cudastereo.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <string>


using namespace std;
using namespace cv;
using namespace cv::ximgproc;


int viz3dshowdepth(viz::Viz3d myWindow, Mat imggray, Mat imgcolor, Mat disparityMap, Mat pointcloud)
{
	// Compute the point cloud
	Mat pointcloud_tresh, color_tresh;

	// To better visualize the result, apply a colormap to the computed disparity
	double min;
	double max;
	minMaxIdx(disparityMap, &min, &max);
	Mat cm_disp, scaledDisparityMap;
	//cout << "disp min " << min << endl << "disp max " << max << endl;
	convertScaleAbs(disparityMap, scaledDisparityMap, 255/(max-min));
	applyColorMap(scaledDisparityMap, cm_disp, COLORMAP_JET);

	// Compute a mask to remove background
	Mat dst, thresholded_disp;
	threshold(scaledDisparityMap, thresholded_disp, 20, 255, THRESH_OTSU + THRESH_BINARY);
	resize(thresholded_disp, dst, Size(640, 480), 0, 0, INTER_LINEAR_EXACT);
	thresholded_disp = 255;
	imshow("threshold disp otsu", thresholded_disp);
	Mat mask = Mat(imgcolor.rows, imgcolor.cols, imgcolor.type());
	cvtColor(thresholded_disp, mask, COLOR_GRAY2RGB);

	// Apply the mask to the point cloud
	pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
	imgcolor.copyTo(color_tresh, mask);

	//myWindow.setBackgroundTexture(imgcolor);
	// Show the point cloud on viz
	myWindow.showWidget("pointcloud", viz::WCloud(pointcloud_tresh, color_tresh));
	//myWindow.showWidget("pointcloud2", viz::WPaintedCloud(pointcloud_tresh), Affine3d().translate(Vec3d(0.0, 0.0, 180.0)));
	myWindow.showWidget("text2d", viz::WText("Point cloud", Point(80, 80), 20, viz::Color::green()));
	//	myWindow.spin();
	myWindow.spinOnce(1, true);
	return 0;
}


int stereo_match_depth_filter()
{
	//1,Load left and right views
	cv::String GT_path = "E:/MyProjects/OpenCV/images/ambush_5_bm_with_filter.png";
	cv::String sleft = "D:\\jupyterNote\\Zoo_Data\\stereo_drivers\\2018-07-09-16-11-56\\2018-07-09-16-11-56_left";
	cv::String sright = "D:\\jupyterNote\\Zoo_Data\\stereo_drivers\\2018-07-09-16-11-56\\2018-07-09-16-11-56_right";
	vector<cv::String> fnl, fnr;
	cv::glob(sleft, fnl, true); // recurse
	cv::glob(sright, fnr, true); // recurse

	viz::Viz3d myWindow("Point cloud with color");
	//myWindow.setBackgroundMeshLab();
	myWindow.setBackgroundTexture();
	myWindow.showWidget("coosys", viz::WCoordinateSystem());

	Mat Q; 
	FileStorage fs("calib_stereo_allparams.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["Q"] >> Q;
		fs.release();
	}

	String dst_path = "";
	String dst_raw_path = "";
	String dst_conf_path = "";
	String algo = "bm"; //"sgbm"
	String filter = "wls_conf";  //wls_conf   wls_no_conf    fbs_conf
	bool no_display = false;
	bool no_downscale = true;
	double vis_mult = 1.0;
	int max_disp = 160;
	double lambda = 8000.0;
	double sigma = 1.5;
	double fbs_spatial = 16.0;
	double fbs_luma = 8.0;
	double fbs_chroma = 8.0;
	double fbs_lambda = 128.0;
	int wsize = 15;

	for (int i = 0; i < fnl.size(); i++)
	{

		Mat left = imread(fnl[i], IMREAD_COLOR);
		if (left.empty())
			return -1;
		Mat right = imread(fnr[i], IMREAD_COLOR);
		if (right.empty())
			return -1;
		namedWindow("scene", WINDOW_AUTOSIZE);
		imshow("scene", left);

		Mat conf_map = Mat(left.rows, left.cols, CV_8U);
		conf_map = Scalar(255);
		Rect ROI;
		double matching_time, filtering_time;
		double solving_time = 0;
		//2, Prepare the views for matching
		Mat left_for_matcher, right_for_matcher;
		if (max_disp % 16 != 0)
			max_disp += 16 - (max_disp % 16);

		//3, Perform matching and create the filter instance
				//! [matching]
		Mat left_disp, right_disp;
		Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
		Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
		Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

		cvtColor(left, left_for_matcher, COLOR_BGR2GRAY);
		cvtColor(right, right_for_matcher, COLOR_BGR2GRAY);

		matching_time = (double)getTickCount();
		left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
		right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
		matching_time = ((double)getTickCount() - matching_time) / getTickFrequency();

		//4 Perform filtering
		Mat filtered_disp, solved_disp, solved_filtered_disp;
		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		filtering_time = (double)getTickCount();
		wls_filter->filter(left_disp, left, filtered_disp, right_disp);
		filtering_time = ((double)getTickCount() - filtering_time) / getTickFrequency();
		//! [filtering]
		conf_map = wls_filter->getConfidenceMap();
		//5 Visualize the disparity maps
//		Mat raw_disp_vis;
//		getDisparityVis(left_disp, raw_disp_vis, vis_mult);

		Mat filtered_disp_vis;
		getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
		namedWindow("filtered disparity", WINDOW_AUTOSIZE);
		imshow("filtered disparity", filtered_disp_vis);

		double min, max;
		minMaxIdx(filtered_disp_vis, &min, &max);
//		filtered_disp_vis = ((filtered_disp_vis-min)*(200.0/(max-min))+50);
		filtered_disp_vis += 60;
		namedWindow("new disparity", WINDOW_AUTOSIZE);
		imshow("new disparity", filtered_disp_vis);

		char key = (char)waitKey(50);
		if (key == 27 || key == 'q' || key == 'Q') // 'ESC'
			break;

		Mat disparityMap;
		Mat pointcloud;
		Mat Q2 = cv::Mat::zeros(Size(4, 4), CV_32FC3);
		Q2.at<float>(0, 0) = 1;
		Q2.at<float>(1, 1) = -1;
		Q2.at<float>(2, 2) = 45;
		Q2.at<float>(3, 3) = 1;
		//		Mat([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 30*0.05, 0],[0, 0, 0, 1]]);
		filtered_disp_vis.convertTo(disparityMap, CV_32FC1);
		reprojectImageTo3D(filtered_disp_vis, pointcloud, Q, true, -1);
		viz3dshowdepth(myWindow, left_for_matcher, left, disparityMap, pointcloud);
	}

}