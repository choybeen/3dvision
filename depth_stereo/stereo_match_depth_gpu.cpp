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

using namespace cv;
using namespace std;

bool help_showed = false;

struct Params
{
    Params();
    static Params read();

    string left;
    string right;

    string method_str() const
    {
        switch (method)
        {
        case BM: return "BM";
        case BP: return "BP";
        case CSBP: return "CSBP";
        }
        return "";
    }
    enum {BM, BP, CSBP} method;
    int ndisp; // Max disparity + 1
};

int viz3dshow(viz::Viz3d myWindow, Mat imggray, Mat imgcolor, Mat disparityMap, Mat pointcloud)
{
	// Compute the point cloud
	Mat pointcloud_tresh, color_tresh;

	// To better visualize the result, apply a colormap to the computed disparity
	double min;
	double max;
	minMaxIdx(disparityMap, &min, &max);
	Mat cm_disp, scaledDisparityMap;
	//cout << "disp min " << min << endl << "disp max " << max << endl;
	convertScaleAbs(disparityMap, scaledDisparityMap, 255 / (30 - min));
	scaledDisparityMap += Scalar(20, 20, 20);
	applyColorMap(scaledDisparityMap, cm_disp, COLORMAP_JET);

	// Compute a mask to remove background
	Mat dst, thresholded_disp;
	threshold(scaledDisparityMap, thresholded_disp, 20, 205, THRESH_OTSU + THRESH_BINARY);
	resize(thresholded_disp, dst, Size(640, 480), 0, 0, INTER_LINEAR_EXACT);
	imshow("threshold disp otsu", imgcolor);
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

struct App
{
    App(const Params& params);
    void run();
    void handleKey(char key);
    void printParams() const;
	tuple<cuda::GpuMat, cuda::GpuMat>   readImage(cv::String sleft, cv::String sright);

    void workBegin() { work_begin = getTickCount(); }
    void workEnd()
    {
        int64 d = getTickCount() - work_begin;
        double f = getTickFrequency();
        work_fps = f / d;
    }

    string text() const
    {
        stringstream ss;
        ss << "(" << p.method_str() << ") FPS: " << setiosflags(ios::left)
            << setprecision(4) << work_fps;
        return ss.str();
    }
private:
    Params p;
    bool running;

	vector<cv::String> fnl, fnr;
	Mat left_src, right_src;
	Mat left_align, right_align;
	Mat left, right;
    cuda::GpuMat d_left, d_right;

    Ptr<cuda::StereoBM> bm;
    Ptr<cuda::StereoBeliefPropagation> bp;
    Ptr<cuda::StereoConstantSpaceBP> csbp;

    int64 work_begin;
    double work_fps;
};

static void printHelp()
{
    cout << "Usage: stereo_match\n"
        << "\t--left <left_view> --right <right_view> # must be rectified\n"
        << "\t--method <stereo_match_method> # BM | BP | CSBP\n"
        << "\t--ndisp <number> # number of disparity levels\n";
    help_showed = true;
}

int stereo_match_depth_gpu()  //main func
{
    try
    {
		Params args = Params::read();
		App app(args);
        app.run();
    }
    catch (const exception& e)
    {
        cout << "error: " << e.what() << endl;
    }
    return 0;
}


Params::Params()
{
    method = BM;
    ndisp = 64;
}


Params Params::read()
{
    Params p;
    p.left = "left_imgs";
    p.right = "right_imgs";
    p.method = BM; // BP CSBP
    p.ndisp = 24;
    return p;
}


App::App(const Params& params)
	: p(params), running(false)
{
    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

    cout << "stereo_match_gpu sample\n";
    cout << "\nControls:\n"
        << "\tesc - exit\n"
        << "\tp - print current parameters\n"
        << "\tg - convert source images into gray\n"
        << "\tm - change stereo match method\n"
        << "\ts - change Sobel prefiltering flag (for BM only)\n"
        << "\t1/q - increase/decrease maximum disparity\n"
        << "\t2/w - increase/decrease window size (for BM only)\n"
        << "\t3/e - increase/decrease iteration count (for BP and CSBP only)\n"
        << "\t4/r - increase/decrease level count (for BP and CSBP only)\n";
}

tuple<cuda::GpuMat, cuda::GpuMat>  App::readImage(cv::String sleft, cv::String sright) {
	left_src = imread(sleft);
	right_src = imread(sright);
	Rect roi(0, 0, left_src.cols / 8 * 8, left_src.rows);
	left_align = left_src(roi);
	right_align = right_src(roi);
	if (left_align.empty()) throw runtime_error("can't open file \"" + sleft + "\"");
	if (right_align.empty()) throw runtime_error("can't open file \"" + sright + "\"");
	if(p.method == Params::BM) {
		cvtColor(left_align, left, COLOR_BGR2GRAY);
		cvtColor(right_align, right, COLOR_BGR2GRAY);
		d_left.upload(left);
		d_right.upload(right);
	}
	else {
		d_left.upload(left_align);
		d_right.upload(right_align);
	}
//	imshow("left", left_src);
//	imshow("right", right_src);

	return make_tuple( d_left, d_right );
}
void App::run()
{
	Mat cameraMatrix[2], distCoeffs[2];
	Mat rmap[2][2];
	Mat R1, R2, P1, P2, Q;   //computer distoration parameters, radial distoration k1/k2/k3 , tangential distortation: p1/p2
	Mat R, T, E, F;  // rotate, translate, Essential, Fundamental
	FileStorage fs("calib_stereo_allparams.yml", FileStorage::READ);
	if (fs.isOpened())
	{
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
//		fs["mapx0"] >> rmap[0][0];   //read mapping data from file
//		fs["mapy0"] >> rmap[0][1];   //in this case, for small data file on github
//		fs["mapx1"] >> rmap[1][0];  // generate mapping file from camera parameters
//		fs["mapy1"] >> rmap[1][1];
		fs.release();
	}
	else
		return;

	p.method = Params::BP;  //BM, BP, CSBP
	p.ndisp = 64;

	viz::Viz3d myWindow("Point cloud with color");
	//myWindow.setBackgroundMeshLab();
	myWindow.setBackgroundTexture();
	myWindow.showWidget("coosys", viz::WCoordinateSystem());
//	auto pose = myWindow.getViewerPose();
//	pose = pose.rotate(Vec3f(3.14, 0, 0));
//	myWindow.setViewerPose(pose);


	// Set common parameters
    bm = cuda::createStereoBM(p.ndisp);
    bp = cuda::createStereoBeliefPropagation(p.ndisp);
    csbp = cv::cuda::createStereoConstantSpaceBP(p.ndisp);

    cout << endl;
	size_t k = 0;
    running = true;
	cv::glob(p.left, fnl, true); // recurse
	cv::glob(p.right, fnr, true); // recurse
	left_src = imread(fnl[0]);
	Size szimg(left_src.cols / 8 * 8, left_src.rows);
    printParams();

	// generate mapping file from camera parameters
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, szimg, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, szimg, CV_16SC2, rmap[1][0], rmap[1][1]);

    // Prepare disparity map of specified type
    Mat disp(szimg, CV_16U);
    cuda::GpuMat d_disp(szimg, CV_16U);
	while (running)
    {
        workBegin();
		//step1,  read images and convert to hsv and hsl
		if (++k >= fnl.size()) k = 0;
		readImage(fnl[k], fnr[k]);
		switch (p.method)
        {
        case Params::BM:  
			bm->compute(d_left, d_right, d_disp);  break;
        case Params::BP:   
			bp->compute(d_left, d_right, d_disp);  break;
        case Params::CSBP:  
			csbp->compute(d_left, d_right, d_disp);  break;
        }
        workEnd();

        // Show results
		Mat unfilted;
		d_disp.download(unfilted);
		//cv::bilateralFilter(unfilted, disp, 7, 50, 50);
		cv::GaussianBlur(unfilted, disp, Size(5, 5), 0, 0); // blur(unfilted, disp, Size(3, 3));
		putText(disp, text(), Point(5, 25), FONT_HERSHEY_SIMPLEX, 1.0, Scalar::all(255));
        imshow("disparity", (Mat_<uchar>)disp);

		Mat disparityMap;
		Mat pointcloud;
		disp.convertTo(disparityMap, CV_32FC1);
		reprojectImageTo3D(disp, pointcloud, Q, true, -1);
		viz3dshow(myWindow, left, left_align, disparityMap, pointcloud);

		char key = (char)waitKey(50);
        handleKey(key);
		if (key == 27 || key == 'q' || key == 'Q') // 'ESC'
			break;
	}
}


void App::printParams() const
{
    cout << "--- Parameters ---\n";
    cout << "image_size: (" << left_src.cols << ", " << left_src.rows << ")\n";
    cout << "image_channels: " << left_src.channels() << endl;
    cout << "method: " << p.method_str() << endl
        << "ndisp: " << p.ndisp << endl;
    switch (p.method)
    {
    case Params::BM:
        cout << "win_size: " << bm->getBlockSize() << endl;
        cout << "prefilter_sobel: " << bm->getPreFilterType() << endl;
        break;
    case Params::BP:
        cout << "iter_count: " << bp->getNumIters() << endl;
        cout << "level_count: " << bp->getNumLevels() << endl;
        break;
    case Params::CSBP:
        cout << "iter_count: " << csbp->getNumIters() << endl;
        cout << "level_count: " << csbp->getNumLevels() << endl;
        break;
    }
    cout << endl;
}


void App::handleKey(char key)
{
    switch (key)
    {
    case 27:
        running = false;
        break;
    case 'p': case 'P':
        printParams();
        break;
    case 'g': case 'G':
        if (left.channels() == 1 && p.method != Params::BM)
        {
            left = left_src;
            right = right_src;
        }
        else
        {
            cvtColor(left_src, left, COLOR_BGR2GRAY);
            cvtColor(right_src, right, COLOR_BGR2GRAY);
        }
        d_left.upload(left);
        d_right.upload(right);
        cout << "image_channels: " << left.channels() << endl;
        imshow("left", left);
        imshow("right", right);
        break;
    case 'm': case 'M':
        switch (p.method)
        {
        case Params::BM:
            p.method = Params::BP;
            break;
        case Params::BP:
            p.method = Params::CSBP;
            break;
        case Params::CSBP:
            p.method = Params::BM;
            break;
        }
        cout << "method: " << p.method_str() << endl;
        break;
    case 's': case 'S':
        if (p.method == Params::BM)
        {
            switch (bm->getPreFilterType())
            {
            case 0:
                bm->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
                break;
            case cv::StereoBM::PREFILTER_XSOBEL:
                bm->setPreFilterType(0);
                break;
            }
            cout << "prefilter_sobel: " << bm->getPreFilterType() << endl;
        }
        break;
    case '1':
        p.ndisp = p.ndisp == 1 ? 8 : p.ndisp + 8;
        cout << "ndisp: " << p.ndisp << endl;
        bm->setNumDisparities(p.ndisp);
        bp->setNumDisparities(p.ndisp);
        csbp->setNumDisparities(p.ndisp);
        break;
    case 'q': case 'Q':
        p.ndisp = max(p.ndisp - 8, 8);
        cout << "ndisp: " << p.ndisp << endl;
        bm->setNumDisparities(p.ndisp);
        bp->setNumDisparities(p.ndisp);
        csbp->setNumDisparities(p.ndisp);
        break;
    case '2':
        if (p.method == Params::BM)
        {
            bm->setBlockSize(min(bm->getBlockSize() + 1, 51));
            cout << "win_size: " << bm->getBlockSize() << endl;
        }
        break;
    case 'w': case 'W':
        if (p.method == Params::BM)
        {
            bm->setBlockSize(max(bm->getBlockSize() - 1, 2));
            cout << "win_size: " << bm->getBlockSize() << endl;
        }
        break;
    case '3':
        if (p.method == Params::BP)
        {
            bp->setNumIters(bp->getNumIters() + 1);
            cout << "iter_count: " << bp->getNumIters() << endl;
        }
        else if (p.method == Params::CSBP)
        {
            csbp->setNumIters(csbp->getNumIters() + 1);
            cout << "iter_count: " << csbp->getNumIters() << endl;
        }
        break;
    case 'e': case 'E':
        if (p.method == Params::BP)
        {
            bp->setNumIters(max(bp->getNumIters() - 1, 1));
            cout << "iter_count: " << bp->getNumIters() << endl;
        }
        else if (p.method == Params::CSBP)
        {
            csbp->setNumIters(max(csbp->getNumIters() - 1, 1));
            cout << "iter_count: " << csbp->getNumIters() << endl;
        }
        break;
    case '4':
        if (p.method == Params::BP)
        {
            bp->setNumLevels(bp->getNumLevels() + 1);
            cout << "level_count: " << bp->getNumLevels() << endl;
        }
        else if (p.method == Params::CSBP)
        {
            csbp->setNumLevels(csbp->getNumLevels() + 1);
            cout << "level_count: " << csbp->getNumLevels() << endl;
        }
        break;
    case 'r': case 'R':
        if (p.method == Params::BP)
        {
            bp->setNumLevels(max(bp->getNumLevels() - 1, 1));
            cout << "level_count: " << bp->getNumLevels() << endl;
        }
        else if (p.method == Params::CSBP)
        {
            csbp->setNumLevels(max(csbp->getNumLevels() - 1, 1));
            cout << "level_count: " << csbp->getNumLevels() << endl;
        }
        break;
    }
}
