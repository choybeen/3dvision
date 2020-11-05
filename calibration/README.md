# 3dvision
 vision based on opencv/opencl/opengl/vtk
 
## one camera calibration
** 1) bshowcalib=false, for calibrate **

** 2)bshowcalib=false,  for show result, read map matrix from file **

calibrate with chessboard: ![GitHub Logo](https://github.com/choybeen/3dvision/blob/main/calibration/Capture.JPG)

## two camera calibration
** 1)  calib_stereodepth_calibration : for calibration **
	//std::thread thread_calibrating(calib_stereodepth_calibration);  
** 2)  calibed_stereodepth_rectifyimage  :  for result showing **
	std::thread thread_calibrating(calibed_stereodepth_rectifyimage);    
two video: ![Alt Text](https://github.com/choybeen/3dvision/blob/main/calibration/Capture5.JPG)

[![Fibonacci RMI Java EE](http://img.youtube.com/vi/nX_inqaAzOI/0.jpg)](https://youtu.be/OIW2UMj6u9w)

