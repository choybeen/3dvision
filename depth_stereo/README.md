# 3dvision
 vision based on opencv/opencl/opengl/vtk
 
## one camera calibration
** 1) bshowcalib=false, for calibrate **
** 2)bshowcalib=false,  for show result, read map matrix from file **
 ![GitHub Logo](https://github.com/choybeen/3dvision/blob/main/calibration/Capture.JPG)
Format: ![Alt Text](url)

## two camera calibration
** 1)  calib_stereodepth_calibration : for calibration **
	//std::thread thread_calibrating(calib_stereodepth_calibration);  
** 2)  calibed_stereodepth_rectifyimage  :  for result showing **
	std::thread thread_calibrating(calibed_stereodepth_rectifyimage);    
 ![GitHub Logo](https://github.com/choybeen/3dvision/blob/main/calibration/Capture5.JPG)
Format: ![Alt Text](url)
