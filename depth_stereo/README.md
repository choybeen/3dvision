# 3dvision
 vision based on opencv/opencl/opengl/vtk
 
 dataset download from : https://drivingstereo-dataset.github.io/
 
 stereo_match_depth_gpu.cpp using GPU, needs cuda
 
##  match difference from two images, stereo vision for self-driving
** 1) compare difference **
** 2) conver diff into depth **
** 3) show depth with viz/VTK **
 ![GitHub Logo](https://github.com/choybeen/3dvision/blob/main/depth_stereo/Captured3.JPG?raw=true)
 
 
 video on youbute:
 
 [![Fibonacci RMI Java EE](https://github.com/choybeen/3dvision/blob/main/depth_stereo/Captured4.JPG?raw=true)](https://youtu.be/OIW2UMj6u9w)
 
 ## depth cloud data filter to more smooth view
 in  stereo_match_depth_filter.cpp, getDisparityVis filter the disparity result to smooth 

 ![GitHub Logo](https://github.com/choybeen/3dvision/blob/main/depth_stereo/Capturef.JPG?raw=true)

1. using left to right match, and right to left match, pair matcher has better disparity
Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
		
2. then filter :
wls_filter->filter(left_disp, left, filtered_disp, right_disp);

3. then convert disparity to image for showing:
Mat filtered_disp_vis;
getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);

