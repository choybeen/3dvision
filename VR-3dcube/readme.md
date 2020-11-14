

# 3dvision
 3d VR 
 a 3d cube generated from the bookface features, as the book tranplants, rotates, the cube pose transformes with the books pose.
 
 #### 1) get features from bookface image, and train the feature
 
 #### 2) get current features from new images
 
 #### 3) match two sets features, find the fundmental matrix between two images
 
 #### 4) refine the matrix by :
     **  warp current image from the fundmental matrix, get new warped image  **
     **  extracte features from warped image, then match the new feature with model features  **
     **  get refined fundmental matrix  **
     **  transform the points in model image to warped image perspectively **

#### 5) get current transplant/rotate matrix from warped points with 3d cube vertex position

#### 6) project the 3d cube vertex position with current transplant/rotate matrix into 2d polygon

#### 7) draw 2d polygon in current image

video on youbute:
 
 [![Fibonacci RMI Java EE](https://github.com/choybeen/3dvision/blob/main/VR-3dcube/Capture.JPG?raw=true)](https://youtu.be/cM2_O_Slqd0)


