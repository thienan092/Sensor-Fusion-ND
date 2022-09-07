# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
  * Linux/Windows: This must be [compiled](https://docs.opencv.org/3.4/d3/d52/tutorial_windows_install.html) from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * Visual Studio: [Build](https://gitlab.com/opencv6/opencv_wiki/-/wikis/Build-OpenCV-libraries-using-vcpkg) OpenCV libraries using vcpkg. 
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  * Visual Studio: MSVC in Visual Studio IDE. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Implemetation Details

[FP.1 Match 3D Objects] 
 - Step 1: Crop Lidar points stay inside the scene and have strong reflect signal. 
   Code: method 'cropLidarPoints'. 
 - Step 2: Assign lidar points to bounding boxes. Remove invalid bounding boxes. 
   Code: method 'clusterLidarWithROI'. 
 - Step 3: Match bounding boxes of the previous frame with bounding boxes of the current frame. 
   Thus, based on keypoint-matches, create a 2d array to count number of keypoint-matches belong to bounding boxes of the previous frame and the current frame. The criteria for matching 2 bounding boxes are: 
    + There is a match connects 2 bounding boxes ("(mm_it->second) > 0"). 
    + The number of matches connect 2 bounding boxes is greater than half of the number of matches connect one of them to other bounding boxes. ("(mm_it->second) >= (int(currCnt[i]*0.5)+1)" and "(mm_it->second) >= (int(prevCnt[max_id]*0.5)+1)"). 
   Code: method 'matchBoundingBoxes'. 

[FP.2 Compute Lidar-based TTC] Code: method 'computeTTCLidar'. 
 - For reducing the effect of outliers on computing TTC, 1st quartile (Q1), 3rd quartile (Q3) and interquartile range (IQR=Q3-Q1) are computed (method 'computeStats'). Thus, points have depth less than Q1 - 1.5*IQR and greater than Q3 + 1.5*IQR are considered outliers. Additionally, to make the removal of outliers make statistical sense, instead of using all lidar points, only 30 lidar points with the smallest depth are used in computing Q1, Q3 and IQR. Thus, these points are assumpted to share the same statistical parameter is the distance from the car to the camera. 

[FP.3 Associate Keypoint Correspondences with Bounding Boxes] Code: method 'clusterKptMatchesWithROI'
 - Assign a match in 'kptMatches' of the current frame to 'kptMatches' of a bounding box if the corresponding keypoint of the match in 'kptsCurr' stays inside that bounding box. Do this act for all matches. 
 - For reducing the effect of outliers on computing TTC, 1st quartile (Q1), 3rd quartile (Q3) and interquartile range (IQR=Q3-Q1) are computed (method 'computeStats'). Thus, points have depth less than Q1 - 1.5*IQR and greater than Q3 + 1.5*IQR are considered outliers. 
 
[FP.4 Compute Camera-based TTC] Code: method 'computeTTCCamera'
 - Step 1: Reduce outlier matches which sometimes stay outside the bounding box of the source frame (the previous frame) through applying an upper-bound (the value of 'maxDist') for the distance of pairs of keypoint used for computing TTC. 
 - Step 2: Remove apparance of outlier matches in computing TTC through comparing between the number of matches have "distRatio>(1.+1e-7)" (group I) and the number of matches have "distRatio<(1.-1e-7)" (group II). Thus, only matches of the major group of matches will be used in computing TTC. 
 - Step 3: Use median of distance-ratios in the Camera-based TTC formula to make the result statistically robust. 

[FP.5 Performance Evaluation 1]
 - There are some outier Lidar points in frames 12th, 17th. Thus, for the frame 12th, there is only one outlier on the left side of the front car. The difference between the depth of the outlier and the nearest point of the car is (7.344 - 7.205 = 0.139). The coordinates of the outlier is (x=-0.702, y=7.205, z=-1.096). For the frame 17th, there is only one outlier on the left side of the front car. The difference between the depth of the outlier and the nearest point of the car is (6.963 - 6.827 = 0.136). The coordinates of the outlier is (x=-0.708, y=6.827, z=-1.093). It seems like the difference of outliers and the nearest point is constant over time. Thus, It's likely that the cause of this error depends on the car. A strong assumption will be used to diagnose the cause is the index of Lidar point reflects the time it was measured. Precisely, a Lidar point of a smaller index is measured before a Lidar point of a greater index. This assumption can be used to interpolate the true coordinates of the outlier. For the frame 12th, the coordinates of 11 Lidar points were measuared around the time the outlier was measured are: 
   index: 152 -         x: 7.414; y: -0.829; z: -1.133
   index: 153 -         x: 7.408; y: -0.805; z: -1.131
   index: 154 -         x: 7.399; y: -0.781; z: -1.129
   index: 155 -         x: 7.384; y: -0.756; z: -1.126
   index: 156 -         x: 7.38 ; y: -0.732; z: -1.125
   index: 157 - Outlier x: 7.205; y: -0.702; z: -1.096
   index: 158 -         x: 7.413; y: -0.7  ; z: -1.13
   index: 159 -         x: 7.427; y: -0.654; z: -1.132
   index: 160 -         x: 7.437; y: -0.631; z: -1.133
   index: 161 -         x: 7.425; y: -0.607; z: -1.131
   index: 162 -         x: 7.436; y: -0.596; z: -1.133
   
   Thus, the index of the outlier is 157. Based on the data, the true z coordinate of the outlier should be -1.124. While, the true xy coordinate lies in a rectangle determined by 2 point are (x=7.38, y=-0.732, z=-1.124) and (x=7.376, y=-0.702, z=-1.124). This is a raw interpolation from points have indices are 155 and 156, which likely have the same z coordinate as the outlier. Projecting (x=7.38, y=-0.732, z=-1.124), (x=7.376, y=-0.702, z=-1.124) and the outlier on an image through the camera matrix shows that the outlier stays inside the rectangle. This means that neither the outlier appears incidentally by noise nor it's the result of a device error. One explaination for the outlier maybe it is the reflection of exhaust from the car's engine. Besides, it maybe a result of human intervention. 
 
[FP.6 Performance Evaluation 2]
 - TTC estimation of all detector / descriptor combinations is in "TTC_estimation.xlxs". 
 - The combination has the best performance are: AKAZE -> ORB. Thus, average absolute deviation (AAD) of this combination is 2.813 second. 
 - The data shows that the estimations of frames 7th, 8th and 9th have the largest deviations. Interestingly, the ground truth TCC of these frames are also the largest TCCs. Presicely, they are 34.3548, 17.4767 and 15.8894 seconds. A reasonable explanation for this phenomenon maybe changes in the position of keypoints is too small for estimation due to the resolution of the camera image. 
