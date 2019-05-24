# Intro to this package
A package wrapping some PCL function into ROS service
Parameter is so far fixed, untuned.

## Hint
1. Recognition rate is quite low due to low point cloud resolution. It is just a test to see if kinect v2 is capable of recognize small and simple instrial metal parts which have textureless appearance.
2. The use of build-in openmp class to accelerate computation feature sometime may fail. PCL is not so stable.
3. Also if computation failed or there is some runtime error, try to cmake with RelWithDeubInfo, and try disable optimization of g++ (ps. default is -O3, try -O2, -O1 or disable it)

My scene data with kinetic vw without calibration [Download](https://docs.google.com/uc?export=download&id=1hu24fzK6UWyHuMvjTJyuWSYJhqoMSfFP)


## Status
Only the code in include folder is usable.

## TODO
- improve matching code
- segmentation for CVFU-OUR
- guide to run matching pipeline, preprocessing, segmentation
## ROS service server
- preprocess
- detectors: iss, sift, uniforsampling keypoints
- descriptors: pfh, fpfh, vfh, usc, sc3d...


## pcl classes:
- preprocessor
- descriptor
- detector (not yet wrapped into class, not yet templated)
