# ToDo
check Liscene from PCL

# Intor to this package
A package wrapping some PCL function into ROS service
Parameter is so far fixed, untuned.

## ROS service server
- preprocess
- detectors: iss, sift, uniforsampling keypoints
- descriptors: pfh, fpfh, vfh, usc, sc3d...

**todo**:  
- matching
- hough voting and Hypothesis verification...

## pcl classes:
- preprocessor
- descriptor
- detector (not yet wrapped into class, not yet templated)
