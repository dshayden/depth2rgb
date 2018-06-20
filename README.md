# depth2rgb
A tool for registering a Kinect2 depth image to a Kinect2 rgb image so that the
(i,j)^th pixel in the depth map corresponds to the (i,j)^th pixel in the rgb
image. You will need to have the jpg and depth images extracted from the
recorded files.

## Prerequisites
* OpenCV >= 2.3
* Freenect2

## Building (Linux/Mac)
```
mkdir build
cmake ..
make
```

## Example Use
(from build/ directory)
```
depth2rgb ../example/dep.png ../example/rgb.jpg ../example/dep_registered.png
../example/recording-2017-09-28_13_37_56.log
```
