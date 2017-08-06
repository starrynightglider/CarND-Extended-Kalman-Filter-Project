# Extended Kalman Filter Project
## Object 
In this project I utilize extened Kalmen Filter with LIDAR and RADAR measurements for object tracking.

![demo_both][demo1]

## Demo Video
The demo video shows the blue car is being tracked. To track the car, two different measurements are used, LIDAR (red circle) and RADAR (blue circle) measurements to estimate the location of the car.

I use extended Kalman filter (EKF) to compute the estimated location (green triangle) of the blue car. The estimated trajectory (green triangle) is compared with the ground true trajectory of the blue car, and the error up to current position is displayed in RMSE format.

## Accuracy
The MSRE of this implementation is [px, py, vx, vy]=[0.0973, 0.0855, 0.4513, 0.4399]

## Build Code
The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF (or ./ExtendedKF output_file)

To dump the estimated car positions and RMSE info, an output_file can be specified.  

### Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

[//]: # (Image References)
[demo1]: ./data/demo_radar_lidar.gif
