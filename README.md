# Extended Kalman Filter Project
## Object 
In this project I utilize extened Kalmen Filter with LIDAR and RADAR measurements for object tracking.

## Tracking Result
The video shows the blue car is being tracked. To track the car, two different measurements are used, LIDAR (red circle) and RADAR (blue circle) measurements to estimate the location of the car.

![][demo1]

I use kalman filter (KF) and extended Kalman filter (EKF) to compute the estimated location (green triangle) of the blue car. The estimated trajectory (green triangle) is compared with the ground true trajectory of the blue car, and the error up to current position is displayed in root-mean-square-error (RMSE) format.

## Accuracy
The MSRE of this implementation is [px, py, vx, vy]=[0.0973, 0.0855, 0.4513, 0.4399]

## Sensor Fusion of Extended Kalman Filter 
The following figure shows the fusion sensor flow using kalman filters. It consists two steps: predict and update with measurement. The updation depends on the type of sensor.

![][fusion]

## Comparison of Kalman Filter and Extended Kalman Filter
The different between KF and EKF is that EKF linearlized the H and F funcion for radar sensor, since that the conversion bewteen Cartesian and polar coordinations is non-linear.

![][kf_and_ekf]
* _x_ : mean state vector
* _F_ : stae transition matrix
* _u_ : external motion vector
* _P_ : state covariance matrix, uncertainty of the tracking
* _Q_ : process covariance matrix.

* _H_ : measurement projection matrix
* _z_ : observed measurement
* _R_ : measurement noise covariance matrix
* _K_ : Kalman filter gain
* _Hj_ and _Fj_ : jacobian matrices

## Comparison of LIDAR, RADAR and Camera
As the table shows, LIDAR provides higer location resolution but no velocity info given. On the other side, RADAR can measure velocity but with lower location resolution. Therefore, sensor fusion method can take the advantages of both sides. In addition, the RADAR sensor is more robust in bad weather conditions, which is another benefit.

|            Sensor type           |  LIDAR |    RADAR  |   Camera   |
|:--------------------------------:|:------:|:---------:|:----------:|
|            Resolution            | median |  low      |  **high**  |
|         Measure veolcity         |   no   |  **yes**  |     no     |
|         Weather resistent        |   bad  |  **good** |    bad     |
|            Sensor size           |  large | **small** |  **small** |


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
[kf_and_ekf]: ./data/kf_and_ekf.png
[fusion]: ./data/fusion.png
