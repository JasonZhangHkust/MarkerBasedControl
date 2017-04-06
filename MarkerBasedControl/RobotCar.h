#include "opencv2\core.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\aruco.hpp"
#include "opencv2\calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include<iostream>

struct Location
{
	cv::Point2f xy;
	float theta;
};

class RobotCar
{
private:
	Location target = {cv::Point2f(-1,-1),-1};
	Location self;
	int Id;
	std::vector<cv::Point2f> vertices;
	cv::Point3f deviation;
public:
	RobotCar(int id, std::vector<cv::Point2f> vertex, cv::Mat cameraMatrix); // cconstructor with four retangle vertices;
	RobotCar(int id, Location self);
	RobotCar(int id, Location self, Location target, cv::Mat cameraMatrix);
	void updatePosition(std::vector<cv::Point2f> vertex,cv::Vec3d& rotationVector,cv::Vec3d& translationVector);//this function uses four vertices to update the whole postion parameters;
	void kalmanUpdate();
	void setLocation(Location newLocation);// this fucntion uses the actual coordination to update the the location;
	Location getLocation();
	int getId();
	void setId(int newId);
	void setTarget(Location newTarget);
	cv::Point3f getDeviation();
	double TargetDegree;
private:
	void kalmanInit();
	void kalCorrect(cv::Point2f  newXY);
private:
	cv::KalmanFilter kf;
	cv::Mat state;
	cv::Mat meas;
	cv::Rect predRect;
	double preTick;
	double ticks = 0;
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;
	unsigned int type = CV_32F;
	cv::Mat tvec = cv::Mat(3, 1, cv::DataType<double>::type);
	cv::Mat rvec = cv::Mat(3, 1, cv::DataType<double>::type);
	cv::Mat rotationMatrix= cv::Mat(3, 3, cv::DataType<double>::type);
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat targetMat = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	cv::Mat tempMat, tempMat2;
	double s;

};
