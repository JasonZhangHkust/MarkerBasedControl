#include "RobotCar.h"

RobotCar::RobotCar(int id, Location self) {
	this->Id = id;
	this->self = self;
}
RobotCar::RobotCar(int id,std::vector<cv::Point2f> vertex, cv::Mat cameraMatrix) {// this constructor uses the four corner of the marker to initialize
	cv::Point2f tmp;
	for (auto point : vertex) {
		tmp += point;
	}
	this->cameraMatrix = cameraMatrix;

	cv::Point2f  k = vertex[1] - vertex[0];
	float theta = cv::fastAtan2(k.y, k.x);
	Location loaction = Location{ tmp / 4, theta };
	this->self = loaction;
	this->targetMat.at<double>(0, 0) = loaction.xy.x;
	this->targetMat.at<double>(1, 0) = loaction.xy.y;
	target = loaction;
	this->Id = id;
	kalmanInit();

}

RobotCar::RobotCar(int id, Location self, Location target, cv::Mat cameraMatrix){
	this->Id = id;
	this->self = self;
	this->target = target;
	this->cameraMatrix = cameraMatrix;
	this->targetMat.at<double>(0, 0) = target.xy.x;
	this->targetMat.at<double>(1, 0) = target.xy.y;

}
void RobotCar::updatePosition(std::vector<cv::Point2f> vertex, cv::Vec3d& rotationVector, cv::Vec3d& translationVector) {
	targetMat.at<double>(0, 0) = target.xy.x;
	targetMat.at<double>(1, 0) = target.xy.y;
	rvec.at<double>(0, 0) = rotationVector[0];
	rvec.at<double>(1, 0) = rotationVector[1];
	rvec.at<double>(2, 0) = rotationVector[2];
	tvec.at<double>(0, 0) = translationVector[0];
	tvec.at<double>(1, 0) = translationVector[1];
	tvec.at<double>(2, 0) = translationVector[2];

	cv::Rodrigues(rvec, rotationMatrix);
	cv::Mat invRotationMatrix;
	cv::transpose(rotationMatrix, invRotationMatrix);
	tempMat = invRotationMatrix * cameraMatrix.inv() * targetMat;
	tempMat2 = invRotationMatrix * tvec;
	s = 0 + tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);

	cv::Mat wcPoint = invRotationMatrix * (s * cameraMatrix.inv() * targetMat - tvec);
	cv::Point2f realPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0));
	//std::cout << realPoint << std::endl;
	cv::Point2f tmp;
	for (auto point : vertex) {
		tmp += point;
	}
	tmp = tmp / 4;
	//kalmanUpdate();
	kalCorrect(tmp);
	if (tmp.x - self.xy.x > 0.5 || self.xy.x - tmp.x > 0.5) {
		self.xy.x = tmp.x;
	}
	if (tmp.y - self.xy.y > 0.5 || self.xy.y - tmp.y > 0.5) {
		self.xy.y = tmp.y;
	}
	cv::Point2f  k = vertex[1] - vertex[2];
	float theta = cv::fastAtan2(k.y, k.x);
	if (theta - self.theta > 0.5 || self.theta - theta > 0.5) {
		self.theta = theta;
	}
	if (1) {
		deviation.x = sqrt(realPoint.dot(realPoint));
		//cv::Point2f tmp = target.xy - self.xy;
		deviation.y = cv::fastAtan2(realPoint.x,realPoint.y)>180? cv::fastAtan2(realPoint.x, realPoint.y) - 360: cv::fastAtan2(realPoint.x, realPoint.y);
		deviation.z = 360 - self.theta + deviation.y;
	}
}
cv::Point3f RobotCar::getDeviation() {
	return deviation;
}


void RobotCar::setLocation(Location newLocation) {
	this->self = newLocation;
}
Location RobotCar::getLocation() {
	return this->self;
}
void RobotCar::setId(int newId) {
	this->Id = newId;
}
int RobotCar::getId() {
	return this->Id;
}
void RobotCar::setTarget(Location newTarget) {
	this->target = newTarget;
	this->targetMat.at<double>(0, 0) = newTarget.xy.x;
	this->targetMat.at<double>(1, 0) = newTarget.xy.y;
}

void RobotCar::kalmanInit() {
	int stateSize = 4;
	int measSize = 2;
	int contrSize = 0;

	unsigned int type = CV_32F;
	kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

	state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
										  //cv::Mat procNoise(stateSize, 1, type)
										  // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

										  // Transition State Matrix A
										  // Note: set dT at each processing step!
										  // [ 1 0 dT 0  0 0 ]
										  // [ 0 1 0  dT 0 0 ]
										  // [ 0 0 1  0  0 0 ]
										  // [ 0 0 0  1  0 0 ]
										  // [ 0 0 0  0  1 0 ]
										  // [ 0 0 0  0  0 1 ]
	cv::setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 ]
	// [ 0 1 0 0 ]
	// [ 0 0 0 0 ]
	// [ 0 0 0 0 ]
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(5) = 1.0f;
	//kf.measurementMatrix.at<float>(16) = 1.0f;
	//kf.measurementMatrix.at<float>(23) = 1.0f;


	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.errorCovPre.at<float>(0) = 1; // px
	kf.errorCovPre.at<float>(5) = 1; // px
	kf.errorCovPre.at<float>(10) = 1;
	kf.errorCovPre.at<float>(15) = 1;
	//kf.errorCovPre.at<float>(28) = 1; // px
	//kf.errorCovPre.at<float>(35) = 1; // px

	kf.processNoiseCov.at<float>(0) = 1e-1;
	kf.processNoiseCov.at<float>(5) = 1e-1;
	kf.processNoiseCov.at<float>(10) = 1e-1;
	kf.processNoiseCov.at<float>(15) = 1e-1;
	//kf.processNoiseCov.at<float>(28) = 1e-2;
	//kf.processNoiseCov.at<float>(35) = 1e-2;

	state.at<float>(0) = self.xy.x;
	state.at<float>(1) = self.xy.y;
	state.at<float>(2) = 0;
	state.at<float>(3) = 0;
	//state.at<float>(4) = 0;
	//state.at<float>(5) = 0;

	kf.statePost = state;
	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-4));

}
void RobotCar::kalmanUpdate() {
	preTick = ticks;
	ticks = (double)cv::getTickCount();
	double dT = (ticks - preTick) / cv::getTickFrequency();
	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(7) = dT;//9

	state = kf.predict();

	self.xy.x = state.at<float>(0);
	self.xy.y = state.at<float>(1);


}
void RobotCar::kalCorrect(cv::Point2f newXY) {
	preTick = ticks;
	ticks = (double)cv::getTickCount();
	double dT = (ticks - preTick) / cv::getTickFrequency();
	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(7) = dT;//9
	meas.at<float>(0) = newXY.x;
	meas.at<float>(1) = newXY.y;

	kf.correct(meas);
}

