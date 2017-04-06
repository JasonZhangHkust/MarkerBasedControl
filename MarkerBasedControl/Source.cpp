#include "Server.h"
#include "RobotCar.h"
#include "pid.h"
#include "pidW.h"
#include "sstream"
#include "iostream"
#include "fstream"
#include "queue"
#include "time.h"
#define MAX_DATE 24
using namespace std;
using namespace cv;
bool SetTarget = false;
bool brakeEnable = true;
bool targetDone = false;
float arucoSquareDimension = 0.07f;
int brakeCount = 0;
int TargetCount = 4;
Location target{cv::Point2f(320,240),0};
queue<cv::Point2f> Aim;
static void onMouse(int event, int x, int y, int, void*) {
	if (SetTarget) {
		switch (event)
		{
		case EVENT_LBUTTONDOWN:
			target = Location{ Point2f{ float(x),float(y) },0 };
			SetTarget = false;
			brakeEnable = true;
			break;
		case EVENT_RBUTTONDOWN:
			target = Location{ Point2f{ float(x),float(y) },0 };
			brakeEnable = true;
			Aim.push(Point2f{ float(x),float(y) });
			
		default:
			break;
		}
	}
}
bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) {
	ifstream inStream(name);
	if (inStream) {
		uint16_t rows;
		uint16_t columns;
		inStream >> rows;
		inStream >> columns;
		cameraMatrix = Mat(Size(columns, rows), CV_64F);
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << endl;

			}
		}
		//Distance Coefficients
		inStream >> rows;
		inStream >> columns;
		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << endl;

			}
		}
		inStream.close();
		return true;
	}
	return false;
}





std::string get_date(void)
{
	time_t rawtime;
	struct tm * timeinfo;
	char the_date[MAX_DATE];
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	if (rawtime != -1)
	{
		strftime(the_date, MAX_DATE, "%I_%M%p_%d_%m_%Y", timeinfo);
	}
	cout << std::string(the_date) << endl;
	return std::string(the_date);
}
template< typename T >
std::string int_to_hex(T i)
{
	std::stringstream stream;
	stream << std::hex << i;
	return stream.str();
}
int main(int argv, char** argc)
{
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;
	loadCameraCalibration("Calibration2", cameraMatrix, distanceCoefficients);
	Mat frame;
	VideoWriter video;
	vector<int> markerIds, lastMarkerIds;
	PID pidServoV;
	PIDW pidServoW;
	vector<int> index; //current register ids;
	vector<RobotCar> Cars;
	vector<PID> Vcontrol;
	vector<PIDW> Wcontrol;
	string order = "6680808080000099";
	bool Start = false, First = true, Record = true; // basic flags for intialization
													  //Initialize the socket server
	Server MyServer(20001); //Create server on port 1111
	for (int i = 0; i < 1; i++) //Up to 100 times...
	{
		cout << "Wait for cars to connection!" << endl;
		MyServer.ListenForNewConnection(); //Accept new connection (if someones trying to connect)
	}

	vector<vector<Point2f>> markerCorners, lastMarkerCorners, rejectedCandidates;
	aruco::DetectorParameters parameters;
	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);// marker dictionary creation;

	VideoCapture cap(0); // construct the camera class

	Size S = Size((int)cap.get(CAP_PROP_FRAME_WIDTH),    // Acquire input size
		(int)cap.get(CAP_PROP_FRAME_HEIGHT));
	cv::String nameFile = "video" + get_date() + ".avi";
	cout << nameFile << endl;
	video.open(nameFile, CV_FOURCC('P', 'I', 'M', '1'), 25, S);
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	setMouseCallback("Webcam", onMouse, 0);
	if (!cap.isOpened()) {
		return -1;
	}
	//

	std::string userinput; //holds the user's chat message
	vector<Vec3d> rotationVectors, translationVectors;
	while (true) {
		if (!cap.read(frame))
			break;
		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds); // find the markers
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
		for (int i = 0; i < markerIds.size(); i++) {
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.035f);

		}
		if (Start&&First) {  //After press spaec button the tracking will intialize the tracking object;
			cout << "Start initial" << endl;
			for (int id = 0; id < markerIds.size();id++) {
				cout << "Initializing" << "Car " << markerCorners[id] << endl;
				Cars.push_back(RobotCar(markerIds[id], markerCorners[id], cameraMatrix));
				index.push_back(markerIds[id]);
				pidServoV = PID("pidServoV_config", markerIds[id]);
				if (pidServoV.fail())
				{
					cerr << pidServoV.getErrorStr() << endl;
					exit(EXIT_FAILURE);
				}
				Vcontrol.push_back(pidServoV);
				pidServoW = PIDW("pidServoW_config", markerIds[id]);
				if (pidServoV.fail())
				{
					cerr << pidServoV.getErrorStr() << endl;
					exit(EXIT_FAILURE);
				}
				Wcontrol.push_back(pidServoW);
				cout << "Finishing Initializing" << "Car " << markerIds[id] << endl;
			}
			First = false;
		}
		if (!Cars.empty()) {
			for (RobotCar & car : Cars) {
				int tmp = car.getId();
				if(targetDone) car.setTarget(target);
				auto iter = find(markerIds.begin(), markerIds.end(), tmp);
				if (iter != markerIds.end()) {
					tmp = distance(markerIds.begin(), iter);
					car.updatePosition(markerCorners[tmp], rotationVectors[tmp], translationVectors[tmp]);

				}
				else {
					//car.kalmanUpdate();
				}
				//cv::circle(frame, car.getLocation().xy, 2, CV_RGB(20, 150, 20), -1);
				//sprintf(y, "Angle = %4.2f", car.getLocation().theta);
				//putText(frame, y, car.getLocation().xy, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 1, 8);
				cv::line(frame, car.getLocation().xy, target.xy, Scalar(0, 0, 0), 2, 8);
				char Target[10] = "Target";
				cv::circle(frame, target.xy, 2, CV_RGB(20, 150, 20), -1);
				putText(frame, Target, target.xy, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 168), 1, 8);
				char deviation[100];
				sprintf(deviation, "p = %4.2f, alpha = %4.2f, beta = %4.2f", car.getDeviation().x, car.getDeviation().y, car.getDeviation().z);

				putText(frame, deviation, Point2f(200, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0, 255, 168), 1, 8);
				auto iter2 = find(index.begin(), index.end(), car.getId());
				tmp = distance(index.begin(), iter2);
				if (car.getDeviation().x > 0.06) {
					Vcontrol[tmp].compute(car.getDeviation().x);
					Wcontrol[tmp].compute(car.getDeviation().y, car.getDeviation().z);
					//cout << "signalX: " << Vcontrol[tmp].getOutput() << endl;
					double outPut = ((Vcontrol[tmp].getOutput() / 100) * 500 + 1500);
					outPut = (outPut - 1000) / 1000 * 255;
					string result = int_to_hex(int(outPut));
					order[6] = result[0];
					order[7] = result[1];
					if ((car.getDeviation().y > 10 || car.getDeviation().y < -10)&& car.getDeviation().x > 0.02) {
						outPut = ((Wcontrol[tmp].getOutput() / 250) * 500 + 1500);
						outPut = (outPut - 1000) / 1000 * 255;
						result = int_to_hex(int(outPut));
						order[2] = result[0];
						order[3] = result[1];
					}
					else {
						order[2] = '8';
						order[3] = '0';
					}

					MyServer.update(order, tmp);
				}
				else {
					if (brakeEnable&&targetDone&&Aim.empty()) {
						brakeCount++;
						if (brakeCount > 4) {
							brakeCount = 0;
							brakeEnable = false;
						}
						
						double outPut = ((Vcontrol[tmp].getOutput() / 100) * 500 + 1500);
						outPut = (outPut - 1000) / 1000 * 256;
						if (outPut - 0x80 > 0) {
							MyServer.update("6680805080000099", tmp);
							cout << "here" << endl;
						}
						else {
							MyServer.update("66D080B080000099", tmp);
							cout << "here2" << endl;
						}
							
					}
					else {
						MyServer.update("6680808080000099", tmp);
						if (targetDone && !Aim.empty()) {
							target.xy = Aim.front();
							Aim.pop();
							brakeEnable = true;
						}
					}

				}

			}




		}

		cv::imshow("Webcam", frame);
		if (Record) {
			video << frame;
		}
		char character = waitKey(30);
		switch (character)
		{
		case ' ':
			Start = true;
			break;
		case 27:
			cap.release();
			
			MyServer.update("6680808080000099", 0);
			memset(MyServer.close, true, sizeof(MyServer.close));
			return 0;
			break;
		case 115://s
			Record = !Record;
			break;
		case 116://t
			SetTarget = true;
			targetDone = false;
			break;
		case 113://q
			targetDone = true;
			SetTarget = false;
			if(!Aim.empty())
				target.xy = Aim.front();
			break;
		default:
			break;
		}
	}
	cap.release();
	video.release();

	return 0;

}