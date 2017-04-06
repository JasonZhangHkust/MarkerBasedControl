#include <string>

// Input for pid is error(t) = x0 - x (difference beetwen required value (x0) and current value (x))
class PIDW
{

public:
	PIDW();
	PIDW(const char* configPath, int ID);                // configPath - path to the PID config file

	~PIDW();

	void initByFile(const char* configPath);    // configPath - path to the PID config file

	void compute(double alpha,double beta);           // Computing the output signal
	double getOutput();                     // Get the output after computing
	int ID;
	bool fail();
	std::string getErrorStr();

private:
	void setFail();
	void setErrorStr(std::string errorString);

	double wP_;                     // P, I, D weights
	double wI_;                     //
	double wD_;  
	//
	double wP_2;                     // P, I, D weights
	double wI_2;                     //
	double wD_2;

	double lastAlpha;              // Last input, given 1 period ago
	double lastBeta;
	double integralSum_;            // Integral sum
	double integralSum_2;
	double output_;                 // Output after computing

	double period_;                 // Refreshing period in milliseconds

	double minOutput_;              // Output is limited
	double maxOutput_;              //

	bool fail_;                     // True on fail
	std::string errorString_;       // If fail_ is true, here is a valid reason of the error
};

