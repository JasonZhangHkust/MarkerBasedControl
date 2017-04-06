#include "pidW.h"

#include <string>
#include <iostream>
#include <fstream>

// Constructors

PIDW::PIDW() : wP_(0.0),
wI_(0.0),
wD_(0.0),

lastAlpha(0.0),
lastBeta(0.0),
integralSum_(0.0),
integralSum_2(0.0),
output_(0.0),

period_(0.0),

minOutput_(0.0),
maxOutput_(0.0),

fail_(false),
errorString_("")
{
	// Everything is in initialization list
}

PIDW::PIDW(const char* configPath, int ID)
{
	initByFile(configPath);
	this->ID = ID;
}

// Destructor
PIDW::~PIDW()
{
	wP_ = -1;
	wI_ = -1;
	wD_ = -1;

	integralSum_ = -1;
	lastAlpha = -1;
	lastBeta = -1;
	output_ = -1;

	period_ = -1;

	minOutput_ = -1;
	maxOutput_ = -1;

	fail_ = true;
	errorString_ = "";
}

// Methods

void
PIDW::initByFile(const char* configPath)
{
	if (configPath == NULL)
	{
		this->setFail();
		this->setErrorStr("NULL pointer was passed to initByFile().");

		return;
	}

	std::ifstream configFile(configPath, std::ifstream::in);
	if (configFile.fail())
	{
		this->setFail();
		this->setErrorStr("Failed to open file (" + (std::string)configPath + ").");

		return;
	}

	std::string tmpStr;

	// PWeight
	configFile >> tmpStr;
	configFile >> wP_;

	// IWeight
	configFile >> tmpStr;
	configFile >> wI_;

	// DWeight
	configFile >> tmpStr;
	configFile >> wD_;

	// PWeight2
	configFile >> tmpStr;
	configFile >> wP_2;

	// IWeight2
	configFile >> tmpStr;
	configFile >> wI_2;

	// DWeight2
	configFile >> tmpStr;
	configFile >> wD_2;

	// Period
	configFile >> tmpStr;
	configFile >> period_;

	// MaxOutput
	configFile >> tmpStr;
	configFile >> maxOutput_;

	// MinOutput
	configFile >> tmpStr;
	configFile >> minOutput_;

	configFile.close();

	// Initializing other fields
	integralSum_ = 0;
	integralSum_2 = 0;
	lastAlpha = 0;
	lastBeta = 0;

	output_ = 0;

	fail_ = false;
	errorString_ = "";
}

void
PIDW::compute(double alpha,double beta)
{
	double proportional = wP_ * alpha;
	double integral = integralSum_ + wI_ * alpha * period_;
	double derivative = wD_ * (alpha - lastAlpha) / period_;
	double proportional2 = wP_2 * beta;
	double integral2 = integralSum_ + wI_2 * beta * period_;
	double derivative2 = wD_2 * (beta - lastAlpha) / period_;
	integralSum_ = integral;
	integralSum_2 = integral2;
	lastAlpha = alpha;
	lastBeta = beta;

	double result = proportional + integral + derivative + proportional2 + integral2 + derivative2;

	output_ = result;
	if (result > maxOutput_)
		output_ = maxOutput_;
	if (result < minOutput_)
		output_ = minOutput_;
}

double
PIDW::getOutput()
{
	return output_;
}

void
PIDW::setFail()
{
	fail_ = true;
}

bool
PIDW::fail()
{
	return fail_;
}

void
PIDW::setErrorStr(std::string errorString)
{
	errorString_ = errorString;
}

std::string
PIDW::getErrorStr()
{
	return errorString_;
}
