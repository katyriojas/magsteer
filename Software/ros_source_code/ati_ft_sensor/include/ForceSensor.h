/********************************************************************
Header file for ForceSensor class
Neal P. Dillon
revised:  6/2/2017
********************************************************************/

#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <comedilib.h>
#include <armadillo>
#include <string.h>

using namespace std;
using namespace arma;

class ForceSensor {

public:
	// Default constructor
	ForceSensor();

	// Destructor
	~ForceSensor();

	// Setup comedi for reading DAQ
	void SetupComedi(string comedi_filepath);
    bool getSetupStatus();
	
	// Load matrix (sensor-specific) for converting volts to forces/torques
	void LoadCalibrationMatrix(string cal_matrix_file);
    bool getCalStatus();
	
	// Sample voltage values to unbias/zero sensor
	void SetBiasVoltages(int nCalSamples);
    bool getZeroedStatus();
	
	// Read voltage values and convert to forces/torques
	mat ReadATIForceSensor();

private: // member variables
//getstatus variables
	bool SetupStatus; 
	bool CalStatus;	
	bool ZeroedStatus;

	// Setup comedi variables
	comedi_t *it;
	int subdev; // sub-device
	int chan; // channel
	int range; // not sure what this means
	int aref = AREF_GROUND;
	lsampl_t data;
	comedi_range * range_info;
	lsampl_t maxdata;

	// Force measurement variables
	//bool ComediSetupStatus; // flag to tell if comedi is setup	
	//bool ZeroedStatus; // flag to tell if the sensor has been zeroed
	mat::fixed<6,6> CalMatrix;
//	mat CalMatrix; // calibration matrix for converting volts to forces/torques
	mat BiasVoltages; // voltages measured with no forces/torques applied
	
};

#endif // FORCESENSOR_H
