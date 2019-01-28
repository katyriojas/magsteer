#include <ForceSensor.h>
#include <string>
using namespace std;
#include <iostream>

ForceSensor::ForceSensor() {
   SetupStatus = false; // not initialized yet
   CalStatus = false; //not calibrated yet	
   ZeroedStatus = false; // not zeroed yet
}

ForceSensor::~ForceSensor() {}

void ForceSensor::SetupComedi(string comedi_filepath) {

	// Comedi variables
	subdev = 0;
	chan = 0;
	range = 0;

	// Comedi setup and test data read function
	it = comedi_open(comedi_filepath.c_str());
	if(it == NULL) {
		comedi_perror("comedi_open"); exit(0);
	}
	int retval = comedi_data_read(it, subdev, chan, range, aref, &data);
	if(retval < 0) {
		comedi_perror("comedi_data_read"); exit(0);
	}
	range_info = comedi_get_range(it,subdev,chan,range);
	maxdata = comedi_get_maxdata(it,subdev,chan);
	SetupStatus = true; //flag for setup force sensor complete
}

bool ForceSensor::getSetupStatus() {
	return SetupStatus;
}

void ForceSensor::LoadCalibrationMatrix(string cal_matrix_file) {

	FILE *p_file; // pointer to file containing target points
	p_file = fopen(cal_matrix_file.c_str(), "r");

	string textLine; // contents of line in text file
	char *ret_code; // code containing possible error in reading line of text file
	char Data[80]; // data for a line in the text file
	int ret_val; // value containing number of fields copied
	float temp0, temp1, temp2, temp3, temp4, temp5; // temporary variables for data in text file

	if (p_file == NULL) {
		cerr << endl << "Unable to open calibration file: " << cal_matrix_file.c_str() << endl;
		exit(0);
	}

	//CalMatrix.set_size(6,6); // Temporary variable

	fseek(p_file, 0, SEEK_SET); // move pointer back to beginning of file
	int row_index = 0; // index of current point being read
	while(row_index < 6) {
		ret_val = fscanf(p_file,"%f, %f, %f, %f, %f, %f", &temp0, &temp1, &temp2, &temp3, &temp4, &temp5);
		CalMatrix(row_index, 0) = temp0; 
		CalMatrix(row_index, 1) = temp1;
		CalMatrix(row_index, 2) = temp2;
		CalMatrix(row_index, 3) = temp3;
		CalMatrix(row_index, 4) = temp4;
		CalMatrix(row_index, 5) = temp5;
		//cout << "Row " << row_index << " = " << temp0 << ", " << temp1 << ", " << temp2 << ", " << temp3 << ", " << temp4 << ", " << temp5 << endl;
		row_index++;
	}
	CalStatus = true;
	fclose(p_file);
}

bool ForceSensor::getCalStatus() {
	return CalStatus;
}

void ForceSensor::SetBiasVoltages(int nCalSamples) {

	mat BiasSamples(6,nCalSamples);
	BiasSamples.fill(0.0);
	lsampl_t comedi_raw;
	float volt;
	for(int ii = 0; ii < nCalSamples; ii++) {
		for (int chan = 0; chan < 6; chan++) {
			comedi_data_read(it, subdev, chan, range, aref, &comedi_raw); // Get raw data
			volt = (double)comedi_to_phys(comedi_raw, range_info, maxdata); // Convert to V.
			BiasSamples(chan,ii) = volt;
		}
		//printf("\nBias Voltage Samples: [%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]", BiasSamples(0,ii), BiasSamples(1,ii), BiasSamples(2,ii), BiasSamples(3,ii), BiasSamples(4,ii), BiasSamples(5,ii)); 
	}
	BiasVoltages = mean(BiasSamples,1);
	//printf("\nMean Bias Voltages: [%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]\n", BiasVoltages(0,0), BiasVoltages(1,0), BiasVoltages(2,0), BiasVoltages(3,0), BiasVoltages(4,0), BiasVoltages(5,0));
	ZeroedStatus = true;
}

bool ForceSensor::getZeroedStatus() {
	return ZeroedStatus;
}

// Read force sensor
mat ForceSensor::ReadATIForceSensor() {

	mat FT(6,1); // Force/torque data
	mat Voltages(6,1); // voltage
	lsampl_t comedi_raw;

	// Sample all channels once
	for (int chan = 0; chan<6; chan++) {
		comedi_data_read(it, subdev, chan, range, aref, &comedi_raw);
		//printf("\ncomedi_raw(%d) = %d", chan, comedi_raw);
		Voltages(chan,0) = (double)comedi_to_phys(comedi_raw, range_info, maxdata); // Convert to V.
	//printf("\nVolts(%d) = %2.4f", chan, v(chan,0);
	}
	FT = CalMatrix*(Voltages-BiasVoltages);
	return(FT);
}
