/********************************************************************
Main file for node that reads the voltage values from the DAQ card 
connected to an ATI force sensor, converts the values to forces (N), 
filters the data, and publishes the data.

Neal P. Dillon
revised:  5/31/2017
********************************************************************/
#include "read_ft_sensor.h"

// Create ForceSensor object
ForceSensor ATI_Sensor; // Class for reading DAQ (using comedi), calculating forces

//Service Function for Zeroing force sensor

bool zeroedFT(ati_ft_sensor::setZeroFT::Request &req, ati_ft_sensor::setZeroFT::Response &res) {
	if (ATI_Sensor.getCalStatus()) {
		ATI_Sensor.SetBiasVoltages(1000);
		res.zeroedFT = ATI_Sensor.getZeroedStatus(); // flag true if actuators are zeroed
        //ROS_INFO("requested zero command: %d", req.zeroFTYESNO); //info stored about request
   		//ROS_INFO("sending back response: %d", res.zeroedFT); //info stored about response
  	}
    return true;
}

int main(int argc, char **argv) {

	// Initialize Node
	ros::init(argc, argv, "read_ft_sensor");
	ros::NodeHandle n;
	ROS_INFO("%s", "Starting node to read force sensor.");

	// Get parameters from launch file
	int fs_pub_freq;
	if (ros::param::has("/read_ft_sensor/fs_pub_freq")) {
		ros::param::get("/read_ft_sensor/fs_pub_freq", fs_pub_freq);
	}
	else {
		fs_pub_freq = 5000; // default value
	}
	string CalFilePath;
	if (ros::param::has("/read_ft_sensor/calibration_matrix_filepath")) {
		ros::param::get("/read_ft_sensor/calibration_matrix_filepath", CalFilePath);
	}
	else {
		//CalFilePath = "/home/caos/catkin_ws/src/ati_ft_sensor/cal_files/CalibrationMatrix_FT8669.cal"; // default filepath for magsteering- this is for nano
		CalFilePath = "/home/caos/catkin_ws/src/ati_ft_sensor/cal_files/CalibrationMatrix_FT25874.cal"; // file for ATI mini
	}

	// Setup publishing of force_data
	ros::Publisher force_data_pub = n.advertise<ati_ft_sensor::force_data>("force_data", 1);
    
	//Set up the service for zeroing the force sensor
	ros::ServiceServer service = n.advertiseService("setZeroFT",zeroedFT); //service created and advertised
	
	// Load calibration file for force sensor
	ATI_Sensor.LoadCalibrationMatrix(CalFilePath); // Load calibration matrix
	ROS_INFO("Calibration Matrix loaded from file: %s.", CalFilePath.c_str());

	// Setup comedi
	ATI_Sensor.SetupComedi("/dev/comedi0");
	ROS_INFO("%s", "Press ENTER to begin sampling voltages to un-bias sensor."); //cin.get();

	//wait for the central node to tell to zero
    ROS_INFO("Ready to zero");
	while (!ATI_Sensor.getZeroedStatus()) {
		ros::spinOnce();
		//ROS_INFO("%s", "Not Zeroed Yet");
	}

	// Loop - read forces
	ati_ft_sensor::force_data msg;
	ros::Rate loop_rate(fs_pub_freq);
	mat FT_vals(6,1);
	ROS_INFO("%s", "Reading force data. Publishing to force_data topic.");
  	while (ros::ok()) {
	
		FT_vals = ATI_Sensor.ReadATIForceSensor(); // Read force data

		// Asign values to msg struct
		msg.Fx = FT_vals(0,0);
		msg.Fy = FT_vals(1,0);
		msg.Fz = FT_vals(2,0);
		msg.Tx = FT_vals(3,0);
		msg.Ty = FT_vals(4,0);
		msg.Tz = FT_vals(5,0);

		// Publish msg
		force_data_pub.publish(msg);
		ROS_INFO("%s = [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", "Force/Torque Values", FT_vals(0,0), FT_vals(1,0), FT_vals(2,0), FT_vals(3,0), FT_vals(4,0), FT_vals(5,0));

		// Spin and Sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
