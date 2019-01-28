/********************************************************************
Header file for read_ft_sensor node
Neal P. Dillon
revised:  5/31/2017
********************************************************************/

#ifndef READ_FT_SENSOR_INCLUDED

#define READ_FT_SENSOR_INCLUDED

#include <comedilib.h>
#include <armadillo>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "ForceSensor.h"
#include "std_msgs/String.h"
#include <ati_ft_sensor/force_data.h> // <-- header that contains the declaration of 
                                            //     ati_forcesensor::force_readings
#include "ati_ft_sensor/setZeroFT.h"

using namespace arma;
using namespace std;

#endif



