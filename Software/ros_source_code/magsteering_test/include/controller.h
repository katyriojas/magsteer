#include "ros/ros.h"
#include "smaract/act_data.h"
#include "smaract/setZeroActs.h"
#include "smaract/moveRelative.h"
#include "smaract/moveAbsolute.h"
#include "smaract/movetoLimit.h"
#include <termios.h> // next three includes needed for kbhit()
#include <unistd.h>
#include <fcntl.h>
#include "iostream"
using namespace std;
#include "ati_ft_sensor/force_data.h"
#include "read_ft_sensor.h"
#include "ati_ft_sensor/setZeroFT.h"

// Function prototypes
int kbhit(void);
void act_dataCallback(const smaract::act_data::ConstPtr &msg);
void force_dataCallback(const ati_ft_sensor::force_data::ConstPtr &msg);
