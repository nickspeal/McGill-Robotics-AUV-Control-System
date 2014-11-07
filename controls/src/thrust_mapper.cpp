/*
Maps thrust to voltage
Maps voltage to motor command
*/

#include "thrust_mapper.h"
#include <math.h>

//For matrix stuff on initialization:
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "invert_matrix.h" //grabbed matrix inversion code from online and saved it in this file in the controls/inculde directory 

//global vars
ros::Publisher voltage_publisher;
ros::Publisher thrust_publisher;
double VOLTAGE_MAX;
int32_t MOTOR_CMD_MAX;
double F_MAX;
double T_MAX;
boost::numeric::ublas::matrix<double> wrench2thrust (5,5);


float saturate(float value, float max, char* value_name) {
	//saturates if out of range
	if (value > max) {
		ROS_DEBUG("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, max);
		value=max;
	}
	else if (value < -1*max){
		ROS_DEBUG("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, -1*max);
		value=-1*max;
	}
	return value;
}


float limit_check(float value, float max, char* value_type, char* value_id){
	//UNUSED METHOD AS OF JUNE 11 2014
	if (value > max | value < -1*max) {
		ROS_WARN("%s: %s value is %f. This exceeds the maximum allowable value of %f.", value_type, value_id, value, max);
		value = 0; 
		return value;
	}
	else {
		return value;
	}
}




float thrust_voltage(float thrust){
	/*
	* Input: desired thrust from a certain thruster
	* Output: voltage to send to that thruster

	* Obtained from thruster characterization experiment with the force-torque sensor, after piecewise best-fit analysis
	* Approximately the same for all the brushed Seabotix BTD150 thrusters
	*/

	float voltage;	
	if (thrust<=-1.05){
        voltage = 1.09*thrust - 3.1542;
    }
    else if (thrust>-1.05 & thrust<=0){
        //voltage = -4.3968*(-1*thrust)^0.4534;
        voltage = -4.3968*pow(-1*thrust, 0.4534);
    }
    else if (thrust>0 & thrust<=1.25){
        //voltage = 4.3968*thrust^0.4534;
        voltage = 4.3968*pow(thrust, 0.4534);
    }
    else if (thrust > 1.25){
        voltage = thrust + 3.6857;
    }
	return voltage;
}

void thrust_callback(geometry_msgs::Wrench wrenchMsg)
{	
	/*
	* Input: net wrench to apply to robot (or that the robot applies on the environment)
	* computes the desired thrust for each thruster
	* calculates the corresponding voltage via function call
	* calculates the corresponding motor command for these voltages
	*	-Motor controller characterization results are applied here
	*	-offsets are implemented on the arduino (adding or subtracting yintercept)	
	* Output: motor commands in array form
	*/

	float thrust[6] = {0, 0, 0, 0, 0, 0};
	float voltage[6] = {0, 0, 0, 0, 0, 0};
	int32_t motor_cmd[6] = {0, 0, 0, 0, 0, 0};

	controls::motorCommands motorCommands;
	boost::numeric::ublas::vector<double> wrenchVector (5);
	boost::numeric::ublas::vector<double> thrustsVector (5);

	//Limit check for input wrench values
	wrenchVector(0)=limit_check(wrenchMsg.force.x, F_MAX, "Net Force", "X");
	wrenchVector(1)=limit_check(wrenchMsg.force.y, F_MAX, "Net Force", "Y");
	wrenchVector(2)=limit_check(wrenchMsg.force.z, F_MAX, "Net Force", "Z");
	wrenchVector(3)=limit_check(wrenchMsg.torque.y, T_MAX, "Net Torque", "Y");
	wrenchVector(4)=limit_check(wrenchMsg.torque.z, T_MAX, "Net Torque", "Z");

	//thrust allocation - the wrench2thrust matrix is created in the main function, according to the geometry of the thrusters, so that matrix inversion only has to happen once
	thrustsVector = prod(wrench2thrust, wrenchVector); //matrix multiplication. Not sure where this is defined. I would have thought I would have needed the namespace prefix for boost...
	thrust[0] = thrustsVector(0);			//surge-starbord
	thrust[1] = thrustsVector(1);			//surge-port
	thrust[2] = thrustsVector(2);			//Sway-Bow
	thrust[3] = thrustsVector(2);			//sway-stern = sway bow
	thrust[4] = thrustsVector(3);			//heave-bow
	thrust[5] = thrustsVector(4);    		//heave-stern
	
	//Account for direction of mounting the thrusters	
	/*
	Directions of thrusters (As of April 14, 2014)
		Surge thrusters positive forward
		Heave thrusters positive upward (should be switched though to avoid splashing at the surface)
		Sway-Bow points in the negative-y direction (applies a negative force and negative torque)
		Sway-Stern points in the positive-y direction (applies a positive force and negative torque)
		
	Propeller shroud is in the direction of positive force (send positive motor command and the robot will move in the direction of the shroud)
	I think this changed in May to keep up with the "world moving around the robot" convention
	*/

	int directions[] = {-1, -1, 1, -1, 1, 1}; //the math assumes thrusters apply force in their positive coordinate directions. -1 here if oriented otherwise. Note that cenvention dictates that the positive force is applied to the environment, not the robot.
	for (int i=0; i<6; i++)
	{
		thrust[i]*=directions[i];
	}
	
	//for each thrust, map to a voltage, considering saturation
	char* voltage_name[] {"one", "two", "three", "four", "five", "six"}; //for limit check error catching
 	for (int i=0; i<6; i++)
	{
		voltage[i] = thrust_voltage(thrust[i]);
		//voltage[i] = limit_check(voltage[i], VOLTAGE_MAX, "VOLTAGE", voltage_name[i]);
		//ROS_DEBUG("Voltage %i: %f",i,voltage[i]);
	}	

	//map voltages to motor commands
	// y intercepts moved because the offsets will be done on the arduino
	// based on experimental data from motor controller characterization processed in excel. Ask Bei.
	//Updated in May or April or something. NOT YET UPDATED FOR ASIMOV 2.0
	motor_cmd[0] = 21.176*voltage[0];
	motor_cmd[1] = 21.2*voltage[1];
	motor_cmd[2] = 20.686*voltage[2];
	motor_cmd[3] = 20.704*voltage[3];
	motor_cmd[4] = 20.583*voltage[4];
	motor_cmd[5] = 20.63*voltage[5];

	char* motor_name[] {"Motor 1: surge-starbord", "Motor 2: surge-port", "Motor 3: sway-bow", "Motor 4: sway-stern", "Motor 5: heave-bow", "Motor 6: heave-stern"};
	for (int i=0; i<6; i++)
	{	
		motor_cmd[i] = saturate(motor_cmd[i], MOTOR_CMD_MAX, motor_name[i]);
	}

	motorCommands.cmd_surge_starboard=motor_cmd[0];
	motorCommands.cmd_surge_port=motor_cmd[1];
	motorCommands.cmd_sway_bow=motor_cmd[2];
	motorCommands.cmd_sway_stern=motor_cmd[3];
	motorCommands.cmd_heave_bow=motor_cmd[4];
	motorCommands.cmd_heave_stern=motor_cmd[5];

	//publish
	voltage_publisher.publish(motorCommands);
}



int main(int argc, char **argv)
{
	// ROS subscriber setup
	ros::init(argc,argv,"thrust_mapper");
	ros::NodeHandle n;

	//Parameters
	n.param<double>("voltage/max", VOLTAGE_MAX, 0.0);
	n.param<int32_t>("motorCommands/max", MOTOR_CMD_MAX, 0.0);
	n.param<double>("force/max", F_MAX, 0.0);
	n.param<double>("torque/max", T_MAX, 0.0);

	if (VOLTAGE_MAX == 0.0){ROS_ERROR("PARAMETER FILE DID NOT LOAD IN THRUST_MAPPER");}
	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	

	//ROS Publisher setup
	voltage_publisher = n.advertise<controls::motorCommands>("/electrical_interface/motor", 100); 
	/*
	** Calculations for the allocation of thrust to each thruster **
	----------------------------------------------------------------
	- Accounts for the geometry of the robot and the thruster placement
	- This is the most complicated part of the code, and is thoroughly explained on the wiki. Consult with Nick if you have questions

	Order of thrust array
		0,1,2,3,4,5  :  x1,x2,y1,y2,z1,z2 :  surge-starbord, surge-port, sway-bow, sway-stern, heave-bow, heave-stern
	*/

	//approx distances from center of AUV to thrusters. In meters; signs determined by NED coordinate system
	double rx1= 0.2;
	double rx2= -0.2;
	double ry1 = 0.4;
	double ry2 = -0.4;
	double rz1 = 0.4;
	double rz2 = -0.4;

	boost::numeric::ublas::matrix<double> thrust2wrench (5,5);
	//set up matrix for the case of 6 thrusters. See Nick's design notebook pg 169, scanned on the wiki
	//NOTE WHEN CHANGING THIS MATRIX - MAKE SURE IT IS INVERTABLE!
	//Multiplying this matrix by the thrust vector would return the wrench vector, hence the name
	//We want the inverse so we can calculate the desired thrust vector
	thrust2wrench(0,0) = 1; thrust2wrench(0,1) = 1; thrust2wrench(0,2) = 0; thrust2wrench(0,3) = 0; thrust2wrench(0,4) = 0;
	thrust2wrench(1,0) = 0; thrust2wrench(1,1) = 0; thrust2wrench(1,2) = 2; thrust2wrench(1,3) = 0; thrust2wrench(1,4) = 0;
	thrust2wrench(2,0) = 0; thrust2wrench(2,1) = 0; thrust2wrench(2,2) = 0; thrust2wrench(2,3) = 1; thrust2wrench(2,4) = 1;
	thrust2wrench(3,0) = 0; thrust2wrench(3,1) = 0; thrust2wrench(3,2) = 0; thrust2wrench(3,3) = -rz1; thrust2wrench(3,4) = -rz2;
	thrust2wrench(4,0) = -rx1; thrust2wrench(4,1) = -rx2; thrust2wrench(4,2) = ry1+ry2; thrust2wrench(4,3) = 0; thrust2wrench(4,4) = 0;

	//wrench2thrust globally defined, for use in thrust_callback
	InvertMatrix(thrust2wrench,wrench2thrust); //Invert the first matrix argument and save it as the second.
	//std::cout << wrench2thrust << std::endl;


	ROS_INFO("Thrust_mapper initialized. Listening for wrench.");
	ros::spin();
	return 0;
}