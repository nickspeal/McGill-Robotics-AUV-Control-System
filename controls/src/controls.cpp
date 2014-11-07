#include "controls.h"
#define PI 3.14159265358979323846
/*
5DOF control system for the McGill Robotics AUV
Subscribes to setpoints and estimated states and publishes a net wrench to minimize error.
Closed loop position control, open loop speed control.

Created by Nick Speal Jan 10 2014.
*/

//global vars

	double z_des = 0;
	double z_est = 0;

	double setPoint_XPos = 0;
	double setPoint_YPos = 0;
	double setPoint_Depth = 0;
	double setPoint_Yaw = 0;
	double setPoint_Pitch = 0;
	double setPoint_XSpeed = 0;
	double setPoint_YSpeed = 0;
	double setPoint_YawSpeed = 0;
	double setPoint_DepthSpeed = 0;

	int8_t isActive_XPos = 0;
	int8_t isActive_YPos = 0;
	int8_t isActive_Depth = 0;
	int8_t isActive_Yaw = 0;
	int8_t isActive_Pitch = 0;
	int8_t isActive_XSpeed = 0;
	int8_t isActive_YSpeed = 0;
	int8_t isActive_YawSpeed = 0;
	int8_t isActive_DepthSpeed = 0;

	std::string frame = ""; //default

	double estimated_XPos = 0;
	double estimated_YPos = 0;
	double depth = 0;
	double estimated_Pitch = 0;
	double estimated_Yaw = 0;

	double F_MAX;
	double T_MAX;

	double MAX_ERROR_X_P;
	double MAX_ERROR_Y_P;
	double MAX_ERROR_Z_P;
	double MAX_ERROR_PITCH_P;
	double MAX_ERROR_YAW_P;

	double MAX_ERROR_X_D;
	double MAX_ERROR_Y_D;
	double MAX_ERROR_Z_D;
	double MAX_ERROR_PITCH_D;
	double MAX_ERROR_YAW_D;

	double XSPEED_MAX;
	double YSPEED_MAX;

	double YAW_MAX;
	double PITCH_MAX;

	//Error Variables
    double ep_XPos = 0; //error
    double ei_XPos = 0; //integral error
    double ed_XPos = 0; //derivative error
    double ep_XPos_prev = 0; //proportional error at last timestep
    double ep_YPos = 0;
    double ei_YPos = 0;
    double ed_YPos = 0;
    double ep_YPos_prev = 0;
    double ep_Depth = 0;
    double ei_Depth = 0;
    double ed_Depth = 0;
    double ep_Depth_prev = 0;
    double ep_Pitch = 0;
    double ei_Pitch = 0;
    double ed_Pitch = 0;
    double ep_Pitch_prev = 0;
    double ep_Yaw = 0;
    double ei_Yaw = 0;
    double ed_Yaw = 0;
    double ep_Yaw_prev = 0;


	bool killed = false;
	ros::Time timeOfLastSetPoint; //for checking if setPoints is still publishing
	planner::setPoints oldSetPointsMsg; //for checking if setPoints has changed.

void soft_kill_callback(const std_msgs::Bool data)
{
	//catches softkill or softstart command
	//if killed == True, then publish zero wrench instead of calculated values. Values are calculated anyway.

	killed = data.data;
}

void setPoints_callback(const planner::setPoints setPointsMsg)
{

	//check if setPoints has changed
	/*
	if (oldSetPointsMsg == setPointsMsg){
		//reset integral errors
	    double ei_XPos = 0; //integral error
	    double ei_YPos = 0;
	    double ei_Depth = 0;
	    double ei_Pitch = 0;
	    double ei_Yaw = 0;
		oldSetPointsMsg = setPointsMsg;
	}
	*/
	timeOfLastSetPoint = ros::Time::now();
	double currentTime = timeOfLastSetPoint.toSec();
	ROS_DEBUG("received setPoints - currentTime: %f", currentTime);

	//save message data into global variables
	setPoint_XPos = setPointsMsg.XPos.data;
	setPoint_YPos = setPointsMsg.YPos.data;
	setPoint_Depth = setPointsMsg.Depth.data;
	setPoint_Yaw = setPointsMsg.Yaw.data;
	setPoint_Pitch = setPointsMsg.Pitch.data;
	setPoint_XSpeed = setPointsMsg.XSpeed.data;
	setPoint_YSpeed = setPointsMsg.YSpeed.data;
	setPoint_YawSpeed = setPointsMsg.YawSpeed.data;
	setPoint_DepthSpeed = setPointsMsg.DepthSpeed.data;

	isActive_XPos = setPointsMsg.XPos.isActive;
	isActive_YPos = setPointsMsg.YPos.isActive;
	isActive_Depth = setPointsMsg.Depth.isActive;
	isActive_Yaw = setPointsMsg.Yaw.isActive;
	isActive_Pitch = setPointsMsg.Pitch.isActive;
	isActive_XSpeed = setPointsMsg.XSpeed.isActive;
	isActive_YSpeed = setPointsMsg.YSpeed.isActive;
	isActive_YawSpeed = setPointsMsg.YawSpeed.isActive;
	isActive_DepthSpeed = setPointsMsg.DepthSpeed.isActive;
	frame = setPointsMsg.Frame;

}

void depth_callback(const std_msgs::Float64 data)
{
	depth = data.data;
	double currentTime = ros::Time::now().toSec();
	ROS_DEBUG("Received depth - currentTime: %f", currentTime);
}

float output_limit_check(float value, float min, float max, char* value_name ){ 
	//DEPRECATED FUNCTION? MIGHT BE ABLE TO DELETE THIS
	//returns zero and warns if out of range
	if (value > max | value < min) { //out of range
		ROS_WARN("%s: value has been exceeded. Value is %f. Setting to 0.", value_name, value);
		value = 0;
		return value;
	}
	else {
		return value;
	}
}

float saturate(float value, float max, char* value_name) {
	//saturates if out of range
	if (value > max) {
		ROS_DEBUG("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, max);
		value=max;
	}
	else if (value < -1*max)
	{
		ROS_DEBUG("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, -1*max);
		value=-1*max;
	}

	return value;
}


void getStateFromTF()
{
	/*
	* Looks up the TF and saves local variables for estimated position.
	* First looks up TF for x,y,yaw between the robot/rotation_center and an arbitrary frame
	* Then looks up TF for pitch seperately because the frame is not arbitrary - always want pitch with respect to the horizon
	*/
	if (frame == "")
	{
		ROS_WARN("No frame specified in setPoints");
		return;
	}
	tf::StampedTransform transform;
	tf::TransformListener tf_listener;

	std::string targetFrame = frame; //specified on the setPoints topic
	std::string originalFrame = "robot/rotation_center"; //rpy,xyz of target frame is expressed w.r.t. the original frame axes

	try
	{
		double currentTime = ros::Time::now().toSec();
		ROS_DEBUG("waiting for transform- currentTime: %f", currentTime);
		tf_listener.waitForTransform(targetFrame, originalFrame, ros::Time(0), ros::Duration(0.4)); //not sure what an appropriate time to wait is. I wanted to wait less than the target 2 Hz.
		currentTime = ros::Time::now().toSec();
		ROS_DEBUG("done waiting for transform - currentTime: %f", currentTime);
		tf_listener.lookupTransform(targetFrame, originalFrame, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}


	estimated_XPos = transform.getOrigin().x();
	estimated_YPos = transform.getOrigin().y();
	//ignore depth. it comes in on its own topic

	tf::Quaternion q = transform.getRotation(); //save the rotation as a quaternion
	tf::Matrix3x3 m(q); //convert quaternion to matrix

	double roll_unused; //unused, but needs to be sent to getRPY method
	double pitch_unused;


	m.getEulerYPR(estimated_Yaw, pitch_unused, roll_unused); //in order to determine yaw, the method also needs to return roll and pitch. ignore those.

	if (frame=="/target/lane"){ estimated_Yaw*=-1;} //At the last minute, Anass and Nick noticed that the sign convention was backwards somewhere only for the lane target. This was a quick fix right before comp.





	//------------------------------------------------------------------------------------------------------------
	//Pitch and not all other dimensions
	//Repeat most of the first half of this method for the different coordinate frame

	tf::TransformListener tf_listener_pitch;

	targetFrame = "sensors/IMU_global_reference";
	originalFrame = "robot/rotation_center";

	try
	{
		double currentTime = ros::Time::now().toSec();
		ROS_DEBUG("waiting for pitch transform- currentTime: %f", currentTime);
		tf_listener_pitch.waitForTransform(targetFrame, originalFrame, ros::Time(0), ros::Duration(0.4)); //not sure what an appropriate time to wait is. I wanted to wait less than the target 2 Hz.
		currentTime = ros::Time::now().toSec();
		ROS_DEBUG("done waiting for pitch transform - currentTime: %f", currentTime);
		tf_listener_pitch.lookupTransform(targetFrame, originalFrame, ros::Time(0), transform); //overwrite transform object
	}
	catch (tf::TransformException ex){
		ROS_ERROR("error looking up pitch transform: \n %s",ex.what());
	}

	q = transform.getRotation(); //save the rotation as a quaternion - overwrite the one defined earlier because it is no longer needed
	tf::Matrix3x3 matrix_for_pitch(q); //convert quaternion to matrix - overwrite this too, same reason.

	double yaw_unused;
	matrix_for_pitch.getEulerYPR(yaw_unused, estimated_Pitch, roll_unused);
}






int main(int argc, char **argv)
{
	//Create ROS Node
	ros::init(argc,argv,"controls");
	ros::NodeHandle n;

	//specify ROSCONSOLE verbosity level
	//Fred Lafrance: temporarily commented because of compile error - not compatible with old ROS installations
	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
	//{
   	//	ros::console::notifyLoggerLevelsChanged();
	//}

	//define variables for Parameters
		double m; //mass in kg
		double g; //acceleration of gravity
		double cd; // drag coefficient
		double buoyancy; // percent buoyancy
		double dt; //theoretical time elapsed between iterations through the main loop. TODO update this dynamically by measuring time elapsed. Currently parameterized

		//Gains for the Proportional, Integral, and Derivative controllers
	    double kp_xPos;
	    double ki_xPos;
	    double kd_xPos;

	    double kp_yPos;
	    double ki_yPos;
	    double kd_yPos;

	    double kp_Depth;
	    double ki_Depth;
	    double kd_Depth;

	    double kp_Yaw;
	    double ki_Yaw;
	    double kd_Yaw;

	    double kp_Pitch;
	    double ki_Pitch;
	    double kd_Pitch;

	    double OL_coef_x;	//set later in the controller
	    double OL_coef_y;
	    double OL_coef_depth;
	    double OL_coef_yaw;
	    double OL_coef_balance; // used to balance surge thrusters so the vehicle goes straight before yaw control, given as a whole number percent, off of evenly balanced

		double z_steady_force;
		double pitch_steady_torque;


	//Define wrench variables. They get reinitialized to zero on each iteration.
	    double Fx = 0;
	    double Fy = 0;
	    double Fz = 0;
	    double Ty = 0;
	    double Tz = 0;

	ros::Duration durationSinceLastSetPoint;

	// ROS subscriber setup
	ros::Subscriber setPoints_subscriber = n.subscribe("setPoints", 1000, setPoints_callback);
	ros::Subscriber depth_subscriber = n.subscribe("/state_estimation/filteredDepth", 1000, depth_callback);
	ros::Subscriber softKill_subscriber = n.subscribe("/controls/softKill", 1000, soft_kill_callback);

	//ROS Publisher setup
	ros::Publisher wrench_publisher = n.advertise<geometry_msgs::Wrench>("/controls/wrench", 100);
	geometry_msgs::Wrench wrenchMsg; //define message variable to publish
	ros::Publisher debug_publisher = n.advertise<controls::DebugControls>("/controls/debug", 100);
	controls::DebugControls debugMsg;




	ROS_INFO("\n----\n Controls node initialized. Waiting for setPoints to be published... \n----");
	while (setPoints_subscriber.getNumPublishers() == 0)
	{
		ROS_DEBUG_THROTTLE(2,"Waiting...");
		ros::Duration(0.5).sleep(); //sleep for this many seconds
	}
	timeOfLastSetPoint = ros::Time::now();
	ROS_INFO("setPoints message has been detected. Starting Controller!");
	ros::Rate loop_rate(1.0/dt);
	while(ros::ok())
	{

	    //ROS Params

		    n.param<double>("gains/kp_xPos", kp_xPos, 0.0);
		    n.param<double>("gains/ki_xPos", ki_xPos, 0.0);
		    n.param<double>("gains/kd_xPos", kd_xPos, 0.0);

		    n.param<double>("gains/kp_yPos", kp_yPos, 0.0);
		    n.param<double>("gains/ki_yPos", ki_yPos, 0.0);
		    n.param<double>("gains/kd_yPos", kd_yPos, 0.0);

		    n.param<double>("gains/kp_Depth", kp_Depth, 0.0);
		    n.param<double>("gains/ki_Depth", ki_Depth, 0.0);
		    n.param<double>("gains/kd_Depth", kd_Depth, 0.0);

		    n.param<double>("gains/kp_Pitch", kp_Pitch, 0.0);
		    n.param<double>("gains/ki_Pitch", ki_Pitch, 0.0);
		    n.param<double>("gains/kd_Pitch", kd_Pitch, 0.0);

		    n.param<double>("gains/kp_Yaw", kp_Yaw, 0.0);
		    n.param<double>("gains/ki_Yaw", ki_Yaw, 0.0);
		    n.param<double>("gains/kd_Yaw", kd_Yaw, 0.0);


		    n.param<double>("coefs/mass", m, -1); //default negative to check for proper loading
		    n.param<double>("coefs/buoyancy", buoyancy, 0.02);
		    n.param<double>("coefs/drag", cd, 0.0);
		    n.param<double>("coefs/gravity", g, 9.81);
		    n.param<double>("coefs/dt", dt, 0.01);

			n.param<double>("force/max", F_MAX, 0.0);
			n.param<double>("torque/max", T_MAX, 0.0);

			n.param<double>("max_error_x_p", MAX_ERROR_X_P, 0.0);
			n.param<double>("max_error_y_p", MAX_ERROR_Y_P, 0.0);
			n.param<double>("max_error_z_p", MAX_ERROR_Z_P, 0.0);
			n.param<double>("max_error_pitch_p", MAX_ERROR_PITCH_P, 0.0);
			n.param<double>("max_error_yaw_p", MAX_ERROR_YAW_P, 0.0);

			n.param<double>("max_error_x_d", MAX_ERROR_X_D, 0.0);
			n.param<double>("max_error_y_d", MAX_ERROR_Y_D, 0.0);
			n.param<double>("max_error_z_d", MAX_ERROR_Z_D, 0.0);
			n.param<double>("max_error_pitch_d", MAX_ERROR_PITCH_D, 0.0);
			n.param<double>("max_error_yaw_d", MAX_ERROR_YAW_D, 0.0);

			n.param<double>("XSpeed/max", XSPEED_MAX, 0.0);
			n.param<double>("YSpeed/max", YSPEED_MAX, 0.0);

			n.param<double>("Pitch/max", PITCH_MAX, 0.0);
			n.param<double>("Yaw/max", YAW_MAX, 0.0);

		    n.param<double>("gains/OL_coef_x", OL_coef_x, 0.0);
		    n.param<double>("gains/OL_coef_y", OL_coef_y, 0.0);
		    n.param<double>("gains/OL_coef_yaw", OL_coef_yaw, 0.0);
		    n.param<double>("gains/OL_coef_depth", OL_coef_depth, 0.0);
		    n.param<double>("gains/OL_coef_balance", OL_coef_balance, 0.0); // used to balance surge thrusters

		    n.param<double>("z_steady_force", z_steady_force, 0);
		    n.param<double>("pitch_steady_torque", pitch_steady_torque, 0);

			if (m<0){ROS_ERROR("PARAMETERS DID NOT LOAD IN CONTROLS.CPP");}


		//make sure all DOF are not controlled unless setPoints says so. No control if no setPoints message recently received
		durationSinceLastSetPoint = ros::Time::now() - timeOfLastSetPoint;
		//ROS_INFO("Duration since last set point: %f", durationSinceLastSetPoint.toSec());
		if (durationSinceLastSetPoint.toSec() > 3) {
			if (durationSinceLastSetPoint.toSec() <6){ //nested if so that this message isn't repeated forever and ever. TODO replace with ROS_INFO_ONCE or soemthing to that effect - look look it up
				ROS_INFO("No recent setPoint. Pausing Controls.");
			}
			isActive_XPos = 0;
			isActive_YPos = 0;
			isActive_Depth = 0;
			isActive_Yaw = 0;
			isActive_Pitch = 0;
			isActive_XSpeed = 0;
			isActive_YSpeed = 0;
			isActive_YawSpeed = 0;
			isActive_DepthSpeed = 0;
			//frame = ""; no need to reset the frame. 
			/*
			Reset integral errors - not sure if this should be done.
			ei_XPos = 0;
			ei_YPos = 0;
			ei_Depth = 0;
			ei_Yaw = 0;
			ei_Pitch = 0;
			*/
		}

		double currentTime = ros::Time::now().toSec();
		ROS_DEBUG("\n--\nTop of main loop - currentTime: %f", currentTime);
		ros::spinOnce();	//Updates all variables
		getStateFromTF();
		currentTime = ros::Time::now().toSec();

		//zero wrench unless otherwise specified
		Fx = 0;
    	Fy = 0;
    	Fz = 0;
    	Ty = 0;
    	Tz = 0;

		//X
		if (isActive_XPos)
		{
			if (true) // TODO check if estimatedStateIsPublished
			{
				ep_XPos_prev = ep_XPos;
				ep_XPos = setPoint_XPos - estimated_XPos;
				ep_XPos=saturate(ep_XPos, MAX_ERROR_X_P, "X Proportional Error");
				ei_XPos += ep_XPos*dt;
				ed_XPos = (ep_XPos - ep_XPos_prev)/dt;
				ed_XPos = saturate(ed_XPos, MAX_ERROR_X_D, "X Derivative Error");
				Fx = kp_xPos*ep_XPos + ki_xPos*ei_XPos + kd_xPos*ed_XPos;
			}
			else
			{
				ROS_WARN("X-Position Control is active, but estimated state is not published");
			}
		}

        if (isActive_XSpeed)
        {
           	setPoint_XSpeed=saturate(setPoint_XSpeed, XSPEED_MAX, "X Speed");
        	Fx = OL_coef_x*setPoint_XSpeed;
        	Tz = OL_coef_balance*.01*setPoint_XSpeed; // add a small torque to balance surge thrusters
        }

        //Y
		if (isActive_YPos)
		{
			if (true) // TODO check if estimatedStateIsPublished
			{
				ep_YPos_prev = ep_YPos;
				ep_YPos = setPoint_YPos - estimated_YPos;
				ep_YPos=saturate(ep_YPos, MAX_ERROR_Y_P, "Y Position Error term");
				ei_YPos += ep_YPos*dt;
				ed_YPos = (ep_YPos - ep_YPos_prev)/dt;
				Fy = kp_yPos*ep_YPos + ki_yPos*ei_YPos + kd_yPos*ed_YPos;
			}
			else
			{
				ROS_WARN("Y-Position Control is active, but estimated state is not published");
			}
        }
        if (isActive_YSpeed)
        {
        	setPoint_YSpeed=saturate(setPoint_YSpeed, YSPEED_MAX, "Y Speed");
        	Fy = OL_coef_y*setPoint_YSpeed;
        }

        //Z
		if (isActive_Depth)
		{
			if (depth_subscriber.getNumPublishers() > 0)
			{
				//ROS_DEBUG("setPointDepth: %f | estimatedDepth: %f", setPoint_Depth, depth);
				ep_Depth_prev = ep_Depth;
				ep_Depth = setPoint_Depth - depth;
				ei_Depth += ep_Depth*dt;
				ed_Depth = (ep_Depth - ep_Depth_prev)/dt;
				Fz = kp_Depth*ep_Depth + ki_Depth*ei_Depth + kd_Depth*ed_Depth;
				Fz += z_steady_force; //account for positive buoyancy bias
                //Fz += buoyancy*m*g; //Account for positive buoyancy bias, deprecated
				Fz*=-1; //Depth is different from all the other coordinates. Depth increases as the surface moves upwards, which is the opposite to our NED convention

			}
			else
			{
				ROS_WARN("Depth position Control is active, but /state_estimation/filteredDepth is not published");
			}
		}

		if (isActive_DepthSpeed)
		{
        	Fz = OL_coef_depth*setPoint_DepthSpeed; 
		}



		//Pitch
		if (isActive_Pitch)
		{
			ep_Pitch_prev = ep_Pitch;
			ep_Pitch = setPoint_Pitch - estimated_Pitch;
			ei_Pitch += ep_Pitch*dt;
			ed_Pitch = (ep_Pitch - ep_Pitch_prev)/dt;
			Ty = kp_Pitch*ep_Pitch + ki_Pitch*ei_Pitch + kd_Pitch*ed_Pitch;
			Ty+=pitch_steady_torque;
		}

		//Yaw
		if (isActive_Yaw)
		{
	       	setPoint_Yaw=saturate(setPoint_Yaw, YAW_MAX, "Yaw");
			ep_Yaw_prev = ep_Yaw;
			ep_Yaw = setPoint_Yaw - estimated_Yaw;
			if (ep_Yaw > PI){
				ep_Yaw -= 2*PI;
			}
			else if (ep_Yaw < -1*PI){
				ep_Yaw += 2*PI;
			}
			ep_Yaw = saturate(ep_Yaw, MAX_ERROR_YAW_P, "Yaw Proportional Error");
			ei_Yaw += ep_Yaw*dt;
			ed_Yaw = (ep_Yaw - ep_Yaw_prev)/dt;
			Tz = kp_Yaw*ep_Yaw + ki_Yaw*ei_Yaw + kd_Yaw*ed_Yaw;
		}

		if (isActive_YawSpeed)
        {
        	Tz = OL_coef_yaw*setPoint_YawSpeed;
        }
		//Limit check for output force/torque values - not meaningful saturation as of July 11 because the limits are intentionally super high. Thrusts will be saturated downstream.
			Fx=saturate(Fx, F_MAX, "Force: X");
			Fy=saturate(Fy, F_MAX, "Force: Y");
			Fz=saturate(Fz, F_MAX, "Force: Z");
			Ty=saturate(Ty, T_MAX, "Torque: Y");
			Tz=saturate(Tz, T_MAX, "Torque: Z");

			if (killed == 1)
			{
				//See documentation on SoftKill. Overwrite forces and torques with zero if killed.
				Fx = 0; Fy = 0; Fz = 0; Ty = 0; Tz = 0;
				ROS_DEBUG("Controls Soft Kill Received - Publishing Zero Wrench");
			}

		// Assemble Wrench Message
			wrenchMsg.force.x = Fx;
			wrenchMsg.force.y = Fy;
			wrenchMsg.force.z = Fz;
			wrenchMsg.torque.x = 0;	//no active roll control
			wrenchMsg.torque.y = Ty;
			wrenchMsg.torque.z = Tz;

		// Assemble Debug Message

			//Estimated Positions

				debugMsg.estimated_x = estimated_XPos;
				debugMsg.estimated_y = estimated_YPos;
				debugMsg.estimated_depth = depth;
				debugMsg.estimated_yaw = estimated_Yaw;
				debugMsg.estimated_pitch = estimated_Pitch; //TODO add reference frame to var.

			// Error
				debugMsg.xError.proportional = ep_XPos;
				debugMsg.yError.proportional = ep_YPos;
				debugMsg.depthError.proportional = ep_Depth;
				debugMsg.pitchError.proportional = ep_Pitch;
				debugMsg.yawError.proportional = ep_Yaw;

				debugMsg.xError.integral = ei_XPos;
				debugMsg.yError.integral = ei_YPos;
				debugMsg.depthError.integral = ei_Depth;
				debugMsg.pitchError.integral = ei_Pitch;
				debugMsg.yawError.integral = ei_Yaw;

				debugMsg.xError.derivative = ed_XPos;
				debugMsg.yError.derivative = ed_YPos;
				debugMsg.depthError.derivative = ed_Depth;
				debugMsg.pitchError.derivative = ed_Pitch;
				debugMsg.yawError.derivative = ed_Yaw;

			// Forces
				debugMsg.xForce.proportional = kp_xPos*ep_XPos;
				debugMsg.yForce.proportional = kp_yPos*ep_YPos;
				debugMsg.depthForce.proportional = kp_Depth*ep_Depth;
				debugMsg.pitchForce.proportional = kp_Pitch*ep_Pitch;
				debugMsg.yawForce.proportional = kp_Yaw*ep_Yaw;

				debugMsg.xForce.integral = ki_xPos*ei_XPos;
				debugMsg.yForce.integral = ki_yPos*ei_YPos;
				debugMsg.depthForce.integral = ki_Depth*ei_Depth;
				debugMsg.pitchForce.integral = ki_Pitch*ei_Pitch;
				debugMsg.yawForce.integral = ki_Yaw*ei_Yaw;

				debugMsg.xForce.derivative = kd_xPos*ed_XPos;
				debugMsg.yForce.derivative = kd_yPos*ed_YPos;
				debugMsg.depthForce.derivative = kd_Depth*ed_Depth;
				debugMsg.pitchForce.derivative = kd_Pitch*ed_Pitch;
				debugMsg.yawForce.derivative = kd_Yaw*ed_Yaw;

		wrench_publisher.publish(wrenchMsg);
		debug_publisher.publish(debugMsg);
		loop_rate.sleep();
	} //end while ros ok
	return 0;
} //end int main
