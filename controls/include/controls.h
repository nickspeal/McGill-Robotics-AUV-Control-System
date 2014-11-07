#ifndef CONTROLS_H
#define CONTROLS_H

#include "ros/ros.h"
#include <ros/console.h> //to change verbosity of ROSINFO ROS_DEBUG etc
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "planner/setPoints.h"	
#include "computer_vision/VisibleObjectData.h"
#include "controls/DebugControls.h"	
#include <tf/transform_listener.h>


void setPoints_callback(const planner::setPoints setPointsMsg);
void estimatedDepth_callback(const std_msgs::Float64 data);
float output_limit_check(float value, float min, float max, char* value_name);
float saturate(float value, float max, char* value_name);
void getStateFromTF();

 #endif //CONTROLS_H