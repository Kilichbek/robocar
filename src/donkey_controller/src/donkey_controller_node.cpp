#include <ros/ros.h>
#include <ros/console.h>
#include <map>
#include <string>
#include <iostream>
#include <cmath>
// messages used for drive movement topic
#include <sensor_msgs/Joy.h>
// messages used for the absolute and proportional movement topics
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include "std_msgs/String.h"

// Steering Constants
const int MAX_STEERING_PULSE = 420;
const int MIN_STEERING_PULSE = 200;
const int CENTER_STEERING_PULSE = 310;
const int STEERING_ACTUATOR_CHANNEL = 2;

// Throttle Constants
const int MIN_THROTTLE_PULSE = 230;
const int MAX_THROTTLE_PULSE = 390;
const int CENTER_THROTTLE_PULSE = 310;
const int THROTTLE_ACTUATOR_CHANNEL = 1;

/** ServoConverter Class

	Converts command values between [-1,1]
	into PMW values between [MIN_PULSE, MAX_PULSE]
*/

class ServoConverter
{
public:
	ServoConverter(int channel, int min, int center, int max);

	void convert_and_update(float value_in);
	int value_out, channel;

private:
	float half_range_;
	int dir_, range_;
	int center_val_;
};



/** DonkeyController Class

	Gets command values from a joystick node or
	autopilot node and publishes Servo values to
	"\servo_absolute" node
*/

class DonkeyController
{
public:

	DonkeyController();
	~DonkeyController();

private:
	ros::NodeHandle nh_;
	ros::Publisher servo_array_pub_;
	ros::Subscriber joy_sub_;
	int linear_, angular_;
	float throttle_scale_, steering_scale_;
	float steering_angle_, throttle_;
	i2cpwm_board::ServoArray msg;
	std::map<std::string, ServoConverter*> actuators;

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void ai_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void increase_max_throttle(void);
	void decrease_max_throttle(void);
	void send_servo_msg(void);
	void send_callback(const sensor_msgs::Joy::ConstPtr& joy);
};



DonkeyController::DonkeyController() :
	steering_angle_(0.0), throttle_(0.0),
	linear_(1), angular_(2),
	throttle_scale_(1.0), steering_scale_(0.98)
{

	ROS_INFO("Setting Up the DonkeyController Node...");
	for (int i = 0; i < 2; i++) {
		msg.servos.push_back(i2cpwm_board::Servo());
	}
	actuators["throttle"] = new ServoConverter(
		THROTTLE_ACTUATOR_CHANNEL,
		MIN_THROTTLE_PULSE,
		CENTER_THROTTLE_PULSE,
		MAX_THROTTLE_PULSE
	);

	actuators["steering"] = new ServoConverter(
		STEERING_ACTUATOR_CHANNEL,
		MIN_STEERING_PULSE,
		CENTER_STEERING_PULSE,
		MAX_STEERING_PULSE
	);

	// create the servo array publisher
	servo_array_pub_ = nh_.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 1);
	ROS_INFO("> Publisher correctly initialized.");

	// create the subscriber to joystick commands
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &DonkeyController::joy_callback, this);
	ROS_INFO("> Subscriber correctly initialized.");
	ROS_INFO("Initialization Complete!");
}

DonkeyController::~DonkeyController()
{
	// free memory dynamically allocated in the constructor
	delete actuators["throttle"];
	delete actuators["steering"];
}



void DonkeyController::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{

	float throttle_axis_val, steering_axis_val;
	int throttle_up, throttle_down;

	// get command values from joystick axes
	steering_axis_val = joy->axes[angular_];
	throttle_axis_val = joy->axes[linear_];
	throttle_up = joy->buttons[4];
	throttle_down = joy->buttons[6];

	if (throttle_up)
		increase_max_throttle();
	else
		if (throttle_down)
			decrease_max_throttle();
		else {
			steering_angle_ = steering_scale_ * steering_axis_val;
			throttle_ = throttle_scale_ * throttle_axis_val;

			// convert joystick values into servo values
			actuators["throttle"]->convert_and_update(throttle_);
			actuators["steering"]->convert_and_update(steering_angle_);
			send_servo_msg(); // send servo values to i2cpwm-board node
		}
}

ServoConverter::ServoConverter(int channel, int min, int center, int max) :
	value_out(310), range_(90), dir_(1)
{
	this->channel = channel;
	center_val_ = center;
	range_ = max - min;
	half_range_ = 0.5 * range_;
}

void ServoConverter::convert_and_update(float value_in)
{
	// convert command values between [-1, 1] to servo values
	// and store for later use
	value_out = static_cast<int>(dir_ * value_in * half_range_ + center_val_);
}

void DonkeyController::send_servo_msg(void)
{
	int channel, value, idx;
	std::string actuator_name;
	// iterate over actuators and get their servo values
	// and publish to /servo_absolute node

	std::map<std::string, ServoConverter*>::const_iterator it;
	for (it = actuators.begin(); it != actuators.end(); it++) {
		actuator_name = it->first;
		channel = it->second->channel;
		idx = channel - 1;
		value = it->second->value_out;
		msg.servos[idx].servo = channel;
		msg.servos[idx].value = value;
	}
	servo_array_pub_.publish(msg);
}

void DonkeyController::increase_max_throttle(void)
{
	// increase throttle scale setting
	throttle_scale_ = std::min(1.0, throttle_scale_ + 0.01);
	ROS_INFO("Throttle Scale is set to %.2f", throttle_scale_);
}

void DonkeyController::decrease_max_throttle(void)
{
	// decrease throttle scale setting
	throttle_scale_ = std::max(0.0, throttle_scale_ - 0.01);
	ROS_INFO("Throttle Scale is set to %.2f", throttle_scale_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "donkey_controller");
	DonkeyController controller;

	ros::spin();
	return 0;
}

