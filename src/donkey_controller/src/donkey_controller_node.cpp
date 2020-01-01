#include <ros/ros.h>
#include <ros/console.h>
#include <map>
#include <string>
#include <iostream>

// messages used for drive movement topic
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// messages used for the absolute and proportional movement topics
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include "std_msgs/String.h"

class ServoConverter
{
public:
	ServoConverter(int channel, int throttle_center_value, int steer_center_value);
	void convert_and_update(float value_in, bool is_throttle = true);
	int channel, value_out;

private:
	float value, half_range_;
	int dir_, range_;
	int throttle_center_, steer_center_;
};

class DonkeyController
{
public:
	DonkeyController();
private:
	ros::NodeHandle nh_;
	ros::Publisher servo_array_pub_;
	ros::Subscriber joy_sub_;
	int linear_, angular_;
	i2cpwm_board::ServoArray msg;
	std::map<std::string, ServoConverter*> actuators;

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void ai_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void send_servo_msg(void);
	void send_callback(const sensor_msgs::Joy::ConstPtr& joy);
};


DonkeyController::DonkeyController() :
	linear_(1), angular_(3) {

	ROS_INFO("Setting Up the DonkeyController Node...");

	for (int i = 0; i < 2; i++) {
		msg.servos.push_back(i2cpwm_board::Servo());
	}

	actuators["throttle"] = new ServoConverter(1, 310, 310);
	actuators["steering"] = new ServoConverter(2, 310, 310);

	// create the servo array publisher
	servo_array_pub_ = nh_.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 1);
	ROS_INFO("> Publisher correctly initialized.");
	
	// create the subscriber to joystick commands
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &DonkeyController::joy_callback, this);
	ROS_INFO("> Subscriber correctly initialized.");



	ROS_INFO("Initialization Complete!");
}

void DonkeyController::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	float steering_angle, throttle;

    steering_angle = joy->axes[angular_];
	throttle = joy->axes[linear_];

	// convert joystick values into servo values
	actuators["throttle"]->convert_and_update(throttle);
	actuators["steering"]->convert_and_update(steering_angle, false);
	ROS_INFO("Got a command v = %2.1f s = %2.1f", throttle, steering_angle);

	send_servo_msg();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "donkey_controller");
	DonkeyController controller;
	
	ros::spin();

	return 0;
}

ServoConverter::ServoConverter(int channel, int throttle_center_value, int steer_center_value) :
	value(0.0), value_out(333), range_(90), dir_(1)
{
	throttle_center_ = throttle_center_value;
	steer_center_ = steer_center_value;
	half_range_ = 0.5 * range_;
	this->channel = channel;
}

void ServoConverter::convert_and_update(float value_in, bool is_throttle)
{
	value = value_in;
	int center_val = is_throttle ? throttle_center_ : steer_center_;
	value_out = static_cast<int>(dir_ * value_in * half_range_ + center_val);
	
	std::cout << channel << "\t" << value_out;
}

void DonkeyController::send_servo_msg(void)
{
	int channel, value, idx;
	std::string actuator_name;

	std::map<std::string, ServoConverter*>::const_iterator it;
	for (it = actuators.begin(); it != actuators.end(); it++) {
		actuator_name = it->first;
		channel = it->second->channel;
		idx = channel - 1;
		value = it->second->value_out;
		msg.servos[idx].servo = channel;
		msg.servos[idx].value = value;
		ROS_INFO("Sending to %s command %d", actuator_name.c_str(), value);
	}
	servo_array_pub_.publish(msg);
}
