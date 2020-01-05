#include <chrono>
#include <string>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>


using namespace std::chrono;

std::ofstream record_file;

class DataRecorder
{
public:
	DataRecorder();
private:
	ros::NodeHandle nh_;
	ros::Subscriber joy_sub_, camera_sub_;
	int linear_, angular_;
	float steering_angle_, throttle_;

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void camera_callback(const sensor_msgs::CompressedImageConstPtr& img);
};

void signal_handler(int sig)
{
	ROS_INFO("Saving CSV training data.");
	record_file.close();
	ROS_INFO("CSV traing data saved.");

	ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_recorder", ros::init_options::NoSigintHandler);
	DataRecorder recorder;

	std::signal(SIGINT, signal_handler);
	ros::spin();

	return 0;
}

DataRecorder::DataRecorder() :
	linear_(1), angular_(3)
{
	std::string record_filename = "training_data/training_data.csv";
	std::string raspicam = "raspicam_node/image/compressed";
	std::string joy = "joy";

	ROS_INFO("Setting Up the DataRecorder Node...");

	camera_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>(raspicam, 30, 
		&DataRecorder::camera_callback, this);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy, 30, &DataRecorder::joy_callback, this);

	record_file.open(record_filename, std::ios_base::app);

	ROS_INFO("Initialization Complete!");
}

void DataRecorder::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	steering_angle_ = joy->axes[angular_];
	throttle_ = joy->axes[linear_];
}

void DataRecorder::camera_callback(const sensor_msgs::CompressedImageConstPtr& img)
{
	cv::Mat cv_img = cv::imdecode(cv::Mat(img->data), CV_LOAD_IMAGE_COLOR);
	cv::Mat resized_cv_img;
	cv::resize(cv_img, resized_cv_img, cv::Size(), 0.25, 0.25);

	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	std::string img_name = "screenshot_" + std::to_string(ms.count()) + ".jpg";

	imwrite("training_data/" + img_name, resized_cv_img);
	record_file << img_name + "," 
		+ std::to_string(steering_angle_) + "," 
		+ std::to_string(throttle_) + "\n";
}
