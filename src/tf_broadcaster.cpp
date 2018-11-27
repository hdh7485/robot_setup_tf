#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

float roll, pitch, yaw, w;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
	ROS_INFO("IMU orientation: %f %f %f %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	roll = msg->orientation.x;
	pitch = msg->orientation.y;
	yaw = msg->orientation.z;
	w = msg->orientation.w;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("imu/data", 1000, imuCallback);
	ros::Rate r(10);

	tf::TransformBroadcaster broadcaster;

	while(n.ok()){
		broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(tf::Quaternion(roll, pitch, yaw, w), tf::Vector3(0.0, 0.0, 0.0)),
					ros::Time::now(), "map", "base_link"));

		/*
		broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.22, 0.0, 0.1)),
					ros::Time::now(), "base_link", "imu_link"));

		broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Quaternion origin(0,0,0,1);
					tf::Quaternion rotation(
					tf::Transform(tf::Quaternion(0, 0, 3.14159, 1), tf::Vector3(0.2, 0.0, 0.2)),
					ros::Time::now(), "base_link", "laser"));
					*/

		ros::spinOnce();
		r.sleep();
	}
}
