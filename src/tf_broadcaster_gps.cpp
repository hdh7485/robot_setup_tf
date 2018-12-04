#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <marvelmind_nav/hedge_pos_ang.h>
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
#include <visualization_msgs/Marker.h>

#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "/hedge_pos_ang"

#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "/beacons_pos_a"

#define HEDGE_IMU_RAW_TOPIC_NAME "/hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "/hedge_imu_fusion"

#define BEACON_RAW_DISTANCE_TOPIC_NAME "/beacon_raw_distance"

#define SCALE_HEDGE 3.0

float x, y, z;
float roll, pitch, yaw, w;

void hedgePosAngCallback(const marvelmind_nav::hedge_pos_ang& hedge_pos_msg)
{
	ROS_INFO("Hedgehog data: Address= %d, timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d", 	
			(int) hedge_pos_msg.address, 
			(int) hedge_pos_msg.timestamp_ms, 
			(float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,
			(float) hedge_pos_msg.angle,  
			(int) hedge_pos_msg.flags);

	x = hedge_pos_msg.x_m;
	y = hedge_pos_msg.y_m;
	z = hedge_pos_msg.z_m;
}

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

	ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imuCallback);
	ros::Subscriber gps_sub = n.subscribe("/hedge_pos_ang", 1000, hedgePosAngCallback);
	ros::Rate r(10);

	tf::TransformBroadcaster broadcaster;

	while(n.ok()){
		broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(tf::Quaternion(roll, pitch, yaw, w), tf::Vector3(x, y, 0.0)),
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
