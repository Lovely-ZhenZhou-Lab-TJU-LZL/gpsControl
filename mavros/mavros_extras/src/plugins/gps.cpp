/**
 * @brief Distance Sensor plugin
 * @file distance_sensor.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <unordered_map>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>


#include <nav_msgs/Odometry.h>

namespace mavros {
namespace extra_plugins {
using utils::enum_value;
class GPSPlugin;


/**
 * @brief Distance sensor plugin
 *
 * This plugin allows publishing distance sensor data, which is connected to
 * an offboard/companion computer through USB/Serial, to the FCU or vice-versa.
 */
class GPSPlugin : public plugin::PluginBase {
public:
	GPSPlugin() : PluginBase(),
		gps_nh("~gps")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		gps_pub = gps_nh.advertise<nav_msgs::Odometry>("real_gps", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&GPSPlugin::handle_GPS),
		};
	}

private:
	

	ros::NodeHandle gps_nh;
	ros::Publisher gps_pub;

	/* -*- mid-level helpers -*- */

	/**
	 * Receive GPS data from FCU.
	 */
	void handle_GPS(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &gps)
	{
		
		auto gps_msg = boost::make_shared<nav_msgs::Odometry>();

		gps_msg->header.frame_id = "world";
		gps_msg->header.stamp = ros::Time::now();
		gps_msg->pose.pose.position.x = gps.lat;
		gps_msg->pose.pose.position.y = gps.lon;
		gps_msg->pose.pose.position.z = gps.alt;
		gps_msg->twist.twist.linear.x = gps.vel;
		gps_msg->twist.twist.linear.y = gps.satellites_visible;
		gps_pub.publish(gps_msg);
	}
};

}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GPSPlugin, mavros::plugin::PluginBase)
