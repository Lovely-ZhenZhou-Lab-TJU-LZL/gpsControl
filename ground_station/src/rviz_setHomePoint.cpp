#include <stdio.h>
#include "rviz_setHomePoint.h"

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <mavros_msgs/CommandHome.h>

namespace ground_station
{
    setHomePoint::setHomePoint(): current_flag_property_(nullptr) {
        nh.param<int>("/gs_num_uav", num_uav, 0);
        // nh.param<float>("home_yaw", h_yaw);
        nh.param<float>("/home_latitude", h_latitude, 0.0);
        nh.param<float>("/home_longitude", h_longitude, 0.1);
        nh.param<float>("/home_altitude", h_altitude, 0.2);
        ROS_WARN_STREAM("[ground station]: control " << num_uav << " uavs" );
        ROS_WARN_STREAM("-----------------------------------------------");
        ROS_WARN_STREAM("=================== home set ==================");
        ROS_WARN_STREAM("lat: " << h_latitude);
        ROS_WARN_STREAM("lon: " << h_longitude);
        ROS_WARN_STREAM("alt: " << h_altitude);
        ROS_WARN_STREAM("-----------------------------------------------");
        sethomepoint_srv_vec.resize(num_uav);
        for (int i = 0; i < num_uav; i++) {
            std::string srv_name = "/mavros";
            srv_name += std::to_string(i+1);
            srv_name += "/cmd/set_home";
            sethomepoint_srv_vec[i] = nh.serviceClient<mavros_msgs::CommandHome>(srv_name);
        }
    }

    
    setHomePoint::~setHomePoint() {
        cmd_process_thread_mutex.unlock();
    }

    void setHomePoint::onInitialize() {
        ROS_INFO("set HomePoint button ready");
    }

    void setHomePoint::activate() {
        ROS_WARN("SET HOME POINT TO EVERY DRONE");
        do_process();
        current_flag_property_ = new rviz::VectorProperty("commander1");
        current_flag_property_->setReadOnly(true);
        getPropertyContainer()->addChild( current_flag_property_);
    }

    void setHomePoint::deactivate() {
        ROS_WARN("Button action done");
        delete current_flag_property_;
        current_flag_property_ = nullptr;
    }

    void setHomePoint::do_process() {
        if ( cmd_process_thread_mutex.try_lock()) {
            std::thread cmd_th(&setHomePoint::process, this);
            if (cmd_th.joinable())
                cmd_th.detach();
        } else {
            ROS_WARN(" set home cmd busy ");
        }
    }

    void setHomePoint::process() {
        mavros_msgs::CommandHome set_hp_srv;
        set_hp_srv.request.current_gps = false;
        // set_hp_srv.request.yaw = h_yaw;
        set_hp_srv.request.latitude = h_latitude;
        set_hp_srv.request.longitude = h_longitude;
        set_hp_srv.request.altitude = h_altitude;
        for (int i = 0; i < sethomepoint_srv_vec.size();i++) {
            if (sethomepoint_srv_vec[i].call(set_hp_srv)) {
                if (set_hp_srv.response.success) {
                    ROS_WARN("%d uav home point set sucess", i+1);
                } else {
                    ROS_WARN("%d uav cannot set home point", i+1);
                }
            } else {
                ROS_WARN("%d uav srv call fail", i+1);
            }
        }
        cmd_process_thread_mutex.unlock();
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ground_station::setHomePoint, rviz::Tool)