#include <stdio.h>
#include "rviz_callTakeOff.h"

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <ctrl_msg/SetTakeoffLand.h>

namespace ground_station
{
    callTakeOff::callTakeOff(): current_flag_property_(nullptr) {
        nh.param<int>("/gs_num_uav", num_uav, 0);
        nh.param<float>("/takeoff_alt", takeoff_alt, 1.0);
        ROS_WARN_STREAM("[ground station]: control " << num_uav << " uavs" );
        ROS_WARN_STREAM("-----------------------------------------------");
        ROS_WARN_STREAM("================== takeoff srv ================");
        ROS_WARN_STREAM("takeoff alt: " << takeoff_alt);
        ROS_WARN_STREAM("-----------------------------------------------");
        calltakeoff_srv_vec.resize(num_uav);
        cmd_process_thread_mutex.resize(num_uav, true);
        for (int i = 0; i < num_uav; i++) {
            std::string srv_name = "/vio_test";
            srv_name += std::to_string(i+1);
            srv_name += "_pos_ctrl_node/controller/takeoff_land";
            calltakeoff_srv_vec[i] = nh.serviceClient<ctrl_msg::SetTakeoffLand>(srv_name);
        }
    }

    
    callTakeOff::~callTakeOff() {
        // for(int i = 0; i < num_uav; i++)
        //     cmd_process_thread_mutex[i].unlock();
    }

    void callTakeOff::onInitialize() {
        ROS_INFO("takeoff button ready");
    }

    void callTakeOff::activate() {
        ROS_WARN("EVERY DRONE TAKEOFF");
        do_process();
        current_flag_property_ = new rviz::VectorProperty("commander1");
        current_flag_property_->setReadOnly(true);
        getPropertyContainer()->addChild( current_flag_property_);
    }

    void callTakeOff::deactivate() {
        ROS_WARN("Button action done");
        delete current_flag_property_;
        current_flag_property_ = nullptr;
    }

    void callTakeOff::do_process() {
        for (int i = 0; i < num_uav; i++) {
            if ( cmd_process_thread_mutex[i]) {
                cmd_process_thread_mutex[i] = false;
                std::thread cmd_th(&callTakeOff::process,this,i+1);
                if (cmd_th.joinable())
                    cmd_th.detach();
            } else {
                ROS_WARN(" UAV %d takeoff cmd busy ", i+1);
            }
        }
    }

    void callTakeOff::process(int id) {
        ctrl_msg::SetTakeoffLand takeoffland_srv;
        takeoffland_srv.request.takeoff = true;
        takeoffland_srv.request.takeoff_altitude = takeoff_alt;
        if (calltakeoff_srv_vec[id-1].call(takeoffland_srv)) {
            if (takeoffland_srv.response.res) {
                ROS_WARN("%d uav takeoff sucess", id);
            } else {
                ROS_WARN("%d uav cannot takeoff", id);
            }
        } else {
            ROS_WARN("%d uav takeoff srv call fail", id);
        }

        cmd_process_thread_mutex[id-1] = true;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ground_station::callTakeOff, rviz::Tool)