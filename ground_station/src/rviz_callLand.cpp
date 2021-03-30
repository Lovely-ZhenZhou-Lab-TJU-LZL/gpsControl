#include <stdio.h>
#include "rviz_callLand.h"

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <ctrl_msg/SetTakeoffLand.h>

namespace ground_station
{
    callLand::callLand(): current_flag_property_(nullptr) {
        nh.param<int>("/gs_num_uav", num_uav, 0);
        ROS_WARN_STREAM("[ground station]: control " << num_uav << " uavs" );
        ROS_WARN_STREAM("-----------------------------------------------");
        ROS_WARN_STREAM("=================== land srv ==================");
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

    
    callLand::~callLand() {
        // for(int i = 0; i < num_uav; i++)
        //     cmd_process_thread_mutex[i].unlock();
    }

    void callLand::onInitialize() {
        ROS_INFO("land button ready");
    }

    void callLand::activate() {
        ROS_WARN("EVERY DRONE LAND");
        do_process();
        current_flag_property_ = new rviz::VectorProperty("commander1");
        current_flag_property_->setReadOnly(true);
        getPropertyContainer()->addChild( current_flag_property_);
    }

    void callLand::deactivate() {
        ROS_WARN("Button action done");
        delete current_flag_property_;
        current_flag_property_ = nullptr;
    }

    void callLand::do_process() {
        for (int i = 0; i < num_uav; i++) {
            if ( cmd_process_thread_mutex[i]) {
                cmd_process_thread_mutex[i] = false;
                std::thread cmd_th(&callLand::process,this,i+1);
                if (cmd_th.joinable())
                    cmd_th.detach();
            } else {
                ROS_WARN(" UAV %d land cmd busy ", i+1);
            }
        }
    }

    void callLand::process(int id) {
        ctrl_msg::SetTakeoffLand takeoffland_srv;
        takeoffland_srv.request.takeoff = false;
        takeoffland_srv.request.takeoff_altitude = 0.0f;
        if (calltakeoff_srv_vec[id-1].call(takeoffland_srv)) {
            if (takeoffland_srv.response.res) {
                ROS_WARN("%d uav land sucess", id);
            } else {
                ROS_WARN("%d uav cannot land", id);
            }
        } else {
            ROS_WARN("%d uav land srv call fail", id);
        }

        cmd_process_thread_mutex[id-1] = true;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ground_station::callLand, rviz::Tool)