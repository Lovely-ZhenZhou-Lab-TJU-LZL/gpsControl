#include "rviz_callDemo.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <ctrl_msg/SetDemo.h>

namespace ground_station {


callDemo::callDemo(): current_flag_property_(nullptr) {
    nh.param<int>("/gs_num_uav", num_uav, 0);
    float _delay_time;
    nh.param<float>("/demo_perpare_time", _delay_time, 10.0);
    delay_time = ros::WallDuration(_delay_time);
    ROS_WARN_STREAM("[ground station]: control " << num_uav << " uavs" );
    ROS_WARN_STREAM("-----------------------------------------------");
    ROS_WARN_STREAM("=================== Demo srv ==================");
    ROS_WARN_STREAM("prepare time: " << _delay_time);
    ROS_WARN_STREAM("-----------------------------------------------");
    calldemo_srv_vec.resize(num_uav);
    cmd_process_thread_mutex.resize(num_uav, true);
    for (int i = 0; i < num_uav; i++) {
        std::string srv_name = "/vio_test";
        srv_name += std::to_string(i+1);
        srv_name += "_pos_ctrl_node/demo_srv";
        calldemo_srv_vec[i] = nh.serviceClient<ctrl_msg::SetDemo>(srv_name);
    }
}

callDemo::~callDemo() {

}

// virtual void onInitialize();
void callDemo::onInitialize() {
    ROS_INFO("demo button ready");
}

// virtual void activate();
void callDemo::activate() {
    ROS_WARN("EVERY DRONE Ready to Demo");
    do_process();
    current_flag_property_ = new rviz::VectorProperty("commander1");
    current_flag_property_->setReadOnly(true);
    getPropertyContainer()->addChild( current_flag_property_);
}

// virtual void deactivate();
void callDemo::deactivate() {
    ROS_WARN("Button action done");
    delete current_flag_property_;
    current_flag_property_ = nullptr;
}


void callDemo::do_process() {
    ros::WallTime demo_start_time = ros::WallTime::now() + delay_time;
    ROS_WARN_STREAM("Demo will begin at " << demo_start_time << "!!!");
    for (int i = 0; i < num_uav; i++) {
        if ( cmd_process_thread_mutex[i]) {
            cmd_process_thread_mutex[i] = false;
            std::thread cmd_th(&callDemo::process,this,i+1, demo_start_time);
            if (cmd_th.joinable())
                cmd_th.detach();
        } else {
            ROS_WARN(" UAV %d demo cmd busy ", i+1);
        }
    }
}

void callDemo::process(int id, ros::WallTime t) {
    ctrl_msg::SetDemo demo_srv;
    demo_srv.request.stamp.sec = t.sec;
    demo_srv.request.stamp.nsec = t.nsec;
    if (calldemo_srv_vec[id-1].call(demo_srv)) {
        if (demo_srv.response.res) {
            ROS_WARN("%d uav demo cmd send sucess", id);
        } else {
            ROS_WARN("%d uav cannot demo", id);
        }
    } else {
        ROS_WARN("%d uav demo srv call fail", id);
    }

    cmd_process_thread_mutex[id-1] = true;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ground_station::callDemo, rviz::Tool)
