#include "bag_handle.h"
// #include "ctrl_msg/SetTakeoffLand.h"
#include "ctrl_msg/ctrl_ref.h"
#include "ctrl_msg/SetDemo.h"
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "geometry_math_type.h"

Eigen::Vector3d positionOffset;
double yawOffset = 0.0f;
bool have_odom = false;
bool have_demo_time = false;
ros::WallTime demo_trigger_time;
// ros::ServiceClient takeoff_land_srv;

void odom_callback(const nav_msgs::Odometry::Ptr nav)
{
  if(have_odom == true)  return;
  else
  {
    positionOffset.x() = nav->pose.pose.position.x;
    positionOffset.y() = nav->pose.pose.position.y;
    positionOffset.z() = nav->pose.pose.position.z;    
    Eigen::Quaterniond quatOffset;
    quatOffset.w() = nav->pose.pose.orientation.w;
    quatOffset.x() = nav->pose.pose.orientation.x;
    quatOffset.y() = nav->pose.pose.orientation.y;
    quatOffset.z() = nav->pose.pose.orientation.z;
    Eigen::Vector3d euler;
    get_euler_from_q(euler, quatOffset);
    yawOffset = euler(2);
    have_odom = true;
  }
}

bool demo_trigger_handle(ctrl_msg::SetDemo::Request& req,
                        ctrl_msg::SetDemo::Response& res) {
    demo_trigger_time.sec = req.stamp.sec;
    demo_trigger_time.nsec = req.stamp.nsec;
    if (have_odom) {
        res.res = true;
        have_demo_time = true;
        return true;
    }
    res.res = false;
    return false;
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"testTaskMode");
    ros::NodeHandle nh("~");
    std::string file_name, topic_name;

    nh.param<std::string>("topic", topic_name, "/vio_test_pos_ctrl_node/controller/ctrl_ref");
    nh.param<std::string>("file", file_name, "/home/zxw/work1/LG/LG12/work/RVO_ws1/src/rvo_ros_node/control_command.bag");

    std::string uav_name;
    nh.param<std::string>("uav_name", uav_name, "juliett");
    std::string uav_ref_topic_name = "/" + uav_name + "_pos_ctrl_node/controller/ctrl_ref";
    ros::Publisher ctrl_command = nh.advertise<ctrl_msg::ctrl_ref>(uav_ref_topic_name, 10);

    ros::Subscriber odom_sub = nh.subscribe("/mavros1/local_position/odom", 10, odom_callback);

    ros::ServiceServer set_demo_srv = nh.advertiseService("/"+uav_name+"_pos_ctrl_node/demo_srv", &demo_trigger_handle);

    /*read bag file*/

	std::vector<ctrl_msg::ctrl_ref> ctrl_msgs_vec = read_bag<ctrl_msg::ctrl_ref>(file_name, topic_name, -1);

    std::cout<< "the size of ctrl_msgs_vec is : " << ctrl_msgs_vec.size() <<std::endl;


    /* wait odom*/
    ros::Rate loop_sleep(10);
    while(!have_odom && nh.ok())
    {
        loop_sleep.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO_STREAM("got odom, wait for trigger");

    /* wait trigger time */  
    while(!have_demo_time && nh.ok()) {
        loop_sleep.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO_STREAM("demo will begin at" << demo_trigger_time);
    /*prepare for demo*/
    while(ros::WallTime::now()<demo_trigger_time && nh.ok()) {
        loop_sleep.sleep();
        ros::spinOnce();
    }
  
    ROS_INFO_STREAM("demo start");
    /*start to demo*/
    ctrl_msg::ctrl_ref last_uav_command;
    for(int i=0; (i<ctrl_msgs_vec.size()&&nh.ok()); i++)
    {
        ctrl_msg::ctrl_ref uav_command;
        double delta_t;
        if(i < ctrl_msgs_vec.size()-1)
        {
            ros::Time t1 = ctrl_msgs_vec[i].header.stamp;
            ros::Time t2 = ctrl_msgs_vec[i+1].header.stamp;
            delta_t = (t2-t1).toSec();
        }
        else delta_t = 0;
        
        uav_command.pos_ref[0] =  ctrl_msgs_vec[i].pos_ref[0] + positionOffset.x();
        uav_command.pos_ref[1] =  ctrl_msgs_vec[i].pos_ref[1] + positionOffset.y();
        uav_command.pos_ref[2] =  ctrl_msgs_vec[i].pos_ref[2] + positionOffset.z();
        uav_command.yaw_ref = yawOffset;
        uav_command.ref_mask = 7;
        uav_command.vel_ref[0] =  ctrl_msgs_vec[i].vel_ref[0];
        uav_command.vel_ref[1] =  ctrl_msgs_vec[i].vel_ref[1];
        uav_command.vel_ref[2] =  ctrl_msgs_vec[i].vel_ref[2];
        uav_command.acc_ref[0] =  0.0f;
        uav_command.acc_ref[1] =  0.0f;
        uav_command.acc_ref[2] =  0.0f;
        ctrl_command.publish(uav_command);
        last_uav_command = uav_command;
        ros::Duration(delta_t).sleep(); 
    }    

    last_uav_command.ref_mask = ctrl_msg::ctrl_ref::POS_CTRL_VALIED;
    for(int i=0; (i<10&&nh.ok()); i++) {
        ctrl_command.publish(last_uav_command);
        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO_STREAM("demo done, hovering");
    ros::spin();
    return 0;
}

