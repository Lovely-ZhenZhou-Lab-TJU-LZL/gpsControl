#ifndef STATE_ESTIMATE_VIO_H_
#define STATE_ESTIMATE_VIO_H_

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "diff_intigral_cal.h"
//#include "sliding_differentiation.h"

class State_Estimate_Vio{
    public:

        typedef struct state_struction {
            ros::Time header;
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Vector3d Acc;
            Eigen::Quaterniond att_q;
        }State_s;
                
        void px4_estimated_data_cb(const nav_msgs::Odometry::Ptr _data) 
        {
            pthread_mutex_lock(&state_mutex);
            nav_msgs::Odometry temp_estimated_ = *_data;
            state_.Pos(0) = temp_estimated_.pose.pose.position.x;
            state_.Pos(1) = temp_estimated_.pose.pose.position.y;
            state_.Pos(2) = temp_estimated_.pose.pose.position.z;
            state_.att_q.w() = temp_estimated_.pose.pose.orientation.w;
            state_.att_q.x() = temp_estimated_.pose.pose.orientation.x;
            state_.att_q.y() = temp_estimated_.pose.pose.orientation.y;
            state_.att_q.z() = temp_estimated_.pose.pose.orientation.z;
            state_.Vel(0) = temp_estimated_.twist.twist.linear.x;
            state_.Vel(1) = temp_estimated_.twist.twist.linear.y;
            state_.Vel(2) = temp_estimated_.twist.twist.linear.z;
            state_.Acc(0) = temp_estimated_.twist.twist.angular.x;
            state_.Acc(1) = temp_estimated_.twist.twist.angular.y;
            state_.Acc(2) = temp_estimated_.twist.twist.angular.z;
            state_.header = temp_estimated_.header.stamp;
            // std::cout << temp_estimated_.header.stamp << std::endl;
            pthread_mutex_unlock(&state_mutex);
        }

        State_Estimate_Vio(int id) : 
        nh_("~state_estimate"),
        rigidbody_id_(id){

            std::string parent_topic_name = "/mavros";

            std::string estimated_topic_channel = parent_topic_name + std::to_string(id);
            estimated_topic_channel += "/local_position/odom";

            estimated_sub_ = nh_.subscribe(estimated_topic_channel,10,
                                           &State_Estimate_Vio::px4_estimated_data_cb,this);
            
            pthread_mutex_init(&state_mutex, NULL);
        }

        ~State_Estimate_Vio() {
            pthread_mutex_destroy(&state_mutex);
        }

        State_s get_state() {
            State_s temp_state_;
            pthread_mutex_lock(&state_mutex);
            temp_state_ = state_;
            pthread_mutex_unlock(&state_mutex);
            return temp_state_; 
        }

        int get_rigidbody_id() { return rigidbody_id_; }

    private:
        ros::NodeHandle nh_;
        int rigidbody_id_;
        State_s state_;
        ros::Subscriber estimated_sub_;
        pthread_mutex_t state_mutex;
};

#endif