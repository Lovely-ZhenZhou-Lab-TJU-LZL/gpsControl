#ifndef RVIZ_CALLDEMO
#define RVIZ_CALLDEMO

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/tool.h>
#include <thread>
#include <mutex>

namespace rviz
{
class VectorProperty;
// class VisualizationManager;
// class ViewportMouseEvent;
}

namespace ground_station {


class callDemo: public rviz::Tool {
    Q_OBJECT
    public:
        callDemo();
        ~callDemo();

        virtual void onInitialize();
        
        virtual void activate();
        
        virtual void deactivate();

        virtual int processMouseEvent( rviz::ViewportMouseEvent& event) {
            return 2;
        }

        void do_process();
        void process(int id, ros::WallTime t);

        // virtual void setIcon(const QIcon& icon);
        
        // virtual void load( const rviz::Config& config);
        // virtual void save( rviz::Config config) const;
    
    private:
        ros::NodeHandle nh;
        int num_uav;
        ros::WallDuration delay_time;
        std::vector<ros::ServiceClient> calldemo_srv_vec;
        // ros::service
        rviz::VectorProperty* current_flag_property_;
        // std::thread cmd_thread;
        std::vector<bool> cmd_process_thread_mutex;
};
}


#endif