#ifndef RVIZ_SETHP
#define RVIZ_SETHP

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

namespace ground_station
{
    class setHomePoint: public rviz::Tool {
    Q_OBJECT
    public:
        setHomePoint();
        ~setHomePoint();

        virtual void onInitialize();
        virtual void activate();
        virtual void deactivate();

        virtual int processMouseEvent( rviz::ViewportMouseEvent& event) {
            return 2;
        }

        void do_process();
        void process();

        // virtual void setIcon(const QIcon& icon);
        
        // virtual void load( const rviz::Config& config);
        // virtual void save( rviz::Config config) const;
    
    private:
        ros::NodeHandle nh;
        int num_uav;
        float h_latitude, h_longitude, h_altitude;
        std::vector<ros::ServiceClient> sethomepoint_srv_vec;
        // ros::service
        rviz::VectorProperty* current_flag_property_;
        // std::thread cmd_thread;
        std::mutex cmd_process_thread_mutex;
    };

}



#endif