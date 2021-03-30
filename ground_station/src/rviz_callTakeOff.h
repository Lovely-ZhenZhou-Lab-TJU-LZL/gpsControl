#ifndef RVIZ_CALLTKO
#define RVIZ_CALLTKO

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
    class callTakeOff: public rviz::Tool {
    Q_OBJECT
    public:
        callTakeOff();
        ~callTakeOff();

        virtual void onInitialize();
        virtual void activate();
        virtual void deactivate();

        virtual int processMouseEvent( rviz::ViewportMouseEvent& event) {
            return 2;
        }

        void do_process();
        void process(int id);

        // virtual void setIcon(const QIcon& icon);
        
        // virtual void load( const rviz::Config& config);
        // virtual void save( rviz::Config config) const;
    
    private:
        ros::NodeHandle nh;
        int num_uav;
        float takeoff_alt;
        std::vector<ros::ServiceClient> calltakeoff_srv_vec;
        // ros::service
        rviz::VectorProperty* current_flag_property_;
        // std::thread cmd_thread;
        // std::vector<std::mutex> cmd_process_thread_mutex;
        std::vector<bool> cmd_process_thread_mutex;
    };

}



#endif