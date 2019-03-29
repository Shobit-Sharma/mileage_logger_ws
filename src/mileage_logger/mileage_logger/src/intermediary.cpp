#include "ros/ros.h"
#include "signal.h"

using namespace std;

void mySigintHandler(int sig)
{
    system("echo ubuntu | sudo -S service mileage restart");
    ros::shutdown();
}
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "intermediary_node");
    ros::NodeHandle n;
    // use custom sigint handler
    signal(SIGINT, mySigintHandler);

    system("rosnode kill /pc/fusion/ego_motion");
    system("rosnode kill /sy/novatel");
    system("rosnode kill /sy/mileage_logger_node");
    // to-do: launch full system in subprocess so that hitting ctrl-c once kills it and triggers mySigintHandler
    system("roslaunch visteon_mkz_black_bringup dc.launch");
    ros::spin();
    return 0;
}