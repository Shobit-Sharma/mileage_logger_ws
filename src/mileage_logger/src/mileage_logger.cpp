#include "ros/ros.h"

#include "unistd.h"
#include "sys/types.h"
#include "pwd.h"

#include "time.h"
#include "boost/filesystem.hpp"
#include "fstream"

#include "visteon_fusion_msgs/EgoFusion.h"
#include "visteon_vehctrl_msgs/FsmStatus.h"

using namespace std;

class MileageLogger
{
    public:
        void start();
        void getHomeDir();
        void getDateTime();
        void checkIfFileExists();
        void writeToFile();
        void egoMotionCb(const visteon_fusion_msgs::EgoFusion::ConstPtr& msg);
        void swcStatusCb(const visteon_vehctrl_msgs::FsmStatus::ConstPtr& msg);

        string test_location;
        string functionality;
        string software_version;
        string test_driver;
        string safety_observer;

        MileageLogger();
        ~MileageLogger();

    private:
        string path2store_logs;
        string date;
        string start_time;
        string end_time;
        float distance_driven;
        string current_mode;
        string previous_mode;
};

MileageLogger::MileageLogger(){};
MileageLogger::~MileageLogger(){};

void MileageLogger::start()
{
    getHomeDir();
    getDateTime();
    checkIfFileExists();
}

void MileageLogger::getHomeDir()
{
    path2store_logs = getenv("HOME");
    // if (path2store_logs == NULL)
    if (path2store_logs == "")
    {
        path2store_logs = getpwuid(getuid())->pw_dir;
    }
}

void MileageLogger::getDateTime()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buf[100];
    time (&rawtime);
    timeinfo = localtime(&rawtime);    
    if (strftime(buf, sizeof(buf), "%F", timeinfo));
    {
        date = buf;
    }
}

void MileageLogger::checkIfFileExists()
{
    path2store_logs = path2store_logs + "/mileage_logs/";
    // check if folder exists
    if (!boost::filesystem::exists(path2store_logs))
    {
        boost::filesystem::create_directories(path2store_logs);
        ROS_INFO("Created directory: [%s]", path2store_logs.c_str());
    }
    path2store_logs = path2store_logs + date + ".csv";
    // check if file exists
    FILE *file;
    if (fopen(path2store_logs.c_str(),"r") == NULL)
    {
        file=fopen(path2store_logs.c_str(), "w+");
        ROS_INFO("Created file: [%s]", path2store_logs.c_str());
        fclose(file);
        std::ofstream myfile;
        myfile.open(path2store_logs.c_str());
        // write csv headers
        myfile << "Date,Start Time,End Time, Test Location,Functionality Tested,Software Version,Kilometers Driven,Test Driven,Safety Observer,Mode\n";
        myfile.close();
    }
}

void MileageLogger::writeToFile()
{
    ROS_INFO("mode changed");
    // ROS_INFO("Path is: [%s]", path2store_logs);
}

void MileageLogger::egoMotionCb(const visteon_fusion_msgs::EgoFusion::ConstPtr& msg)
{
    // calculate distance travelled in kilometers
    float distance = msg->longitudinalVelocity * 3.6 * (0.02 / 3600);
    distance_driven = distance_driven + distance;
    if (previous_mode != current_mode)
    {
        writeToFile();
        previous_mode = current_mode;
    }
    else{
        ROS_INFO("same mode");
    }
    
    // ROS_INFO("Path is: [%s]", path2store_logs.c_str());
}

void MileageLogger::swcStatusCb(const visteon_vehctrl_msgs::FsmStatus::ConstPtr& msg)
{
    if(msg->fsm_DrivingModeStatus == 1)
    {
        current_mode = "open loop"; 
    }
    else if(msg->fsm_DrivingModeStatus == 2 | 
            msg->fsm_DrivingModeStatus == 3 | 
            msg->fsm_DrivingModeStatus == 4 | 
            msg->fsm_DrivingModeStatus == 5)
    {
        current_mode = "closed loop";
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mileage_logger_node");
    ros::NodeHandle n;
    MileageLogger ml;
    
    // get parameters
    n.param<string>("/sy/mileage_logger_node/test_location", ml.test_location, "Integration Tests");
    n.param<string>("/sy/mileage_logger_node/functionality", ml.functionality, "Integration Tests");
    n.param<string>("/sy/mileage_logger_node/software_version", ml.software_version, "55");
    n.param<string>("/sy/mileage_logger_node/test_driver", ml.test_driver, "James");
    n.param<string>("/sy/mileage_logger_node/safety_observer", ml.safety_observer, "Bond");
    
    // begin logging
    ml.start();
    ros::Subscriber ego_motion_sub = n.subscribe("/pc/fusion/egomotion",1,&MileageLogger::egoMotionCb, &ml);
    ros::Subscriber swc_status_sub = n.subscribe("/ex/fsm/swc_status",1,&MileageLogger::swcStatusCb, &ml);
    ros::spin();
    return 0;
}