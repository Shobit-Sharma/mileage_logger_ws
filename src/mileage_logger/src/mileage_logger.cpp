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
        // void writeToFile();
        void egoMotionCb(const visteon_fusion_msgs::EgoFusion::ConstPtr& msg);

        MileageLogger();
        ~MileageLogger();

    private:
        char *path2store_logs = NULL;
        char *date = NULL;
        FILE *file;
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
    if (path2store_logs == NULL)
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
    // cout << "date from here: " << date << endl;
    strcat(path2store_logs, "/mileage_logs/");
    // check if folder exists
    if (!boost::filesystem::exists(path2store_logs))
    {
        boost::filesystem::create_directories(path2store_logs);
        ROS_INFO("Created directory: [%s]", path2store_logs);
    }
    strcat(path2store_logs, date);
    strcat(path2store_logs, ".csv");
    // check if file exists
    if (fopen(path2store_logs,"r") == NULL)
    {
        file=fopen(path2store_logs, "w+");
        ROS_INFO("Created file: [%s]", path2store_logs);
        fclose(file);
        std::ofstream myfile;
        myfile.open(path2store_logs);
        // write csv headers
        myfile << "Date,Start Time,End Time, Test Location,Functionality Tested,Software Version,Kilometers Driven,Test Driven,Safety Observer,Mode\n";
        myfile.close();
    }
}

// void MileageLogger::writeToFile()
// {
//     ROS_INFO("Date is: [%s]", date);
//     ROS_INFO("Path is: [%s]", path2store_logs);
// }

void MileageLogger::egoMotionCb(const visteon_fusion_msgs::EgoFusion::ConstPtr& msg)
{
    // ROS_INFO("Date is: [%s]", date);
    ROS_INFO("Path is: [%s]", path2store_logs);
    // cout << "date is: " << date << endl;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mileage_logger_node");
    ros::NodeHandle n;
    MileageLogger ml;
    ml.start();
    ros::Subscriber ego_motion_sub = n.subscribe("/pc/fusion/egomotion",1,&MileageLogger::egoMotionCb, &ml);
    ros::spin();
    return 0;
}