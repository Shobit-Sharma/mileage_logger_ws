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
        void getDateTime(string mode);
        void checkIfFileExists();
        void readCSV();
        void writeToFile(string edit_mode);
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
        vector<vector<string> > parsedCsv;
        int number_of_rows;
};

MileageLogger::MileageLogger(){};
MileageLogger::~MileageLogger(){};

void MileageLogger::start()
{
    // system always boots up in open loop
    current_mode = "open loop";
    previous_mode = "open loop";
    number_of_rows = 0;
    getHomeDir();
    getDateTime("date");
    checkIfFileExists();
}

void MileageLogger::getHomeDir()
{
    // to-do: getenv("HOME") not working with a system service, to be debugged.
    // path2store_logs = getenv("HOME");
    // if (path2store_logs == "")
    // {
    path2store_logs = getpwuid(getuid())->pw_dir;
    // }
}

void MileageLogger::getDateTime(string mode)
{
    time_t rawtime;
    struct tm *timeinfo;
    char buf[100];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    if (mode == "date")
    {
        if (strftime(buf, sizeof(buf), "%F", timeinfo));
        {
            date = buf;
        }
    }
    else if (mode == "start")
    {
        if (strftime(buf, sizeof(buf), "%T", timeinfo));
        {
            start_time = buf;
        }
    }
    else if (mode == "end")
    {
        if (strftime(buf, sizeof(buf), "%T", timeinfo));
        {
            end_time = buf;
        }        
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
        getDateTime("start");
        file=fopen(path2store_logs.c_str(), "w+");
        ROS_INFO("Created file: [%s]", path2store_logs.c_str());
        fclose(file);
        fstream log_file;
        log_file.open(path2store_logs.c_str());
        // write csv headers
        log_file << "Date,Start Time,End Time, Test Location,Functionality Tested,Software Version,Kilometers Driven,Test Driven,Safety Observer,Mode\n";
        log_file << date << "," << start_time << "," << end_time << "," << test_location << "," << functionality << "," << software_version << ",";
        log_file << distance_driven << "," << test_driver << "," << safety_observer << "," << current_mode << "\n";
        log_file.close();
    }
    // compare current values with existing ones to prevent overwrite
    else
    {
        string mode_old;
        readCSV();
        // distance_driven_old = parsedCsv[number_of_rows-1][6];
        mode_old = parsedCsv[number_of_rows-1][9];
        if (mode_old == current_mode)
        {
            getDateTime("start");
            writeToFile("append");
        }
        number_of_rows = 0;
        parsedCsv.clear();
    }
}

void MileageLogger::readCSV()
{
    fstream log_file(path2store_logs.c_str());
    string line;
    while(getline(log_file, line))
    {
        number_of_rows++;
        stringstream lineStream(line);
        string cell;
        vector<string> parsedRow;
        while(getline(lineStream,cell,','))
        {
            parsedRow.push_back(cell);
        }
        parsedCsv.push_back(parsedRow);
    }
    log_file.close();
}

void MileageLogger::writeToFile(string edit_mode)
{
    if (edit_mode == "append")
    {
        fstream log_file;
        log_file.open(path2store_logs.c_str(), ios::app);
        log_file << date << "," << start_time << "," << end_time << "," << test_location << "," << functionality << "," << software_version << ",";
        log_file << distance_driven << "," << test_driver << "," << safety_observer << "," << current_mode << "\n";
        log_file.close();
    }
    else if (edit_mode == "edit")
    {
        readCSV();
        parsedCsv[number_of_rows-1][2] = end_time;
        parsedCsv[number_of_rows-1][3] = test_location;
        parsedCsv[number_of_rows-1][4] = functionality;
        parsedCsv[number_of_rows-1][5] = software_version;
        string temp = boost::lexical_cast<string>(distance_driven);
        parsedCsv[number_of_rows-1][6] = temp;
        parsedCsv[number_of_rows-1][7] = test_driver;
        parsedCsv[number_of_rows-1][8] = safety_observer;
        parsedCsv[number_of_rows-1][9] = current_mode;
        fstream log_file;
        log_file.open(path2store_logs.c_str(), ios::out);
        for (int row=0; row<number_of_rows; row++)
        {
            for (int col = 0; col<10; col++)
            {
                log_file << parsedCsv[row][col] << ",";
            }
            log_file << "\n";
        }
        log_file.close();
        number_of_rows = 0;
        parsedCsv.clear();
    }
}
     

void MileageLogger::egoMotionCb(const visteon_fusion_msgs::EgoFusion::ConstPtr& msg)
{
    // calculate distance travelled in kilometers
    // to-do: calculate time delta between two messages to get more accurate distance calulations
    float distance = msg->longitudinalVelocity * 3.6 * (0.02 / 3600);
    distance_driven = distance_driven + distance;
    if (previous_mode != current_mode)
    {
        getDateTime("end");
        start_time = end_time;
        writeToFile("append");
        previous_mode = current_mode;
        distance_driven = 0.0;
    }
    else
    {
        getDateTime("end");
        writeToFile("edit");
    }    
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