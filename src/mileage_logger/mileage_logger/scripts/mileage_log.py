#!/usr/bin/env python

import os
import rospy
import datetime
import pandas
import time

from visteon_fusion_msgs.msg import EgoFusion
from visteon_vehctrl_msgs.msg import FsmStatus

class LogMiles():
    def __init__(self):
        # global variables
        self.csv_header = ['Date', 'Start Time', 'End Time', 'Test Location', 'Functionality Tested', 
                            'Software Version', 'Kilometers Driven', 'Test Driver', 'Safety Observer', 'Mode']
        self.log_file = None
        # to do: find alternative command that can fetch relative path on startup
        # self.path2store_logs = str(os.environ['HOME'] + '/mileage_logs/')
        self.path2store_logs = '/home/shobit/mileage_logs/'
        self.date = str(datetime.datetime.now().strftime("%Y-%m-%d"))        
        self.start_time = str(datetime.datetime.now().strftime("%H-%M-%S"))
        self.end_time = str(datetime.datetime.now().strftime("%H-%M-%S"))
        self.test_location = "ACM"
        self.functionality = "Integration Tests"
        self.software_version = "35"
        self.distance_driven = 0.0
        self.test_driver = "James"
        self.safety_observer = "Bond"
        self.current_mode = "open_loop"
        self.previous_mode = "open_loop"
        
        # check if path exists
        if not os.path.exists(self.path2store_logs):
            os.makedirs(self.path2store_logs)
        # check if file exists
        if not os.path.isfile(self.path2store_logs + self.date + '.csv'):
            file = open(self.path2store_logs + self.date + '.csv', "w+")
            file.write(self.csv_header[0] + ',' + self.csv_header[1] + ',' + self.csv_header[2] + ',' + self.csv_header[3] + ',' + self.csv_header[4] + ',' +
                        self.csv_header[5] + ',' + self.csv_header[6] + ',' + self.csv_header[7] + ',' + self.csv_header[8] + ',' + self.csv_header[9] + '\r')
            file.close()
        # load log file 1st time
        self.log_file = pandas.read_csv(self.path2store_logs + self.date + '.csv', names = self.csv_header, index_col = False)

        # test information
        if len(self.log_file) <= 1:
            # self.get_test_info()
            self.write_test_info('append')
            # reload log file after append to ensure headers are not lost on edit
            self.reload_test_info(reset_flag=False)
        else:
            new_info_flag = raw_input("Do you want to update test information? y or n: ") or "n"
            if new_info_flag == 'y':
                self.reload_test_info(reset_flag=True)
                self.get_test_info()
                # if mode doesn't change, do not add new line item on relaunch
                self.current_mode = self.previous_mode
                self.write_test_info('append')
                # reload log file to ensure previous data is not lost on edit
                self.reload_test_info(reset_flag=False)
            elif new_info_flag == 'n':
                # reload log file to ensure previous data is not lost on edit
                self.reload_test_info(reset_flag=False)
                # if mode doesn't change, do not add new line item on relaunch
                self.current_mode = self.previous_mode
                
        # subscribers
        self.fsm_sub = rospy.Subscriber('/ex/fsm/swc_status', FsmStatus, self.fsm_cb)
        self.ego_motion_sub = rospy.Subscriber('/pc/fusion/egomotion', EgoFusion, self.ego_motion_cb)
        
    # get user inputs
    def get_test_info(self):
        print('Please enter the requested information.')
        self.test_location = raw_input("Test Location: ") or "ACM"
        self.functionality = raw_input("Feature being tested: ") or "Integration Tests"
        self.software_version = raw_input("Software Version: ") or "35"
        self.test_driver = raw_input("Test driver's name: ") or "James"
        self.safety_observer = raw_input("Safety observer's name: ") or "Bond"
    
    # load log file
    def reload_test_info(self, reset_flag):
        self.log_file = pandas.read_csv(self.path2store_logs + self.date + '.csv', names = self.csv_header, index_col = False)
        self.date = self.log_file.loc[len(self.log_file) -1, 'Date']
        # self.start_time = self.log_file.loc[len(self.log_file) -1, 'Start Time']
        self.end_time = self.log_file.loc[len(self.log_file) -1, 'End Time']
        self.test_location = self.log_file.loc[len(self.log_file) -1, 'Test Location']
        self.functionality = self.log_file.loc[len(self.log_file) -1, 'Functionality Tested']
        self.software_version = self.log_file.loc[len(self.log_file) -1, 'Software Version']
        # self.distance_driven = float(self.log_file.loc[len(self.log_file) -1, 'Kilometers Driven'])
        self.test_driver = self.log_file.loc[len(self.log_file) -1, 'Test Driver']
        self.safety_observer = self.log_file.loc[len(self.log_file) -1, 'Safety Observer']
        self.previous_mode = self.log_file.loc[len(self.log_file) -1, 'Mode']
        if reset_flag == True:
            self.start_time = str(datetime.datetime.now().strftime("%H-%M-%S"))
            self.distance_driven = 0.0
        elif reset_flag == False:
            self.start_time = self.log_file.loc[len(self.log_file) -1, 'Start Time']
            self.distance_driven = float(self.log_file.loc[len(self.log_file) -1, 'Kilometers Driven'])

    # write to log file
    def write_test_info(self, edit_mode):
        if edit_mode == 'append':
            frame = {'Date': [self.date],
                     'Start Time': [self.start_time],
                     'End Time': [self.end_time],
                     'Test Location': [self.test_location],
                     'Functionality Tested': [self.functionality],
                     'Software Version': [self.software_version],
                     'Kilometers Driven': [self.distance_driven],
                     'Test Driver': [self.test_driver],
                     'Safety Observer': [self.safety_observer],
                     'Mode': [self.current_mode]}
            df = pandas.DataFrame(frame, columns = ['Date', 'Start Time', 'End Time', 'Test Location', 'Functionality Tested', 
                                                    'Software Version', 'Kilometers Driven', 'Test Driver', 'Safety Observer', 'Mode'])
            df.to_csv(self.path2store_logs + self.date + '.csv', mode='a', header=False, index=False)
        elif edit_mode == 'edit':
            self.log_file.loc[len(self.log_file) - 1, 'Date'] = self.date
            self.log_file.loc[len(self.log_file) - 1, 'Start Time'] = self.start_time
            self.log_file.loc[len(self.log_file) - 1, 'End Time'] = self.end_time
            self.log_file.loc[len(self.log_file) - 1, 'Test Location'] = self.test_location
            self.log_file.loc[len(self.log_file) - 1, 'Functionality Tested'] = self.functionality
            self.log_file.loc[len(self.log_file) - 1, 'Software Version'] = self.software_version
            self.log_file.loc[len(self.log_file) - 1, 'Kilometers Driven'] = self.distance_driven
            self.log_file.loc[len(self.log_file) - 1, 'Test Driver'] = self.test_driver
            self.log_file.loc[len(self.log_file) - 1, 'Safety Observer'] = self.safety_observer
            self.log_file.loc[len(self.log_file) - 1, 'Mode'] = self.current_mode
            self.log_file.to_csv(self.path2store_logs + self.date + '.csv', mode='w', header=False, index=False)

    # fsm callback
    def fsm_cb(self, data):
        if data.fsm_DrivingModeStatus == 1:
            self.current_mode = 'open_loop'
        elif(data.fsm_DrivingModeStatus == 2 or data.fsm_DrivingModeStatus == 3 or
             data.fsm_DrivingModeStatus == 4 or data.fsm_DrivingModeStatus == 5):
             self.current_mode = 'closed_loop'

    # ego motion callback
    def ego_motion_cb(self, data):
        # new line item for driving mode change
        if self.current_mode != self.previous_mode:
            self.write_test_info('append')
            self.reload_test_info(reset_flag=True)
        else:
            # calculate distance travelled in kilometers
            distance = data.longitudinalVelocity * 3.6 * (0.02 / 3600)
            self.distance_driven = self.distance_driven + distance
            self.end_time = str(datetime.datetime.now().strftime("%H-%M-%S"))
            self.write_test_info('edit')

def main():
    rospy.init_node('mileage_logger_node')
    lm = LogMiles()
    rospy.spin()
    
if __name__ == '__main__':
    main()