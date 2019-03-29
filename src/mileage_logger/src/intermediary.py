#!/usr/bin/env python

import os
import subprocess
import rospy
import time

def main():
    rospy.init_node('intermediary_node', disable_signals=True)
    sudo_password = rospy.get_param('/sy/mileage_logger_node/sudo_password', 'ubuntu')
    stop_command = rospy.get_param('/sy/mileage_logger_node/stop_command', 'service mileage stop')
    start_command = rospy.get_param('/sy/mileage_logger_node/start_command', 'service mileage start')
    try:
        os.system('echo %s|sudo -S %s' % (sudo_password,stop_command))
        # allow service to exit
        time.sleep(5)
        # subprocess.call(["roslaunch","visteon_mkz_black_bringup","dc.launch"])
        subprocess.call(["roslaunch","visteon_mkz_black_description","mkz.launch"])
        rospy.spin()
    except KeyboardInterrupt:
        # allow nodes to exit properly before restarting service
        time.sleep(10)
        os.system('echo %s|sudo -S %s' % (sudo_password,start_command))
        rospy.signal_shutdown('abc')

if __name__=='__main__':
    main()