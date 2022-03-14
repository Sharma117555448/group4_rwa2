#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from nist_gear.srv import AGVToAssemblyStation
from ariac_utilities import Ariac, Orders
from std_msgs.msg import String
from nist_gear.msg import Proximity, LogicalCameraImage

def logical_camera_agv1_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        rospy.loginfo_throttle(5, "Callback triggered for Topic /ariac/logical_camera_agv1")
        rospy.loginfo_throttle(5, data)



def quality_control_sensor_1_callback(data):
    """
    This callback method prints a statement on the terminal when it is triggered.

    Returns
    -------
    None
    """
    
    if(len(data.models) > 0):
        rospy.loginfo_once("Faulty Part Detected:")
        rospy.loginfo_once(data.models)

def quality_control_sensor_2_callback(data):
    """
    This callback method prints a statement on the terminal when it is triggered.

    Returns
    -------
    None
    """
    
    if(len(data.models) > 0):
        rospy.loginfo_once("Faulty Part Detected:")
        rospy.loginfo_once(data.models)

def quality_control_sensor_3_callback(data):
    """
    This callback method prints a statement on the terminal when it is triggered.

    Returns
    -------
    None
    """
    
    if(len(data.models) > 0):
        rospy.loginfo_once("Faulty Part Detected:")
        rospy.loginfo_once(data.models)

def quality_control_sensor_4_callback(data):
    """
    This callback method prints a statement on the terminal when it is triggered.

    Returns
    -------
    None
    """
    
    if(len(data.models) > 0):
        rospy.loginfo_once("Faulty Part Detected:")
        rospy.loginfo_once(data.models)



if __name__ == '__main__':
    try:
        rospy.init_node('rwa2', anonymous=True)
        ariac = Ariac()
        if(ariac.status()):
            rospy.Subscriber("/ariac/quality_control_sensor_1", LogicalCameraImage, quality_control_sensor_1_callback)
            rospy.Subscriber("/ariac/quality_control_sensor_2", LogicalCameraImage, quality_control_sensor_2_callback)
            rospy.Subscriber("/ariac/quality_control_sensor_3", LogicalCameraImage, quality_control_sensor_3_callback)
            rospy.Subscriber("/ariac/quality_control_sensor_4", LogicalCameraImage, quality_control_sensor_4_callback)
            #rospy.Subscriber("/ariac/logical_camera_agv1", LogicalCameraImage, logical_camera_agv1_callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        pass