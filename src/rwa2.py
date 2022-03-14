#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from ariac_utilities import Ariac
from nist_gear.msg import LogicalCameraImage, Order
import threading

parts = {}


def broadcast(pose,frame):
    buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))
    ts1 = TransformStamped()
    ts1.header.stamp = rospy.Time(0)
    ts1.header.frame_id = frame
    ts1.child_frame_id = 'world'
    ts1.transform.translation.x = pose.position.x
    ts1.transform.translation.y = pose.position.y
    ts1.transform.translation.z = pose.position.z
    ts1.transform.rotation.x = pose.orientation.x
    ts1.transform.rotation.y = pose.orientation.y
    ts1.transform.rotation.z = pose.orientation.z
    ts1.transform.rotation.w = pose.orientation.w
    buffer_core.set_transform(ts1, "default_authority")

    return buffer_core.lookup_transform_core(frame, 'world', rospy.Time(0)).transform

def order_callback(data):
    kitting_order_shipment = data.kitting_shipments
    kitting_order_parts = []
    assembly_order_shipment = data.assembly_shipments
    assembly_order_parts = []
    
    if(len(kitting_order_shipment) > 0):
        kitting_order_products = kitting_order_shipment[0].products
        for part_type in kitting_order_products:
            #kitting_order_parts.append(part.type)
            endtime = rospy.Time.now() + rospy.Duration(20)
            flag = True
            part_found = False
            while(rospy.Time.now() < endtime):
                for part in parts.keys():
                    rospy.loginfo_once("Needed Parts")
                    if(part_type.type in parts[part] and not part_found):
                        rospy.loginfo(('In: {location}'.format(location=part)))
                        rospy.loginfo(('Part: {location}'.format(location=part_type.type)))
                        rospy.loginfo(('Location(World Frame): {location}'.format(location=parts[part][part_type.type])))
                        rospy.loginfo("="*50)
                        flag = False
                        part_found = True
                        break
            if(flag):
                rospy.logwarn_once(('{location} not found'.format(location=part_type.type)))
                    
    # if(len(assembly_order_shipment) > 0):
    #     assembly_order_products = assembly_order_shipment[0].products
    #     for part in assembly_order_products:
    #         assembly_order_parts.append(part.type)
    
    
def check_model(agv_id, model_type, model_pose):
    if(model_type not in parts[agv_id]):
        parts[agv_id][model_type] = [model_pose]
    else:
        parts[agv_id][model_type].append([model_pose])




def logical_camera_agv1_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
        
            agv_id = "agv"+str(1)

            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)
                
        
def logical_camera_agv2_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
            agv_id = "agv"+str(2)
            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)
        
def logical_camera_agv3_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
            agv_id = "agv"+str(3)
            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)
        
def logical_camera_agv4_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
            agv_id = "agv"+str(4)
            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)

def logical_camera_bins0_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
            agv_id = "bin"+str(0)
            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)
        
def logical_camera_bins1_callback(data):
        """
        This callback method prints a statement on the terminal when it is triggered.

        Returns
        -------
        None
        """
        if(data):
            agv_id = "bin"+str(1)
            if(len(data.models)>0):
                parts[agv_id] = {}
                for model in data.models:
                    transform = broadcast(model.pose, agv_id+model.type)
                    check_model(agv_id, model.type, transform)
        



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
            
        rospy.Subscriber("/ariac/quality_control_sensor_1", LogicalCameraImage, quality_control_sensor_1_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_2", LogicalCameraImage, quality_control_sensor_2_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_3", LogicalCameraImage, quality_control_sensor_3_callback)
        rospy.Subscriber("/ariac/quality_control_sensor_4", LogicalCameraImage, quality_control_sensor_4_callback)
        rospy.Subscriber("/ariac/logical_camera_agv1", LogicalCameraImage, logical_camera_agv1_callback)
        rospy.Subscriber("/ariac/logical_camera_agv2", LogicalCameraImage, logical_camera_agv2_callback)
        rospy.Subscriber("/ariac/logical_camera_agv3", LogicalCameraImage, logical_camera_agv3_callback)
        rospy.Subscriber("/ariac/logical_camera_agv4", LogicalCameraImage, logical_camera_agv4_callback)
        rospy.Subscriber("/ariac/logical_camera_bins0", LogicalCameraImage, logical_camera_bins0_callback)
        rospy.Subscriber("/ariac/logical_camera_bins1", LogicalCameraImage, logical_camera_bins1_callback)
        bool = ariac.status()
        if(bool):
            rospy.Subscriber("/ariac/orders", Order, order_callback)
            rospy.spin()

    except rospy.ROSInterruptException:
        pass