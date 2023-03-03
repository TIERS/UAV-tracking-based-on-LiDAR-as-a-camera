#!/usr/bin/python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import NavSatFix
import tf

 
rtk_quat = []

def opti_cb(data): 
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    f = open("optitrack.csv", "a")
    f.write("{},{},{},{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_nsec(),  data.pose.position.x  
                                            , data.pose.position.y,  data.pose.position.z
                                            , roll, pitch, yaw 
                                            , data.pose.orientation.x , data.pose.orientation.y 
                                            , data.pose.orientation.z , data.pose.orientation.w ))
    print(data.header.stamp.to_sec(),  data.pose.position.x  , data.pose.position.y,  data.pose.position.z)
    f.close()

    # f = open("evoOptkPose.csv", "a")
    # f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.position.x  
    #                                     , data.pose.position.y,  data.pose.position.z 
    #                                     , data.pose.orientation.x , data.pose.orientation.y 
    #                                     , data.pose.orientation.z , data.pose.orientation.w ))
    # print("-> Saving Optk Pose TUM : ", data.header.stamp.to_sec(),  data.pose.position.x   , data.pose.position.y,  data.pose.position.z ) 
    # f.close()


def drone_cb(data): 
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    # f = open("optitrack.csv", "a")
    # f.write("{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_sec(),  data.pose.position.x  
    #                                         , data.pose.position.y,  data.pose.position.z
    #                                         # , roll, pitch, yaw 
    #                                         , data.pose.orientation.x , data.pose.orientation.y 
    #                                         , data.pose.orientation.z , data.pose.orientation.w ))
    # print(data.header.stamp.to_sec(),  data.pose.position.x  , data.pose.position.y,  data.pose.position.z)
    # f.close()

    f = open("drone_t3_ours.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.position.x  
                                        , data.pose.position.y,  data.pose.position.z 
                                        , data.pose.orientation.x , data.pose.orientation.y 
                                        , data.pose.orientation.z , data.pose.orientation.w ))
    print("-> Saving NDT Pose TUM : ", data.header.stamp.to_sec(),  data.pose.position.x   , data.pose.position.y,  data.pose.position.z ) 
    f.close()

def mocap_cb(data): 
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    # f = open("optitrack.csv", "a")
    # f.write("{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_sec(),  data.pose.position.x  
    #                                         , data.pose.position.y,  data.pose.position.z
    #                                         # , roll, pitch, yaw 
    #                                         , data.pose.orientation.x , data.pose.orientation.y 
    #                                         , data.pose.orientation.z , data.pose.orientation.w ))
    # print(data.header.stamp.to_sec(),  data.pose.position.x  , data.pose.position.y,  data.pose.position.z)
    # f.close()

    f = open("drone_mocap.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.position.x  
                                        , data.pose.position.y,  data.pose.position.z 
                                        , data.pose.orientation.x , data.pose.orientation.y 
                                        , data.pose.orientation.z , data.pose.orientation.w ))
    print("-> Saving NDT Pose TUM : ", data.header.stamp.to_sec(),  data.pose.position.x   , data.pose.position.y,  data.pose.position.z ) 
    f.close()




def ndt_cb(data): 
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    # f = open("optitrack.csv", "a")
    # f.write("{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_sec(),  data.pose.position.x  
    #                                         , data.pose.position.y,  data.pose.position.z
    #                                         # , roll, pitch, yaw 
    #                                         , data.pose.orientation.x , data.pose.orientation.y 
    #                                         , data.pose.orientation.z , data.pose.orientation.w ))
    # print(data.header.stamp.to_sec(),  data.pose.position.x  , data.pose.position.y,  data.pose.position.z)
    # f.close()

    f = open("evoNDTPose1.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.position.x  
                                        , data.pose.position.y,  data.pose.position.z 
                                        , data.pose.orientation.x , data.pose.orientation.y 
                                        , data.pose.orientation.z , data.pose.orientation.w ))
    print("-> Saving NDT Pose TUM : ", data.header.stamp.to_sec(),  data.pose.position.x   , data.pose.position.y,  data.pose.position.z ) 
    f.close()

def loam_cb(data):  
    # f = open("VeloLIOPose.csv", "a")

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    # f.write("{},{},{},{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_nsec(),  data.pose.pose.position.x  
    #                                         , data.pose.pose.position.y,  data.pose.pose.position.z
    #                                         , roll, pitch, yaw 
    #                                         , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
    #                                         , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    # print("-> Saving Loam Pose: ", data.header.stamp.to_sec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    # f.close()


    f = open("evoTumVeloLIOPose.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.pose.position.x  
                                        , data.pose.pose.position.y,  data.pose.pose.position.z 
                                        , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
                                        , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    print("-> Saving Loam Pose TUM : ", data.header.stamp.to_sec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    f.close()

def fastlio_cb(data):  
    # f = open("FastLIOPose.csv", "a")

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]

    # f.write("{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_sec(),  data.pose.pose.position.x  
    #                                         , data.pose.pose.position.y,  data.pose.pose.position.z
    #                                         # , roll, pitch, yaw 
    #                                         , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
    #                                         , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    # print("-> Saving Loam Pose: ", data.header.stamp.to_nsec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    # f.close()


    f = open("EvoLIOPose.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec(),  data.pose.pose.position.x  
                                        , data.pose.pose.position.y,  data.pose.pose.position.z 
                                        , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
                                        , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    print("-> Saving Loam Pose TUM : ", data.header.stamp.to_sec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    f.close()


def lego_cb(data):  
    # f = open("legoPose.csv", "a")

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # f.write("{},{},{},{},{},{},{},{},{},{},{}, \n".format(data.header.stamp.to_sec(),  data.pose.pose.position.z, 
    #                                         data.pose.pose.position.x, data.pose.pose.position.y,  
    #                                          roll, pitch, yaw 
    #                                         , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
    #                                         , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    # # print("-> Saving Loam Pose: ", data.header.stamp.to_sec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    # f.close()


    f = open("evoTumLegoPose.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(data.header.stamp.to_sec() ,  data.pose.pose.position.z 
                                        , data.pose.pose.position.x , data.pose.pose.position.y   
                                        , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
                                        , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    print("-> Saving Loam Pose TUM : ", data.header.stamp.to_sec(),  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z ) 
    f.close()

def rtk_pose_cb(data):
    global rtk_quat   
    rtk_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    


def geo2ecef( lat, lon, height):
    a  = 6378137.0
    b  = 6356752.314245
    a2 = a * a
    b2 = b * b
    e2 = 1.0 - (b2 / a2)
    e  = e2 / (1.0 - e2)

    phi = np.deg2rad(lat)
    lmd = np.deg2rad(lon)
    
    cPhi = np.cos(phi)
    cLmd = np.cos(lmd)
    sPhi = np.sin(phi)
    sLmd = np.sin(lmd)
    
    N = a / np.sqrt(1.0 - e2 * sPhi * sPhi)
    
    x = (N + height) * cPhi * cLmd
    y = (N + height) * cPhi * sLmd
    z = ((b2 / a2) * N + height) * sPhi
    
    # return np.array([[x], [y], [z]])
    return x,y,z

def rtk_cb(msg):
    # msg = NavSatFix()
    # print(msg.altitude)
    # earthRadius = 6378.137;
    # lat = msg.latitude / 180 * 3.1415926
    # lon = msg.longitude  / 180 * 3.1415926
    # x = earthRadius * np.cos(lat)*np.cos(lon) * 1000;
    # y = earthRadius * np.cos(lat)*np.sin(lon) * 1000;
    # z = msg.altitude

    x,y,z = geo2ecef(msg.latitude, msg.longitude, msg.altitude)
    
    global rtk_quat  
 
    f = open("evoRTKPose.csv", "a")
    f.write("{} {} {} {} {} {} {} {}\n".format(msg.header.stamp.to_sec() , x, y, z  
                                        , rtk_quat[0]  , rtk_quat[1]  
                                        , rtk_quat[2] , rtk_quat[3] ))
    print("{} {} {} {} {} {} {} {}\n".format(msg.header.stamp.to_sec() , x, y, z  
                                        , rtk_quat[0]  , rtk_quat[1]  
                                        , rtk_quat[2] , rtk_quat[3] ))
    # print("-> Saving RTK Pose TUM : ", msg.header.stamp.to_sec(),  x   , y,  z ) 
    f.close()
    

def listener(): 
    rospy.init_node('listener', anonymous=True)
 
    # rospy.Subscriber("/aft_mapped_to_init", Odometry, lego_cb)                  # lego-loam
    rospy.Subscriber("/livox_odometry_mapped", Odometry, loam_cb)               # lio-livox
    # rospy.Subscriber("/odom_mapped", Odometry, loam_cb)                         # lili-om
    rospy.Subscriber("/Odometry", Odometry, fastlio_cb)                            # fast_lio

    rospy.Subscriber("/ndt_pose", PoseStamped, ndt_cb)   # optrick
    rospy.Subscriber("/vrpn_client_node/UWBTest/pose", PoseStamped, opti_cb)    # optrick
    # rospy.Subscriber("/vrpn_client_node/optitest/pose", PoseStamped, opti_cb)   # optrick
    # rospy.Subscriber("/vrpn_client_node/optk/pose", PoseStamped, opti_cb)   # optrick

    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, rtk_cb)   # optrick
    rospy.Subscriber("/mavros/local_position/pose",     PoseStamped, rtk_pose_cb)   # optrick
    rospy.Subscriber("/drone",     PoseStamped, drone_cb)
    rospy.Subscriber("/vrpn_client_node/drone/pose",     PoseStamped, mocap_cb)
    
    
    rospy.spin()

if __name__ == '__main__':
    listener()