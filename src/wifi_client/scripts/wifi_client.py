#! /usr/bin/env python
# -*- coding=utf-8 -*-
import socket
import struct
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from sensor_msgs.msg import PointCloud2
import numpy

#message proto
# id |  length | data
def send_msg(sock, msg ,id):
    # Prefix each message with a 4-byte id and length (network byte order)
    msg = struct.pack('>I',int(id)) + struct.pack('>I', len(msg)) + msg
    print str(len(msg))
    sock.sendall(msg)


def odomCallback(msg):
    global odom_socket
    
    data = ""
    seq = msg.header.seq
    frameid = msg.header.frame_id
    stamp = msg.header.stamp
    height = msg.height
    width = msg.width

    fields_x_offset = msg.fields[0].offset
    fields_x_datatype = msg.fields[0].datatype
    fields_x_count = msg.fields[0].count
    
    fields_y_offset = msg.fields[1].offset
    fields_y_datatype = msg.fields[1].datatype
    fields_y_count = msg.fields[1].count

    fields_z_offset = msg.fields[2].offset
    fields_z_datatype = msg.fields[2].datatype
    fields_z_count = msg.fields[2].count

    fields_intensity_offset = msg.fields[3].offset
    fields_intensity_datatype = msg.fields[3].datatype
    fields_intensity_count = msg.fields[3].count

    is_bigendian = msg.is_bigendian
    point_step = msg.point_step
    row_step = msg.row_step
    is_dense = msg.is_dense


    id = 1565888
    print "send id: ",id
   # fields = msg.fields.pose.position.x
    #y = msg.pose.pose.position.y
    #orientation
    #orien_z = msg.pose.pose.orientation.z
    #orien_w = msg.pose.pose.orientation.w

    data += str(frameid) + "," + str(seq) + "," + str(height)+"," + str(width)\
            +"," + str(fields_x_offset)+ "," + str(fields_x_datatype)+ "," + str(fields_x_count)\
             +"," + str(fields_y_offset)+ "," + str(fields_y_datatype)+ "," + str(fields_y_count)\
             +"," + str(fields_z_offset)+ "," + str(fields_z_datatype)+ "," + str(fields_z_count)\
             +"," + str(fields_intensity_offset)+ "," + str(fields_intensity_datatype)+ "," + str(fields_intensity_count)\
            + "," + str(is_bigendian) +"," + str(point_step)+ "," + str(row_step)+ "," + str(is_dense)+ "," 

    count = height*row_step
    
    for i in range(0,count):
        data += msg.data[i]

    send_msg(odom_socket,data,id)


odom_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
odom_socket.connect(('10.42.0.1',8000))
print "ready to send: "
rospy.init_node('client_node')

rospy.Subscriber('/laser_cloud_surround',PointCloud2,odomCallback)

rospy.spin()