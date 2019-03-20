#! /usr/bin/env python
# -*- coding=utf-8 -*-
import socket
import time,os,fcntl
import struct
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
#import numpy as np

#message proto
# id | length | data
def recv_msg(sock):
    try:
        # Read message length and unpack it into an integer
        raw_id = recvall(sock, 4)
        if not raw_id:
            return None
        id = struct.unpack('>I', raw_id)[0]
        if id == 1565888:
            print "receive id: ",str(id)
        else:
            return None
        raw_msglen = recvall(sock, 4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        print "receive data number",str(msglen)
        # Read the message data
        return recvall(sock, msglen)
    except Exception ,e:
        return None



def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = ''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

##把接受的数据重新打包成ros topic发出去
def msg_construct(data):
    print"decode"
    list = data.split(',',20)
    Cloud = PointCloud2()
    Cloud.header.seq = int(list[1])#uint32
    Cloud.header.frame_id = str(list[0])
    Cloud.header.stamp = rospy.Time.now()
    Cloud.height = int(list[2])#uint32
    Cloud.width = int(list[3])#uint32

    Fieldx = PointField()
    Fieldx.name = "x"
    Fieldx.offset = int(list[4])#uint32
    Fieldx.datatype = int(list[5])
    Fieldx.count = int(list[6])
    Cloud.fields.append(Fieldx)

    Fieldy = PointField()
    Fieldy.name = "y"
    Fieldy.offset = int(list[7])#uint32
    Fieldy.datatype = int(list[8])
    Fieldy.count = int(list[9])
    Cloud.fields.append(Fieldy)

    Fieldz= PointField()
    Fieldz.name = "z"
    Fieldz.offset = int(list[10])#uint32
    Fieldz.datatype = int(list[11])
    Fieldz.count = int(list[12])
    Cloud.fields.append(Fieldz)

    Fieldi = PointField()
    Fieldi.name = "intensity"
    Fieldi.offset = int(list[13])#uint32
    Fieldi.datatype = int(list[14])
    Fieldi.count = int(list[15])
    Cloud.fields.append(Fieldi)

    if list[16]=="false":
        Cloud.is_bigendian = bool(0)
    else:
        Cloud.is_bigendian = bool(1)
    Cloud.point_step = int(list[17])#uint32
    Cloud.row_step = int(list[18])#uint32
    if list[19]=="false":
        Cloud.is_dense = bool(0)
    else:
        Cloud.is_dense = bool(1)

    count = Cloud.height*Cloud.row_step

    data_ = list[20]
    for i in range(0,count):
        Cloud.data+=data_[i]
    #print str(Cloud)
    print "save points from seq:",Cloud.header.seq
    return Cloud


#初始化socket，监听8000端口
odom_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
odom_socket.bind(('',8000))
odom_socket.listen(5)

(client,address) = odom_socket.accept()

rospy.init_node("server_node")
odom_pub = rospy.Publisher("/laser_cloud_surround",PointCloud2,queue_size=30)
r = rospy.Rate(100)

#设置noblock，否则会阻塞在接听，下面while不会一直循环，只有在有数据才进行下一次循环
fcntl.fcntl(client, fcntl.F_SETFL, os.O_NONBLOCK)


while not rospy.is_shutdown():
    data = recv_msg(client)
    if data:
        odom_pub.publish(msg_construct(data))
    r.sleep()