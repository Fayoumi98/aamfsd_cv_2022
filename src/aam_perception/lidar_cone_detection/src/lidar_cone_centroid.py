#!/usr/bin/env python

from cmath import cos, sin
from threading import local
import numpy as np
import math
from numpy.core.fromnumeric import shape, size
from numpy.lib.function_base import append
from numpy.lib.type_check import asfarray
import rospy
from sensor_msgs import msg
import tf
import matplotlib
import matplotlib.pyplot as plt
import ros_numpy as rnp
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
import sys
import random


class projection_pipeline():

    def __init__(self, namespace = 'lidar_cone_centroid'):

        rospy.init_node("lidar_cone_centroid", anonymous = True)

        self.no_ground_pc = rospy.Subscriber("/geomtrical_filter_points",PointCloud2,self.pc_callback)
        self.centroidpublisher = rospy.Publisher('/centroid_lidar_cones_marker', MarkerArray,queue_size=0)
        self.thresh=2
    



    def pc_callback(self, no_ground_pc):
        xyz_points = rnp.point_cloud2.pointcloud2_to_xyz_array(no_ground_pc, remove_nans=True)
        avgx,avgy=self.centroid(xyz_points)
        self.centroid_visuals(avgx,avgy)







    def centroid_visuals(self,resultX,resultY):
        c = 0

        self.rviz_msg = Marker()
        self.rviz_msg_array = MarkerArray()

        while c < len(resultX):
        
            
            x_cone = resultX[c]
            y_cone = resultY[c]

            c +=1
            count = 0
            MARKERS_MAX = 100

            self.rviz_msg = Marker()
            self.rviz_msg.header.frame_id = "os1_sensor"
            self.rviz_msg.ADD
            self.rviz_msg.SPHERE
            self.rviz_msg.pose.position.x = x_cone
            self.rviz_msg.pose.position.y = y_cone
            self.rviz_msg.pose.position.z = 0
            self.rviz_msg.lifetime = rospy.Duration(0.5)
            self.rviz_msg.pose.orientation.w = 1
            self.rviz_msg.scale.x = 1
            self.rviz_msg.scale.y = 1
            self.rviz_msg.scale.z = 1
            self.rviz_msg.color.a = 1

            self.rviz_msg.mesh_resource = "package://aamfsd_description/meshes/cone_blue.dae"
            self.rviz_msg.type = Marker.MESH_RESOURCE
            self.rviz_msg.mesh_use_embedded_materials = True


            if(count > MARKERS_MAX):
                self.rviz_msg_array.markers.pop(0)
            
            self.rviz_msg_array.markers.append(self.rviz_msg)
            m = 0
            id = 0
            for m in self.rviz_msg_array.markers:
                m.id = id
                id += 1
        
        self.centroidpublisher.publish(self.rviz_msg_array)

        return






    def centroid(self, xyz_points):
        angle = 0
        distance = 0
        ang = []
        dist = []
        list = xyz_points
        avgx = []
        avgy = []
        sumx = 0
        sumy = 0
        #print(size(list))
        count=0
        if size(list) != 0:
            for idx in range(len(list)-1):

                if list[idx][2]<0.5:
                    angle = math.atan(list[idx][1]/list[idx][0]) 
                    distance = math.sqrt((list[idx][0]**2) + (list[idx][1]**2)) 

                ang.append(angle)
                dist.append(distance) 


        i = 0


    
        if size(list) != 0:
            for idx in range(len(list)-1):
                if(ang[idx-1]- ang[idx]<0.02  and dist[idx]-dist[idx-1]) < 1 and dist[idx]-dist[idx-1] > -1:
                    sumx+=list[idx][0]
                    sumy+=list[idx][1]
                    count=count+1
                    #print('out number')
                else:
                    if count == 0:
                        continue
                    avgx.append(sumx/count)
                    avgy.append(sumy/count)
                    sumx=0
                    sumy=0
                    count=0


            #plt.scatter(avgx, avgy)
            #print('no of cones is ')
            #print(len(avgx))
            #plt.pause(.01)
        return avgx,avgy





if __name__ == '__main__':
    try:
        project = projection_pipeline()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
