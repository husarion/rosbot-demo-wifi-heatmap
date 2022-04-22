#ROS
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from rosbot_interfaces.msg import RssiAtWaypoint
from std_msgs.msg import Empty
#Utils
from collections import namedtuple
import yaml
import cv2
from .submodules.generate_heatmap import generate_heatmap,add_heatmap,add_waypoints,cmapGR
import tkinter
import numpy as np
from multiprocessing import Process
import datetime


RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

Waypoint = namedtuple("Waypoint",'x y')

class HeatmapGenerator(Node):
    def __init__(self):
        super().__init__('heatmap_generator')
#Create rssi data subscriber
        self.subscription = self.create_subscription(RssiAtWaypoint,'/rssi_data',self.rssi_data_callback,10)
#Create trigger subsrciber
        self.subscription = self.create_subscription(Empty,'/heatmap_generator_trigger',self.trigger_callback,10)
#User params:
        self.declare_parameter('path_to_yaml','/map/map.yaml')
#Map params:
        with open(self.get_parameter('path_to_yaml').get_parameter_value().string_value,'r') as file:
            map_data = yaml.safe_load(file)
        self.map_origin = Waypoint(map_data['origin'][0],map_data['origin'][1])
        self.map_resolution = map_data['resolution']
        self.map = cv2.imread(map_data['image'])
#Class attributes:
        self.rssi_data = [] # array to store data from topic
        self.p = Process(target=self.display_maps,args=(self.map,))
        self.get_logger().info('Heatmap generator node starting...')



    def rssi_data_callback(self,msg:RssiAtWaypoint):
        x = int((msg.coordinates.x - self.map_origin.x) / self.map_resolution) #Change coordinates from real to map's
        y = (len(self.map))  -  int((msg.coordinates.y - self.map_origin.y) / self.map_resolution) #Origin is set at left-bottom corner, so subtraction from map size is needed
        data = RssiWaypoint(x,y,int(msg.rssi))
        self.rssi_data.append(data) # store data sent from topic

#Method running in separate process
    def display_maps(self,map_with_waypoints,heatmap,final_map,rel_heatmap,rel_final_map):
        fig,axs = plt.subplots(2,3)
        axs[0][0].imshow(map_with_waypoints)
        axs[0][0].axis("off")
        axs[0][0].set_title("Map with waypoints")
        axs[0][1].imshow(heatmap)
        axs[0][1].axis("off")
        axs[0][1].set_title("Heatmap")
        axs[0][2].imshow(final_map)
        axs[0][2].axis("off")
        axs[0][2].set_title("Heatmap with lidar data")

        axs[1][0].imshow(map_with_waypoints)
        axs[1][0].axis("off")
        axs[1][0].set_title("Map with waypoints")
        axs[1][1].imshow(rel_heatmap)
        axs[1][1].axis("off")
        axs[1][1].set_title("relative heatmap")
        axs[1][2].imshow(rel_final_map)
        axs[1][2].axis("off")
        axs[1][2].set_title("Relative heatmap with lidar data")

        plt.show()
    
    def trigger_callback(self,msg):
        self.get_logger().info("adding waypoints to map...")
        map_with_waypoints = add_waypoints(self.map,self.rssi_data,)
        self.get_logger().info('generating heatmap...')
        heatmap = generate_heatmap(self.rssi_data,len(self.map),len(self.map[0]),1,filtered=False)
        rel_heatmap = generate_heatmap(self.rssi_data,len(self.map),len(self.map[0]),1,filtered=True,relative=True)
        self.get_logger().info('adding heatmap to map...')
        final_map = add_heatmap(self.map,heatmap)
        rel_final_map = add_heatmap(self.map,rel_heatmap)
        self.get_logger().info("maps generated, displaying")
        self.p = Process(target=self.display_maps,args=(map_with_waypoints,heatmap,final_map,rel_heatmap,rel_final_map))
        self.p.start()
        self.get_logger().info("maps generated, displaying")
        cv2.imwrite('/heatmaps/map_with_waypoints_{date}.jpg'.format(date = datetime.datetime.now),map_with_waypoints)
        cv2.imwrite('/heatmaps/heatmap_{date}.jpg'.format(date = datetime.datetime.now),final_map)
        cv2.imwrite('/heatmaps/rel_heatmap_{date}.jpg'.format(date = datetime.datetime.now),rel_final_map)

def main(args = None):
    rclpy.init(args=args)
    heatmapgenerator = HeatmapGenerator()
    rclpy.spin(heatmapgenerator)
if __name__ == '__main__':
    main()