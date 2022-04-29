#ROS
from cv2 import imshow
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
from matplotlib import gridspec, pyplot as plt
import matplotlib
from matplotlib.colors import LinearSegmentedColormap



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
    def display_maps(self,map_with_waypoints,heatmap,final_map,rel_heatmap,rel_final_map,rssi_bounds):

        fig1  = plt.figure()
        fig1.suptitle("Absolute wifi rssi heatmap")
        gs = gridspec.GridSpec(ncols=1, nrows=2, wspace=0.000001,hspace=0.35, height_ratios=[25,1])
        ax1 = fig1.add_subplot(gs[0,0])
        ax2 = fig1.add_subplot(gs[1,0])
        norm1 = matplotlib.colors.Normalize(vmin = -100,vmax=0)
        cb1 = matplotlib.colorbar.ColorbarBase(ax2, cmap=cmapGR,norm=norm1,orientation='horizontal')
        ax1.imshow(final_map)
        ax1.axis('off')

        fig2  = plt.figure()
        fig2.suptitle("Relative wifi rssi heatmap")
        gs = gridspec.GridSpec(ncols=1, nrows=2, wspace=0.000001,hspace=0.35, height_ratios=[25,1])
        ax1 = fig2.add_subplot(gs[0,0])
        ax2 = fig2.add_subplot(gs[1,0])
        norm1 = matplotlib.colors.Normalize(vmin= rssi_bounds[0], vmax= rssi_bounds[1])
        cb1 = matplotlib.colorbar.ColorbarBase(ax2, cmap=cmapGR,norm=norm1,orientation='horizontal')
        ax1.imshow(rel_final_map)
        ax1.axis('off')

        fig3  = plt.figure()
        fig3.suptitle("Waypoints map")
        plt.imshow(map_with_waypoints)

        plt.show()
# Method run when receiving trigger message
    def trigger_callback(self,msg):
        self.get_logger().info("adding waypoints to map...")
        map_with_waypoints = add_waypoints(self.map,self.rssi_data,)
        self.get_logger().info('generating heatmap...')
        heatmap = generate_heatmap(self.rssi_data,len(self.map),len(self.map[0]),1,filtered=False)[0]
        rel_heatmap,rssi_bounds = generate_heatmap(self.rssi_data,len(self.map),len(self.map[0]),1,filtered=True,relative=True)
        self.get_logger().info('adding heatmap to map...')
        final_map = add_heatmap(self.map,heatmap)
        rel_final_map = add_heatmap(self.map,rel_heatmap)
        self.get_logger().info("maps generated, displaying")
        self.p = Process(target=self.display_maps,args=(map_with_waypoints,heatmap,final_map,rel_heatmap,rel_final_map,rssi_bounds))
        self.p.start()
        self.get_logger().info("maps generated, displaying")
        cv2.imwrite('/heatmaps/map_with_waypoints_{date}.jpg'.format(date = datetime.datetime.now()),map_with_waypoints)
        cv2.imwrite('/heatmaps/heatmap_{date}.jpg'.format(date = datetime.datetime.now()),final_map)
        cv2.imwrite('/heatmaps/rel_heatmap_{date}.jpg'.format(date = datetime.datetime.now()),rel_final_map)

def main(args = None):
    rclpy.init(args=args)
    heatmapgenerator = HeatmapGenerator()
    rclpy.spin(heatmapgenerator)
if __name__ == '__main__':
    main()