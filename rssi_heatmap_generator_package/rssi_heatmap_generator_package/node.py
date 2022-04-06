#ROS
import rclpy
from rclpy.node import Node
from rosbot_interfaces.msg import RssiAtWaypoint
#Utils
from collections import namedtuple
import yaml
import cv2

RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

Waypoint = namedtuple("Waypoint",'x y')


class HeatmapGenerator(Node):
    def __init__(self):
        super.__init__('heatmap_generator')
        self.subscription = self.create_subscription(RssiAtWaypoint,'/rssi_data',self.rssi_data_callback,10)
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

    def rssi_data_callback(self,msg:RssiAtWaypoint):
        x = int((msg.coordinates.x - self.map_origin.x) / self.map_resolution) #Change coordinates from real to map's
        y = (len(self.map[0]) - 1)  -  int((msg.coordinates.y - self.map_origin.y) / self.map_resolution) #Origin is set at left-bottom corner, so subtraction from map size is needed
        self.rssi_data.append(RssiWaypoint(x,y,msg.rssi)) # store data sent from topic 

def main(args = None):
    pass


if __name__ == '__main__':
    main()
