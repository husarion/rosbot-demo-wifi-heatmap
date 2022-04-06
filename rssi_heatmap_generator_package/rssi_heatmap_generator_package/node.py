#ROS
import rclpy
from rclpy.node import Node
from rosbot_interfaces.msg import RssiAtWaypoint
#Utils
from collections import namedtuple
import yaml

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

#Class attributes:
        self.raw_rssi_data = [] # array to store data from topic

    def rssi_data_callback(self,msg:RssiAtWaypoint):
        self.raw_rssi_data.append(msg) # store data sent from topic
    
def main(args = None):
    pass


if __name__ == '__main__':
    main()
