#ROS
import imp
from numpy import block
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
#Utils
import tkinter
import matplotlib.pyplot as plt
import cv2
import yaml
from collections import namedtuple
from multiprocessing import Process


def show_map(map):
    plt.imshow(map)
    plt.show()


#Named tuple representing a single waypoint
Waypoint = namedtuple('Waypoint','x y')

class NavigateThroughPosesClient(Node):
    def __init__(self,yaml_path:str,density:int,collision_range:int):
        super().__init__('navigate_through_poses_client')
        self._action_client = ActionClient(self,NavigateThroughPoses,'/navigate_through_poses')
#User params:
        self.density = density
        self.collision_range = collision_range
#Map params:
        with open(yaml_path,'r') as file:
            data = yaml.safe_load(file)
        self.origin = Waypoint(data['origin'][0],data['origin'][1])
        self.resolution = data['resolution']
        self.map = cv2.imread(data['image'])
        self.waypoint_array = []
        self.robot_frame_waypoint_array = []

    def send_goal(self):
        self.set_waypoints()
        msg = NavigateThroughPoses.Goal()
        goals = []
        for waypoint_ in self.robot_frame_waypoint_array:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = waypoint_.x
            waypoint.pose.position.y = waypoint_.y
            waypoint.pose.position.z = 0.
            waypoint.pose.orientation.x = 0.
            waypoint.pose.orientation.y = 0.
            waypoint.pose.orientation.z = 0.
            waypoint.pose.orientation.w = 1.
            goals.append(waypoint)
        msg.poses = goals
        print('waiting for server')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(msg)
    
    def set_waypoints(self):
        waypoint_array = []
# Add available waypoints based on selected density
        for i in range(0,len(self.map),self.density):
            for j in range(0,len(self.map[0]),self.density):
                if self.map[i][j][0] == 254:
                    waypoint_array.append(Waypoint(i,j))
# create array of valid waypoints based on collision_range param         
        valid_waypoint_array = [waypoint for waypoint in waypoint_array if self.check_safety(waypoint)]
#########DEBUG################        
        for waypoint in valid_waypoint_array:   
            self.map[waypoint.x,waypoint.y] = [0,255,0] #Mark valid waypoints
        p = Process(target=show_map,args=(self.map,)) #Show valid waypoints in different process (app doesn't block)
        p.start()
#########DEBUG################  
# Set proper waypoint coordinates based on map params
        for waypoint in valid_waypoint_array:
            x = waypoint.x * self.resolution + self.origin.x
            y = waypoint.y * self.resolution + self.origin.y
            world_frame_waypoint = Waypoint(x,y)
########DEBUG################
            print('='*20)
            print(waypoint.x,waypoint.y)
            print(world_frame_waypoint.x,world_frame_waypoint.y)
            print('='*20)
########DEBUG################  
            self.robot_frame_waypoint_array.append(world_frame_waypoint)

    def check_safety(self,waypoint:Waypoint): #method for checking if selected waypoint is close to occupied or unknown space
        xbegin,xend = waypoint.x - self.collision_range,waypoint.x + self.collision_range
        ybegin,yend = waypoint.y - self.collision_range,waypoint.y + self.collision_range
        for i in range(xbegin,xend + 1):
            for j in range(ybegin,yend + 1):
                if (self.map[i][j][0] == 205 or self.map[i][j][0] == 0):
                    self.map[waypoint.x][waypoint.y] = [255,0,0] #DEBUG mark invalid waypoints
                    return False
        return True

def main(args = None):
    rclpy.init(args=args)
    action_client = NavigateThroughPosesClient('/map/map.yaml',5,3)
    future = action_client.send_goal()
    print("goal sent")
    rclpy.spin_until_future_complete(action_client,future)

if __name__ == '__main__':
    main()

