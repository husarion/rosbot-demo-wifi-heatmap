#ROS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
#Utils
import tkinter
import matplotlib.pyplot as plt
import cv2
import yaml
from collections import namedtuple
from multiprocessing import Process
from std_msgs.msg import Bool

# Function displaying map of waypoints
def show_map(map):
    plt.imshow(map)
    plt.show()

#Named tuple representing a single waypoint
Waypoint = namedtuple('Waypoint','x y')

class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_client')
        self._action_client = ActionClient(self,FollowWaypoints,'/follow_waypoints')
        self.publisher = self.create_publisher(Bool,'/heatmap_generator_trigger',1)
        self.p = Process(target=show_map,args=(self.map,)) #Show valid waypoints in different process (app doesn't block)
#User params:
        self.declare_parameter('density',5)
        self.declare_parameter('collision_range',3)
        self.declare_parameter('path_to_yaml','/map/map.yaml')
        self.density = self.get_parameter('density').get_parameter_value().integer_value
        self.collision_range = self.get_parameter('collision_range').get_parameter_value().integer_value
#Map params:
        with open(self.get_parameter('path_to_yaml').get_parameter_value().string_value,'r') as file:
            data = yaml.safe_load(file)
        self.origin = Waypoint(data['origin'][0],data['origin'][1])
        self.resolution = data['resolution']
        self.map = cv2.imread(data['image'])
        self.waypoint_array = []
        self.robot_frame_waypoint_array = []

    def send_goal(self):
        self.set_waypoints()
        msg = FollowWaypoints.Goal()
        goals = []
        #FIX move unnecessary assignments out of the loop
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
        # Get action result
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(msg)
        self._send_goal_future.add_done_callback(self.response_callback)
    
    def response_callback(self,future):
        goal_handle = future.result()
        # msg = Bool
        # msg.data = True
        # self.publisher.publish(msg)
        # self.p.kill() # Kill process displaying waypoints
        self.get_logger().info("goal recieved")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def result_callback(self,future):
        goal_handle = future.result()
        msg = Bool
        msg.data = True
        self.publisher.publish(msg)
        self.p.kill() # Kill process displaying waypoints
        self.get_logger().info("goal achieved")
        rclpy.shutdown() #Kill node

    def set_waypoints(self):
        waypoint_array = []
# Add available waypoints based on selected density
        for i in range(0,len(self.map),self.density):
            for j in range(0,len(self.map[0]),self.density):
                if self.map[i][j][0] == 254:
                    waypoint_array.append(Waypoint(i,j))
# create array of valid waypoints based on collision_range param         
        valid_waypoint_array = [waypoint for waypoint in waypoint_array if self.check_safety(waypoint)]
        valid_waypoint_array= valid_waypoint_array #DEBUG get only one waypoint !nodebug!
#########DEBUG################        
        for waypoint in valid_waypoint_array:   
            self.map[waypoint.x,waypoint.y] = [0,255,0] #Mark valid waypoints
        self.map[len(self.map) - 1][0] = [0,0,255] #DEBUG show map origin
        self.p.start() #Start process displaying waypoints
#########DEBUG################  
# Set proper waypoint coordinates based on map params
        for waypoint in valid_waypoint_array:
            x = waypoint.y * self.resolution + self.origin.x
            y = (len(self.map - 1) - waypoint.x) * self.resolution + self.origin.y
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
    action_client = FollowWaypointsClient()
    future = action_client.send_goal()
    rclpy.spin(action_client)
if __name__ == '__main__':
    main()

