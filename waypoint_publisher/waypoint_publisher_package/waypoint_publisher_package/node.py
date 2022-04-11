#ROS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
#Utils
import tkinter
import matplotlib.pyplot as plt
import cv2
import yaml
from collections import namedtuple
from multiprocessing import Process

# Function displaying map of waypoints
def show_map(map):
    plt.imshow(map)
    plt.axis('off')
    plt.show()

#Named tuple representing a single waypoint
Waypoint = namedtuple('Waypoint','x y')

class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_client')
#Create action client
        self._action_client = ActionClient(self,FollowWaypoints,'/follow_waypoints')
#Subscribe to topic sending trigger message
        self.publisher = self.create_publisher(Empty,'/heatmap_generator_trigger',1)
#User params:
        self.declare_parameter('density',8)
        self.declare_parameter('collision_range',4)
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
#Show valid waypoints in different process (app doesn't block)
        self.p = Process(target=show_map,args=(self.map,))

#Send created waypoints to FollowWaypoints action server
    def send_goal(self):
#Generate waypoints based on saved map and user params
        self.set_waypoints()
        msg = FollowWaypoints.Goal()
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
        self.get_logger().info('waiting for server...')
#Send goal and wait for result
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(msg)
        self._send_goal_future.add_done_callback(self.response_callback)

#Get response on goal sent
    def response_callback(self,future):
        goal_handle = future.result()
        self.get_logger().info("Goal received")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

#Respond to action completed
    def result_callback(self,future):
#Publlish trigger message
        msg = Empty()
        self.get_logger().info("Publishing trigger...")
        self.publisher.publish(msg)
#Kill process displaying waypoints
        self.p.kill()
        self.get_logger().info("Goal achieved")
#Kill node
        rclpy.shutdown()

#Create array of waypoints based on saved map and user params
    def set_waypoints(self):
        waypoint_array = []
# Add available waypoints based on selected density
        for i in range(0,len(self.map),self.density):
            for j in range(0,len(self.map[0]),self.density):
                if self.map[i][j][0] == 254:
                    waypoint_array.append(Waypoint(i,j))
# Create array of valid waypoints based on collision_range param         
        valid_waypoint_array = [waypoint for waypoint in waypoint_array if self.check_safety(waypoint)]
        valid_waypoint_array= valid_waypoint_array
# Show map with valid waypoints     
        for waypoint in valid_waypoint_array:   
            self.map[waypoint.x,waypoint.y] = [0,255,0] #Mark valid waypoints
        self.p.start() #Start process displaying waypoints

# Create real-world waypoints based on map waypoints
        for waypoint in valid_waypoint_array:
            x = waypoint.y * self.resolution + self.origin.x
            y = (len(self.map - 1) - waypoint.x) * self.resolution + self.origin.y
            world_frame_waypoint = Waypoint(x,y)
            self.robot_frame_waypoint_array.append(world_frame_waypoint)

# Check if waypoint is not too close to occupied or unknown space
    def check_safety(self,waypoint:Waypoint):
        xbegin,xend = waypoint.x - self.collision_range,waypoint.x + self.collision_range
        ybegin,yend = waypoint.y - self.collision_range,waypoint.y + self.collision_range
        for i in range(xbegin,xend + 1):
            for j in range(ybegin,yend + 1):
                if (self.map[i][j][0] == 205 or self.map[i][j][0] == 0):
                    self.map[waypoint.x][waypoint.y] = [255,0,0]
                    return False
        return True

def main(args = None):
    rclpy.init(args=args)
    action_client = FollowWaypointsClient()
    action_client.send_goal()
    rclpy.spin(action_client)
if __name__ == '__main__':
    main()

