#include <chrono>
#include <thread>

#include "nav2_read_rssi_at_waypoint/read_rssi_at_waypoint.hpp"
#include "nav2_util/node_utils.hpp"
// include custom message
#include "rosbot_interfaces/msg/rssi_at_waypoint.hpp"
#include "read_rssi.cpp"

namespace nav2_read_rssi_at_waypoint
{ //start namespace

ReadRssiAtWaypoint::ReadRssiAtWaypoint() : is_enabled_(true), n_measurements_(10){}
ReadRssiAtWaypoint::~ReadRssiAtWaypoint(){}

//Method running on plugin initialization
void ReadRssiAtWaypoint::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name)
{
    auto node = parent.lock();
    RCLCPP_INFO(logger_,"Starting...");

    if (!node) {
        throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
    }
    logger_ = node -> get_logger();
    //Params declaration
    nav2_util::declare_parameter_if_not_declared( //is plugin enabled
        node,
        plugin_name + ".enabled",
        rclcpp::ParameterValue(true));

    nav2_util::declare_parameter_if_not_declared( //number of measurements in one point
        node,
        plugin_name + ".number_of_measurements",
        rclcpp::ParameterValue(10));
    //Get class field values from params
    node->get_parameter(plugin_name + ".enabled",is_enabled_);
    node->get_parameter(plugin_name + ".number_of_measurements",n_measurements_);
    
    if(n_measurements_ == 0){
        is_enabled_ = false;
    }

    if(is_enabled_){
        RCLCPP_INFO(logger_,"Rssi Measurement plugin enabled");
    //Create and activate rssi data publisher
        rssi_data_publisher = node->create_publisher<rosbot_interfaces::msg::RssiAtWaypoint>("rssi_data",(10));
        rssi_data_publisher->on_activate();
    }
}
// Method running when waypoints achieved
bool ReadRssiAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose __attribute__((unused)),
    const int & curr_pose_index __attribute__((unused))
    )
{
    if(!is_enabled_){
        return true;
    }
    RCLCPP_INFO(logger_,"Measuring...");
    auto msg = rosbot_interfaces::msg::RssiAtWaypoint();
    // Get robot's position
    msg.coordinates.x = curr_pose.pose.position.x;
    msg.coordinates.y = curr_pose.pose.position.y;
    msg.coordinates.z = curr_pose.pose.position.z;
    int rssi_ = 0;
    int measurements_counter = 0;
    // Measure mean value of rssi at waypoint
    for (int i = 0; i < n_measurements_; i++){ //read rssi n_measurements_ times
       int rssi = read_rssi();
        if (rssi <= 0){
            rssi_ += rssi;
            measurements_counter++; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); //wait 0.5 sec between measurements
    }
    rssi_ = rssi_ /  measurements_counter;
    msg.rssi = rssi_;
    RCLCPP_INFO(logger_,"rssi = %i", &rssi_);
    rssi_data_publisher->publish(msg);
    return true;
}
} //end namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_read_rssi_at_waypoint::ReadRssiAtWaypoint,
  nav2_core::WaypointTaskExecutor)