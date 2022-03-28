#include "nav2_read_rssi_at_waypoint/read_rssi_at_waypoint.hpp"
#include "nav2_util/node_utils.hpp"
// include custom message
#include "rosbot_interfaces/msg/rssi_at_waypoint.hpp"

namespace nav2_read_rssi_at_waypoint
{ //start namespace

ReadRssiAtWaypoint::ReadRssiAtWaypoint() : is_enabled_(true), n_measurements_(10){}

ReadRssiAtWaypoint::~ReadRssiAtWaypoint(){}

void ReadRssiAtWaypoint::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name)
{
    auto node = parent.lock();

    // rssi_data_msg = std::make_shared<rosbot_interfaces::msg::RssiAtWaypoint>(); //change for auto message in processAtWaypoint?

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
        rssi_data_publisher = node->create_publisher<rosbot_interfaces::msg::RssiAtWaypoint>("rssi_data",(10));
    }
}

bool ReadRssiAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose __attribute__((unused)),
    const int & curr_pose_index __attribute__((unused))
    )
{
    if(!is_enabled_){
        return true;
    }
    auto msg = rosbot_interfaces::msg::RssiAtWaypoint();
    msg.coordinates.x = curr_pose.pose.position.x;
    msg.coordinates.y = curr_pose.pose.position.y;
    msg.coordinates.z = curr_pose.pose.position.z;
    msg.rssi = -15.45;
    rssi_data_publisher->publish(msg);
    return true;
}
} //end namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_read_rssi_at_waypoint::ReadRssiAtWaypoint,
  nav2_core::WaypointTaskExecutor)