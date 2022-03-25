#include "read_rssi_at_waypoint/read_rssi_at_waypoint.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{ //start namespace

// ReadRssiAtWaypoint::ReadRssiAtWaypoint() : is_enabled_(true), n_measurements_(10){}

// ReadRssiAtWaypoint::~ReadRssiAtWaypoint(){}

void ReadRssiAtWaypoint::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name)
{
    // create publisher in parent??
    auto node = parent.lock();
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
};

bool ReadRssiAtWaypoint::processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_pose_index
    )
{
    if(!is_enabled_){
        return true;
    }
    RCLCPP_INFO(logger_,"Number of measurements is %i",n_measurements_);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    return true;
}
} //end namespace
PLUGINLIB_EXPORT_CLASS(
    nav2_waypoint_follower::ReadRssiAtWaypoint,
    nav2_core::WaypointTaskExecutor
)