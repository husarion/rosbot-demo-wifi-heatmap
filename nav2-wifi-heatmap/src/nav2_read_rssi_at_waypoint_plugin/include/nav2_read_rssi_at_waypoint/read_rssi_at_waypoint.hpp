#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__READ_RSSI_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__READ_RSSI_AT_WAYPOINT_HPP_
#pragma once

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
// include custom message
#include "rosbot_interfaces/msg/rssi_at_waypoint.hpp"
namespace nav2_read_rssi_at_waypoint
{ //start namespace

class ReadRssiAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:

    ReadRssiAtWaypoint();

    ~ReadRssiAtWaypoint();

    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & plugin_name
    );

    bool processAtWaypoint(
        const geometry_msgs::msg::PoseStamped & curr_pose __attribute__((unused)),
        const int & curr_pose_index __attribute__((unused))
    );
protected:
    bool is_enabled_;
    int n_measurements_;
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
    rosbot_interfaces::msg::RssiAtWaypoint::SharedPtr rssi_data_msg; //message declaration
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<rosbot_interfaces::msg::RssiAtWaypoint>> rssi_data_publisher;
}; //end namespace
}
#endif