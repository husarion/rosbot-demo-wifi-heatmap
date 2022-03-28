#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__READ_RSSI_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__READ_RSSI_AT_WAYPOINT_HPP_
#pragma once

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{ //start namespace

class ReadRssiAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:

    ReadRssiAtWaypoint(): is_enabled_(true), n_measurements_(10){};

    ~ReadRssiAtWaypoint();

    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & plugin_name
    );

    bool processAtWaypoint(
        const geometry_msgs::msg::PoseStamped & curr_pose,
        const int & curr_pose_index
    );
protected:
    bool is_enabled_;
    int n_measurements_;
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
    //publisher
}; //end namespace
}
#endif