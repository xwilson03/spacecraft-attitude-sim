#pragma once

import adcs.Actuator;
import adcs.Magnetorquer;
import adcs.PDController;
import adcs.ReactionWheel;
import adcs.Rigidbody;
import adcs.Spacecraft;
import adcs.Transform;

#include <Eigen/Dense>
#include <adcs_msgs/msg/actuator.hpp>
#include <adcs_msgs/msg/magnetorquer.hpp>
#include <adcs_msgs/msg/pd_controller_float.hpp>
#include <adcs_msgs/msg/reaction_wheel.hpp>
#include <adcs_msgs/msg/rigidbody.hpp>
#include <adcs_msgs/msg/spacecraft.hpp>
#include <adcs_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

using Eigen::Quaternionf;
using Eigen::Vector3f;
using adcs::Actuator;
using adcs::Magnetorquer;
using adcs::PDController;
using adcs::ReactionWheel;
using adcs::Rigidbody;
using adcs::Spacecraft;
using adcs::Transform;
using rclcpp::Publisher;
using rclcpp::create_publisher;


namespace adcs {

class SimSnooper : public rclcpp::Node {

    const Spacecraft& mCraft;
    adcs_msgs::msg::Spacecraft mMsg;
    Publisher<adcs_msgs::msg::Spacecraft>::SharedPtr mCraftPub;

    void toMsg (const Vector3f& in, geometry_msgs::msg::Vector3& out);
    void toMsg (const Quaternionf& in, geometry_msgs::msg::Quaternion& out);
    void toMsg (const Actuator& in, adcs_msgs::msg::Actuator& out);
    void toMsg (const Magnetorquer& in, adcs_msgs::msg::Magnetorquer& out);
    void toMsg (const PDController<float>& in, adcs_msgs::msg::PDControllerFloat& out);
    void toMsg (const ReactionWheel& in, adcs_msgs::msg::ReactionWheel& out);
    void toMsg (const Rigidbody& in, adcs_msgs::msg::Rigidbody& out);
    void toMsg (const Spacecraft& in, adcs_msgs::msg::Spacecraft& out);
    void toMsg (const Transform& in, adcs_msgs::msg::Transform& out);

public:

    SimSnooper(
        const Spacecraft& aCraft
    )
    : Node("sim_snooper")
    , mCraft(aCraft)
    , mCraftPub(create_publisher<adcs_msgs::msg::Spacecraft>("craft", 10))
    {};

    void publish();
};

} // namespace sdcs
