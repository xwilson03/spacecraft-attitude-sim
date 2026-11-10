#pragma once

#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>

import adcs.Spacecraft;

using adcs::Spacecraft;
using rclcpp::Publisher;
using rclcpp::create_publisher;
using geometry_msgs::msg::QuaternionStamped;


namespace adcs {

class SimSnooper : public rclcpp::Node {

    const Spacecraft& mCraft;
    Publisher<QuaternionStamped>::SharedPtr mCraftAttitudePub;

public:

    SimSnooper(
        const Spacecraft& aCraft
    )
    : Node("sim_snooper")
    , mCraft(aCraft)
    , mCraftAttitudePub(create_publisher<QuaternionStamped>("attitude", 10))
    {};

    void publish() const;
};

} // namespace sdcs
