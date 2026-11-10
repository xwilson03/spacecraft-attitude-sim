#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include "simsnooper.hpp"

import adcs.Spacecraft;

using adcs::Spacecraft;


namespace adcs {

void SimSnooper::publish() const {
    QuaternionStamped msg;

    msg.header.stamp = now();
    msg.header.frame_id = "world";

    const auto attitude = mCraft.rotation();
    msg.quaternion.set__w(attitude.w());
    msg.quaternion.set__x(attitude.w());
    msg.quaternion.set__y(attitude.y());
    msg.quaternion.set__z(attitude.z());

    mCraftAttitudePub->publish(msg);
}

} // namespace adcs
