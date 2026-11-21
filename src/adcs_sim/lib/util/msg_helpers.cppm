module;
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
export module adcs.MsgHelpers;

import adcs.Actuator;
import adcs.Magnetorquer;
import adcs.PDController;
import adcs.ReactionWheel;
import adcs.Rigidbody;
import adcs.Spacecraft;
import adcs.Transform;

using Eigen::Quaternionf;
using Eigen::Vector3f;
using adcs::Actuator;
using adcs::Magnetorquer;
using adcs::PDController;
using adcs::ReactionWheel;
using adcs::Rigidbody;
using adcs::Spacecraft;
using adcs::Transform;


export namespace adcs {

void toMsg (
    const Vector3f& in,
    geometry_msgs::msg::Vector3& out
) {
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}

void toMsg (
    const Quaternionf& in,
    geometry_msgs::msg::Quaternion& out
) {
    out.w = in.w();
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}

void toMsg (
    const Actuator& in,
    adcs_msgs::msg::Actuator& out
) {
    out.command_poll_interval = in.commandPollInterval();
    out.max_voltage           = in.maxVoltage();
    out.max_current           = in.maxCurrent();
    out.resistance            = in.resistance();
    out.motor_constant        = in.motorConstant();
    out.voltage               = in.voltage();
    out.next_voltage          = in.nextVoltage();
    out.time_to_next_poll     = in.timeToNextPoll();
}

void toMsg (
    const Magnetorquer& in,
    adcs_msgs::msg::Magnetorquer& out
) {
    out.command_poll_interval = in.commandPollInterval();
    out.max_voltage           = in.maxVoltage();
    out.resistance            = in.resistance();
    out.inductance            = in.inductance();
    out.dipole_per_amp        = in.dipolePerAmp();
    out.voltage               = in.voltage();
    out.current               = in.current();
    out.next_voltage          = in.nextVoltage();
    out.time_to_next_poll     = in.timeToNextPoll();
}

void toMsg (
    const PDController<float>& in,
    adcs_msgs::msg::PDControllerFloat& out
) {
    out.kp         = in.kP();
    out.kd         = in.kD();
    out.last_error = in.lastError();
}

void toMsg (
    const Transform& in,
    adcs_msgs::msg::Transform& out
) {
    toMsg(in.position(), out.position);
    toMsg(in.rotation(), out.rotation);
    toMsg(in.scale(),    out.scale);
}

void toMsg (
    const Rigidbody& in,
    adcs_msgs::msg::Rigidbody& out
) {
    toMsg(static_cast<const Transform&>(in), out.transform);
    toMsg(in.angularVelocity(),              out.angular_velocity);
    toMsg(in.torque(),                       out.torque);
    toMsg(in.rotationalInertia(),            out.rotational_inertia);
}

void toMsg (
    const ReactionWheel& in,
    adcs_msgs::msg::ReactionWheel& out
) {
    out.command_poll_interval       = in.commandPollInterval();
    out.wheel_rotational_inertia    = in.wheelRotationalInertia();
    out.wheel_axis                  = in.wheelAxis();
    out.time_to_next_poll           = in.timeToNextPoll();
    out.angular_momentum_goal       = in.angularMomentumGoal();
    out.next_angular_momentum_error = in.nextAngularMomentumError();
    toMsg(in.rigidbody(),          out.rigidbody);
    toMsg(in.actuator(),           out.actuator);
    toMsg(in.actuatorController(), out.actuator_controller);
}

void toMsg (
    const Spacecraft& in,
    adcs_msgs::msg::Spacecraft& out
) {
    toMsg(static_cast<const Rigidbody&>(in), out.rigidbody);
    toMsg(in.rWheelX(),    out.rwheel_x);
    toMsg(in.rWheelY(),    out.rwheel_y);
    toMsg(in.rWheelZ(),    out.rwheel_z);
    toMsg(in.magtorqueX(), out.magtorque_x);
    toMsg(in.magtorqueY(), out.magtorque_y);
    toMsg(in.magtorqueZ(), out.magtorque_z);
}

} // namespace adcs
