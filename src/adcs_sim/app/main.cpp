import adcs.MsgHelpers;
import adcs.Rigidbody;
import adcs.Spacecraft;

#include <Eigen/Dense>
#include <adcs_msgs/msg/spacecraft.hpp>
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

using Eigen::Vector3f;
using adcs::Rigidbody;
using adcs::Spacecraft;
using rclcpp::Node;
using rclcpp::Publisher;
using rclcpp::TimerBase;


class SimNode : public Node {

    Spacecraft mSpacecraft;
    Rigidbody  mPlanet;
    const float mDeltaTime = 0.001f;
    float mAbsoluteTime = 0.0f;

    adcs_msgs::msg::Spacecraft mMsg;
    Publisher<adcs_msgs::msg::Spacecraft>::SharedPtr mSpacecraftPub;
    TimerBase::SharedPtr mTimer;

    void step () {
        mAbsoluteTime += mDeltaTime;

        mSpacecraft.step(mAbsoluteTime, mDeltaTime);
        mPlanet.step(mDeltaTime);

        adcs::toMsg(mSpacecraft, mMsg);
        mSpacecraftPub->publish(mMsg);
    }

public:

    SimNode()
    : Node("sim_node")
    , mSpacecraftPub(create_publisher<adcs_msgs::msg::Spacecraft>("adcs/spacecraft", 10))
    , mTimer(create_timer(std::chrono::duration<float>(mDeltaTime), [this]() { this->step(); }))
    {
        const float& pi = std::numbers::pi_v<float>;

        mSpacecraft.setScale(Vector3f(1,1,1) / 64);
        mSpacecraft.setRotationalInertia( Vector3f(4454.7f, 4061.2f, 5967.6f) ); // kgm^2. Galileo's pre-launch values (https://arxiv.org/pdf/gr-qc/0104064 section F.1)
        mSpacecraft.setAngularVelocity( Vector3f(pi / 10.0f, 0.01f, 0.01f) );    // rad/s. Galileo spun at 3rpm around its major axis; we'll spin around the intermediate.

        mPlanet.setRotation( Vector3f(-45.0f, 0.0f, 15.0f) * pi / 180.0f );
        mPlanet.setPosition( Vector3f(5.5f, 4.0f, 4.5f) );
        mPlanet.setAngularVelocity( Vector3f(0.f, -0.01f, 0.0f) );
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
