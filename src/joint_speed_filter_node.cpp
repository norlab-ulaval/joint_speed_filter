#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const int WINDOW_SIZE = 5;
const int LEFT_JOINT_INDEX = 0;
const int RIGHT_JOINT_INDEX = 1;

class JointSpeedFilterNode : public rclcpp::Node
{
public:
    JointSpeedFilterNode():
            Node("joint_speed_filter_node")
    {
        leftSidePublisher = this->create_publisher<std_msgs::msg::Float64>("left_wheel_vel", 10);
        rightSidePublisher = this->create_publisher<std_msgs::msg::Float64>("right_wheel_vel", 10);
        subscription = this->create_subscription<sensor_msgs::msg::JointState>("joint_states_in", 10, std::bind(&JointSpeedFilterNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::JointState& msgIn)
    {
        previousJointStates.push_back(msgIn);
        if(previousJointStates.size() > WINDOW_SIZE)
        {
            previousJointStates.pop_front();
        }
        else
        {
            return;
        }

        std_msgs::msg::Float64 leftWheelVel;
        leftWheelVel.data = float(previousJointStates.back().position[LEFT_JOINT_INDEX] - previousJointStates.front().position[LEFT_JOINT_INDEX]) /
                            (rclcpp::Time(previousJointStates.back().header.stamp) - rclcpp::Time(previousJointStates.front().header.stamp)).seconds();
        leftSidePublisher->publish(leftWheelVel);
        std_msgs::msg::Float64 rightWheelVel;
        rightWheelVel.data = float(previousJointStates.back().position[RIGHT_JOINT_INDEX] - previousJointStates.front().position[RIGHT_JOINT_INDEX]) /
                             (rclcpp::Time(previousJointStates.back().header.stamp) - rclcpp::Time(previousJointStates.front().header.stamp)).seconds();
        rightSidePublisher->publish(rightWheelVel);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftSidePublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightSidePublisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
    std::list<sensor_msgs::msg::JointState> previousJointStates;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSpeedFilterNode>());
    rclcpp::shutdown();
    return 0;
}
