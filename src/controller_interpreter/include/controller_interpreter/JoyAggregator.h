#ifndef JOY_AGGREGATOR_H
#define JOY_AGGREGATOR_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "shared_interfaces/msg/joy_list.hpp"

#include <vector>
#include <map>

namespace controller_interpreter {

class JoyAggregator : public rclcpp::Node {
public:

    JoyAggregator();

private:

    int32_t _numRobots;

    std::map<int32_t, sensor_msgs::msg::Joy> _lastJoys;

    void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg, int32_t robotId);
    void _timerCallback();

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> _joySubscribers;
    rclcpp::Publisher<shared_interfaces::msg::JoyList>::SharedPtr _joyListPublisher;
    rclcpp::TimerBase::SharedPtr _timer;

};

}

#endif