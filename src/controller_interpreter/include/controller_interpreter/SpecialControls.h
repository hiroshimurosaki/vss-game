#ifndef SPECIAL_CONTROLS_H
#define SPECIAL_CONTROLS_H

#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/joy_list.hpp"
#include "shared_interfaces/msg/actions_list.hpp"

namespace controller_interpreter {

class SpecialControls : public rclcpp::Node
{
public:

    SpecialControls();

private:

    void _joyListCallback(const shared_interfaces::msg::JoyList::SharedPtr msg);

    rclcpp::Subscription<shared_interfaces::msg::JoyList>::SharedPtr _joyListSubscriber;
    rclcpp::Publisher<shared_interfaces::msg::ActionsList>::SharedPtr _actionsPublisher;

};

}

#endif