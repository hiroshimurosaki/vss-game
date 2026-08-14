#ifndef DIRECTION_NODE_H
#define DIRECTION_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/joy_list.hpp"
#include "shared_interfaces/msg/direction_list.hpp"

namespace controller_interpreter {

class DirectionNode : public rclcpp::Node {
public:

    DirectionNode();

private:

    // Como o produtor do Joy reporta os gatilhos. Ver o construtor para a
    // medição que define cada um.
    enum class TriggerMode {
        SDL,        // solto  0.0, fundo -1.0  — o game_controller_node
        UNIT,       // solto  0.0, fundo +1.0
        SIGNED,     // solto +1.0, fundo -1.0  — joy_node clássico
    };

    double _maxLinearVelocity;
    double _maxAngularVelocity;
    bool _invertDirection;
    TriggerMode _triggerMode;
    bool _verbose;

    // Converte a leitura crua de um gatilho para [0.0, 1.0], com 0 = solto.
    double _normalizeTrigger(double raw) const;

    void _joyListCallback(const shared_interfaces::msg::JoyList::SharedPtr msg);

    rclcpp::Publisher<shared_interfaces::msg::DirectionList>::SharedPtr _directionPublisher;
    rclcpp::Subscription<shared_interfaces::msg::JoyList>::SharedPtr _joyListSubscriber;

};

}

#endif