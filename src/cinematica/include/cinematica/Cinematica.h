#ifndef CINEMATICA_H
#define CINEMATICA_H

#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/actions_list.hpp"
#include "shared_interfaces/msg/direction_list.hpp"
#include "shared_interfaces/msg/motor_velocities.hpp"
#include "shared_interfaces/msg/motor_velocities_list.hpp"
#include <map>

class Cinematica : public rclcpp::Node{
    public:
        Cinematica();

    private:

        double _axleLength;   // distância entre as rodas, em metros
        double _wheelSpeedMax; // velocidade de roda que corresponde a PWM 100%, em m/s
        double _spinSpeed;
        bool _verbose;

        shared_interfaces::msg::MotorVelocities _inverseKinematics(int32_t id, double linear, double angular);

        void _PreProcessActions(int32_t id, double &linear, double &angular);

        void directionCallback(const shared_interfaces::msg::DirectionList::SharedPtr msg);
        void actionCallback(const shared_interfaces::msg::ActionsList::SharedPtr msg);

        rclcpp::Subscription<shared_interfaces::msg::DirectionList>::SharedPtr _directionSubscriber;
        rclcpp::Subscription<shared_interfaces::msg::ActionsList>::SharedPtr _actionsSubscriber;

        rclcpp::Publisher<shared_interfaces::msg::MotorVelocitiesList>::SharedPtr _motorVelocitiesPublisher;

        std::map<int32_t, shared_interfaces::msg::Actions> _lastaction;

};

#endif
