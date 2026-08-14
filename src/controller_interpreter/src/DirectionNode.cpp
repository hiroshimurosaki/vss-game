#include "controller_interpreter/DirectionNode.h"

#include <algorithm>
#include <cmath>

namespace controller_interpreter {

DirectionNode::DirectionNode() : Node("direction") {

    this->declare_parameter("max_linear_velocity", 0.6);
    this->declare_parameter("max_angular_velocity", 5.0);
    this->declare_parameter("invert_direction", false);

    // Como o produtor do Joy reporta os gatilhos analógicos (L2/R2):
    //   "sdl"    -> solto = 0.0, fundo = -1.0   (o game_controller_node)
    //   "unit"   -> solto = 0.0, fundo = +1.0
    //   "signed" -> solto = +1.0, fundo = -1.0  (joy_node clássico)
    //
    // "sdl" é a convenção do repo inteiro, porque é a que o hardware fala.
    // Medido em 13/08/2026 com um DualShock 4, gatilho apertado até o fim:
    //
    //     repouso  L2  0.00   R2  0.00
    //     fundo    L2 -1.00   R2 -1.00
    //
    // O sinal negativo vem do próprio `game_controller_node`, que nega os
    // eixos do SDL. A IA, o simulador e o keyboard_input foram alinhados a
    // isso; o neutro continua sendo o zero do array, que ninguém esquece de
    // preencher.
    //
    // Errar isto não é sutil, e o sintoma não aponta para a causa: em "unit"
    // o clamp joga o -1.0 do fundo para 0.0, os dois gatilhos lêem 0 e o robô
    // fica IMÓVEL enquanto o volante continua funcionando. Para conferir, rode
    // `ros2 topic echo /joy_1` e aperte R2: axes[5] tem de ir a -1.0.
    this->declare_parameter("trigger_mode", "sdl");

    this->declare_parameter("verbose", false);

    _maxLinearVelocity = this->get_parameter("max_linear_velocity").as_double();
    _maxAngularVelocity = this->get_parameter("max_angular_velocity").as_double();
    _invertDirection = this->get_parameter("invert_direction").as_bool();
    _verbose = this->get_parameter("verbose").as_bool();

    const std::string mode = this->get_parameter("trigger_mode").as_string();

    if (mode == "sdl") {
        _triggerMode = TriggerMode::SDL;
    } else if (mode == "unit") {
        _triggerMode = TriggerMode::UNIT;
    } else if (mode == "signed") {
        _triggerMode = TriggerMode::SIGNED;
    } else {
        _triggerMode = TriggerMode::SDL;
        RCLCPP_WARN(this->get_logger(),
            "trigger_mode '%s' desconhecido. Usando 'sdl'.", mode.c_str());
    }

    _joyListSubscriber =
        this->create_subscription<shared_interfaces::msg::JoyList> ("/joy_list", 10,
            [this](const shared_interfaces::msg::JoyList::SharedPtr msg) {
                _joyListCallback(msg);
            });

    _directionPublisher =
        this->create_publisher<shared_interfaces::msg::DirectionList> ("/direction", 30);

    RCLCPP_INFO(this->get_logger(),
            "DirectionNode inicializado | v_lin max: %.2f | v_ang max: %.2f | "
            "invertido: %s | gatilhos: %s",
            _maxLinearVelocity, _maxAngularVelocity,
            _invertDirection ? "sim" : "não",
            mode.c_str());
}

double DirectionNode::_normalizeTrigger(double raw) const {
    // Sai sempre em [0.0, 1.0], com 0 = solto.
    double value = 0.0;
    switch (_triggerMode) {
        case TriggerMode::SDL:    value = -raw;             break;
        case TriggerMode::UNIT:   value = raw;              break;
        case TriggerMode::SIGNED: value = (1.0 - raw) / 2.0; break;
    }
    return std::clamp(value, 0.0, 1.0);
}

void DirectionNode::_joyListCallback(const shared_interfaces::msg::JoyList::SharedPtr msg) {

    auto direction_list = shared_interfaces::msg::DirectionList();

    for (const auto& robot_controller : msg->joys) {

        const auto& axes = robot_controller.joy.axes;
        const int32_t id = robot_controller.id;

        const double left_x = axes.size() > 0 ? axes[0] : 0.0;   // [-1.0, 1.0]

        // Índices padrão do game_controller_node: 4 = L2, 5 = R2.
        const double l2 = _normalizeTrigger(axes.size() > 4 ? axes[4] : 0.0);
        const double r2 = _normalizeTrigger(axes.size() > 5 ? axes[5] : 0.0);

        // R2 acelera, L2 ré. Como os dois já estão em [0, 1], a diferença fica em
        // [-1, 1] e o resultado respeita de fato o max_linear_velocity.
        const double throttle = std::clamp(r2 - l2, -1.0, 1.0);

        double linearVelocity = throttle * _maxLinearVelocity;
        double angularVelocity = left_x * _maxAngularVelocity;

        if (_invertDirection) {
            angularVelocity *= -1.0;
        }

        if (_verbose) {
            RCLCPP_INFO(this->get_logger(),
                "Robô %d - X: %.2f; R2: %.2f; L2: %.2f; v_lin: %.2f; v_ang: %.2f",
                id, left_x, r2, l2, linearVelocity, angularVelocity);
        }

        shared_interfaces::msg::Direction direction;
        direction.id = id;
        direction.linear_vel = linearVelocity;
        direction.angular_vel = angularVelocity;
        direction_list.directions.push_back(direction);

    }

    _directionPublisher->publish(direction_list);

}

}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_interpreter::DirectionNode>());
    rclcpp::shutdown();
    return 0;

}
