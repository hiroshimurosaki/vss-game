#include "controller_interpreter/SpecialControls.h"

namespace controller_interpreter {

SpecialControls::SpecialControls()
: Node("special_controls")
{

    _actionsPublisher = this->create_publisher<shared_interfaces::msg::ActionsList>(
        "/actions",
        10
    );

    _joyListSubscriber = 
        this->create_subscription<shared_interfaces::msg::JoyList> ("/joy_list", 10,
            [this](const shared_interfaces::msg::JoyList::SharedPtr msg) {
                _joyListCallback(msg);
            });

    RCLCPP_INFO(this->get_logger(), "Special Controls iniciado.");
    
}

void SpecialControls::_joyListCallback(const shared_interfaces::msg::JoyList::SharedPtr msg)
{

    auto actions_list = shared_interfaces::msg::ActionsList();

    for (const auto& robot_controller : msg->joys) {

        const auto& buttons = robot_controller.joy.buttons;
        const int32_t id = robot_controller.id;

        shared_interfaces::msg::Actions actions;
        actions.id = id;

        //logica botao X (girar/spin); se valer 1 esta apertado;
        if (buttons.size() > 0 && buttons[0] == 1) {
            actions.spin = 1;
        } else if (buttons.size() > 2 && buttons[2] == 1) {
            actions.spin = -1;
        } else {
            actions.spin = 0;
        }

        //logica botao O (boost velocidade); se apertar, vale 1,5. se soltar, vale apenas 1.0;
        if (!(actions.spin) && (buttons.size() > 1 && buttons[1] == 1)) {
            actions.speed_mult = 1.5f; 
        } else {
            actions.speed_mult = 1.0f;
        }

        actions_list.actions.push_back(actions);
    
        RCLCPP_INFO(
            this->get_logger(),
            "Robô %d - Joy recebido -> buttons: %ld",
            id, buttons.size());

    }

    _actionsPublisher->publish(actions_list);

}

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<controller_interpreter::SpecialControls>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}