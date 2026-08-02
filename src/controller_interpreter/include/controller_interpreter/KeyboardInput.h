#ifndef KEYBOARD_INPUT_H
#define KEYBOARD_INPUT_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <unordered_map>
#include <SDL2/SDL.h>

namespace controller_interpreter {

class KeyboardInput : public rclcpp::Node {
public:

    KeyboardInput();
    ~KeyboardInput();

private:

    SDL_Window* _window = nullptr;
    bool _capturing = false;
    std::unordered_map<SDL_Keycode, bool> _keys;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr _inputPublisher;

    void _keyboardCallback();
    void _renderStatus();
    
};

}

#endif