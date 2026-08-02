#ifndef RADIO_COMMUNICATION_H
#define RADIO_COMMUNICATION_H

#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/motor_velocities_list.hpp"
#include "RadioMessage.h"

#include <map>

class Radio_Communication : public rclcpp::Node
{
    public:

        Radio_Communication();
        ~Radio_Communication();

    private:

        void _motorVelocitiesCallback(
            const shared_interfaces::msg::MotorVelocitiesList::SharedPtr msg);

        // Envio periódico. Desacopla a taxa do rádio da taxa de publicação dos nós
        // de cima: guardamos o último comando de cada robô e transmitimos num ritmo
        // fixo, dividido entre os robôs, para não saturar a serial nem o nRF24.
        void _txTimerCallback();

        void _initSerial();

        uint8_t _calculateChecksum(const shared_interfaces::RadioMessage &radioMessage) const;
        bool _writeSerial(const void *data, size_t size);
        void _sendRadioMessage(int32_t id, double left, double right);

        int _serial_fd = -1;
        uint8_t _start_byte = 0x14;
        double _commandTimeout = 0.5;
        bool _verbose = false;

        struct Command {
            double left;
            double right;
            rclcpp::Time stamp;
        };

        std::map<int32_t, Command> _lastCommands;
        size_t _txCursor = 0;

        rclcpp::Subscription<shared_interfaces::msg::MotorVelocitiesList>::SharedPtr _motorVelocitiesSubscriber;
        rclcpp::TimerBase::SharedPtr _txTimer;
};

#endif
