#include "robot_communication/Radio_Communication.h"

#include <algorithm>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

Radio_Communication::Radio_Communication()
: Node("radio_communication")
{
    this->declare_parameter("device_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("start_byte", 0x14);

    // Frequência total de pacotes na serial. É dividida entre os robôs ativos:
    // com 2 robôs e 60 Hz, cada um recebe 30 atualizações por segundo.
    this->declare_parameter("tx_rate_hz", 60.0);

    // Se um robô ficar este tempo sem comando novo, paramos de retransmitir o
    // último valor e mandamos zero. O firmware tem o seu próprio timeout de 1 s,
    // mas parar aqui é mais rápido e evita um robô "fugindo" se um nó morrer.
    this->declare_parameter("command_timeout", 0.5);

    this->declare_parameter("verbose", false);

    _start_byte = static_cast<uint8_t>(this->get_parameter("start_byte").as_int());
    _commandTimeout = this->get_parameter("command_timeout").as_double();
    _verbose = this->get_parameter("verbose").as_bool();

    _initSerial();

    _motorVelocitiesSubscriber =
        this->create_subscription<shared_interfaces::msg::MotorVelocitiesList>(
            "/motorVelocities",
            10,
            std::bind(
                &Radio_Communication::_motorVelocitiesCallback,
                this,
                std::placeholders::_1));

    const double rate = std::max(1.0, this->get_parameter("tx_rate_hz").as_double());
    const auto period = std::chrono::duration<double>(1.0 / rate);

    _txTimer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { _txTimerCallback(); });

    RCLCPP_INFO(this->get_logger(),
        "Radio_Communication iniciado | start_byte: 0x%02X | tx_rate: %.1f Hz | timeout: %.2f s",
        _start_byte, rate, _commandTimeout);
}

Radio_Communication::~Radio_Communication()
{
    // Última cortesia: manda todo mundo parar antes de fechar a porta.
    if (_serial_fd >= 0) {
        for (const auto &entry : _lastCommands) {
            _sendRadioMessage(entry.first, 0.0, 0.0);
        }
        close(_serial_fd);
        _serial_fd = -1;
    }
}

void Radio_Communication::_initSerial()
{
    std::string device = this->get_parameter("device_name").as_string();
    int baud = this->get_parameter("baud_rate").as_int();

    // Sem O_NONBLOCK: queremos que a escrita bloqueie até a serial aceitar os bytes.
    // Com escrita não-bloqueante, write() retorna curto quando o buffer enche e
    // pacotes saem pela metade, dessincronizando o parser do Arduino.
    _serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);

    if (_serial_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao abrir device serial: %s (Erro: %s)",
                     device.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    if (tcgetattr(_serial_fd, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Erro ao obter atributos da serial.");
        close(_serial_fd);
        _serial_fd = -1;
        return;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed = B115200;
    if (baud == 9600) speed = B9600;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(_serial_fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao configurar atributos serial.");
        close(_serial_fd);
        _serial_fd = -1;
        return;
    }

    // Abrir a porta reseta o Arduino (DTR). Ele leva ~2 s para subir; qualquer
    // byte enviado antes disso cai no bootloader e é descartado.
    tcflush(_serial_fd, TCIOFLUSH);

    RCLCPP_INFO(this->get_logger(), "Conexão serial estabelecida em %s a %d baud.",
                device.c_str(), baud);
}

void Radio_Communication::_motorVelocitiesCallback(
    const shared_interfaces::msg::MotorVelocitiesList::SharedPtr msg)
{
    const rclcpp::Time now = this->now();

    for (const auto &velocity : msg->velocities) {
        _lastCommands[velocity.id] = Command{velocity.left, velocity.right, now};

        if (_verbose) {
            RCLCPP_INFO(this->get_logger(),
                "Velocidades recebidas -> id: %d, left: %.3f, right: %.3f",
                velocity.id, velocity.left, velocity.right);
        }
    }
}

void Radio_Communication::_txTimerCallback()
{
    if (_lastCommands.empty()) return;

    // Round-robin: um robô por tick. Assim o intervalo entre pacotes na serial é
    // constante, independente de quantos robôs estão em campo.
    auto it = _lastCommands.begin();
    std::advance(it, _txCursor % _lastCommands.size());
    _txCursor = (_txCursor + 1) % _lastCommands.size();

    const int32_t id = it->first;
    Command &cmd = it->second;

    const double age = (this->now() - cmd.stamp).seconds();

    if (age > _commandTimeout && (cmd.left != 0.0 || cmd.right != 0.0)) {
        RCLCPP_WARN(this->get_logger(),
            "Robô %d sem comando há %.2f s. Zerando velocidades.", id, age);
        cmd.left = 0.0;
        cmd.right = 0.0;
    }

    _sendRadioMessage(id, cmd.left, cmd.right);
}

void Radio_Communication::_sendRadioMessage(int32_t id, double left, double right)
{
    shared_interfaces::RadioMessage radioMessage;

    radioMessage.startByte = _start_byte;
    radioMessage.velMotor1 = static_cast<float>(std::clamp(left, -1.0, 1.0));
    radioMessage.velMotor2 = static_cast<float>(std::clamp(right, -1.0, 1.0));
    radioMessage.robotId = id;
    radioMessage.checksum = 0;
    radioMessage.checksum = _calculateChecksum(radioMessage);

    if (!_writeSerial(&radioMessage, sizeof(radioMessage))) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Falha ao enviar RadioMessage pela serial para id = %d.", id);
        return;
    }

    if (_verbose) {
        RCLCPP_INFO(this->get_logger(),
            "TX -> id: %d, vel1: %.3f, vel2: %.3f, checksum: 0x%02X",
            radioMessage.robotId, radioMessage.velMotor1, radioMessage.velMotor2,
            radioMessage.checksum);
    }
}

uint8_t Radio_Communication::_calculateChecksum(
    const shared_interfaces::RadioMessage &radioMessage) const
{
    // XOR de velMotor1 até robotId. Pula o startByte (que serve de sincronismo) e
    // o próprio checksum. Precisa ser idêntico ao cálculo do firmware.
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&radioMessage);

    const size_t startOffset = offsetof(shared_interfaces::RadioMessage, velMotor1);
    const size_t checksumOffset = offsetof(shared_interfaces::RadioMessage, checksum);

    uint8_t checksum = 0;
    for (size_t i = startOffset; i < checksumOffset; i++) {
        checksum ^= bytes[i];
    }

    return checksum;
}

bool Radio_Communication::_writeSerial(const void *data, size_t size)
{
    if (_serial_fd < 0) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Serial não inicializada.");
        return false;
    }

    const uint8_t *buffer = static_cast<const uint8_t *>(data);
    size_t totalWritten = 0;

    while (totalWritten < size) {
        ssize_t bytesWritten = write(_serial_fd, buffer + totalWritten, size - totalWritten);

        if (bytesWritten < 0) {
            if (errno == EINTR) continue;
            RCLCPP_ERROR(this->get_logger(), "Erro ao escrever na serial: %s", strerror(errno));
            return false;
        }

        totalWritten += static_cast<size_t>(bytesWritten);
    }

    return true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Radio_Communication>());
    rclcpp::shutdown();
    return 0;
}
