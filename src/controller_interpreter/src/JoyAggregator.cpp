#include "controller_interpreter/JoyAggregator.h"

#include <algorithm>

namespace controller_interpreter {

JoyAggregator::JoyAggregator() : Node("joy_aggregator") {

    this->declare_parameter("num_robots", 1);
    _numRobots = this->get_parameter("num_robots").as_int();
    if (_numRobots < 1) {
        RCLCPP_WARN(this->get_logger(),
            "Número de robôs inválido (%d). Usando valor mínimo: 1.", _numRobots);
        _numRobots = 1;
    }

    // Segundos de silêncio até considerar um controle perdido. Sem isto, o
    // último Joy recebido ficaria retido para sempre e seria republicado a
    // 50 Hz: se o link cai com o acelerador no fundo, o robô sai desgovernado
    // e nada no log aponta para a causa. 0.3 s dá folga sobre o
    // autorepeat_rate de 20 Hz do game_controller_node (um pacote a cada
    // 0,05 s). 0 ou menos desliga o watchdog.
    this->declare_parameter("input_timeout", 0.3);
    _inputTimeout = this->get_parameter("input_timeout").as_double();

    _joyListPublisher = 
        this->create_publisher<shared_interfaces::msg::JoyList> ("/joy_list", 30);

    // ================================================================================================================
    // Cria um subscriber para cada robô no tópico /joy_n, onde n ∈ {0, ..., _numRobots-1} equivale ao ID do robô
    // Para cada robô faz-se necessário rodar o game_controller_node redirecionando sua saída para o tópico em questão:
    //    ros2 run joy game_controller_node --ros-args -p device_id:=D -r /joy:=/joy_n
    // Onde device_id:=D indica qual dispositivo será lido (/dev/input/jsD)
    // Para identificar o device_id, rode:
    //    cat /proc/bus/input/devices | grep -A 4 "js"
    // ou ros2 run joy game_controller_node
    // e observe o log para confirmar qual dispositivo foi aberto.

    // Caso o teclado também seja utilizado como entrada, rode:
    //    ros2 run controller_interpreter keyboard_input
    // O robô controlado pelo teclado SEMPRE será o de ID 0.
    // Ou seja, deve-se rodar o game_controller_node para os demais robôs redirecionando a saída para os tópicos /joy_1 e subsequentes

    for (int32_t id = 0; id < _numRobots; ++id) {

        std::string topic = "/joy_" + std::to_string(id);
        auto sub = this->create_subscription<sensor_msgs::msg::Joy> (topic, 10,
            [this, id](const sensor_msgs::msg::Joy::SharedPtr msg) {
                _joyCallback(msg, id);
            });
        _joySubscribers.push_back(sub);

    }
    // ================================================================================================================

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { _timerCallback(); }
    );

    RCLCPP_INFO(this->get_logger(),
            "JoyAggregator inicializado com número de robôs: %d", 
            _numRobots);

}
    
void JoyAggregator::_joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg, int32_t robotId) {

    _lastJoys[robotId] = *msg;
    _lastSeen[robotId] = this->now();

}

void JoyAggregator::_timerCallback() {

    if(_lastJoys.empty()) return;

    const rclcpp::Time now = this->now();

    auto joy_list = shared_interfaces::msg::JoyList();
    for (const auto& pair : _lastJoys) {

        const int32_t id = pair.first;

        shared_interfaces::msg::RobotController robot_controller;
        robot_controller.id = id;
        robot_controller.joy = pair.second;

        if (_inputTimeout > 0.0) {

            // .at() e não [] de propósito: o operator[] inseriria um
            // rclcpp::Time default, de clock type diferente do this->now(),
            // e a subtração lançaria. _joyCallback preenche os dois mapas
            // juntos, então a chave existe sempre que _lastJoys tem o id.
            const double silence = (now - _lastSeen.at(id)).seconds();
            const bool stale = silence > _inputTimeout;

            if (stale) {
                // O neutro é o array zerado — na convenção "unit" isso é
                // gatilho solto e volante centrado. Zeramos no lugar em vez de
                // omitir o robô da lista porque o DirectionNode e o
                // SpecialControls só publicam para quem aparece: sumir daqui
                // deixaria o último /motorVelocities valendo, que é
                // exatamente o que queremos evitar. Os tamanhos ficam como
                // estavam, porque os dois indexam por posição.
                std::fill(robot_controller.joy.axes.begin(),
                          robot_controller.joy.axes.end(), 0.0f);
                std::fill(robot_controller.joy.buttons.begin(),
                          robot_controller.joy.buttons.end(), 0);
            }

            if (stale != _stale[id]) {
                _stale[id] = stale;
                if (stale)
                    RCLCPP_WARN(this->get_logger(),
                        "Robô %d: sem Joy há %.2f s. Controle zerado.", id, silence);
                else
                    RCLCPP_INFO(this->get_logger(),
                        "Robô %d: controle de volta.", id);
            }

        }

        joy_list.joys.push_back(robot_controller);

    }
    _joyListPublisher->publish(joy_list);

}

}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_interpreter::JoyAggregator>());
    rclcpp::shutdown();
    return 0;

}
