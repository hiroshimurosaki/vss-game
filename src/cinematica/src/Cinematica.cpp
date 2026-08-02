#include "cinematica/Cinematica.h"

#include <algorithm>
#include <cmath>

Cinematica::Cinematica() : Node("kinematics_node") {

  _directionSubscriber = this->create_subscription<shared_interfaces::msg::DirectionList>(
      "/direction", 10,
      std::bind(&Cinematica::directionCallback, this, std::placeholders::_1));

  _actionsSubscriber = this->create_subscription<shared_interfaces::msg::ActionsList>(
      "/actions", 10,
      std::bind(&Cinematica::actionCallback, this, std::placeholders::_1));

  _motorVelocitiesPublisher =
      this->create_publisher<shared_interfaces::msg::MotorVelocitiesList>("/motorVelocities", 10);

  this->declare_parameter("axle_length", 0.0625);

  // Velocidade de roda que corresponde a PWM máximo. É o fator que converte m/s
  // para a faixa [-1, 1] que o firmware espera. Medir na prática: solte o robô a
  // PWM 100% e cronometre 1 metro.
  this->declare_parameter("wheel_speed_max", 0.75);

  this->declare_parameter("spin_speed", 5.0);
  this->declare_parameter("verbose", false);

  _axleLength = this->get_parameter("axle_length").as_double();
  _wheelSpeedMax = this->get_parameter("wheel_speed_max").as_double();
  _spinSpeed = this->get_parameter("spin_speed").as_double();
  _verbose = this->get_parameter("verbose").as_bool();

  if (_wheelSpeedMax <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
        "wheel_speed_max inválido (%.3f). Usando 0.75.", _wheelSpeedMax);
    _wheelSpeedMax = 0.75;
  }

  RCLCPP_INFO(this->get_logger(),
      "Cinematica inicializada | axle_length: %.4f m | wheel_speed_max: %.3f m/s",
      _axleLength, _wheelSpeedMax);
}

void Cinematica::directionCallback(const shared_interfaces::msg::DirectionList::SharedPtr msg){

  auto velocitiesList = shared_interfaces::msg::MotorVelocitiesList();

  for (const auto& direction : msg->directions) {

    double linear_vel = direction.linear_vel;
    double angular_vel = direction.angular_vel;

    _PreProcessActions(direction.id, linear_vel, angular_vel);

    velocitiesList.velocities.push_back(
      _inverseKinematics(direction.id, linear_vel, angular_vel));

    if (_verbose) {
      RCLCPP_INFO(this->get_logger(),
        "Direção recebida -> id: %d, linear: %.3f, angular: %.3f",
        direction.id, direction.linear_vel, direction.angular_vel);
    }
  }

  _motorVelocitiesPublisher->publish(velocitiesList);

}

void Cinematica::actionCallback(const shared_interfaces::msg::ActionsList::SharedPtr msg){

  for (const auto& action : msg->actions) {

    _lastaction[action.id] = action;

    if (_verbose) {
      RCLCPP_INFO(this->get_logger(),
        "id: %d; Ação recebida -> Spin: %d, Speed Mult: %.2f",
        action.id, action.spin, action.speed_mult);
    }
  }

}

void Cinematica::_PreProcessActions(int32_t id, double &linear, double &angular){

  auto robotAction = _lastaction.find(id);
  if (robotAction == _lastaction.end()) return;

  const auto& action = robotAction->second;

  if (action.spin == 1) {
    linear = 0.0;
    angular = _spinSpeed;
  } else if (action.spin == -1) {
    linear = 0.0;
    angular = -_spinSpeed;
  } else {
    linear *= action.speed_mult;
    angular *= action.speed_mult;
  }

}

shared_interfaces::msg::MotorVelocities Cinematica::_inverseKinematics(
    int32_t id, double linear, double angular){

  shared_interfaces::msg::MotorVelocities vel;
  vel.id = id;

  // Cinemática diferencial, em m/s de roda.
  double right = linear + (angular * _axleLength / 2.0); // Vr = V + (ω * l) / 2
  double left  = linear - (angular * _axleLength / 2.0); // Vl = V - (ω * l) / 2

  // Converte para a faixa normalizada que o firmware espera.
  right /= _wheelSpeedMax;
  left  /= _wheelSpeedMax;

  // Se estourar, escala as DUAS rodas pelo mesmo fator. Saturar cada roda em
  // separado (o que o firmware faria com constrain) muda a razão entre elas e o
  // robô curva errado justamente quando o jogador pede o máximo.
  const double peak = std::max(std::abs(left), std::abs(right));

  if (peak > 1.0) {
    left  /= peak;
    right /= peak;
  }

  vel.left = left;
  vel.right = right;

  return vel;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Cinematica>());
  rclcpp::shutdown();
  return 0;
}
