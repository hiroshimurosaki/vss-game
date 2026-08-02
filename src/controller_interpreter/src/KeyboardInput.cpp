#include "controller_interpreter/KeyboardInput.h"

namespace controller_interpreter {

KeyboardInput::KeyboardInput() : Node("keyboard_input") {

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);

    _window = SDL_CreateWindow(
        "Controle do robô via teclado",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        600, 80,
        SDL_WINDOW_SHOWN
    );

    _renderStatus();

    _inputPublisher = 
        this->create_publisher<sensor_msgs::msg::Joy>("/joy_0", 10);
    
    _timer = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { _keyboardCallback(); }
    );

    RCLCPP_INFO(this->get_logger(),
            "KeyboardInput inicializado. "
            "Acesse a janela 'Controle do robô via teclado' para iniciar a captura do teclado.");

}

KeyboardInput::~KeyboardInput() {

    if (_window) 
        SDL_DestroyWindow(_window);
    SDL_Quit();

}

void KeyboardInput::_renderStatus() {

    SDL_Surface* surface = SDL_GetWindowSurface(_window);

    SDL_Color bg = _capturing 
        ? SDL_Color{34, 139, 34, 255}
        : SDL_Color{180, 30, 30, 255};
    
    SDL_FillRect(surface, nullptr,
        SDL_MapRGB(surface->format, bg.r, bg.g, bg.b));

    SDL_UpdateWindowSurface(_window);

    std::string title = _capturing
        ? "Controle do robô via teclado | Capturando | Pressione ESC para pausar"
        : "Controle do robô via teclado | Pausado | Pressione ESC para retomar";

    SDL_SetWindowTitle(_window, title.c_str());

}

void KeyboardInput::_keyboardCallback() {

    SDL_Event event;

    while (SDL_PollEvent(&event)) {

        switch(event.type) {

            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
                    if (_capturing)
                        RCLCPP_INFO(this->get_logger(),
                            "Captura do teclado pausada.");
                    _capturing = false;
                    _keys.clear();
                    _renderStatus();
                }
            break;

            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {

                    _capturing = !_capturing;
                    if (!_capturing) {
                        _keys.clear();
                        RCLCPP_INFO(this->get_logger(),
                            "Captura do teclado pausada.");
                    } 
                    _renderStatus();

                } else if (_capturing)
                    _keys[event.key.keysym.sym] = true;
            break;

            case SDL_KEYUP:
                _keys[event.key.keysym.sym] = false;
            break;

        }

    }

    auto msg = sensor_msgs::msg::Joy();
    msg.axes.resize(6, 0.0f);
    msg.buttons.resize(3, 0);

    // Emulamos a convenção "signed" dos gatilhos: solto = +1.0, apertado = -1.0.
    // O neutro precisa ser explícito — deixar em 0.0 seria lido como meio
    // acelerado e o robô sairia andando sozinho.
    msg.axes[4] = 1.0f;
    msg.axes[5] = 1.0f;

    if(_capturing) {

        if(_keys[SDLK_w]) msg.axes[5] = -1.0f;
        if(_keys[SDLK_s]) msg.axes[4] = -1.0f;
        if(_keys[SDLK_d]) msg.axes[0] = 1.0f;
        if(_keys[SDLK_a]) msg.axes[0] = -1.0f;
        if(_keys[SDLK_q]) msg.buttons[2] = 1;
        if(_keys[SDLK_e]) msg.buttons[0] = 1;
        if(_keys[SDLK_b]) msg.buttons[1] = 1;

        RCLCPP_DEBUG(this->get_logger(),
            "Teclas: W = %s; A = %s; S = %s; D = %s; Q = %s; E = %s; B = %s",
            msg.axes[5] == -1.0f ? "true" : "false",
            msg.axes[0] == -1.0f ? "true" : "false",
            msg.axes[4] == -1.0f ? "true" : "false",
            msg.axes[0] == 1.0f ? "true" : "false",
            msg.buttons[2] == 1 ? "true" : "false",
            msg.buttons[0] == 1 ? "true" : "false",
            msg.buttons[1] == 1 ? "true" : "false");

    }

    _inputPublisher->publish(msg);

}

}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_interpreter::KeyboardInput>());
    rclcpp::shutdown();
    return 0;

}
