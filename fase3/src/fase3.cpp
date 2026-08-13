#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Estados padrao, do stdstates -- servem a qualquer missao.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/yaw_sweep_state.hpp"
#include "stdstates/precision_landing_state.hpp"
#include "stdstates/return_home_state.hpp"
#include "stdstates/land_and_disarm_state.hpp"

// Desta missao.
#include "fase3/gesture_fase3.hpp"
#include "fase3/states/search_hand_state.hpp"
#include "fase3/states/gesture_control_state.hpp"

/**
 * @brief Maquina de estados da missao fase3.
 *
 * Ja vem com o ciclo minimo que toda missao tem: armar, decolar, pousar.
 * Para estender, veja os blocos marcados com ACRESCENTE.
 */
class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM(
        std::shared_ptr<Drone> drone,
        std::shared_ptr<GestureFase3> vision,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // O drone e o no de visao ficam na blackboard: todo estado os acessa
        // por aqui. Em 2025 a leitura de gestos vivia dentro do proprio Drone.
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<GestureFase3>>("vision", vision);

        // Parametros do ROS 2 (vindos do YAML) viram entradas da blackboard.
        for (const auto &[key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        const double home_x = std::get<double>(params.at("fictual_home_x"));
        const double home_y = std::get<double>(params.at("fictual_home_y"));
        const double home_z = std::get<double>(params.at("fictual_home_z"));
        this->blackboard_set<Eigen::Vector3d>(
            "home_position", Eigen::Vector3d(home_x, home_y, home_z));

        // ========================= ESTADOS =========================
        this->add_state("ARMING",  std::make_unique<ArmingState>());

        // DUAS decolagens. TakeoffState() reancora o referencial do mundo na
        // posicao atual (setHomePosition), o que e necessario na decolagem
        // INICIAL, depois de armar, com o EKF ja convergido. Na REDECOLAGEM
        // seria destrutivo: a origem do mundo pularia para onde o drone pousou,
        // e a posicao de casa passaria a apontar para um referencial extinto.
        // Ver stdstates v0.2.1 e o que aconteceu na fase 1.
        this->add_state("TAKEOFF",       std::make_unique<TakeoffState>(true));
        this->add_state("TAKEOFF AGAIN", std::make_unique<TakeoffState>(false));

        this->add_state("SEARCH HAND",       std::make_unique<SearchHandState>());
        this->add_state("GESTURE CONTROL",   std::make_unique<GestureControlState>());
        this->add_state("PRECISION LANDING", std::make_unique<PrecisionLandingState>());
        this->add_state("RETURN HOME",       std::make_unique<ReturnHomeState>());
        this->add_state("LAND AND DISARM",   std::make_unique<LandAndDisarmState>());

        // ======================= TRANSICOES ========================
        // Cada linha e: {outcome retornado pelo estado, proximo estado}.
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "SEARCH HAND"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH HAND", {
            {"HAND FOUND", "GESTURE CONTROL"},
            {"ERROR", "ERROR"}
        });

        // Perder a mao volta para a BUSCA, e nao aborta. O operador pode ter
        // saido do quadro porque o drone girou demais, e girar de volta e
        // exatamente o que resolve. Em 2025 nao havia esta saida: o estado
        // ficava controlando com a ultima posicao vista.
        this->add_transitions("GESTURE CONTROL", {
            {"LAND NOW",  "PRECISION LANDING"},
            {"GO HOME",   "RETURN HOME"},
            {"LOST HAND", "SEARCH HAND"},
            {"ERROR", "ERROR"}
        });

        // Pousou por gesto: redecola e volta a obedecer. E o ciclo que permite
        // ao operador usar o pouso como pausa.
        this->add_transitions("PRECISION LANDING", {
            {"LANDED", "TAKEOFF AGAIN"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF AGAIN", {
            {"TAKEOFF COMPLETED", "GESTURE CONTROL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RETURN HOME", {
            {"AT HOME", "LAND AND DISARM"},
            {"ERROR", "ERROR"}
        });

        // TIMEOUT tambem termina a missao, mas por um caminho proprio: o log
        // diz que o drone pode ter ficado armado, o que ninguem quer descobrir
        // caminhando ate ele.
        this->add_transitions("LAND AND DISARM", {
            {"DISARMED", "FINISHED"},
            {"TIMEOUT",  "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

/**
 * @brief No ROS 2 que executa a FSM da missao fase3.
 *
 * Declara os parametros (sobrescritos pelo YAML no launch), monta a FSM e a
 * executa a 20 Hz.
 */
class Fase3Node : public rclcpp::Node {
public:
    Fase3Node(std::shared_ptr<Drone> drone, std::shared_ptr<GestureFase3> vision)
        : rclcpp::Node("fase3_node"), drone_(drone), vision_(vision) {

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e trocar
        // de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Casa ficticia (FRD: z negativo e para cima)
            {"fictual_home_x",           0.0},
            {"fictual_home_y",           0.0},
            {"fictual_home_z",          -0.6},

            // Decolagem
            {"takeoff_height",          -1.8},
            {"max_vertical_velocity",    1.0},
            {"position_tolerance",       0.15},

            // Pouso. A altura de PARTIDA nao esta aqui: o PrecisionLandingState
            // a mede ao entrar, porque o drone pousa de onde o gesto o deixou.
            {"landing_velocity_max",     0.5},
            {"landing_velocity_min",     0.2},
            {"max_base_height",         -0.2},

            // Retorno e desarme
            {"return_home_timeout",     30.0},
            {"disarm_grace",             3.0},
            {"disarm_timeout",          20.0},

            // Busca da mao
            {"yaw_speed",                0.35},
            {"search_yaw_range",         1.0472},   // 60 graus
            {"search_confirm_cycles",    5.0},

            // Controle por gestos
            {"control_speed",            0.4},
            {"max_horizontal_velocity",  1.0},
            {"detection_timeout",        3.0},
            {"command_confirm_cycles",  10.0},

            // PID de guinada (mantem a mao centrada em x)
            {"yaw_pid_kp",               0.6},
            {"yaw_pid_ki",               0.0},
            {"yaw_pid_kd",               0.06},

            // PID de altitude (mantem a mao centrada em y)
            {"climb_pid_kp",             0.9},
            {"climb_pid_ki",             0.0},
            {"climb_pid_kd",             0.09},
        };

        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase3FSM>(drone_, vision_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&Fase3Node::executeFSM, this));

        // Trajetoria para o RViz2 (convertida de NED para ENU).
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // ACRESCENTE: assinaturas dos nos de visao desta missao. O padrao e o
        // callback escrever na blackboard e os estados apenas lerem de la.
        //
        // cv_sub_ = this->create_subscription<custom_msgs::msg::MinhaDeteccao>(
        //     "minha_deteccao", 10,
        //     std::bind(&Fase3Node::cv_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FSM da missao fase3 iniciada");
    }

private:
    void executeFSM() {
        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        // NED -> ENU para visualizar no RViz2.
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp       = this->now();
        ps.header.frame_id    = "map";
        ps.pose.position.x    =  static_cast<float>(pos.y());   // East  = NED y
        ps.pose.position.y    =  static_cast<float>(pos.x());   // North = NED x
        ps.pose.position.z    = -static_cast<float>(pos.z());   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        // Log de estado e posicao a cada 2 s (40 ticks a 20 Hz).
        if (log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] pos=(%.2f, %.2f, %.2f) yaw=%.2f rad",
                        fsm_->get_current_state().c_str(),
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        static_cast<float>(pos.z()),
                        static_cast<float>(orient[2]));
        }

        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "FSM terminou com: %s",
                        fsm_->get_fsm_outcome().c_str());
            rclcpp::shutdown();
        }
    }

    /// Declara cada parametro com seu padrao e le o valor efetivo.
    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>> &defaults) {

        std::map<std::string, std::variant<double, std::string>> result;
        for (const auto &[name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }
        return result;
    }

    std::shared_ptr<Drone> drone_;
    std::shared_ptr<GestureFase3> vision_;
    std::unique_ptr<Fase3FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
    int log_counter_ = 0;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // O Drone JA sobe o proprio executor e a propria thread de spin no
    // construtor (veja drone_lib/src/Drone.cpp). Adiciona-lo a um executor
    // aqui lanca em tempo de execucao:
    //
    //     terminate called after throwing an instance of 'std::runtime_error'
    //       what():  Node '/Drone' has already been added to an executor.
    //
    // Por isso so o no da missao entra no executor deste main.
    auto drone        = std::make_shared<Drone>();
    auto vision       = std::make_shared<GestureFase3>();
    auto mission_node = std::make_shared<Fase3Node>(drone, vision);

    rclcpp::executors::MultiThreadedExecutor executor;
    // O no de visao E um rclcpp::Node comum e PRECISA entrar no executor. Sem
    // isso o callback de gestos nunca roda e a missao fica cega sem dizer por
    // que -- o drone giraria para sempre procurando uma mao.
    executor.add_node(vision);
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
