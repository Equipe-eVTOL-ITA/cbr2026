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

// Estados padrao, do stdstates
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

class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM(
        std::shared_ptr<Drone> drone,
        std::shared_ptr<GestureFase3> vision,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

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
        this->add_state("TAKEOFF",       std::make_unique<TakeoffState>(true));
        this->add_state("TAKEOFF AGAIN", std::make_unique<TakeoffState>(false));
        this->add_state("SEARCH HAND",       std::make_unique<SearchHandState>());
        this->add_state("GESTURE CONTROL",   std::make_unique<GestureControlState>());
        this->add_state("PRECISION LANDING", std::make_unique<PrecisionLandingState>());
        this->add_state("RETURN HOME",       std::make_unique<ReturnHomeState>());
        this->add_state("LAND AND DISARM",   std::make_unique<LandAndDisarmState>());

        // ======================= TRANSICOES ========================
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

        this->add_transitions("GESTURE CONTROL", {
            {"LAND NOW",  "PRECISION LANDING"},
            {"GO HOME",   "RETURN HOME"},
            {"LOST HAND", "SEARCH HAND"},
            {"ERROR", "ERROR"}
        });

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

        this->add_transitions("LAND AND DISARM", {
            {"DISARMED", "FINISHED"},
            {"TIMEOUT",  "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

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

            // Rastreio por guinada. 0 = DESLIGADO, e o padrao e deliberado: a
            // camera vai presa ao drone, entao girar move o proprio sensor, e o
            // drone acaba orbitando o operador em vez de andar reto.
            {"yaw_tracking",             0.0},

            // Rastreio em altura. 0 = DESLIGADO. A altitude vira setpoint de
            // POSICAO (setMixedSetpoint), mantida onde a decolagem deixou; um
            // comando de translacao nao mexe nela.
            {"climb_tracking",           0.0},

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

    auto drone        = std::make_shared<Drone>();
    auto vision       = std::make_shared<GestureFase3>();
    auto mission_node = std::make_shared<Fase3Node>(drone, vision);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(vision);
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
