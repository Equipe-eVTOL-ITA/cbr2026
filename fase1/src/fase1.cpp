// Missão da fase 1 da CBR 2026: varrer a arena, pousar em cada base, voltar.
//
// Este arquivo é fino de propósito. A geometria de visão está na
// `vision_geometry`, os estados de voo estão no `stdstates`, e a detecção está
// no `base_detector` — todos com teste e CI próprios. O que sobra aqui é o que
// só esta fase sabe: a grade, o registro de bases, e como os estados se ligam.
//
// Em 2025 o equivalente a isto tinha sete estados escritos do zero, 493 linhas
// de visão embutidas e duas bibliotecas forkadas. A prova é a mesma.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Estados padrão, do stdstates.
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/precision_align_state.hpp"
#include "stdstates/precision_landing_state.hpp"
#include "stdstates/return_home_state.hpp"

// Desta missão.
#include "fase1/grid.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/states/search_base_state.hpp"
#include "fase1/states/register_base_state.hpp"

class Fase1FSM : public fsm::FSM
{
public:
  Fase1FSM(
    std::shared_ptr<Drone> drone,
    std::shared_ptr<VisionFase1> vision,
    const std::map<std::string, std::variant<double, std::string>> & params)
  : fsm::FSM({"ERROR", "FINISHED"})
  {
    this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
    this->blackboard_set<std::shared_ptr<VisionFase1>>("vision", vision);

    // Os parâmetros do ROS entram como FLOAT, nunca double.
    //
    // Isto não é detalhe de estilo: `fsm::Blackboard::get<T>` faz `(Value<T>*)`
    // — um cast C sem checagem. Gravar `double` e ler `float` compila, roda e
    // devolve lixo. Todo o stdstates lê `float`, então a missão grava `float`.
    for (const auto & [key, value] : params) {
      if (std::holds_alternative<double>(value)) {
        this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
      } else if (std::holds_alternative<std::string>(value)) {
        this->blackboard_set<std::string>(key, std::get<std::string>(value));
      }
    }

    const double home_x = std::get<double>(params.at("fictual_home_x"));
    const double home_y = std::get<double>(params.at("fictual_home_y"));
    const double home_z = std::get<double>(params.at("fictual_home_z"));

    this->blackboard_set<Eigen::Vector3d>("home_position", Eigen::Vector3d(home_x, home_y, home_z));

    // ---- Grade de varredura -------------------------------------------
    auto waypoints = fase1::makeSerpentineGrid(
      home_x, home_y,
      std::get<double>(params.at("grid_y_length")),
      std::get<double>(params.at("grid_step_x")),
      static_cast<int>(std::get<double>(params.at("grid_num_steps"))),
      std::get<double>(params.at("takeoff_height")));
    this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);

    // ---- Registro de bases --------------------------------------------
    // "Casa" entra como primeira base para que o drone não trate o próprio
    // ponto de decolagem como alvo a pousar. O RegisterBaseState desconta esse
    // elemento ao contar quantas faltam.
    std::vector<Eigen::Vector3d> bases{Eigen::Vector3d(home_x, home_y, home_z)};
    this->blackboard_set<std::vector<Eigen::Vector3d>>("bases", bases);
    this->blackboard_set<bool>("finished_bases", false);

    // ---- Estados -------------------------------------------------------
    this->add_state("ARMING", std::make_unique<ArmingState>());
    this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
    this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
    this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
    this->add_state("PRECISION LANDING", std::make_unique<PrecisionLandingState>());
    this->add_state("REGISTER BASE", std::make_unique<RegisterBaseState>());
    this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

    // ---- Transições ----------------------------------------------------
    //
    //   ARMING → TAKEOFF → SEARCH BASE ⇄ PRECISION ALIGN → PRECISION LANDING
    //                           ↑                                  ↓
    //                       TAKEOFF ←────────────────────── REGISTER BASE
    //                           ↓ (todas as bases feitas)
    //                      RETURN HOME → FINISHED
    //
    // Note que não há estado GO TO BASE. O PrecisionAlignState já leva o drone
    // até o alvo por PID; um estado separado só para se aproximar seria uma
    // etapa a mais para o alvo sair do campo de visão.
    this->add_transitions("ARMING", {
      {"ARMED", "TAKEOFF"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("TAKEOFF", {
      {"TAKEOFF COMPLETED", "SEARCH BASE"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("SEARCH BASE", {
      {"BASE FOUND", "PRECISION ALIGN"},
      {"SEARCH ENDED", "RETURN HOME"},
      {"ERROR", "ERROR"}
    });

    // Perder o alvo volta para a busca em vez de abortar: o drone retoma a
    // grade de onde parou, porque o progresso mora em `is_visited`.
    this->add_transitions("PRECISION ALIGN", {
      {"PRECISELY ALIGNED", "PRECISION LANDING"},
      {"LOST TARGET", "SEARCH BASE"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("PRECISION LANDING", {
      {"LANDED", "REGISTER BASE"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("REGISTER BASE", {
      {"REGISTERED", "TAKEOFF"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("RETURN HOME", {
      {"AT HOME", "FINISHED"},
      {"ERROR", "ERROR"}
    });

    this->set_initial_state("ARMING");
  }
};

class Fase1Node : public rclcpp::Node
{
public:
  Fase1Node(std::shared_ptr<Drone> drone, std::shared_ptr<VisionFase1> vision)
  : rclcpp::Node("fase1_node"), drone_(drone), vision_(vision)
  {
    std::map<std::string, std::variant<double, std::string>> default_params = {
      // Casa fictícia (FRD; z negativo é para cima)
      {"fictual_home_x", 0.0},
      {"fictual_home_y", 0.0},
      {"fictual_home_z", -0.6},

      // Prova
      {"num_bases", 6.0},
      {"known_base_radius", 1.5},

      // Grade de varredura
      {"grid_y_length", -6.0},
      {"grid_step_x", 2.0},
      {"grid_num_steps", 3.0},

      // Altitudes (FRD, negativas)
      {"takeoff_height", -2.5},
      {"align_height", -2.0},
      {"max_base_height", -1.5},
      {"mean_base_height", -0.75},

      // Velocidades
      {"max_vertical_velocity", 1.2},
      {"max_horizontal_velocity", 1.0},
      {"landing_velocity_max", 0.5},
      {"landing_velocity_min", 0.2},
      {"align_descent_velocity", 0.15},

      // Tolerâncias e tempos
      {"position_tolerance", 0.15},
      {"align_tolerance", 0.10},
      {"detection_timeout", 5.0},

      // PID de alinhamento
      {"pid_pos_kp", 1.0},
      {"pid_pos_ki", 0.01},
      {"pid_pos_kd", 0.05},
    };

    auto params = declareAndGetParameters(default_params);
    fsm_ = std::make_unique<Fase1FSM>(drone_, vision_, params);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&Fase1Node::executeFSM, this));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
    trajectory_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "FSM da fase 1 iniciada");
  }

private:
  void executeFSM()
  {
    const auto pos = drone_->getLocalPosition();
    const auto orient = drone_->getOrientation();

    // A idade da detecção é atualizada AQUI, e não pelo estado, porque só o nó
    // conhece o relógio da visão. O PrecisionAlignState apenas a lê — foi o
    // que permitiu que ele ficasse genérico, sem saber o que é uma câmera.
    fsm_->blackboard_set<float>(
      "align_target_age", static_cast<float>(vision_->secondsSinceDetection()));

    // Enquanto há detecção, o alvo é reprojetado a cada ciclo: alinhar sobre
    // uma estimativa congelada no instante da descoberta acumularia o erro de
    // paralaxe conforme o drone desce.
    vision_geometry::BoundingBox box;
    if (vision_->closestBox(box)) {
      fsm_->blackboard_set<Eigen::Vector3d>("align_target", vision_->project(pos, orient, box));
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = "map";
    ps.pose.position.x = pos.y();    // East  = FRD y
    ps.pose.position.y = pos.x();    // North = FRD x
    ps.pose.position.z = -pos.z();   // Up    = -FRD z
    ps.pose.orientation.w = 1.0;
    trajectory_.header.stamp = ps.header.stamp;
    trajectory_.poses.push_back(ps);
    path_pub_->publish(trajectory_);

    if (log_counter_++ % 40 == 0) {
      RCLCPP_INFO(
        this->get_logger(), "[%s] pos=(%.2f, %.2f, %.2f) yaw=%.2f",
        fsm_->get_current_state().c_str(), pos.x(), pos.y(), pos.z(), orient[2]);
    }

    if (rclcpp::ok() && !fsm_->is_finished()) {
      fsm_->execute();
    } else {
      RCLCPP_INFO(this->get_logger(), "FSM terminou com: %s", fsm_->get_fsm_outcome().c_str());
      rclcpp::shutdown();
    }
  }

  std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
    const std::map<std::string, std::variant<double, std::string>> & defaults)
  {
    std::map<std::string, std::variant<double, std::string>> result;
    for (const auto & [name, default_value] : defaults) {
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
  std::shared_ptr<VisionFase1> vision_;
  std::unique_ptr<Fase1FSM> fsm_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path trajectory_;
  int log_counter_ = 0;
};

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);

  // O Drone JÁ sobe o próprio executor e a própria thread de spin no
  // construtor. Adicioná-lo a um executor aqui lança em tempo de execução:
  //
  //     what():  Node '/Drone' has already been added to an executor.
  //
  // O nó de visão, esse, é um rclcpp::Node comum e PRECISA entrar no executor
  // — sem isso o callback de detecções nunca roda e a missão fica cega sem
  // dizer por quê.
  auto drone = std::make_shared<Drone>();
  auto vision = std::make_shared<VisionFase1>();
  auto mission_node = std::make_shared<Fase1Node>(drone, vision);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(vision);
  executor.add_node(mission_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
