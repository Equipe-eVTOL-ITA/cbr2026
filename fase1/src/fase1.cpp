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
#include <limits>
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
    // DUAS decolagens, e a diferenca entre elas nao e cosmetica.
    //
    // TakeoffState() reancora o referencial do mundo na posicao atual do drone
    // (setHomePosition). Isso e necessario na decolagem INICIAL, depois de
    // armar, para que o EKF ja tenha convergido para o heading verdadeiro.
    //
    // Na REDECOLAGEM seria destrutivo: a origem do mundo pularia para a base
    // recem-visitada, e a lista de bases, a grade de varredura e a posicao de
    // casa passariam a se referir a um referencial que nao existe mais. Medido
    // em SITL: o NED cru do PX4 ficou parado em (3.417, -0.159) o tempo todo --
    // o drone nunca saiu do lugar -- enquanto o FRD da missao saltava de
    // (-0.16, -3.03) para (0.00, 0.03) a cada redecolagem, e o drone pousava
    // na mesma base indefinidamente.
    this->add_state("TAKEOFF", std::make_unique<TakeoffState>(true));
    this->add_state("TAKEOFF AGAIN", std::make_unique<TakeoffState>(false));
    this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
    this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
    this->add_state("PRECISION LANDING", std::make_unique<PrecisionLandingState>());
    this->add_state("REGISTER BASE", std::make_unique<RegisterBaseState>());
    this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

    // ---- Transições ----------------------------------------------------
    //
    //   ARMING → TAKEOFF → SEARCH BASE ⇄ PRECISION ALIGN → PRECISION LANDING
    //                           ↑                                  ↓
    //                   TAKEOFF AGAIN ←────────────────────── REGISTER BASE
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
      {"REGISTERED", "TAKEOFF AGAIN"},
      {"ERROR", "ERROR"}
    });

    this->add_transitions("TAKEOFF AGAIN", {
      {"TAKEOFF COMPLETED", "SEARCH BASE"},
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
      // Raio em que uma caixa reprojetada ainda conta como a MESMA base do
      // ciclo anterior. Menor que a menor distancia entre duas bases da arena,
      // ou o alvo caminha de base em base durante a descida.
      {"target_association_radius", 1.0},

      // PID de alinhamento
      {"pid_pos_kp", 1.0},
      {"pid_pos_ki", 0.01},
      {"pid_pos_kd", 0.05},
    };

    auto params = declareAndGetParameters(default_params);
    association_radius_ = std::get<double>(params.at("target_association_radius"));
    last_association_ = std::chrono::steady_clock::now();

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

    refreshAlignTarget(pos, orient);

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

  /// Mantém `align_target` apontando para A MESMA base que o SearchBaseState
  /// escolheu, reprojetada a cada ciclo.
  ///
  /// A versão anterior desta função escrevia, todo ciclo, a projeção da caixa
  /// mais próxima do CENTRO DA IMAGEM. A intenção era boa — reprojetar evita
  /// que a estimativa congelada no instante da descoberta acumule erro de
  /// paralaxe conforme o drone desce — mas ela ignora qual base a busca
  /// escolheu.
  ///
  /// O efeito medido em SITL: logo após redecolar de uma base, a caixa mais
  /// central é a base de onde o drone acabou de sair. A busca anunciava
  /// "Base NOVA em {-1.98, -0.88}", a 2,16 m, e meio segundo depois o
  /// alinhamento reportava erro de 0,07 m — porque estava mirando outra coisa.
  /// O drone pousava de novo na mesma base, indefinidamente.
  ///
  /// A correção é associar por PROXIMIDADE AO ALVO, não ao centro da imagem:
  /// entre as caixas do quadro, vale a que projeta mais perto de onde o alvo
  /// estava no ciclo anterior, e só se estiver dentro de um raio de associação.
  void refreshAlignTarget(const Eigen::Vector3d & pos, const Eigen::Vector3d & orient)
  {
    // O SearchBaseState é quem ESCOLHE a base. Quando ele escreve um alvo novo,
    // adotamos essa escolha e reiniciamos a associação.
    if (auto * escolhido = fsm_->blackboard_get<Eigen::Vector3d>("align_target")) {
      if (!have_target_ || (*escolhido - last_written_).norm() > 1e-9) {
        target_ = *escolhido;
        have_target_ = true;
        last_association_ = std::chrono::steady_clock::now();
      }
    }

    if (!have_target_) return;

    // Entre as caixas deste quadro, a que projeta mais perto do alvo atual.
    double melhor = std::numeric_limits<double>::max();
    Eigen::Vector3d candidato = target_;
    for (const auto & box : vision_->boxes()) {
      const Eigen::Vector3d p = vision_->project(pos, orient, box);
      const double d = (p.head<2>() - target_.head<2>()).norm();
      if (d < melhor) {
        melhor = d;
        candidato = p;
      }
    }

    // O portão é o que separa "reprojetei a mesma base" de "peguei a vizinha".
    // Sem ele, uma base a dois metros seria adotada como se fosse a mesma, e o
    // alvo caminharia de base em base durante a descida.
    if (melhor <= association_radius_) {
      target_ = candidato;
      last_association_ = std::chrono::steady_clock::now();
    }

    fsm_->blackboard_set<Eigen::Vector3d>("align_target", target_);
    last_written_ = target_;

    // A idade agora conta o tempo sem ver ESTA base, e não sem ver base alguma.
    // Com a contagem antiga, ficar olhando para a base vizinha mantinha a idade
    // em zero e o PrecisionAlignState nunca desistia de um alvo já perdido.
    const std::chrono::duration<double> idade =
      std::chrono::steady_clock::now() - last_association_;
    fsm_->blackboard_set<float>("align_target_age", static_cast<float>(idade.count()));
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

  // Estado da associacao entre o alvo escolhido pela busca e as caixas do
  // quadro atual. Ver refreshAlignTarget.
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_written_ = Eigen::Vector3d::Zero();
  bool have_target_ = false;
  double association_radius_ = 1.0;
  std::chrono::steady_clock::time_point last_association_;
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
