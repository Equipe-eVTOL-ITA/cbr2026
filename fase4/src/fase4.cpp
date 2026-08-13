#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "stdbt/registrar.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Nos desta missao. Crie em include/fase4/nodes/ e inclua aqui.
// #include "fase4/nodes/meu_no.hpp"

/**
 * @brief Missao fase4, modelada como Behavior Tree.
 *
 * A DIFERENCA PARA UMA FSM, em uma frase: aqui nao ha transicoes nomeadas. A
 * ESTRUTURA da arvore -- Sequence, Fallback, decoradores -- faz o papel delas.
 *
 *   Sequence   executa os filhos em ordem; para no primeiro que falhar
 *   Fallback   tenta os filhos em ordem; para no primeiro que der certo
 *   Retry, Timeout, Inverter, ...  decoradores, que modificam um filho
 *
 * A arvore vive em trees/fase4.xml e e instalada no share/ do pacote. Trocar a
 * logica da missao e editar esse XML e relancar -- sem recompilar. E e o que
 * permite abrir a arvore no Groot2 e ver os nos piscando enquanto o drone voa.
 */
class Fase4Node : public rclcpp::Node {
public:
    explicit Fase4Node(std::shared_ptr<Drone> drone)
        : rclcpp::Node("fase4_node"), drone_(drone) {

        // Valores padrao. O launch sobrescreve com config/simulation.yaml ou
        // config/flight.yaml -- por isso trocar de simulacao para voo e trocar
        // de YAML, nao editar codigo.
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Decolagem
            {"takeoff_height",          -2.5},   // metros, FRD: negativo e para cima
            {"max_vertical_velocity",    1.2},
            {"position_tolerance",       0.15},

            // Pouso
            {"landing_velocity_max",     0.5},
            {"landing_velocity_min",     0.15},
            {"max_base_height",         -0.2},

            // Movimento horizontal
            {"max_horizontal_velocity",  1.5},

            // Qual arvore carregar, relativa a share/fase4/trees/. Os dois
            // perfis podem apontar para arvores diferentes -- e uma das razoes
            // de a arvore ser um arquivo e nao codigo.
            {"tree_file",                std::string("fase4.xml")},

            // Porta do Groot2. 0 desliga.
            //
            // Com ela ligada, abra o Groot2, conecte nesta porta e veja a
            // arvore ao vivo: qual no esta RUNNING, qual falhou, o valor de
            // cada porta. E a maior vantagem pratica da BT sobre a FSM na hora
            // de depurar.
            {"groot2_port",           1667.0},

            // ACRESCENTE aqui os parametros desta missao, e replique-os nos
            // dois YAML de config/.
        };

        auto params = declareAndGetParameters(default_params);

        // A blackboard da FSM continua sendo a fonte de parametros.
        //
        // Os estados do stdstates leem dela, e o adaptador do stdbt os executa
        // sem modificacao. O efeito pratico e que esta missao usa os MESMOS
        // YAML que uma missao em FSM usaria.
        //
        // Os parametros entram como FLOAT, nunca double: fsm::Blackboard faz
        // cast sem checagem, e gravar double para ler float devolve lixo.
        for (const auto &[key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                blackboard_.set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                blackboard_.set<std::string>(key, std::get<std::string>(value));
            }
        }
        blackboard_.set<std::shared_ptr<Drone>>("drone", drone_);

        // ======================= ARVORE =======================
        BT::BehaviorTreeFactory factory;
        stdbt::registerAll(factory);
        // ACRESCENTE aqui os nos desta missao, ex.:
        // factory.registerNodeType<MeuNo>("MeuNo");

        auto bt_blackboard = BT::Blackboard::create();
        bt_blackboard->set(stdbt::kFsmBlackboardKey, &blackboard_);

        const auto arquivo = std::get<std::string>(params.at("tree_file"));
        const auto caminho =
            ament_index_cpp::get_package_share_directory("fase4") + "/trees/" + arquivo;

        try {
            tree_ = std::make_unique<BT::Tree>(
                factory.createTreeFromFile(caminho, bt_blackboard));
        } catch (const std::exception &e) {
            // Nome de no errado no XML NAO e erro de compilacao: aparece aqui.
            // O test/test_tree.cpp existe para pegar isso no CI, antes do voo.
            RCLCPP_FATAL(this->get_logger(),
                         "nao consegui carregar a arvore '%s': %s", caminho.c_str(), e.what());
            throw;
        }

        RCLCPP_INFO(this->get_logger(), "arvore carregada de %s", caminho.c_str());

        const int porta = static_cast<int>(std::get<double>(params.at("groot2_port")));
        if (porta > 0) {
            groot_ = std::make_unique<BT::Groot2Publisher>(*tree_, porta);
            RCLCPP_INFO(this->get_logger(), "Groot2 escutando na porta %d", porta);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),                 // 20 Hz
            std::bind(&Fase4Node::tick, this));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "missao fase4 iniciada (Behavior Tree)");
    }

private:
    void tick() {
        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        // FRD -> ENU para visualizar no RViz2.
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp       = this->now();
        ps.header.frame_id    = "map";
        ps.pose.position.x    =  static_cast<float>(pos.y());
        ps.pose.position.y    =  static_cast<float>(pos.x());
        ps.pose.position.z    = -static_cast<float>(pos.z());
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        if (log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "pos=(%.2f, %.2f, %.2f) yaw=%.2f rad",
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        static_cast<float>(pos.z()),
                        static_cast<float>(orient[2]));
        }

        if (!rclcpp::ok() || terminou_) {
            return;
        }

        // tickOnce(), NUNCA tickWhileRunning().
        //
        // O tickWhileRunning -- que e o exemplo da documentacao oficial --
        // BLOQUEIA ate a arvore terminar. Dentro do callback de um timer isso
        // congela o executor: a telemetria para, a visao para, e o no fica sem
        // responder enquanto o drone voa.
        //
        // O timer ja da o ritmo, do mesmo jeito que da para a FSM.
        const BT::NodeStatus status = tree_->tickOnce();

        if (status != BT::NodeStatus::RUNNING) {
            terminou_ = true;
            RCLCPP_INFO(this->get_logger(), "arvore terminou com: %s",
                        BT::toStr(status).c_str());
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
    fsm::Blackboard blackboard_;
    std::unique_ptr<BT::Tree> tree_;
    std::unique_ptr<BT::Groot2Publisher> groot_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
    int log_counter_ = 0;
    bool terminou_ = false;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    // O Drone JA sobe o proprio executor e a propria thread de spin no
    // construtor. Adiciona-lo a um executor aqui lanca em tempo de execucao:
    //
    //     what():  Node '/Drone' has already been added to an executor.
    //
    // Por isso so o no da missao entra no executor deste main.
    auto drone        = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Fase4Node>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
