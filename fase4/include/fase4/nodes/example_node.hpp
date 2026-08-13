#ifndef FASE4__NODES__EXAMPLE_NODE_HPP_
#define FASE4__NODES__EXAMPLE_NODE_HPP_

// No de exemplo -- copie este arquivo para criar os seus.
//
// ANTES DE ESCREVER UM NO, PERGUNTE SE PRECISA DELE.
//
// A maior parte do que uma missao faz ja existe: o stdbt registra Arming,
// Takeoff, TakeoffAgain, GoTo, PrecisionLanding, ReturnHome e LandAndDisarm,
// todos adaptados dos estados do stdstates que ja voaram. E boa parte das
// DECISOES cabe em BlackboardBool e BlackboardMenorQue, sem codigo nenhum.
//
// Escreva um no quando a missao tiver logica propria -- ver uma base, decidir
// se ela e nova, contar quantas faltam.
//
// AS TRES FORMAS DE NO, e qual usar:
//
//   BT::SyncActionNode      termina no mesmo tick. Para calculo, registro,
//                           escrever na blackboard.
//   BT::StatefulActionNode  ocupa varios ticks: onStart, onRunning, onHalted.
//                           E o que usar para qualquer coisa que envolva
//                           mover o drone.
//   BT::ConditionNode       so responde sim ou nao, sem efeito colateral.
//
// Este exemplo e um StatefulActionNode, que e o caso mais comum e o unico com
// armadilha: o onHalted PRECISA existir e desfazer o que o no comecou. A
// arvore pode interromper um no a qualquer momento -- um decorador de timeout,
// um ReactiveFallback cuja condicao mudou -- e um no interrompido no meio de um
// deslocamento deixaria o drone se movendo.

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include "stdbt/fsm_action_node.hpp"   // kFsmBlackboardKey

class ExampleNode : public BT::StatefulActionNode
{
public:
    ExampleNode(const std::string &nome, const BT::NodeConfig &config)
        : BT::StatefulActionNode(nome, config) {}

    /// Portas sao a interface do no com o XML. Sem declarar, nao da para usar.
    ///
    ///     <ExampleNode voltas="5"/>
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("voltas", 5, "quantos ciclos ficar RUNNING")};
    }

    BT::NodeStatus onStart() override
    {
        // A blackboard da FSM traz o drone e os parametros do YAML. E a mesma
        // que os nos do stdbt usam, entao este no e os adaptados enxergam
        // exatamente o mesmo estado.
        if (auto bb = config().blackboard) {
            bb->get<fsm::Blackboard *>(stdbt::kFsmBlackboardKey, fsm_bb_);
        }
        if (fsm_bb_ == nullptr) {
            return BT::NodeStatus::FAILURE;
        }

        auto *drone_ptr = fsm_bb_->get<std::shared_ptr<Drone>>("drone");
        if (drone_ptr == nullptr) return BT::NodeStatus::FAILURE;
        drone_ = *drone_ptr;

        getInput("voltas", restantes_);
        drone_->log("");
        drone_->log("NO: ExampleNode");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (--restantes_ > 0) {
            return BT::NodeStatus::RUNNING;
        }
        drone_->log("ExampleNode terminou.");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        // Desfaca aqui o que o no comecou. Num no que move o drone, isto e
        // zerar a velocidade -- sem isso ele continua se movendo depois de
        // interrompido.
        if (drone_ != nullptr) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        }
    }

private:
    std::shared_ptr<Drone> drone_;
    fsm::Blackboard *fsm_bb_ = nullptr;
    int restantes_ = 0;
};

#endif  // FASE4__NODES__EXAMPLE_NODE_HPP_
