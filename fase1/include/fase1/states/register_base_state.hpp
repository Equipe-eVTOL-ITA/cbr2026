#pragma once

// RegisterBaseState — anota no registro a base em que o drone acabou de pousar.
//
// Em 2025 isto vivia no `on_exit` do estado de pouso. Não podia continuar lá:
// um estado GENÉRICO de pouso não pode saber o que é uma "base", nem quantas a
// prova tem. Ao portar o pouso para o `stdstates`, esta contabilidade ficou de
// fora de propósito, e é aqui que ela reaparece — na fase, que é de quem ela é.
//
// O estado não voa: executa uma vez e transita.
//
// Contrato com a blackboard:
//
//   entrada/saída  "bases"      std::vector<Eigen::Vector3d>
//   entrada        "num_bases"  float  (quantas a prova tem)
//
// Outcome: "REGISTERED", ou "ERROR" se faltar parâmetro.

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "stdstates/blackboard_params.hpp"

#include "fase1/vision_fase1.hpp"

class RegisterBaseState : public fsm::State
{
public:
  RegisterBaseState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: REGISTER BASE");

    bases_ = blackboard.get<std::vector<Eigen::Vector3d>>("bases");
    if (bases_ == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'bases'");
      return;
    }

    if (!stdstates::require(blackboard, drone_, "num_bases", num_bases_)) return;

    auto vision_ptr = blackboard.get<std::shared_ptr<VisionFase1>>("vision");
    if (vision_ptr != nullptr) vision_ = *vision_ptr;

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    if (!ok_) return "ERROR";

    // A posição em que o drone REALMENTE pousou, não a que a visão estimou.
    // Esta é a medida boa: o pouso aconteceu, então a base está debaixo dele.
    // É contra ela que as detecções futuras serão comparadas, e é por isso que
    // o drone não volta a pousar na mesma base.
    const Eigen::Vector3d pos = drone_->getLocalPosition();
    bases_->push_back(pos);

    if (vision_ != nullptr) vision_->publishBase("known", pos);

    drone_->log(
      "Base " + std::to_string(bases_->size()) + " de " +
      std::to_string(static_cast<int>(num_bases_)) + " registrada em {" +
      std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + "}");

    // O "casa" entrou no vetor como primeiro elemento na montagem da FSM, para
    // que o drone não trate o próprio ponto de decolagem como base a visitar.
    // Por isso a conta desconta um.
    const size_t visitadas = bases_->size() > 0 ? bases_->size() - 1 : 0;
    const bool acabou = visitadas >= static_cast<size_t>(num_bases_);

    blackboard.set<bool>("finished_bases", acabou);
    if (acabou) {
      drone_->log("Todas as " + std::to_string(static_cast<int>(num_bases_)) + " bases visitadas.");
    }

    return "REGISTERED";
  }

private:
  std::shared_ptr<Drone> drone_;
  std::shared_ptr<VisionFase1> vision_;
  std::vector<Eigen::Vector3d> * bases_ = nullptr;
  float num_bases_ = 0.0f;
  bool ok_ = false;
};
