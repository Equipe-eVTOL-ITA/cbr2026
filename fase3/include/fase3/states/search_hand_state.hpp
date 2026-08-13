#pragma once

// SearchHandState — gira procurando a mão do operador.
//
// A navegação (girar dentro de um setor, invertendo nos extremos, sem quebrar
// na descontinuidade em pi) é genérica e vem herdada do `YawSweepState`. O que
// fica aqui é a regra da fase: qual gesto significa "achei", e por quanto tempo
// ele precisa aparecer.
//
// Mesmo padrão de `SearchBaseState` sobre `WaypointListState` na fase 1.
//
// Contrato com a blackboard:
//
//   entrada   "vision"            std::shared_ptr<GestureFase3>
//   opcional  "hand_found_gesture" std::string  (padrão: "Open_Palm")
//             + tudo o que o YawSweepState exige
//
// Outcomes: ""            (procurando)
//           "HAND FOUND"  (gesto de chamada, estável)
//           "ERROR"

#include <memory>
#include <string>

#include "stdstates/yaw_sweep_state.hpp"

#include "fase3/gesture_fase3.hpp"

class SearchHandState : public YawSweepState
{
public:
  SearchHandState()
  : YawSweepState() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    YawSweepState::on_enter(blackboard);
    if (!ok_) return;

    ok_ = false;
    ciclos_com_gesto_ = 0;

    auto vision_ptr = blackboard.get<std::shared_ptr<GestureFase3>>("vision");
    if (vision_ptr == nullptr || *vision_ptr == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'vision'");
      return;
    }
    vision_ = *vision_ptr;

    gesto_de_chamada_ =
      stdstates::optional<std::string>(blackboard, "hand_found_gesture", "Open_Palm");

    if (!stdstates::require(blackboard, drone_, "search_confirm_cycles", confirmacoes_)) {
      return;
    }

    drone_->log("STATE: SEARCH HAND (procurando " + gesto_de_chamada_ + ")");
    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    // A confirmação é por ciclos CONSECUTIVOS, não por um quadro solto.
    //
    // O classificador oscila, e girar sobre o próprio eixo passa a mão pelo
    // campo de visão de raspão. Sem confirmação, o drone pararia de girar por
    // causa de um único quadro em que a silhueta de qualquer coisa foi
    // classificada como palma aberta.
    if (vision_->gesture() == gesto_de_chamada_) {
      ciclos_com_gesto_++;
    } else {
      ciclos_com_gesto_ = 0;
    }

    if (ciclos_com_gesto_ >= static_cast<int>(confirmacoes_)) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Mao encontrada.");
      return "HAND FOUND";
    }

    sweep();
    return "";
  }

private:
  std::shared_ptr<GestureFase3> vision_;
  std::string gesto_de_chamada_{"Open_Palm"};
  float confirmacoes_ = 5.0f;
  int ciclos_com_gesto_ = 0;
};
