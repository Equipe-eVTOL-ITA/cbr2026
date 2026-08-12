#pragma once

// SearchBaseState — varre a grade procurando bases ainda não visitadas.
//
// É o único estado genuinamente específico desta fase, e por isso o único que
// mora aqui em vez de no `stdstates`. Ele junta duas coisas:
//
//   - a NAVEGAÇÃO, que é genérica e vem herdada do WaypointListState;
//   - a REGRA DE NEGÓCIO, que é: uma detecção só interessa se estiver longe o
//     bastante de todas as bases já visitadas.
//
// Sem essa segunda parte o drone pousaria eternamente na primeira base: ao
// redecolar ele a veria de novo, e a veria como nova.
//
// Contrato com a blackboard:
//
//   entrada  "waypoints"          std::vector<ArenaPoint>   (a FSM gera)
//            "bases"              std::vector<Eigen::Vector3d>  já visitadas
//            "known_base_radius"  float
//            + tudo o que o WaypointListState exige
//
//   saída    "align_target"       Eigen::Vector3d   base nova encontrada
//
// Outcomes: ""              (varrendo)
//           "BASE FOUND"    (achou uma base nova; escreveu "align_target")
//           "SEARCH ENDED"  (grade toda percorrida)
//           "ERROR"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "stdstates/next_waypoints.hpp"

#include "fase1/vision_fase1.hpp"

class SearchBaseState : public WaypointListState
{
public:
  SearchBaseState()
  : WaypointListState() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    // A navegação prepara drone_, waypoints_, tolerâncias e ok_.
    WaypointListState::on_enter(blackboard);
    if (!ok_) return;

    ok_ = false;
    log_counter_ = 0;

    auto vision_ptr = blackboard.get<std::shared_ptr<VisionFase1>>("vision");
    if (vision_ptr == nullptr || *vision_ptr == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'vision'");
      return;
    }
    vision_ = *vision_ptr;

    bases_ = blackboard.get<std::vector<Eigen::Vector3d>>("bases");
    if (bases_ == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'bases'");
      return;
    }

    if (!stdstates::require(blackboard, drone_, "known_base_radius", known_base_radius_)) {
      return;
    }

    drone_->log("STATE: SEARCH BASE");
    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    if (!ok_) return "ERROR";
    log_counter_++;

    // Duas razões para encerrar a busca, e esta é a primeira: já se pousou em
    // todas as bases que a prova tem. Continuar varrendo depois disso só gasta
    // bateria e arrisca um pouso a mais numa base já contada.
    if (stdstates::optional<bool>(blackboard, "finished_bases", false)) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Todas as bases visitadas; encerrando a busca.");
      return "SEARCH ENDED";
    }

    // A visão primeiro: achar uma base interrompe a varredura. O
    // WaypointListState guarda o progresso em `is_visited`, então voltar a
    // este estado depois do pouso retoma de onde parou.
    if (vision_->hasDetection()) {
      const Eigen::Vector3d pos = drone_->getLocalPosition();
      const Eigen::Vector3d rpy = drone_->getOrientation();

      for (const auto & box : vision_->boxes()) {
        const Eigen::Vector3d estimada = vision_->project(pos, rpy, box);
        vision_->publishBase("estimate", estimada);

        if (isNew(estimada)) {
          drone_->log(
            "Base NOVA em {" + std::to_string(estimada.x()) + ", " +
            std::to_string(estimada.y()) + "}");
          vision_->publishBase("first_estimate", estimada);

          // O tipo é parte do contrato: o PrecisionAlignState lê
          // Eigen::Vector3d. Gravar outro tipo aqui compila e produz lixo,
          // porque a blackboard faz cast sem checagem.
          blackboard.set<Eigen::Vector3d>("align_target", estimada);
          blackboard.set<float>("align_target_age", 0.0f);
          return "BASE FOUND";
        }
      }
    }

    // Nenhuma base nova: segue a grade.
    return navigate() == "WAYPOINTS ENDED" ? "SEARCH ENDED" : "";
  }

private:
  /// Uma estimativa é base nova se está a mais de `known_base_radius` de TODAS
  /// as bases já visitadas.
  ///
  /// A comparação é HORIZONTAL de propósito. A componente z da estimativa
  /// carrega o erro da projeção — e, quando o PnP entra, o erro de escala do
  /// tamanho aparente do alvo. Incluir z na distância faria uma base conhecida
  /// parecer nova só porque a altura estimada oscilou.
  bool isNew(const Eigen::Vector3d & candidata) const
  {
    double menor = std::numeric_limits<double>::max();
    for (const auto & base : *bases_) {
      const double d = (candidata.head<2>() - base.head<2>()).norm();
      menor = std::min(menor, d);
      if (d < known_base_radius_) return false;
    }

    if (log_counter_ % 20 == 0 && !bases_->empty()) {
      drone_->log("Distancia a base conhecida mais proxima: " + std::to_string(menor) + " m");
    }
    return true;
  }

  std::shared_ptr<VisionFase1> vision_;
  std::vector<Eigen::Vector3d> * bases_ = nullptr;
  float known_base_radius_ = 0.0f;
  int log_counter_ = 0;
};
