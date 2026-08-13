#pragma once

// GestureControlState — o drone obedece à mão.
//
// Enquanto está neste estado, o drone faz duas coisas ao mesmo tempo:
//
//   - MANTÉM A MÃO CENTRADA na imagem, por PID: o erro horizontal do centroide
//     vira taxa de guinada, o vertical vira velocidade de subida/descida. É o
//     que faz o drone acompanhar o operador sem que ele precise mandar;
//   - OBEDECE ao gesto direcional, deslocando-se em eixos do corpo.
//
// Contrato com a blackboard:
//
//   entrada  "vision"          std::shared_ptr<GestureFase3>
//            "control_speed"           float
//            "max_horizontal_velocity" float
//            "detection_timeout"       float
//            "command_confirm_cycles"  float
//            "yaw_pid_kp" / "_ki" / "_kd"     float
//            "climb_pid_kp" / "_ki" / "_kd"   float
//
// Outcomes: ""            (controlando)
//           "LAND NOW"    (Thumb_Down confirmado)
//           "GO HOME"     (Open_Palm confirmado)
//           "LOST HAND"   (sem mão há tempo demais)
//           "ERROR"

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"
#include "drone/transformations.hpp"

#include "stdstates/blackboard_params.hpp"

#include "fase3/gesture_commands.hpp"
#include "fase3/gesture_fase3.hpp"

class GestureControlState : public fsm::State
{
public:
  GestureControlState()
  : fsm::State(),
    pid_yaw_(0.0f, 0.0f, 0.0f, 0.5f, stdstates::kPidSampleTime),
    pid_climb_(0.0f, 0.0f, 0.0f, 0.5f, stdstates::kPidSampleTime) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;
    ciclos_confirmando_ = 0;
    gesto_em_confirmacao_.clear();
    log_counter_ = 0;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: GESTURE CONTROL");

    auto vision_ptr = blackboard.get<std::shared_ptr<GestureFase3>>("vision");
    if (vision_ptr == nullptr || *vision_ptr == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'vision'");
      return;
    }
    vision_ = *vision_ptr;

    if (!stdstates::require(blackboard, drone_, "control_speed", control_speed_)) return;
    if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", max_vel_)) return;
    if (!stdstates::require(blackboard, drone_, "detection_timeout", timeout_)) return;
    if (!stdstates::require(blackboard, drone_, "command_confirm_cycles", confirmacoes_)) return;

    float ykp = 0, yki = 0, ykd = 0, ckp = 0, cki = 0, ckd = 0;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_kp", ykp)) return;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_ki", yki)) return;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_kd", ykd)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_kp", ckp)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_ki", cki)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_kd", ckd)) return;

    // Setpoint 0.5: o centro da imagem, em coordenada normalizada.
    //
    // O sample_time explícito é obrigatório. Com o padrão de 0.05 s igual ao
    // timer da FSM, `compute()` devolve 0.0f — não o último valor — sempre que
    // o jitter faz o intervalo ficar de menos. O sintoma é um drone que
    // "engasga", e é indistinguível de PID mal sintonizado.
    pid_yaw_ = PidController(ykp, yki, ykd, 0.5f, stdstates::kPidSampleTime);
    pid_climb_ = PidController(ckp, cki, ckd, 0.5f, stdstates::kPidSampleTime);
    pid_yaw_.reset();
    pid_climb_.reset();

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";
    log_counter_++;

    // NÃO HÁ `usleep` AQUI, e a ausência é o ponto.
    //
    // A versão de 2025 terminava o `act()` com `usleep(50000)`, sobre um timer
    // que já é de 50 ms. Isso bloqueia a thread do executor: a telemetria, a
    // visão e o próprio `Drone` param de responder por meio ciclo, e a taxa
    // efetiva da FSM cai pela metade. O timer já dá o ritmo.

    if (vision_->secondsSinceHand() > timeout_) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log(
        "Mao perdida: " + std::to_string(vision_->secondsSinceHand()) +
        "s sem posicao (limite " + std::to_string(timeout_) + "s).");
      return "LOST HAND";
    }

    const auto [hand_x, hand_y] = vision_->hand();
    const std::string gesto = vision_->gesture();

    // Correção contínua para manter a mão no centro do quadro. Vale para
    // qualquer gesto, inclusive nenhum.
    const float yaw_rate = pid_yaw_.compute(hand_x);

    // Sinal invertido: y da imagem cresce para BAIXO, e z do corpo também. Mão
    // acima do centro (y pequeno) tem de fazer o drone SUBIR, isto é z
    // negativo. O PID já devolve erro positivo nesse caso, daí a negação.
    const float climb_rate = -pid_climb_.compute(hand_y);

    // Os comandos que ENCERRAM o controle exigem confirmação; os direcionais
    // não. A assimetria é deliberada: pousar e voltar para casa são
    // irreversíveis dentro desta fase, enquanto andar para a frente se corrige
    // sozinho no ciclo seguinte. Um único quadro mal classificado não pode
    // mandar o drone pousar.
    if (fase3::ehGestoDeEncerramento(gesto)) {
      if (gesto == gesto_em_confirmacao_) {
        ciclos_confirmando_++;
      } else {
        gesto_em_confirmacao_ = gesto;
        ciclos_confirmando_ = 1;
      }

      if (ciclos_confirmando_ >= static_cast<int>(confirmacoes_)) {
        drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        if (gesto == fase3::kGestoPousar) {
          drone_->log("Comando: POUSAR");
          return "LAND NOW";
        }
        drone_->log("Comando: VOLTAR PARA CASA");
        return "GO HOME";
      }
    } else {
      gesto_em_confirmacao_.clear();
      ciclos_confirmando_ = 0;
    }

    // Gesto direcional -> velocidade em eixos do CORPO, depois rotacionada
    // para o mundo pelo yaw atual. Sem isso o "para a frente" do operador
    // seria o +x do mundo, e não a frente do drone.
    Eigen::Vector3d v_corpo = fase3::velocidadeDoGesto(gesto, control_speed_, climb_rate);

    Eigen::Vector2d horizontal(v_corpo.x(), v_corpo.y());
    if (horizontal.norm() > max_vel_) {
      horizontal = horizontal.normalized() * max_vel_;
      v_corpo.x() = horizontal.x();
      v_corpo.y() = horizontal.y();
    }

    const Eigen::Vector3d v_mundo =
      adjust_velocity_using_yaw(v_corpo, drone_->getOrientation().z());

    drone_->setLocalVelocity(v_mundo.x(), v_mundo.y(), v_mundo.z(), yaw_rate);

    if (log_counter_ % 20 == 0) {
      drone_->log(
        "Gesto: '" + (gesto.empty() ? std::string("-") : gesto) +
        "' | mao=(" + std::to_string(hand_x) + ", " + std::to_string(hand_y) +
        ") | guinada=" + std::to_string(yaw_rate));
    }

    return "";
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (drone_ != nullptr) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
    }
  }

private:
  std::shared_ptr<Drone> drone_;
  std::shared_ptr<GestureFase3> vision_;

  PidController pid_yaw_;
  PidController pid_climb_;

  float control_speed_ = 0.0f;
  float max_vel_ = 0.0f;
  float timeout_ = 0.0f;
  float confirmacoes_ = 5.0f;

  std::string gesto_em_confirmacao_;
  int ciclos_confirmando_ = 0;
  int log_counter_ = 0;
  bool ok_ = false;
};
