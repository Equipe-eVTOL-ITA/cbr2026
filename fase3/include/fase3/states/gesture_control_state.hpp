#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

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

    confirmador_.reiniciar();
    log_counter_ = 0;
    ultimo_act_ = std::chrono::steady_clock::now();

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

    yaw_tracking_ = stdstates::optional<float>(blackboard, "yaw_tracking", 0.0f) > 0.5f;

    climb_tracking_ = stdstates::optional<float>(blackboard, "climb_tracking", 0.0f) > 0.5f;

    max_vz_ = stdstates::optional<float>(blackboard, "max_vertical_velocity", 1.0f);

    z_ref_ = static_cast<float>(drone_->getLocalPosition().z());
    yaw_ref_ = static_cast<float>(drone_->getOrientation().z());

    float ykp = 0, yki = 0, ykd = 0, ckp = 0, cki = 0, ckd = 0;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_kp", ykp)) return;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_ki", yki)) return;
    if (!stdstates::require(blackboard, drone_, "yaw_pid_kd", ykd)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_kp", ckp)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_ki", cki)) return;
    if (!stdstates::require(blackboard, drone_, "climb_pid_kd", ckd)) return;

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


    if (vision_->secondsSinceHand() > timeout_) {
      drone_->setMixedSetpoint(0.0f, 0.0f, z_ref_, yaw_ref_);
      drone_->log(
        "Mao perdida: " + std::to_string(vision_->secondsSinceHand()) +
        "s sem posicao (limite " + std::to_string(timeout_) + "s).");
      return "LOST HAND";
    }

    const auto [hand_x, hand_y] = vision_->hand();
    const std::string gesto = vision_->gesture();

    const float passo = dt();

    if (yaw_tracking_) {
      yaw_ref_ += pid_yaw_.compute(hand_x) * passo;
    }

    if (climb_tracking_) {
      const float climb_rate =
        std::clamp(-pid_climb_.compute(hand_y), -max_vz_, max_vz_);
      z_ref_ += climb_rate * passo;
    }

    const std::string confirmado =
      confirmador_.atualiza(gesto, static_cast<int>(confirmacoes_));

    if (!confirmado.empty()) {
      drone_->setMixedSetpoint(0.0f, 0.0f, z_ref_, yaw_ref_);
      if (confirmado == fase3::kGestoPousar) {
        drone_->log("Comando: POUSAR");
        return "LAND NOW";
      }
      drone_->log("Comando: VOLTAR PARA CASA");
      return "GO HOME";
    }

    Eigen::Vector3d v_corpo = fase3::velocidadeDoGesto(gesto, control_speed_, 0.0);

    Eigen::Vector2d horizontal(v_corpo.x(), v_corpo.y());
    if (horizontal.norm() > max_vel_) {
      horizontal = horizontal.normalized() * max_vel_;
      v_corpo.x() = horizontal.x();
      v_corpo.y() = horizontal.y();
    }

    drone_->setMixedSetpoint(
      static_cast<float>(v_corpo.x()), static_cast<float>(v_corpo.y()),
      z_ref_, yaw_ref_);

    if (log_counter_ % 20 == 0) {
      drone_->log(
        "Gesto: '" + (gesto.empty() ? std::string("-") : gesto) +
        "' | mao=(" + std::to_string(hand_x) + ", " + std::to_string(hand_y) +
        ") | z_ref=" + std::to_string(z_ref_) +
        " z=" + std::to_string(drone_->getLocalPosition().z()) +
        (confirmador_.travado() ? " | (solte a mao para habilitar os comandos)" : ""));
    }

    return "";
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (drone_ != nullptr) {
      drone_->setMixedSetpoint(0.0f, 0.0f, z_ref_, yaw_ref_);
    }
  }

private:
  float dt()
  {
    const auto agora = std::chrono::steady_clock::now();
    const std::chrono::duration<float> passado = agora - ultimo_act_;
    ultimo_act_ = agora;
    return std::clamp(passado.count(), 0.0f, 0.2f);
  }

  std::shared_ptr<Drone> drone_;
  std::shared_ptr<GestureFase3> vision_;

  PidController pid_yaw_;
  PidController pid_climb_;

  float control_speed_ = 0.0f;
  float max_vel_ = 0.0f;
  float timeout_ = 0.0f;
  float confirmacoes_ = 5.0f;
  float max_vz_ = 1.0f;

  bool yaw_tracking_ = false;
  bool climb_tracking_ = false;

  // O que o estado defende enquanto obedece aos gestos.
  float z_ref_ = 0.0f;
  float yaw_ref_ = 0.0f;

  std::chrono::steady_clock::time_point ultimo_act_;

  fase3::ConfirmadorDeEncerramento confirmador_;
  int log_counter_ = 0;
  bool ok_ = false;
};
