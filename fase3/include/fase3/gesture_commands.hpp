#pragma once

#include <string>

#include <Eigen/Eigen>

namespace fase3
{

// Gestos que encerram o controle.
inline constexpr const char * kGestoPousar = "Thumb_Down";
inline constexpr const char * kGestoVoltar = "Open_Palm";

inline Eigen::Vector3d velocidadeDoGesto(
  const std::string & gesto, double control_speed, double climb_rate)
{
  if (gesto == "Pointing_Up") {
    return {control_speed, 0.0, climb_rate};      // frente
  }
  if (gesto == "Closed_Fist") {
    return {-control_speed, 0.0, climb_rate};     // ré
  }
  if (gesto == "Victory") {
    return {0.0, control_speed, climb_rate};      // direita
  }
  if (gesto == "ILoveYou") {
    return {0.0, -control_speed, climb_rate};     // esquerda
  }
  return {0.0, 0.0, climb_rate};
}

// O gesto encerra o controle por gestos?
inline bool ehGestoDeEncerramento(const std::string & gesto)
{
  return gesto == kGestoPousar || gesto == kGestoVoltar;
}
class ConfirmadorDeEncerramento
{
public:
  // Rearma a trava. Tem de ser chamado no `on_enter` do estado, sempre.
  void reiniciar()
  {
    gesto_.clear();
    ciclos_ = 0;
    travado_ = true;
  }

  // Alimenta um ciclo e devolve o gesto CONFIRMADO, ou "" se não há comando.
  std::string atualiza(const std::string & gesto, int ciclos_necessarios)
  {
    if (!ehGestoDeEncerramento(gesto)) {
      // O gesto de chamada foi solto: comandos de encerramento passam a valer.
      travado_ = false;
      gesto_.clear();
      ciclos_ = 0;
      return "";
    }

    if (travado_) {
      return "";
    }

    if (gesto == gesto_) {
      ciclos_++;
    } else {
      gesto_ = gesto;
      ciclos_ = 1;
    }

    return ciclos_ >= ciclos_necessarios ? gesto : "";
  }

  // A trava ainda está segurando? Só para log e teste.
  bool travado() const {return travado_;}

private:
  std::string gesto_;
  int ciclos_ = 0;
  bool travado_ = true;
};

}  // namespace fase3
