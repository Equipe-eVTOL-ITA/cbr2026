#pragma once

// O vocabulário de gestos, como função pura.
//
// Separado do estado pelo mesmo motivo que a `vision_geometry` foi separada do
// nó ROS: é a regra que decide para onde o drone vai, e conferi-la não pode
// depender de subir simulação e olhar. Aqui um teste alimenta uma string e
// confere um vetor.

#include <string>

#include <Eigen/Eigen>

namespace fase3
{

/// Gestos que encerram o controle. Ficam nomeados para que o estado e os testes
/// falem da mesma coisa, e para que trocar um deles seja uma linha.
inline constexpr const char * kGestoPousar = "Thumb_Down";
inline constexpr const char * kGestoVoltar = "Open_Palm";

/// Velocidade em EIXOS DO CORPO para um gesto direcional.
///
/// x é frente, y é direita, z é para BAIXO (FRD). O componente z vem de fora,
/// já calculado pelo PID de altitude, porque ele vale para qualquer gesto —
/// inclusive nenhum.
///
/// Gesto desconhecido, vazio, ou um dos de encerramento devolve apenas a
/// correção vertical: o drone mantém a posição horizontal. Isso é deliberado —
/// o estado neutro do controle por gestos é PARAR, não continuar o último
/// comando. Um operador que baixa a mão espera que o drone pare.
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

/// O gesto encerra o controle por gestos?
inline bool ehGestoDeEncerramento(const std::string & gesto)
{
  return gesto == kGestoPousar || gesto == kGestoVoltar;
}

}  // namespace fase3
