#pragma once

// Geração da grade de varredura em zigue-zague.
//
// Portado do laço que em 2025 vivia solto dentro do construtor da FSM. Aqui é
// uma função pura — recebe números, devolve pontos — o que a torna conferível
// sem subir nada.
//
// O padrão é uma serpentina: avança em x, cruza em y, avança em x, volta em y.
// Varrer assim cobre a área sem repetir trecho e sem exigir que o drone volte
// ao começo de cada faixa.
//
//      home ●───────────────┐
//                           │
//           ┌───────────────┘
//           │
//           └───────────────┐

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

#include "stdstates/arena_point.hpp"

namespace fase1
{

/// Monta a serpentina a partir de casa.
///
/// `y_length` é assinado: negativo varre para a esquerda do drone. `altitude`
/// vem em FRD (negativa é para cima).
inline std::vector<ArenaPoint> makeSerpentineGrid(
  double home_x, double home_y,
  double y_length, double step_x,
  int num_steps, double altitude)
{
  std::vector<ArenaPoint> waypoints;
  if (num_steps <= 0 || std::abs(step_x) <= 0.0) return waypoints;

  double x = home_x;
  double y = home_y;
  int passos = 0;

  // O ciclo de quatro estados é o que produz a serpentina:
  //   0: cruza para o lado oposto      2: cruza de volta
  //   1 e 3: avança uma faixa em x
  int estado = 0;
  bool na_ponta = false;

  while (passos < num_steps || !na_ponta) {
    switch (estado % 4) {
      case 0:
        y = home_y + y_length;
        na_ponta = true;
        break;
      case 2:
        y = home_y;
        na_ponta = true;
        break;
      default:  // 1 e 3: avança
        x += step_x;
        passos++;
        na_ponta = false;
        break;
    }
    estado++;
    waypoints.push_back({Eigen::Vector3d(x, y, altitude), false});
  }

  return waypoints;
}

}  // namespace fase1
