#ifndef FASE4__NODES__SEGURANCA_HPP_
#define FASE4__NODES__SEGURANCA_HPP_

// A guarda de proximidade, e o cuidado que ela exige NESTE labirinto.
//
// A ideia é a de sempre: num `ReactiveSequence` o primeiro filho é reavaliado a
// cada tick, então uma condição de segurança ali aborta na hora o que estiver
// em curso. Numa FSM isso seria uma checagem repetida em todo estado, e bastaria
// esquecê-la em um deles.
//
// SÓ QUE AQUI A PAREDE ESTÁ SEMPRE PERTO.
//
// Num cômodo de 0,95 m, um drone de 0,40 m PERFEITAMENTE centrado tem 27 cm até
// cada parede. Atravessando um vão de 0,60 m, tem 10 cm até cada batente. Uma
// guarda com limite "razoável" -- 30 cm, 25 cm -- dispararia o tempo todo, e o
// primeiro reflexo de quem visse isso seria desligá-la.
//
// Por isso o limite padrão é 12 cm: abaixo do batente da janela, e portanto
// perto do impossível sem contato. Esta guarda NÃO é a que mantém o drone longe
// das paredes -- disso cuidam a centralização e o alinhamento. Ela é o último
// recurso, para quando as duas falharam.
//
// E ela ignora um SETOR À FRENTE, porque atravessar uma janela é, por
// construção, aproximar-se de paredes de propósito.

#include <cmath>
#include <string>

#include "fase4/nodes/contexto.hpp"

namespace fase4 {

/**
 * @brief SUCCESS quando alguma parede está perto demais.
 *
 * Feita para ser usada invertida, como primeira condição de um
 * `ReactiveSequence`.
 */
class ParedePerigosamentePerto : public BT::ConditionNode {
 public:
  ParedePerigosamentePerto(const std::string& nome, const BT::NodeConfig& config)
      : BT::ConditionNode(nome, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("limite", 0.12,
                              "metros; abaixo disto a guarda dispara"),
        BT::InputPort<double>("setor_livre", 0.5,
                              "radianos à frente que a guarda ignora, porque "
                              "atravessar uma janela é aproximar-se de "
                              "propósito"),
    };
  }

  BT::NodeStatus tick() override {
    const Contexto ctx = obterContexto(*this);
    // Sem contexto ou sem scan, a guarda NÃO dispara.
    //
    // É deliberado, e é o oposto do reflexo usual de "na dúvida, aborte". Aqui
    // a guarda é o último recurso, não a proteção principal: fazê-la disparar
    // por falta de dado transformaria um LIDAR mudo por meio segundo numa
    // missão abortada, e o drone ficaria parado no meio de um corredor -- que
    // não é mais seguro que continuar.
    if (!ctx.completo() || !ctx.scan->recebido) return BT::NodeStatus::FAILURE;

    double limite = 0.12, setor = 0.5;
    getInput("limite", limite);
    getInput("setor_livre", setor);

    const auto& s = *ctx.scan;
    for (std::size_t i = 0; i < s.alcances.size(); ++i) {
      const double d = static_cast<double>(s.alcances[i]);
      if (!std::isfinite(d) || d <= 0.0) continue;
      if (d >= limite) continue;

      const double ang = normalizar(s.ang_min + s.inc * static_cast<double>(i));
      if (std::abs(ang) < setor) continue;   // à frente, é a janela

      ctx.drone->log("PAREDE A " + std::to_string(d) + " m, a " +
                     std::to_string(ang * 180.0 / M_PI) + " graus.");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace fase4

#endif  // FASE4__NODES__SEGURANCA_HPP_
