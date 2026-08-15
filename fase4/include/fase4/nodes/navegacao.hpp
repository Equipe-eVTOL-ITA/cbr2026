#ifndef FASE4__NODES__NAVEGACAO_HPP_
#define FASE4__NODES__NAVEGACAO_HPP_

// Os nós que movem o drone pelo labirinto.
//
// O CICLO DE UM CÔMODO, e por que ele tem três partes
//
//   CentralizarNoComodo   olha o LIDAR, corrige o VIÉS da odometria, e vai
//                         para o centro. É a única parte em que a localização
//                         é refeita.
//   AlinharComAJanela     desliza até o alinhamento lateral do vão e vira de
//                         frente para ele. Ainda dá para errar aqui.
//   AtravessarJanela      avança em MALHA ABERTA. Aqui já não dá.
//
// A separação existe por causa da última linha. Dentro do vão o LIDAR vê duas
// paredes muito próximas e nenhuma geometria de cômodo: qualquer correção ali
// usa uma fórmula que supõe um cômodo e recebe outra coisa. A resposta certa é
// SUSPENDER a correção, o que só é seguro se o alinhamento já estiver feito --
// daí o passo do meio ser um passo separado, e não o começo da travessia.

#include <algorithm>
#include <memory>
#include <string>

#include "fase4/nodes/contexto.hpp"

namespace fase4 {

/// Parâmetros do ajuste de scan, lidos da blackboard.
inline maze_geometry::ParamsAjuste paramsDoAjuste(fsm::Blackboard* bb) {
  maze_geometry::ParamsAjuste p;
  p.alcance_max = param(bb, "lidar_alcance_max", 8.0f);
  p.salto_max = param(bb, "scan_salto_max", 0.15f);
  p.tolerancia = param(bb, "scan_tolerancia", 0.03f);
  p.comprimento_min = param(bb, "scan_parede_min", 0.20f);
  return p;
}

/**
 * @brief Entra no labirinto pela janela de entrada.
 *
 * Dois trechos, ambos axiais: primeiro até o ponto alinhado com o vão, ainda do
 * lado de fora; depois para dentro. O primeiro trecho existe para o caso de a
 * decolagem não deixar o drone exatamente na frente da janela -- e a decolagem
 * nunca deixa.
 *
 * NÃO usa o LIDAR. Do lado de fora não há cômodo conhecido para casar com o
 * scan, e o viés inicial vem da posição de decolagem, que é medida e não
 * estimada.
 */
class EntrarNaCasa : public NoDaFase {
 public:
  using NoDaFase::NoDaFase;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    if (!resolverContexto()) return BT::NodeStatus::FAILURE;

    const auto entrada = maze_geometry::entradaDoLabirinto(*ctx_.casa);
    const auto& c = ctx_.casa->porId(entrada.comodo);
    const double recuo = param(ctx_.bb, "recuo_da_janela", 0.35f);

    // O rumo aponta para DENTRO: `rumo(entrada.saida)` apontaria para fora,
    // que é de onde viemos.
    yaw_ = maze_geometry::rumo(maze_geometry::oposta(entrada.saida));
    fora_ = maze_geometry::transposicao(c, entrada.saida, recuo);
    dentro_ = maze_geometry::aproximacao(c, entrada.saida, recuo);
    z_ = alturaDeVoo();

    ctx_.drone->log("");
    ctx_.drone->log("ENTRANDO no labirinto pelo " +
                    maze_geometry::paraTexto(entrada.saida) + " de '" + c.nome + "'");

    etapa_ = 0;
    mov_.iniciar(paraOdometria(ctx_, fora_), yawParaOdometria(ctx_, yaw_), z_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const double tol = param(ctx_.bb, "tolerancia_movimento", 0.06f);
    const double tol_yaw = param(ctx_.bb, "yaw_tolerance", 0.05f);

    if (!mov_.passo(ctx_.drone, tol, tol_yaw)) return BT::NodeStatus::RUNNING;

    if (etapa_ == 0) {
      etapa_ = 1;
      mov_.iniciar(paraOdometria(ctx_, dentro_), yawParaOdometria(ctx_, yaw_), z_);
      return BT::NodeStatus::RUNNING;
    }
    ctx_.drone->log("Dentro do labirinto.");
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int etapa_{0};
  double yaw_{0.0}, z_{-1.0};
  Eigen::Vector2d fora_, dentro_;
};

/**
 * @brief Refaz a localização pelo LIDAR e vai para o centro do cômodo.
 *
 * O único nó que corrige o viés da odometria -- e, portanto, o único que faz o
 * LIDAR valer alguma coisa. Ir para o centro não é vaidade: é o ponto de maior
 * folga para todas as paredes antes de manobrar, e num cômodo de 0,95 m com um
 * drone de 0,40 m a folga é de 27 cm.
 *
 * SE O SCAN NÃO CASAR COM O CÔMODO, o nó fica RUNNING em vez de desistir. A
 * política de desistência mora na árvore, num `<Timeout>`, e não aqui: assim
 * dá para mudá-la sem recompilar, que é metade da razão de existir uma BT.
 */
class CentralizarNoComodo : public NoDaFase {
 public:
  using NoDaFase::NoDaFase;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    if (!resolverContexto()) return BT::NodeStatus::FAILURE;
    const auto* p = passoAtual();
    if (p == nullptr) return BT::NodeStatus::FAILURE;

    const auto& c = ctx_.casa->porId(p->comodo);
    ctx_.drone->log("");
    ctx_.drone->log("CENTRALIZANDO em '" + c.nome + "'");
    estaveis_ = 0;
    z_ = alturaDeVoo();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const auto* p = passoAtual();
    if (p == nullptr) return BT::NodeStatus::FAILURE;
    const auto& c = ctx_.casa->porId(p->comodo);

    const auto ajuste = atualizarVies(ctx_, c, paramsDoAjuste(ctx_.bb));
    if (!ajuste.valido) {
      estaveis_ = 0;
      MovimentoAxial::parar(ctx_.drone);
      return BT::NodeStatus::RUNNING;
    }

    // Já com o viés novo: o centro do cômodo em coordenadas da odometria, e o
    // rumo da saída também. Comandar posição e guinada juntos aqui é seguro
    // porque a correção é pequena -- não é um deslocamento, é um ajuste.
    const Eigen::Vector2d alvo = paraOdometria(ctx_, c.centro());
    const double yaw = yawParaOdometria(ctx_, maze_geometry::rumo(p->saida));
    ctx_.drone->setLocalPosition(static_cast<float>(alvo.x()),
                                 static_cast<float>(alvo.y()),
                                 static_cast<float>(z_),
                                 static_cast<float>(yaw));

    const double erro_pos = (posicaoNoMapa(ctx_) - c.centro()).norm();
    const double erro_yaw = std::abs(normalizar(
        ctx_.drone->getOrientation()[2] - yaw));

    const double tol = param(ctx_.bb, "tolerancia_centro", 0.08f);
    const double tol_yaw = param(ctx_.bb, "yaw_tolerance", 0.05f);

    // Estabilidade por vários ciclos: um único ciclo dentro da tolerância pode
    // ser o drone passando de largo.
    if (erro_pos < tol && erro_yaw < tol_yaw) {
      ++estaveis_;
    } else {
      estaveis_ = 0;
    }

    if (estaveis_ < static_cast<int>(param(ctx_.bb, "ciclos_estaveis", 5.0f))) {
      return BT::NodeStatus::RUNNING;
    }
    ctx_.drone->log("Centralizado. Viés da odometria: (" +
                    std::to_string(ctx_.vies->x()) + ", " +
                    std::to_string(ctx_.vies->y()) + ")");
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int estaveis_{0};
  double z_{-1.0};
};

/**
 * @brief Desliza até o alinhamento lateral do vão e vira de frente para ele.
 *
 * DOIS TRECHOS AXIAIS, e nunca uma diagonal:
 *
 *   1. gira para o eixo TANGENTE à parede de saída e avança até a coordenada
 *      lateral da janela;
 *   2. gira para o rumo da saída e avança até o ponto de aproximação.
 *
 * É a decomposição que o edital pede -- frente e quartos de volta -- e é
 * também a que mantém o drone longe das quinas: uma diagonal dentro de um
 * cômodo de 0,95 m passa mais perto do canto do que qualquer dos dois trechos.
 *
 * O LIDAR continua corrigindo o viés durante o trecho 1, mas os alvos são
 * calculados uma vez, no início. Recalcular o destino a cada ciclo com um viés
 * que muda faria o alvo perseguir o drone.
 */
class AlinharComAJanela : public NoDaFase {
 public:
  using NoDaFase::NoDaFase;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    if (!resolverContexto()) return BT::NodeStatus::FAILURE;
    const auto* p = passoAtual();
    if (p == nullptr) return BT::NodeStatus::FAILURE;

    const auto& c = ctx_.casa->porId(p->comodo);
    const double recuo = param(ctx_.bb, "recuo_da_janela", 0.35f);
    const Eigen::Vector2d janela = maze_geometry::pontoDaJanela(c, p->saida);
    const Eigen::Vector2d aqui = posicaoNoMapa(ctx_);

    saida_ = p->saida;
    aproximacao_ = maze_geometry::aproximacao(c, p->saida, recuo);
    z_ = alturaDeVoo();

    // O eixo tangente à parede de saída é o outro eixo. Para as paredes norte e
    // sul a normal é x, então a tangente é y -- e vice-versa.
    const bool normal_em_x = (p->saida == maze_geometry::Parede::Norte ||
                              p->saida == maze_geometry::Parede::Sul);
    if (normal_em_x) {
      lateral_ = Eigen::Vector2d(aqui.x(), janela.y());
      yaw_lateral_ = maze_geometry::rumo(janela.y() > aqui.y()
                                             ? maze_geometry::Parede::Leste
                                             : maze_geometry::Parede::Oeste);
    } else {
      lateral_ = Eigen::Vector2d(janela.x(), aqui.y());
      yaw_lateral_ = maze_geometry::rumo(janela.x() > aqui.x()
                                             ? maze_geometry::Parede::Norte
                                             : maze_geometry::Parede::Sul);
    }

    ctx_.drone->log("");
    ctx_.drone->log("ALINHANDO com a janela ao " +
                    maze_geometry::paraTexto(p->saida) + " de '" + c.nome + "'");

    etapa_ = 0;
    mov_.iniciar(paraOdometria(ctx_, lateral_),
                 yawParaOdometria(ctx_, yaw_lateral_), z_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const auto* p = passoAtual();
    if (p == nullptr) return BT::NodeStatus::FAILURE;

    const double tol = param(ctx_.bb, "tolerancia_movimento", 0.06f);
    const double tol_yaw = param(ctx_.bb, "yaw_tolerance", 0.05f);

    if (!mov_.passo(ctx_.drone, tol, tol_yaw)) return BT::NodeStatus::RUNNING;

    if (etapa_ == 0) {
      etapa_ = 1;
      mov_.iniciar(paraOdometria(ctx_, aproximacao_),
                   yawParaOdometria(ctx_, maze_geometry::rumo(saida_)), z_);
      return BT::NodeStatus::RUNNING;
    }
    ctx_.drone->log("Alinhado, de frente para o vão.");
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int etapa_{0};
  maze_geometry::Parede saida_{maze_geometry::Parede::Norte};
  Eigen::Vector2d lateral_, aproximacao_;
  double yaw_lateral_{0.0}, z_{-1.0};
};

/**
 * @brief Atravessa o vão em malha aberta, e avança o passo da rota.
 *
 * NÃO corrige nada, de propósito. Dentro da janela o LIDAR vê duas paredes
 * muito próximas e nenhuma geometria de cômodo; a fórmula da centralização
 * supõe um cômodo e devolveria um número plausível e errado, na hora em que um
 * número errado custa mais caro. É por isso que o alinhamento é um passo
 * anterior e separado.
 *
 * O deslocamento é curto -- do ponto de aproximação ao ponto logo depois do vão
 * -- e a odometria não deriva o bastante nessa distância para importar.
 */
class AtravessarJanela : public NoDaFase {
 public:
  using NoDaFase::NoDaFase;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    if (!resolverContexto()) return BT::NodeStatus::FAILURE;
    const auto* p = passoAtual();
    if (p == nullptr) return BT::NodeStatus::FAILURE;

    const auto& c = ctx_.casa->porId(p->comodo);
    const double avanco = param(ctx_.bb, "avanco_apos_janela", 0.35f);
    const Eigen::Vector2d destino = maze_geometry::transposicao(c, p->saida, avanco);

    ctx_.drone->log("");
    ctx_.drone->log("ATRAVESSANDO (malha aberta) a janela ao " +
                    maze_geometry::paraTexto(p->saida) + " de '" + c.nome + "'");

    mov_.iniciar(paraOdometria(ctx_, destino),
                 yawParaOdometria(ctx_, maze_geometry::rumo(p->saida)),
                 alturaDeVoo());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const double tol = param(ctx_.bb, "tolerancia_movimento", 0.06f);
    const double tol_yaw = param(ctx_.bb, "yaw_tolerance", 0.05f);
    if (!mov_.passo(ctx_.drone, tol, tol_yaw)) return BT::NodeStatus::RUNNING;

    ++(*ctx_.passo);
    ctx_.drone->log("Atravessou. Passo " + std::to_string(*ctx_.passo) + " de " +
                    std::to_string(ctx_.casa->rota.size()) + ".");
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief Vai até a plataforma de pouso, em dois trechos axiais.
 *
 * SEM PLATAFORMA DECLARADA, ele diz isso no log e devolve SUCCESS.
 *
 * Não é descuido: a posição da plataforma não é um número que se possa supor --
 * errá-la é pousar fora dela. Enquanto a coordenada não vier do edital, a
 * missão sai do labirinto e pousa onde está, que é o que "atravessar a casa"
 * exige, e o log deixa claro que a última etapa não foi feita.
 *
 * O silêncio é que seria descuido. Um nó que sucede sem fazer nada e sem dizer
 * nada vira uma etapa que todo mundo acha que existe.
 */
class IrParaPlataforma : public NoDaFase {
 public:
  using NoDaFase::NoDaFase;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    if (!resolverContexto()) return BT::NodeStatus::FAILURE;

    if (!ctx_.casa->tem_plataforma) {
      ctx_.drone->log("");
      ctx_.drone->log("SEM PLATAFORMA declarada no mapa: pousando onde estou. "
                      "Acrescente 'plataforma: {x: ..., y: ...}' ao YAML.");
      return BT::NodeStatus::SUCCESS;
    }

    const Eigen::Vector2d alvo = ctx_.casa->plataforma;
    const Eigen::Vector2d aqui = posicaoNoMapa(ctx_);
    z_ = alturaDeVoo();

    // Dois trechos axiais, como o resto da fase: primeiro o eixo de maior
    // deslocamento, depois o outro. Nunca uma diagonal.
    const double dx = alvo.x() - aqui.x();
    const double dy = alvo.y() - aqui.y();
    if (std::abs(dx) >= std::abs(dy)) {
      meio_ = Eigen::Vector2d(alvo.x(), aqui.y());
      yaw_meio_ = maze_geometry::rumo(dx > 0 ? maze_geometry::Parede::Norte
                                             : maze_geometry::Parede::Sul);
      yaw_fim_ = maze_geometry::rumo(dy > 0 ? maze_geometry::Parede::Leste
                                            : maze_geometry::Parede::Oeste);
    } else {
      meio_ = Eigen::Vector2d(aqui.x(), alvo.y());
      yaw_meio_ = maze_geometry::rumo(dy > 0 ? maze_geometry::Parede::Leste
                                             : maze_geometry::Parede::Oeste);
      yaw_fim_ = maze_geometry::rumo(dx > 0 ? maze_geometry::Parede::Norte
                                            : maze_geometry::Parede::Sul);
    }
    fim_ = alvo;

    ctx_.drone->log("");
    ctx_.drone->log("INDO para a plataforma em (" + std::to_string(alvo.x()) +
                    ", " + std::to_string(alvo.y()) + ")");
    etapa_ = 0;
    mov_.iniciar(paraOdometria(ctx_, meio_), yawParaOdometria(ctx_, yaw_meio_), z_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const double tol = param(ctx_.bb, "tolerancia_movimento", 0.06f);
    const double tol_yaw = param(ctx_.bb, "yaw_tolerance", 0.05f);
    if (!mov_.passo(ctx_.drone, tol, tol_yaw)) return BT::NodeStatus::RUNNING;

    if (etapa_ == 0) {
      etapa_ = 1;
      mov_.iniciar(paraOdometria(ctx_, fim_), yawParaOdometria(ctx_, yaw_fim_), z_);
      return BT::NodeStatus::RUNNING;
    }
    ctx_.drone->log("Sobre a plataforma.");
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int etapa_{0};
  Eigen::Vector2d meio_, fim_;
  double yaw_meio_{0.0}, yaw_fim_{0.0}, z_{-1.0};
};

// ── Condições da rota ───────────────────────────────────────────────────────

/// SUCCESS enquanto houver passo a executar.
class AindaHaPassos : public BT::ConditionNode {
 public:
  AindaHaPassos(const std::string& nome, const BT::NodeConfig& config)
      : BT::ConditionNode(nome, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    const Contexto c = obterContexto(*this);
    if (!c.completo()) return BT::NodeStatus::FAILURE;
    return (static_cast<std::size_t>(*c.passo) < c.casa->rota.size())
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief SUCCESS só se a rota foi percorrida até o fim.
 *
 * Vai depois do laço. O `KeepRunningUntilFailure` devolve SUCCESS tanto quando
 * a rota acaba quanto quando algo falha no meio -- e sem esta checagem uma
 * missão abortada na terceira janela seguiria para o pouso como se tivesse
 * dado tudo certo.
 */
class RotaCompleta : public BT::ConditionNode {
 public:
  RotaCompleta(const std::string& nome, const BT::NodeConfig& config)
      : BT::ConditionNode(nome, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    const Contexto c = obterContexto(*this);
    if (!c.completo()) return BT::NodeStatus::FAILURE;
    if (static_cast<std::size_t>(*c.passo) >= c.casa->rota.size()) {
      return BT::NodeStatus::SUCCESS;
    }
    c.drone->log("A rota parou no passo " + std::to_string(*c.passo) + " de " +
                 std::to_string(c.casa->rota.size()) + " -- missão incompleta.");
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace fase4

#endif  // FASE4__NODES__NAVEGACAO_HPP_
