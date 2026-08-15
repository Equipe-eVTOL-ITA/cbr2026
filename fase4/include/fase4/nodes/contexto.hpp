#ifndef FASE4__NODES__CONTEXTO_HPP_
#define FASE4__NODES__CONTEXTO_HPP_

// O que todo nó da fase 4 precisa, e o primitivo de movimento que todos usam.
//
// SEM PidController, POR DECISÃO
//
// Os comandos de posição vão direto pela classe `Drone`, e quem fecha a malha é
// o Pixhawk. Um PID nosso por cima do controlador do PX4 seria uma segunda
// malha em cascata com a primeira, com ganhos que ninguém sintonizou, dentro de
// um cômodo de 0,95 m -- e a instabilidade apareceria justamente onde não há
// espaço para ela.
//
// FRENTE E 90 GRAUS, E NADA MAIS
//
// Todo deslocamento passa pelo `MovimentoAxial`, que separa GIRAR de AVANÇAR em
// duas fases. Não é preciosismo: girar e transladar ao mesmo tempo varre uma
// pegada maior que a do drone parado, e num corredor de 0,95 m com um drone de
// 0,40 m a diferença entre varrer e não varrer é a diferença entre passar e
// bater.
//
// Como o comando é de POSIÇÃO e não de velocidade, a fase de girar comanda a
// posição atual com a guinada nova -- o drone gira parado -- e só então a fase
// de avançar muda a posição.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include "stdbt/fsm_action_node.hpp"   // kFsmBlackboardKey

#include "maze_geometry/casa.hpp"
#include "maze_geometry/scan_fit.hpp"

namespace fase4 {

/// O último `LaserScan`, copiado para fora do ROS.
///
/// O callback do `/scan` escreve aqui e os nós apenas leem -- o mesmo padrão
/// que as fases de visão usam para as detecções. Os nós não conhecem ROS.
struct ScanRecebido {
  std::vector<float> alcances;
  double ang_min{0.0};
  double inc{0.0};
  double idade{1e9};    ///< segundos desde a última mensagem
  bool recebido{false};
};

/// Tudo o que um nó desta fase precisa achar na blackboard.
struct Contexto {
  std::shared_ptr<Drone> drone;
  fsm::Blackboard* bb{nullptr};
  maze_geometry::Casa* casa{nullptr};
  ScanRecebido* scan{nullptr};
  int* passo{nullptr};        ///< índice do passo atual da rota

  /// A TRANSFORMAÇÃO DO MAPA PARA A ODOMETRIA.
  ///
  ///     p_odom = R(vies_yaw) * p_mapa + vies
  ///     yaw_odom = yaw_mapa + vies_yaw
  ///
  /// É a peça central desta fase. O mapa está em coordenadas da arena; o
  /// `Drone` só aceita comandos em coordenadas da odometria; e as duas se
  /// afastam sozinhas, porque não há GPS. Mantê-la atualizada a partir do LIDAR
  /// é literalmente todo o trabalho de localização desta fase.
  ///
  /// É UMA ROTAÇÃO MAIS UMA TRANSLAÇÃO, e não só uma translação.
  ///
  /// O `setHomePosition` do Takeoff reancora o referencial na PROA de
  /// decolagem: o eixo x da odometria aponta para onde o nariz do drone estava,
  /// e não para o norte do mapa. Nesta fase o drone decola virado para o leste,
  /// então os dois referenciais estão a 90 graus um do outro.
  ///
  /// Tratar isso como translação pura -- que foi o que tentei primeiro --
  /// manda a correção de centralização para o eixo errado. Ela afasta o drone
  /// do centro, o erro cresce, a correção cresce junto, e o drone sai voando
  /// para fora da planta. Não é um erro pequeno que se acumula: diverge no
  /// primeiro cômodo.
  Eigen::Vector2d* vies{nullptr};
  double* vies_yaw{nullptr};

  bool completo() const {
    return drone != nullptr && bb != nullptr && casa != nullptr &&
           scan != nullptr && passo != nullptr && vies != nullptr &&
           vies_yaw != nullptr;
  }
};

inline Contexto obterContexto(const BT::TreeNode& no) {
  Contexto c;
  fsm::Blackboard* bb = nullptr;
  if (auto arvore_bb = no.config().blackboard) {
    // O retorno é `nodiscard`, e ignorá-lo dá aviso com -Wall. Aqui ele não
    // acrescenta nada: a chave ausente deixa `bb` nulo, que é a mesma
    // condição, testada logo abaixo. Fica explícito que foi decisão.
    (void)arvore_bb->get<fsm::Blackboard*>(stdbt::kFsmBlackboardKey, bb);
  }
  if (bb == nullptr) return c;

  c.bb = bb;
  if (auto* d = bb->get<std::shared_ptr<Drone>>("drone")) c.drone = *d;
  c.casa = bb->get<maze_geometry::Casa>("casa");
  c.scan = bb->get<ScanRecebido>("scan");
  c.passo = bb->get<int>("passo_da_rota");
  c.vies = bb->get<Eigen::Vector2d>("vies_odometria");
  c.vies_yaw = bb->get<double>("vies_yaw");
  return c;
}

inline double normalizar(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// ── A ponte entre o mapa e a odometria ──────────────────────────────────────

/// Roda um vetor 2D. Convenção NED: x norte, y leste, ângulo positivo do norte
/// para o leste.
inline Eigen::Vector2d rodar(const Eigen::Vector2d& v, double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {v.x() * c - v.y() * s, v.x() * s + v.y() * c};
}

/// Um ponto do MAPA no referencial em que o `Drone` aceita comandos.
inline Eigen::Vector2d paraOdometria(const Contexto& ctx,
                                     const Eigen::Vector2d& no_mapa) {
  return rodar(no_mapa, *ctx.vies_yaw) + *ctx.vies;
}

/// Uma guinada do MAPA no referencial da odometria.
inline double yawParaOdometria(const Contexto& ctx, double yaw_no_mapa) {
  return normalizar(yaw_no_mapa + *ctx.vies_yaw);
}

/// Um ponto da ODOMETRIA em coordenadas do mapa. Inverso de `paraOdometria`.
inline Eigen::Vector2d paraMapa(const Contexto& ctx,
                                const Eigen::Vector2d& na_odometria) {
  return rodar(na_odometria - *ctx.vies, -*ctx.vies_yaw);
}

/// Onde o drone realmente está, em coordenadas do MAPA.
inline Eigen::Vector2d posicaoNoMapa(const Contexto& ctx) {
  const Eigen::Vector3d p = ctx.drone->getLocalPosition();
  return paraMapa(ctx, Eigen::Vector2d(p.x(), p.y()));
}

/// A transformação que leva o mapa à odometria, dada uma correspondência.
///
/// Fixado `vies_yaw`, a translação sai de uma única correspondência conhecida
/// entre um ponto do mapa e o mesmo ponto na odometria.
inline void fixarTransformacao(const Contexto& ctx,
                               const Eigen::Vector2d& no_mapa,
                               const Eigen::Vector2d& na_odometria,
                               double yaw_mapa, double yaw_odometria) {
  *ctx.vies_yaw = normalizar(yaw_odometria - yaw_mapa);
  *ctx.vies = na_odometria - rodar(no_mapa, *ctx.vies_yaw);
}

/**
 * @brief Recalcula o viés a partir do scan, e devolve o ajuste usado.
 *
 * É aqui que o LIDAR ganha o seu sustento. Sem esta chamada, a fase inteira
 * seria malha aberta -- e a bateria do `sim2d` mostrou que a malha aberta
 * quebra entre 1 e 2 cm de deriva por metro percorrido, com colisão na quarta
 * travessia.
 *
 * Cada eixo é corrigido por si. Num corredor estreito é comum enxergar as duas
 * laterais e nenhuma das pontas: aceitar só o eixo transversal é melhor que
 * recusar os dois, e muito melhor que aceitar um número inventado para o
 * longitudinal.
 *
 * Quando o ajuste não vale -- dentro do vão de uma janela, tipicamente -- o
 * viés fica como estava. Isso é deliberado: continuar com o último viés bom é
 * exatamente o que "atravessar em malha aberta" significa.
 */
inline maze_geometry::Ajuste atualizarVies(
    const Contexto& ctx, const maze_geometry::Comodo& comodo,
    const maze_geometry::ParamsAjuste& params, double idade_max = 0.5) {
  maze_geometry::Ajuste a;
  if (!ctx.scan->recebido || ctx.scan->idade > idade_max) {
    a.motivo = "sem scan recente";
    return a;
  }

  const double yaw_odom = ctx.drone->getOrientation()[2];
  // O palpite de guinada verdadeira vem da odometria menos o viés atual. É o
  // que desfaz a ambiguidade de 90 graus do `ajustar` -- ver scan_fit.hpp.
  const double yaw_chute = normalizar(yaw_odom - *ctx.vies_yaw);

  a = maze_geometry::ajustar(ctx.scan->alcances, ctx.scan->ang_min,
                             ctx.scan->inc, yaw_chute, comodo, params);
  if (!a.valido) return a;

  // A posição verdadeira segundo o LIDAR. Os eixos que o ajuste não conseguiu
  // resolver ficam com o que a transformação atual já dizia -- assim um
  // corredor que só mostra as laterais corrige a lateral e não estraga a
  // longitudinal.
  Eigen::Vector2d verdade = posicaoNoMapa(ctx);
  if (a.desvio_x_valido) verdade.x() = comodo.centro().x() + a.desvio.x();
  if (a.desvio_y_valido) verdade.y() = comodo.centro().y() + a.desvio.y();

  const Eigen::Vector3d p_odom = ctx.drone->getLocalPosition();
  fixarTransformacao(ctx, verdade, Eigen::Vector2d(p_odom.x(), p_odom.y()),
                     a.guinada, yaw_odom);
  return a;
}

/// Lê um float da blackboard, com padrão quando a chave não existe.
inline float param(fsm::Blackboard* bb, const std::string& chave, float padrao) {
  if (bb == nullptr) return padrao;
  float* v = bb->get<float>(chave);
  return v == nullptr ? padrao : *v;
}

/**
 * @brief Girar e depois avançar, nunca ao mesmo tempo.
 *
 * Usado por todos os nós que movem o drone. Mantém a restrição do edital --
 * frente e quartos de volta -- como ESTRUTURA, e não como disciplina de quem
 * escreve cada nó.
 *
 * Os alvos são dados no referencial da ODOMETRIA, que é o único em que o
 * `Drone` aceita comandos. Converter da verdade para a odometria é
 * responsabilidade de quem chama -- ver `CentralizarNoComodo`.
 */
class MovimentoAxial {
 public:
  void iniciar(const Eigen::Vector2d& destino_odom, double yaw_odom_alvo,
               double z_alvo) {
    destino_ = destino_odom;
    yaw_alvo_ = yaw_odom_alvo;
    z_ = z_alvo;
    fase_ = Fase::Girando;
  }

  bool terminou() const { return fase_ == Fase::Pronto; }

  /// Um passo. Devolve `true` quando o movimento acabou.
  bool passo(const std::shared_ptr<Drone>& drone, double tol_pos,
             double tol_yaw) {
    const Eigen::Vector3d p = drone->getLocalPosition();
    const double yaw = drone->getOrientation()[2];

    if (fase_ == Fase::Girando) {
      // Comanda a POSIÇÃO ATUAL com a guinada nova: o drone gira parado.
      drone->setLocalPosition(static_cast<float>(p.x()), static_cast<float>(p.y()),
                              static_cast<float>(z_), static_cast<float>(yaw_alvo_));
      if (std::abs(normalizar(yaw_alvo_ - yaw)) < tol_yaw) {
        fase_ = Fase::Avancando;
      }
      return false;
    }

    if (fase_ == Fase::Avancando) {
      drone->setLocalPosition(static_cast<float>(destino_.x()),
                              static_cast<float>(destino_.y()),
                              static_cast<float>(z_),
                              static_cast<float>(yaw_alvo_));
      const double resta =
          std::hypot(destino_.x() - p.x(), destino_.y() - p.y());
      if (resta < tol_pos) fase_ = Fase::Pronto;
      return fase_ == Fase::Pronto;
    }
    return true;
  }

  /// Congela onde está. É o que o `onHalted` de cada nó chama.
  static void parar(const std::shared_ptr<Drone>& drone) {
    if (drone == nullptr) return;
    const Eigen::Vector3d p = drone->getLocalPosition();
    const double yaw = drone->getOrientation()[2];
    drone->setLocalPosition(static_cast<float>(p.x()), static_cast<float>(p.y()),
                            static_cast<float>(p.z()), static_cast<float>(yaw));
  }

 private:
  enum class Fase { Girando, Avancando, Pronto };
  Fase fase_{Fase::Pronto};
  Eigen::Vector2d destino_{0.0, 0.0};
  double yaw_alvo_{0.0};
  double z_{0.0};
};

/**
 * @brief Base dos nós de ação desta fase.
 *
 * Resolve o contexto no `onStart` e implementa o `onHalted` de uma vez. O
 * `onHalted` é a armadilha dos `StatefulActionNode`: a árvore pode interromper
 * um nó a qualquer tick -- um `Timeout`, ou a condição de um `ReactiveSequence`
 * que mudou -- e um nó interrompido no meio de um deslocamento deixaria o drone
 * indo. Aqui não dá para esquecer.
 */
class NoDaFase : public BT::StatefulActionNode {
 public:
  NoDaFase(const std::string& nome, const BT::NodeConfig& config)
      : BT::StatefulActionNode(nome, config) {}

  void onHalted() override { MovimentoAxial::parar(ctx_.drone); }

 protected:
  bool resolverContexto() {
    ctx_ = obterContexto(*this);
    return ctx_.completo();
  }

  /// O passo da rota que está sendo executado, ou nulo se a rota acabou.
  const maze_geometry::Passo* passoAtual() const {
    if (ctx_.passo == nullptr || ctx_.casa == nullptr) return nullptr;
    const auto i = static_cast<std::size_t>(*ctx_.passo);
    if (i >= ctx_.casa->rota.size()) return nullptr;
    return &ctx_.casa->rota[i];
  }

  double alturaDeVoo() const {
    return ctx_.drone == nullptr ? -1.0 : ctx_.drone->getLocalPosition().z();
  }

  Contexto ctx_;
  MovimentoAxial mov_;
};

}  // namespace fase4

#endif  // FASE4__NODES__CONTEXTO_HPP_
