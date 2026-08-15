/**
 * @file test_transformacao.cpp
 * @brief A ponte entre o mapa e a odometria.
 *
 * ESTE ARQUIVO EXISTE POR CAUSA DE UM DEFEITO ESPECIFICO.
 *
 * A primeira versao tratava a relacao entre o mapa e a odometria como uma
 * TRANSLACAO. Ela e uma transformacao RIGIDA: o `setHomePosition` reancora o
 * referencial na PROA de decolagem, e nesta fase o drone decola virado para o
 * leste -- 90 graus de diferenca entre os dois sistemas.
 *
 * Com os eixos rodados, a correcao de centralizacao ia para o eixo errado.
 * Afastava o drone do centro, o erro crescia, a correcao crescia junto, e o
 * drone saia voando para fora da planta. Nao e erro que se acumula: DIVERGE NO
 * PRIMEIRO COMODO.
 *
 * E so foi pego por uma simulacao de ponta a ponta, de cinquenta segundos,
 * depois de olhar um log. Estes testes rodam em microssegundos e falham
 * apontando para a linha.
 */

#include <cmath>

#include <gtest/gtest.h>

#include "fase4/nodes/contexto.hpp"

using fase4::Contexto;
using fase4::fixarTransformacao;
using fase4::paraMapa;
using fase4::paraOdometria;
using fase4::rodar;
using fase4::yawParaOdometria;

namespace {

constexpr double kGrau = M_PI / 180.0;

/// Um contexto so com a transformacao -- sem drone, sem ROS, sem blackboard.
struct Transformacao {
  Eigen::Vector2d vies{0.0, 0.0};
  double vies_yaw{0.0};
  Contexto ctx;

  Transformacao() {
    ctx.vies = &vies;
    ctx.vies_yaw = &vies_yaw;
  }

  /// A situacao logo apos a decolagem: a odometria le (0,0) e guinada 0 no
  /// ponto `inicio` do mapa, com o nariz apontando para `yaw_inicial`.
  void decolarEm(const Eigen::Vector2d& inicio, double yaw_inicial) {
    fixarTransformacao(ctx, inicio, Eigen::Vector2d::Zero(), yaw_inicial, 0.0);
  }
};

}  // namespace

// ── A rotacao ───────────────────────────────────────────────────────────────

TEST(Transformacao, rodarSegueOSentidoNED) {
  // NED: x norte, y leste, angulo positivo do norte para o leste.
  const Eigen::Vector2d norte(1.0, 0.0);
  const Eigen::Vector2d girado = rodar(norte, M_PI_2);
  EXPECT_NEAR(girado.x(), 0.0, 1e-9);
  EXPECT_NEAR(girado.y(), 1.0, 1e-9) << "girar o norte de +90 graus da o leste";
}

// ── O caso que quebrou ──────────────────────────────────────────────────────

/**
 * O DEFEITO, em numeros.
 *
 * O drone decola em (4.175, -0.60) do mapa, virado para o LESTE -- e a janela
 * de entrada esta 0,95 m a leste dele, em (4.175, 0.35).
 *
 * Como o nariz aponta para o alvo, o comando em coordenadas da odometria tem de
 * ser 0,95 m PARA A FRENTE, ou seja (0.95, 0).
 *
 * Tratando como translacao pura sairia (0, 0.95): 0,95 m para a DIREITA. Noventa
 * graus errado, na primeira manobra da missao.
 */
TEST(Transformacao, decolandoViradoParaOLesteOAlvoAFrenteVaiParaOEixoX) {
  Transformacao t;
  t.decolarEm({4.175, -0.60}, 90.0 * kGrau);

  const Eigen::Vector2d alvo = paraOdometria(t.ctx, {4.175, 0.35});

  EXPECT_NEAR(alvo.x(), 0.95, 1e-9) << "a frente do drone";
  EXPECT_NEAR(alvo.y(), 0.00, 1e-9)
      << "0.95 aqui seria a translacao pura: 90 graus errado";
}

/// E o mesmo ponto de volta ao mapa fecha o circulo.
TEST(Transformacao, idaEVoltaDevolveOMesmoPonto) {
  Transformacao t;
  t.decolarEm({4.175, -0.60}, 90.0 * kGrau);

  for (const Eigen::Vector2d p : {Eigen::Vector2d(4.175, 0.35),
                                  Eigen::Vector2d(1.75, 1.90),
                                  Eigen::Vector2d(-3.0, 7.25)}) {
    const Eigen::Vector2d volta = paraMapa(t.ctx, paraOdometria(t.ctx, p));
    EXPECT_NEAR(volta.x(), p.x(), 1e-9);
    EXPECT_NEAR(volta.y(), p.y(), 1e-9);
  }
}

/**
 * Decolando virado para o NORTE, os dois referenciais coincidem.
 *
 * E o caso em que a translacao pura acerta -- e por isso um teste feito so com
 * ele nao provaria nada. Fica aqui para deixar claro que o teste de cima
 * depende da proa, e nao de coincidencia.
 */
TEST(Transformacao, decolandoParaONorteAConversaoEUmaTranslacao) {
  Transformacao t;
  t.decolarEm({2.0, 3.0}, 0.0);

  const Eigen::Vector2d alvo = paraOdometria(t.ctx, {5.0, 3.0});
  EXPECT_NEAR(alvo.x(), 3.0, 1e-9);
  EXPECT_NEAR(alvo.y(), 0.0, 1e-9);
  EXPECT_NEAR(t.vies_yaw, 0.0, 1e-9);
}

// ── A guinada ───────────────────────────────────────────────────────────────

TEST(Transformacao, aGuinadaDoMapaVaiParaAOdometria) {
  Transformacao t;
  t.decolarEm({4.175, -0.60}, 90.0 * kGrau);

  // Apontar para o leste no mapa e apontar para a frente na odometria.
  EXPECT_NEAR(yawParaOdometria(t.ctx, 90.0 * kGrau), 0.0, 1e-9);
  // Apontar para o norte no mapa e virar 90 graus para a esquerda.
  EXPECT_NEAR(yawParaOdometria(t.ctx, 0.0), -90.0 * kGrau, 1e-9);
  // E para o sul, 90 para a direita.
  EXPECT_NEAR(yawParaOdometria(t.ctx, 180.0 * kGrau), 90.0 * kGrau, 1e-9);
}

TEST(Transformacao, aGuinadaEnvolvidaFicaEmMenosPiAPi) {
  Transformacao t;
  t.decolarEm({0.0, 0.0}, -170.0 * kGrau);

  const double y = yawParaOdometria(t.ctx, 170.0 * kGrau);
  EXPECT_LE(y, M_PI + 1e-9);
  EXPECT_GT(y, -M_PI - 1e-9);
  // 170 - (-170) = 340, que envolvido e -20.
  EXPECT_NEAR(y, -20.0 * kGrau, 1e-9);
}

// ── A correcao pelo LIDAR ───────────────────────────────────────────────────

/**
 * Redefinir a transformacao a partir de uma correspondencia conhecida.
 *
 * E o que `atualizarVies` faz a cada ciclo de centralizacao: o LIDAR diz onde o
 * drone esta NO MAPA, a odometria diz onde ela acha que ele esta, e as duas
 * juntas fixam a transformacao.
 */
TEST(Transformacao, oLidarRedefineATransformacaoInteira) {
  Transformacao t;
  t.decolarEm({4.175, -0.60}, 90.0 * kGrau);

  // A odometria derivou: ela le (3.0, 0.2) e guinada 0.1, mas o LIDAR diz que
  // o drone esta de fato em (2.725, 0.95) do mapa, apontando para o leste.
  const Eigen::Vector2d verdade(2.725, 0.95);
  const Eigen::Vector2d odom(3.0, 0.2);
  fixarTransformacao(t.ctx, verdade, odom, 90.0 * kGrau, 0.1);

  // Depois disso, converter a verdade tem de devolver exatamente a odometria.
  const Eigen::Vector2d ida = paraOdometria(t.ctx, verdade);
  EXPECT_NEAR(ida.x(), odom.x(), 1e-9);
  EXPECT_NEAR(ida.y(), odom.y(), 1e-9);

  // E a leitura da odometria tem de devolver a verdade.
  const Eigen::Vector2d volta = paraMapa(t.ctx, odom);
  EXPECT_NEAR(volta.x(), verdade.x(), 1e-9);
  EXPECT_NEAR(volta.y(), verdade.y(), 1e-9);

  EXPECT_NEAR(yawParaOdometria(t.ctx, 90.0 * kGrau), 0.1, 1e-9);
}

/**
 * Comandar o centro do comodo depois da correcao aproxima o drone dele.
 *
 * O teste que reproduz a divergencia em miniatura. Com a transformacao certa, o
 * comando fica a poucos centimetros da odometria atual -- e uma correcao. Com a
 * translacao pura ele salta para longe, e o passo seguinte salta mais.
 */
TEST(Transformacao, aCorrecaoEPequenaEnaoUmSalto) {
  Transformacao t;
  t.decolarEm({4.175, -0.60}, 90.0 * kGrau);

  const Eigen::Vector2d centro(4.175, 0.475);   // centro do comodo de entrada
  const Eigen::Vector2d odom_agora(0.90, 0.03);  // onde a odometria diz estar

  // O LIDAR diz que o drone esta 3 cm ao norte e 2 cm a leste do centro.
  fixarTransformacao(t.ctx, centro + Eigen::Vector2d(0.03, 0.02), odom_agora,
                     90.0 * kGrau, 0.0);

  const Eigen::Vector2d comando = paraOdometria(t.ctx, centro);
  const double salto = (comando - odom_agora).norm();

  EXPECT_LT(salto, 0.05)
      << "a correcao tem de ter o tamanho do erro medido (3.6 cm), e nao o "
         "tamanho do labirinto";
}
