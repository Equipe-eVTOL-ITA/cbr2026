// Testes da geração da grade de varredura.
//
// Em 2025 este laço vivia solto dentro do construtor da FSM, o que só permitia
// conferi-lo subindo a simulação e olhando o drone voar. Como função pura, cada
// caso é aritmética.
//
// A grade é o que decide se a arena inteira é coberta. Errar aqui não gera erro
// nenhum: o drone voa bonito e simplesmente não passa por cima de uma das bases.

#include <vector>

#include <gtest/gtest.h>

#include "fase1/grid.hpp"

using fase1::makeSerpentineGrid;

TEST(Grid, ComecaCruzandoParaOLadoOposto)
{
  const auto pts = makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  // O primeiro movimento é atravessar em y, sem avançar em x: varrer a faixa
  // onde o drone já está antes de seguir para a próxima.
  EXPECT_DOUBLE_EQ(pts[0].coordinates.x(), 0.0);
  EXPECT_DOUBLE_EQ(pts[0].coordinates.y(), -6.0);
}

TEST(Grid, TodosNaAltitudePedida)
{
  const auto pts = makeSerpentineGrid(1.0, -1.0, -6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  for (const auto & p : pts) {
    EXPECT_DOUBLE_EQ(p.coordinates.z(), -2.5);
  }
}

TEST(Grid, TodosNascemNaoVisitados)
{
  const auto pts = makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  for (const auto & p : pts) {
    EXPECT_FALSE(p.is_visited);
  }
}

TEST(Grid, AvancaExatamenteONumeroDePassosPedido)
{
  const int passos = 3;
  const double step = 2.0;
  const auto pts = makeSerpentineGrid(0.0, 0.0, -6.0, step, passos, -2.5);

  ASSERT_FALSE(pts.empty());
  // O x final é o número de avanços vezes o tamanho do passo. Se a serpentina
  // avançasse a mais, o drone sairia da arena pelo fundo.
  EXPECT_DOUBLE_EQ(pts.back().coordinates.x(), passos * step);
}

TEST(Grid, TerminaNumaPontaEmY)
{
  const auto pts = makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  // A grade não pode acabar no meio de uma faixa: o último ponto tem de estar
  // numa das duas bordas em y, ou a última faixa fica varrida pela metade.
  const double y = pts.back().coordinates.y();
  EXPECT_TRUE(y == 0.0 || y == -6.0) << "terminou em y=" << y;
}

TEST(Grid, SoAlternaEntreAsDuasBordasEmY)
{
  const auto pts = makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, 4, -2.5);

  for (const auto & p : pts) {
    const double y = p.coordinates.y();
    EXPECT_TRUE(y == 0.0 || y == -6.0)
      << "y=" << y << " nao e nenhuma das bordas; a serpentina vazou";
  }
}

TEST(Grid, RespeitaOSinalDeYLength)
{
  // y_length positivo varre para o outro lado. É assinado de propósito: a
  // arena pode ficar à direita ou à esquerda do ponto de decolagem.
  const auto pts = makeSerpentineGrid(0.0, 0.0, +6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  EXPECT_DOUBLE_EQ(pts[0].coordinates.y(), 6.0);
  for (const auto & p : pts) {
    EXPECT_GE(p.coordinates.y(), 0.0);
  }
}

TEST(Grid, ParteDeCasaEDeslocaComEla)
{
  const auto pts = makeSerpentineGrid(5.0, 2.0, -6.0, 2.0, 3, -2.5);

  ASSERT_FALSE(pts.empty());
  // Casa deslocada desloca a grade inteira; nada é medido a partir da origem.
  EXPECT_DOUBLE_EQ(pts[0].coordinates.x(), 5.0);
  EXPECT_DOUBLE_EQ(pts[0].coordinates.y(), 2.0 - 6.0);
}

TEST(Grid, ParametrosDegeneradosNaoTravam)
{
  // Zero passos e passo de tamanho zero devolvem lista vazia em vez de entrar
  // em laço infinito. O laço de 2025 é `while (passos < n || !na_ponta)`: com
  // step_x = 0 ele nunca sairia.
  EXPECT_TRUE(makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, 0, -2.5).empty());
  EXPECT_TRUE(makeSerpentineGrid(0.0, 0.0, -6.0, 0.0, 3, -2.5).empty());
  EXPECT_TRUE(makeSerpentineGrid(0.0, 0.0, -6.0, 2.0, -1, -2.5).empty());
}
