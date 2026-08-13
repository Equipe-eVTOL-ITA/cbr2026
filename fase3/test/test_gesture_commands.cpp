// Testes do vocabulário de gestos.
//
// É a regra que decide para onde o drone vai. Em 2025 ela vivia dentro de um
// `handleGesture` privado num estado que exigia um `Drone` para ser
// instanciado, e conferi-la significava subir simulação e olhar o drone se
// mexer. Como função pura, cada caso é uma linha.

#include <string>

#include <gtest/gtest.h>

#include "fase3/gesture_commands.hpp"

using fase3::ehGestoDeEncerramento;
using fase3::velocidadeDoGesto;

namespace
{
constexpr double kVel = 0.4;
constexpr double kSubida = 0.1;
}  // namespace

// --- Os quatro direcionais ---------------------------------------------------
//
// Eixos do CORPO, FRD: x frente, y direita, z para baixo.

TEST(Vocabulario, PointingUpVaiParaFrente)
{
  const auto v = velocidadeDoGesto("Pointing_Up", kVel, kSubida);
  EXPECT_DOUBLE_EQ(v.x(), kVel);
  EXPECT_DOUBLE_EQ(v.y(), 0.0);
}

TEST(Vocabulario, ClosedFistVaiParaTras)
{
  const auto v = velocidadeDoGesto("Closed_Fist", kVel, kSubida);
  EXPECT_DOUBLE_EQ(v.x(), -kVel);
  EXPECT_DOUBLE_EQ(v.y(), 0.0);
}

TEST(Vocabulario, VictoryVaiParaADireita)
{
  const auto v = velocidadeDoGesto("Victory", kVel, kSubida);
  EXPECT_DOUBLE_EQ(v.x(), 0.0);
  EXPECT_DOUBLE_EQ(v.y(), kVel) << "y positivo e a direita em FRD";
}

TEST(Vocabulario, ILoveYouVaiParaAEsquerda)
{
  const auto v = velocidadeDoGesto("ILoveYou", kVel, kSubida);
  EXPECT_DOUBLE_EQ(v.x(), 0.0);
  EXPECT_DOUBLE_EQ(v.y(), -kVel);
}

TEST(Vocabulario, FrenteETrasSaoOpostos)
{
  const auto frente = velocidadeDoGesto("Pointing_Up", kVel, 0.0);
  const auto tras = velocidadeDoGesto("Closed_Fist", kVel, 0.0);
  EXPECT_DOUBLE_EQ(frente.x(), -tras.x());
}

TEST(Vocabulario, DireitaEEsquerdaSaoOpostos)
{
  const auto dir = velocidadeDoGesto("Victory", kVel, 0.0);
  const auto esq = velocidadeDoGesto("ILoveYou", kVel, 0.0);
  EXPECT_DOUBLE_EQ(dir.y(), -esq.y());
}

// --- O estado neutro ---------------------------------------------------------

TEST(Vocabulario, GestoDesconhecidoParaNoLugar)
{
  // O estado neutro do controle por gestos é PARAR, não continuar o último
  // comando. Um operador que baixa a mão espera que o drone pare.
  for (const std::string g : {"", "Thumb_Up", "coisa_nenhuma", "OK"}) {
    const auto v = velocidadeDoGesto(g, kVel, kSubida);
    EXPECT_DOUBLE_EQ(v.x(), 0.0) << "gesto: '" << g << "'";
    EXPECT_DOUBLE_EQ(v.y(), 0.0) << "gesto: '" << g << "'";
  }
}

TEST(Vocabulario, CorrecaoVerticalValeParaQualquerGesto)
{
  // A correção de altitude vem do PID que mantém a mão centrada, e não do
  // gesto. Ela tem de sair inclusive quando não há gesto nenhum — senão o
  // drone para de seguir o operador no instante em que ele baixa a mão.
  for (const std::string g :
    {"Pointing_Up", "Closed_Fist", "Victory", "ILoveYou", "", "Thumb_Down"})
  {
    const auto v = velocidadeDoGesto(g, kVel, kSubida);
    EXPECT_DOUBLE_EQ(v.z(), kSubida) << "gesto: '" << g << "'";
  }
}

TEST(Vocabulario, GestosDeEncerramentoNaoDeslocam)
{
  // Enquanto o comando de pousar está sendo confirmado, o drone não pode
  // continuar andando.
  for (const std::string g : {"Thumb_Down", "Open_Palm"}) {
    const auto v = velocidadeDoGesto(g, kVel, 0.0);
    EXPECT_DOUBLE_EQ(v.x(), 0.0) << "gesto: '" << g << "'";
    EXPECT_DOUBLE_EQ(v.y(), 0.0) << "gesto: '" << g << "'";
  }
}

// --- Reconhecimento dos comandos de encerramento -----------------------------

TEST(Vocabulario, ReconheceOsDoisGestosDeEncerramento)
{
  EXPECT_TRUE(ehGestoDeEncerramento("Thumb_Down"));
  EXPECT_TRUE(ehGestoDeEncerramento("Open_Palm"));
}

TEST(Vocabulario, DirecionaisNaoEncerram)
{
  for (const std::string g :
    {"Pointing_Up", "Closed_Fist", "Victory", "ILoveYou", ""})
  {
    EXPECT_FALSE(ehGestoDeEncerramento(g)) << "gesto: '" << g << "'";
  }
}

TEST(Vocabulario, OpenPalmEncerraMasTambemChamaODrone)
{
  // O mesmo gesto tem dois papéis: no SEARCH HAND ele é o chamado, no GESTURE
  // CONTROL ele manda voltar para casa. Não é conflito -- os estados são
  // exclusivos -- mas é uma armadilha para quem for mexer no vocabulário sem
  // ler os dois estados.
  EXPECT_TRUE(ehGestoDeEncerramento(fase3::kGestoVoltar));
  EXPECT_STREQ(fase3::kGestoVoltar, "Open_Palm");
}
