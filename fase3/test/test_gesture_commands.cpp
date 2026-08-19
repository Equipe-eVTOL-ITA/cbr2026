// Testes do vocabulário de gestos.
//
// É a regra que decide para onde o drone vai. Em 2025 ela vivia dentro de um
// `handleGesture` privado num estado que exigia um `Drone` para ser
// instanciado, e conferi-la significava subir simulação e olhar o drone se
// mexer. Como função pura, cada caso é uma linha.

#include <string>

#include <gtest/gtest.h>

#include "fase3/gesture_commands.hpp"

using fase3::ConfirmadorDeEncerramento;
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
  // CONTROL ele manda voltar para casa. Os estados são exclusivos no tempo,
  // mas o GESTO ATRAVESSA A TRANSIÇÃO -- é por isso que a confirmação sozinha
  // não bastava, e existe a trava do ConfirmadorDeEncerramento.
  EXPECT_TRUE(ehGestoDeEncerramento(fase3::kGestoVoltar));
  EXPECT_STREQ(fase3::kGestoVoltar, "Open_Palm");
}

// --- A trava de entrada ------------------------------------------------------
//
// Regressão do voo em que o drone voltava para casa 0.5 s depois de encontrar a
// mão: a palma que o chamou continuava no quadro e completava sozinha os 10
// ciclos de confirmação.

namespace
{
constexpr int kCiclos = 10;

/// Alimenta o confirmador N vezes com o mesmo gesto; devolve o que confirmou.
std::string segura(ConfirmadorDeEncerramento & c, const std::string & gesto, int n)
{
  std::string ultimo;
  for (int i = 0; i < n; i++) {
    ultimo = c.atualiza(gesto, kCiclos);
  }
  return ultimo;
}
}  // namespace

TEST(TravaDeEntrada, PalmaQueChamouNaoMandaVoltarParaCasa)
{
  ConfirmadorDeEncerramento c;
  c.reiniciar();

  // O operador nunca soltou a palma: ela vem do SEARCH HAND direto para cá.
  EXPECT_EQ(segura(c, "Open_Palm", kCiclos * 5), "")
    << "a palma de chamada nao pode virar comando sozinha";
  EXPECT_TRUE(c.travado());
}

TEST(TravaDeEntrada, DepoisDeSoltarAPalmaOComandoVale)
{
  ConfirmadorDeEncerramento c;
  c.reiniciar();

  segura(c, "Open_Palm", kCiclos * 2);   // ainda travado
  c.atualiza("", kCiclos);               // operador baixa a mao: destrava
  EXPECT_FALSE(c.travado());

  EXPECT_EQ(segura(c, "Open_Palm", kCiclos), "Open_Palm");
}

TEST(TravaDeEntrada, QualquerGestoNaoEncerradorDestrava)
{
  // Baixar a mao nao e a unica forma de soltar: um direcional tambem serve, e
  // e o caso comum -- o operador chama, aponta, e so depois manda voltar.
  for (const std::string g : {"", "Pointing_Up", "Closed_Fist", "Victory", "ILoveYou"}) {
    ConfirmadorDeEncerramento c;
    c.reiniciar();
    c.atualiza(g, kCiclos);
    EXPECT_FALSE(c.travado()) << "gesto: '" << g << "'";
  }
}

TEST(TravaDeEntrada, ThumbDownDaRedecolagemNaoPousaDeNovo)
{
  // Mesmo bug pelo outro caminho: PRECISION LANDING -> TAKEOFF AGAIN ->
  // GESTURE CONTROL com o Thumb_Down ainda na mao do operador.
  ConfirmadorDeEncerramento c;
  c.reiniciar();
  EXPECT_EQ(segura(c, "Thumb_Down", kCiclos * 5), "");
}

TEST(TravaDeEntrada, ExigeCiclosCONSECUTIVOS)
{
  ConfirmadorDeEncerramento c;
  c.reiniciar();
  c.atualiza("", kCiclos);   // destrava

  segura(c, "Thumb_Down", kCiclos - 1);
  EXPECT_EQ(c.atualiza("Pointing_Up", kCiclos), "") << "um quadro solto zera a contagem";
  EXPECT_EQ(segura(c, "Thumb_Down", kCiclos - 1), "");
  EXPECT_EQ(c.atualiza("Thumb_Down", kCiclos), "Thumb_Down");
}

TEST(TravaDeEntrada, TrocarDeGestoDeEncerramentoReiniciaAContagem)
{
  ConfirmadorDeEncerramento c;
  c.reiniciar();
  c.atualiza("", kCiclos);

  segura(c, "Open_Palm", kCiclos - 1);
  // Trocar direto de um encerrador para o outro nao pode somar os ciclos.
  EXPECT_EQ(c.atualiza("Thumb_Down", kCiclos), "");
  EXPECT_EQ(segura(c, "Thumb_Down", kCiclos - 1), "Thumb_Down");
}

TEST(TravaDeEntrada, ReiniciarTravaDeNovo)
{
  // Voltar ao estado (LOST HAND -> SEARCH HAND -> GESTURE CONTROL) rearma.
  ConfirmadorDeEncerramento c;
  c.reiniciar();
  c.atualiza("", kCiclos);
  EXPECT_FALSE(c.travado());

  c.reiniciar();
  EXPECT_TRUE(c.travado());
  EXPECT_EQ(segura(c, "Open_Palm", kCiclos * 2), "");
}
