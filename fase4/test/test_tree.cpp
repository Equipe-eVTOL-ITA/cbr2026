// Teste da arvore.
//
// POR QUE ELE E OBRIGATORIO NUMA MISSAO EM BT
//
// Um nome de no errado no XML NAO e erro de compilacao. `<Takeof/>` em vez de
// `<Takeoff/>` compila, instala, e so falha quando a missao tenta carregar a
// arvore -- no chao, se voce tiver sorte; no ar, se nao tiver.
//
// Este teste carrega o XML INSTALADO com a mesma fabrica que a missao usa. Se
// um no nao existir, ou uma porta obrigatoria faltar, ele reprova no CI.
//
// E o preco de a arvore ser um arquivo editavel em vez de codigo, e vale a
// pena: em troca, mudar a logica da missao nao exige recompilar.

#include <string>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "stdbt/registrar.hpp"

#include "fase4/nodes/navegacao.hpp"
#include "fase4/nodes/seguranca.hpp"

namespace
{
BT::BehaviorTreeFactory fabricaDaMissao()
{
    BT::BehaviorTreeFactory factory;
    stdbt::registerAll(factory);

    // A MESMA lista que o src/fase4.cpp registra. Se as duas divergirem, o
    // teste passa e a missao nao carrega -- exatamente o que ele existe para
    // impedir.
    factory.registerNodeType<fase4::EntrarNaCasa>("EntrarNaCasa");
    factory.registerNodeType<fase4::CentralizarNoComodo>("CentralizarNoComodo");
    factory.registerNodeType<fase4::AlinharComAJanela>("AlinharComAJanela");
    factory.registerNodeType<fase4::AtravessarJanela>("AtravessarJanela");
    factory.registerNodeType<fase4::IrParaPlataforma>("IrParaPlataforma");
    factory.registerNodeType<fase4::AindaHaPassos>("AindaHaPassos");
    factory.registerNodeType<fase4::RotaCompleta>("RotaCompleta");
    factory.registerNodeType<fase4::ParedePerigosamentePerto>("ParedePerigosamentePerto");
    return factory;
}
}  // namespace

TEST(Arvore, TodosOsNosDoXmlEstaoRegistrados)
{
    auto factory = fabricaDaMissao();
    const auto caminho =
        ament_index_cpp::get_package_share_directory("fase4") + "/trees/fase4.xml";

    EXPECT_NO_THROW({
        auto arvore = factory.createTreeFromFile(caminho);
        (void)arvore;
    }) << "a arvore em " << caminho << " usa algum no que nao foi registrado";
}

TEST(Arvore, ONomeErradoRealmenteFalha)
{
    // O contraponto: confirma que o teste acima tem poder de fogo. Se esta
    // arvore obviamente errada passasse, o teste de cima nao provaria nada.
    auto factory = fabricaDaMissao();
    static const char *kXml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="Principal">
          <NoQueNaoExiste/>
        </BehaviorTree>
      </root>)";

    EXPECT_ANY_THROW({
        auto arvore = factory.createTreeFromText(kXml);
        (void)arvore;
    });
}


// ── O labirinto ─────────────────────────────────────────────────────────────

namespace
{
maze_geometry::Casa casaDaMissao()
{
    return maze_geometry::carregar(
        ament_index_cpp::get_package_share_directory("fase4") +
        "/maps/cbr2026_fase4.yaml");
}
}  // namespace

/// A planta e a rota instaladas carregam e passam nas validacoes.
///
/// O `carregar` confere que cada saida da rota tem janela e leva ao comodo do
/// passo seguinte. Sem este teste, uma rota errada so apareceria quando a
/// missao subisse -- e o CI e mais barato que descobrir isso na arena.
TEST(Labirinto, APlantaEARotaInstaladasSaoCoerentes)
{
    maze_geometry::Casa casa;
    ASSERT_NO_THROW({ casa = casaDaMissao(); });

    EXPECT_EQ(casa.comodos.size(), 6u);
    EXPECT_EQ(casa.rota.size(), 6u);
}

/// Da para deduzir por onde se entra, e ela nao e a saida do primeiro passo.
TEST(Labirinto, AEntradaEDeduzivelESoHaUma)
{
    const auto casa = casaDaMissao();
    maze_geometry::Passo entrada{};
    ASSERT_NO_THROW({ entrada = maze_geometry::entradaDoLabirinto(casa); });

    EXPECT_EQ(entrada.comodo, casa.rota.front().comodo);
    EXPECT_NE(entrada.saida, casa.rota.front().saida);
    EXPECT_EQ(maze_geometry::vizinho(casa, casa.porId(entrada.comodo), entrada.saida),
              nullptr) << "a entrada tem de dar para fora";
}

/// Cada ponto de aproximacao cabe DENTRO do seu comodo.
///
/// Com comodos de 0.95 m, um recuo escolhido sem olhar o tamanho do comodo poe
/// o ponto de aproximacao do lado de fora da parede oposta -- e o drone iria
/// para la com toda a confianca. Este teste amarra o recuo a planta.
TEST(Labirinto, OsPontosDeAproximacaoCabemNosComodos)
{
    const auto casa = casaDaMissao();
    constexpr double kRecuo = 0.35;   // o padrao de config/

    for (const auto & passo : casa.rota) {
        const auto & c = casa.porId(passo.comodo);
        const auto a = maze_geometry::aproximacao(c, passo.saida, kRecuo);
        EXPECT_TRUE(c.contem(a.x(), a.y()))
            << "a aproximacao da saida " << maze_geometry::paraTexto(passo.saida)
            << " de '" << c.nome << "' caiu fora do comodo: recuo grande demais "
            << "para um comodo de " << c.altura << " x " << c.largura << " m";
    }
}

/// E cada ponto de transposicao cai FORA -- ja do outro lado.
TEST(Labirinto, OsPontosDeTransposicaoCaemDoOutroLado)
{
    const auto casa = casaDaMissao();
    constexpr double kAvanco = 0.35;

    for (const auto & passo : casa.rota) {
        const auto & c = casa.porId(passo.comodo);
        const auto t = maze_geometry::transposicao(c, passo.saida, kAvanco);
        EXPECT_FALSE(c.contem(t.x(), t.y()))
            << "a transposicao da saida " << maze_geometry::paraTexto(passo.saida)
            << " de '" << c.nome << "' ficou dentro do proprio comodo";
    }
}
