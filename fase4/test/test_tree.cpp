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

// ACRESCENTE aqui os includes dos nos desta missao, e registre-os abaixo.
// #include "fase4/nodes/meu_no.hpp"

namespace
{
BT::BehaviorTreeFactory fabricaDaMissao()
{
    BT::BehaviorTreeFactory factory;
    stdbt::registerAll(factory);
    // ACRESCENTE: factory.registerNodeType<MeuNo>("MeuNo");
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
