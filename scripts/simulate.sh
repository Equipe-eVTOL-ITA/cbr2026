#!/usr/bin/env bash
# =============================================================================
# Template: simulate.sh — sobe o PX4 SITL + Gazebo para um mundo.
# =============================================================================
#
#   ./scripts/simulate.sh <mundo>
#
# Copie para o scripts/ da sua competição e preencha o bloco `case` com os
# mundos, modelos e poses iniciais dela. É o ÚNICO arquivo dos três templates
# que exige customização de verdade.
# =============================================================================
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# <ws>/src/<competicao>/scripts/  ->  <ws>
ws_root="$(cd "$script_dir/../../.." && pwd)"

# Carrega a distro do perfil desta máquina + o install/ do workspace.
# Nunca escreva `source /opt/ros/humble/setup.bash` aqui: o time voa com
# Jetson (Humble) e Raspberry Pi (Jazzy), e o mesmo script tem que servir aos
# dois. Veja docs/ARCHITECTURE.md, "O Contrato de Ambiente".
source "$ws_root/scripts/ros_env.sh"

# MODELOS: resolvidos pelo Gazebo via GZ_SIM_RESOURCE_PATH, e nao precisam
# estar dentro da arvore do PX4. O proprio PX4 acrescenta a arvore dele ao
# final desta variavel (veja gz_env.sh), entao o que exportamos aqui vem
# ANTES e tem prioridade.
#
# A ordem importa e ja causou bug: quando a arvore do PX4 vinha primeiro, uma
# copia antiga de modelo escondia a deste repositorio, e a versao que rodava
# nao era a versionada. Com o repo da equipe na frente, ele e a fonte da
# verdade e pode ate sobrescrever um modelo do proprio PX4.
#
# (As variaveis GAZEBO_* sao do Gazebo classico e nao funcionam aqui.)
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-gazebo-models/models:${GZ_SIM_RESOURCE_PATH:-}"

cd "$HOME/PX4-Autopilot"

# ---------------------------------------------------------------------------
# Sobrou simulacao da vez passada?
#
# O `gz sim` SOBREVIVE quando o PX4 morre. Na proxima tentativa o PX4 sobe um
# gz novo, mas a chamada de servico que pede o spawn do drone pode ir parar no
# servidor VELHO -- que tem outro mundo e nao responde. O sintoma nao menciona
# processo nenhum:
#
#     ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH
#     ERROR [init] gz_bridge failed to start and spawn model
#
# ...e manda todo mundo mexer no GZ_SIM_RESOURCE_PATH, que esta certo. O drone
# simplesmente nao aparece.
#
# O agente da o mesmo tipo de pista enganosa, uma janela depois:
#
#     bind error | port: 8888, errno: 98
#
# Melhor recusar de saida, dizendo o que matar.
# ---------------------------------------------------------------------------
# A lista vem do scripts/processos.sh, e nao daqui. O `pgrep -f 'gz sim'` que
# estava neste arquivo casava com a propria linha de comando de quem o rodava.
# shellcheck source=../../../scripts/processos.sh
source "$ws_root/scripts/processos.sh"

# `|| true` obrigatorio: evtol_simulacao_viva devolve 1 quando NAO ha
# simulacao, e com `set -e` + `pipefail` o script morreria em silencio no caso
# bom. Sem o agente na lista de proposito -- quem o sobe e o agent.sh.
sobrando="$(evtol_simulacao_viva px4 gazebo | tr '\n' ' ' || true)"

if [[ -n "${sobrando// /}" ]]; then
    echo "ERRO: ja ha simulacao rodando ($sobrando)." >&2
    echo >&2
    echo "      O gz sim sobrevive quando o PX4 morre, e a proxima tentativa" >&2
    echo "      falha com 'Service call timed out' -- que aponta para o lugar" >&2
    echo "      errado. O drone nao aparece e a mensagem fala de outra coisa." >&2
    echo >&2
    echo "      Rode a task 'sim: parar tudo', ou:" >&2
    echo "          ./scripts/parar.sh" >&2
    exit 1
fi

PX4_SYS_AUTOSTART=4001

case "${1:-}" in
    # ---- CUSTOMIZE: os mundos da sua competição -------------------------
    # fase1)
    #     PX4_GZ_WORLD=fase1_27                       # nome do .sdf
    #     PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"   # x,y,z,r,p,y
    #     PX4_SIM_MODEL=x500_sae                      # modelo do drone
    #     ;;
    fase1)
        PX4_GZ_WORLD=cbr2026_fase1                       # nome do .sdf
        PX4_GZ_MODEL_POSE="8.0, 2.0, 1.5, 0.0, 0.0, 0.0"   # x,y,z,r,p,y
        PX4_SIM_MODEL=x500_cbr2026                      # modelo do drone
        ;;
    fase4)
        # NOME PROPRIO, e nao `fase4`.
        #
        # Ja existe um worlds/fase4.sdf no PX4-gazebo-models, de outra prova:
        # arena, banner e plataformas. Gerar por cima dele destruiria aquilo --
        # eu fiz exatamente isso uma vez, e so percebi porque o `git status`
        # marcou o arquivo como MODIFICADO em vez de novo.
        #
        # O nome segue a convencao que a fase 1 ja usa: cbr2026_<fase>.
        PX4_GZ_WORLD=cbr2026_fase4

        # O MUNDO E REGERADO AQUI, a cada simulacao, e contem SO o labirinto e
        # o chao de grama.
        #
        # Ele sai do MESMO YAML que a missao le. Se o mapa mudasse e o .sdf nao,
        # o drone voaria num labirinto diferente do que a missao acredita -- e
        # nada acusaria: cada arquivo estaria certo sozinho.
        #
        # Regerar custa um piscar de olhos e elimina a classe inteira de
        # problema. Fica aqui, e nao numa task separada, porque o caminho para
        # rodar uma simulacao tem de ser o mesmo para todas as fases.
        echo "Regerando o mundo da fase 4 a partir do mapa..."
        ros2 run sim2d gerar_sdf fase4:cbr2026_fase4.yaml \
            --nome cbr2026_fase4 \
            -o "$HOME/PX4-gazebo-models/worlds/cbr2026_fase4.sdf"

        # A POSE ESTA EM ENU, e o mapa esta em NED. Os eixos TROCAM:
        #
        #     gazebo.x = mapa.y   (leste)
        #     gazebo.y = mapa.x   (norte)
        #
        # A decolagem no mapa e (x=4.175, y=-0.600) virada para o LESTE, que e
        # o alinhamento com a janela de entrada. Em ENU isso vira (-0.600,
        # 4.175) com guinada ZERO, porque no ENU o angulo cresce do leste para
        # o norte e o leste e o proprio eixo x.
        #
        # Estes tres numeros TEM de casar com inicio_x/inicio_y/inicio_yaw do
        # config/simulation.yaml da fase: e de la que sai a transformacao
        # inicial entre o mapa e a odometria. Se divergirem, o drone comeca a
        # missao acreditando estar noutro lugar -- e nao ha erro nenhum.
        PX4_GZ_MODEL_POSE="-0.600, 4.175, 0.15, 0.0, 0.0, 0.0"

        # O `wanda`: o x500 em escala 0.45, com o mesmo LIDAR 2D no topo
        # (1080 amostras, 270 graus, 30 m, 30 Hz).
        #
        # O x500 NAO SERVE para esta fase: ele tem 0.772 m de envergadura e as
        # janelas do labirinto tem 0.60 m -- faltam 8.6 cm de cada lado, e nao e
        # apertado, e impossivel. O wanda tem 0.347 m.
        #
        # Gerado por tools/gerar_wanda.py no repositorio PX4-gazebo-models, com
        # as leis de escala escritas no proprio script: massa NAO segue o cubo,
        # inercia segue massa vezes comprimento ao quadrado, e a constante do
        # motor e ajustada para pairar na mesma fracao da rotacao maxima.
        PX4_SIM_MODEL=wanda
        ;;
    default)
        PX4_GZ_WORLD=default
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500
        ;;
    *)
        echo "Mundo desconhecido: '${1:-}'" >&2
        echo "Uso: $0 <mundo>" >&2
        echo "Disponíveis: fase1, fase4, default" >&2
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Confere que o mundo e o modelo existem ANTES de chamar o PX4.
#
# Sem isto, o PX4 falha com uma mensagem que aponta para o lugar errado:
#
#     Unable to find or download file
#     ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH
#     ERROR [init] gz_bridge failed to start and spawn model
#
# ...o que faz todo mundo ir mexer no GZ_SIM_RESOURCE_PATH, quando a causa
# quase sempre e outra: o .sdf existe em ~/PX4-gazebo-models mas nao foi
# symlinkado para dentro do PX4. Os symlinks sao feitos uma vez; arquivo novo
# adicionado depois nao ganha symlink sozinho.
# ---------------------------------------------------------------------------
px4_gz="$HOME/PX4-Autopilot/Tools/simulation/gz"
faltando=0

# MUNDOS: o PX4 monta um caminho ABSOLUTO na arvore dele e passa ao gz sim
# (px4-rc.simulator: `gz sim -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf"`).
# Diferente dos modelos, aqui o arquivo precisa mesmo aparecer la dentro.
# Criamos o link do mundo que vamos lancar, e so dele -- assim ninguem
# precisa lembrar de refazer symlink quando um mundo novo entra no repo.
if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" \
   && -e "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "Criando link para o mundo '$PX4_GZ_WORLD' na arvore do PX4"
    ln -sf "$HOME/PX4-gazebo-models/worlds/$PX4_GZ_WORLD.sdf" "$px4_gz/worlds/"
fi

if [[ ! -e "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" ]]; then
    echo "ERRO: mundo '$PX4_GZ_WORLD.sdf' nao encontrado em" >&2
    echo "      $px4_gz/worlds/" >&2
    echo "      -> nao existe nem em ~/PX4-gazebo-models. Crie o .sdf la." >&2
    faltando=1
fi

# MODELOS: basta existir no repositorio da equipe ou na arvore do PX4 --
# os dois estao no GZ_SIM_RESOURCE_PATH. Nao ha symlink de modelo.
if [[ ! -e "$HOME/PX4-gazebo-models/models/$PX4_SIM_MODEL" \
   && ! -e "$px4_gz/models/$PX4_SIM_MODEL" ]]; then
    echo "ERRO: modelo '$PX4_SIM_MODEL' nao encontrado." >&2
    echo "      Procurado em ~/PX4-gazebo-models/models/ e em" >&2
    echo "      $px4_gz/models/" >&2
    echo "      -> crie o modelo em ~/PX4-gazebo-models/models/." >&2
    faltando=1
fi

if (( faltando )); then
    echo >&2
    echo "Veja docs/gazebo_models_setup.md." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Sobe o Gazebo NOS MESMOS, e so chama o PX4 quando o mundo estiver pronto.
#
# Deixar o PX4 subir o Gazebo perde uma corrida que ele nao tenta ganhar. Em
# px4-rc.simulator ele lanca `gz sim -s` em background e segue direto; logo
# depois o GZBridge pede o spawn do drone com UM UNICO request de 1000 ms e
# sem retry (GZBridge.cpp, ramo `else` de PX4_GZ_STANDALONE):
#
#     ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH
#     ERROR [init] gz_bridge failed to start and spawn model
#
# Medido nesta arena: o mundo leva ~2,4 s ate anunciar /world/<mundo>/create,
# com o cache quente. Contra 1 s de paciencia, o PX4 perde quase sempre -- dai
# o "as vezes funciona": funciona quando um `gz sim` de uma tentativa ANTERIOR
# ficou vivo e ja tinha o mundo carregado. Ou seja, o caso que parecia sucesso
# dependia do lixo da falha anterior, e podia spawnar no mundo errado.
#
# A mensagem culpa o GZ_SIM_RESOURCE_PATH, que nao tem nada a ver.
#
# Com PX4_GZ_STANDALONE=1 o PX4 nao sobe o Gazebo e passa a usar o outro ramo
# do GZBridge, que reteta a cada 2 s ate conseguir. Como nos so o chamamos
# depois do mundo pronto, ele acerta de primeira.
#
# CUIDADO com o valor: px4-rc.simulator testa se a variavel e NAO-VAZIA para
# decidir se sobe o Gazebo, mas o GZBridge exige exatamente "1" para ativar o
# retry. Qualquer outro valor (`true`, `yes`) da o pior dos dois mundos: o PX4
# nao sobe o Gazebo E continua com o request unico de 1 s.
# ---------------------------------------------------------------------------
gz_log="$(mktemp -t evtol_gz_XXXXXX.log)"

# --verbose=3 (e nao o =1 que o px4-rc.simulator usa) porque e em 3 que o gz
# imprime as linhas [Msg] com os servicos que anunciou. Em 1 ele so loga erro,
# o arquivo fica VAZIO e a espera abaixo ia ate o timeout com o mundo pronto.
# Vai para arquivo, entao nao polui o console.
gz sim --verbose=3 -r -s "$px4_gz/worlds/$PX4_GZ_WORLD.sdf" > "$gz_log" 2>&1 &
gz_pid=$!

# Sem isto, um Ctrl+C no PX4 (ou um erro aqui) deixa exatamente o `gz sim`
# orfao que o bloco "Sobrou simulacao da vez passada?" no topo deste arquivo
# existe para barrar -- e a proxima execucao ja comeca recusada.
limpar_gazebo() {
    kill "$gz_pid" 2>/dev/null || true
    rm -f "$gz_log"
}
trap limpar_gazebo EXIT

if [[ -z "${HEADLESS:-}" ]]; then
    gz sim -g > /dev/null 2>&1 &
fi

# Espera o SERVICO DE SPAWN, e nao o processo: o `gz sim` existe muito antes
# de o mundo estar carregado, e e justamente essa janela que derruba o PX4.
# Olhamos o log em vez de chamar `gz service -l` porque a descoberta do gz
# custa ~2 s por chamada -- mais que o proprio carregamento do mundo.
echo "Aguardando o Gazebo carregar '$PX4_GZ_WORLD'..."
pronto=0
for _ in $(seq 1 600); do   # 600 x 0,1 s = 60 s
    # Casamos so o TOPICO, que e uma substring contigua. O gz intercala
    # codigos ANSI de cor no meio da frase ("Create service on [" ESC
    # "/world/.../create" ESC "]"), entao procurar a linha inteira como texto
    # literal nunca casa -- e a espera ia ate o timeout com o mundo pronto.
    if grep -qa "/world/$PX4_GZ_WORLD/create" "$gz_log" 2>/dev/null; then
        pronto=1
        break
    fi
    if ! kill -0 "$gz_pid" 2>/dev/null; then
        echo "ERRO: o Gazebo morreu antes de carregar o mundo. Log:" >&2
        tail -20 "$gz_log" >&2
        exit 1
    fi
    sleep 0.1
done

if (( ! pronto )); then
    echo "ERRO: o Gazebo nao anunciou /world/$PX4_GZ_WORLD/create em 60 s." >&2
    echo "      Ultimas linhas do log:" >&2
    tail -20 "$gz_log" >&2
    exit 1
fi

echo "Gazebo pronto. Subindo o PX4."

PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_GZ_MODEL_POSE=$PX4_GZ_MODEL_POSE \
PX4_GZ_WORLD=$PX4_GZ_WORLD \
PX4_SIM_MODEL=$PX4_SIM_MODEL \
./build/px4_sitl_default/bin/px4
