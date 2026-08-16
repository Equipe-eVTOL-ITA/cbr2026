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
sobrando=""
pgrep -x px4            >/dev/null 2>&1 && sobrando+=" px4"
pgrep -f 'gz sim'       >/dev/null 2>&1 && sobrando+=" gz"
pgrep -x MicroXRCEAgent >/dev/null 2>&1 && sobrando+=" agente"

if [[ -n "$sobrando" ]]; then
    echo "ERRO: ja ha simulacao rodando ($sobrando)." >&2
    echo >&2
    echo "      O gz sim sobrevive quando o PX4 morre, e a proxima tentativa" >&2
    echo "      falha com 'Service call timed out' -- que aponta para o lugar" >&2
    echo "      errado. O drone nao aparece e a mensagem fala de outra coisa." >&2
    echo >&2
    echo "      Rode a task 'sim: parar tudo', ou:" >&2
    echo "          pkill -x px4; pkill -f 'gz sim'; pkill -x MicroXRCEAgent" >&2
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

PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_GZ_MODEL_POSE=$PX4_GZ_MODEL_POSE \
PX4_GZ_WORLD=$PX4_GZ_WORLD \
PX4_SIM_MODEL=$PX4_SIM_MODEL \
./build/px4_sitl_default/bin/px4
