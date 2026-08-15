# Fase 4 — CBR 2026

Travessia de uma **casa escura, sem GPS**. O drone entra por uma janela,
atravessa seis cômodos por um caminho em zigue-zague, e sai pela janela oposta.

Esta é a primeira fase modelada como **Behavior Tree** em vez de máquina de
estados, e a primeira que navega **só por LIDAR 2D**.

## A ideia, numa frase

**A planta é conhecida.** O edital dá as áreas dos cômodos e a posição das
janelas. Então o LIDAR **não serve para descobrir o caminho** — serve para
responder *onde eu estou*, que sem GPS e no escuro é a única pergunta difícil.

```
                    ┌──────────────────────────────────────┐
   maps/*.yaml ────▶│ casa.hpp   para onde ir              │
   (planta+rota)    │            (maze_geometry)           │
        │           └──────────────────────────────────────┘
        │           ┌──────────────────────────────────────┐
        │    /scan ▶│ scan_fit.hpp   onde estamos          │
        │           └──────────────────────────────────────┘
        │                          │
        ▼                          ▼
     sim2d  ◀── o MESMO arquivo ── a árvore (trees/fase4.xml)
```

O mapa é lido por três programas — a missão, o simulador 2D e o gerador do mundo
do Gazebo — e é **um arquivo só**. Uma segunda cópia produziria o pior sintoma
possível: funciona no simulador, bate na arena, e cada arquivo está certo
sozinho.

## O ciclo de um cômodo

| nó | o que faz | corrige? |
|---|---|---|
| `CentralizarNoComodo` | lê o LIDAR, corrige a transformação da odometria, vai ao centro | **sim** |
| `AlinharComAJanela` | desliza até o alinhamento lateral do vão, em dois trechos axiais, e vira de frente | ainda dá |
| `AtravessarJanela` | avança pelo vão | **não** |

A última linha é a razão da divisão. Dentro do vão o LIDAR vê duas paredes muito
próximas e nenhuma geometria de cômodo: corrigir ali usa uma fórmula que supõe um
cômodo e recebe outra coisa, devolvendo um número plausível e errado exatamente
quando um número errado custa mais caro. A resposta é **suspender** a correção —
o que só é seguro se o alinhamento já estiver feito, e por isso ele é um passo
anterior e separado.

### Frente e quartos de volta, e nada mais

Todo deslocamento passa pelo `MovimentoAxial`, que separa **girar** de
**avançar**. Não é preciosismo: girar e transladar ao mesmo tempo varre uma
pegada maior que a do drone parado, e num corredor de 0,95 m com um drone de
0,40 m a diferença entre varrer e não varrer é a diferença entre passar e bater.

**Sem `PidController`.** Os comandos de posição vão direto pela classe `Drone`, e
quem fecha a malha é o Pixhawk. Um PID nosso por cima do controlador do PX4 seria
uma segunda malha em cascata com ganhos que ninguém sintonizou, dentro de um
cômodo de 0,95 m.

## Rodando

```bash
# Simulador 2D (rápido, sem Gazebo) — terminal 1
ros2 run sim2d sim2d --ros-args \
  -p mapa:=$(ros2 pkg prefix fase4)/share/fase4/maps/cbr2026_fase4.yaml \
  -p inicio_x:=4.175 -p inicio_y:=-0.60 -p inicio_yaw:=1.5707963

# A missão — terminal 2
ros2 launch fase4 simulation.launch.py
```

A janela do `sim2d` mostra dois círculos: **azul** onde o drone está, **laranja**
onde a missão *acha* que está. Vê-los se separarem e voltarem a juntar é ver a
correção por LIDAR funcionando.

### A bateria, que é o que decide se a fase está pronta

```bash
ros2 run sim2d lote $(ros2 pkg prefix fase4)/share/fase4/lotes/fase4.yaml
```

| deriva da odometria | malha aberta | esta fase |
|---|---|---|
| 0 | 3/3 | 3/3 |
| 0,01 m/m | 1/3 | **3/3** |
| 0,02 m/m | 0/3 | **3/3** |
| 0,05 m/m | 0/3 | **3/3** |

As duas colunas vêm do **mesmo** detector de colisão, e isso não é detalhe: a
primeira medição usou um detector que lançava raios e não enxergava as quinas
dos batentes. Ele era otimista em até 2 cm, e as folgas aqui são de dez. Refeitas
com a distância ponto-segmento exata, a coluna do LIDAR não mudou e a da malha
aberta **piorou** — o contraste é maior do que eu tinha reportado.

A coluna da esquerda é o **piso**: a mesma rota voada em malha aberta, pela
`missao_reta`, que não lê o LIDAR. Ela quebra entre 1 e 2 cm de deriva por metro
percorrido, e quebra com apenas **7 cm** de erro acumulado — uma janela de 60 cm
com um drone de 40 cm deixa 10 cm de folga por lado, e o percurso tem 10 m.

Reproduzir o piso: `lotes/travessia_reta.yaml`.

### Gazebo

O mundo é **gerado do mesmo mapa**, e não desenhado à mão:

```bash
ros2 run sim2d gerar_sdf fase4:cbr2026_fase4.yaml -o labirinto.sdf
```

## O que ainda falta

**A posição da plataforma central.** O nó `IrParaPlataforma` existe e diz no log
que a chave falta. Ela **não está suposta de propósito**: as dimensões dos
cômodos dá para estimar das áreas do edital, a posição da plataforma não, e
errá-la é pousar fora. Preencher é uma linha em `maps/cbr2026_fase4.yaml`.

**Os QR Codes.** Fora do escopo por decisão — a travessia vem primeiro. O formato
do mapa já os prevê, e a discretização em grafo simplifica a busca depois: cada
parede de cada cômodo é um lugar enumerável para olhar.

**A cadeia de localização real.** O `sim2d` *fabrica* a odometria — é o que lhe
permite mentir de propósito, e é também o que ele **não** testa. A cadeia
`LIDAR → slam_toolbox → adaptador → slam_bridge → EKF do PX4` só se valida no
Gazebo com LIDAR simulado, e depois no drone.

> ### As dimensões são preliminares
>
> O edital dá **áreas** (0,9 m², 3,7 m²), não medidas de lado. Os 0,95 m de lado
> e a janela de 0,60 m em `maps/cbr2026_fase4.yaml` são **suposições**, marcadas
> como tal no arquivo.
>
> Todo número de folga citado acima depende delas. Confira com a planta oficial
> antes de concluir qualquer coisa sobre o drone caber.

## A guarda de proximidade, e por que o limite é 12 cm

Num `ReactiveSequence` o primeiro filho é reavaliado a cada tick, então
`ParedePerigosamentePerto` aborta na hora o que estiver em curso.

Mas **neste labirinto a parede está sempre perto**: um drone de 0,40 m
perfeitamente centrado num cômodo de 0,95 m tem 27 cm de folga, e atravessando um
vão de 0,60 m tem 10 cm. Uma guarda com limite "razoável" — 30 cm, 25 cm —
dispararia o tempo todo, e o primeiro reflexo de quem visse isso seria desligá-la.

Ela é o **último recurso**, não a proteção principal. Quem mantém o drone longe
das paredes são a centralização e o alinhamento.
