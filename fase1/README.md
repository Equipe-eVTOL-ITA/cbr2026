# Fase 1 — CBR 2026

Varrer a arena em grade, pousar em cada base encontrada, voltar para casa.

```bash
# terminal 1
bash src/cbr2026/scripts/simulate.sh <mundo>
# terminal 2
bash src/cbr2026/scripts/agent.sh
# terminal 3
ros2 launch fase1 simulation.launch.py
```

## O que esta fase escreve, e o que ela apenas usa

O pacote é fino de propósito. Quase tudo que a fase faz mora em bibliotecas com
teste e CI próprios; o que ficou aqui é o que só esta prova sabe.

| Peça | Onde vive |
|---|---|
| Projeção pixel → mundo | [`vision_geometry`](https://github.com/Equipe-eVTOL-ITA/vision_geometry) |
| Armar, decolar, alinhar, pousar, voltar | [`stdstates`](https://github.com/Equipe-eVTOL-ITA/stdstates) |
| Detecção de bases azul+amarelo | [`cv_nodes/base_detector`](https://github.com/Equipe-eVTOL-ITA/cv_nodes) |
| **Grade de varredura** | `include/fase1/grid.hpp` |
| **"Esta base é nova?"** | `include/fase1/states/search_base_state.hpp` |
| **Registro de bases visitadas** | `include/fase1/states/register_base_state.hpp` |
| **Cola entre detector e missão** | `include/fase1/vision_fase1.hpp` |

Em 2025 o equivalente tinha sete estados escritos do zero, 493 linhas de visão
embutidas no nó da missão e duas bibliotecas forkadas (`cbr_drone_lib`,
`cbr_cv_utils`). A prova é a mesma.

## Máquina de estados

```
ARMING → TAKEOFF → SEARCH BASE ⇄ PRECISION ALIGN → PRECISION LANDING
                        ↑                                  ↓
                    TAKEOFF ←────────────────────── REGISTER BASE
                        ↓  (todas as bases feitas)
                   RETURN HOME → FINISHED
```

Duas decisões que diferem de 2025:

- **Não há estado `GO TO BASE`.** O `PrecisionAlignState` já leva o drone até o
  alvo por PID. Um estado separado só para se aproximar é uma etapa a mais em
  que o alvo pode sair do campo de visão.
- **Perder o alvo volta para `SEARCH BASE`, não aborta.** O progresso da
  varredura mora em `is_visited` dentro dos waypoints, então o drone retoma a
  grade de onde parou.

## Os dois perfis

`config/simulation.yaml` e `config/flight.yaml` têm **as mesmas chaves** e 22
valores diferentes. As diferenças estão marcadas com `[SIM]` e `[VOO]`. Trocar
de perfil é trocar de arquivo — nunca editar o `.cpp`.

```bash
diff config/simulation.yaml config/flight.yaml
```

O que muda de verdade: velocidades e ganhos (mais conservadores em voo),
tolerâncias (mais folgadas, porque a posição real tem ruído), faixas HSV (mais
largas, porque lona sob sol não tem cor uniforme), tópico de imagem, frequência
de processamento e nível de debug.

### Antes do primeiro voo

Três coisas no `flight.yaml` **precisam** ser conferidas:

1. **Intrínsecos da câmera**, medidos com o `camera_calibrator`. Os do perfil de
   simulação são derivados do FOV de uma câmera ideal; uma lente real tem
   distorção e centro óptico deslocado, e tratá-la como pinhole produz erro que
   cresce em direção às bordas — justamente onde a base aparece na aproximação.
2. **Faixas HSV**, na iluminação do dia.
3. **Posição de casa**, que em voo raramente é a origem.

## Extrínsecos da câmera — leia antes de mexer

`camera_yaw: 1.570796327` não é arbitrário. É a montagem em que o topo da imagem
aponta para a frente do drone:

```
imagem +x  →  corpo +Y (direita)
imagem +y  →  corpo -X (trás)
eixo óptico +Z_cam  →  corpo +Z (baixo)
```

**Extrínsecos nulos também dão uma câmera olhando para baixo**, mas girada 90° —
a direita da imagem viraria a frente do drone. Os dois "olham para baixo"; um
mede tudo 90° fora de lugar, e não há erro nenhum: o drone só alinha ao lado da
base. Os configs de 2025 discordavam entre si exatamente aqui (a fase 1 usava
`yaw=π/2`; as fases 2 e 4, `pitch=π/2`, que aponta o eixo óptico para a frente).

## Uma armadilha do workspace

Se você tiver os repositórios legados (`itajuba`, `sae_2025`) no `src/`, o
`ros2 run base_detector base_detector` pode subir o **detector errado**:

```
$ python3 -c "import base_detector.base_detector as m; print(m.__file__)"
.../build/itajuba_cv_utils/base_detector/base_detector.py
```

O `itajuba_cv_utils` expõe um módulo Python de topo chamado `base_detector`, e o
`build/itajuba_cv_utils` vem antes no `PYTHONPATH`. O nó sobe, não reclama de
nada e detecta outra coisa.

Uma máquina montada só a partir do `evtol.repos` **não** tem esse problema —
`itajuba` e `sae_2025` não estão no manifesto. Se você tem os dois no `src/`
por motivo histórico, confira qual módulo resolve antes de culpar a detecção.

## Testes

```bash
colcon test --packages-select fase1 --ctest-args -R test_grid
```

A grade é a única lógica pura desta missão, e é a que decide se a arena inteira
é coberta. Errar ali não gera erro: o drone voa bonito e simplesmente não passa
por cima de uma das bases. Em 2025 esse laço vivia dentro do construtor da FSM e
só dava para conferi-lo subindo a simulação.
