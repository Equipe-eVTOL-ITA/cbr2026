# Fase 3 — CBR 2026

Controle do drone por **gestos da mão**. O drone decola, gira procurando o
operador, e a partir daí obedece: avança, recua, translada, pousa, volta.

## O arranjo, que explica os quatro launches

Ao contrário da fase 1, o processamento **não é embarcado**:

```
DRONE                              COMPUTADOR DE SOLO
  camera_publisher
    /camera/image/compressed  ──rede──▶  gesture_detector
                                              │ /gesture_detector/*
                                              ▼
                                           fase3 (FSM)
    ◀────── /fmu/in/trajectory_setpoint ──────┘
```

A câmera fica no drone olhando para a frente, e **a imagem comprimida atravessa
a rede**. Isso muda o que importa na configuração: `processing_frequency` e a
qualidade do JPEG deixam de ser orçamento de CPU e passam a ser de **banda**.

Se a missão estiver lenta para responder, meça isto **antes** de mexer em ganho
de PID:

```bash
ros2 topic bw /camera/image/compressed
```

| launch | onde roda | o que sobe |
|---|---|---|
| `simulation.launch.py` | uma máquina só | webcam local + detector + missão |
| `onboard.launch.py` | **drone** | só a câmera |
| `ground.launch.py` | **solo** | detector + missão + gravação |
| `replay.launch.py` | qualquer | só a missão, consumindo um bag |

### Simulação

```bash
# terminal 1
bash src/cbr2026/scripts/simulate.sh <mundo>
# terminal 2
bash src/cbr2026/scripts/agent.sh
# terminal 3
ros2 launch fase3 simulation.launch.py
```

A **webcam da sua máquina faz o papel da câmera do drone** — o Gazebo não
renderiza uma mão. Você gesticula para a webcam e o drone responde na simulação.
É feio, e é o único jeito de exercitar o laço inteiro, incluindo a latência do
classificador. Foi o que 2025 fez.

### Voo

Suba o **onboard primeiro**. O `ground.launch.py` espera 8 s antes de iniciar a
FSM, mas se o drone ainda não estiver publicando imagem quando ela começar, o
drone decola e gira procurando uma mão que ninguém está enxergando.

## Vocabulário de gestos

| gesto | no `SEARCH HAND` | no `GESTURE CONTROL` |
|---|---|---|
| `Open_Palm` | **chama o drone** | volta para casa |
| `Pointing_Up` | — | avança |
| `Closed_Fist` | — | recua |
| `Victory` | — | translada para a direita |
| `ILoveYou` | — | translada para a esquerda |
| `Thumb_Down` | — | **pousa** |
| qualquer outro | — | mantém posição |

O `Open_Palm` tem dois papéis. Não é conflito — os estados são exclusivos — mas
é uma armadilha para quem for mexer no vocabulário sem ler os dois estados.

**Os comandos direcionais valem no ato; `Thumb_Down` e `Open_Palm` exigem
confirmação** (`command_confirm_cycles`, 10 ciclos = meio segundo a 20 Hz). A
assimetria é deliberada: andar para a frente se corrige sozinho no ciclo
seguinte, pousar não.

Enquanto obedece, o drone também **mantém a mão centrada no quadro** por PID: o
erro horizontal vira guinada, o vertical vira subida. É o que faz ele acompanhar
o operador sem que seja preciso mandar.

## Máquina de estados

```
ARMING → TAKEOFF → SEARCH HAND → GESTURE CONTROL ──"LAND NOW"──→ PRECISION LANDING
                       ↑              │                                 ↓
                       └──────────────┘                           TAKEOFF AGAIN
                        ("LOST HAND")  │                                │
                                       └──"GO HOME"──→ RETURN HOME → LAND AND DISARM
                       ↑                                                      ↓
                       └──────────────────────────────────────────────── FINISHED
```

Três decisões que diferem de 2025:

- **`TAKEOFF AGAIN` não reancora o referencial.** `TakeoffState()` chama
  `setHomePosition`, o que é necessário na decolagem inicial e destrutivo na
  redecolagem — a origem do mundo pularia para onde o drone pousou. Foi o bug
  que fez a fase 1 pousar na mesma base indefinidamente.
- **Perder a mão volta para a busca**, em vez de continuar controlando com a
  última posição vista. O operador pode ter saído do quadro porque o drone girou
  demais, e girar de volta é exatamente o que resolve.
- **O desarme final é um estado**, não um `on_exit`. Em 2025 era `land()` +
  `disarmSync()` dentro do `on_exit`, e `disarmSync` é um laço bloqueante — no
  callback do timer da FSM ele congela o executor, e a telemetria para junto.

## O que esta fase escreve, e o que ela apenas usa

| Peça | Onde vive |
|---|---|
| Reconhecimento de gestos (MediaPipe) | [`cv_nodes/gesture_detector`](https://github.com/Equipe-eVTOL-ITA/cv_nodes) |
| Armar, decolar, varrer em guinada, pousar, voltar, desarmar | [`stdstates`](https://github.com/Equipe-eVTOL-ITA/stdstates) |
| **Cola entre detector e missão** | `include/fase3/gesture_fase3.hpp` |
| **Vocabulário de gestos** | `include/fase3/gesture_commands.hpp` |
| **"Achei a mão?"** | `include/fase3/states/search_hand_state.hpp` |
| **O controle propriamente dito** | `include/fase3/states/gesture_control_state.hpp` |

## Testes

```bash
colcon test --packages-select fase3 --ctest-args -R test_gesture_commands
```

12 testes do vocabulário. Em 2025 essa regra vivia num método privado de um
estado que exigia um `Drone` para ser instanciado — conferir para que lado o
drone ia significava subir simulação e olhar.

Para regressão sem hardware, grave um bag de uma sessão boa e reproduza:

```bash
ros2 bag play <bag> --topics /gesture_detector/gestures \
                             /gesture_detector/hand_location
ros2 launch fase3 replay.launch.py
```

## Antes do primeiro voo

1. **Meça a banda** do enlace (`ros2 topic bw`), e ajuste `processing_frequency`
   e `debug_jpeg_quality` a ela.
2. **Confira o `video_source`** da câmera do drone no `flight.yaml`.
3. **Ajuste a posição de casa**, que em voo raramente é a origem.
