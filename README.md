# ROS 2 Mini Sumo Simulator 🤖🥋

Simulador de robôs de **Mini Sumô** desenvolvido com **ROS 2 Jazzy** e **Gazebo Harmonic**.

O projeto permite simular dois robôs autônomos em um Dohyo, incluindo motores, sensores de distância, posicionamento inicial aleatório e uma interface para controlar o início de cada luta.

A proposta é utilizar o ambiente como uma plataforma de testes para diferentes estratégias de controle e tomada de decisão antes da implementação no robô físico.

---

## 📌 Visão geral

O simulador possui dois robôs Mini Sumô independentes:

- `carro1`
- `carro2`

Cada robô possui:

- duas rodas motorizadas;
- controle por torque;
- motor N20 como referência de hardware;
- um sensor ToF frontal;
- cinco sensores Sharp direcionais;
- namespace ROS 2 independente;
- controller próprio;
- sensores próprios.

Além disso, o simulador possui uma interface para:

- gerar uma nova configuração de luta;
- posicionar os robôs aleatoriamente;
- manter posições válidas dentro do Dohyo;
- iniciar os dois agentes simultaneamente.

---

# 🏟️ Arena

A arena utilizada representa um Dohyo de Mini Sumô.

Principais dimensões utilizadas no modelo:

| Item | Valor |
|---|---:|
| Diâmetro total do Dohyo | 770 mm |
| Raio total | 385 mm |
| Raio da região preta | 360 mm |
| Altura do Dohyo | 25 mm |
| Área externa (Yochi) | 1,8 m × 1,8 m |

O arquivo principal da arena está em:

```text
worlds/pista_dupla.sdf
```

O nome do mundo no Gazebo é:

```text
pista
```

---

# 🤖 Robô

O robô simulado possui aproximadamente:

| Característica | Valor |
|---|---:|
| Comprimento | 100 mm |
| Largura | 100 mm |
| Altura | 84 mm |
| Massa total aproximada | 500 g |
| Diâmetro da roda | 32 mm |
| Raio da roda | 16 mm |

A geometria principal está definida em:

```text
urdf/carro_multi.xacro
```

---

# ⚙️ Motores

O modelo utiliza como referência motores **N20 com encoder**.

Parâmetros utilizados:

| Característica | Valor |
|---|---:|
| Tensão nominal | 6 V |
| Rotação de saída | 750 RPM |
| Redução | 20:1 |
| Torque de stall | 0,009 N·m |
| Encoder no motor | 7 PPR |
| Encoder após redução | 140 PPR |

O controle das rodas é realizado utilizando interfaces de:

```text
effort
```

O controller utilizado é:

```text
effort_controllers/JointGroupEffortController
```

Cada robô possui o tópico:

```text
/carro1/wheel_effort_controller/commands
/carro2/wheel_effort_controller/commands
```

Uma mensagem de torque possui o formato:

```yaml
data:
- torque_roda_esquerda
- torque_roda_direita
```

Exemplo:

```yaml
data:
- 0.009
- 0.009
```

---

# 📡 Sensores

Cada robô possui **6 sensores de distância**.

## ToF frontal

Localizado na frente do robô.

```text
/tof/front/scan
```

Configuração atual:

| Parâmetro | Valor |
|---|---:|
| Campo de visão | 0,10 rad |
| Raios | 5 |
| Alcance mínimo | 0,04 m |
| Alcance máximo | 2,0 m |
| Frequência | 20 Hz |

---

## Sharp frontal esquerdo

Orientação aproximada:

```text
+30°
```

Tópico:

```text
/sharp/front_left/scan
```

---

## Sharp frontal direito

Orientação aproximada:

```text
-30°
```

Tópico:

```text
/sharp/front_right/scan
```

---

## Sharp lateral esquerdo

Orientação:

```text
+90°
```

Tópico:

```text
/sharp/left/scan
```

---

## Sharp lateral direito

Orientação:

```text
-90°
```

Tópico:

```text
/sharp/right/scan
```

---

## Sharp traseiro

Orientação:

```text
180°
```

Tópico:

```text
/sharp/rear/scan
```

---

## Disposição aproximada

```text
                         FRENTE

                           ToF
                            |
                            |
                Sharp +30°  |  Sharp -30°
                         \   |   /
                          \  |  /
                           \ | /
                            \|/

Sharp +90°   <---------- [ ROBÔ ] ---------->   Sharp -90°

                            |
                            |
                      Sharp traseiro
                           180°
```

Os sensores são definidos principalmente em:

```text
urdf/sensores_multi.xacro
```

> Atualmente, a estratégia `ataque_tof.py` utiliza somente o ToF frontal para tomada de decisão. Os sensores Sharp já existem na simulação, mas ainda serão incorporados às estratégias de controle.

---

# 🎲 Inicialização aleatória das lutas

As posições dos robôs podem ser geradas aleatoriamente.

O algoritmo verifica a geometria do robô antes de aceitar uma pose.

A posição só é aceita quando:

- todos os cantos do robô permanecem dentro do Dohyo;
- o robô permanece na região permitida para sua posição inicial;
- o corpo completo respeita a região das linhas Shikiri.

Também é sorteado um ângulo inicial:

```text
-π ≤ yaw ≤ +π
```

Isso permite testar os agentes em diferentes situações iniciais em vez de executar todas as lutas nas mesmas condições.

---

# 🎮 Painel de controle

O projeto possui uma pequena interface gráfica feita com Tkinter.

Execute:

```bash
ros2 run carro_sim controle_luta
```

A interface possui dois comandos principais:

```text
┌──────────────────────────────┐
│          MINI SUMÔ           │
│                              │
│        [ NOVA LUTA ]         │
│                              │
│       [ INICIAR LUTA ]       │
│                              │
│          Luta #1             │
│                              │
│ Carro 1: x=... y=... θ=...   │
│ Carro 2: x=... y=... θ=...   │
│                              │
│ Aguardando início.           │
└──────────────────────────────┘
```

## NOVA LUTA

O botão:

```text
NOVA LUTA
```

realiza automaticamente:

```text
parar motores
      ↓
pausar Gazebo
      ↓
sortear poses válidas
      ↓
reposicionar carro 1
      ↓
reposicionar carro 2
      ↓
resetar agentes
      ↓
retomar Gazebo
```

Os robôs permanecem parados após o reset.

---

## INICIAR LUTA

O botão:

```text
INICIAR LUTA
```

publica:

```text
/iniciar_luta
```

Os agentes recebem o sinal simultaneamente e passam a executar suas estratégias.

---

# 🧠 Estratégia atual

A estratégia básica está em:

```text
carro_sim/ataque_tof.py
```

O comportamento atual é:

```text
NOVA LUTA
    ↓
AGUARDANDO
    ↓
INICIAR LUTA
    ↓
ToF liberado
    ↓
adversário detectado?
    ├── não → permanece parado
    │
    └── sim
          ↓
        ATAQUE
          ↓
   torque nas duas rodas
```

Atualmente o ataque aplica:

```text
0.009 N·m
```

em cada roda.

A estratégia ainda será expandida para utilizar os sensores laterais e traseiro.

---

# 📁 Estrutura do projeto

```text
carro_sim/
│
├── carro_sim/
│   ├── __init__.py
│   ├── ataque_tof.py
│   ├── controle_luta.py
│   ├── motor_n20.py
│   └── ...
│
├── config/
│   └── controllers_multi.yaml
│
├── launch/
│   └── dois_carros.launch.py
│
├── resource/
│
├── urdf/
│   ├── carro_multi.xacro
│   └── sensores_multi.xacro
│
├── worlds/
│   └── pista_dupla.sdf
│
├── package.xml
├── setup.py
├── setup.cfg
├── .gitignore
└── README.md
```

---

# 💻 Requisitos

Ambiente utilizado durante o desenvolvimento:

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
Python 3
```

Pacotes principais:

```bash
sudo apt update

sudo apt install \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  python3-tk
```

---

# 📥 Instalação

Crie um workspace ROS 2:

```bash
mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy/src
```

Clone o projeto:

```bash
git clone https://github.com/MaysaIzaias/ros2-mini-sumo-simulator.git carro_sim
```

Entre no workspace:

```bash
cd ~/ros2_jazzy
```

Carregue o ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Instale dependências declaradas pelo projeto:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

Compile:

```bash
colcon build \
  --packages-select carro_sim \
  --symlink-install
```

Carregue o workspace:

```bash
source ~/ros2_jazzy/install/setup.bash
```

---

# 🚀 Executando a simulação

No primeiro terminal:

```bash
cd ~/ros2_jazzy

source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash

ros2 launch carro_sim dois_carros.launch.py
```

O Gazebo deverá abrir com:

```text
Dohyo
+
carro1
+
carro2
```

---

# 🕹️ Abrindo o painel

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash

ros2 run carro_sim controle_luta
```

---

# 🤖 Executando os agentes

## Carro 1

Abra outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash

ros2 run carro_sim ataque_tof \
  --ros-args \
  -r __ns:=/carro1 \
  -p use_sim_time:=true
```

## Carro 2

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash

ros2 run carro_sim ataque_tof \
  --ros-args \
  -r __ns:=/carro2 \
  -p use_sim_time:=true
```

Agora:

```text
1. clique em NOVA LUTA
2. espere os robôs serem reposicionados
3. clique em INICIAR LUTA
```

---

# 📡 Verificando sensores

Liste os tópicos:

```bash
ros2 topic list | grep -E "tof|sharp"
```

ToF do carro 1:

```bash
ros2 topic echo /carro1/tof/front/scan --once
```

ToF do carro 2:

```bash
ros2 topic echo /carro2/tof/front/scan --once
```

Exemplo:

```yaml
ranges:
- 0.297
- 0.296
- 0.296
- 0.296
- 0.297
```

Os valores estão em metros.

Por exemplo:

```text
0.297 m = 29,7 cm
```

Quando aparece:

```text
.inf
```

o raio não encontrou nenhum objeto dentro de seu alcance.

---

# 🌉 Bridge Gazebo → ROS 2

Os sensores são criados no Gazebo e precisam estar disponíveis também no ROS 2.

Para verificar os tópicos diretamente no Gazebo:

```bash
gz topic -l | grep carro
```

Se os tópicos existirem no Gazebo, mas não aparecerem no ROS 2, é possível testar manualmente a bridge dos ToFs:

```bash
ros2 run ros_gz_bridge parameter_bridge \
'/carro1/tof/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
'/carro2/tof/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```

Depois:

```bash
ros2 topic echo /carro1/tof/front/scan --once
```

---

# 🔧 Verificando controllers

Carro 1:

```bash
ros2 control list_controllers \
  -c /carro1/controller_manager
```

Carro 2:

```bash
ros2 control list_controllers \
  -c /carro2/controller_manager
```

O esperado é encontrar:

```text
joint_state_broadcaster    active
wheel_effort_controller    active
```

Para verificar as interfaces:

```bash
ros2 control list_hardware_interfaces \
  -c /carro1/controller_manager
```

Devem aparecer:

```text
left_wheel_joint/effort     [available] [claimed]
right_wheel_joint/effort    [available] [claimed]
```

---

# 🧪 Teste manual dos motores

Para aplicar torque diretamente no carro 1:

```bash
ros2 topic pub --rate 50 \
/carro1/wheel_effort_controller/commands \
std_msgs/msg/Float64MultiArray \
"{data: [0.009, 0.009]}"
```

Para parar:

```bash
ros2 topic pub --once \
/carro1/wheel_effort_controller/commands \
std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0]}"
```

O mesmo pode ser feito no carro 2 substituindo:

```text
carro1
```

por:

```text
carro2
```

---

# 🔄 Recompilando após alterações

Sempre que arquivos Python, launch, URDF ou configurações forem alterados:

```bash
cd ~/ros2_jazzy

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select carro_sim \
  --symlink-install

source install/setup.bash
```

---

# 🧹 Reiniciando o Gazebo

Caso seja necessário fechar uma simulação antiga:

```bash
pkill -f "gz sim"
```

Depois execute novamente:

```bash
ros2 launch carro_sim dois_carros.launch.py
```

---

# 🐛 Problemas comuns

## `Package 'carro_sim' not found`

Carregue o workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
```

---

## Sensor não publica no ROS 2

Primeiro veja se existe no Gazebo:

```bash
gz topic -l | grep tof
```

Se existir no Gazebo e não no ROS 2, verifique a bridge.

---

## Controller não está ativo

Execute:

```bash
ros2 control list_controllers \
  -c /carro1/controller_manager
```

O `wheel_effort_controller` deve estar:

```text
active
```

---

## Alterei um arquivo mas nada mudou

Recompile:

```bash
cd ~/ros2_jazzy

colcon build \
  --packages-select carro_sim \
  --symlink-install

source install/setup.bash
```

Depois reinicie a simulação.

---

# 📷 Imagens do projeto

Uma estrutura sugerida é:

```text
docs/
└── images/
    ├── simulacao.png
    ├── robo.png
    ├── sensores.png
    └── painel.png
```

Depois é possível adicionar imagens ao README usando:

```markdown
![Simulação no Gazebo](docs/images/simulacao.png)
```

### Simulação

<!-- Adicione um print em docs/images/simulacao.png -->

### Robô e sensores

<!-- Adicione um print em docs/images/sensores.png -->

### Painel de controle

<!-- Adicione um print em docs/images/painel.png -->

---

# 🛣️ Próximas etapas

O projeto ainda está em desenvolvimento.

Algumas evoluções planejadas:

- integrar os cinco sensores Sharp à estratégia;
- criar comportamento de busca do adversário;
- implementar diferentes controladores;
- adicionar sensores de linha para detectar a borda;
- detectar automaticamente vitória e derrota;
- registrar duração de cada luta;
- registrar primeiro contato;
- salvar posição inicial de cada robô;
- utilizar seeds para reproduzir exatamente uma luta;
- executar múltiplas lutas automaticamente;
- exportar resultados para CSV;
- comparar estratégias estatisticamente;
- adicionar sistema de replay.

---

# 🎯 Objetivo futuro

A ideia é transformar o simulador em um ambiente experimental para comparar diferentes estratégias de controle de Mini Sumô.

Exemplo:

```text
Nova luta
    ↓
pose aleatória
    ↓
agente A × agente B
    ↓
resultado
    ↓
métricas
    ↓
próxima luta
```

Permitindo executar centenas ou milhares de combates e analisar métricas como:

```text
vitórias
derrotas
empates
tempo médio de luta
primeiro contato
posição inicial
estratégia utilizada
```
