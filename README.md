# Gaia Simulation Bridge

Pacote ROS2 que integra os controles do `gaia_controller` com a simulação do `cylidrone` no Gazebo.

## Descrição

Este pacote implementa uma ponte (bridge) entre comandos de velocidade (`cmd_vel`) e os controladores individuais das juntas do robô Cylidrone na simulação. Suporta dois modos de controle cinemático:

1. **Modo Shim**: Cinemática completa de 4 rodas com direção independente (steering)
2. **Modo Diferencial**: Cinemática diferencial simplificada (rodas de direção fixas em 0°)

A implementação é baseada nas bibliotecas de cinemática do `gaia_controller`, portada para Python.

## Estrutura do Pacote

```
gaia_simulation_bridge/
├── config/
│   ├── controller_config.yaml          # Configuração dos controladores Gazebo
│   ├── gaia_params.yaml               # Parâmetros para modo shim
│   └── gaia_params_differential.yaml  # Parâmetros para modo diferencial
├── launch/
│   ├── gaia_simulation_shim.launch.py        # Launch em modo shim
│   └── gaia_simulation_differential.launch.py # Launch em modo diferencial
└── gaia_simulation_bridge/
    ├── __init__.py
    └── gaia_bridge_node.py            # Nó principal do bridge
```

## Instalação

1. Certifique-se de que você tem ROS2 instalado (Humble ou superior recomendado)

2. Clone este repositório no seu workspace ROS2:
```bash
cd ~/xr4000/src
```

3. Instale as dependências:
```bash
cd ~/xr4000
rosdep install --from-paths src --ignore-src -r -y
```

4. Compile o workspace:
```bash
colcon build --packages-select gaia_simulation_bridge cylidrone_simulation
```

5. Source o workspace:
```bash
source install/setup.bash
```

## Uso

### Modo Shim (Cinemática Completa)

Para iniciar a simulação com controle shim:

```bash
ros2 launch gaia_simulation_bridge gaia_simulation_shim.launch.py
```

Este modo utiliza a cinemática completa de 4 rodas com direção independente, permitindo movimento omnidirecional.

### Modo Diferencial

Para iniciar a simulação com controle diferencial:

```bash
ros2 launch gaia_simulation_bridge gaia_simulation_differential.launch.py
```

Este modo simplifica o controle para um modelo diferencial clássico, onde as rodas de direção ficam fixas e apenas as rodas motrizes são controladas.

### Controlando o Robô

Em outro terminal, você pode enviar comandos de velocidade:

```bash
# Mover para frente
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Movimento lateral (apenas em modo shim)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Ou use o `teleop_twist_keyboard`:

```bash
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Parâmetros

Os parâmetros geométricos do robô podem ser ajustados nos arquivos YAML em `config/`:

- `control_mode`: `'shim'` ou `'differential'`
- `wheels_radius`: Raio das rodas (metros)
- `caster_offset`: Offset do caster (metros)
- `h_positions`: Array com 4 distâncias do centro às rodas
- `beta_angles`: Array com 4 ângulos das rodas (radianos)
- `update_rate`: Taxa de atualização do controlador (Hz)

### Parâmetros do Cylidrone (default)

```yaml
wheels_radius: 0.055        # 5.5 cm
caster_offset: 0.11         # 11 cm
h_positions: [0.54, 0.54, 0.54, 0.54]  # 54 cm
beta_angles: [0.0, 1.5708, 3.14159, -1.5708]  # 0°, 90°, 180°, -90°
```

## Arquitetura

### Fluxo de Dados

```
cmd_vel (Twist) 
    ↓
[Gaia Bridge Node]
    ├─→ Cinemática Inversa (shim ou differential)
    ↓
[Comandos de Junta]
    ├─→ position_controllers (steers)
    └─→ velocity_controllers (wheels)
    ↓
[Gazebo Simulation]
```

### Tópicos ROS2

**Subscribers:**
- `/cmd_vel` (geometry_msgs/Twist): Comandos de velocidade do robô
- `/joint_states` (sensor_msgs/JointState): Estado atual das juntas

**Publishers:**
- `/position_controllers/commands` (Float64MultiArray): Comandos para juntas de direção
- `/velocity_controllers/commands` (Float64MultiArray): Comandos para rodas

## Cinemática

### Modo Shim

A cinemática shim implementa o modelo completo de 4 rodas com direção independente:

- Cada roda possui 2 motores: direção (steering) e rotação (wheel)
- Total de 8 motores controlados
- Permite movimento omnidirecional (x, y, θ)

**Cinemática Inversa:**
```
q̇ = J̃(φ) · ẋ
```

Onde:
- `q̇`: Velocidades das 8 juntas (4 steers + 4 wheels)
- `ẋ`: Velocidade do robô [vx, vy, ω]
- `φ`: Ângulos atuais dos steers
- `J̃`: Jacobiana que depende da geometria e dos ângulos atuais

### Modo Diferencial

Modelo simplificado de tração diferencial:

- Steers ficam fixos em 0°
- Apenas rodas esquerda/direita são controladas
- Movimento restrito ao plano (x, θ)

**Cinemática Inversa:**
```
v_left = v - ω
v_right = v + ω
```

## Troubleshooting

### O robô não se move

1. Verifique se os controladores foram carregados:
```bash
ros2 control list_controllers
```

2. Verifique se há comandos sendo publicados:
```bash
ros2 topic echo /cmd_vel
```

3. Verifique os logs do bridge node:
```bash
ros2 node info /gaia_bridge_node
```

### Movimento instável em modo shim

- Ajuste o parâmetro `update_rate` para um valor mais alto (ex: 100 Hz)
- Verifique se os valores de `h_positions` e `beta_angles` estão corretos

### Rodas giram muito rápido/devagar

- Ajuste o parâmetro `wheels_radius` para corresponder ao raio real das rodas no URDF

## Desenvolvimento

Para modificar ou estender a cinemática, edite o arquivo:
```
gaia_simulation_bridge/gaia_simulation_bridge/gaia_bridge_node.py
```

A classe `GaiaKinematics` contém toda a lógica cinemática e pode ser facilmente modificada ou estendida.

## Referências

- Baseado no `gaia_controller`: Controlador original em C++ para o robô Gaia
- Implementação da cinemática de robôs móveis com direção independente
- ROS2 Control framework

## Licença

MIT

## Autor

Bruno Porto
