# Guia de Início Rápido - Gaia Simulation Bridge

Este guia mostra como começar a usar a simulação do Cylidrone com os controles do Gaia.

## 1. Compilar o Pacote

```bash
cd ~/xr4000
colcon build --packages-select gaia_simulation_bridge cylidrone_simulation
source install/setup.bash
```

## 2. Iniciar a Simulação

```bash
ros2 launch gaia_simulation_bridge gaia_simulation.launch.py
```

### Escolher o Modo de Controle

**Via argumento de launch:**

```bash
# Modo Shim (cinemática completa) - padrão
ros2 launch gaia_simulation_bridge gaia_simulation.launch.py control_mode:=shim

# Modo Diferencial (simplificado)
ros2 launch gaia_simulation_bridge gaia_simulation.launch.py control_mode:=differential
```

**Via tópico ROS (trocar modo durante execução):**

```bash
# Trocar para modo shim
ros2 topic pub /control_mode std_msgs/msg/String "data: 'shim'" --once

# Trocar para modo diferencial
ros2 topic pub /control_mode std_msgs/msg/String "data: 'differential'" --once
```

### Comparação dos Modos

| Modo | Movimento Omnidirecional | Complexidade | Melhor Para |
|------|-------------------------|--------------|-------------|
| **Shim** | ✅ Sim (x, y, θ) | Maior | Navegação precisa, espaços apertados |
| **Differential** | ❌ Não (apenas x, θ) | Menor | Movimento simples, linha reta e curvas |

## 3. Controlar o Robô

### Método 1: Teleoperação com Teclado (Recomendado)

Instale o pacote de teleoperação (se ainda não tiver):
```bash
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard
```

Execute em outro terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controles:**
- `i` - Mover para frente
- `k` - Parar
- `,` - Mover para trás
- `j` - Girar à esquerda
- `l` - Girar à direita
- `u` - Diagonal frente-esquerda
- `o` - Diagonal frente-direita
- `m` - Diagonal trás-esquerda
- `.` - Diagonal trás-direita

**No modo shim apenas:**
- `J` - Mover lateralmente à esquerda
- `L` - Mover lateralmente à direita

### Método 2: Script de Teste

Execute o script interativo:
```bash
cd ~/xr4000/src/gaia_simulation_bridge/scripts
./test_control.sh
```

Escolha entre os comandos pré-definidos no menu.

### Método 3: Comandos Manuais via Topic

```bash
# Mover para frente a 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10

# Girar à esquerda
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
  --rate 10

# Parar (Ctrl+C para cancelar o comando anterior, depois:)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
```

## 4. Monitoramento

### Visualizar Estado das Juntas

```bash
ros2 topic echo /joint_states
```

### Listar Controladores Ativos

```bash
ros2 control list_controllers
```

Você deve ver:
- `joint_state_broadcaster` - ativo
- `position_controllers` - ativo
- `velocity_controllers` - ativo

### Verificar Comandos Recebidos

```bash
# Ver comandos de velocidade
ros2 topic echo /cmd_vel

# Ver comandos para steers
ros2 topic echo /position_controllers/commands

# Ver comandos para wheels
ros2 topic echo /velocity_controllers/commands
```

### Visualizar em RViz (Opcional)

Em outro terminal:
```bash
rviz2
```

Adicione:
1. `RobotModel` - para ver o modelo do robô
2. `TF` - para ver os frames de coordenadas
3. Configure `Fixed Frame` para `base_footprint`

## 5. Próximos Passos

### Integrar com Navegação

O bridge pode ser usado com o Nav2:

```bash
# Instalar Nav2 (se necessário)
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup

# Seu nó de navegação pode publicar em /cmd_vel e o bridge traduz para a simulação
```

### Criar Seu Próprio Controlador

Você pode criar um nó que publica em `/cmd_vel`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def send_velocity(self, vx, vy, w):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        self.publisher.publish(msg)
```

### Ajustar Parâmetros

Edite os arquivos de configuração em:
- `~/xr4000/src/gaia_simulation_bridge/config/gaia_params.yaml`
- `~/xr4000/src/gaia_simulation_bridge/config/gaia_params_differential.yaml`

Após modificar, recompile:
```bash
colcon build --packages-select gaia_simulation_bridge
source install/setup.bash
```

## Troubleshooting Rápido

### Simulação não inicia
```bash
# Verifique se Gazebo está instalado
gazebo --version

# Instale se necessário
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
```

### Robô não responde a comandos
```bash
# 1. Verifique se o bridge está rodando
ros2 node list | grep gaia_bridge

# 2. Verifique os controladores
ros2 control list_controllers

# 3. Reinicie a simulação
# Ctrl+C no terminal da simulação, depois relance
```

### Movimento errático
- Reduza as velocidades nos comandos (tente valores menores que 0.5)
- Aumente `update_rate` no arquivo de configuração para 100 Hz

## Exemplos de Trajetórias

### Quadrado (shim mode)
```bash
# Terminal 1: Iniciar simulação
ros2 launch gaia_simulation_bridge gaia_simulation_shim.launch.py

# Terminal 2: Executar trajetória
for i in {1..4}; do
  # Mover para frente
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --rate 10 &
  PID=$!
  sleep 3
  kill $PID
  
  # Parar
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --once
  sleep 0.5
  
  # Girar 90 graus
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.785}}" \
    --rate 10 &
  PID=$!
  sleep 2
  kill $PID
  
  # Parar
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --once
  sleep 0.5
done
```

## Recursos Adicionais

- README completo: `~/xr4000/src/gaia_simulation_bridge/README.md`
- Código fonte: `~/xr4000/src/gaia_simulation_bridge/gaia_simulation_bridge/`
- Configurações: `~/xr4000/src/gaia_simulation_bridge/config/`

## Suporte

Para problemas ou dúvidas, verifique:
1. Os logs do nó: `ros2 node info /gaia_bridge_node`
2. O estado dos controladores: `ros2 control list_controllers`
3. Os tópicos ativos: `ros2 topic list`
