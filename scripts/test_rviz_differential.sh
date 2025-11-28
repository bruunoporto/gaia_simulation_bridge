#!/bin/bash

# Script para testar visualização RViz no modo differential

echo "=== Testando visualização RViz (modo differential) ==="
echo ""
echo "Este script vai:"
echo "  1. Parar processos anteriores"
echo "  2. Lançar robot_state_publisher + gaia_bridge (sem RViz)"
echo "  3. Verificar se joint_states está sendo publicado"
echo ""

# Parar processos anteriores
echo "Parando processos anteriores..."
pkill -f "gaia_rviz.launch" 2>/dev/null
sleep 1

# Ir para workspace
cd /home/bruno/xr4000

# Source workspace
source install/setup.bash

# Lançar sistema em background
echo "Lançando sistema..."
ros2 launch gaia_simulation_bridge gaia_rviz.launch.py control_mode:=differential rviz:=false &
LAUNCH_PID=$!

# Aguardar inicialização
sleep 3

# Verificar joint_states
echo ""
echo "Verificando publicação de joint_states..."
timeout 2 ros2 topic echo /joint_states --once 2>&1 | head -20

echo ""
echo "Sistema rodando (PID: $LAUNCH_PID)"
echo ""
echo "Para visualizar:"
echo "  1. Abra RViz manualmente: rviz2"
echo "  2. Adicione display RobotModel"
echo "  3. Configure Fixed Frame = base_link"
echo ""
echo "Para testar movimento:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.1}, angular: {z: 0.0}}\" --once"
echo ""
echo "Para parar: kill $LAUNCH_PID"
