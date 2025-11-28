#!/bin/bash

echo "=== Limpando processos anteriores ==="
pkill -9 -f "joint_state_publisher|robot_state_publisher|gaia_bridge_node" 2>/dev/null
sleep 2

echo "=== Lançando sistema ==="
cd /home/bruno/xr4000
source install/setup.bash
ros2 launch gaia_simulation_bridge gaia_rviz.launch.py control_mode:=differential rviz:=false

# Modo shim aqui
#source /home/bruno/xr4000/install/setup.bash && ros2 launch gaia_simulation_bridge gaia_rviz.launch.py control_mode:=shim rviz:=false