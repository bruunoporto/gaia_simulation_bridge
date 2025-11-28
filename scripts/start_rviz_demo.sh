#!/bin/bash
# Script para iniciar demonstração completa com RViz

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname $(dirname $(dirname $SCRIPT_DIR)))"

cd $WORKSPACE_DIR

# Source the workspace
source install/setup.bash

echo "=== Iniciando sistema com RViz ==="
echo ""
echo "1. O gaia_bridge_node está rodando e aguardando comandos em /cmd_vel"
echo "2. O joint_state_publisher publicará os estados das juntas"
echo "3. Abra outro terminal e execute:"
echo "   cd $WORKSPACE_DIR"
echo "   source install/setup.bash"
echo "   rviz2"
echo ""
echo "4. No RViz:"
echo "   - Mude Fixed Frame para: base_link"
echo "   - Clique em 'Add' -> RobotModel"
echo "   - Clique em 'Add' -> TF"
echo ""
echo "5. Para testar movimento, em outro terminal:"
echo "   cd $WORKSPACE_DIR"
echo "   source install/setup.bash"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}}\" --rate 10"
echo ""
echo "Iniciando sistema..."
echo ""

ros2 launch gaia_simulation_bridge gaia_rviz.launch.py
