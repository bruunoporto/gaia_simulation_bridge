#!/bin/bash
# Script para testar movimentação no RViz

echo "=== Enviando comandos de velocidade ==="
echo "Movimento para frente por 5 segundos..."

# Publicar velocidade para frente
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

sleep 2

echo "Rotação no lugar por 5 segundos..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"

sleep 2

echo "Movimento circular..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.3}}"

echo "Para parar, publique velocidade zero:"
echo "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\""
