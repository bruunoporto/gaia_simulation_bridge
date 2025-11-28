#!/bin/bash

# Script para testar o controle do robô com comandos básicos

echo "=== Gaia Simulation Test Script ==="
echo ""
echo "Este script envia comandos de teste para o robô."
echo "Certifique-se de que a simulação está rodando!"
echo ""

# Função para enviar comando de velocidade
send_cmd() {
    local vx=$1
    local vy=$2
    local w=$3
    local duration=$4
    
    echo "Enviando comando: vx=$vx, vy=$vy, w=$w por $duration segundos..."
    
    timeout ${duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $vx, y: $vy, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $w}}" \
        --rate 10
}

# Menu
while true; do
    echo ""
    echo "Escolha um comando:"
    echo "1) Mover para frente"
    echo "2) Mover para trás"
    echo "3) Girar à esquerda"
    echo "4) Girar à direita"
    echo "5) Mover lateral esquerda (apenas shim)"
    echo "6) Mover lateral direita (apenas shim)"
    echo "7) Parar"
    echo "8) Círculo"
    echo "9) Sair"
    echo ""
    read -p "Opção: " option
    
    case $option in
        1)
            send_cmd 0.3 0.0 0.0 3
            ;;
        2)
            send_cmd -0.3 0.0 0.0 3
            ;;
        3)
            send_cmd 0.0 0.0 0.5 3
            ;;
        4)
            send_cmd 0.0 0.0 -0.5 3
            ;;
        5)
            send_cmd 0.0 0.3 0.0 3
            ;;
        6)
            send_cmd 0.0 -0.3 0.0 3
            ;;
        7)
            send_cmd 0.0 0.0 0.0 1
            ;;
        8)
            send_cmd 0.2 0.0 0.3 5
            ;;
        9)
            echo "Parando robô..."
            send_cmd 0.0 0.0 0.0 1
            echo "Saindo..."
            exit 0
            ;;
        *)
            echo "Opção inválida!"
            ;;
    esac
done
