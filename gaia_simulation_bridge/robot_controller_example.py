#!/usr/bin/env python3
"""
Exemplo de como controlar o robô Cylidrone programaticamente.
Demonstra diferentes padrões de movimento.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class RobotController(Node):
    """Controlador de exemplo para o Cylidrone."""
    
    def __init__(self):
        super().__init__('robot_controller_example')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Robot Controller Example iniciado')
    
    def send_velocity(self, vx=0.0, vy=0.0, w=0.0, duration=1.0):
        """
        Envia comando de velocidade por um tempo determinado.
        
        Args:
            vx: Velocidade linear em x (m/s)
            vy: Velocidade linear em y (m/s)
            w: Velocidade angular em z (rad/s)
            duration: Duração do comando (segundos)
        """
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration and rclpy.ok():
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def stop(self):
        """Para o robô."""
        self.send_velocity(0.0, 0.0, 0.0, 0.1)
    
    def move_forward(self, speed=0.3, duration=2.0):
        """Move o robô para frente."""
        self.get_logger().info(f'Movendo para frente a {speed} m/s por {duration}s')
        self.send_velocity(vx=speed, duration=duration)
        self.stop()
    
    def move_backward(self, speed=0.3, duration=2.0):
        """Move o robô para trás."""
        self.get_logger().info(f'Movendo para trás a {speed} m/s por {duration}s')
        self.send_velocity(vx=-speed, duration=duration)
        self.stop()
    
    def move_left(self, speed=0.3, duration=2.0):
        """Move o robô lateralmente à esquerda (apenas modo shim)."""
        self.get_logger().info(f'Movendo lateralmente à esquerda a {speed} m/s por {duration}s')
        self.send_velocity(vy=speed, duration=duration)
        self.stop()
    
    def move_right(self, speed=0.3, duration=2.0):
        """Move o robô lateralmente à direita (apenas modo shim)."""
        self.get_logger().info(f'Movendo lateralmente à direita a {speed} m/s por {duration}s')
        self.send_velocity(vy=-speed, duration=duration)
        self.stop()
    
    def rotate_left(self, angular_speed=0.5, duration=2.0):
        """Gira o robô à esquerda."""
        self.get_logger().info(f'Girando à esquerda a {angular_speed} rad/s por {duration}s')
        self.send_velocity(w=angular_speed, duration=duration)
        self.stop()
    
    def rotate_right(self, angular_speed=0.5, duration=2.0):
        """Gira o robô à direita."""
        self.get_logger().info(f'Girando à direita a {angular_speed} rad/s por {duration}s')
        self.send_velocity(w=-angular_speed, duration=duration)
        self.stop()
    
    def circle(self, linear_speed=0.2, angular_speed=0.3, duration=10.0):
        """Move o robô em círculo."""
        self.get_logger().info(f'Movendo em círculo por {duration}s')
        self.send_velocity(vx=linear_speed, w=angular_speed, duration=duration)
        self.stop()
    
    def figure_eight(self, speed=0.2, radius=1.0):
        """Move o robô em forma de 8."""
        self.get_logger().info('Executando movimento em forma de 8')
        
        # Calcular velocidade angular para o raio desejado
        w = speed / radius
        
        # Primeira metade do 8 (sentido horário)
        self.send_velocity(vx=speed, w=-w, duration=math.pi * radius / speed)
        
        # Segunda metade do 8 (sentido anti-horário)
        self.send_velocity(vx=speed, w=w, duration=math.pi * radius / speed)
        
        self.stop()
    
    def square(self, side_length=1.0, speed=0.3):
        """Move o robô em um quadrado (apenas modo shim)."""
        self.get_logger().info(f'Executando movimento em quadrado (lado={side_length}m)')
        
        duration_per_side = side_length / speed
        
        for i in range(4):
            # Mover para frente
            self.get_logger().info(f'Lado {i+1}/4')
            self.send_velocity(vx=speed, duration=duration_per_side)
            self.stop()
            time.sleep(0.5)
            
            # Girar 90 graus
            self.rotate_left(angular_speed=0.785, duration=2.0)
            time.sleep(0.5)
        
        self.get_logger().info('Quadrado completo!')
    
    def diagonal_movement(self, speed=0.3, duration=3.0):
        """Movimento diagonal (apenas modo shim)."""
        self.get_logger().info('Movendo em diagonal')
        # 45 graus = velocidades iguais em x e y
        vx = vy = speed / math.sqrt(2)
        self.send_velocity(vx=vx, vy=vy, duration=duration)
        self.stop()


def main(args=None):
    """Função principal com exemplos de uso."""
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        # Aguardar um momento para garantir que tudo está conectado
        controller.get_logger().info('Aguardando 2 segundos antes de iniciar...')
        time.sleep(2)
        
        # Exemplo 1: Movimento básico para frente e para trás
        controller.get_logger().info('=== Exemplo 1: Frente e Trás ===')
        controller.move_forward(speed=0.3, duration=3.0)
        time.sleep(1)
        controller.move_backward(speed=0.3, duration=3.0)
        time.sleep(1)
        
        # Exemplo 2: Rotação
        controller.get_logger().info('=== Exemplo 2: Rotações ===')
        controller.rotate_left(angular_speed=0.5, duration=2.0)
        time.sleep(1)
        controller.rotate_right(angular_speed=0.5, duration=2.0)
        time.sleep(1)
        
        # Exemplo 3: Movimento em círculo
        controller.get_logger().info('=== Exemplo 3: Círculo ===')
        controller.circle(linear_speed=0.2, angular_speed=0.3, duration=8.0)
        time.sleep(1)
        
        # Exemplo 4: Movimento lateral (apenas modo shim)
        # Descomente se estiver usando modo shim
        # controller.get_logger().info('=== Exemplo 4: Lateral (apenas shim) ===')
        # controller.move_left(speed=0.3, duration=2.0)
        # time.sleep(1)
        # controller.move_right(speed=0.3, duration=2.0)
        # time.sleep(1)
        
        # Exemplo 5: Quadrado (apenas modo shim)
        # Descomente se estiver usando modo shim
        # controller.get_logger().info('=== Exemplo 5: Quadrado (apenas shim) ===')
        # controller.square(side_length=1.0, speed=0.3)
        
        controller.get_logger().info('=== Todos os exemplos concluídos! ===')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrompido pelo usuário')
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
