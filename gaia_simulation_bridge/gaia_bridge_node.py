#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np


class GaiaKinematics:
    """
    Implementação da cinemática do Gaia para simulação.
    Baseada na implementação C++ do gaia_controller.
    """
    
    def __init__(self, r, b, h, beta):
        """
        Args:
            r: Raio das rodas (WHEELS_RADIUS)
            b: Offset do caster (CASTER_OFFSET_B)
            h: Array com 4 posições H (distâncias)
            beta: Array com 4 ângulos beta
        """
        self.r = r
        self.b = b
        self.h = np.array(h)
        self.beta = np.array(beta)
        
        # Calcular matriz Jp (8x3)
        self.Jp = np.zeros((8, 3))
        for i in range(4):
            self.Jp[2*i, 0] = 1.0
            self.Jp[2*i, 1] = 0.0
            self.Jp[2*i, 2] = -h[i] * np.sin(beta[i])
            
            self.Jp[2*i+1, 0] = 0.0
            self.Jp[2*i+1, 1] = 1.0
            self.Jp[2*i+1, 2] = h[i] * np.cos(beta[i])
        
        # Pseudo-inversa
        self.Jp_plus = np.linalg.pinv(self.Jp)
    
    def calculate_inverse_kinematics(self, robot_vel, current_phi_angles):
        """
        Calcula velocidades das juntas a partir da velocidade do robô.
        
        Args:
            robot_vel: numpy array [vx, vy, w]
            current_phi_angles: numpy array [phi0, phi1, phi2, phi3] - ângulos dos steers
            
        Returns:
            numpy array de 8 elementos com velocidades das juntas
        """
        J_tilde = np.zeros((8, 3))
        
        for i in range(4):
            phi = current_phi_angles[i]
            c_phi = np.cos(phi)
            s_phi = np.sin(phi)
            
            h = self.h[i]
            beta = self.beta[i]
            c_beta = np.cos(beta)
            s_beta = np.sin(beta)
            
            # Steer joint (índices pares: 0, 2, 4, 6)
            J_tilde[2*i, 0] = -s_phi / self.b
            J_tilde[2*i, 1] = c_phi / self.b
            J_tilde[2*i, 2] = h * (c_beta * c_phi + s_beta * s_phi) / self.b - 1.0
            
            # Wheel joint (índices ímpares: 1, 3, 5, 7)
            J_tilde[2*i+1, 0] = c_phi / self.r
            J_tilde[2*i+1, 1] = s_phi / self.r
            J_tilde[2*i+1, 2] = h * (c_beta * s_phi - s_beta * c_phi) / self.r
        
        return J_tilde @ robot_vel
    
    def calculate_inverse_kinematics_differential(self, robot_vel):
        """
        Calcula velocidades no modo diferencial.
        Apenas usa vx e w, ignora vy.
        
        Args:
            robot_vel: numpy array [vx, vy, w]
            
        Returns:
            numpy array de 8 elementos com velocidades das juntas
        """
        v = robot_vel[0]  # linear x
        w = robot_vel[2]  # angular z
        
        # Cinemática diferencial: v_left = v - w, v_right = v + w
        v_left = v - w
        v_right = v + w
        
        wheels_vel = np.zeros(8)
        
        # Steers ficam em zero (índices pares: 0, 2, 4, 6)
        wheels_vel[0] = 0.0
        wheels_vel[2] = 0.0
        wheels_vel[4] = 0.0
        wheels_vel[6] = 0.0
        
        # Wheels (índices ímpares: 1, 3, 5, 7)
        # Left wheels: 1 (front_left), 7 (rear_left)
        wheels_vel[1] = v_left
        wheels_vel[7] = v_left
        
        # Right wheels: 3 (front_right), 5 (rear_right)
        wheels_vel[3] = v_right
        wheels_vel[5] = v_right
        
        return wheels_vel
    
    def calculate_inverse_kinematics_shim(self, robot_vel, current_phi_angles):
        """
        Calcula velocidades usando o controlador SHIM.
        Alinha as rodas antes de mover, separa movimento linear de rotação.
        
        Args:
            robot_vel: numpy array [vx, vy, w]
            current_phi_angles: numpy array [phi0, phi1, phi2, phi3] - ângulos dos steers
            
        Returns:
            numpy array de 8 elementos com velocidades das juntas
        """
        v = robot_vel[0]  # linear velocity
        w = robot_vel[2]  # angular velocity
        shim_gain = 5.0
        shim_threshold = 0.1
        
        wheels_vel = np.zeros(8)
        
        turning = abs(w) > 0.001
        moving = abs(v) > 0.001
        
        if turning:
            # Turn Mode - align wheels to turn around center
            for i in range(4):
                current_angle = current_phi_angles[i]
                
                # Calculate joint position
                joint_x = self.h[i] * np.cos(self.beta[i])
                joint_y = self.h[i] * np.sin(self.beta[i])
                
                # Calculate initial wheel position (wheel facing forward, offset in Y)
                wheel_x = joint_x
                wheel_y = joint_y + self.b
                
                # Calculate angle to wheel center based on INITIAL position
                target_wheel_angle = np.arctan2(wheel_y, wheel_x)
                
                # Calculate actual wheel position considering current rotation
                wheel_x_rotated = joint_x + self.b * np.sin(current_angle)
                wheel_y_rotated = joint_y + self.b * np.cos(current_angle)
                wheel_radius = np.hypot(wheel_x_rotated, wheel_y_rotated)
                
                # Calculate error (normalize to [-pi, pi])
                error = self._normalize_angle(target_wheel_angle - current_angle)
                
                # Steering velocity (even indices: 0, 2, 4, 6)
                wheels_vel[2*i] = error * shim_gain
                
                # Driving velocity (odd indices: 1, 3, 5, 7)
                if abs(error) < shim_threshold:
                    # Aligned, drive - positive velocity moves robot CCW
                    wheels_vel[2*i + 1] = w * wheel_radius
                else:
                    # Not aligned, stop driving
                    wheels_vel[2*i + 1] = 0.0
        
        elif moving:
            # Move Mode - align wheels forward
            for i in range(4):
                target_angle = 0.0  # Forward
                current_angle = current_phi_angles[i]
                
                # Calculate error
                error = self._normalize_angle(target_angle - current_angle)
                
                # Steering velocity
                wheels_vel[2*i] = error * shim_gain
                
                # Driving velocity
                if abs(error) < shim_threshold:
                    wheels_vel[2*i + 1] = v
                else:
                    wheels_vel[2*i + 1] = 0.0
        
        else:
            # Stop
            wheels_vel.fill(0.0)
        
        return wheels_vel
    
    def _normalize_angle(self, angle):
        """Normaliza ângulo para o intervalo [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


class GaiaBridgeNode(Node):
    """
    Nó que faz a ponte entre comandos cmd_vel e os controladores da simulação.
    Suporta três modos: velocity (cinemática completa), differential e shim.
    """
    
    def __init__(self):
        super().__init__('gaia_bridge_node')
        
        # Declarar parâmetros
        self.declare_parameter('control_mode', 'velocity')  # 'velocity', 'differential' ou 'shim'
        self.declare_parameter('wheels_radius', 0.055)
        self.declare_parameter('caster_offset', 0.11)
        self.declare_parameter('h_positions', [0.54, 0.54, 0.54, 0.54])
        self.declare_parameter('beta_angles', [0.0, 1.5708, 3.14159, -1.5708])  # 0, 90, 180, -90 graus
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('publish_joint_states', True)  # Publicar joint_states para RViz
        
        # Obter parâmetros
        self.control_mode = self.get_parameter('control_mode').value
        r = self.get_parameter('wheels_radius').value
        b = self.get_parameter('caster_offset').value
        h = self.get_parameter('h_positions').value
        beta = self.get_parameter('beta_angles').value
        update_rate = self.get_parameter('update_rate').value
        
        # Inicializar cinemática
        self.kinematics = GaiaKinematics(r, b, h, beta)
        
        # Estado atual dos steers
        self.current_steer_positions = np.zeros(4)
        self.joint_states_received = False
        
        # Subscriber para cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscriber para trocar modo de controle
        self.mode_sub = self.create_subscription(
            String,
            'control_mode',
            self.mode_callback,
            10
        )
        
        # Subscriber para joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers para controladores
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controllers/commands',
            10
        )
        
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controllers/commands',
            10
        )
        
        # Publisher para joint states (para visualização no RViz)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Estado atual das juntas para integração
        self.joint_positions = {
            'front_left_steer_joint': 0.0,
            'front_right_steer_joint': 0.0,
            'rear_right_steer_joint': 0.0,
            'rear_left_steer_joint': 0.0,
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0
        }
        self.joint_velocities = {name: 0.0 for name in self.joint_positions.keys()}
        
        # Timer para publicação periódica
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        
        # Último comando recebido
        self.last_cmd_vel = np.zeros(3)
        
        self.get_logger().info(f'Gaia Bridge Node initialized in {self.control_mode} mode')
    
    def mode_callback(self, msg):
        """Troca o modo de controle dinamicamente."""
        new_mode = msg.data.lower()
        if new_mode in ['velocity', 'differential', 'shim']:
            self.control_mode = new_mode
            self.get_logger().info(f'Control mode changed to: {self.control_mode}')
        else:
            self.get_logger().warn(f'Invalid control mode: {new_mode}. Use "velocity", "differential" or "shim"')
    
    def joint_state_callback(self, msg):
        """Atualiza os ângulos atuais dos steers."""
        try:
            # Mapear nomes das juntas para índices
            # URDF: front_left, front_right, rear_right, rear_left
            steer_names = [
                'front_left_steer_joint',    # índice 0 - roda 1 (superior esquerda)
                'front_right_steer_joint',   # índice 2 - roda 2 (superior direita)
                'rear_right_steer_joint',    # índice 4 - roda 3 (inferior direita)
                'rear_left_steer_joint'      # índice 6 - roda 4 (inferior esquerda)
            ]
            
            for i, name in enumerate(steer_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_steer_positions[i] = msg.position[idx]
            
            self.joint_states_received = True
            
        except Exception as e:
            self.get_logger().warn(f'Error processing joint states: {e}')
    
    def cmd_vel_callback(self, msg):
        """Recebe comandos de velocidade."""
        self.last_cmd_vel[0] = msg.linear.x
        self.last_cmd_vel[1] = msg.linear.y
        self.last_cmd_vel[2] = msg.angular.z
    
    def timer_callback(self):
        """Calcula e publica comandos para os controladores."""
        
        # Calcular velocidades das juntas
        if self.control_mode == 'differential':
            joint_velocities = self.kinematics.calculate_inverse_kinematics_differential(
                self.last_cmd_vel
            )
        elif self.control_mode == 'shim':
            # No modo shim, usar as posições dos steers que o próprio nó mantém
            steer_positions = np.array([
                self.joint_positions['front_left_steer_joint'],
                self.joint_positions['front_right_steer_joint'],
                self.joint_positions['rear_right_steer_joint'],
                self.joint_positions['rear_left_steer_joint']
            ])
            
            joint_velocities = self.kinematics.calculate_inverse_kinematics_shim(
                self.last_cmd_vel,
                steer_positions
            )
        else:  # velocity (cinemática completa)
            # No modo velocity, também usar as posições mantidas pelo próprio nó
            steer_positions = np.array([
                self.joint_positions['front_left_steer_joint'],
                self.joint_positions['front_right_steer_joint'],
                self.joint_positions['rear_right_steer_joint'],
                self.joint_positions['rear_left_steer_joint']
            ])
            
            joint_velocities = self.kinematics.calculate_inverse_kinematics(
                self.last_cmd_vel,
                steer_positions
            )
        
        # Separar comandos para steers (posição) e wheels (velocidade)
        # Índices pares: steers, Índices ímpares: wheels
        steer_commands = Float64MultiArray()
        wheel_commands = Float64MultiArray()
        
        # Steers: índices 0, 2, 4, 6
        steer_velocities = [joint_velocities[0], joint_velocities[2], 
                           joint_velocities[4], joint_velocities[6]]
        
        # No modo differential, steers ficam em zero
        # Nos modos velocity e shim, steers se movem
        if self.control_mode == 'differential':
            steer_commands.data = [0.0, 0.0, 0.0, 0.0]
        elif self.control_mode in ['velocity', 'shim']:
            # Converter velocidades em comandos de posição incrementais
            dt = 1.0 / self.get_parameter('update_rate').value
            new_positions = self.current_steer_positions + np.array(steer_velocities) * dt
            steer_commands.data = new_positions.tolist()
        
        # Wheels: índices 1, 3, 5, 7
        wheel_commands.data = [
            joint_velocities[1],  # front_left_wheel (roda 1: superior esquerda)
            joint_velocities[3],  # front_right_wheel (roda 2: superior direita)
            joint_velocities[5],  # rear_right_wheel (roda 3: inferior direita)
            joint_velocities[7]   # rear_left_wheel (roda 4: inferior esquerda)
        ]
        
        # Publicar comandos
        self.steer_pub.publish(steer_commands)
        self.wheel_pub.publish(wheel_commands)
        
        # Publicar joint states para visualização no RViz (se habilitado)
        if self.get_parameter('publish_joint_states').value:
            self.publish_joint_states(joint_velocities)
    
    def publish_joint_states(self, joint_velocities):
        """Publica os estados das juntas para visualização."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Atualizar posições integrando velocidades
        dt = 1.0 / self.get_parameter('update_rate').value
        
        # SEMPRE publicar todas as 8 juntas (steers + wheels)
        # No modo differential: steers ficam fixos em zero, wheels giram
        # Nos outros modos: steers e wheels podem se mover
        joint_names = [
            'front_left_steer_joint', 'front_left_wheel_joint',
            'front_right_steer_joint', 'front_right_wheel_joint',
            'rear_right_steer_joint', 'rear_right_wheel_joint',
            'rear_left_steer_joint', 'rear_left_wheel_joint'
        ]
        
        steer_names = ['front_left_steer_joint', 'front_right_steer_joint', 
                       'rear_right_steer_joint', 'rear_left_steer_joint']
        
        for i, name in enumerate(joint_names):
            self.joint_velocities[name] = joint_velocities[i]
            
            # Steers ficam em zero no modo differential e velocity
            # Apenas no modo shim os steers se movem
            if name in steer_names and self.control_mode in ['differential', 'velocity']:
                self.joint_positions[name] = 0.0
            else:
                # Wheels sempre integram velocidade
                self.joint_positions[name] += joint_velocities[i] * dt
        
        # Preencher mensagem com TODAS as juntas
        joint_state_msg.name = joint_names
        joint_state_msg.position = [self.joint_positions[name] for name in joint_names]
        joint_state_msg.velocity = [self.joint_velocities[name] for name in joint_names]
        
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaiaBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
