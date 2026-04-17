"""
Controlador UR5 con comunicación por socket para la aplicación web
Versión con conexión por puerto 30002 y control de velocidades continuas
"""

import socket
import struct
import numpy as np
import time
import threading
import logging
import json
from datetime import datetime

# Importaciones para control Xbox (opcional)
try:
    import pygame
    PYGAME_AVAILABLE = True
    print("✅ pygame disponible - Control Xbox habilitado")
except ImportError:
    PYGAME_AVAILABLE = False
    print("❌ pygame no disponible - Control Xbox deshabilitado")

# Importaciones para control del gripper
try:
    from robot_modules.gripper_config import get_gripper_controller
    GRIPPER_AVAILABLE = True
    print("✅ Módulo gripper disponible")
except ImportError:
    GRIPPER_AVAILABLE = False
    print("❌ Módulo gripper no disponible")

logger = logging.getLogger(__name__)

class UR5WebController:
    def __init__(self, robot_ip="192.168.0.101", robot_port=30002):
        """Inicializar controlador UR5 para aplicación web con comunicación por socket"""
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None  # Socket para envío de comandos (puerto 30002)
        self.read_socket = None  # Socket para lectura de estado (puerto 30001)
        
        # Variables para almacenar posiciones actuales
        self.current_joint_positions_rad = None
        self.current_tcp_pose = None
        self.position_lock = threading.Lock()
        
        # Thread para lectura continua de posiciones
        self.position_thread = None
        self.position_reading = False
        
        # Parámetros de velocidad - múltiples niveles
        self.speed_levels = [0.2, 0.4, 0.6, 0.8, 1.0]
        self.current_speed_level = 1  # Iniciado en nivel 2 (40%)
        
        # Velocidades máximas para movimiento lineal (m/s)
        self.max_linear_velocity = {
            'xy': 0.1,   # Velocidad máxima en X e Y
            'z': 0.08,   # Velocidad máxima en Z
            'rot': 0.5   # Velocidad máxima rotacional (rad/s)
        }
        
        # Velocidades máximas para movimiento articular (rad/s)
        self.max_joint_velocity = [
            1.0,  # Joint 0 (base)
            1.0,  # Joint 1 (shoulder)
            1.5,  # Joint 2 (elbow)
            2.0,  # Joint 3 (wrist1)
            2.0,  # Joint 4 (wrist2)
            2.0   # Joint 5 (wrist3)
        ]
        
        # Configuración de deadzone
        self.deadzone = 0.15
        self.trigger_deadzone = 0.1
        
        # Aceleración para comandos de velocidad
        self.acceleration = 0.5
        self.time_step = 0.1  # Tiempo para comandos de velocidad
        
        # Estados
        self.connected = False
        self.movement_active = False
        self.emergency_stop_active = False
        self.emergency_stop_time = 0
        
        # Posición home
        self.home_joint_angles_deg = [-58.49, -78.0, -98.4, -94.67, 88.77, -109.86]
        self.home_joint_angles_rad = np.radians(self.home_joint_angles_deg)
        
        # Límites del workspace
        self.UR5E_MAX_REACH = 0.85
        self.UR5E_MIN_REACH = 0.18
        
        # Lock para acceso thread-safe
        self.lock = threading.Lock()
        
        # Control Xbox - Control de velocidades continuas
        self.xbox_enabled = True
        self.joystick = None
        self.xbox_thread = None
        self.xbox_running = False
        self.previous_button_states = {}
        self.control_mode = "linear"  # "linear" o "joint"
        
        # Control de hilo de velocidad
        self.velocity_thread = None
        self.velocity_active = False
        self.current_velocities = {
            'linear': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # [vx, vy, vz, wx, wy, wz]
            'joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # velocidades articulares
        }
        self.velocity_lock = threading.Lock()
        
        # Control para evitar spam de comandos de parada
        self.last_movement_state = False
        self.stop_command_sent = False
        
        # Debug
        self.debug_mode = True
        self.last_debug_time = 0
        
        # ========== CONTROL DEL GRIPPER ==========
        self.gripper_controller = None
        self.gripper_enabled = GRIPPER_AVAILABLE
        
        # Variables para control del gatillo derecho (mapeo 0-5000 steps)
        self.right_trigger_values = []  # Buffer para promedio de 4 segundos
        self.right_trigger_buffer_duration = 4.0  # segundos
        self.last_trigger_activation = 0
        self.trigger_threshold = 0.8  # Umbral para cerrar gripper
        self.trigger_was_above_threshold = False
        self.close_steps = 1000  # Pasos a cerrar cuando se activa
        self.last_mapped_steps = 0  # Para evitar movimientos redundantes
        
        # Variables para controlar estado de gatillos (LT y RT)
        self.left_trigger_pressed = False  # Estado del gatillo izquierdo (LT)
        self.right_trigger_custom_pressed = False  # Estado del gatillo derecho (RT) para comando personalizado
        self.trigger_press_threshold = 0.5  # Umbral para considerar gatillo presionado
        
        # Inicializar gripper si está disponible
        if self.gripper_enabled:
            try:
                self.gripper_controller = get_gripper_controller()
                if self.gripper_controller:
                    # CONEXIÓN NO BLOQUEANTE - no conectar aquí, se conectará cuando se necesite
                    logger.info("🦾 Controlador gripper inicializado (conexión lazy)")
            except Exception as e:
                logger.error(f"❌ Error inicializando gripper: {e}")
                self.gripper_enabled = False
        
        # Intentar conectar al robot
        self.initialize_robot()
        
        # Inicializar Xbox controller automáticamente
        self.initialize_xbox_controller()
        
        logger.info(f"UR5WebController inicializado - IP: {robot_ip}:{robot_port}")
        logger.info(f"🎮 Control Xbox: {'Habilitado' if self.xbox_enabled else 'Deshabilitado'}")

    def initialize_robot(self):
        """Inicializar conexión con el robot UR5e mediante socket"""
        try:
            logger.info(f"🤖 Conectando al robot UR5e en {self.robot_ip}...")
            
            # Conectar socket de comandos (puerto 30002)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robot_ip, self.robot_port))
            logger.info(f"✅ Socket de comandos conectado en puerto {self.robot_port}")
            
            # Conectar socket de lectura (puerto 30001)
            try:
                self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.read_socket.settimeout(1.0)  # Timeout de 1 segundo
                self.read_socket.connect((self.robot_ip, 30001))
                logger.info("✅ Socket de lectura conectado en puerto 30001")
                
                # Iniciar hilo de lectura de posiciones
                self.start_position_reading()
                
            except Exception as read_error:
                logger.warning(f"⚠️ No se pudo conectar socket de lectura: {read_error}")
                logger.info("📝 Continuando solo con envío de comandos")
                self.read_socket = None
            
            self.connected = True
            logger.info("✅ Robot UR5e conectado exitosamente por socket!")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error conectando al robot: {e}")
            self.connected = False
            return False

    def is_connected(self):
        """Verificar si el robot está conectado"""
        try:
            return self.connected and self.socket is not None
        except:
            return False
    
    def can_control(self):
        """Verificar si se pueden enviar comandos de control"""
        return self.is_connected()

    def send_command(self, command):
        """Enviar comando al robot"""
        try:
            if self.socket:
                cmd_bytes = (command + "\n").encode('utf-8')
                self.socket.send(cmd_bytes)
                
                # Debug: mostrar comando enviado si el debug está activo
                if self.debug_mode:
                    if not (command.startswith('stopl(') or command.startswith('stopj(')):
                        logger.info(f"📤 Comando enviado: {command}")
                    elif not hasattr(self, '_last_debug_stop') or self._last_debug_stop != command:
                        logger.info(f"📤 Comando enviado: {command} (parada)")
                        self._last_debug_stop = command
                
                return True
            else:
                logger.warning("❌ No hay conexión con el robot")
                return False
        except Exception as e:
            logger.error(f"❌ Error enviando comando: {e}")
            return False
    
    def send_speedl(self, vx, vy, vz, wx, wy, wz, a=None, t=None):
        """Enviar comando de velocidad lineal"""
        if a is None:
            a = self.acceleration
        if t is None:
            t = self.time_step
        
        cmd = f"speedl([{vx:.5f}, {vy:.5f}, {vz:.5f}, {wx:.5f}, {wy:.5f}, {wz:.5f}], {a}, {t})"
        return self.send_command(cmd)
    
    def send_speedj(self, q0, q1, q2, q3, q4, q5, a=None, t=None):
        """Enviar comando de velocidad articular"""
        if a is None:
            a = self.acceleration
        if t is None:
            t = self.time_step
        
        cmd = f"speedj([{q0:.5f}, {q1:.5f}, {q2:.5f}, {q3:.5f}, {q4:.5f}, {q5:.5f}], {a}, {t})"
        return self.send_command(cmd)
    
    def send_stopl(self, a=None):
        """Detener movimiento lineal"""
        if a is None:
            a = self.acceleration
        cmd = f"stopl({a})"
        return self.send_command(cmd)
    
    def send_stopj(self, a=None):
        """Detener movimiento articular"""
        if a is None:
            a = self.acceleration
        cmd = f"stopj({a})"
        return self.send_command(cmd)

    # ========== MÉTODOS DE LECTURA DE POSICIONES VÍA SOCKET ==========
    
    def start_position_reading(self):
        """Iniciar hilo de lectura continua de posiciones"""
        if not self.position_reading and self.read_socket:
            self.position_reading = True
            self.position_thread = threading.Thread(target=self.position_reading_thread, daemon=True)
            self.position_thread.start()
            logger.info("📊 Lectura de posiciones iniciada")

    def stop_position_reading(self):
        """Detener hilo de lectura de posiciones"""
        if self.position_reading:
            self.position_reading = False
            if self.position_thread and self.position_thread.is_alive():
                self.position_thread.join(timeout=1.0)
            logger.info("📊 Lectura de posiciones detenida")

    def position_reading_thread(self):
        """Hilo para lectura continua de posiciones del robot"""
        logger.info("📡 Iniciando lectura continua de posiciones...")
        
        while self.position_reading and self.read_socket:
            try:
                pose_data = self.get_pose_from_socket()
                if pose_data:
                    x, y, z, rx, ry, rz, joints = pose_data
                    
                    with self.position_lock:
                        self.current_tcp_pose = [x, y, z, rx, ry, rz]
                        self.current_joint_positions_rad = joints
                
                time.sleep(0.1)  # Leer posiciones cada 100ms
                
            except Exception as e:
                if self.position_reading:  # Solo mostrar error si no estamos cerrando
                    logger.warning(f"Error leyendo posición: {e}")
                time.sleep(1.0)  # Esperar más tiempo en caso de error
        
        logger.info("📡 Lectura de posiciones terminada")

    def get_pose_from_socket(self):
        """
        Función para obtener tanto coordenadas articulares como cartesianas del robot vía Socket
        Basada en tu código sugerido
        """
        if not self.read_socket:
            return None
            
        try:
            cdr = 0
            x = y = z = rx = ry = rz = 0
            angles = [0] * 6
            
            while cdr < 3:
                # Recibir 4096 bytes de datos
                data = self.read_socket.recv(4096)
                i = 0
                
                if data and len(data) >= 18:
                    # Información del paquete recibido
                    packlen = (struct.unpack('!i', data[0:4]))[0]
                    timestamp = (struct.unpack('!Q', data[10:18]))[0]
                    packtype = (struct.unpack('!b', data[4:5]))[0]

                    # Si el tipo de paquete es el estado del robot
                    if packtype == 16:
                        while i + 5 < packlen and i + 10 < len(data):
                            try:
                                # Tamaño y tipo de mensaje
                                msglen = (struct.unpack('!i', data[5+i:9+i]))[0]
                                msgtype = (struct.unpack('!b', data[9+i:10+i]))[0]

                                if msgtype == 1 and i + 10 + 6*41 <= len(data):
                                    # Coordenadas articulares
                                    j = 0
                                    while j < 6:
                                        if 10+i+(j*41)+8 <= len(data):
                                            angles[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
                                        j += 1

                                elif msgtype == 4 and i + 10 + 48 <= len(data):
                                    # Coordenadas cartesianas
                                    x = (struct.unpack('!d', data[10+i:18+i]))[0]
                                    y = (struct.unpack('!d', data[18+i:26+i]))[0]
                                    z = (struct.unpack('!d', data[26+i:34+i]))[0]
                                    rx = (struct.unpack('!d', data[34+i:42+i]))[0]
                                    ry = (struct.unpack('!d', data[42+i:50+i]))[0]
                                    rz = (struct.unpack('!d', data[50+i:58+i]))[0]

                                # Incrementar i por el tamaño del mensaje
                                if msglen > 0:
                                    i += msglen
                                else:
                                    break
                                    
                            except struct.error:
                                break
                                
                cdr += 1

            return x, y, z, rx, ry, rz, angles
            
        except socket.timeout:
            return None  # Timeout normal, no es un error
        except Exception as e:
            logger.warning(f"Error en get_pose_from_socket: {e}")
            return None

    def get_current_joint_positions(self):
        """Obtener posiciones actuales de las articulaciones"""
        try:
            with self.position_lock:
                if self.current_joint_positions_rad is not None:
                    return self.current_joint_positions_rad.copy()
                else:
                    # Si no hay datos reales, devolver posición home
                    return self.home_joint_angles_rad
        except Exception as e:
            logger.error(f"Error obteniendo posiciones articulares: {e}")
            return self.home_joint_angles_rad

    def get_current_tcp_pose(self):
        """Obtener pose actual del TCP"""
        try:
            with self.position_lock:
                if self.current_tcp_pose is not None:
                    return self.current_tcp_pose.copy()
                else:
                    # Si no hay datos reales, devolver pose por defecto
                    return [0.3, -0.2, 0.5, 0, 0, 0]
        except Exception as e:
            logger.error(f"Error obteniendo pose TCP: {e}")
            return [0.3, -0.2, 0.5, 0, 0, 0]

    def get_current_pose(self):
        """Obtener pose actual formateada para la web"""
        try:
            tcp_pose = self.get_current_tcp_pose()
            # Convertir a mm y grados para la interfaz
            return [
                round(tcp_pose[0] * 1000, 2),  # X en mm
                round(tcp_pose[1] * 1000, 2),  # Y en mm
                round(tcp_pose[2] * 1000, 2),  # Z en mm
                round(np.degrees(tcp_pose[3]), 2),  # RX en grados
                round(np.degrees(tcp_pose[4]), 2),  # RY en grados
                round(np.degrees(tcp_pose[5]), 2),  # RZ en grados
            ]
        except Exception as e:
            logger.error(f"Error obteniendo pose formateada: {e}")
            return [300.0, -200.0, 500.0, 0.0, 0.0, 0.0]

    def is_point_within_reach(self, x, y, z):
        """Verificar si el punto está dentro del workspace"""
        # Convertir de mm a metros si es necesario
        if abs(x) > 10:  # Probablemente está en mm
            x, y, z = x/1000, y/1000, z/1000
        
        point = np.array([x, y, z])
        distance = np.linalg.norm(point)
        return self.UR5E_MIN_REACH <= distance <= self.UR5E_MAX_REACH

    def move_to_coordinates(self, x, y, z, rx, ry, rz):
        """
        Mover robot a coordenadas especificadas
        Acepta coordenadas en mm y grados para facilitar interfaz web
        """
        try:
            with self.lock:
                if self.emergency_stop_active:
                    logger.warning("No se puede mover: parada de emergencia activa")
                    return False
                
                if self.movement_active:
                    logger.warning("Movimiento ya en progreso")
                    return False
                
                # Convertir de mm a metros y grados a radianes
                x_m = x / 1000.0 if abs(x) > 10 else x
                y_m = y / 1000.0 if abs(y) > 10 else y
                z_m = z / 1000.0 if abs(z) > 10 else z
                
                rx_rad = np.radians(rx) if abs(rx) > 0.1 else rx
                ry_rad = np.radians(ry) if abs(ry) > 0.1 else ry
                rz_rad = np.radians(rz) if abs(rz) > 0.1 else rz
                
                # Validar workspace
                if not self.is_point_within_reach(x_m, y_m, z_m):
                    distance = np.linalg.norm(np.array([x_m, y_m, z_m]))
                    logger.warning(f"Punto fuera del alcance: {distance:.3f}m")
                    return False
                
                target_pose = [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]
                
                if self.can_control():
                    # Enviar comando por socket
                    self.movement_active = True
                    logger.info(f"🤖 Moviendo robot a: {target_pose}")
                    
                    # Crear comando URScript para movimiento lineal con formato p[]
                    # Los datos rx, ry, rz ya están en radianes
                    cmd = f"movel(p[{x_m:.6f},{y_m:.6f},{z_m:.6f},{rx_rad:.6f},{ry_rad:.6f},{rz_rad:.6f}], a = 1.2, v = 0.25, t = 0, r = 0)"
                    success = self.send_command(cmd)
                    
                    if success:
                        logger.info("✅ Comando de movimiento enviado exitosamente")
                        time.sleep(3.0)  # Tiempo estimado para completar movimiento
                        self.movement_active = False
                        return True
                    else:
                        logger.error("❌ Fallo enviando comando de movimiento")
                        self.movement_active = False
                        return False
                elif self.is_connected():
                    # Robot conectado pero sin control
                    logger.warning("📖 Robot conectado en modo solo lectura - comando no enviado")
                    logger.info(f"📝 Comando registrado: mover a {target_pose}")
                    return True
                else:
                    # MODO DESCONECTADO - Solo loggar el comando 
                    logger.info(f"📝 Comando registrado: mover a {target_pose}")
                    logger.info("⚠️ Robot no conectado - comando no enviado")
                    time.sleep(1)  # Simular tiempo de procesamiento
                    return True
                
        except Exception as e:
            logger.error(f"❌ Error en movimiento: {e}")
            self.movement_active = False
            return False

    def go_home(self):
        """Mover robot a posición home"""
        try:
            with self.lock:
                if self.emergency_stop_active:
                    logger.warning("No se puede ir a home: parada de emergencia activa")
                    return False
                
                if self.can_control():
                    # Enviar comando por socket
                    self.movement_active = True
                    logger.info("🏠 Moviendo robot a posición home...")
                    
                    # Detener cualquier movimiento actual
                    self.send_stopl()
                    self.send_stopj()
                    time.sleep(0.1)
                    
                    # Usar sintaxis URScript que funciona
                    joint_angles = ", ".join([f"{angle:.5f}" for angle in self.home_joint_angles_rad])
                    cmd = f"movej([{joint_angles}], a = 2.5, v = 1.5)"
                    
                    success = self.send_command(cmd)
                    
                    if success:
                        logger.info("✅ Robot movido a posición home exitosamente")
                        time.sleep(5.0)  # Tiempo estimado para llegar a home
                        self.movement_active = False
                        return True
                    else:
                        logger.error("❌ Error enviando comando al robot")
                        self.movement_active = False
                        return False
                elif self.is_connected():
                    # Robot conectado pero sin control
                    logger.warning("📖 Robot conectado en modo solo lectura - comando no enviado")
                    logger.info("📝 Comando registrado: ir a posición home")
                    return True
                else:
                    # MODO DESCONECTADO - Solo loggar el comando
                    logger.info("📝 Comando registrado: ir a posición home")
                    logger.info("⚠️ Robot no conectado - comando no enviado")
                    time.sleep(2)
                    return True
                
        except Exception as e:
            logger.error(f"❌ Error yendo a home: {e}")
            self.movement_active = False
            return False

    def emergency_stop(self):
        """Activar parada de emergencia"""
        try:
            with self.lock:
                self.movement_active = False
                self.emergency_stop_active = True
                
                if self.can_control():
                    # Ejecutar parada de emergencia real en el robot
                    logger.warning("🚨 EJECUTANDO PARADA DE EMERGENCIA EN ROBOT")
                    self.control.stopScript()  # Detener script actual
                elif self.is_connected():
                    logger.warning("🚨 PARADA DE EMERGENCIA REGISTRADA (modo solo lectura)")
                else:
                    logger.warning("🚨 PARADA DE EMERGENCIA REGISTRADA (robot no conectado)")
                
                return True
                
        except Exception as e:
            logger.error(f"Error ejecutando parada de emergencia: {e}")
            return False

    def deactivate_emergency_stop(self):
        """Desactivar parada de emergencia"""
        with self.lock:
            self.emergency_stop_active = False
            logger.info("✅ Parada de emergencia DESACTIVADA")

    def wait_for_movement_completion_joint(self, target_joints, timeout=5.0):
        """Esperar a que termine el movimiento articular"""
        if not self.is_connected():
            return not self.emergency_stop_active
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.emergency_stop_active:
                return False
            
            current_joints = self.receive.getActualQ()
            if np.allclose(current_joints, target_joints, atol=self.position_tolerance_joint):
                return True
            
            time.sleep(0.1)
        
        logger.warning("Timeout esperando completar movimiento articular")
        return False

    def wait_for_movement_completion_tcp(self, target_pose, timeout=5.0):
        """Esperar a que termine el movimiento lineal"""  
        if not self.is_connected():
            return not self.emergency_stop_active
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.emergency_stop_active:
                return False
            
            current_pose = self.receive.getActualTCPPose()
            if np.allclose(current_pose, target_pose, atol=self.position_tolerance_tcp):
                return True
            
            time.sleep(0.1)
        
        logger.warning("Timeout esperando completar movimiento lineal")
        return False

    def get_robot_status(self):
        """Obtener estado completo del robot"""
        with self.lock:
            is_connected = self.is_connected()
            
            status = {
                'connected': is_connected,
                'can_control': self.can_control() if is_connected else False,
                'movement_active': self.movement_active,
                'emergency_stop_active': self.emergency_stop_active,
                'current_position': self.get_current_pose(),
                'speed_level': self.current_speed_level + 1,
                'speed_percentage': int(self.speed_levels[self.current_speed_level] * 100),
                'mode': 'CONECTADO' if is_connected else 'DESCONECTADO',
                'robot_ip': self.robot_ip,
                'position_reading': self.position_reading,
                'read_socket_connected': self.read_socket is not None
            }
            
            # Para comunicación por socket, no tenemos acceso a información detallada
            if is_connected:
                try:
                    status.update({
                        'robot_mode': 'RUNNING',  # Estado por defecto
                        'safety_mode': 'NORMAL',  # Estado por defecto
                        'joint_temperatures': [25.0] * 6,  # Temperaturas simuladas
                        'runtime_state': 'RUNNING'  # Estado por defecto
                    })
                except Exception as e:
                    logger.warning(f"Error obteniendo estado extendido: {e}")
        
        # Incluir posiciones articulares
        joints = self.get_current_joint_positions()
        status['joint_positions'] = [np.degrees(j) for j in joints]
        
        # Incluir información del control Xbox
        status.update(self.get_xbox_status())
        
        # Incluir información del gripper
        status.update(self.get_gripper_status())
        
        return status

    def set_speed_level(self, level):
        """Cambiar nivel de velocidad (0-4)"""
        if 0 <= level < len(self.speed_levels):
            with self.lock:
                self.current_speed_level = level
            logger.info(f"Velocidad cambiada a nivel {level+1} ({self.speed_levels[level]*100:.0f}%)")
            return True
        return False

    def disconnect(self):
        """Desconectar del robot y limpiar recursos"""
        try:
            with self.lock:
                # Detener lectura de posiciones
                self.stop_position_reading()
                
                # Detener control de velocidad
                if hasattr(self, 'velocity_active'):
                    self.stop_velocity_control()
                
                # Detener control Xbox
                self.xbox_running = False
                if hasattr(self, 'xbox_thread') and self.xbox_thread and self.xbox_thread.is_alive():
                    self.xbox_thread.join(timeout=2.0)
                
                # Cerrar sockets
                if hasattr(self, 'socket') and self.socket:
                    self.socket.close()
                    self.socket = None
                
                if hasattr(self, 'read_socket') and self.read_socket:
                    self.read_socket.close()
                    self.read_socket = None
                
                # Desconectar gripper
                if hasattr(self, 'gripper_controller') and self.gripper_controller:
                    try:
                        self.gripper_controller.disconnect()
                    except Exception as e:
                        logger.warning(f"Error desconectando gripper: {e}")
                    self.gripper_controller = None
                    
                self.connected = False
                
            logger.info("📝 Controlador UR5 cerrado (modo socket)")
            
        except Exception as e:
            logger.error(f"Error cerrando controlador: {e}")

    def move_joints(self, target_joints, speed=2.5, acceleration=1.5, asynchronous=False):
        """Mover articulaciones específicas usando socket"""
        if not self.can_control():
            logger.warning("⚠️ Robot no puede ser controlado")
            return False
        
        try:
            with self.lock:
                if self.movement_active and not asynchronous:
                    logger.warning("⚠️ Movimiento ya en progreso")
                    return False
                
                self.movement_active = True
            
            logger.info(f"🦾 Moviendo articulaciones a: {[np.degrees(j) for j in target_joints]}")
            
            # Crear comando URScript
            joint_str = ", ".join([f"{j:.5f}" for j in target_joints])
            cmd = f"movej([{joint_str}], a = {acceleration}, v = {speed})"
            success = self.send_command(cmd)
            
            if success and not asynchronous:
                time.sleep(3.0)  # Tiempo estimado para completar movimiento
            
            return success
            
        except Exception as e:
            logger.error(f"❌ Error moviendo articulaciones: {e}")
            return False
        finally:
            if not asynchronous:
                with self.lock:
                    self.movement_active = False

    def move_linear(self, target_pose, speed=0.5, acceleration=1.5, asynchronous=False):
        """Mover TCP linealmente usando socket"""
        if not self.can_control():
            logger.warning("⚠️ Robot no puede ser controlado")
            return False
        
        try:
            with self.lock:
                if self.movement_active and not asynchronous:
                    logger.warning("⚠️ Movimiento ya en progreso")
                    return False
                
                self.movement_active = True
            
            logger.info(f"🎯 Moviendo TCP a: {target_pose}")
            
            # Crear comando URScript
            pose_str = ", ".join([f"{p:.5f}" for p in target_pose])
            cmd = f"movel(p[{pose_str}], a = {acceleration}, v = {speed}, t = 0, r = 0)"
            success = self.send_command(cmd)
            
            if success and not asynchronous:
                time.sleep(3.0)  # Tiempo estimado para completar movimiento
            
            return success
            
        except Exception as e:
            logger.error(f"❌ Error moviendo TCP linealmente: {e}")
            return False
        finally:
            if not asynchronous:
                with self.lock:
                    self.movement_active = False

    # ========== MÉTODOS DE CONTROL DE VELOCIDAD ==========
    
    def apply_deadzone(self, value, deadzone=None):
        """Aplicar zona muerta a valor analógico"""
        if deadzone is None:
            deadzone = self.deadzone
        return 0.0 if abs(value) < deadzone else value

    def velocity_control_thread(self):
        """Hilo para envío continuo de comandos de velocidad"""
        while self.velocity_active:
            try:
                with self.velocity_lock:
                    has_movement = False
                    
                    if self.control_mode == "linear":
                        vx, vy, vz, wx, wy, wz = self.current_velocities['linear']
                        if any(abs(v) > 0.001 for v in [vx, vy, vz, wx, wy, wz]):
                            self.send_speedl(vx, vy, vz, wx, wy, wz)
                            has_movement = True
                            self.stop_command_sent = False
                        else:
                            if self.last_movement_state and not self.stop_command_sent:
                                self.send_stopl()
                                self.stop_command_sent = True
                                
                    else:  # joint mode
                        q0, q1, q2, q3, q4, q5 = self.current_velocities['joint']
                        if any(abs(q) > 0.001 for q in [q0, q1, q2, q3, q4, q5]):
                            self.send_speedj(q0, q1, q2, q3, q4, q5)
                            has_movement = True
                            self.stop_command_sent = False
                        else:
                            if self.last_movement_state and not self.stop_command_sent:
                                self.send_stopj()
                                self.stop_command_sent = True
                    
                    self.last_movement_state = has_movement
                
                time.sleep(0.03)  # ~33 Hz
                
            except Exception as e:
                logger.error(f"Error en hilo de velocidad: {e}")
                time.sleep(0.1)

    def start_velocity_control(self):
        """Iniciar control de velocidad continuo"""
        if not self.velocity_active:
            self.velocity_active = True
            self.velocity_thread = threading.Thread(target=self.velocity_control_thread)
            self.velocity_thread.daemon = True
            self.velocity_thread.start()
            logger.info("Control de velocidad iniciado")

    def stop_velocity_control(self):
        """Detener control de velocidad continuo"""
        if self.velocity_active:
            self.velocity_active = False
            if self.velocity_thread:
                self.velocity_thread.join(timeout=1.0)
            
            # Enviar comando de parada
            if self.control_mode == "linear":
                self.send_stopl()
            else:
                self.send_stopj()
            
            logger.info("Control de velocidad detenido")

    def update_velocities(self, velocities, mode):
        """Actualizar velocidades objetivo"""
        with self.velocity_lock:
            if mode == "linear":
                self.current_velocities['linear'] = velocities[:]
            else:
                self.current_velocities['joint'] = velocities[:]

    def stop_all_movement(self):
        """Detener todos los movimientos"""
        # Limpiar velocidades
        with self.velocity_lock:
            self.current_velocities['linear'] = [0.0] * 6
            self.current_velocities['joint'] = [0.0] * 6
        
        # Resetear flags
        self.last_movement_state = False
        self.stop_command_sent = False
        
        # Enviar comandos de parada
        self.send_stopl()
        self.send_stopj()
        
        self.stop_command_sent = True

    # ========== FUNCIONES PARA CONTROL XBOX - CONTROL DE VELOCIDADES ==========
    
    def initialize_xbox_controller(self):
        """Inicializar el control Xbox - SIEMPRE HABILITADO"""
        if not PYGAME_AVAILABLE:
            logger.warning("❌ pygame no disponible - Control Xbox deshabilitado")
            self.xbox_enabled = False
            return False
        
        try:
            pygame.init()
            pygame.joystick.init()
            
            # Verificar controles conectados
            if pygame.joystick.get_count() == 0:
                logger.warning("⚠️ No se detectaron controles Xbox conectados")
                self.xbox_enabled = False
                return False
            
            # Conectar al primer control
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logger.info(f"🎮 Control conectado: {self.joystick.get_name()}")
            
            # Inicializar estados de botones
            self.previous_button_states = {}
            for i in range(self.joystick.get_numbuttons()):
                self.previous_button_states[i] = False
            
            # Iniciar hilos de control Xbox y velocidad automáticamente
            self.xbox_enabled = True
            self.xbox_running = True
            self.xbox_thread = threading.Thread(target=self._xbox_control_loop, daemon=True)
            self.xbox_thread.start()
            
            # Iniciar control de velocidad automáticamente
            self.start_velocity_control()
            
            logger.info("🎮 Control Xbox HABILITADO con control de velocidades continuas")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error inicializando control Xbox: {e}")
            self.xbox_enabled = False
            return False

    def disable_xbox_control(self):
        """Deshabilitar control Xbox temporalmente"""
        with self.lock:
            if not self.xbox_running:
                return True
            
            self.xbox_running = False
            if self.xbox_thread:
                self.xbox_thread.join(timeout=2)
            logger.info("🎮 Control Xbox deshabilitado temporalmente")
            return True

    def is_xbox_enabled(self):
        """Verificar si el control Xbox está habilitado"""
        return self.xbox_enabled and self.xbox_running

    def _xbox_control_loop(self):
        """Bucle principal del control Xbox - Control de velocidades continuas"""
        logger.info("🎮 Iniciando bucle de control Xbox con velocidades...")
        
        try:
            clock = pygame.time.Clock()
            
            while self.xbox_running and self.xbox_enabled:
                if not self.joystick:
                    break
                    
                try:
                    # Procesar entrada del control con velocidades
                    self.process_xbox_input()
                    clock.tick(60)  # 60 FPS para respuesta fluida
                    
                except Exception as e:
                    logger.error(f"Error en bucle Xbox: {e}")
                    time.sleep(0.1)
            
        except Exception as e:
            logger.error(f"Error crítico en bucle Xbox: {e}")
        finally:
            logger.info("🎮 Bucle de control Xbox terminado")

    def _has_active_input(self):
        """Verificar si hay entrada activa del usuario"""
        if not self.joystick:
            return False
            
        # Verificar botones
        for i in range(self.joystick.get_numbuttons()):
            if self.joystick.get_button(i):
                return True
        
        # Verificar ejes analógicos
        for i in range(self.joystick.get_numaxes()):
            if abs(self.joystick.get_axis(i)) > 0.1:
                return True
        
        # Verificar D-pad
        for i in range(self.joystick.get_numhats()):
            if self.joystick.get_hat(i) != (0, 0):
                return True
        
        return False

    def _process_xbox_input(self):
        """Procesar entrada del control Xbox con velocidades continuas"""
        self.process_xbox_input()  # Usar el nuevo método de velocidades

    def _process_xbox_buttons(self):
        """Procesar botones del control Xbox"""
        # MAPEO CORREGIDO según move_controler.py
        button_mapping = {
            0: "A",          # Cambiar modo de control
            1: "B",          # Parada de emergencia  
            3: "X",          # Ir a home
            4: "Y",          # Deactivar emergencia
            6: "LB",         # Velocidad -
            7: "RB",         # Velocidad +
            10: "Menu",      # Toggle debug
            11: "Start",     # Show status
        }
        
        for button_id in range(self.joystick.get_numbuttons()):
            current_state = self.joystick.get_button(button_id)
            previous_state = self.previous_button_states.get(button_id, False)
            
            # Detectar presión de botón (flanco ascendente)
            if current_state and not previous_state:
                self._handle_xbox_button_press(button_id)
            
            self.previous_button_states[button_id] = current_state

    def _handle_xbox_button_press(self, button_id):
        """Manejar presión de botones específicos del Xbox - MAPEO CORREGIDO"""
        button_actions = {
            0: "A",          # Cambiar modo
            1: "B",          # Emergencia
            3: "X",          # Home
            4: "Y",          # Desactivar emergencia
            6: "LB",         # Velocidad -
            7: "RB",         # Velocidad +
            10: "Menu",      # Debug
            11: "Start",     # Status
        }
        
        button_name = button_actions.get(button_id, f"Btn{button_id}")
        logger.info(f"🎮 Procesando botón: {button_name} (ID: {button_id})")
        
        if button_id == 0:  # A - Cambiar modo
            self.control_mode = "linear" if self.control_mode == "joint" else "joint"
            logger.info(f"🔄 Modo cambiado a: {self.control_mode.upper()}")
        
        elif button_id == 1:  # B - Parada de emergencia
            self.activate_emergency_stop()
        
        elif button_id == 3:  # X - Ir a home
            logger.info("🏠 Moviendo a posición home...")
            if not self.emergency_stop_active and not self.movement_active:
                threading.Thread(target=self.go_home, daemon=True).start()
        
        elif button_id == 4:  # Y - Desactivar emergencia
            self.deactivate_emergency_stop()
        
        elif button_id == 6:  # LB - Reducir velocidad
            if self.current_speed_level > 0:
                self.current_speed_level -= 1
                speed_percent = self.speed_levels[self.current_speed_level] * 100
                logger.info(f"🔽 Velocidad reducida a {speed_percent:.0f}%")
        
        elif button_id == 7:  # RB - Aumentar velocidad
            if self.current_speed_level < len(self.speed_levels) - 1:
                self.current_speed_level += 1
                speed_percent = self.speed_levels[self.current_speed_level] * 100
                logger.info(f"🔼 Velocidad aumentada a {speed_percent:.0f}%")
        
        elif button_id == 11:  # Start - Mostrar estado
            self._show_xbox_status()
        
        elif button_id == 10:  # Menu - Toggle debug
            self.debug_mode = not self.debug_mode
            logger.info(f"🐛 Debug: {'ON' if self.debug_mode else 'OFF'}")

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        try:
            # Enviar comandos de parada por socket
            self.send_stopl(2.0)  # Parada suave lineal
            self.send_stopj(2.0)  # Parada suave en articulaciones
            
            self.emergency_stop_active = True
            self.emergency_stop_time = time.time()
            self.movement_active = False
            logger.warning("🚨 PARADA DE EMERGENCIA ACTIVADA")
        except Exception as e:
            logger.error(f"Error en parada de emergencia: {e}")

    def emergency_stop(self):
        """Activar parada de emergencia"""
        self.activate_emergency_stop()
        return True

    def deactivate_emergency_stop(self):
        """Desactivar parada de emergencia"""
        with self.lock:
            self.emergency_stop_active = False
            logger.info("✅ Parada de emergencia DESACTIVADA")

    def _process_xbox_analog(self):
        """Procesar entrada analógica del Xbox con filtros avanzados para suavizar"""
        # Obtener valores de joysticks
        left_x = self.joystick.get_axis(0)
        left_y = self.joystick.get_axis(1)
        right_x = self.joystick.get_axis(2)
        right_y = self.joystick.get_axis(3)
        
        # Obtener triggers CON MAPEO CORREGIDO
        raw_lt = (self.joystick.get_axis(4) + 1) / 2 if self.joystick.get_numaxes() > 4 else 0
        raw_rt = (self.joystick.get_axis(5) + 1) / 2 if self.joystick.get_numaxes() > 5 else 0
        
        # Intercambiar porque están mapeados al revés
        left_trigger = raw_rt
        right_trigger = raw_lt
        
        # Obtener D-pad
        dpad = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
        
        # === FILTRADO EXPONENCIAL MEJORADO ===
        # Alpha más bajo para mayor suavizado
        left_x = self._apply_input_filter('left_x', left_x)
        left_y = self._apply_input_filter('left_y', left_y)
        right_x = self._apply_input_filter('right_x', right_x)
        right_y = self._apply_input_filter('right_y', right_y)
        left_trigger = self._apply_input_filter('left_trigger', left_trigger)
        right_trigger = self._apply_input_filter('right_trigger', right_trigger)
        
        # === DEADZONE MEJORADA CON RAMPA SUAVE ===
        def apply_smooth_deadzone(value, zone=0.4):  # AUMENTADO de 0.3 a 0.4 - deadzone más amplia
            """Aplicar deadzone con rampa suave en lugar de corte abrupto"""
            abs_value = abs(value)
            if abs_value < zone:
                return 0.0
            # Rampa suave: transición gradual desde deadzone hasta valor máximo
            mapped = (abs_value - zone) / (1.0 - zone)
            # Curva cúbica para control aún más fino en valores bajos
            mapped = mapped ** 2.0  # AUMENTADO de 1.5 a 2.0 para respuesta más lenta
            return mapped * np.sign(value)
        
        left_x = apply_smooth_deadzone(left_x)
        left_y = apply_smooth_deadzone(left_y)
        right_x = apply_smooth_deadzone(right_x)
        right_y = apply_smooth_deadzone(right_y)
        
        # Triggers con umbral más alto
        left_trigger = left_trigger if left_trigger > 0.2 else 0
        right_trigger = right_trigger if right_trigger > 0.2 else 0
        
        # === ACUMULACIÓN TEMPORAL ===
        # Acumular pequeños movimientos antes de ejecutar
        self._accumulate_movement(left_x, left_y, right_x, right_y, left_trigger, right_trigger, dpad)
        
        # Solo ejecutar si hay movimiento acumulado significativo
        self._execute_accumulated_movement()

    def _accumulate_movement(self, left_x, left_y, right_x, right_y, left_trigger, right_trigger, dpad):
        """Acumular movimientos pequeños antes de ejecutar"""
        current_time = time.time()
        
        # Verificar cooldown más estricto
        if current_time - self.last_movement_time < self.movement_cooldown:
            return
        
        if self.control_mode == "joint":
            # Control articular - usar incrementos ULTRA PEQUEÑOS
            increment = self.joint_increment * 0.2  # REDUCIDO de 0.4 a 0.2 para incrementos mínimos
            
            if abs(left_x) > 0.02:  # AUMENTADO el threshold mínimo de 0.01 a 0.02
                self.accumulated_movement[0] += left_x * increment
            if abs(left_y) > 0.02:  # Shoulder  
                self.accumulated_movement[1] += left_y * increment
            if abs(right_x) > 0.02:  # Elbow
                self.accumulated_movement[2] += right_x * increment
            if abs(right_y) > 0.02:  # Wrist 1
                self.accumulated_movement[3] += right_y * increment
            
            # D-pad para articulaciones 4 y 5 (aún más lento)
            dpad_increment = increment * 0.2  # REDUCIDO de 0.3 a 0.2
            if dpad[0] != 0:  # Wrist 2
                self.accumulated_movement[4] += dpad[0] * dpad_increment
            if dpad[1] != 0:  # Wrist 3
                self.accumulated_movement[5] += dpad[1] * dpad_increment
                
        else:  # TCP mode
            # Control lineal - usar incrementos ULTRA PEQUEÑOS
            linear_inc = self.linear_increment * 0.3  # REDUCIDO de 0.5 a 0.3 para incrementos mínimos
            
            if abs(left_x) > 0.02:  # X - AUMENTADO el threshold mínimo
                self.accumulated_movement[0] += left_x * linear_inc
            if abs(left_y) > 0.02:  # Y
                self.accumulated_movement[1] += left_y * linear_inc
            if abs(right_y) > 0.02:  # Z
                self.accumulated_movement[2] += right_y * linear_inc
            if abs(right_x) > 0.02:  # RZ
                self.accumulated_movement[5] += right_x * 0.01  # REDUCIDO de 0.02 a 0.01
        
        # Procesar triggers para velocidad
        self._handle_speed_triggers(left_trigger, right_trigger)

    def _execute_accumulated_movement(self):
        """Ejecutar movimiento acumulado si supera threshold"""
        movements_to_execute = []
        
        if self.control_mode == "joint":
            # Threshold AUMENTADO para evitar micro-movimientos
            threshold = self.movement_threshold * 1.5  # AUMENTADO de 0.8 a 1.5
            
            for i in range(6):
                if abs(self.accumulated_movement[i]) >= threshold:
                    movements_to_execute.append((i, self.accumulated_movement[i]))
                    self.accumulated_movement[i] = 0.0  # Reset
                    
        else:  # TCP mode
            # Thresholds AUMENTADOS para posición y rotación
            for i in range(6):
                if i < 3:  # Posición X, Y, Z
                    threshold = 0.008  # AUMENTADO de 0.004 a 0.008
                else:  # Rotación RX, RY, RZ
                    threshold = 0.03  # AUMENTADO de 0.015 a 0.03
                
                if abs(self.accumulated_movement[i]) >= threshold:
                    movements_to_execute.append((i, self.accumulated_movement[i]))
                    self.accumulated_movement[i] = 0.0
        
        # Ejecutar movimientos acumulados
        if movements_to_execute:
            if self.control_mode == "joint":
                self.execute_simultaneous_joint_movements(movements_to_execute)
            else:
                self.execute_simultaneous_tcp_movements(movements_to_execute)

    def _apply_input_filter(self, input_name, new_value, alpha=None):
        """Aplicar filtro exponencial mejorado para suavizar entrada"""
        if alpha is None:
            alpha = self.filter_alpha
            
        if input_name not in self.input_filter:
            self.input_filter[input_name] = new_value
            return new_value
        
        # Filtro exponencial con detección de cambios grandes
        previous_value = self.input_filter[input_name]
        
        # Si hay un cambio grande (como soltar el joystick), usar alpha más alto
        # para respuesta más rápida
        change_magnitude = abs(new_value - previous_value)
        dynamic_alpha = min(alpha * 2, 0.8) if change_magnitude > 0.5 else alpha
        
        filtered_value = (dynamic_alpha * new_value + 
                         (1 - dynamic_alpha) * previous_value)
        
        self.input_filter[input_name] = filtered_value
        return filtered_value

    def _handle_speed_triggers(self, left_trigger, right_trigger):
        """Manejar cambios de velocidad con triggers"""
        current_time = time.time()
        
        # Evitar cambios muy frecuentes de velocidad
        if current_time - self.last_speed_change < 0.3:  # 300ms mínimo entre cambios
            return
        
        if left_trigger > 0.2:
            new_speed = max(0, self.current_speed_level - 1)
            if new_speed != self.current_speed_level:
                self.current_speed_level = new_speed
                self.last_speed_change = current_time
                logger.info(f"🔽 Velocidad: {self.current_speed_level * 20 + 10}%")
                    
        elif right_trigger > 0.2:
            new_speed = min(len(self.speed_levels) - 1, self.current_speed_level + 1)
            if new_speed != self.current_speed_level:
                self.current_speed_level = new_speed
                self.last_speed_change = current_time
                logger.info(f"🔼 Velocidad: {self.current_speed_level * 20 + 10}%")

    def _apply_input_filter(self, input_name, new_value):
        """Aplicar filtro exponencial para suavizar entrada"""
        if input_name not in self.input_filter:
            self.input_filter[input_name] = new_value
            return new_value
        
        # Filtro exponencial: output = alpha * input + (1-alpha) * previous_output
        filtered_value = (self.filter_alpha * new_value + 
                         (1 - self.filter_alpha) * self.input_filter[input_name])
        
        self.input_filter[input_name] = filtered_value
        return filtered_value

    def _get_accumulated_tcp_movements(self):
        """Obtener movimientos TCP acumulados que superen el umbral"""
        movements = []
        tcp_mapping = {'tcp_x': 0, 'tcp_y': 1, 'tcp_z': 2, 'tcp_rx': 3, 'tcp_ry': 4, 'tcp_rz': 5}
        
        for tcp_name, accumulated in list(self.movement_accumulator.items()):
            if tcp_name.startswith('tcp_') and abs(accumulated) >= self.accumulator_threshold * 0.5:  # Umbral más bajo para TCP
                axis_idx = tcp_mapping[tcp_name]
                movements.append((axis_idx, accumulated))
                # Resetear acumulador
                self.movement_accumulator[tcp_name] = 0
        
        return movements

    def execute_simultaneous_joint_movements(self, movements):
        """Ejecutar múltiples movimientos articulares simultáneamente - SUAVIZADO"""
        if self.emergency_stop_active or self.movement_active:
            return
        
        # Control de cooldown más estricto
        current_time = time.time()
        if current_time - self.last_movement_time < self.movement_cooldown:
            return
        
        try:
            if not self.can_control():
                return
            
            # Usar asíncrono para evitar bloqueos
            current_joints = self.get_current_joint_positions()
            target_joints = current_joints.copy()
            
            # Aplicar todos los movimientos
            speed_factor = self.speed_levels[self.current_speed_level]
            
            for joint_idx, increment in movements:
                if 0 <= joint_idx < 6:
                    target_joints[joint_idx] += increment * speed_factor
            
            # Verificar límites de articulaciones
            joint_limits = {
                0: (-6.28, 6.28),    # Base: ±360°
                1: (-6.28, 6.28),    # Shoulder: ±360° 
                2: (-3.14, 3.14),    # Elbow: ±180°
                3: (-6.28, 6.28),    # Wrist 1: ±360°
                4: (-6.28, 6.28),    # Wrist 2: ±360°
                5: (-6.28, 6.28)     # Wrist 3: ±360°
            }
            
            for i, angle in enumerate(target_joints):
                min_limit, max_limit = joint_limits[i]
                target_joints[i] = np.clip(angle, min_limit, max_limit)
            
            # Parámetros de movimiento ULTRA suaves
            move_speed = self.joint_speed * speed_factor * 0.3  # REDUCIDO de 0.6 a 0.3 para máxima suavidad
            move_accel = self.joint_accel * speed_factor * 0.2  # REDUCIDO de 0.4 a 0.2 para máxima suavidad
            
            # Usar blend radius aumentado para movimientos más suaves
            blend_radius = self.joint_blend_radius * 1.5  # REDUCIDO de 2 a 1.5 para evitar sobresuavizado
            
            # NUEVO: Usar moveJ con blend radius para suavidad
            try:
                self.control.moveJ(
                    target_joints,
                    speed=move_speed,
                    acceleration=move_accel,
                    asynchronous=True,  # Asíncrono para no bloquear
                    blend=blend_radius
                )
                
                if self.debug_mode and len(movements) > 1:
                    joint_names = [f"J{i}" for i, _ in movements]
                    logger.debug(f"🔄 Movimiento suave joints: {', '.join(joint_names)}")
                
            except TypeError:
                # Fallback si blend no está soportado
                self.control.moveJ(
                    target_joints,
                    speed=move_speed,
                    acceleration=move_accel,
                    asynchronous=True
                )
            
            self.last_movement_time = current_time
            
        except Exception as e:
            logger.error(f"Error en movimiento articular suave: {e}")

    def execute_simultaneous_tcp_movements(self, movements):
        """Ejecutar múltiples movimientos TCP simultáneamente - SUAVIZADO"""
        if self.emergency_stop_active or self.movement_active:
            return
        
        # Control de cooldown más estricto
        current_time = time.time()
        if current_time - self.last_movement_time < self.movement_cooldown:
            return
        
        try:
            if not self.can_control():
                return
            
            current_pose = self.get_current_tcp_pose()
            target_pose = current_pose.copy()
            
            # Aplicar todos los movimientos
            speed_factor = self.speed_levels[self.current_speed_level]
            
            for axis_idx, increment in movements:
                if 0 <= axis_idx < 6:
                    target_pose[axis_idx] += increment * speed_factor
            
            # Verificar límites del workspace para posición
            x, y, z = target_pose[:3]
            if not self.is_point_within_reach(x, y, z):
                if self.debug_mode:
                    logger.warning(f"🚫 Posición fuera del workspace: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
                return
            
            # Parámetros de movimiento ULTRA suaves
            move_speed = self.linear_speed * speed_factor * 0.25  # REDUCIDO de 0.5 a 0.25 para máxima suavidad
            move_accel = self.linear_accel * speed_factor * 0.15  # REDUCIDO de 0.3 a 0.15 para máxima suavidad
            
            # Usar blend radius aumentado
            blend_radius = self.linear_blend_radius * 2  # Mantener triplicado para TCP
            
            # NUEVO: Usar moveL con blend radius para suavidad
            try:
                self.control.moveL(
                    target_pose,
                    speed=move_speed,
                    acceleration=move_accel,
                    asynchronous=True,
                    blend=blend_radius
                )
                
                if self.debug_mode and len(movements) > 1:
                    axis_names = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
                    moved_axes = [axis_names[i] for i, _ in movements]
                    logger.debug(f"🔄 Movimiento suave TCP: {', '.join(moved_axes)}")
                
            except TypeError:
                # Fallback si blend no está soportado
                self.control.moveL(
                    target_pose,
                    speed=move_speed,
                    acceleration=move_accel,
                    asynchronous=True
                )
            
            self.last_movement_time = current_time
            
        except Exception as e:
            logger.error(f"Error en movimiento TCP suave: {e}")

    def _show_xbox_status(self):
        """Mostrar estado del control Xbox"""
        logger.info("🎮 === ESTADO CONTROL XBOX ===")
        logger.info(f"🔄 Modo: {self.control_mode.upper()}")
        logger.info(f"⚡ Velocidad: {self.speed_levels[self.current_speed_level]*100:.0f}%")
        logger.info(f"🚨 Emergencia: {'ACTIVA' if self.emergency_stop_active else 'INACTIVA'}")
        logger.info(f"🔗 Robot conectado: {'SÍ' if self.is_connected() else 'NO'}")
        logger.info(f"🎮 Xbox habilitado: {'SÍ' if self.xbox_enabled else 'NO'}")

    def _debug_all_inputs(self):
        """Debug completo de todas las entradas"""
        if not self.joystick:
            return
            
        logger.debug("🐛 === DEBUG COMPLETO ===")
        
        # Botones
        pressed_buttons = []
        for i in range(self.joystick.get_numbuttons()):
            if self.joystick.get_button(i):
                pressed_buttons.append(f"Btn{i}")
        
        if pressed_buttons:
            logger.debug(f"🔘 Botones: {', '.join(pressed_buttons)}")
        
        # Ejes analógicos
        axes_info = []
        for i in range(self.joystick.get_numaxes()):
            value = self.joystick.get_axis(i)
            if abs(value) > 0.1:
                axes_info.append(f"Axis{i}: {value:.3f}")
        
        if axes_info:
            logger.debug(f"📊 Ejes: {', '.join(axes_info)}")
        
        # D-pad
        for i in range(self.joystick.get_numhats()):
            hat = self.joystick.get_hat(i)
            if hat != (0, 0):
                logger.debug(f"🎯 D-pad {i}: {hat}")

    def get_xbox_status(self):
        """Obtener estado del control Xbox para la interfaz web"""
        return {
            'xbox_enabled': self.xbox_enabled,
            'xbox_connected': self.joystick is not None if self.xbox_enabled else False,
            'xbox_running': self.xbox_running,
            'control_mode': self.control_mode if self.xbox_enabled else None,
            'debug_mode': self.debug_mode if self.xbox_enabled else False,
            'speed_level': self.current_speed_level,
            'speed_percent': self.speed_levels[self.current_speed_level] * 100 if self.current_speed_level < len(self.speed_levels) else 0
        }

    # ========== MÉTODOS DE CONTROL DE VELOCIDADES CONTINUAS ==========

    def process_xbox_input(self):
        """Procesar entrada del control Xbox"""
        pygame.event.pump()
        
        # Procesar botones
        for button_id in range(self.joystick.get_numbuttons()):
            current_state = self.joystick.get_button(button_id)
            previous_state = self.previous_button_states.get(button_id, False)
            
            # Detectar presión de botón (transición False -> True)
            if current_state and not previous_state:
                self.handle_button_press(button_id)
            
            self.previous_button_states[button_id] = current_state
        
        # Procesar entradas analógicas solo si no hay parada de emergencia
        if not self.emergency_stop_active:
            self.process_analog_input()

    def handle_button_press(self, button_id):
        """Manejar presión de botones específicos"""
        button_actions = {
            0: "A",          # a -> 0
            1: "B",          # b -> 1  
            3: "X",          # x -> 3
            4: "Y",          # y -> 4
            6: "LB",         # lb -> 6
            7: "RB",         # rb -> 7
            10: "Menu",      # menu -> 10
            11: "Start",     # start -> 11
        }
        
        button_name = button_actions.get(button_id, f"Btn{button_id}")
        
        if self.debug_mode:
            logger.info(f"🎮 Botón presionado: {button_name} (ID: {button_id})")
        
        if button_id == 0:  # Botón A - Cambiar modo
            if not self.emergency_stop_active:
                old_mode = self.control_mode
                self.control_mode = "joint" if self.control_mode == "linear" else "linear"
                logger.info(f"🔄 Modo cambiado: {old_mode} → {self.control_mode}")
                
                # Detener movimiento al cambiar modo
                self.stop_all_movement()
        
        elif button_id == 1:  # Botón B - Parada de emergencia / Desactivar
            if self.emergency_stop_active:
                self.deactivate_emergency_stop()
            else:
                self.activate_emergency_stop()
        
        elif button_id == 3:  # Botón X - Ir a posición Home
            if not self.emergency_stop_active:
                logger.info("🏠 Yendo a posición Home...")
                self.go_home()
        
        elif button_id == 4:  # Botón Y - Enviar comando TFORCE 500
            if not self.emergency_stop_active and self.gripper_enabled:
                logger.info("🦾 Enviando comando MOVE GRIP TFORCE 500...")
                self.gripper_move_tforce(500)
        
        elif button_id == 6:  # LB - Reducir velocidad
            if not self.emergency_stop_active and self.current_speed_level > 0:
                self.current_speed_level -= 1
                logger.info(f"🔽 Velocidad reducida: Nivel {self.current_speed_level + 1}/5 ({self.speed_levels[self.current_speed_level]*100:.0f}%)")
        
        elif button_id == 7:  # RB - Aumentar velocidad
            if not self.emergency_stop_active and self.current_speed_level < len(self.speed_levels) - 1:
                self.current_speed_level += 1
                logger.info(f"🔼 Velocidad aumentada: Nivel {self.current_speed_level + 1}/5 ({self.speed_levels[self.current_speed_level]*100:.0f}%)")
        
        elif button_id == 11:  # Start - Mostrar información y toggle luz gripper
            self.show_status()
            if self.gripper_enabled:
                self.gripper_light_toggle()
        
        elif button_id == 10:  # Menu - Toggle debug mode
            self.debug_mode = not self.debug_mode
            logger.info(f"🐛 Modo debug: {'ACTIVADO' if self.debug_mode else 'DESACTIVADO'}")

    def process_analog_input(self):
        """Procesar entrada de joysticks analógicos"""
        # Obtener valores de joysticks
        left_x = self.apply_deadzone(self.joystick.get_axis(0))
        left_y = self.apply_deadzone(-self.joystick.get_axis(1))  # Invertir Y
        right_x = self.apply_deadzone(self.joystick.get_axis(2))
        right_y = self.apply_deadzone(-self.joystick.get_axis(3))  # Invertir Y
        
        # Obtener triggers para control del gripper
        if self.joystick.get_numaxes() > 4:
            left_trigger = self.joystick.get_axis(4)  # Gatillo izquierdo (LT)
            right_trigger = self.joystick.get_axis(5)  # Gatillo derecho (RT)
            
            # Normalizar triggers (de -1,1 a 0,1)
            left_trigger = (left_trigger + 1) / 2 if self.joystick.get_numaxes() > 4 else 0
            right_trigger = (right_trigger + 1) / 2 if self.joystick.get_numaxes() > 5 else 0
            
            # Procesar comandos personalizados de gatillos
            self.process_custom_trigger_commands(left_trigger, right_trigger)
        
        # Obtener D-pad (ahora usado en lugar de triggers)
        dpad = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
        
        # Aplicar curva de respuesta suave
        def smooth_response(value):
            return np.sign(value) * (value ** 2)
        
        # Calcular velocidades según el modo
        if self.control_mode == "linear":
            velocities = self.calculate_linear_velocities(
                left_x, left_y, right_x, right_y, dpad, smooth_response
            )
            self.update_velocities(velocities, "linear")
        else:
            velocities = self.calculate_joint_velocities(
                left_x, left_y, right_x, right_y, dpad, smooth_response
            )
            self.update_velocities(velocities, "joint")

    def calculate_linear_velocities(self, left_x, left_y, right_x, right_y, dpad, smooth_func):
        """Calcular velocidades lineales del TCP"""
        speed_factor = self.speed_levels[self.current_speed_level]
        
        # Velocidades lineales
        vx = smooth_func(left_x) * self.max_linear_velocity['xy'] * speed_factor
        vy = smooth_func(left_y) * self.max_linear_velocity['xy'] * speed_factor
        vz = smooth_func(right_y) * self.max_linear_velocity['z'] * speed_factor
        
        # Velocidades rotacionales
        wx = smooth_func(right_x) * self.max_linear_velocity['rot'] * speed_factor * 0.3
        
        # D-pad controla rotación Y (arriba/abajo del D-pad)
        wy = dpad[1] * self.max_linear_velocity['rot'] * speed_factor * 0.3
        
        # D-pad controla rotación Z (izquierda/derecha del D-pad)
        wz = dpad[0] * self.max_linear_velocity['rot'] * speed_factor * 0.3
        
        return [vx, vy, vz, wx, wy, wz]

    def calculate_joint_velocities(self, left_x, left_y, right_x, right_y, dpad, smooth_func):
        """Calcular velocidades articulares"""
        speed_factor = self.speed_levels[self.current_speed_level]
        velocities = [0.0] * 6
        
        # Joystick izquierdo controla joints 0 y 1
        velocities[0] = smooth_func(left_x) * self.max_joint_velocity[0] * speed_factor
        velocities[1] = smooth_func(left_y) * self.max_joint_velocity[1] * speed_factor
        
        # Joystick derecho controla joints 2 y 3
        velocities[2] = smooth_func(right_y) * self.max_joint_velocity[2] * speed_factor
        velocities[3] = smooth_func(right_x) * self.max_joint_velocity[3] * speed_factor
        
        # D-pad controla joint 4 (arriba/abajo del D-pad)
        velocities[4] = dpad[1] * self.max_joint_velocity[4] * speed_factor
        
        # D-pad controla joint 5 (izquierda/derecha del D-pad)
        velocities[5] = dpad[0] * self.max_joint_velocity[5] * speed_factor
        
        return velocities

    def show_status(self):
        """Mostrar estado actual del sistema"""
        logger.info("\n" + "="*60)
        logger.info("🤖 ESTADO DEL CONTROLADOR UR5e POR VELOCIDAD")
        logger.info("="*60)
        logger.info(f"🎮 Control: {self.joystick.get_name() if self.joystick else 'No conectado'}")
        logger.info(f"🔄 Modo: {self.control_mode.upper()}")
        logger.info(f"⚡ Velocidad: Nivel {self.current_speed_level + 1}/5 ({self.speed_levels[self.current_speed_level]*100:.0f}%)")
        logger.info(f"📡 Conexión: {'OK' if self.is_connected() else 'ERROR'}")
        logger.info(f"🚨 Parada emergencia: {'ACTIVA' if self.emergency_stop_active else 'INACTIVA'}")
        logger.info(f"🐛 Debug mode: {'ON' if self.debug_mode else 'OFF'}")
        logger.info(f"⚡ Control velocidad: {'ACTIVO' if self.velocity_active else 'INACTIVO'}")
        
        # Velocidades actuales
        try:
            with self.velocity_lock:
                if self.control_mode == "linear":
                    vx, vy, vz, wx, wy, wz = self.current_velocities['linear']
                    logger.info(f"\n🎯 Velocidades lineales actuales:")
                    logger.info(f"  VX: {vx*1000:+7.1f} mm/s")
                    logger.info(f"  VY: {vy*1000:+7.1f} mm/s")
                    logger.info(f"  VZ: {vz*1000:+7.1f} mm/s")
                    logger.info(f"  WX: {np.degrees(wx):+7.1f} °/s")
                    logger.info(f"  WY: {np.degrees(wy):+7.1f} °/s")
                    logger.info(f"  WZ: {np.degrees(wz):+7.1f} °/s")
                else:
                    q0, q1, q2, q3, q4, q5 = self.current_velocities['joint']
                    logger.info(f"\n🔗 Velocidades articulares actuales:")
                    for i, vel in enumerate([q0, q1, q2, q3, q4, q5]):
                        logger.info(f"  Joint {i}: {np.degrees(vel):+7.1f} °/s")
        except Exception as e:
            logger.info(f"  Error mostrando velocidades: {e}")
        
        logger.info("="*60)

    # ========== MÉTODOS DE CONTROL DEL GRIPPER ==========

    def process_custom_trigger_commands(self, left_trigger, right_trigger):
        """Procesar comandos personalizados de gatillos LT y RT"""
        if not self.gripper_enabled or not self.gripper_controller:
            return
            
        # Procesar gatillo izquierdo (LT) - "MOVE GRIP OPEN"
        if left_trigger > self.trigger_press_threshold:
            if not self.left_trigger_pressed:
                # Gatillo recién presionado - enviar comando
                logger.info("🦾 LT presionado - Enviando comando MOVE GRIP OPEN...")
                self.gripper_move_open()
                self.left_trigger_pressed = True
        else:
            # Gatillo liberado
            self.left_trigger_pressed = False
            
        # Procesar gatillo derecho (RT) - "MOVE GRIP STEPS 1000"
        if right_trigger > self.trigger_press_threshold:
            if not self.right_trigger_custom_pressed:
                # Gatillo recién presionado - enviar comando
                logger.info("🦾 RT presionado - Enviando comando MOVE GRIP STEPS 1000...")
                self.gripper_move_steps_custom(1000)
                self.right_trigger_custom_pressed = True
        else:
            # Gatillo liberado
            self.right_trigger_custom_pressed = False

    def gripper_move_tforce(self, force_val):
        """Enviar comando MOVE GRIP TFORCE"""
        if not self.gripper_enabled or not self.gripper_controller:
            return False
        try:
            command = f"MOVE GRIP TFORCE {force_val}"
            logger.info(f"🦾 Enviando comando raw: {command}")
            success, response = self.gripper_controller.send_raw_command(command)
            return success
        except Exception as e:
            logger.error(f"❌ Error enviando TFORCE: {e}")
            return False

    def gripper_move_open(self):
        """Enviar comando MOVE GRIP OPEN"""
        if not self.gripper_enabled or not self.gripper_controller:
            return False
        try:
            command = "MOVE GRIP OPEN"
            logger.info(f"🦾 Enviando comando raw: {command}")
            success, response = self.gripper_controller.send_raw_command(command)
            return success
        except Exception as e:
            logger.error(f"❌ Error enviando OPEN: {e}")
            return False

    def gripper_move_steps_custom(self, steps):
        """Enviar comando MOVE GRIP STEPS"""
        if not self.gripper_enabled or not self.gripper_controller:
            return False
        try:
            command = f"MOVE GRIP STEPS {steps}"
            logger.info(f"🦾 Enviando comando raw: {command}")
            success, response = self.gripper_controller.send_raw_command(command)
            return success
        except Exception as e:
            logger.error(f"❌ Error enviando STEPS: {e}")
            return False

    def process_gripper_control(self, right_trigger_value):
        """Procesar control del gripper con gatillo derecho"""
        if not self.gripper_enabled or not self.gripper_controller:
            return
        
        current_time = time.time()
        
        # Agregar valor al buffer para promedio de 4 segundos
        self.right_trigger_values.append({
            'value': right_trigger_value,
            'time': current_time
        })
        
        # Limpiar valores antiguos (más de 4 segundos)
        self.right_trigger_values = [
            item for item in self.right_trigger_values 
            if current_time - item['time'] <= self.right_trigger_buffer_duration
        ]
        
        # Calcular promedio de los últimos 4 segundos
        if len(self.right_trigger_values) > 0:
            average_trigger = sum(item['value'] for item in self.right_trigger_values) / len(self.right_trigger_values)
            
            # Mapear de 0-1 a 0-5000 steps y redondear a 1 decimal
            mapped_steps = round(average_trigger * 5000, 1)
            
            # Verificar si se debe activar cierre por umbral
            if right_trigger_value > self.trigger_threshold:
                if not self.trigger_was_above_threshold:
                    # Primer cruce del umbral - cerrar gripper
                    logger.info(f"🦾 Gatillo > {self.trigger_threshold}: Cerrando gripper {self.close_steps} pasos")
                    self.gripper_close_steps(self.close_steps)
                    self.trigger_was_above_threshold = True
            else:
                # Gatillo por debajo del umbral
                if self.trigger_was_above_threshold:
                    # Resetear estado para permitir nuevo cierre
                    self.trigger_was_above_threshold = False
            
            # Mover gripper a posición basada en promedio (solo si hay cambio significativo)
            if len(self.right_trigger_values) >= 5:  # Esperar suficientes datos
                if abs(mapped_steps - getattr(self, 'last_mapped_steps', 0)) > 50:  # Cambio mínimo de 50 pasos
                    if self.debug_mode:
                        logger.info(f"🦾 Gatillo promedio: {average_trigger:.3f} → {mapped_steps:.1f} pasos")
                    
                    self.gripper_move_to_steps(mapped_steps)
                    self.last_mapped_steps = mapped_steps

    def gripper_home(self):
        """Mover gripper a posición home"""
        if not self.gripper_enabled or not self.gripper_controller:
            logger.warning("❌ Gripper no disponible")
            return False
        
        try:
            logger.info("🦾 Ejecutando MOVE GRIP HOME...")
            result = self.gripper_controller.usense_home_gripper(caller_info="XboxController")
            if result:
                logger.info("✅ Gripper movido a HOME exitosamente")
                return True
            else:
                logger.warning("⚠️ Comando de HOME enviado (sin respuesta del gripper)")
                return True  # Consideramos éxito si se envió el comando
        except Exception as e:
            logger.error(f"❌ Error moviendo gripper a HOME: {e}")
            return False

    def gripper_close_steps(self, steps):
        """Cerrar gripper un número específico de pasos"""
        if not self.gripper_enabled or not self.gripper_controller:
            logger.warning("❌ Gripper no disponible")
            return False
        
        try:
            logger.info(f"🦾 Cerrando gripper {steps} pasos...")
            result = self.gripper_controller.usense_move_steps(-abs(steps), caller_info="XboxController")  # Negativo para cerrar
            if result:
                logger.info(f"✅ Gripper cerrado {steps} pasos exitosamente")
                return True
            else:
                logger.warning(f"⚠️ Comando de cierre {steps} pasos enviado (sin respuesta del gripper)")
                return True  # Consideramos éxito si se envió el comando
        except Exception as e:
            logger.error(f"❌ Error cerrando gripper: {e}")
            return False

    def gripper_move_to_steps(self, target_steps):
        """Mover gripper a posición específica en pasos absolutos"""
        if not self.gripper_enabled or not self.gripper_controller:
            return False
        
        try:
            # Convertir de posición absoluta a movimiento relativo
            # Nota: Este es un mapeo simplificado, podría necesitar ajustes
            result = self.gripper_controller.usense_move_steps(int(target_steps), caller_info="XboxController")
            if self.debug_mode:
                logger.debug(f"🦾 Gripper moviéndose a {target_steps:.1f} pasos")
            return result
        except Exception as e:
            if self.debug_mode:
                logger.error(f"❌ Error moviendo gripper a {target_steps} pasos: {e}")
            return False

    def get_gripper_status(self):
        """Obtener estado del gripper para la interfaz web"""
        if not self.gripper_enabled or not self.gripper_controller:
            return {
                'gripper_enabled': False,
                'gripper_connected': False,
                'current_steps': 0,
                'trigger_average': 0.0
            }
        
        # Calcular promedio actual del gatillo
        current_time = time.time()
        valid_values = [
            item['value'] for item in self.right_trigger_values 
            if current_time - item['time'] <= self.right_trigger_buffer_duration
        ]
        
        average_trigger = sum(valid_values) / len(valid_values) if valid_values else 0.0
        mapped_steps = round(average_trigger * 5000, 1)
        
        return {
            'gripper_enabled': self.gripper_enabled,
            'gripper_connected': self.gripper_controller.connected if self.gripper_controller else False,
            'current_steps': mapped_steps,
            'trigger_average': average_trigger,
            'trigger_above_threshold': self.trigger_was_above_threshold
        }

    def gripper_light_toggle(self):
        """Toggle de la luz del gripper"""
        if not self.gripper_enabled or not self.gripper_controller:
            logger.warning("❌ Gripper no disponible")
            return False
        
        try:
            logger.info("💡 Ejecutando toggle de luz del gripper...")
            result = self.gripper_controller.usense_light_toggle()
            if result:
                logger.info("✅ Toggle de luz ejecutado exitosamente")
                return True
            else:
                logger.warning("⚠️ Comando de toggle de luz enviado (sin respuesta del gripper)")
                return True  # Consideramos éxito si se envió el comando
        except Exception as e:
            logger.error(f"❌ Error ejecutando toggle de luz: {e}")
            return False

