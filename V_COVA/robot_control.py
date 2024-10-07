import time
import rtde_control
import rtde_receive
import rtde_io

class RobotControl:
    def __init__(self, ip="192.168.1.1"):
        self.ip = ip
        self.control = None
        self.receive = None
        self.io = None
    
    def initialize_robot(self):
        """
        Inicializa la conexión con el robot UR5e utilizando RTDE.
        """
        try:
            self.control = rtde_control.RTDEControlInterface(self.ip)
            self.receive = rtde_receive.RTDEReceiveInterface(self.ip)
            self.io = rtde_io.RTDEIOInterface(self.ip)
            print("Robot inicializado correctamente.")
            return True
        except Exception as e:
            print(f"Error al inicializar el robot: {e}")
            time.sleep(1)
            return False

    def move_to_position(self, position, speed=0.5):
        """
        Mueve el robot a la posición especificada con una velocidad opcional.
        """
        if self.control:
            self.control.moveL(position, speed)
        else:
            print("Control no inicializado.")

    def get_current_position(self):
        """
        Retorna la posición actual del robot.
        """
        if self.receive:
            return self.receive.getActualTCPPose()
        else:
            print("Recepción no inicializada.")
            return None

    def close_connection(self):
        """
        Cierra la conexión con el robot.
        """
        if self.control:
            self.control.disconnect()
            print("Conexión con el robot cerrada.")
        else:
            print("Control no inicializado.")
