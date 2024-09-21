import numpy as np
import time

import rtde_control
import rtde_receive
import rtde_io

def inicializar_robot():
    try:
        ip = "192.168.1.1"
        control = rtde_control.RTDEControlInterface(ip)
        receive = rtde_receive.RTDEReceiveInterface(ip)
        io = rtde_io.RTDEIOInterface(ip)
        return control, receive, io
    except:
        print("Error al inicializar el robot.")
        time.sleep(1)
        return None, None, None

def gohome(control):
    # Función para mover el robot a la posición "Home"
    # Coordenadas articulares para el home
    home_joint_angles_deg = [-51.9, -71.85, -112.7, -85.96, 90, 38]
    # Convertir la lista de ángulos a radianes
    home_joint_angles_rad = np.radians(home_joint_angles_deg)
    # Mover el robot a la posición "Home" usando control.moveJ
    # Velocidad = 1 rad/s, Aceleración = 1 rad/s^2
    control.moveJ(home_joint_angles_rad, 1, 1)

def move_robot(xtransfn, ytransfn, ofzn, control, receive):
    def is_point_within_reach(point):
        """
        Verifica si un punto está dentro del alcance del UR5-e.
        
        Parámetro:
        point (tuple): Una tupla con las coordenadas (x, y, z) del punto en milímetros.
        
        Retorna:
        bool: True si el punto está dentro del alcance, False de lo contrario.
        """
        # Calcula la distancia euclidiana desde la base (0, 0, 0) al punto
        distance = np.linalg.norm(point)
        print(distance)
    
        # Verifica si la distancia está dentro del rango máximo
        UR5E_MAX_REACH = .9
        return distance <= UR5E_MAX_REACH and distance >= .3
    is_point_on_work=is_point_within_reach([xtransfn, ytransfn, ofzn])
    if not is_point_on_work:
        print("Punto fuera de alcance")
        return
    xr, yr, zr, rxr, ryr, rzr = receive.getActualTCPPose()

    control.moveL([xtransfn, ytransfn, ofzn, rxr, ryr, rzr], .5, .5)
    return

def main():
    # Inicializar el robot
    control = None
    while control is None:
        control, receive, io = inicializar_robot()
    
    # Mover el robot a la posición "Home"
    gohome(control)
    time.sleep(3)

    # while True:
    #     # Input de coordenadas x, y, z
    #     xtransfn = float(input("Ingrese la coordenada x: "))
    #     ytransfn = float(input("Ingrese la coordenada y: "))
    #     ofzn = float(input("Ingrese la coordenada z: "))

    #     xtransfn = xtransfn / 1000
    #     ytransfn = ytransfn / 1000
    #     ofzn = ofzn / 1000

    #     # Mover el robot a la posición deseada

    #     move_robot(xtransfn, ytransfn, ofzn, control, receive)

    xtransfn = -.15
    ytransfn = -.15
    ofzn = .835
    # Mover el robot a la posición deseada

    move_robot(xtransfn, ytransfn, ofzn, control, receive)

if __name__ == "__main__":
    main()
