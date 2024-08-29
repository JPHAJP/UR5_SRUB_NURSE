import numpy as np
from time import sleep
import rtde_control
import rtde_receive

control = rtde_control.RTDEControlInterface("192.168.1.1")
receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")

def saludar_continuo(ur5_ip, ur5_port):
    #Saludar variable
    #HOME
    Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
    Angles_list_1=[-71.96,-90.60,-99.21,2.23,122.58,37.98]
    Angles_list_2=[-17.52,-134.93,-70.15,30.38,68.76,42.41]
    Angles_list_3=[-65.14,-147.04,-70.09,42.23,117.62,44.25]
    Angles_list_4=[-65.21,-65.16,-69.04,-62.73,110.99,37.60]
    #Convertir a radianes la lista de angulos
    Angles_list_0=[np.radians(i) for i in Angles_list_0]
    Angles_list_1=[np.radians(i) for i in Angles_list_1]
    Angles_list_2=[np.radians(i) for i in Angles_list_2]
    Angles_list_3=[np.radians(i) for i in Angles_list_3]
    Angles_list_4=[np.radians(i) for i in Angles_list_4]
    while True:
        for i in range(0, 5):
            variable_name = f"Angles_list_{i}"
            angles
            angles = eval(variable_name)  # Obtener la lista usando eval
            control.moveJ(angles, 1, 1)
            print(f"{variable_name} en radianes: {angles}")
            print(receive.getActualTCPPose())

#Home
Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]

#Ejemplo movimiento lineal
ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n"
#Ejemplo movimiento por angulos
ur5_move_command_7 = f"movej([{Angles_list_0[0]}, {Angles_list_0[1]}, {Angles_list_0[2]}, {Angles_list_0[3]}, {Angles_list_0[4]}, {Angles_list_0[5]}], a=1, v=1)\n"


#Comando para activar el modo de control manual
ur5_move_command_10 = 'freedrive_mode()\n'

saludar_continuo()