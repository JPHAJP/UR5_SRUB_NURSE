import numpy as np
from time import sleep
import rtde_control
import rtde_receive

control = rtde_control.RTDEControlInterface("192.168.1.1")
receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")


def saludar_continuo():
    ## Posicion 1A
    Angles_list_1a=[38.52,-56.92,-82.72,-130.37,90.09,128.50]
    Angles_list_2a=[-65.37,-71.16,-76.84,-122.03,90,24.61] 
    Angles_list_3a=[-95.32,-100.84,-49.71,-119.44,89.95,-5.37]
    Angles_list_4a=[-76.41,-119.97,-21.90,-128.13,89.93,13.52]
    Angles_list_5a=[-46.44,-97.95,-59.21,-112.85,88.59,37.12]
    Angles_list_6a=[-9.08,-94.58,-62.79,-113.49,88.90,74.50]
    #Convertir a radianes la lista de angulos
    Angles_list_1a=[np.radians(i) for i in Angles_list_1a]
    Angles_list_2a=[np.radians(i) for i in Angles_list_2a]
    Angles_list_3a=[np.radians(i) for i in Angles_list_3a]
    Angles_list_4a=[np.radians(i) for i in Angles_list_4a]
    Angles_list_5a=[np.radians(i) for i in Angles_list_5a]
    Angles_list_6a=[np.radians(i) for i in Angles_list_6a]
    
    # Posicion 1M
    Angles_list_1m=[32.22,-46.08,-134.34,-90.94,89.84,115.80]
    Angles_list_2m=[-60.65,-52.42,-132.92,-84.32,88.72,22.63] #mal
    Angles_list_3m=[-99.85,-100.97,-96.56,-71.36,89.13,-16.30]
    Angles_list_4m=[-72.70,-129.06,-55.30,-85.03,88.68,10.81]
    Angles_list_5m=[-46.34,-105.23,-90.90,-73.89,88.58,37.22] #mal
    Angles_list_6m=[-13.19,-100.92,-96.11,-73.74,88.83,70.38]
    #Convertir a radianes la lista de angulos
    Angles_list_1m=[np.radians(i) for i in Angles_list_1m]
    Angles_list_2m=[np.radians(i) for i in Angles_list_2m]
    Angles_list_3m=[np.radians(i) for i in Angles_list_3m]
    Angles_list_4m=[np.radians(i) for i in Angles_list_4m]
    Angles_list_5m=[np.radians(i) for i in Angles_list_5m]
    Angles_list_6m=[np.radians(i) for i in Angles_list_6m]
    
    # Posicion 1B
    Angles_list_1b=[18.61,-60.30,-155.26,-55.70,89.51,102.20]
    Angles_list_2b=[-63.34,-68.71,-152.37,-48.53,88.70,20.24]
    Angles_list_3b=[-99.54,-112.04,-108.85,-48,89.10,-16]
    Angles_list_4b=[-72.51,-137.10,-65.57,-66.74,88.68,10.98]
    Angles_list_5b=[-47.34,-113.87,-105.65,-50.49,88.57,36.19]
    Angles_list_6b=[-12.36,-112.87,-106.94,-50.98,88.83,71.19]
    #Convertir a radianes la lista de angulos
    Angles_list_1b=[np.radians(i) for i in Angles_list_1b]
    Angles_list_2b=[np.radians(i) for i in Angles_list_2b]
    Angles_list_3b=[np.radians(i) for i in Angles_list_3b]
    Angles_list_4b=[np.radians(i) for i in Angles_list_4b]
    Angles_list_5b=[np.radians(i) for i in Angles_list_5b]
    Angles_list_6b=[np.radians(i) for i in Angles_list_6b]
    
    while True:
        for i in range(1, 7):
            variable_name = f"Angles_list_{i}a"
            angles = eval(variable_name)  # Obtener la lista usando eval
            control.moveJ(angles, 1, 1)
            print(f"{variable_name} en radianes: {angles}")
            print(receive.getActualTCPPose())
        for i in range(1, 7):
            variable_name = f"Angles_list_{i}m"
            angles = eval(variable_name)  # Obtener la lista usando eval
            control.moveJ(angles, 1, 1)
            print(f"{variable_name} en radianes: {angles}")
            print(receive.getActualTCPPose())
        for i in range(1, 7):
            variable_name = f"Angles_list_{i}b"
            angles = eval(variable_name)  # Obtener la lista usando eval
            control.moveJ(angles, 1, 1)
            print(f"{variable_name} en radianes: {angles}")
            print(receive.getActualTCPPose())

#Home
Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]

#Ejemplo movimiento lineal
#ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n"
#Ejemplo movimiento por angulos
#ur5_move_command_7 = f"movej([{Angles_list_0[0]}, {Angles_list_0[1]}, {Angles_list_0[2]}, {Angles_list_0[3]}, {Angles_list_0[4]}, {Angles_list_0[5]}], a=1, v=1)\n"

#Comunicacion en tiempo real
control.moveJ([Angles_list_0[0], Angles_list_0[1], Angles_list_0[2], Angles_list_0[3], Angles_list_0[4], Angles_list_0[5]], 1, 1)
#print(receive.getActualTCPPose())
#Comando para activar el modo de control manual
#ur5_move_command_10 = 'freedrive_mode()\n'
saludar_continuo()