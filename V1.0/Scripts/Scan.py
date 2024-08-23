import socket
import numpy as np
import struct
from time import sleep

def send_instruction_to_ur5(ip, port, instruction):
    try:
        #Socket object
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        #Connect to robot
        sock.connect((ip, port))
        
        #Send the instruction to the robot
        sock.sendall(instruction.encode())
        
        print("Instruction sent to UR5 robot:", instruction)
        
        #Close the socket
        sock.close()
        
    except ConnectionRefusedError:
        print("Connection refused")
    
    except TimeoutError:
        print("Timeout, can't connect")

def freedrive(ur5_ip, ur5_port):
    while True:
        send_instruction_to_ur5(ur5_ip, ur5_port, 'freedrive_mode()\n')
        sleep(1)

def saludar_continuo(ur5_ip, ur5_port):
    ## Posicion 1A
    Angles_list_1=[38.52,-56.92,-82.72,-130.37,90.09,128.50]
    Angles_list_2=[-65.37,-71.16,-76.84,-122.03,90,24.61] 
    Angles_list_3=[-95.32,-100.84,-49.71,-119.44,89.95,-5.37]
    Angles_list_4=[-76.41,-119.97,-21.90,-128.13,89.93,13.52]
    Angles_list_5=[-46.44,-97.95,-59.21,-112.85,88.59,37.12]
    Angles_list_6=[-9.08,-94.58,-62.79,-113.49,88.90,74.50]
    #Convertir a radianes la lista de angulos
    Angles_list_1=[np.radians(i) for i in Angles_list_1]
    Angles_list_2=[np.radians(i) for i in Angles_list_2]
    Angles_list_3=[np.radians(i) for i in Angles_list_3]
    Angles_list_4=[np.radians(i) for i in Angles_list_4]
    Angles_list_5=[np.radians(i) for i in Angles_list_5]
    Angles_list_6=[np.radians(i) for i in Angles_list_6]
    ur5_move_command_a1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
    ur5_move_command_a2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"
    ur5_move_command_a3 = f"movej([{Angles_list_3[0]}, {Angles_list_3[1]}, {Angles_list_3[2]}, {Angles_list_3[3]}, {Angles_list_3[4]}, {Angles_list_3[5]}], a=1, v=1)\n"
    ur5_move_command_a4 = f"movej([{Angles_list_4[0]}, {Angles_list_4[1]}, {Angles_list_4[2]}, {Angles_list_4[3]}, {Angles_list_4[4]}, {Angles_list_4[5]}], a=1, v=1)\n"
    ur5_move_command_a5 = f"movej([{Angles_list_5[0]}, {Angles_list_5[1]}, {Angles_list_5[2]}, {Angles_list_5[3]}, {Angles_list_5[4]}, {Angles_list_5[5]}], a=1, v=1)\n"
    ur5_move_command_a6 = f"movej([{Angles_list_6[0]}, {Angles_list_6[1]}, {Angles_list_6[2]}, {Angles_list_6[3]}, {Angles_list_6[4]}, {Angles_list_6[5]}], a=1, v=1)\n"

    # Posicion 1M
    Angles_list_1=[32.22,-46.08,-134.34,-90.94,89.84,115.80]
    Angles_list_2=[-60.65,-52.42,-132.92,-84.32,88.72,22.63] #mal
    Angles_list_3=[-99.85,-100.97,-96.56,-71.36,89.13,-16.30]
    Angles_list_4=[-72.70,-129.06,-55.30,-85.03,88.68,10.81]
    Angles_list_5=[-46.34,-105.23,-90.90,-73.89,88.58,37.22] #mal
    Angles_list_6=[-13.19,-100.92,-96.11,-73.74,88.83,70.38]
    #Convertir a radianes la lista de angulos
    Angles_list_1=[np.radians(i) for i in Angles_list_1]
    Angles_list_2=[np.radians(i) for i in Angles_list_2]
    Angles_list_3=[np.radians(i) for i in Angles_list_3]
    Angles_list_4=[np.radians(i) for i in Angles_list_4]
    Angles_list_5=[np.radians(i) for i in Angles_list_5]
    Angles_list_6=[np.radians(i) for i in Angles_list_6]
    ur5_move_command_m1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
    ur5_move_command_m2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"
    ur5_move_command_m3 = f"movej([{Angles_list_3[0]}, {Angles_list_3[1]}, {Angles_list_3[2]}, {Angles_list_3[3]}, {Angles_list_3[4]}, {Angles_list_3[5]}], a=1, v=1)\n"
    ur5_move_command_m4 = f"movej([{Angles_list_4[0]}, {Angles_list_4[1]}, {Angles_list_4[2]}, {Angles_list_4[3]}, {Angles_list_4[4]}, {Angles_list_4[5]}], a=1, v=1)\n"
    ur5_move_command_m5 = f"movej([{Angles_list_5[0]}, {Angles_list_5[1]}, {Angles_list_5[2]}, {Angles_list_5[3]}, {Angles_list_5[4]}, {Angles_list_5[5]}], a=1, v=1)\n"
    ur5_move_command_m6 = f"movej([{Angles_list_6[0]}, {Angles_list_6[1]}, {Angles_list_6[2]}, {Angles_list_6[3]}, {Angles_list_6[4]}, {Angles_list_6[5]}], a=1, v=1)\n"

    # Posicion 1B
    Angles_list_1=[18.61,-60.30,-155.26,-55.70,89.51,102.20]
    Angles_list_2=[-63.34,-68.71,-152.37,-48.53,88.70,20.24]
    Angles_list_3=[-99.54,-112.04,-108.85,-48,89.10,-16]
    Angles_list_4=[-72.51,-137.10,-65.57,-66.74,88.68,10.98]
    Angles_list_5=[-47.34,-113.87,-105.65,-50.49,88.57,36.19]
    Angles_list_6=[-12.36,-112.87,-106.94,-50.98,88.83,71.19]
    #Convertir a radianes la lista de angulos
    Angles_list_1=[np.radians(i) for i in Angles_list_1]
    Angles_list_2=[np.radians(i) for i in Angles_list_2]
    Angles_list_3=[np.radians(i) for i in Angles_list_3]
    Angles_list_4=[np.radians(i) for i in Angles_list_4]
    Angles_list_5=[np.radians(i) for i in Angles_list_5]
    Angles_list_6=[np.radians(i) for i in Angles_list_6]
    ur5_move_command_b1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
    ur5_move_command_b2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"
    ur5_move_command_b3 = f"movej([{Angles_list_3[0]}, {Angles_list_3[1]}, {Angles_list_3[2]}, {Angles_list_3[3]}, {Angles_list_3[4]}, {Angles_list_3[5]}], a=1, v=1)\n"
    ur5_move_command_b4 = f"movej([{Angles_list_4[0]}, {Angles_list_4[1]}, {Angles_list_4[2]}, {Angles_list_4[3]}, {Angles_list_4[4]}, {Angles_list_4[5]}], a=1, v=1)\n"
    ur5_move_command_b5 = f"movej([{Angles_list_5[0]}, {Angles_list_5[1]}, {Angles_list_5[2]}, {Angles_list_5[3]}, {Angles_list_5[4]}, {Angles_list_5[5]}], a=1, v=1)\n"
    ur5_move_command_b6 = f"movej([{Angles_list_6[0]}, {Angles_list_6[1]}, {Angles_list_6[2]}, {Angles_list_6[3]}, {Angles_list_6[4]}, {Angles_list_6[5]}], a=1, v=1)\n"
    # while True:
    #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s1)
    #     sleep(3)
    #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s2)
    #     sleep(3)
    #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s3)
    #     sleep(3.5)
    #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s4)
    #     sleep(4)
    send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_m6)

x = 0.2
y = -.7
z = 0.01

ur5_ip = "192.168.1.1" 
ur5_port = 30002 

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

saludar_continuo(ur5_ip, ur5_port)
#send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_7)