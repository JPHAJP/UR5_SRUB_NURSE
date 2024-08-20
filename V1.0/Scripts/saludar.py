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
    #Saludar variable
    Angles_list_1=[-71.96,-90.60,-99.21,2.23,122.58,37.98]
    Angles_list_2=[-17.52,-134.93,-70.15,30.38,68.76,42.41]
    Angles_list_3=[-65.14,-147.04,-70.09,42.23,117.62,44.25]
    Angles_list_4=[-65.21,-65.16,-69.04,-62.73,110.99,37.60]
    #Convertir a radianes la lista de angulos
    Angles_list_1=[np.radians(i) for i in Angles_list_1]
    Angles_list_2=[np.radians(i) for i in Angles_list_2]
    Angles_list_3=[np.radians(i) for i in Angles_list_3]
    Angles_list_4=[np.radians(i) for i in Angles_list_4]
    ur5_move_command_s1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
    ur5_move_command_s2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"
    ur5_move_command_s3 = f"movej([{Angles_list_3[0]}, {Angles_list_3[1]}, {Angles_list_3[2]}, {Angles_list_3[3]}, {Angles_list_3[4]}, {Angles_list_3[5]}], a=1, v=1)\n"
    ur5_move_command_s4 = f"movej([{Angles_list_4[0]}, {Angles_list_4[1]}, {Angles_list_4[2]}, {Angles_list_4[3]}, {Angles_list_4[4]}, {Angles_list_4[5]}], a=1, v=1)\n"
    while True:
        send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s1)
        sleep(3)
        send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s2)
        sleep(3)
        send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s3)
        sleep(3.5)
        send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_s4)
        sleep(4)

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