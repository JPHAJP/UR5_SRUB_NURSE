#Communication between the robot and python
import socket
import numpy as np
import struct
from time import sleep

def getStatus():
    # Conexión con el UR5
    HOST = '192.168.1.1'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))

    cdr = 0
    while cdr < 3:
        data = s.recv(4096)
        i = 0
        if data:
            packlen = (struct.unpack('!i', data[0:4]))[0]
            timestamp = (struct.unpack('!Q', data[10:18]))[0]
            packtype = (struct.unpack('!b', data[4:5]))[0]

            if packtype == 16:
                while i + 5 < packlen:
                    msglen = (struct.unpack('!i', data[5+i:9+i]))[0]
                    msgtype = (struct.unpack('!b', data[9+i:10+i]))[0]

                    if msgtype == 1:
                        angle = [0] * 6
                        j = 0
                        while j < 6:
                            angle[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
                            j += 1

                    elif msgtype == 4:
                        x = (struct.unpack('!d', data[10+i:18+i]))[0]
                        y = (struct.unpack('!d', data[18+i:26+i]))[0]
                        z = (struct.unpack('!d', data[26+i:34+i]))[0]
                        rx = (struct.unpack('!d', data[34+i:42+i]))[0]
                        ry = (struct.unpack('!d', data[42+i:50+i]))[0]
                        rz = (struct.unpack('!d', data[50+i:58+i]))[0]

                    # Para depurar, imprimimos el mensaje completo
                    print(f"msgtype: {msgtype}, data: {data[5+i:5+i+msglen]}")
                    
                    i += msglen
        cdr += 1

    s.close()
    return x, y, z, rx, ry, rz, angle[5]

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

x = 0.2
y = -.7
z = 0.01

ur5_ip = "192.168.1.1" 
ur5_port = 30002 

#Home
Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]

#Saludar variable
Angles_list_1=[-106.75,-100.55,-79.78,-3.93,152.10,38]
Angles_list_2=[-241.02,2.18,-59.69,-123.5,290.52,-315.06]
#Convertir a radianes la lista de angulos
Angles_list_1=[np.radians(i) for i in Angles_list_1]
Angles_list_2=[np.radians(i) for i in Angles_list_2]

ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n"
ur5_move_command_2 = "movel(p[.064,-.269,.325,2.471,1.940,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 2 para pruebas
ur5_move_command_3 = "movel(p[.064,-.269,.285,3.139,0.126,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 3 para pruebas
ur5_move_command_4 = "movel(p[-.060,-.270,.284,1.388,2.819,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 4 para pruebas
ur5_move_command_5 = "movel(p[-.174,-.286,.281,0.204,3.135,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 5 para pruebas
ur5_move_command_6 = "movel(p[.064,-.269,.400,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 6 para pruebas

ur5_move_command_7 = f"movej([{Angles_list_0[0]}, {Angles_list_0[1]}, {Angles_list_0[2]}, {Angles_list_0[3]}, {Angles_list_0[4]}, {Angles_list_0[5]}], a=1, v=1)\n"
ur5_move_command_8 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
ur5_move_command_9 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"

#Comando para activar el modo de control manual
ur5_move_command_10 = 'freedrive_mode()\n'

send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_7)