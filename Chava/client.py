#Communication between the robot and python
import socket
import numpy as np
import struct
import time

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
        time.sleep(1)

x = 0.2
y = -.7
z = 0.01

ur5_ip = "192.168.1.1" 
ur5_port = 30002 

#Home hardcodeado
T1 = np.radians(-51.9)
T2 = np.radians(-71.85)
T3 = np.radians(-112.7)
T4 = np.radians(-85.96)
T5 = np.radians(90)
T6 = np.radians(38)

ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n"
ur5_move_command_2 = "movel(p[.064,-.269,.325,2.471,1.940,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 2 para pruebas
ur5_move_command_3 = "movel(p[.064,-.269,.285,3.139,0.126,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 3 para pruebas
ur5_move_command_4 = "movel(p[-.060,-.270,.284,1.388,2.819,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 4 para pruebas
ur5_move_command_5 = "movel(p[-.174,-.286,.281,0.204,3.135,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 5 para pruebas
ur5_move_command_6 = "movel(p[.064,-.269,.400,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 6 para pruebas
ur5_move_command_7 = f"movej([{T1}, {T2}, {T3}, {T4}, {T5}, {T6}], a=1, v=1)\n"
#ur5_move_command_7 = "movej([0, -1.5708, -1.5708, -1.5708, 1.5708, 3.1416], a=1, v=1.05)\n"

#Comando para activar el modo de control manual
ur5_move_command_8 = 'freedrive_mode()\n'


send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_7)