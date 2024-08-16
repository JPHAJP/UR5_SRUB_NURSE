####### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #######
import cv2
import pyrealsense2 as rs
import numpy as np
import socket
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

# configurar el pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# configurar UR5
ur5_ip = "192.168.1.1" 
ur5_port = 30002

# Configurar angulos
#MEDIO (HOME)
Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]

#BAJO
Angles_list_1=[-51.9,-84.20,-113.57,-50.73,90,38]
#Convertir a radianes la lista de angulos
Angles_list_1=[np.radians(i) for i in Angles_list_1]

#ALTO
Angles_list_2=[-51.9,-82.72,-60.07,-127.72,90,38]
#Convertir a radianes la lista de angulos
Angles_list_2=[np.radians(i) for i in Angles_list_2]


ur5_move_command_0 = f"movej([{Angles_list_0[0]}, {Angles_list_0[1]}, {Angles_list_0[2]}, {Angles_list_0[3]}, {Angles_list_0[4]}, {Angles_list_0[5]}], a=1, v=1)\n"
ur5_move_command_1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
ur5_move_command_2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"

# inicio de la captura de la pantalla
profile = pipeline.start(config)

# crear un contador para las imagenes
count = 0
dir_name = "dataset"
class_name = "fist"
max_images = 100

try:
    dir_name = input("Ingrese el nombre de la carpeta: ")
    class_name = input("Ingrese el nombre de la clase: ")
    max_images = int(input("Ingrese el numero de imagenes: "))

    
# crear un while para capturar la imagen
    while max_images > count:
        #movimiento del robot a tres posiciones
        if count == 0:
            print("Moviendo a la posición 0")
            send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_0)
        elif count >= count/3 and count < count/3*2:
            print("Moviendo a la posición 1")
            send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_1)
        elif count >= count/3*2:
            print("Moviendo a la posición 2")
            send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_2)


        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        # leer la imagen
        """ ret, frame = cap.read() """
        # agregar un filtro de escalas de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (480, 480), interpolation=cv2.INTER_AREA)

        # mostrar la imagen
        # cv2.imshow("Captura", frame)
        cv2.imshow("Captura", resized)


        # tomar una foto si se presiona la tecla "c" y guardarla en la carpeta llamada "dataset"
        if cv2.waitKey(1) & 0xFF == ord('c'):
            count += 1
            file_name = f"dataset/{dir_name}/{class_name}_{count}.jpg"
            cv2.imwrite(file_name, resized)
            print(f"Imagen guardada en {file_name} - {count}/{max_images}")

        # si se presiona la tecla "q" se cierra la ventana
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:
    pipeline.stop()
    cv2.destroyAllWindows()
# liberar la camara y cerrar la ventana

""" cap.release() """
cv2.destroyAllWindows()

##################################### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #####################################