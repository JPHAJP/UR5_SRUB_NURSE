####### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #######
import cv2
import pyrealsense2 as rs
import numpy as np
import socket
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
        
        #print("Instruction sent to UR5 robot:", instruction)
        
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
Angles_list_1=[-51.85,-86.16,-136.82,-47.73,90,38]
#Convertir a radianes la lista de angulos
Angles_list_1=[np.radians(i) for i in Angles_list_1]

#ALTO
Angles_list_2=[-51.9,-82.72,-60.07,-127.72,90,38]
#Convertir a radianes la lista de angulos
Angles_list_2=[np.radians(i) for i in Angles_list_2]


ur5_move_command_0 = f"movej([{Angles_list_0[0]}, {Angles_list_0[1]}, {Angles_list_0[2]}, {Angles_list_0[3]}, {Angles_list_0[4]}, {Angles_list_0[5]}], a=1, v=1)\n"
ur5_move_command_1 = f"movej([{Angles_list_1[0]}, {Angles_list_1[1]}, {Angles_list_1[2]}, {Angles_list_1[3]}, {Angles_list_1[4]}, {Angles_list_1[5]}], a=1, v=1)\n"
ur5_move_command_2 = f"movej([{Angles_list_2[0]}, {Angles_list_2[1]}, {Angles_list_2[2]}, {Angles_list_2[3]}, {Angles_list_2[4]}, {Angles_list_2[5]}], a=1, v=1)\n"
send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_1)

# inicio de la captura de la pantalla
profile = pipeline.start(config)

# crear un contador para las imagenes
count = 0
mov = 0
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
        # if count == 0:
        #     print("Moviendo a la posición 0")
        #     print(count)
        #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_0)
        #     sleep(3)
        # elif count == count/3:
        #     print("Moviendo a la posición 1")
        #     print(count)
        #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_1)
        #     sleep(3)
        # elif count == count/3*2:
        #     print("Moviendo a la posición 2")
        #     print(count)
        #     send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_2)
        #     sleep(3)


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

        if cv2.waitKey(1) & 0xFF == ord('p'):
            mov = mov + 1
            print(mov)
            if mov == 1:
                print("Moviendo a la posición 1")
                send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_1)
                sleep(3)
            elif mov == 2:
                print("Moviendo a la posición 2")
                send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_0)
                sleep(3)
            else:
                print("Moviendo a la posición 0")
                send_instruction_to_ur5(ur5_ip, ur5_port, ur5_move_command_2)
                mov = 0
                sleep(3)


finally:
    pipeline.stop()
    cv2.destroyAllWindows()
# liberar la camara y cerrar la ventana

""" cap.release() """
cv2.destroyAllWindows()

##################################### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #####################################