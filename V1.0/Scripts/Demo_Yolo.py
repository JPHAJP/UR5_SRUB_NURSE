import socket
import cv2
import numpy as np
import socket
import pyrealsense2 as rs
from ultralytics import YOLO
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

# configurar UR5
ur5_ip = "192.168.1.1" 
ur5_port = 30002 

# configurar el pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# inicio de la captura de la pantalla
profile = pipeline.start(config)

try: 
# crear un while para capturar la imagen
    while True:
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
        # si se presiona la tecla "q" se cierra la ventana
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
# liberar la camara y cerrar la ventana

""" cap.release() """
cv2.destroyAllWindows()

saludar_continuo(ur5_ip, ur5_port)