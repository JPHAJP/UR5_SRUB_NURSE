import socket
import struct
import pyrealsense2 as rs
import cv2
import numpy as np
import mediapipe as mp
from ultralytics import YOLO
from time import sleep


def mapValue(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def getPose():
    #Conexión con el UR5
    HOST = '192.168.1.1'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))

    cdr = 0
    while cdr < 3:
        #Recibir 4096 bytes de datos (suficiente para almacenar cualquier paquete)
        data = s.recv(4096)
        #Inicializar i para trackear la posición en el paquete
        i = 0
        if data:
            #Información del paquete recibido
            packlen = (struct.unpack('!i', data[0:4]))[0]
            timestamp = (struct.unpack('!Q', data[10:18]))[0]
            packtype = (struct.unpack('!b', data[4:5]))[0]

            #Si el tipo de paquete es el estado del robot, loopear hasta alcanzar el final del paquete
            if packtype == 16:
                while i + 5 < packlen:
                    #Tamaño y tipo de mensaje
                    msglen = (struct.unpack('!i', data[5+i:9+i]))[0]
                    msgtype = (struct.unpack('!b', data[9+i:10+i]))[0]

                    if msgtype == 1:
                        #Si el mensaje es coordenada articular, crear una lista para guardar los ángulos
                        angle = [0] * 6
                        j = 0
                        while j < 6:
                            #Bytes del 10 al 18 tienen ángulo j0, cada coordenada articular es de 41 bytes (saltamos j*41 cada vez)
                            angle[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
                            j += 1

                    #Si el tipo de mensaje son coordenadas cartesianas, almacenamos las coordenadas actuales del TCP
                    elif msgtype == 4:
                        x = (struct.unpack('!d', data[10+i:18+i]))[0]
                        y = (struct.unpack('!d', data[18+i:26+i]))[0]
                        z = (struct.unpack('!d', data[26+i:34+i]))[0]
                        rx = (struct.unpack('!d', data[34+i:42+i]))[0]
                        ry = (struct.unpack('!d', data[42+i:50+i]))[0]
                        rz = (struct.unpack('!d', data[50+i:58+i]))[0]

                    #Incrementamos i por el tamaño del mensaje para movernos al siguiente mensaje en el paquete
                    i += msglen
        cdr = cdr + 1

    return x, y, z, rx, ry, rz, angle[5]

def sendInstructionToUr5(ip, port, instruction):
    try:
        #Objeto socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        #Conectarse al robot
        sock.connect((ip, port))
        
        #Manda la instrucción al robot
        sock.sendall(instruction.encode())
        
        print("Instrucción mandada al UR5: ", instruction)
        
        #Cierra la conexión
        sock.close()
        
    except ConnectionRefusedError:
        print("Conexión rechazada")
    
    except TimeoutError:
        print("Tiempo agotado, no se puede conectar")

def startCamera():
    #Inicializar la cámara RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            #Esperar a que haya datos disponibles
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            #Convertir los frames a matrices numpy
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            #Dibujar un punto en el centro del frame de color
            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int(height / 2)
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

            #Obtener la distancia al punto central
            distance = depth_frame.get_distance(center_x, center_y)

            #Calcular distancias horizontales y verticales basadas en la distancia
            H = distance*np.tan(23) #23
            V = distance*np.tan(83.96) 
            V = np.abs(V)

            #Mostrar la imagen con el punto dibujado y la distancia
            cv2.putText(color_image, f"Distance: {distance:.2f} m, H: {H:.2f} m, V: {V:.2f} m", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('RealSense Camera', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

def frameOriginCoordinates(xtool, ytool, H, V, wrist3):
    radio = 39.18
    longitud_tangente = 11.5

    #Ángulo del vector de posición (xf, yf)
    theta = np.arctan2(ytool, xtool)

    #Coordenadas del punto de tangencia
    angulo_tangencia = theta + np.radians(45) - np.radians(wrist3) + np.radians(28)

    #Dirección de la tangente
    angulo_tangente = angulo_tangencia - np.pi / 2

    #Componentes rectangulares del lente principal de la cámara
    Ctcp = np.sqrt(radio**2 + longitud_tangente**2)
    theta1 = np.arccos(radio / Ctcp)
    theta2 = angulo_tangencia - theta1
    cfx = Ctcp * np.cos(theta2) + xtool
    cfy = Ctcp * np.sin(theta2) + ytool

    #Vector de posición del origen del frame
    ofx = cfx - (H/2) * np.cos(angulo_tangente) - (V/2) * np.sin(angulo_tangente)
    ofy = cfy - (H/2) * np.sin(angulo_tangente) + (V/2) * np.cos(angulo_tangente)

    return ofx, ofy, angulo_tangencia

def transformCoordinates(x1, y1, ofx, ofy, theta):
    r = np.array([
                    [0, -1, 0],
                    [-1, 0, 0],
                    [0, 0, 1]
    ])

    R_z = np.array([
                    [np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]
    ])

    R0_f = np.dot(R_z, r)

    P0_f = np.array([
                        [ofx],
                        [ofy],
                        [0]
    ])

    #Poderosas matrices de transformación homogenea
    H0_f = np.concatenate((R0_f, P0_f), 1)
    H0_f = np.concatenate((H0_f, [[0,0,0,1]]), 0)

    punto = np.array([
        [x1],
        [y1],
        [0],
        [1]
    ])

    coordTransf = np.dot(H0_f, punto)
    
    return coordTransf[0][0], coordTransf[1][0]

#Función para obtener las coordenadas de la predición
def showAndGetPredictionsLive(model):
    #Configurar el pipeline y el stream de la cámara RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    scalpelx = 0
    scalpely = 0
    pliersx = 0
    pliersy = 0
    cscissorsx = 0
    cscissorsy = 0
    sscissorsx = 0
    sscissorsy = 0

    try: 
        while True:
            #Esperar a que haya frames disponibles
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int((height / 2))
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

            distance = depth_frame.get_distance(center_x, center_y)

            H = distance*np.tan(23) #23
            V = distance*np.tan(83.96) 
            V = np.abs(V)

            H = H * 1000
            V = V * 1000

            resized_frame = cv2.resize(color_image, (640, 480))
            
            #Aplicar el modelo de detección
            results = model(resized_frame, conf=0.7, classes=[0, 2, 3, 4])
            annotated_frame = results[0].plot()

            #Procesar los resultados del modelo como en tu función original
            boxes = results[0].boxes
            for box in boxes:
                if 0 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    scalpelx = int((coordinates[0] + coordinates[2]) / 2)
                    scalpely = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Bisturí x pixels: {scalpelx}")
                    print(f"Bisturí y pixels: {scalpely}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)
                if 2 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    pliersx = int((coordinates[0] + coordinates[2]) / 2)
                    pliersy = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Pinzas x pixels: {scalpelx}")
                    print(f"Pinzas y pixels: {scalpely}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)
                if 3 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    cscissorsx = int((coordinates[0] + coordinates[2]) / 2)
                    cscissorsy = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Tijeras curvas x pixels: {scalpelx}")
                    print(f"Tijeras curvas y pixels: {scalpely}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)
                if 4 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    sscissorsx = int((coordinates[0] + coordinates[2]) / 2)
                    sscissorsy = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Tijeras rectas x pixels: {scalpelx}")
                    print(f"Tijeras rectas y pixels: {scalpely}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)

            #Mostrar la imagen anotada
            cv2.imshow('Resultado', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Presiona 'q' para salir
                break
    finally:
        #Detener la captura y cerrar todas las ventanas
        pipeline.stop()
        cv2.destroyAllWindows()
    
    return H, V, scalpelx, scalpely, pliersx, pliersy, cscissorsx, cscissorsy, sscissorsx, sscissorsy

def showAndGetPredictionsLiveHand():
    # Configurar el pipeline y el stream de la cámara RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            # Esperar a que haya frames disponibles
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int((height / 2))
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

            distance = depth_frame.get_distance(center_x, center_y)

            H = distance*np.tan(23) #23
            V = distance*np.tan(83.96) 
            V = np.abs(V)

            H = H * 1000
            V = V * 1000

             # Inicializar el detector de manos de MediaPipe
            mp_hands = mp.solutions.hands
            hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)

            # Convertir la imagen a RGB para mediapipe
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            # Detectar manos en la imagen
            results = hands.process(image_rgb)

            # Procesar los resultados de Mediapipe
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Obtener las coordenadas del keypoint 5 (la muñeca)
                    keypoint_5_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * color_frame.width
                    keypoint_5_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * color_frame.height
                    print(f"Hand x pixels: {keypoint_5_x}")
                    print(f"Hand y pixels: {keypoint_5_y}")
                    
                    # Calcula el centro de la muñeca y muestra en la imagen
                    calculateCenter(color_image, int(keypoint_5_x), int(keypoint_5_y), 0, 255, 0)

            # Mostrar la imagen anotada
            cv2.imshow('Resultado', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Presiona 'q' para salir
                break
    finally:
        # Detener la captura y cerrar todas las ventanas
        pipeline.stop()
        cv2.destroyAllWindows()

    return H, V, keypoint_5_x, keypoint_5_y

def calculateCenter(image, center_x, center_y, red, green, blue):
    cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)

model = YOLO('Med_IA.pt')

while True:
    #IP y puerto del robot
    ur5_ip = "192.168.1.1" 
    ur5_port = 30002 

    #Parámetros para obtener las coordenadas cartesianas del origen del frame
    xr, yr, zr, rxr, ryr, rzr, wrist3r = getPose()
    xr = xr * 1000
    yr = yr * 1000
    Hr, Vr, scalpelxr, scalpelyr, pliersxr, pliersyr, cscissorsxr, cscissorsyr, sscissorsxr, sscissorsyr = showAndGetPredictionsLive(model)
    angle = np.rad2deg(wrist3r)
    ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, Hr, Vr, angle)

    #Transformación de coordenadas locales del frame a coordenadas globales (base del robot) Intrumental

    xfinal1 = mapValue(scalpelxr, 0, 640, 0, Hr)
    yfinal1 = mapValue(scalpelyr, 0, 480, 0, Vr)
    xfinal2 = mapValue(pliersxr, 0, 640, 0, Hr)
    yfinal2 = mapValue(pliersyr, 0, 480, 0, Vr)
    xfinal3 = mapValue(cscissorsxr, 0, 640, 0, Hr)
    yfinal3 = mapValue(cscissorsyr, 0, 480, 0, Vr)
    xfinal4 = mapValue(sscissorsxr, 0, 640, 0, Hr)
    yfinal4 = mapValue(sscissorsyr, 0, 480, 0, Vr)
    xtransf1, ytransf1 = transformCoordinates(xfinal1, yfinal1, ofxr, ofyr, thetar)
    xtransf2, ytransf2 = transformCoordinates(xfinal2, yfinal2, ofxr, ofyr, thetar)
    xtransf3, ytransf3 = transformCoordinates(xfinal3, yfinal3, ofxr, ofyr, thetar)
    xtransf4, ytransf4 = transformCoordinates(xfinal4, yfinal4, ofxr, ofyr, thetar)

    #Conversión de unidades
    ofxr = ofxr / 1000
    ofyr = ofyr / 1000

    xtransf1 = xtransf1 / 1000
    ytransf1 = ytransf1 / 1000
    xtransf2 = xtransf2 / 1000
    ytransf2 = ytransf2 / 1000
    xtransf3 = xtransf3 / 1000
    ytransf3 = ytransf3 / 1000
    xtransf4 = xtransf4 / 1000
    ytransf4 = ytransf4 / 1000
    ofzr = 0.015
    ofzr1 = 0.10
    #Ir al instrumental

    print("1.- Bisturí")
    print("2.- Pinzas")
    print("3.- Tijeras curvas")
    print("4.- Tijeras rectas")
    resp = int(input("Seleccione el instrumento deseado: "))


    if resp == 1:
        #Se mandan las coordenadas obtenidas al robot
        ur5_move_command = f"movel(p[{xtransf1},{ytransf1},{ofzr},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)

    if resp == 2:
        #Se mandan las coordenadas obtenidas al robot
        ur5_move_command = f"movel(p[{xtransf2},{ytransf2},{ofzr},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)
    
    if resp == 3:
        #Se mandan las coordenadas obtenidas al robot
        ur5_move_command = f"movel(p[{xtransf3},{ytransf3},{ofzr},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)
    
    if resp == 4:
        #Se mandan las coordenadas obtenidas al robot
        ur5_move_command = f"movel(p[{xtransf4},{ytransf4},{ofzr},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)

    #Regresar a casa
    """
    resp2 = int(input("Presione 1 para mandar al robot a su posición original: "))
    if resp2 == 1:
        ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
    sleep(3)
    """

    sleep(2)
    ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
    sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
    sleep(3)
    #Parámetros para obtener las coordenadas cartesianas del origen del frame
    xr, yr, zr, rxr, ryr, rzr, wrist3r = getPose()
    xr = xr * 1000
    yr = yr * 1000
    Hr, Vr, middle_point_xr, middle_point_yr= showAndGetPredictionsLiveHand()
    angle = np.rad2deg(wrist3r)
    ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, Hr, Vr, angle)

    #Transformación de coordenadas locales del frame a coordenadas globales (base del robot)
    xfinal1 = mapValue(middle_point_xr, 0, 640, 0, Hr)
    yfinal1 = mapValue(middle_point_yr, 0, 480, 0, Vr)
    print(f"Tamaño del frame horizontal: {Hr}")
    print(f"Tamaño del frame vertical: {Vr}")
    print(f"xfinal: {xfinal1}")
    print(f"yfinal: {yfinal1}")
    xtransh1, ytransh1 = transformCoordinates(xfinal1, yfinal1, ofxr, ofyr, thetar)

    #Conversión de unidades
    ofxr = ofxr / 1000
    ofyr = ofyr / 1000
    xtransh1 = xtransh1 / 1000
    ytransh1 = ytransh1 / 1000

    print(f"xtransh1 = {xtransh1}")
    print(f"ytransh1 = {ytransh1}")

    #Ir a la mano

    resp3 = int(input("Presione 1 para ir a la mano: "))
    if resp3 == 1:
        ur5_move_command = f"movel(p[{xtransh1},{ytransh1},{ofzr1},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)

    #Regresar a casa
    """
    sleep(3)
    resp4 = int(input("Presione 1 para mandar al robot a su posición original: "))
    if resp4 == 1:
        ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
    sleep(3)
    """

    sleep(3)
    ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
    sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
    sleep(3)