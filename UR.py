import socket
import struct
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
from time import sleep

#Función para obtener tanto coordenadas articulares como cartesianas del robot vía Socket
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

#Función para calcular las coordenadas cartesianas del origen del frame respecto a la base del robot
def frameOriginCoordinates(xtool, ytool, H, V, wrist3):
    radio = 39.18
    longitud_tangente = 11.5

    #Ángulo del vector de posición (xf, yf)
    theta = np.arctan2(ytool, xtool)

    #Coordenadas del punto de tangencia
    angulo_tangencia = theta + np.radians(45) - np.radians(wrist3) + np.radians(22)

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

#Función para calcular el centro de las predicciones
def calculateCenter(image, center_x, center_y, red, green, blue):
    cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)

#Función para obtener las coordenadas de la predición
def showAndGetPredictionsLive(model):
    #Configuración del pipeline y el stream de la cámara RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    bbox_center_bluex = 0
    bbox_center_bluey = 0
    bbox_center_greenx = 0
    bbox_center_greeny = 0
    bbox_center_redx = 0
    bbox_center_redy = 0
    bbox_center_yellowx = 0
    bbox_center_yellowy = 0

    try: 
        while True:
            #Espera a que hayan frames disponibles
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

            H = distance * np.tan(23) 
            V = distance * np.tan(83.96) 
            V = np.abs(V)

            H = mTomm(H)
            V = mTomm(V)

            resized_frame = cv2.resize(color_image, (640, 480))
            
            #Se aplica el modelo de detección
            results = model(resized_frame, conf=0.7, classes=[0, 1, 2, 3])
            annotated_frame = results[0].plot()

            #print("Results: ")
            #print(results)

            for coordinates in results:  #Se itera sobre todas las detecciones
                for bbox, cls in zip(coordinates.obb.xyxy, coordinates.obb.cls):  #Se itera sobre todas las cajas y sus clases
                    if cls == 0:
                        bbox_coordinates_blue = bbox.tolist()
                        bbox_center_blue = ((bbox_coordinates_blue[0] + bbox_coordinates_blue[2]) // 2, (bbox_coordinates_blue[1] + bbox_coordinates_blue[3]) // 2)
                        bbox_center_blue = (int(bbox_center_blue[0]), int(bbox_center_blue[1]))
                        color1 = (56, 56, 255)  
                        cv2.circle(annotated_frame, bbox_center_blue, 3, color1, -1)
                        bbox_center_bluex = bbox_center_blue[0]
                        bbox_center_bluey = bbox_center_blue[1]
                    elif cls == 2:
                        bbox_coordinates_green = bbox.tolist()
                        bbox_center_green = ((bbox_coordinates_green[0] + bbox_coordinates_green[2]) // 2, (bbox_coordinates_green[1] + bbox_coordinates_green[3]) // 2)
                        bbox_center_green = (int(bbox_center_green[0]), int(bbox_center_green[1]))
                        color2 = (31, 112, 255)  
                        cv2.circle(annotated_frame, bbox_center_green, 3, color2, -1)    
                        bbox_center_greenx = bbox_center_green[0]
                        bbox_center_greeny = bbox_center_green[1]
                    elif cls == 3:
                        bbox_coordinates_red = bbox.tolist()
                        bbox_center_red = ((bbox_coordinates_red[0] + bbox_coordinates_red[2]) // 2, (bbox_coordinates_red[1] + bbox_coordinates_red[3]) // 2)
                        bbox_center_red = (int(bbox_center_red[0]), int(bbox_center_red[1]))
                        color3 = (29, 178, 255)  
                        cv2.circle(annotated_frame, bbox_center_red, 3, color3, -1) 
                        bbox_center_redx = bbox_center_red[0]
                        bbox_center_redy = bbox_center_red[1]
                    elif cls == 4:
                        bbox_coordinates_yellow = bbox.tolist()
                        bbox_center_yellow = ((bbox_coordinates_yellow[0] + bbox_coordinates_yellow[2]) // 2, (bbox_coordinates_yellow[1] + bbox_coordinates_yellow[3]) // 2)
                        bbox_center_yellow = (int(bbox_center_yellow[0]), int(bbox_center_yellow[1]))
                        color4 = (255, 255, 255)  
                        cv2.circle(annotated_frame, bbox_center_yellow, 3, color4, -1)
                        bbox_center_yellowx = bbox_center_yellow[0]
                        bbox_center_yellowy = bbox_center_yellow[1]

            #Se muestra la imágen con anotaciones
            cv2.imshow('Resultado', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:

        pipeline.stop()
        cv2.destroyAllWindows()

    return H, V, bbox_center_bluex, bbox_center_bluey, bbox_center_greenx, bbox_center_greeny, bbox_center_redx, bbox_center_redy, bbox_center_yellowx, bbox_center_yellowy

#Función para encontrar la densidad de pixeles por metro
def mapValue(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

#Función para transformar coordenadas locales de la fotografía en coordenadas globales (respecto la base del robot)
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

#Función para pedirle al UR5 hacer un movimiento lineal 
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

#Función para convertir metros a milímetros
def mTomm(toconvert):
    return toconvert * 1000

#Función para convertir de milímetros a metros
def mmTom(toconvert):
    return toconvert / 1000

#Función para definir el origen y el destino de una trayectoria
def defineOriginAndDestination(xtransfn, ytransfn, ofzn):

    destinationf = [xtransfn, ytransfn, ofzn]
    currentPosexf, currentPoseyf, currentPosezf, _, _, _, _ = getPose()
    currentPosef = [currentPosexf, currentPoseyf, currentPosezf]
    #Comando para mover el robot a la posición deseada
    #CHAVA PENDEJO, CAMBIA ESTO
    #ur5_move_command = f"movel(p[{xtransfn},{ytransfn},{ofzn},{rxr},{ryr},{rzr}], a = 1.2, v = 0.25, t = 0, r = 0)\n"
    control.moveJ([Angles_list_0[0], Angles_list_0[1], Angles_list_0[2], Angles_list_0[3], Angles_list_0[4], Angles_list_0[5]], 1, 1)
    
    sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command)
        
    #Normalizamos para poder hacer comparativas
    destinationf = np.around(destinationf, decimals=2)    

    return currentPosef, destinationf

#Función para manejar las solicitudes de cambio de pose para el robot
def handleRobotPoseMoveRequest(destination, currentPose):

    while destination[0] != currentPose[0] and destination[1] != currentPose[1] and destination[2] != currentPose[2]:
        currentPosex, currentPosey, currentPosez, _, _, _, _ = getPose()
        currentPosex = np.around(currentPosex, decimals=2)
        currentPosey = np.around(currentPosey, decimals=2)
        currentPosez = np.around(currentPosez, decimals=2)
        currentPose = [currentPosex, currentPosey, currentPosez]

        print("Current pose: ")
        print(currentPose)
        print("Destination: ")
        print(destination)

#Coordenadas articulares para el home
T1 = np.radians(-51.92)
T2 = np.radians(-71.84)
T3 = np.radians(-112.63)
T4 = np.radians(-85.53)
T5 = np.radians(90.02)
T6 = np.radians(38.05)

model = YOLO('V1.0/Models/best.pt')
ur5_ip = "192.168.1.1" 
ur5_port = 30002 
ur5_move_command_1 = f"movej([{T1}, {T2}, {T3}, {T4}, {T5}, {T6}], a=1, v=1.05)\n"
sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
sleep(10)

while True: 

    #Parámetros para obtener las coordenadas cartesianas del origen del frame
    xr, yr, zr, rxr, ryr, rzr, wrist3r = getPose()
    xr = mTomm(xr)
    yr = mTomm(yr)
    Hr, Vr, bluexr, blueyr, greenxr, greenyr, redxr, redyr, yellowxr, yellowyr = showAndGetPredictionsLive(model)
    angle = np.rad2deg(wrist3r)
    ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, Hr, Vr, angle)

    #Transformación de coordenadas locales del frame a coordenadas globales (base del robot)
    xfinal1 = mapValue(bluexr, 0, 640, 0, Hr)
    yfinal1 = mapValue(blueyr, 0, 480, 0, Vr)
    xfinal2 = mapValue(greenxr, 0, 640, 0, Hr)
    yfinal2 = mapValue(greenyr, 0, 480, 0, Vr)
    xfinal3 = mapValue(redxr, 0, 640, 0, Hr)
    yfinal3 = mapValue(redyr, 0, 480, 0, Vr)
    xfinal4 = mapValue(yellowxr, 0, 640, 0, Hr)
    yfinal4 = mapValue(yellowyr, 0, 480, 0, Vr)
    xtransf1, ytransf1 = transformCoordinates(xfinal1, yfinal1, ofxr, ofyr, thetar)
    xtransf2, ytransf2 = transformCoordinates(xfinal2, yfinal2, ofxr, ofyr, thetar)
    xtransf3, ytransf3 = transformCoordinates(xfinal3, yfinal3, ofxr, ofyr, thetar)
    xtransf4, ytransf4 = transformCoordinates(xfinal4, yfinal4, ofxr, ofyr, thetar)

    #Conversión de milímetros a metros
    ofxr = mmTom(ofxr)
    ofyr = mmTom(ofyr)
    xtransf1 = mmTom(xtransf1)
    ytransf1 = mmTom(ytransf1)
    xtransf2 = mmTom(xtransf2)
    ytransf2 = mmTom(ytransf2)
    xtransf3 = mmTom(xtransf3)
    ytransf3 = mmTom(ytransf3)
    xtransf4 = mmTom(xtransf4)
    ytransf4 = mmTom(ytransf4)
    ofzr = 0.03

    print("1.- Bisturí")
    print("2.- Pinzas")
    print("3.- Tijeras curvas")
    print("4.- Tijeras rectas")
    resp = int(input("Seleccione el instrumento deseado: "))

    if resp == 1:
        currentPose, destination = defineOriginAndDestination(xtransf1, ytransf1, ofzr)
        print(f"Current Pose: {currentPose}")

    if resp == 2:
        currentPose, destination = defineOriginAndDestination(xtransf2, ytransf2, ofzr)
        print(f"Current Pose: {currentPose}")

    if resp == 3:
        currentPose, destination = defineOriginAndDestination(xtransf3, ytransf3, ofzr)
    
    if resp == 4:
        currentPose, destination = defineOriginAndDestination(xtransf4, ytransf4, ofzr)

    resp2 = int(input("Presione 1 para volver al home: "))

    handleRobotPoseMoveRequest(destination, currentPose)

    if resp2 == 1:
        ur5_move_command_1 = "movel(p[.117,-.364,.375,0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
        destination = [0.12,-0.36,0.38]
        currentPosexr, currentPoseyr, currentPosezr, _, _, _, _ = getPose()
        currentPosefr = [currentPosexr, currentPoseyr, currentPosezr]
        sendInstructionToUr5(ur5_ip, ur5_port, ur5_move_command_1)
        handleRobotPoseMoveRequest(destination, currentPosefr)