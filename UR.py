import pyrealsense2 as rs
import cv2

import numpy as np

from ultralytics import YOLO

import rtde_control
import rtde_receive
import rtde_io

import threading
import time

control = rtde_control.RTDEControlInterface("192.168.1.1")
receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")
io = rtde_io.RTDEIOInterface("192.168.1.1")

model = YOLO('V1.0/Models/V4_best.pt')

# Función para calcular las coordenadas cartesianas del origen del frame respecto a la base del robot
def frameOriginCoordinates(xtool, ytool, H, V, wrist3):
    radio = 39.18
    longitud_tangente = 11.5

    # Ángulo del vector de posición (xf, yf)
    theta = np.arctan2(ytool, xtool)

    # Coordenadas del punto de tangencia
    angulo_tangencia = theta + np.radians(45) - np.radians(wrist3) + np.radians(22)

    # Dirección de la tangente
    angulo_tangente = angulo_tangencia - np.pi / 2

    # Componentes rectangulares del lente principal de la cámara
    Ctcp = np.sqrt(radio**2 + longitud_tangente**2)
    theta1 = np.arccos(radio / Ctcp)
    theta2 = angulo_tangencia - theta1
    cfx = Ctcp * np.cos(theta2) + xtool
    cfy = Ctcp * np.sin(theta2) + ytool

    # Vector de posición del origen del frame
    ofx = cfx - (H/2) * np.cos(angulo_tangente) - (V/2) * np.sin(angulo_tangente)
    ofy = cfy - (H/2) * np.sin(angulo_tangente) + (V/2) * np.cos(angulo_tangente)

    return ofx, ofy, angulo_tangencia

# Función para calcular el centro de las predicciones
def calculateCenter(image, center_x, center_y, red, green, blue):
    cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)

# Función para obtener las coordenadas de la predición
def showAndGetPredictionsLive(model):
    # Configuración del pipeline y el stream de la cámara RealSense
    print("Iniciando pipeline...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("Pipeline iniciado.")

    bbox_center_bluex = 0
    bbox_center_bluey = 0
    bbox_center_greenx = 0
    bbox_center_greeny = 0
    bbox_center_redx = 0
    bbox_center_redy = 0
    bbox_center_yellowx = 0
    bbox_center_yellowy = 0
    bbox_center_pinkx = 0
    bbox_center_pinky = 0

    try: 
        while True:
            # Espera a que hayan frames disponibles
            print("Esperando frames...")
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

            H = H*1000
            V = V*1000

            resized_frame = cv2.resize(color_image, (640, 480))
            
            # Se aplica el modelo de detección
            results = model(resized_frame, conf=0.7, classes=[0, 1, 2, 3])
            annotated_frame = results[0].plot()

            for coordinates in results:  # Se itera sobre todas las detecciones
                for bbox, cls in zip(coordinates.obb.xyxy, coordinates.obb.cls):  # Se itera sobre todas las cajas y sus clases
                    if cls == 0:
                        bbox_coordinates_blue = bbox.tolist() #ti
                        bbox_center_blue = ((bbox_coordinates_blue[0] + bbox_coordinates_blue[2]) // 2, (bbox_coordinates_blue[1] + bbox_coordinates_blue[3]) // 2)
                        bbox_center_blue = (int(bbox_center_blue[0]), int(bbox_center_blue[1]))
                        color1 = (56, 56, 255)
                        cv2.circle(annotated_frame, bbox_center_blue, 3, color1, -1)
                        bbox_center_bluex = bbox_center_blue[0]
                        bbox_center_bluey = bbox_center_blue[1]
                    elif cls == 5:
                        bbox_coordinates_green = bbox.tolist() #ti
                        bbox_center_green = ((bbox_coordinates_green[0] + bbox_coordinates_green[2]) // 2, (bbox_coordinates_green[1] + bbox_coordinates_green[3]) // 2)
                        bbox_center_green = (int(bbox_center_green[0]), int(bbox_center_green[1]))
                        color2 = (31, 112, 255)  
                        cv2.circle(annotated_frame, bbox_center_green, 3, color2, -1)    
                        bbox_center_greenx = bbox_center_green[0]
                        bbox_center_greeny = bbox_center_green[1]
                    elif cls == 3:
                        bbox_coordinates_red = bbox.tolist() #bisturi
                        bbox_center_red = ((bbox_coordinates_red[0] + bbox_coordinates_red[2]) // 2, (bbox_coordinates_red[1] + bbox_coordinates_red[3]) // 2)
                        bbox_center_red = (int(bbox_center_red[0]), int(bbox_center_red[1]))
                        color3 = (29, 178, 255)  
                        cv2.circle(annotated_frame, bbox_center_red, 3, color3, -1) 
                        bbox_center_redx = bbox_center_red[0]
                        bbox_center_redy = bbox_center_red[1]
                    elif cls == 4:
                        bbox_coordinates_yellow = bbox.tolist() #pinzas
                        bbox_center_yellow = ((bbox_coordinates_yellow[0] + bbox_coordinates_yellow[2]) // 2, (bbox_coordinates_yellow[1] + bbox_coordinates_yellow[3]) // 2)
                        bbox_center_yellow = (int(bbox_center_yellow[0]), int(bbox_center_yellow[1]))
                        color4 = (255, 255, 255)  
                        cv2.circle(annotated_frame, bbox_center_yellow, 3, color4, -1)
                        bbox_center_yellowx = bbox_center_yellow[0]
                        bbox_center_yellowy = bbox_center_yellow[1]
                    elif cls == 1:
                        bbox_coordinates_pink = bbox.tolist() #manita
                        bbox_center_pink = ((bbox_coordinates_pink[0] + bbox_coordinates_pink[2]) // 2, (bbox_coordinates_pink[1] + bbox_coordinates_pink[3]) // 2)
                        bbox_center_pink = (int(bbox_center_pink[0]), int(bbox_center_pink[1]))
                        color5 = (255, 192, 203)  
                        cv2.circle(annotated_frame, bbox_center_pink, 3, color5, -1)
                        bbox_center_pinkx = bbox_center_pink[0]
                        bbox_center_pinky = bbox_center_pink[1]

            # Se muestra la imagen con anotaciones
            cv2.imshow('Resultado', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if cv2.waitKey(1) & 0xFF == ord('r'):
                io.setStandardDigitalOut(0, False)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return H, V, bbox_center_bluex, bbox_center_bluey, bbox_center_greenx, bbox_center_greeny, bbox_center_redx, bbox_center_redy, bbox_center_yellowx, bbox_center_yellowy, bbox_center_pinkx, bbox_center_pinky

# Función para encontrar la densidad de pixeles por metro
def mapValue(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

# Función para transformar coordenadas locales de la fotografía en coordenadas globales (respecto la base del robot)
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

    # Matrices de transformación homogenea
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

# Función para definir el origen y el destino de una trayectoria
def defineOriginAndDestination(xtransfn, ytransfn, ofzn):
    destinationf = [xtransfn, ytransfn, ofzn]
    #currentPosexf, currentPoseyf, currentPosezf, _, _, _ = receive.getActualTCPPose()
    # Comando para mover el robot a la posición deseada
    control.moveL([xtransfn, ytransfn, ofzn, rxr, ryr, rzr], .5, .5, asynchronous=True)
    # Normalizamos para poder hacer comparativas
    destinationf = np.around(destinationf, decimals=2)

# Función para mover el robot a la posición "Home"
def gohome():
    # Coordenadas articulares para el home
    home_joint_angles_deg = [-51.9, -71.85, -112.7, -85.96, 90, 38]
    # Convertir la lista de ángulos a radianes
    home_joint_angles_rad = np.radians(home_joint_angles_deg)
    # Mover el robot a la posición "Home" usando control.moveJ
    # Velocidad = 1 rad/s, Aceleración = 1 rad/s^2
    control.moveJ(home_joint_angles_rad, 1, 1, asynchronous=True)

# Función para monitorear el estado del sensor en un hilo separado
def monitor_io_and_interrupt():
    while True:
        sensor_state = receive.getDigitalInState(0)  # Leer el estado del pin 0

        if sensor_state:  # Si el sensor se activa
            print("Sensor activado, deteniendo el robot.")
            control.stopL(1.0)  # Detener el movimiento del robot si se está moviendo por trayectorias
            control.stopJ(1.0)  # Detener el movimiento del robot si se está moviendo por articulaciones
            gohome()

        time.sleep(0.01)  # Esperar un poco antes de verificar de nuevo


#--------------------Inicio del programa--------------------#
gohome()

# Crear y comenzar el hilo para monitorear el sensor
print("Iniciando hilo del sensor...")
sensor_thread = threading.Thread(target=monitor_io_and_interrupt)
sensor_thread.start()

print("Iniciando el programa principal...")
try:
    while True: 
        Hr, Vr, bluexr, blueyr, greenxr, greenyr, redxr, redyr, yellowxr, yellowyr, pinkxr, pinkyr = showAndGetPredictionsLive(model)
        # Parámetros para obtener las coordenadas cartesianas del origen del frame
        xr, yr, zr, rxr, ryr, rzr = receive.getActualTCPPose()
        # Obtén las posiciones actuales de las articulaciones (en radianes)
        joint_positions = receive.getActualQ()
        # El ángulo del joint 5 es el quinto elemento de la lista
        wrist3r = joint_positions[5]
        xr = xr*1000
        yr = yr*1000
        print(f'Posición actual del robot:{receive.getActualTCPPose()}')
        angle = np.rad2deg(wrist3r)
        ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, Hr, Vr, angle)

        # Transformación de coordenadas locales del frame a coordenadas globales (base del robot)
        xfinal1 = mapValue(bluexr, 0, 640, 0, Hr)
        yfinal1 = mapValue(blueyr, 0, 480, 0, Vr)
        xfinal2 = mapValue(greenxr, 0, 640, 0, Hr)
        yfinal2 = mapValue(greenyr, 0, 480, 0, Vr)
        xfinal3 = mapValue(redxr, 0, 640, 0, Hr)
        yfinal3 = mapValue(redyr, 0, 480, 0, Vr)
        xfinal4 = mapValue(yellowxr, 0, 640, 0, Hr)
        yfinal4 = mapValue(yellowyr, 0, 480, 0, Vr)
        xfinal5 = mapValue(pinkxr, 0, 640, 0, Hr)
        yfinal5 = mapValue(pinkyr, 0, 480, 0, Vr)

        xtransf1, ytransf1 = transformCoordinates(xfinal1, yfinal1, ofxr, ofyr, thetar)
        xtransf2, ytransf2 = transformCoordinates(xfinal2, yfinal2, ofxr, ofyr, thetar)
        xtransf3, ytransf3 = transformCoordinates(xfinal3, yfinal3, ofxr, ofyr, thetar)
        xtransf4, ytransf4 = transformCoordinates(xfinal4, yfinal4, ofxr, ofyr, thetar)
        xtransf5, ytransf5 = transformCoordinates(xfinal5, yfinal5, ofxr, ofyr, thetar)

        # Conversión de milímetros a metros
        ofxr = ofxr/1000
        ofyr = ofyr/1000
        xtransf1 = xtransf1/1000
        ytransf1 = ytransf1/1000
        xtransf2 = xtransf2/1000
        ytransf2 = ytransf2/1000
        xtransf3 = xtransf3/1000
        ytransf3 = ytransf3/1000
        xtransf4 = xtransf4/1000
        ytransf4 = ytransf4/1000
        xtransf5 = xtransf5/1000
        ytransf5 = ytransf5/1000
        ofzr = 0.02

        print("1.- Bisturí")
        print("2.- Pinzas")
        print("3.- Tijeras curvas")
        print("4.- Tijeras rectas")
        print("5.- Mano")
        resp = int(input("Seleccione el instrumento deseado: "))
        #Encender electroiman
        io.setStandardDigitalOut(0, True)

        if resp == 1:
            defineOriginAndDestination(xtransf1, ytransf1, ofzr)
        
        if resp == 2:
            defineOriginAndDestination(xtransf2, ytransf2, ofzr)

        if resp == 3:
            defineOriginAndDestination(xtransf3, ytransf3, ofzr)
        
        if resp == 4:
            defineOriginAndDestination(xtransf4, ytransf4, ofzr)

        if resp == 5:
            defineOriginAndDestination(xtransf5, ytransf5, ofzr)

        resp2 = int(input("Presione 1 para volver al home: "))

        if resp2 == 1:
            gohome()

        # resp3 = int(input("Presione 1 para ir a la mano: "))
        # if resp3 == 1:
        #     defineOriginAndDestination(xtransf5, ytransf5, ofzr)
        #     gohome()

finally:
    sensor_thread.join()
    print("Thread finalizado.")
    io.setStandardDigitalOut(0, False)
    print("Electroiman apagado.")
    