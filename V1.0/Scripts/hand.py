import rtde_control
import rtde_receive
import numpy as np
import mediapipe as mp
import cv2
import pyrealsense2 as rs

control = rtde_control.RTDEControlInterface("192.168.1.1")
receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")

Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]
#Enviar el comando de movimiento al robot
control.moveJ([Angles_list_0[0], Angles_list_0[1], Angles_list_0[2], Angles_list_0[3], Angles_list_0[4], Angles_list_0[5]], 1, 1)

#print("Posición actual: ", receive.getActualTCPPose())
#print("Posición actual: ", receive.getActualQ())

tcp_pose = receive.getActualTCPPose()
joint_positions = receive.getActualQ()

xtool = tcp_pose[0]  # Coordenada x del TCP
ytool = tcp_pose[1]  # Coordenada y del TCP
wrist3 = joint_positions[5]  # Posición del joint 5 (wrist3)

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

#Función que enciende la cámara, detecta manos y envía comandos al robot
def cameracontrol():
    # Inicializar MediaPipe Hands
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)

    # Inicializar RealSense Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            # Capturar cuadros de RealSense
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Convertir imágenes a array de numpy
            color_image = np.asanyarray(color_frame.get_data())
            
            # Convertir la imagen a RGB
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int((height / 2))
            distance = depth_frame.get_distance(center_x, center_y)

            H = distance * np.tan(23) 
            V = distance * np.tan(83.96) 
            V = np.abs(V)

            H = H/1000
            V = V/1000

            # Procesar la imagen y detectar manos
            results = hands.process(image_rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Extraer keypoint 9 (Middle Finger Tip)
                    keypoint_9 = hand_landmarks.landmark[9]
                    x, y = keypoint_9.x * color_image.shape[1], keypoint_9.y * color_image.shape[0]

                    # Calcular la densidad de pixeles por metro (esto necesita ser calibrado)
                    x_meters = mapValue(x, 0, color_image.shape[1], -0.5, 0.5)
                    y_meters = mapValue(y, 0, color_image.shape[0], -0.5, 0.5)
                    
                    # Obtener posición actual del TCP y orientación
                    tcp_pose = receive.getActualTCPPose()
                    joint_positions = receive.getActualQ()
                    
                    # Calcular el origen del frame
                    ofx, ofy, theta = frameOriginCoordinates(tcp_pose[0], tcp_pose[1], H, V, joint_positions[5])
                    
                    # Transformar coordenadas de la mano al espacio del robot
                    global_x, global_y = transformCoordinates(x_meters, y_meters, ofx, ofy, theta)
                    
                    # Generar y enviar el comando de movimiento al robot
                    #control.moveL([global_x, global_y, tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]], 0.5, 0.5)

                    # Dibujar keypoint 9 en la imagen
                    cv2.circle(color_image, (int(x), int(y)), 10, (0, 255, 0), cv2.FILLED)
                    cv2.putText(color_image, "Keypoint 9", (int(x), int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(color_image, f"X: {global_x:.2f} Y: {global_y:.2f}", (int(x), int(y) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    print(receive.getActualTCPPose())

            # Mostrar el resultado
            cv2.imshow('RealSense', color_image)
            
            # Salir del bucle si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Liberar recursos
        pipeline.stop()
        cv2.destroyAllWindows()

cameracontrol()
# Cerrar conexión con el robot al final
control.stopScript()

