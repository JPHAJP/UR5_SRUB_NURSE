import pyrealsense2 as rs
import rtde_control
import rtde_receive
import mediapipe as mp
import numpy as np
import cv2

control = rtde_control.RTDEControlInterface("192.168.1.1")
receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")

Angles_list_0=[-51.9,-71.85,-112.7,-85.96,90,38]
#Convertir a radianes la lista de angulos
Angles_list_0=[np.radians(i) for i in Angles_list_0]
#Enviar el comando de movimiento al robot
control.moveJ([Angles_list_0[0], Angles_list_0[1], Angles_list_0[2], Angles_list_0[3], Angles_list_0[4], Angles_list_0[5]], 1, 1)

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

def calculateCenter(image, center_x, center_y, red, green, blue):
    cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)

def showAndGetPredictionsLive():
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
                    handx = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * color_frame.width
                    handy = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * color_frame.height
                    #print(f"Hand x pixels: {handx}")
                    #print(f"Hand y pixels: {handy}")
                    
                    # Calcula el centro de la muñeca y muestra en la imagen
                    calculateCenter(color_image, int(handx), int(handy), 0, 255, 0)

            # Mostrar la imagen anotada
            cv2.imshow('Resultado', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Presiona 'q' para salir
                break
    finally:
        # Detener la captura y cerrar todas las ventanas
        pipeline.stop()
        cv2.destroyAllWindows()

    return H, V, handx, handy

while True:
    Hr , Vr , handx, handy = showAndGetPredictionsLive()

    xr, yr, zr, rxr, ryr, rzr = receive.getActualTCPPose()

    tcp_pose = receive.getActualTCPPose()
    joint_positions = receive.getActualQ()

    xtool = tcp_pose[0]  # Coordenada x del TCP
    ytool = tcp_pose[1]  # Coordenada y del TCP
    wrist3 = joint_positions[5]  # Posición del joint 5 (wrist3)

    angle = np.rad2deg(wrist3)
    ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, Hr, Vr, angle)

    #xfinal1 = mapValue(bluexr, 0, 640, 0, Hr)
    xfinal = mapValue(handx, 0 ,640, 0, Hr)
    yfinal = mapValue(handy, 0 ,640, 0, Vr)

    #xtransf1, ytransf1 = transformCoordinates(xfinal1, yfinal1, ofxr, ofyr, thetar)
    xtransf, ytransf = transformCoordinates(xfinal ,yfinal, ofxr, ofyr, thetar)

    #Conversion mm a m
    ofxr = ofxr/1000
    ofyr = ofyr/1000
    ofzr = .373
    xtransf = xtransf/1000
    ytransf = ytransf/1000

    #print(f'mano x pixeles: {handx:.2f}')
    #print(f'mano y pixeles: {handy:.2f}')

    #print(f'x final: {xfinal:.2f}')
    #print(f'y final: {yfinal:.2f}')    

    print(f'xtransf: {xtransf:.2f}')
    print(f'ytransf: {ytransf:.2f}')

