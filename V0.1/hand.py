
# import socket
# import struct
# import pyrealsense2 as rs
# import cv2
# import numpy as np
# from time import sleep
# import mediapipe as mp

# #Función para obtener tanto coordenadas articulares como cartesianas del robot vía Socket
# def getPose():
#     #Conexión con el UR5
#     HOST = '192.168.1.1'
#     PORT = 30002

#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     s.connect((HOST, PORT))

#     cdr = 0
#     while cdr < 3:
#         #Recibir 4096 bytes de datos (suficiente para almacenar cualquier paquete)
#         data = s.recv(4096)
#         #Inicializar i para trackear la posición en el paquete
#         i = 0
#         if data:
#             #Información del paquete recibido
#             packlen = (struct.unpack('!i', data[0:4]))[0]
#             timestamp = (struct.unpack('!Q', data[10:18]))[0]
#             packtype = (struct.unpack('!b', data[4:5]))[0]

#             #Si el tipo de paquete es el estado del robot, loopear hasta alcanzar el final del paquete
#             if packtype == 16:
#                 while i + 5 < packlen:
#                     #Tamaño y tipo de mensaje
#                     msglen = (struct.unpack('!i', data[5+i:9+i]))[0]
#                     msgtype = (struct.unpack('!b', data[9+i:10+i]))[0]

#                     if msgtype == 1:
#                         #Si el mensaje es coordenada articular, crear una lista para guardar los ángulos
#                         angle = [0] * 6
#                         j = 0
#                         while j < 6:
#                             #Bytes del 10 al 18 tienen ángulo j0, cada coordenada articular es de 41 bytes (saltamos j*41 cada vez)
#                             angle[j] = (struct.unpack('!d', data[10+i+(j*41):18+i+(j*41)]))[0]
#                             j += 1

#                     #Si el tipo de mensaje son coordenadas cartesianas, almacenamos las coordenadas actuales del TCP
#                     elif msgtype == 4:
#                         x = (struct.unpack('!d', data[10+i:18+i]))[0]
#                         y = (struct.unpack('!d', data[18+i:26+i]))[0]
#                         z = (struct.unpack('!d', data[26+i:34+i]))[0]
#                         rx = (struct.unpack('!d', data[34+i:42+i]))[0]
#                         ry = (struct.unpack('!d', data[42+i:50+i]))[0]
#                         rz = (struct.unpack('!d', data[50+i:58+i]))[0]

#                     #Incrementamos i por el tamaño del mensaje para movernos al siguiente mensaje en el paquete
#                     i += msglen
#         cdr = cdr + 1

#     return x, y, z, rx, ry, rz, angle[5]

# #Función para calcular las coordenadas cartesianas del origen del frame respecto a la base del robot
# def frameOriginCoordinates(xtool, ytool, H, V, wrist3):
#     radio = 39.18
#     longitud_tangente = 11.5

#     #Ángulo del vector de posición (xf, yf)
#     theta = np.arctan2(ytool, xtool)

#     #Coordenadas del punto de tangencia
#     angulo_tangencia = theta + np.radians(45) - np.radians(wrist3) + np.radians(22)

#     #Dirección de la tangente
#     angulo_tangente = angulo_tangencia - np.pi / 2

#     #Componentes rectangulares del lente principal de la cámara
#     Ctcp = np.sqrt(radio**2 + longitud_tangente**2)
#     theta1 = np.arccos(radio / Ctcp)
#     theta2 = angulo_tangencia - theta1
#     cfx = Ctcp * np.cos(theta2) + xtool
#     cfy = Ctcp * np.sin(theta2) + ytool

#     #Vector de posición del origen del frame
#     ofx = cfx - (H/2) * np.cos(angulo_tangente) - (V/2) * np.sin(angulo_tangente)
#     ofy = cfy - (H/2) * np.sin(angulo_tangente) + (V/2) * np.cos(angulo_tangente)

#     return ofx, ofy, angulo_tangencia

# #Función para calcular el centro de las predicciones
# def calculateCenter(image, center_x, center_y, red, green, blue):
#     cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)

# #Función para encontrar la densidad de pixeles por metro
# def mapValue(value, from_low, from_high, to_low, to_high):
#     return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

# #Función para transformar coordenadas locales de la fotografía en coordenadas globales (respecto la base del robot)
# def transformCoordinates(x1, y1, ofx, ofy, theta):
#     r = np.array([
#                     [0, -1, 0],
#                     [-1, 0, 0],
#                     [0, 0, 1]
#     ])

#     R_z = np.array([
#                     [np.cos(theta), -np.sin(theta), 0],
#                     [np.sin(theta), np.cos(theta), 0],
#                     [0, 0, 1]
#     ])

#     R0_f = np.dot(R_z, r)

#     P0_f = np.array([
#                         [ofx],
#                         [ofy],
#                         [0]
#     ])

#     #Poderosas matrices de transformación homogenea
#     H0_f = np.concatenate((R0_f, P0_f), 1)
#     H0_f = np.concatenate((H0_f, [[0,0,0,1]]), 0)

#     punto = np.array([
#         [x1],
#         [y1],
#         [0],
#         [1]
#     ])

#     coordTransf = np.dot(H0_f, punto)
    
#     return coordTransf[0][0], coordTransf[1][0]

# #Función para pedirle al UR5 hacer un movimiento lineal 
# def sendInstructionToUr5(ip, port, instruction):
#     try:
#         #Objeto socket
#         sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
#         #Conectarse al robot
#         #sock.connect((ip, port))
        
#         #Manda la instrucción al robot
#         #sock.sendall(instruction.encode())
        
#         print("Instrucción mandada al UR5: ", instruction)
        
#         #Cierra la conexión
#         sock.close()
        
#     except ConnectionRefusedError:
#         print("Conexión rechazada")
    
#     except TimeoutError:
#         print("Tiempo agotado, no se puede conectar")

# #Función para manejar las solicitudes de cambio de pose para el robot
# def handleRobotPoseMoveRequest(destination, currentPose):

#     while destination[0] != currentPose[0] and destination[1] != currentPose[1] and destination[2] != currentPose[2]:
#         currentPosex, currentPosey, currentPosez, _, _, _, _ = getPose()
#         currentPosex = np.around(currentPosex, decimals=2)
#         currentPosey = np.around(currentPosey, decimals=2)
#         currentPosez = np.around(currentPosez, decimals=2)
#         currentPose = [currentPosex, currentPosey, currentPosez]

#         print("Current pose: ")
#         print(currentPose)
#         print("Destination: ")
#         print(destination)

# def cameracontrol():
#     # Initialize MediaPipe Hands
#     mp_hands = mp.solutions.hands
#     hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)

#     # Initialize RealSense Camera
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#     pipeline.start(config)

#     try:
#         while True:
#             # Capture frames from RealSense
#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 continue
            
#             # Convert images to numpy array
#             color_image = np.asanyarray(color_frame.get_data())
            
#             # Convert the image to RGB
#             image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
#             # Process the image and detect hands
#             results = hands.process(image_rgb)
            
#             if results.multi_hand_landmarks:
#                 for hand_landmarks in results.multi_hand_landmarks:
#                     # Extract keypoint 9 (Middle Finger Tip)
#                     keypoint_9 = hand_landmarks.landmark[9]
#                     x, y = int(keypoint_9.x * color_image.shape[1]), int(keypoint_9.y * color_image.shape[0])
                    
#                     # Draw keypoint 9 on the image
#                     cv2.circle(color_image, (x, y), 10, (0, 255, 0), cv2.FILLED)
#                     cv2.putText(color_image, "Keypoint 9", (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
#             # Display the output
#             cv2.imshow('RealSense', color_image)
            
#             # Break the loop if 'q' is pressed
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#     finally:
#         # Release resources
#         pipeline.stop()
#         cv2.destroyAllWindows()

# T1 = np.radians(-56.42)
# T2 = np.radians(-110.61)
# T3 = np.radians(-85.44)
# T4 = np.radians(17.86)
# T5 = np.radians(90)
# T6 = np.radians(45)

# ur5_move_command_1 = f"movej([{T1}, {T2}, {T3}, {T4}, {T5}, {T6}], a=1, v=1.05)\n"

# URip = '192.168.1.1'
# URport = 30002
# xi = 0.117
# yi = -0.364
# zi = 0.375

# #sendInstructionToUr5(URip, URport, ur5_move_command_1)
# cameracontrol()
# """
# for i in range(1,5,1):
#     xi = xi + 0.02
#     yi = yi - 0.02 
#     #ur5_move_command_1 = f"movel(p[{xi},{yi},{zi},0,3.142,0], a = 1.2, v = 0.25, t = 0, r = 0)\n" #Origen 1 para pruebas
#     currentx, currenty, currentz, _, _, _, _ = getPose()
#     currentPose = [currentx, currenty, currentz]
#     destination = [xi, yi, zi]
#     destination = np.around(destination, decimals=2)
#     handleRobotPoseMoveRequest(destination, currentPose)
#     sendInstructionToUr5(URip, URport, ur5_move_command_1)
# """

import URBasic
import numpy as np
import cv2
import time
import pyrealsense2 as rs
import math3d as m3d
import mediapipe as mp

ROBOT_IP = '192.168.1.1'
ACCELERATION = 0.9
VELOCITY = 0.8
"""
robot_startposition = (np.radians(-218),
                       np.radians(-63),
                       np.radians(-93),
                       np.radians(-20),
                       np.radians(88),
                       np.radians(0))
"""

robot_startposition = (np.radians(-56.42),
                       np.radians(-110.61),
                       np.radians(-85.44),
                       np.radians(17.86),
                       np.radians(90),
                       np.radians(45))

video_resolution = (640, 480)
video_midpoint = (int(video_resolution[0] / 2),
                  int(video_resolution[1] / 2))
m_per_pixel = 0.00009

max_x = 0.2
max_y = 0.2

hor_rot_max = np.radians(50)
vert_rot_max = np.radians(25)

# Initialize MediaPipe Hand model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, video_resolution[0], video_resolution[1], rs.format.bgr8, 30)
pipeline.start(config)
time.sleep(0.2)

def find_hand_position(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Get keypoint 9 position
            x = int(hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * video_resolution[0])
            y = int(hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * video_resolution[1])
            position_from_center = (x - video_midpoint[0], y - video_midpoint[1])
            return position_from_center, image
    return None, image

def show_frame(frame):
    cv2.imshow('RobotCamera', frame)
    cv2.waitKey(6)

def check_max_xy(xy_coord):
    x_y = [0, 0]
    x_y[0] = xy_coord[0] if -max_x <= xy_coord[0] <= max_x else (-max_x if xy_coord[0] < -max_x else max_x)
    x_y[1] = xy_coord[1] if -max_y <= xy_coord[1] <= max_y else (-max_y if xy_coord[1] < -max_y else max_y)
    return x_y

def set_lookorigin():
    position = robot.get_actual_tcp_pose()
    orig = m3d.Transform(position)
    return orig

def move_to_hand(hand_pos, robot_pos):
    hand_from_center = list(hand_pos)
    prev_robot_pos = robot_pos
    scaled_hand_pos = [c * m_per_pixel for c in hand_from_center]
    robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_hand_pos)]
    robot_target_xy = check_max_xy(robot_target_xy)
    prev_robot_pos = robot_target_xy

    x = robot_target_xy[0]
    y = robot_target_xy[1]
    z = 0
    xyz_coords = m3d.Vector(x, y, z)

    x_pos_perc = x / max_x
    y_pos_perc = y / max_y

    x_rot = x_pos_perc * hor_rot_max
    y_rot = y_pos_perc * vert_rot_max * -1

    tcp_rotation_rpy = [y_rot, x_rot, 0]
    tcp_orient = m3d.Orientation.new_euler(tcp_rotation_rpy, encoding='xyz')
    position_vec_coords = m3d.Transform(tcp_orient, xyz_coords)

    oriented_xyz = origin * position_vec_coords
    oriented_xyz_coord = oriented_xyz.get_pose_vector()

    coordinates = oriented_xyz_coord
    robot.set_realtime_pose(coordinates)

    return prev_robot_pos

print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

robot.reset_error()
print("robot initialised")
time.sleep(1)

robot.movej(q=robot_startposition, a=ACCELERATION, v=VELOCITY)

robot_position = [0, 0]
origin = set_lookorigin()

robot.init_realtime_control()
time.sleep(1)

try:
    print("starting loop")
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        hand_position, new_frame = find_hand_position(frame)
        show_frame(new_frame)
        if hand_position:
            robot_position = move_to_hand(hand_position, robot_position)

    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    robot.close()

except:
    robot.close()
