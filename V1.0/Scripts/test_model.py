import numpy as np
import time
import threading

import pyrealsense2 as rs
import cv2

from ultralytics import YOLO

import rtde_control
import rtde_receive
import rtde_io

# Diccionario que mapea identificadores de clase a nombres de clase
CLASS_NAMES = {
    0: "Bisturi",
    1: "Mano",
    2: "No_Objeto",
    3: "Pinzas",
    4: "Tijeras_curvas",
    5: "Tijeras_rectas"
}

ip = "192.168.1.1"
control = rtde_control.RTDEControlInterface(ip)
receive = rtde_receive.RTDEReceiveInterface(ip)
io = rtde_io.RTDEIOInterface(ip)

# Offset en el eje z para la posición del robot
ofzr = 0.02

def initialize_pipeline():
    # Inicializa el pipeline de la cámara RealSense.
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgra8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    return pipeline

def get_color_frame_and_distance(pipeline):
    # Captura el frame de color desde el pipeline de la cámara RealSense.
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    if not color_frame or not depth_frame:
        print("Error: No se pudo obtener el frame.")
        return None
    
    color_image = np.asanyarray(color_frame.get_data())

    height, width, _ = color_image.shape
    center_x, center_y = int(width / 2), int((height / 2))
    distance = depth_frame.get_distance(center_x, center_y)

    return color_image, distance

def convert_to_grayscale(color_image):
    # Convierte una imagen BGR a escala de grises y luego la reconvierte a 3 canales.
    color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
    gray_image = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2GRAY)
    gray_image_3_channels = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    return gray_image_3_channels

def run_yolo(model, image, conf_threshold=0.6):
    # Ejecuta el modelo YOLO en la imagen capturada con un umbral de confianza.
    results = model(image, conf=conf_threshold)  # Aplicar el umbral de confianza
    return results[0].plot(), results

def calculate_object_points(results):
    # Calcula los puntos en el centro de cada objeto detectado y los guarda en una lista junto con el nombre de la clase.
    points = []  # Lista para almacenar las coordenadas de los puntos, su clase y su confianza
    for result in results:
        for bbox, cls, conf in zip(result.obb.xyxy, result.obb.cls, result.obb.conf):
            # Calcular el centro del cuadro delimitador (bbox)
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)

            # Obtener el nombre de la clase usando el diccionario CLASS_NAMES
            class_name = CLASS_NAMES.get(int(cls), "Desconocido")

            # Obtener la confianza de la detección
            confidence = round(float(conf), 2)

            # Agregar las coordenadas y el nombre de la clase a la lista
            points.append((center_x, center_y, class_name, confidence))  # (x, y, nombre de clase)

    return points

def draw_center_points(image, points):
    # Dibuja un punto en las coordenadas de los puntos detectados.
    for point in points:
        # Dibujar el punto en rojo, usando las coordenadas (x, y)
        cv2.circle(image, (point[0], point[1]), 3, (0, 0, 255), -1)
    return image

def display_image(window_name, image):
    # Muestra la imagen en una ventana.
    cv2.imshow(window_name, image)

def robot_and_camara(distance, object_points):
    # Crear una lista para almacenar los nuevos puntos
    new_object_points = []

    # Calcular la posición del robot y el tamaño de la cámara
    xr, yr, zr, rxr, ryr, rzr = receive.getActualTCPPose()
    joint_positions = receive.getActualQ()
    angle_3 = np.rad2deg(joint_positions[5])

    # Cálculo de H_cam y V_cam
    H_cam = distance * np.tan(23) 
    V_cam = np.abs(distance * np.tan(83.96))

    # Obtener el origen del frame de la cámara
    ofxr, ofyr, thetar = frameOriginCoordinates(xr, yr, H_cam, V_cam, angle_3)

    # Procesar cada punto en object_points
    for point in object_points:
        point = list(point)  # Convertir la tupla a lista para modificarla

        # Calcular la posición del objeto respecto a la cámara
        x_obj, y_obj = transformCoordinates(
            mapValue(point[0], 0, 640, 0, H_cam), 
            mapValue(point[1], 0, 480, 0, V_cam), 
            ofxr, 
            ofyr, 
            thetar
        )

        # Actualizar las coordenadas x y y en la lista
        point[0] = x_obj
        point[1] = y_obj

        # Añadir el punto modificado a la nueva lista
        new_object_points.append(tuple(point))  # Si deseas devolver una tupla, vuelve a convertirla

        # Imprimir el resultado
        #print(f"Clase: {point[2]}, Coordenadas (x, y): ({point[0]}, {point[1]})")

    # Devolver la lista de nuevos puntos
    return new_object_points

def mapValue(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

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

def frameOriginCoordinates(xtool, ytool, H_cam, V_cam, wrist3):
    # En metros? AYUDAAAA
    radio = 39.18/1000
    longitud_tangente = 11.5/1000

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
    ofx = cfx - (H_cam/2) * np.cos(angulo_tangente) - (V_cam/2) * np.sin(angulo_tangente)
    ofy = cfy - (H_cam/2) * np.sin(angulo_tangente) + (V_cam/2) * np.cos(angulo_tangente)

    return ofx, ofy, angulo_tangencia

def move_robot(xtransfn, ytransfn, ofzn):
    xr, yr, zr, rxr, ryr, rzr = receive.getActualTCPPose()
    #destinationf = [xtransfn, ytransfn, ofzn]
    control.moveL([xtransfn, ytransfn, ofzn, rxr, ryr, rzr], .5, .5, asynchronous=True)
    # Normalizamos para poder hacer comparativas
    #destinationf = np.around(destinationf, decimals=2)

def gohome():
    # Función para mover el robot a la posición "Home"
    # Coordenadas articulares para el home
    home_joint_angles_deg = [-51.9, -71.85, -112.7, -85.96, 90, 38]
    # Convertir la lista de ángulos a radianes
    home_joint_angles_rad = np.radians(home_joint_angles_deg)
    # Mover el robot a la posición "Home" usando control.moveJ
    # Velocidad = 1 rad/s, Aceleración = 1 rad/s^2
    control.moveJ(home_joint_angles_rad, 1, 1, asynchronous=True)

def monitor_io_and_interrupt():
    # Función para monitorear el estado del sensor digital y detener el robot si se activa
    while True:
        sensor_state = receive.getDigitalInState(0)  # Leer el estado del pin 0

        if sensor_state:  # Si el sensor se activa
            print("Sensor activado, deteniendo el robot.")
            control.stopL(1.0)  # Detener el movimiento del robot si se está moviendo por trayectorias
            control.stopJ(1.0)  # Detener el movimiento del robot si se está moviendo por articulaciones
            gohome()
            time.sleep(5)  # Esperar un poco antes de verificar de nuevo
            io.setStandardDigitalOut(0, False)  # Apagar el electroimán
        time.sleep(0.1)  # Esperar un poco antes de verificar de nuevo

def safe_move_to_home():
    #check_and_recover_protective_stop()
    if control.isConnected():  # Verifica que RTDE esté conectado
        gohome()  # Mover el robot a "home"
    else:
        pass
        #restart_rtde_script()  # Reiniciar la conexión RTDE si no está conectada

def seguir_mano(object_points):
    clase_seleccionada = 'Mano'  # Mano
    # Filtrar los puntos de la clase seleccionada
    puntos_clase = [(x, y, clase, score) for (x, y, clase, score) in object_points if clase == clase_seleccionada]
    print(f"Puntos de la clase {clase_seleccionada}: {puntos_clase}")
    
    # Asegurarse de que hay puntos para la clase seleccionada
    if puntos_clase:
        # Selecciona el primer punto de la clase seleccionada
        xtransf, ytransf = puntos_clase[0][0], puntos_clase[0][1]
        print(f"Coordenadas del punto seleccionado: ({xtransf}, {ytransf})")
        
        # Obtener las coordenadas actuales del robot
        xrobot, yrobot, zrobot, _, _, _ = receive.getActualTCPPose()
        print(f"Coordenadas actuales del robot: ({xrobot}, {yrobot})")
        
        # Función para comprobar si el robot se ha detenido
        def robot_esta_detenido():
            velocidad_actual = receive.getActualTCPSpeed()  # Obtén la velocidad actual del robot
            velocidad_limite = 0.01  # Define un umbral para considerar que el robot está detenido
            return all(abs(v) < velocidad_limite for v in velocidad_actual)  # Comprueba si todas las componentes de la velocidad son menores que el umbral
        
        # Función para calcular el error porcentual entre las coordenadas actuales y las objetivo
        def error_dentro_del_rango(x_actual, y_actual, x_objetivo, y_objetivo, porcentaje_error):
            error_x = abs(x_objetivo - x_actual) / abs(x_objetivo) if x_objetivo != 0 else 0
            error_y = abs(y_objetivo - y_actual) / abs(y_objetivo) if y_objetivo != 0 else 0
            return error_x <= porcentaje_error and error_y <= porcentaje_error
        
        # Establecer el porcentaje de error permitido
        porcentaje_error_permitido = 0.1
        
        # Verificar si ya estamos dentro del margen de error antes de mover
        if error_dentro_del_rango(xrobot, yrobot, xtransf, ytransf, porcentaje_error_permitido):
            print(f"El robot ya está dentro del margen de error del {porcentaje_error_permitido*100}%, no se requiere corrección.")
        else:
            # Esperar a que el robot se detenga antes de moverse
            while not robot_esta_detenido():
                print("Esperando a que el robot se detenga...")
                #time.sleep(0.1)  # Esperar 100 ms antes de comprobar de nuevo
            
            # Imprimir el movimiento que se va a realizar
            print(f"Moviendo el robot a las coordenadas objetivo ({xtransf}, {ytransf})")
            
            # Mover el robot directamente a las coordenadas transformadas (objetivo)
            move_robot(xtransf, ytransf, ofzr)  
            
            print("El robot ha alcanzado las coordenadas objetivo.")
        
    else:
        print(f"No se encontraron puntos para la clase {clase_seleccionada}")

def mostrar_menu(object_points):
    # Extraer las clases únicas detectadas en el object_points
    clases_detectadas = list({point[2] for point in object_points})  # Usamos un set para evitar duplicados

    # Mostrar solo las opciones que están en las clases detectadas
    print("Menu:")
    opciones_menu = {}
    for key, value in CLASS_NAMES.items():
        if value in clases_detectadas:
            print(f"{key + 1}.- {value}")
            opciones_menu[key + 1] = value

    # Capturar la respuesta del usuario en un bucle hasta que se ingrese una opción válida
    while True:
        try:
            resp = int(input("Seleccione el instrumento deseado: "))

            if resp in opciones_menu:
                # Si la opción es válida, romper el bucle
                break
            else:
                print("Opción no válida. Por favor, seleccione una opción válida.")
        except ValueError:
            # Si no se ingresa un número entero
            print("Entrada no válida. Por favor, ingrese un número.")

    # Encender electroimán
    io.setStandardDigitalOut(0, True)

    # Obtener la clase seleccionada
    clase_seleccionada = opciones_menu[resp]

    # Filtrar los puntos de la clase seleccionada
    puntos_clase = [(x, y, clase, score) for (x, y, clase, score) in object_points if clase == clase_seleccionada]

    # Asegurarse de que hay puntos para la clase seleccionada
    if puntos_clase:
        # Selecciona el primer punto de la clase seleccionada
        xtransf, ytransf = puntos_clase[0][0], puntos_clase[0][1]

        print(f"Coordenadas del punto seleccionado: ({xtransf}, {ytransf})")
        # Mover el robot a las coordenadas transformadas
        move_robot(xtransf, ytransf, ofzr)
    else:
        print(f"No se encontraron puntos para la clase {clase_seleccionada}")

    # Opción para volver al home
    while True:
        try:
            resp2 = int(input("Presione 1 para volver al home: "))
            if resp2 == 1:
                gohome()
                break
            else:
                print("Opción no válida. Solo presione 1 para volver al home.")
        except ValueError:
            print("Entrada no válida. Por favor, ingrese un número.")

def main():
    safe_move_to_home()
    # Inicializar la cámara y el modelo YOLO
    pipeline = initialize_pipeline()
    model = YOLO(r'V1.0/Models/V5_best.pt')

    # Iniciar un hilo para monitorear el estado del sensor digital y detener el robot si se activa
    monitor_thread = threading.Thread(target=monitor_io_and_interrupt)
    monitor_thread.start()
    try:
        while True:
            while True:
                # Obtener el frame de la cámara
                color_image, distance = get_color_frame_and_distance(pipeline)
                if color_image is None:
                    continue

                # Convertir la imagen a escala de grises
                gray_image_3_channels = convert_to_grayscale(color_image)

                # Ejecutar YOLO en la imagen en escala de grises con un umbral de confianza de 0.6
                annotated_image, results = run_yolo(model, gray_image_3_channels, conf_threshold=0.6)

                # Calcular los puntos de los objetos detectados y guardarlos en una lista
                object_points = calculate_object_points(results)

                # Dibujar puntos en los centros de los objetos detectados
                annotated_image_with_points = draw_center_points(annotated_image, object_points)

                # Calcular posición de robot y objeto respecto a la cámara
                transformed_object_points = robot_and_camara(distance, object_points)

                # Mostrar la imagen con detecciones y puntos
                display_image('RealSense', annotated_image_with_points)

                # Presionar 'q' para salir
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Imprimir las coordenadas de los puntos junto con el nombre de la clase
                print("Coordenadas de los puntos detectados con clase:", transformed_object_points)
            
            mostrar_menu(transformed_object_points)

    finally:
        # Detener el pipeline y cerrar las ventanas
        io.setStandardDigitalOut(0, False)
        pipeline.stop()
        cv2.destroyAllWindows()
        # Terminar el hilo de monitoreo
        monitor_thread.join()


# Ejecutar el programa principal
if __name__ == "__main__":
    main()