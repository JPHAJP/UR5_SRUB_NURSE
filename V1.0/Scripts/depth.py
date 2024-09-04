import pyrealsense2 as rs
import numpy as np
import cv2

# Configurar la cámara Intel RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Iniciar la cámara
pipeline.start(config)

try:
    while True:
        # Capturar un frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convertir la imagen de profundidad a un array de numpy
        depth_image = np.asanyarray(depth_frame.get_data())

        # Obtener las dimensiones de la imagen
        height, width = depth_image.shape

        # Calcular el punto central
        center_x, center_y = width // 2, height // 2

        # Obtener la distancia en el punto central
        distance = depth_frame.get_distance(center_x, center_y)

        # Convertir la imagen de profundidad a un formato visualizable
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Dibujar un círculo en el punto central
        cv2.circle(depth_colormap, (center_x, center_y), 5, (0, 0, 255), -1)

        # Escribir la distancia en la imagen
        cv2.putText(depth_colormap, f"{distance:.2f} meters", (center_x + 10, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Mostrar la imagen con el punto y la distancia
        cv2.imshow('Stream con Punto Central y Distancia', depth_colormap)

        # Salir del loop al presionar 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Detener la cámara
    pipeline.stop()
    cv2.destroyAllWindows()

