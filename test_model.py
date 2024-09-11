import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# Diccionario que mapea identificadores de clase a nombres de clase
CLASS_NAMES = {
    0: "Bisturi",
    1: "Mano",
    2: "No_Objeto",
    3: "Pinzas",
    4: "Tijeras_curvas",
    5: "Tijeras_rectas"
}

def initialize_pipeline():
    # Inicializa el pipeline de la cámara RealSense.
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgra8, 30)
    pipeline.start(config)
    return pipeline

def get_color_frame(pipeline):
    # Captura el frame de color desde el pipeline de la cámara RealSense.
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None
    return np.asanyarray(color_frame.get_data())

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
    points = []  # Lista para almacenar las coordenadas de los puntos y su clase
    for result in results:
        for bbox, cls in zip(result.obb.xyxy, result.obb.cls):
            # Calcular el centro del cuadro delimitador (bbox)
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)

            # Obtener el nombre de la clase usando el diccionario CLASS_NAMES
            class_name = CLASS_NAMES.get(int(cls), "Desconocido")

            # Agregar las coordenadas y el nombre de la clase a la lista
            points.append((center_x, center_y, class_name))  # (x, y, nombre de clase)

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

def main():
    # Inicializar la cámara y el modelo YOLO
    pipeline = initialize_pipeline()
    model = YOLO('V1.0/Models/V5_best.pt')

    try:
        while True:
            # Obtener el frame de la cámara
            color_image = get_color_frame(pipeline)
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

            # Mostrar la imagen con detecciones y puntos
            display_image('RealSense', annotated_image_with_points)

            # Presionar 'q' para salir
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Imprimir las coordenadas de los puntos junto con el nombre de la clase
            print("Coordenadas de los puntos detectados con clase:", object_points)

    finally:
        # Detener el pipeline y cerrar las ventanas
        pipeline.stop()
        cv2.destroyAllWindows()

# Ejecutar el programa principal
if __name__ == "__main__":
    main()
