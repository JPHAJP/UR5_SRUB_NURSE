import pyrealsense2 as rs
import numpy as np
import cv2
from classModelYolo import YoloModel  # Importar la clase YoloModel

class Camera:
    def __init__(self, model_path):
        # Inicializar el pipeline de RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configurar el stream de la cámara (Resolución 640x480 y 30 FPS)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Inicializar el modelo YOLO utilizando la clase YoloModel
        self.yolo_model = YoloModel(model_path)

    def start(self):
        # Iniciar la cámara
        self.pipeline.start(self.config)

    def stop(self):
        # Detener la cámara
        if self.pipeline:
            self.pipeline.stop()

    def get_frame(self):
        # Captura un frame de la cámara
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return None, None

        # Convertir la imagen a un array de NumPy
        frame = np.asanyarray(color_frame.get_data())

        # Codificar el frame en formato JPEG para el streaming
        ret, buffer = cv2.imencode('.jpg', frame)
        return buffer.tobytes(), frame  # Devolver el frame JPEG y el frame en formato NumPy

    def run_yolo(self, image, conf_threshold=0.6):
        # Utilizar el modelo YOLO para ejecutar detecciones en la imagen
        return self.yolo_model.run(image, conf_threshold)
