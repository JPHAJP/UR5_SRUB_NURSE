import pyrealsense2 as rs
import numpy as np

class CameraPipeline:
    def __init__(self):
        self.pipeline = None

    def initialize_pipeline(self):
        """
        Inicializa el pipeline de la cámara RealSense.
        """
        try:
            print("Iniciando pipeline de la cámara...")
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgra8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            print("Pipeline de RealSense inicializado correctamente.")
        except Exception as e:
            print(f"Error al inicializar la cámara: {e}")
            self.pipeline = None

    def get_color_frame_and_distance(self):
        """
        Captura el frame de color y la distancia desde el centro de la imagen.
        """
        try:
            if self.pipeline is None:
                print("Pipeline no inicializado.")
                return None, None
            
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame or not depth_frame:
                print("Error: No se pudo obtener el frame.")
                return None, None

            color_image = np.asanyarray(color_frame.get_data())
            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int(height / 2)
            distance = depth_frame.get_distance(center_x, center_y)
            print(f"Distancia desde el centro: {distance:.2f} metros")
            return color_image, distance
        except Exception as e:
            print(f"Error al capturar frames: {e}")
            return None, None

    def release_pipeline(self):
        """
        Libera los recursos del pipeline de la cámara.
        """
        if self.pipeline:
            self.pipeline.stop()
            print("Pipeline detenido.")
