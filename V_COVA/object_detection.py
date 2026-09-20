import os

from ultralytics import YOLO

class ObjectDetection:
    def __init__(self, model_path=None):
        """
        Inicializa el modelo YOLO con la ruta especificada.
        """
        if model_path is None:
            repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            model_path = os.getenv(
                "VISION_MODEL_PATH",
                os.path.join(
                    repo_dir,
                    "V1.0",
                    "Train",
                    "runs_intsoob",
                    "obb",
                    "ur5_yolo11n_obb_512_ultralight",
                    "weights",
                    "best.pt",
                ),
            )
        try:
            self.model = YOLO(model_path)
            print(f"Modelo YOLO cargado desde {model_path}")
        except FileNotFoundError as e:
            print(f"Error al cargar el modelo: {e}")

    def run_yolo(self, image):
        """
        Ejecuta el modelo YOLO en la imagen capturada.
        """
        try:
            results = self.model(image)
            annotated_image = results[0].plot()  # Imagen con anotaciones de detección
            return annotated_image
        except Exception as e:
            print(f"Error al ejecutar YOLO: {e}")
            return image
