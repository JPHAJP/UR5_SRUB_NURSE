from ultralytics import YOLO

class ObjectDetection:
    def __init__(self, model_path="V1.0/Models/V5_best.pt"):
        """
        Inicializa el modelo YOLO con la ruta especificada.
        """
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
