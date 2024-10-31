from ultralytics import YOLO

class YoloModel:
    def __init__(self, model_path):
        # Inicializar el modelo YOLO con la ruta proporcionada
        self.model = YOLO(model_path)

    def run(self, image, conf_threshold=0.6):
        # Ejecuta el modelo YOLO en la imagen capturada con un umbral de confianza
        results = self.model(image, conf=conf_threshold)  # Aplicar el umbral de confianza
        return results[0].plot(), results  # Devolver la imagen con detecciones y los resultados
