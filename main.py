import cv2
from camera_pipeline import CameraPipeline
from object_detection import ObjectDetection

def main():
    # Inicializa la cámara
    camera = CameraPipeline()
    camera.initialize_pipeline()
    
    if not camera.pipeline:
        print("No se pudo inicializar el pipeline de la cámara.")
        return

    # Carga el modelo YOLO con la ruta correcta
    yolo_model = ObjectDetection()

    while True:
        color_image, distance = camera.get_color_frame_and_distance()
        if color_image is None:
            print("No se obtuvieron frames. Saliendo...")
            break

        # Ejecuta YOLO en el frame capturado
        annotated_image = yolo_model.run_yolo(color_image)

        # Muestra el frame anotado
        cv2.imshow("Detección de Objetos", annotated_image)

        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libera los recursos de la cámara
    camera.release_pipeline()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
