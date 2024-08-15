##################################### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #####################################

#Libreria de OpenCV
import cv2
import pyrealsense2 as rs
import numpy as np

# iniciar la camara
# cap = cv2.VideoCapture(2)

# configurar el pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# inicio de la captura de la pantalla
profile = pipeline.start(config)

# crear un contador para las imagenes
count = 0

try:
    
# crear un while para capturar la imagen
    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        # leer la imagen
        """ ret, frame = cap.read() """
        # agregar un filtro de escalas de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # mostrar la imagen
        # cv2.imshow("Captura", frame)
        cv2.imshow("Captura", gray)


        # tomar una foto si se presiona la tecla "c" y guardarla en la carpeta llamada "dataset"
        if cv2.waitKey(1) & 0xFF == ord('c'):
            count += 1
            file_name = f"dataset/background/background_{count}.jpg"
            cv2.imwrite(file_name, gray)
            print(f"Imagen guardada en {file_name}")

        # si se presiona la tecla "q" se cierra la ventana
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:
    pipeline.stop()
# liberar la camara y cerrar la ventana

""" cap.release() """
cv2.destroyAllWindows()

##################################### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #####################################