#Libreria de OpenCV
import cv2

# iniciar la camara
cap = cv2.VideoCapture(0)

# crear un contador para las imagenes
count = 0

# crear un while para capturar la imagen
while True:
    # leer la imagen
    ret, frame = cap.read()
    # agregar un filtro de escalas de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # mostrar la imagen
    # cv2.imshow("Captura", frame)
    cv2.imshow("Captura", gray)


    # tomar una foto si se presiona la tecla "c" y guardarla en la carpeta llamada "dataset"
    if cv2.waitKey(1) & 0xFF == ord('c'):
        count += 1
        file_name = f"dataset/imagen_{count}.jpg"
        cv2.imwrite(file_name, gray)
        print(f"Imagen guardada en {file_name}")

    # si se presiona la tecla "q" se cierra la ventana
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# liberar la camara y cerrar la ventana

cap.release()
cv2.destroyAllWindows()

##################################### EN ESTE CODIGO SE TOMARAN LAS FOTOS PARA EL DATASET #####################################