
from flask import Flask, render_template, request, Response
import numpy as np
import time
import threading

import pyrealsense2 as rs
import cv2

from ultralytics import YOLO

# import rtde_control
# import rtde_receive
# import rtde_io

# Diccionario que mapea identificadores de clase a nombres de clase
CLASS_NAMES = {
    0: "Bisturi",
    1: "Mano",
    2: "No_Objeto",
    3: "Pinzas",
    4: "Tijeras_curvas",
    5: "Tijeras_rectas"
}

# ip = "192.168.1.1"
# control = rtde_control.RTDEControlInterface(ip)
# receive = rtde_receive.RTDEReceiveInterface(ip)
# io = rtde_io.RTDEIOInterface(ip)

# Offset en el eje z para la posición del robot
ofzr = 0.01

app = Flask(__name__)

cap = cv2.VideoCapture(0)

# Funcion para activar la camara con opencv
def get_frame():
    if not cap.isOpened():
        print("Error al abrir la camara")
        return
    
    while True:
        while True:
            # Capturar frame a frame
            success, frame = cap.read()
            if not success: 
                break
            else:
                # Codificar el frame en formato JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()

                # Usar el frame como un flujo de video en formato multipart/x-mixed-replace
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/', methods=['GET', 'POST'])
def home():
    # Capturar los valores de los checkboxes
    main()
    if request.method == 'POST':
        interruptor = request.form.get('interruptor') == 'True'
        iman = request.form.get('iman') == 'True'
        conexion_ur = request.form.get('conexion_ur') == 'True'
    
        valores = {
            'ur_x': 0, 'ur_y': 0, 'ur_z': 0, 
            'ur_rx': 0, 'ur_ry': 0, 'ur_rz': 0, # ur son las coordenadas del robot
            'x': 0, 'y': 0, 'z': 0,             # x, y, z son las coordenadas del objeto
            'rx': 0, 'ry': 0, 'rz': 0,
            'interruptor': interruptor,
            'iman': iman,
            'conexion_ur': conexion_ur
            }
    
    else:
        valores = {
            'ur_x': 'Null', 'ur_y': 'Null', 'ur_z': 'Null', 
            'ur_rx': 'Null', 'ur_ry': 'Null', 'ur_rz': 'Null',
            'x': 'Null', 'y': 'Null', 'z': 'Null',
            'rx': 'Null', 'ry': 'Null', 'rz': 'Null',
            'interruptor': 'Null',
            'iman': 'Null',
            'conexion_ur': 'Null'
            }
    return render_template('index.html', **valores)

@app.route('/video_feed')
def video_feed():
    # Ruta que genera los frames de video
    return Response(get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    pass

if __name__ == '__main__':
    app.run(debug=True)