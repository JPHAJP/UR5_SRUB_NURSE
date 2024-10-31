import os
from flask import Flask, render_template, request, Response
from classCam import Camera
import cv2

# Diccionario que mapea identificadores de clase a nombres de clase
CLASS_NAMES = {
    0: "Bisturi",
    1: "Mano",
    2: "No_Objeto",
    3: "Pinzas",
    4: "Tijeras_curvas",
    5: "Tijeras_rectas"
}

# Offset en el eje z para la posición del robot
ofzr = 0.01

app = Flask(__name__)

# Usar la ruta absoluta del modelo YOLO
model_path = os.path.abspath('V6_best.pt')
camera = Camera(model_path)

# Función para capturar frames de la cámara RealSense y ejecutar YOLO
def get_frame():
    try:
        while True:
            # Capturar un frame de la cámara usando la clase Camera
            frame_jpeg, frame_numpy = camera.get_frame()
            if frame_numpy is None:
                continue

            # Ejecutar YOLO sobre el frame capturado
            yolo_frame, results = camera.run_yolo(frame_numpy)

            # Codificar la imagen con detecciones de YOLO
            ret, buffer = cv2.imencode('.jpg', yolo_frame)
            frame = buffer.tobytes()

            # Usar el frame como un flujo de video en formato multipart/x-mixed-replace
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    except Exception as e:
        print(f"Error capturando frames: {e}")
    finally:
        camera.stop()

@app.route('/', methods=['GET', 'POST'])
def home():
    # Valores predeterminados
    valores = {
        'ur_x': 0, 'ur_y': 0, 'ur_z': 0, 
        'ur_rx': 0, 'ur_ry': 0, 'ur_rz': 0,
        'x': 0, 'y': 0, 'z': 0,
        'rx': 0, 'ry': 0, 'rz': 0,
        'interruptor': False,
        'iman': False,
        'conexion_ur': False
    }

    if request.method == 'POST':
        # Capturar los valores de los checkboxes y otros campos del formulario
        valores['interruptor'] = request.form.get('interruptor') == 'on'
        valores['iman'] = request.form.get('iman') == 'on'
        valores['conexion_ur'] = request.form.get('conexion_ur') == 'on'

    return render_template('index.html', **valores)

@app.route('/video_feed')
def video_feed():
    # Iniciar la cámara antes de generar los frames de video
    camera.start()
    return Response(get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    pass

if __name__ == '__main__':
    try:
        app.run(debug=True)
    except KeyboardInterrupt:
        camera.stop()
