# SILVIA V2.0

Aplicacion Flask modular para controlar el UR5 desde una interfaz web moderna, con webcam, voz continua, seguimiento de mano, recoleccion de objetos y reglas de seguridad centralizadas.

## Que incluye

- Dashboard unico para estado, safety, feed de camara y conversacion.
- Control del UR5 por sockets URScript:
  - `30002` comandos
  - `30001` lectura de pose
  - `29999` dashboard/safety
  - Logs de comandos dashboard configurables con `UR5_DASHBOARD_LOG_COMMANDS`
- Vision unificada:
  - YOLO con el modelo actual configurable por `VISION_MODEL_PATH`
  - Device de inferencia configurable por `VISION_DEVICE` (`auto`, `cpu`, `cuda:0`)
- Limites de carga configurables con `VISION_INFERENCE_FPS`, `VISION_PREVIEW_FPS` y `VISION_DEPTH_PREVIEW_FPS`; usa `0` para dejar el stream sin limite
  - Umbral configurable por `VISION_CONFIDENCE_THRESHOLD` (default `0.50`)
  - Backend principal `webcam`
  - `X/Y` por homografia 2D de webcam
  - `Z` fija por robot usando `plane_z_mm - ROBOT_PLANE_Z_OFFSET_MM`
  - Instrumentos con YOLO
  - Mano con MediaPipe `HandLandmarker` por default, con fallback a YOLO si MediaPipe no carga al iniciar
  - Seguimiento de mano con Z objetivo `HAND_PLANE_Z_MM - ROBOT_PLANE_Z_OFFSET_MM` (default `350 - 95 = 255` mm)
  - Demo `/hand-demo` en modo `VISION_DETECTOR_MODE=hand_only`, usando backend de mano configurable por `HAND_DETECTOR_BACKEND`
  - Paro por target/camara viejos con `HAND_TARGET_MAX_AGE_S`, `TRACK_COMMAND_HZ` y `MAX_TRACK_ACCEL_MM_S2`
  - Captura ROS2 e inferencia corren en hilos separados
- Voz con OpenAI:
  - STT: `gpt-4o-mini-transcribe`
  - TTS: `gpt-4o-mini-tts`
  - Sesion Realtime preparada por `/api/realtime/session`
- Demo de calibracion para webcam comun.

## Estructura

```text
V2.0/
  run.py
  README.md
  requirements.txt
  .env.example
  app/
  data/
  demo/
  tests/
```

## Dependencias ROS2 para HP60C

- `rclpy`, `cv_bridge` y los mensajes ROS no se instalan con `pip`.
- Deben venir del entorno ROS 2 Jazzy del sistema y de la workspace de `ascamera`.
- Esta ruta queda deshabilitada para el flujo normal con `VISION_BACKEND=webcam`.
- La app puede autoarrancar `ascamera_node` con `ROS_AUTO_LAUNCH_CAMERA=true`.
- Si prefieres correr la camara por fuera, debe publicar:
  - `/ascamera/camera_publisher/rgb0/image`
  - `/ascamera/camera_publisher/depth0/image_raw`
  - `/ascamera/camera_publisher/rgb0/camera_info`
  - `/ascamera/camera_publisher/depth0/camera_info`

## Flujo recomendado

1. Copia `V2.0/.env.example` a `V2.0/.env` o usa la raiz `.env`.
2. Ajusta `VISION_MODEL_PATH`, `VISION_DEVICE`, los FPS de vision, `UR5_HOST`, `CAMERA_INDEX`, `CAMERA_WIDTH`, `CAMERA_HEIGHT` y `CAMERA_FPS`.
3. Deja `VISION_BACKEND=webcam` para el flujo normal.
4. Si necesitas volver a HP60C, reactiva `VISION_BACKEND=hp60c_ros2` y configura ROS 2 por fuera.
5. Si necesitas una calibracion planar legacy, corre la demo:

```bash
python3 V2.0/demo/webcam_calibrator.py
```

6. Instala dependencias Python:

```bash
pip install -r V2.0/requirements.txt
```

Si `cv_bridge` falla con un mensaje sobre NumPy 1.x vs 2.x, fuerza el entorno a `numpy<2`:

```bash
pip install --upgrade "numpy<2" -r V2.0/requirements.txt
```

Si quieres forzar GPU NVIDIA con una build compatible de PyTorch para drivers CUDA 12.1/12.2:

```bash
./venv/bin/pip install --upgrade --force-reinstall \
  torch==2.3.1 torchvision==0.18.1 \
  --index-url https://download.pytorch.org/whl/cu121
```

Para bajar carga de CPU/GPU sin tocar codigo, empieza con:

```bash
VISION_INFERENCE_FPS=0
VISION_PREVIEW_FPS=0
VISION_DEPTH_PREVIEW_FPS=0
ROS_CAMERA_NODE_FPS=10
ROS_CAMERA_STARTUP_GRACE_S=8
ROS_CAMERA_STALL_TIMEOUT_S=3
ROS_CAMERA_NO_CAMERA_INFO_TIMEOUT_S=4
ROS_CAMERA_HARD_RESTART_TIMEOUT_S=6
ROS_CAMERA_HEALTHY_RESET_S=20
ROS_CAMERA_BACKOFF_SEQUENCE_S=2,5,10,20
VISION_DETECTOR_MODE=full
ROBOT_PLANE_Z_OFFSET_MM=95
HAND_TARGET_MAX_AGE_S=0.5
TRACK_COMMAND_HZ=10
MAX_TRACK_ACCEL_MM_S2=300
UR5_DASHBOARD_LOG_COMMANDS=false
```

7. Arranca la app:

```bash
python3 V2.0/run.py
```

8. Abre la app:

- En la misma maquina: `http://localhost:5050`
- Desde otro equipo en la red: `http://<IP-LAN-DEL-SERVIDOR>:5050`

9. Calibra la homografia webcam con `python3 V2.0/demo/webcam_calibrator.py`.
10. Ajusta el plano Z del robot en el dashboard o en `/hand-demo`.
11. Usa `Seguir Mano`; YOLO toma la deteccion `Mano` y el UR5e mantiene `X` fijo para pruebas, corrige `Y` y conserva Z TCP en `plano_mano - 95 mm`.

## Watchdog HP60C y USB

- SILVIA ahora vigila la salud real del stream ROS2 y reinicia `ascamera_node` si el proceso sigue vivo pero deja de publicar.
- El estado se expone en `/api/state` y `/api/vision/depth` dentro de `camera_recovery`.
- La HP60C detectada en esta maquina es `3482:6723` y hoy cuelga de un hub USB 2.0. Si el watchdog sigue recuperando demasiado seguido, el siguiente paso obligatorio es moverla a un puerto directo o a un hub alimentado.

### Desactivar autosuspend de la HP60C

```bash
sudo ./V2.0/scripts/install_hp60c_no_autosuspend.sh
```

### Verificacion operativa

```bash
lsusb -t
cat /sys/bus/usb/devices/3-2.3/power/control
source /opt/ros/jazzy/setup.bash
source hp60c_portable_bundle/workspace/install/setup.bash
ros2 topic echo --once /ascamera/camera_publisher/rgb0/camera_info
tail -f V2.0/data/logs/hp60c_autolaunch.log
```

## Demo Hand Follow

```bash
python3 V2.0/hand_demo.py
```

Abre `http://localhost:5050/hand-demo`. Esta ruta muestra RGB crudo, deteccion anotada, calibracion del plano base Z del robot, target XYZ, delta contra el TCP y velocidad enviada. El seguimiento corrige `X` y `Y`, y mantiene `Z` por offset. El runner fuerza `VISION_DETECTOR_MODE=hand_only` y deja `VISION_INFERENCE_FPS=0`; por default usa MediaPipe para mano y puede caer a YOLO si MediaPipe no esta disponible al iniciar.

## Cloudflare Tunnel

- Si quieres acceso externo o evitar problemas de microfono en navegadores remotos, usa Cloudflare Tunnel.
- El quick tunnel entrega una URL `https://...trycloudflare.com` sin abrir puertos ni tocar el router.
- La URL cambia en cada arranque y esta pensada para pruebas.

### Levantar un quick tunnel

1. Asegurate de que la V2 este corriendo en `127.0.0.1:5050`.
2. Instala `cloudflared` localmente en el repo:

```bash
./V2.0/scripts/install_cloudflared.sh
```

3. Abre el tunel:

```bash
./V2.0/scripts/start_cloudflare_tunnel.sh
```

4. Copia la URL `https://...trycloudflare.com` que imprime `cloudflared`.

Por defecto el script publica `http://127.0.0.1:5050`. Si cambiaste el puerto, exporta `FLASK_PORT` antes de correrlo:

```bash
export FLASK_PORT=5051
./V2.0/scripts/start_cloudflare_tunnel.sh
```

## LAN y HTTPS

- La app ya escucha en `0.0.0.0`, asi que el dashboard, la camara y los controles HTTP funcionan por LAN.
- La voz continua del navegador usa el microfono del cliente y, fuera de `localhost`, normalmente requiere `HTTPS`.
- Si prefieres no usar Cloudflare Tunnel, tambien puedes habilitar HTTPS local con `FLASK_SSL_MODE=adhoc` o con tu propio certificado.

## Comandos utiles

- `Saludar`
- `Seguir Mano`
- `Recoger Objetos`
- `Home`
- `Cancelar`
- `Nueva Conversacion`
- `Liberar Objeto`

## Notas

- El modelo por defecto apunta a `V1.0/Train/runs_intsoob/obb/ur5_yolo11n_obb_512_ultralight/weights/best.pt`.
- El electroiman usa `DO0` por defecto.
- Si el robot entra en fault de safety, la app bloquea los modos automaticos hasta que un operador lo libere manualmente.
- El dashboard principal usa webcam + YOLO y calcula `X/Y` por homografia.
- La Z objetivo ahora se toma del plano del robot con un offset fijo hacia abajo.
