# SILVIA V2.0

Aplicacion Flask modular para controlar el UR5 desde una interfaz web moderna, con webcam, voz continua, seguimiento de mano, recoleccion de objetos y reglas de seguridad centralizadas.

## Que incluye

- Dashboard unico para estado, safety, feed de camara y conversacion.
- Control del UR5 por sockets URScript:
  - `30002` comandos
  - `30001` lectura de pose
  - `29999` dashboard/safety
- Vision unificada:
  - YOLO con el modelo actual configurable por `VISION_MODEL_PATH`
  - MediaPipe para seguimiento de mano
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

## Flujo recomendado

1. Copia `V2.0/.env.example` a `V2.0/.env` o usa la raiz `.env`.
2. Ajusta `VISION_MODEL_PATH`, `UR5_HOST` y parametros de camara.
3. Corre la calibracion:

```bash
python3 V2.0/demo/webcam_calibrator.py
```

4. Instala dependencias:

```bash
pip install -r V2.0/requirements.txt
```

5. Arranca la app:

```bash
python3 V2.0/run.py
```

6. Abre `http://localhost:5050`.

## Comandos utiles

- `Saludar`
- `Seguir Mano`
- `Recoger Objetos`
- `Home`
- `Cancelar`
- `Nueva Conversacion`
- `Liberar Objeto`

## Notas

- El modelo por defecto apunta a `V_COVA/Dashboard/V6_best.pt`.
- El electroiman usa `DO0` por defecto.
- Si el robot entra en fault de safety, la app bloquea los modos automaticos hasta que un operador lo libere manualmente.
