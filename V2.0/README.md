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
  - Umbral configurable por `VISION_CONFIDENCE_THRESHOLD` (default `0.50`)
  - La mano/guante tambien se sigue con YOLO usando la clase del modelo
  - Captura de camara e inferencia corren en hilos separados
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

6. Abre la app:

- En la misma maquina: `http://localhost:5050`
- Desde otro equipo en la red: `http://<IP-LAN-DEL-SERVIDOR>:5050`

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

- El modelo por defecto apunta a `V_COVA/Dashboard/V6_best.pt`.
- El electroiman usa `DO0` por defecto.
- Si el robot entra en fault de safety, la app bloquea los modos automaticos hasta que un operador lo libere manualmente.
