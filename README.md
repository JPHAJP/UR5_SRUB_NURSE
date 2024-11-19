# 🤖 UR5_SRUB_NURSE — S.I.L.V.I.A. | **Surgical Instrument Logistics Virtual Intelligent Assistant**

Repositorio para el trabajo con el robot UR5 utilizando una cámara de profundidad para la detección de instrumental quirúrgico mediante inteligencia artificial.

---

## ⚙️ **Sistema de Cambio Rápido de Herramental**

Adaptado por **JPHA**  

Archivos principales para el cambio de herramienta:
- **🔗 Base**
- **🔩 Cople**
- **🔒 Pestillo**

> **Nota**: Pronto estarán disponibles los planos completos. Mientras tanto, usa los archivos STEP en la carpeta **3D_Models**.

---

## 📚 **Librerías Utilizadas**

- [Mediapipe](https://google.github.io/mediapipe/) 📷
- [Ultralytics](https://ultralytics.com/) 🧠
- `pyrealsense2` 🌐
- `numpy` 📐
- `cv2` 🎥
- [UR RTDE](https://sdurobotics.gitlab.io/ur_rtde/index.html) 🤖
- `ollama` 🦙
- [Whisper](https://github.com/openai/whisper) 🗣️
- `gradio` 🌉
- `gtts` 🔊
- `flask` 🌐

---

## 🐋 **UR en Docker**

Puedes acceder a la imagen de UR en Docker:
- [Universal Robots URSim e-Series](https://hub.docker.com/r/universalrobots/ursim_e-series)

---

## 🛠️ **Instalaciones adicionales**

Para que todo funcione correctamente, instala los siguientes paquetes:

```bash
sudo apt-get install portaudio19-dev
```

---

## 🔗 **Referencias adicionales**

### UR_RTDE
Documentación de UR_RTDE para la comunicación con el robot UR5:
- [UR RTDE Documentation](https://sdurobotics.gitlab.io/ur_rtde/index.html)

### Whisper
Sistema de transcripción de OpenAI para detección de voz:
- [Whisper en GitHub](https://github.com/openai/whisper)

### Llama3.1 (8B)
Para utilizar la inteligencia artificial de Llama, instala el modelo:
- [Descarga en Ollama](https://ollama.com/download)
- Comando de instalación:
  ```bash
  ollama pull llama3.1:latest
  ```

--- 

¡Explora, experimenta y disfruta mejorando la asistencia quirúrgica con **S.I.L.V.I.A.**! 👩‍⚕️🤖
