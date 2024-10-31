# UR5_SRUB_NURSE -- S.I.L.V.I.A. - Surgical Instrument Logistics Virtual Intelligent Assistant
Repositorio para trabajo de robot UR5 con camara de profundidad para detectar objetos de instrumental quirurjico con IA.

-- QUICK CHANGE TOOL SYSTEM -- 
Adapted by JPHA - 
-Los archivos principales para el cambio de herramental son:
    -Base
    -Cople
    -Pestillo
*Pronto estarán disponibles los planos, por el momento usar archivos STEP en carpeta 3D_Models

Librerias de trabajo:
-Mediapipe
-Ultralytics
-pyrealsense2
-numpy
-cv2
-ur_rtde
-ollama
-whisper
-gradio
-gtts
-flask

-UR en docker
https://hub.docker.com/r/universalrobots/ursim_e-series

-Libreria UR_rtde
https://sdurobotics.gitlab.io/ur_rtde/index.html

-Whisper
https://github.com/openai/whisper

-llama3.1 8B
https://ollama.com/download
    ollama pull llama3.1:latest


-sudo apt-get install portaudio19-dev
