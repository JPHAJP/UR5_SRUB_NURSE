# UR5_SRUB_NURSE
Repositorio para trabajo de robot UR5 con camara de profundidad para detectar objetos de instrumental quirurjico con IA.


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

-UR en docker
https://hub.docker.com/r/universalrobots/ursim_e-series

-Libreria UR_rtde
https://sdurobotics.gitlab.io/ur_rtde/index.html

-Whisper
https://github.com/openai/whisper

-llama3.1 8B
https://ollama.com/download
    ollama pull llama3.1:latest



bool teachMode()
Set robot in freedrive mode.

In this mode the robot can be moved around by hand in the same way as by pressing the “freedrive” button. The robot will not be able to follow a trajectory (eg. a movej) in this mode.

bool endTeachMode()
Set robot back in normal position control mode after freedrive mode.