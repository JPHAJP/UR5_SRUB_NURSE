# Instalar la librería TTS
# En tu terminal o entorno de desarrollo, primero instala la librería con:
# pip install TTS

from TTS.api import TTS

# Cargar el modelo preentrenado en español
tts = TTS(model_name="tts_models/es/css10/spanish", progress_bar=False)

# Texto a convertir en audio
texto = "Hola, este es un ejemplo de síntesis de voz con Mozilla TTS."

# Convertir el texto a archivo de audio
tts.tts_to_file(text=texto, file_path="output.wav")

print("Conversión completa. El archivo de audio se ha guardado como 'output.wav'.")
