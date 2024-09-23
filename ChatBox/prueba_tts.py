from TTS.api import TTS

# Ruta al archivo de checkpoint del modelo
model_checkpoint_path = "/home/jpha/.local/share/tts/tts_models--es--mai--tacotron2-DDC/model_file.pth"

# Ruta al archivo de configuración del modelo
model_config_path = "/home/jpha/.local/share/tts/tts_models--es--mai--tacotron2-DDC/config.json"

# Cargar el modelo usando ambas rutas
tts = TTS(model_path=model_checkpoint_path, config_path=model_config_path, progress_bar=False, gpu=False)

# Forzar configuración: El modelo no es multilingüe
tts.is_multi_lingual = False  # Añadimos esta línea

# Texto de prueba para síntesis
texto = "Hola, este es un ejemplo de síntesis de voz."

# Generar y guardar el archivo de audio
tts.tts_to_file(text=texto, file_path="output_example.wav")
