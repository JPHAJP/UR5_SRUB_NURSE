import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
import whisper

def grabar_audio(duracion=5, frecuencia_muestreo=44100):
    print("Grabando...")
    grabacion = sd.rec(int(duracion * frecuencia_muestreo), samplerate=frecuencia_muestreo, channels=2)
    sd.wait()  #Espera hasta que termine la grabación
    print("Grabación finalizada")
    return grabacion

def guardar_audio(grabacion, frecuencia_muestreo=44100, archivo='output.wav'):
    write(archivo, frecuencia_muestreo, grabacion)  #Guarda como archivo WAV

def transcribir_audio(archivo='output.wav'):
    model = whisper.load_model("base")
    result = model.transcribe(archivo)
    return result["text"]

#Main script
if __name__ == "__main__":
    frecuencia_muestreo = 44100  #Calidad de CD
    archivo = "audio_temp.wav"
    
    #Grabar y guardar el audio
    audio = grabar_audio(duracion=5, frecuencia_muestreo=frecuencia_muestreo)
    guardar_audio(audio, frecuencia_muestreo, archivo)
    
    #Transcribir el audio
    texto = transcribir_audio(archivo)
    print("Texto transcrito:", texto)