import whisper
import speech_recognition as sr
import os
import ollama
import re
from playsound import playsound
from gtts import gTTS
from pydub import AudioSegment

# Cargar el modelo de Whisper
model = whisper.load_model("base")

# Crear el reconocedor de voz
recognizer = sr.Recognizer()

# Palabra clave que activará la respuesta
palabra_activacion = "ur5"

# Inicializar el micrófono solo una vez para mejorar el rendimiento
mic = sr.Microphone()

# Inicializar el motor de síntesis de voz de pyttsx3
# Cargar el modelo de TTS local de Coqui TTS
#tts = TTS(model_name="tts_models/es/mai/tacotron2-DDC", progress_bar=False, gpu=False)

# Función para generar y reproducir el audio con TTS
# def reproducir_audio(mensaje):
#     archivo_audio = "esperando_comando.wav"
#     tts.tts_to_file(text=mensaje, file_path=archivo_audio)
#     os.system(f"aplay {archivo_audio}")  # En Linux puedes usar 'aplay'; en Windows usa 'start'
#     os.remove(archivo_audio)

def reproducir_audio(mensaje):
    tts = gTTS(mensaje, lang='es')
    archivo_audio = "esperando_comando.mp3"
    tts.save(archivo_audio)
    audio = AudioSegment.from_file(archivo_audio)
    velocidad_deseada = 1.2
    audio_rapido = audio.speedup(playback_speed=velocidad_deseada)
    audio_rapido.export(archivo_audio, format="mp3")
    playsound(archivo_audio)
    os.remove(archivo_audio)

def escuchar():
    with mic as source:
        print("Escuchando...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        audio = recognizer.listen(source, phrase_time_limit=2)
        return audio

def transcribir_con_whisper(audio):
    with open("temp_audio.wav", "wb") as f:
        f.write(audio.get_wav_data())
    
    result = model.transcribe("temp_audio.wav", language="es")
    os.remove("temp_audio.wav")
    return result["text"]

def detectar_palabra_clave(audio):
    try:
        texto = transcribir_con_whisper(audio)
        print(f"Has dicho: {texto}")
        if palabra_activacion.lower() in texto.lower():
            print("¡Palabra de activación detectada!")
            reproducir_audio("Esperando comando")
            return True
    except Exception as e:
        print(f"Error al transcribir el audio: {e}")
    return False

def detectar_comando_llama(text):
    try:
        messages = [
            {"role": "system", "content": """
            Eres un robot asistente de un cirujano. 
            Tu nombre es "UR5 Scrube Nurse".
            El usuario puede pedirte instrumentos quirúrgicos específicos.
            Puedes ayudar con tareas simples de manejo de instrumental quirúrgico, como un enfermero instrumentista.
            Tu tarea es identificar el instrumento solicitado de la siguiente lista:
            
            Lista de comandos:
            Casa: 0
            Bisturí: 1
            Pinzas: 2
            Tijeras curvas: 3
            Tijeras rectas: 4
            Mano: 5
            Cancelar: 6
             
            Debes responder en el formato: "Detectando [instrumento], ejecutando comando [número]."
            Si escuchas visturí o bisturí, debes responder con bisturí.
            El comando "Casa" y "Cancelar" no son instrumentos, pero si comandos a ejecutar; si se te pide uno de estos comandos, responde con que robot regresa a su posición de casa commando [Casa: 0], o que se a cancelado cualquier otro commando.
            Si no puedes ayudar, trata de dar una respuesta más adecuada.
            Si no tienes el instrumental puedes responder: "Parece que no tengo esa herramienta en mi inventario. Intenta otra vez."
            Las tijeras curvas y rectas son diferentes, asegúrate de identificarlas correctamente; tabien se les conoce como tijeras de Metzenbaum y Mayo (Tijeras Mayo Curvas o Rectas).
            Debes responder de manera clara y concisa, se requiere una respuesta corta y precisa para ejecutar los commandos rápido.
            Si ya diste un commando no es necesario que continues con más texto, solo espera la siguiente instrucción.
            
            Muy importante: Siempre debes responder con un solo comando ejecutado por respuesta, si no puedes ayudar, responde con un mensaje adecuado.
            """},
            {"role": "user", "content": "¿Qué instrumentos tienes?"},
            {"role": "system", "content": "Tengo los siguientes instrumentos: Bisturí, Pinzas, Tijeras curvas, Tijeras rectas, Mano."},
            {"role": "user", "content": "Las tijeras por favor."},
            {"role": "system", "content": "¿Tijeras curvas o rectas?"},
            {"role": "user", "content": "Pásame las pinzas rectas."},
            {"role": "system", "content": "Detectando las Tijeras rectas, voy por ellas. Ejecutando comando 4."},
            {"role": "user", "content": "Dame el bisturí."},
            {"role": "system", "content": "Claro! Detectando el bisturi, voy por el. Ejecutando comando 1."},
            {"role": "user", "content": "Ve a casa."},
            {"role": "system", "content": "De acuerdo, regresando a mi posición original. Ejecutando comando 0."},
            {"role": "user", "content": "Busca las tijeras."},
            {"role": "system", "content": "Nececito más información, ¿tijeras curvas o rectas?"},
            {"role": "user", "content": "   "},
            {"role": "system", "content": "Estoy para ayudarte, ¿en qué puedo asistirte?"},
            {"role": "user", "content": "-----"},
            {"role": "system", "content": "No recibí instrucciones, estoy para ayudarte, ¿en qué puedo asistirte?"},
            {"role": "user", "content": "Detén el proceso."},
            {"role": "system", "content": "Proceso detenido. Ejecutando comando 6."},
            {"role": "user", "content": "Cancela"},
            {"role": "system", "content": "Proceso cancelado. Ejecutando comando 6."},
            {"role": "user", "content": text}
        ]

        stream = ollama.chat(model='llama3.1:latest', messages=messages, stream=True)

        command_response = ""
        for chunk in stream:
            command_response += chunk['message']['content']
        print(command_response)
        return command_response, extract_command_number(command_response)
    except Exception as e:
        print(f"Error en la generación del comando: {e}")
        return None

def extract_command_number(response):
    # Usamos una expresión regular para buscar el patrón "ejecutando comando" seguido de un número
    match = re.search(r"ejecutando comando\s*(\d+)", response.lower())
    if match:
        return int(match.group(1))  # Devolver el número del comando encontrado
    return None


def ciclo_de_comandos():
    while True:
        # Primero escuchamos la palabra clave
        print("Esperando la palabra clave...")
        audio = escuchar()
        if detectar_palabra_clave(audio):
            while True:
                # Ahora esperamos instrucciones después de la palabra clave
                print("Esperando instrucciones...")
                audio = escuchar()
                texto = transcribir_con_whisper(audio)
                print(f"Instrucción recibida: {texto}")
                respuesta, comando = detectar_comando_llama(texto)
                if respuesta:
                    print(f"Comando ejecutado: {comando}")
                    reproducir_audio(respuesta)
                    break  # Después de ejecutar un comando, volvemos a escuchar la palabra clave
                else:
                    print("No se pudo ejecutar el comando.")
            # Después de ejecutar un comando, el sistema vuelve a pedir la palabra clave

if __name__ == "__main__":
    ciclo_de_comandos()
