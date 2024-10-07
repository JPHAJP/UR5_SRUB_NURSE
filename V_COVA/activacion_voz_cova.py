import whisper
import speech_recognition as sr
import os
import ollama
import re
from playsound import playsound
from gtts import gTTS
from pydub import AudioSegment
import threading
import queue

class ActivacionVoz:
    def __init__(self, command_queue):
        # Cargar el modelo de Whisper
        self.model = whisper.load_model("base")

        # Crear el reconocedor de voz
        self.recognizer = sr.Recognizer()

        # Palabra clave que activará la respuesta
        self.palabra_activacion = "silvia"

        # Inicializar el micrófono
        self.mic = sr.Microphone()

        # Cola para enviar comandos al hilo principal
        self.command_queue = command_queue

        # Variable para controlar el ciclo de comandos
        self.escuchando = True

    def reproducir_audio(self, mensaje):
        mensaje = "aaa.........." + mensaje
        tts = gTTS(mensaje, lang='es')
        archivo_audio = "esperando_comando.mp3"
        tts.save(archivo_audio)
        audio = AudioSegment.from_file(archivo_audio)
        velocidad_deseada = 1.2
        audio_rapido = audio.speedup(playback_speed=velocidad_deseada)
        audio_rapido.export(archivo_audio, format="mp3")
        playsound(archivo_audio)
        os.remove(archivo_audio)

    def escuchar(self):
        with self.mic as source:
            print("Escuchando...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio = self.recognizer.listen(source, phrase_time_limit=2)
            return audio

    def transcribir_con_whisper(self, audio):
        with open("temp_audio.wav", "wb") as f:
            f.write(audio.get_wav_data())
        
        result = self.model.transcribe("temp_audio.wav", language="es")
        os.remove("temp_audio.wav")
        return result["text"]

    def detectar_palabra_clave(self, audio):
        try:
            texto = self.transcribir_con_whisper(audio)
            print(f"Has dicho: {texto}")
            if self.palabra_activacion.lower() in texto.lower():
                print("¡Palabra de activación detectada!")
                self.reproducir_audio("Esperando comando")
                return True
        except Exception as e:
            print(f"Error al transcribir el audio: {e}")
        return False

    def detectar_comando_llama(self, text):
        try:
            messages = [
            {"role": "system", "content": """
            Eres un robot asistente de un cirujano. 
            Tu nombre es "SILVIA" - (Surgical Instrument Logistics Virtual Intelligent Assistant); eres un UR5e que ayuda a los cirujanos en el quirófano.
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
            Si ya diste un commando no es necesario que continues con más texto, solo espera la siguiente instrucción; si no recibes instrucciones, puedes preguntar "¿En qué puedo asistirte?".
            
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
            return command_response, self.extract_command_number(command_response)
        except Exception as e:
            print(f"Error en la generación del comando: {e}")
            return None

    def extract_command_number(self, response):
        match = re.search(r"ejecutando comando\s*(\d+)", response.lower())
        if match:
            self.command_queue.put(int(match.group(1)))
            return int(match.group(1))
        return None

    def ciclo_de_comandos(self):
        while self.escuchando:
            print("Esperando la palabra clave...")
            audio = self.escuchar()
            if self.detectar_palabra_clave(audio):
                while True:
                    print("Esperando instrucciones...")
                    audio = self.escuchar()
                    texto = self.transcribir_con_whisper(audio)
                    print(f"Instrucción recibida: {texto}")
                    respuesta, comando = self.detectar_comando_llama(texto)
                    if respuesta:
                        print(f"Comando ejecutado: {comando}")
                        self.reproducir_audio(respuesta)
                        break
                    else:
                        print("No se pudo ejecutar el comando.")

    def iniciar_hilo(self):
        self.hilo = threading.Thread(target=self.ciclo_de_comandos)
        self.hilo.daemon = True
        self.hilo.start()

    def detener(self):
        self.escuchando = False
        if self.hilo.is_alive():
            self.hilo.join()
        else:
            print("El hilo de escucha ya ha terminado.")

###################################################### usando google ############################################
# import speech_recognition as sr
# from gtts import gTTS
# from pydub import AudioSegment
# from pydub.playback import play
# import io
# import json
# import os
# import sys

# class SilviaAssistant:
#     def __init__(self, command_file='comandos.json'):
#         """Inicializa el asistente, carga los comandos desde el archivo JSON."""
#         self.command_file = command_file
#         self.commands = self.load_commands()
#         sys.stderr = open(os.devnull, 'w')  # Silenciar mensajes de error de ALSA

#     def load_commands(self):
#         """Carga los comandos desde el archivo JSON especificado."""
#         try:
#             with open(self.command_file, 'r') as file:
#                 return json.load(file)
#         except FileNotFoundError:
#             print("El archivo de comandos no se encontró.")
#             return {}

#     def silvia_speak(self, text):
#         """Convierte texto en voz utilizando gTTS y reproduce el audio."""
#         print(f"Silvia dice: {text}")  # Mostrar el texto en la terminal
#         tts = gTTS(text=text, lang='es')
#         audio = io.BytesIO()
#         tts.write_to_fp(audio)
#         audio.seek(0)
#         song = AudioSegment.from_file(audio, format="mp3")
#         play(song)

#     def listen_for_commands(self):
#         """Escucha y reconoce comandos de voz."""
#         recognizer = sr.Recognizer()
#         recognizer.energy_threshold = 300  # Ajustar según el entorno

#         with sr.Microphone() as source:
#             print("Ajustando al ruido ambiental, espera un momento...")
#             recognizer.adjust_for_ambient_noise(source)  # Ajuste de ruido ambiental
#             print("Escuchando...")
#             audio = recognizer.listen(source)
#             try:
#                 print("Procesando el audio...")
#                 command = recognizer.recognize_google(audio, language="es-ES")
#                 print(f"Comando escuchado: {command}")
#                 return command.lower()
#             except sr.UnknownValueError:
#                 print("No se pudo entender el audio")
#                 return ""
#             except sr.RequestError as e:
#                 print(f"Error al conectar con el servicio de reconocimiento de voz: {e}")
#                 return ""

#     def process_command(self, command):
#         """Procesa el comando recibido."""
#         if "silvia" in command:
#             # Extraer el comando después de mencionar "Silvia"
#             action = command.replace("silvia", "").strip()
#             if action:
#                 # Busca si el comando está en el JSON
#                 if action in self.commands:
#                     print(f"Ejecutando comando: {action}")
#                     self.silvia_speak(self.commands[action])
#                 else:
#                     self.silvia_speak("Lo siento, no sé cómo hacer eso.")
#             else:
#                 # Si solo se menciona "Silvia" sin un comando
#                 self.silvia_speak("Dime qué quieres que haga.")
#         else:
#             print("No se mencionó a 'Silvia'.")

#     def run(self):
#         """Inicia el asistente en un bucle para escuchar comandos de forma continua."""
#         if not self.commands:
#             self.silvia_speak("Lo siento, no hay comandos disponibles en este momento.")
#             return

#         while True:
#             print("Esperando un comando...")
#             command = self.listen_for_commands()
#             self.process_command(command)

# # Si se ejecuta directamente el archivo, se inicia el asistente
# if __name__ == "__main__":
#     assistant = SilviaAssistant()
#     assistant.run()
##############################################################################################################



#################################### usando vosk ############################################################
# import os
# import sys
# import json
# import pyttsx3
# import wave
# import vosk
# import pyaudio

# class SilviaAssistantOffline:
#     def __init__(self, command_file='comandos.json', vosk_model_path='vosk-model-es-0.42'):
#         """Inicializa el asistente, carga los comandos desde el archivo JSON y el modelo Vosk."""
#         self.command_file = command_file
#         self.commands = self.load_commands()
#         self.model = vosk.Model(vosk_model_path)  # Carga el modelo de Vosk
#         self.engine = pyttsx3.init()
#         self.engine.setProperty('rate', 150)  # Velocidad del habla
#         sys.stderr = open(os.devnull, 'w')  # Silenciar mensajes de error de ALSA

#     def load_commands(self):
#         """Carga los comandos desde el archivo JSON especificado."""
#         try:
#             with open(self.command_file, 'r') as file:
#                 return json.load(file)
#         except FileNotFoundError:
#             print("El archivo de comandos no se encontró.")
#             return {}

#     def silvia_speak(self, text):
#         """Convierte texto en voz utilizando pyttsx3."""
#         print(f"Silvia dice: {text}")  # Mostrar el texto en la terminal
#         self.engine.say(text)
#         self.engine.runAndWait()

#     def listen_for_commands(self):
#         """Escucha y reconoce comandos de voz usando Vosk (sin conexión)."""
#         recognizer = pyaudio.PyAudio()
#         stream = recognizer.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
#         stream.start_stream()

#         print("Escuchando...")

#         rec = vosk.KaldiRecognizer(self.model, 16000)
#         while True:
#             data = stream.read(4096)
#             if len(data) == 0:
#                 break
#             if rec.AcceptWaveform(data):
#                 result = rec.Result()
#                 text = json.loads(result)['text']
#                 print(f"Comando escuchado: {text}")
#                 return text.lower()

#     def process_command(self, command):
#         """Procesa el comando recibido."""
#         if "silvia" in command:
#             # Extraer el comando después de mencionar "Silvia"
#             action = command.replace("silvia", "").strip()
#             if action:
#                 # Busca si el comando está en el JSON
#                 if action in self.commands:
#                     print(f"Ejecutando comando: {action}")
#                     self.silvia_speak(self.commands[action])
#                 else:
#                     self.silvia_speak("Lo siento, no sé cómo hacer eso.")
#             else:
#                 # Si solo se menciona "Silvia" sin un comando
#                 self.silvia_speak("Dime qué quieres que haga.")
#         else:
#             print("No se mencionó a 'Silvia'.")

#     def run(self):
#         """Inicia el asistente en un bucle para escuchar comandos de forma continua."""
#         if not self.commands:
#             self.silvia_speak("Lo siento, no hay comandos disponibles en este momento.")
#             return

#         while True:
#             print("Esperando un comando...")
#             command = self.listen_for_commands()
#             self.process_command(command)


# # Si se ejecuta directamente el archivo, se inicia el asistente
# if __name__ == "__main__":
#     assistant = SilviaAssistantOffline(vosk_model_path='vosk-model-es-0.42')  # Ruta al modelo de Vosk
#     assistant.run()
##############################################################################################################


#################################### usando whisper ########################################################
# import os
# import sys
# import json
# import pyttsx3
# import whisper
# import subprocess
# import time

# class SilviaAssistantWhisper:
#     def __init__(self, command_file='comandos.json', whisper_model_name='base'):
#         """Inicializa el asistente, carga los comandos desde el archivo JSON y Whisper."""
#         self.command_file = command_file
#         self.commands = self.load_commands()
#         self.model = whisper.load_model(whisper_model_name)  # Cargar el modelo de Whisper
#         self.engine = pyttsx3.init()
#         self.set_voice_parameters()  # Ajustar los parámetros de voz
#         sys.stderr = open(os.devnull, 'w')  # Silenciar mensajes de error de ALSA

#     def set_voice_parameters(self):
#         """Ajusta los parámetros de la voz de Silvia."""
#         voices = self.engine.getProperty('voices')
#         # Seleccionar una voz femenina si está disponible
#         for voice in voices:
#             if 'spanish' in voice.languages:  # Cambiar a voz en español
#                 self.engine.setProperty('voice', voice.id)
#                 break
#         # Ajustar la velocidad y el tono para que suene más natural
#         self.engine.setProperty('rate', 135)  # Ajustar velocidad (135 es más natural que 150)
#         self.engine.setProperty('volume', 1)  # Volumen máximo

#     def load_commands(self):
#         """Carga los comandos desde el archivo JSON especificado."""
#         try:
#             with open(self.command_file, 'r') as file:
#                 return json.load(file)
#         except FileNotFoundError:
#             print("El archivo de comandos no se encontró.")
#             return {}

#     def silvia_speak(self, text):
#         """Convierte texto en voz utilizando pyttsx3."""
#         print(f"Silvia dice: {text}")  # Mostrar el texto en la terminal
#         self.engine.say(text)  # Convertir el texto en voz
#         self.engine.runAndWait()  # Esperar a que termine de hablar

#     def listen_for_commands(self):
#         """Graba y transcribe comandos de voz usando Whisper (en español, sin conexión)."""
#         print("Escuchando...")

#         # Ajustar la grabación para mejorar la calidad
#         subprocess.run(["arecord", "-d", "5", "-f", "S16_LE", "-r", "44100", "-c", "1", "command.wav"])

#         # Transcribir el audio usando Whisper y forzar el idioma español
#         result = self.model.transcribe("command.wav", language="es")
#         text = result['text'].lower().strip()  # Convertir a minúsculas y eliminar espacios
#         print(f"Comando escuchado: {text}")
#         return text

#     def process_command(self, command):
#         """Procesa el comando recibido."""
#         if "silvia" in command:
#             # Extraer el comando después de mencionar "Silvia"
#             action = command.replace("silvia", "").strip()

#             # Esperar 3 segundos para ver si el usuario da un comando adicional
#             if not action:
#                 print("Esperando un comando adicional...")
#                 time.sleep(3)  # Esperar 3 segundos
#                 action = self.listen_for_commands().replace("silvia", "").strip()

#                 if not action:
#                     print("No se detectó ningún comando adicional.")
#                     return  # No hace nada si no hay comando

#             # Si hay un comando después de "Silvia"
#             if action:
#                 if action in self.commands:
#                     print(f"Ejecutando comando: {action}")
#                     self.silvia_speak(self.commands[action])  # Silvia responde con la acción
#                 else:
#                     self.silvia_speak("Lo siento, no sé cómo hacer eso.")
#         else:
#             print("No se mencionó a 'Silvia'.")

#     def run(self):
#         """Inicia el asistente en un bucle para escuchar comandos de forma continua."""
#         if not self.commands:
#             self.silvia_speak("Lo siento, no hay comandos disponibles en este momento.")
#             return

#         while True:
#             print("Esperando un comando...")
#             command = self.listen_for_commands()
#             self.process_command(command)


# # Si se ejecuta directamente el archivo, se inicia el asistente
# if __name__ == "__main__":
#     assistant = SilviaAssistantWhisper(whisper_model_name='base')  # Usar modelo base de Whisper
#     assistant.run()
##############################################################################################################