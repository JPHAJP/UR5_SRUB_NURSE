import ollama
import whisper
import gradio as gr
from gtts import gTTS
import os

# Función de transcripción de audio usando Whisper
def speech_to_text(audio):
    try:
        model = whisper.load_model('base')
        audio_data = whisper.load_audio(audio)
        audio_data = whisper.pad_or_trim(audio_data)
        mel = whisper.log_mel_spectrogram(audio_data).to(model.device)
        _, probs = model.detect_language(mel)
        print(f"Detected language: {max(probs, key=probs.get)}")
        result = whisper.decode(model, mel)
        return result.text  # Acceder al texto de la transcripción
    except Exception as e:
        print(f"Error en la transcripción del audio: {e}")
        return None

# Función para generar la respuesta con Llama
def detect_command_llama(text):
    try:
        messages = [
            {"role": "user", "content": text}
        ]

        # messages=[
        #         {"role": "system", "content": """
        #         Eres un robot asistente de un cirujano, el usuario te pedira un instrumento quirurgico y deberas indentificarlo contra la siguiente lista:

        #         Casa:0
        #         Bisturi:1
        #         Pinzas:2
        #         Tijeras curvas:3
        #         Tijeras rectas:4
        #         Cancelar:5
                 
        #         Debes responder en dos partes, la primera es la detección o recolección del intrumental y la parte dos es el comando a ejecutar.
        #         El comando "Casa" y "Cancelar" no son instrumentos, pero si comandos a ejecutar.
                
        #         """},
        #         {"role": "user", "content": "Pásame las pinzas rectas."},
        #         {"role": "system", "content": "Detectando las Tijeras rectas, voy por ellas. Ejecutando comando 4."},
        #         {"role": "user", "content": "Dame el bisturi."},
        #         {"role": "system", "content": "Claro! Detectando el bisturi, voy por el. Ejecutando comando 1."},
        #         {"role": "user", "content": "Ve a casa."},
        #         {"role": "system", "content": "De acuerdo, regresando a mi posición original. Ejecutando comando 0."},
        #         {"role": "user", "content": text},
        #     ]


        stream = ollama.chat(model='llama3.1:latest', messages=messages, stream=True)

        command_response = ""
        for chunk in stream:
            command_response += chunk['message']['content']
        return command_response
    except Exception as e:
        print(f"Error en la generación del comando: {e}")
        return None

# Función que convierte texto a audio usando gTTS
def text_to_speech(text, output_path="ChatBox/output.mp3"):
    try:
        tts = gTTS(text=text, lang='es')
        tts.save(output_path)
        return output_path
    except Exception as e:
        print(f"Error en TTS: {e}")
        return None

# Función que combina todas las partes: transcripción, comando y TTS
def multi_model(audio):
    # Transcribir el audio
    text = speech_to_text(audio)
    if text is None:
        return None, None

    # Detectar el comando usando Llama
    command = detect_command_llama(text)

    # Convertir el comando a audio usando TTS
    audio_path = text_to_speech(command)
    
    return command, audio_path

# Configuración de la interfaz en Gradio
demo = gr.Interface(
    fn=multi_model,
    inputs=gr.Audio(sources="microphone", type="filepath", label="""
        Lista de comandos:
        0 - Casa
        1 - Bisturi
        2 - Pinzas
        3 - Tijeras curvas
        4 - Tijeras rectas
        5 - Mano
        6 - Cancelar
        """),
    outputs=["text", gr.Audio(format="mp3", streaming=True, autoplay=True)],
    live=True,
)

if __name__ == "__main__":
    demo.launch(share=True)