from text_to_speech import TextToSpeech
from openai import OpenAI
import gradio as gr
from io import BytesIO
from pathlib import Path

client = OpenAI()
tts = TextToSpeech()


def speech_to_text(audio):
    try:
        with open(audio, "rb") as audio_file:
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                response_format="text"
            )
        print(transcript)
        return transcript  # Asegúrate de acceder a la clave correcta del dict de respuesta
    except Exception as e:
        print(f"Error en la transcripción del audio: {e}")
        return None


def detect_command(text):
    try:
        completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": """
                Eres un robot asistente de un cirujano, el usuario te pedira un instrumento quirurgico y deberas indentificarlo contra la siguiente lista:

                Casa:0
                Bisturi:1
                Pinzas:2
                Tijeras curvas:3
                Tijeras rectas:4
                Cancelar:5
                 
                Debes responder en dos partes, la primera es la detección o recolección del intrumental y la parte dos es el comando a ejecutar.
                El comando "Casa" y "Cancelar" no son instrumentos, pero si comandos a ejecutar.
                
                """},
                {"role": "user", "content": "Pásame las pinzas rectas."},
                {"role": "system", "content": "Detectando las Tijeras rectas, voy por ellas. Ejecutando comando 4."},
                {"role": "user", "content": "Dame el bisturi."},
                {"role": "system", "content": "Claro! Detectando el bisturi, voy por el. Ejecutando comando 1."},
                {"role": "user", "content": "Ve a casa."},
                {"role": "system", "content": "De acuerdo, regresando a mi posición original. Ejecutando comando 0."},
                {"role": "user", "content": text},
            ]
        )
        return completion.choices[0].message.content
    except Exception as e:
        print(f"Error en la detección del comando: {e}")
        return ""


def multi_model(audio):
    text = speech_to_text(audio)
    if text is None:
        return None
    command = detect_command(text)
    # audio_path = text_to_speech(command)
    audio_path = tts.synthesise(text=command, output_path="speech.mp3")
    return command, audio_path


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
    demo.launch()
