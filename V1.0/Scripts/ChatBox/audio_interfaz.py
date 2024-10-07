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
        print(result.text)  # Acceder al texto de la transcripción
        return result.text  # Acceder al texto de la transcripción
    except Exception as e:
        print(f"Error en la transcripción del audio: {e}")
        return None

# Función para detección de palabra clave
def detect_keyword(text, keyword="ur5"):
    return keyword.lower() in text.lower()


# Función principal que siempre escucha
def continuous_listen(audio):
    # Transcribir el audio
    text = speech_to_text(audio)
    
    if text is None:
        return "No se pudo transcribir el audio", None, None

    # Detectar la palabra clave
    if detect_keyword(text, keyword="activar"):
        # Si se detecta la palabra clave, cambiar a modo de espera
        response = "Esperando instrucciones"
        audio_path = text_to_speech(response)
        return text, response, audio_path
    else:
        # Si no detecta la palabra clave, buscar comandos quirúrgicos
        response, command = detect_command_llama(text)
        if response is not None:
            audio_path = text_to_speech(response)
            return text, response, audio_path
        else:
            return text, "No se detectaron comandos quirúrgicos", None

# Función para generar la respuesta con Llama
def detect_command_llama(text):
    try:
        messages = [
            {"role": "system", "content": """r
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
    # Suponiendo que la respuesta está en el formato esperado
    for line in response.split('\n'):
        if "ejecutando comando" in line:
            parts = line.split("ejecutando comando")
            if len(parts) > 1:
                command_part = parts[1].strip()
                # Manejar posibles variaciones
                if command_part.isdigit():
                    return command_part
                else:
                    return command_part.split()[0]
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
    response, command  = detect_command_llama(text)
    print(f"Comando ejecutado: {command}")
    # Convertir el comando a audio usando TTS
    audio_path = text_to_speech(response)
    
    return text, response, audio_path

# Configuración de la interfaz en Gradio
demo = gr.Interface(
    fn=continuous_listen,
    inputs=gr.Audio(sources="microphone", type="filepath", label="""
        Lista de comandos:
        0 - Casa
        1 - Bisturí
        2 - Pinzas
        3 - Tijeras curvas
        4 - Tijeras rectas
        5 - Mano
        6 - Cancelar
        """),
    outputs=["text", "text", gr.Audio(format="mp3", streaming=True, autoplay=True)],
    live=True,
)

if __name__ == "__main__":
    demo.launch(share=False)