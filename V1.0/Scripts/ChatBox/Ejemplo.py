import ollama
import whisper

def speech_to_text(audio):
    try:
        model = whisper.load_model('base')
        audio = whisper.load_audio(audio)
        audio = whisper.pad_or_trim(audio)
        mel = whisper.log_mel_spectrogram(audio).to(model.device)
        _, probs = model.detect_language(mel)
        print(f"Detected language: {max(probs, key=probs.get)}")
        result = whisper.decode(model, mel)
        return result.text  # Asegúrate de acceder a la clave correcta del dict de respuesta
    
    except Exception as e:
        print(e)


def detect_command_llama(text):
    messages = [
        {
            "role": "user",
            "content": "Que es un UR5 y como funciona?"}]

    stream = ollama.chat(model='llama3.1:latest', messages=messages, stream=True)

    for chunk in stream:
        print(chunk['message']['content'], end='', flush=True)
    print('')

text = speech_to_text('ChatBox/audio.mp3')
print(text)
