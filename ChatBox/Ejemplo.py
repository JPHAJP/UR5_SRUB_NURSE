import ollama

messages = [
    {
        "role": "user",
        "content": "Que es un UR5 y como funciona?"}]

stream = ollama.chat(model='llama3.1:latest', messages=messages, stream=True)

for chunk in stream:
    print(chunk['message']['content'], end='', flush=True)