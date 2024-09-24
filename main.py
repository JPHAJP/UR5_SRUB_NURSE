import time
import queue
from activacion_voz import ActivacionVoz

if __name__ == "__main__":
    # Crear una cola para pasar datos entre hilos
    command_queue = queue.Queue()

    # Crear una instancia de la clase ActivacionVoz y pasarle la cola
    activador = ActivacionVoz(command_queue)

    # Iniciar el hilo que corre la escucha
    activador.iniciar_hilo()

    # Simular una tarea en el hilo principal (contador que imprime)
    contador = 0
    try:
        while True:
            # Verificar si hay comandos en la cola
            if not command_queue.empty():
                comando = command_queue.get()
                print(f"Comando recibido en el hilo principal: {comando}")

            # Continuar con otra tarea en el hilo principal
            print(f"Contador en hilo principal: {contador}")
            contador += 1
            time.sleep(1)  # Pausa de 1 segundo entre cada iteración
    except KeyboardInterrupt:
        print("Deteniendo...")
        activador.detener()
