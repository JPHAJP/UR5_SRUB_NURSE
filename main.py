import time
import queue
from alt import ActivacionVoz  # Import your class from the updated file

if __name__ == "__main__":
    # Create an instance of the class ActivacionVoz
    activacion_voz = ActivacionVoz()
    command_queue = queue.Queue()


    # Start the ActivacionVoz thread
    activacion_voz.start()

    # Simulate the main thread doing other tasks (e.g., a counter that prints)
    contador = 0
    try:
        while True:
            if not command_queue.empty():
                comando = command_queue.get()
                print(f"Comando recibido en el hilo principal: {comando}")
            # Simulate doing something else in the main thread
            print(f"Main thread counter: {contador}")
            contador += 1
            time.sleep(1)  # Sleep for 1 second between each iteration
    except KeyboardInterrupt:
        print("Stopping...")
        # Call the detener() method to stop the ActivacionVoz thread
        activacion_voz.detener()
        # Optionally, wait for the thread to finish before exiting
        activacion_voz.join()  # Wait for the thread to stop completely
        print("ActivacionVoz thread stopped.")
