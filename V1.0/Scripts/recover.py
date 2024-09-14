import socket
import time

robot_ip = "192.168.1.1"  # Cambia esto por la IP de tu robot
robot_port = 30002

# URScript para intentar recuperar del "protective stop" y liberar frenos si es necesario
recovery_script = """
def recover_from_protective_stop():
    if (robotmode() == ROBOT_MODE_PROTECTIVE_STOP):
        textmsg("Intentando desactivar el protective stop...")
        protective_stop_recover()  # Intentar recuperación
        brake_release()  # Liberar los frenos si es necesario
        textmsg("Se ha enviado el comando de recuperación.")
    else:
        textmsg("El robot no está en protective stop.")
end
recover_from_protective_stop()
"""

def send_urscript(ip, port, script):
    try:
        # Crear un socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Conectarse al robot
        s.connect((ip, port))
        
        # Enviar el script URScript al robot
        s.sendall(script.encode('utf-8'))
        print("Script enviado al robot.")
        
        # Esperar un momento para que el robot procese el script
        time.sleep(2)
        
        # Cerrar la conexión
        s.close()
    except Exception as e:
        print(f"Error al enviar el script: {e}")

def recover_robot_if_needed():
    while True:
        try:
            # Enviar el script de recuperación cada cierto tiempo para monitorear el estado
            send_urscript(robot_ip, robot_port, recovery_script)
            
            # Esperar un poco antes de intentar de nuevo
            time.sleep(5)
        
        except Exception as e:
            print(f"Error en la recuperación automática: {e}")

if __name__ == "__main__":
    recover_robot_if_needed()  # Llamar al bucle de recuperación automática
