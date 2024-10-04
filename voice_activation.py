from activacion_voz import ActivacionVoz

class VoiceActivation:
    def __init__(self, command_queue):
        """
        Inicializa el sistema de activación por voz.
        
        Parámetros:
        - command_queue: Cola para enviar los comandos detectados.
        """
        self.activacion = ActivacionVoz(command_queue)
        print("Sistema de activación por voz inicializado.")

    def listen(self):
        """
        Inicia el ciclo de escucha para detectar comandos de voz.
        """
        self.activacion.ciclo_de_comandos()

    def add_keyword(self, keyword):
        """
        Agrega una nueva palabra clave a la lista de comandos.
        
        Parámetros:
        - keyword: Palabra clave para agregar.
        """
        self.activacion.palabra_activacion = keyword
        print(f"Palabra clave '{keyword}' agregada.")

    def remove_keyword(self, keyword):
        """
        Elimina una palabra clave de la lista de comandos.
        
        Parámetros:
        - keyword: Palabra clave para eliminar.
        """
        if self.activacion.palabra_activacion == keyword:
            self.activacion.palabra_activacion = "silvia"
            print(f"Palabra clave '{keyword}' eliminada, regresando a 'silvia'.")
