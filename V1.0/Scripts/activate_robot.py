import rtde_control
import time

def inicializar_robot():
    try:
        ip = "192.168.1.1"  # Replace with your robot's IP address
        # Create the RTDE control interface
        control = rtde_control.RTDEControlInterface(ip)
        
        # Release the robot brakes
        control.unlockProtectionStop()
        print("Brakes released and robot initialized.")
        
        return control
    except Exception as e:
        print(f"Error al inicializar el robot: {e}")
        return None

# Initialize the robot and release the brakes
control = inicializar_robot()

if control is not None:
    print("Robot is ready for operation.")
else:
    print("Failed to initialize robot.")
