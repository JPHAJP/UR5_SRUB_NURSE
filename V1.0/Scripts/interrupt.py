import rtde_control
import rtde_receive
import time
import threading

# Set up communication with the robot
rtde_c = rtde_control.RTDEControlInterface("192.168.1.1")  # Replace "ROBOT_IP" with the IP of your UR5
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.1")

# Define a linear movement target (pose is in meters and radians)
target_pose = [0.5, 0.2, 0.3, 0, 3.14, 0]

# A flag to track if the stop signal has been issued
stop_signal_received = False

# Function to monitor digital input and stop the robot if triggered
def stop_robot_on_input():
    global stop_signal_received
    while rtde_c.isProgramRunning():
        digital_input_state = rtde_r.getDigitalInState(0)  # Correct method name
        if digital_input_state:
            print("Input triggered, stopping robot...")
            stop_signal_received = True
            # Use stopJ to stop the robot immediately
            rtde_c.stopJ(10)  # Deceleration parameter (adjust as needed)
            break
        # More frequent checking for faster response

# Start a separate thread to monitor the digital input
stop_thread = threading.Thread(target=stop_robot_on_input)
stop_thread.start()

# Send the movel command in non-blocking mode
print("Sending movel command...")
rtde_c.moveL(target_pose, 0.25, 1.2, asynchronous=True)  # Non-blocking motion

# Continuously check if the robot is still moving or if the stop signal is received
while rtde_c.isProgramRunning() and not stop_signal_received:
    time.sleep(0.05)

# Ensure the stop thread has completed its task
stop_thread.join()

# Close connection to the robot
rtde_c.disconnect()
rtde_r.disconnect()

print("Program finished")