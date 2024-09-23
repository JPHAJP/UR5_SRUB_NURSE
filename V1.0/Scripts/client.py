import rtde_control
import rtde_receive


receive = rtde_receive.RTDEReceiveInterface("192.168.1.1")
control = rtde_control.RTDEControlInterface("192.168.1.1")

control.moveL([.309,-.277,.373,0,3.14,0],0.5,0.5)

print(receive.getActualTCPPose())