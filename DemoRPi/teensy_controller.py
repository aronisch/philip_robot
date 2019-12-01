import serial

class TeensyController:
    def __init__(self, serial_port, baud_rate):
        self.ser = serial.Serial(serial_port, baud_rate)
    
    def __del__(self):
        self.ser.close()
        
    def set_velocities(self, linear_vel, angular_vel):
        vel_packet = "L" + linear_vel + "A" + angular_vel + "\n"
        self.ser.write(vel_packet)
        
    def close_plate_holder(self):
        self.ser.write("P0\n")
    
    def open_plate_holder(self):
        self.ser.write("P1\n")
    
    def close_gripper(self):
        self.ser.write("G0\n")
    
    def open_gripper(self):
        self.ser.write("G1\n")
        
    def set_gripper_grabbing_position(self):
        self.ser.write("R0\n")
        
    def set_gripper_releasing_position(self):
        self.ser.write("R1\n")
        
    def obstacle_present(self):
        self.ser.write("O\n")
        return self.read() == "T"
    