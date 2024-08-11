import serial

class SerialCommunication:
    def __init__(self, port: str, baudrate: int):
        self.ser = serial.Serial(port, baudrate)
    
    def send_speed_to_arduino(self, speed: list):
        self.ser.write(f'{speed[0]:.2f}\n'.encode())
        self.ser.write(f'{speed[1]:.2f}\n'.encode())
        self.ser.flushInput()
        self.ser.flushOutput()
    
    def close(self):
        self.ser.close()
        