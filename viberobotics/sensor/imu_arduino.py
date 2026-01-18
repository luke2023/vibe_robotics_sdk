from viberobotics.utils.math import *
from viberobotics.sensor.bno055 import BNO055
from viberobotics.sensor.sensor_base import IMU

import numpy as np
import serial
import threading
from scipy.spatial.transform import Rotation as R

class RemoteIMU(IMU):
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])
    
        self.arduino = serial.Serial(self.port, self.baudrate, timeout=.1)
        self.bno055 = BNO055()

        self.process = threading.Thread(target=self.fetch_loop)
        self.process.start()
        self.inv_quat = np.array([1.0, 0.0, 0.0, 0.0])

    def zero(self):
        self.inv_quat = quat_inv(self.quaternion)
        
    def fetch_loop(self):
        while True:
            try:
                line = self.arduino.readline().decode('utf-8').rstrip()
                if line:
                    data = line.split(',')
                    if len(data) == 11:
                        gyro = np.array([float(data[0]), float(data[1]), float(data[2])])
                        quat = np.array([float(data[3]), float(data[4]), float(data[5]), float(data[6])])
                        acc = np.array([float(data[7]), float(data[8]), float(data[9])])
                        self.quaternion, self.gyro, self.acc = self.bno055.update(quat, gyro, acc)
                        
            except Exception as e:
                # print(f"Error reading from IMU: {e}")
                pass
    
    def get_quaternion(self):
        return quat_mult(self.inv_quat, self.quaternion)

    def get_gyro(self):
        return quat_mul_vec(self.inv_quat, self.gyro)

    def get_acc(self):
        return self.acc.copy()