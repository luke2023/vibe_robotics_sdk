from viberobotics.controller.controller_base import SundayA1Controller
from viberobotics.sensor.imu_arduino import RemoteIMU
from viberobotics.motor.motor_controller import MotorController

class RealController(SundayA1Controller):
    def __init__(self, config):
        super().__init__(config)
        self.real_config = config.real_config
        
        self.sensor = RemoteIMU(self.real_config.imu_port.port, 
                                self.real_config.imu_port.baudrate)
        self.motor_controller = 
        