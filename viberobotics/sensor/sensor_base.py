class IMU:
    def __init__(self):
        pass

    def get_quaternion(self):
        raise NotImplementedError
    
    def get_gyro(self): 
        raise NotImplementedError
    
    def get_acc(self):
        raise NotImplementedError