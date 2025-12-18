from viberobotics.motor.ftservo_python_sdk.scservo_sdk.port_handler import PortHandler
from viberobotics.motor.ftservo_python_sdk.scservo_sdk import sms_sts
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.group_sync_read import GroupSyncRead
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.group_sync_write import GroupSyncWrite
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import *
from viberobotics.utils.math import step2rad
from viberobotics.exceptions.motor import *

import numpy as np

class MotorController:
    def ___init__(self, motor_ids, port_name):
        self.motor_ids = motor_ids
        self.port_name = port_name
    
        self._init_port_handler()
        self.packetHandler = sms_sts(self.portHandler)
        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
        self.groupSyncRead_Current = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_CURRENT_L, 2)
        self.groupSyncRead_Load = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_LOAD_L, 2)
    
    def _init_port_handler(self):
        self.portHandler = PortHandler(self.port_name)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate 1000000
        if self.portHandler.setBaudRate(1000000):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()
    
    def _group_read(func, read_addr, read_length):
        def wrapper(self: MotorController):
            for i in range(len(self.motor_ids)):
                scs_addparam_result = self.groupSyncRead.addParam(self.motor_ids[i])
                if scs_addparam_result != True:
                    raise GroupAddParamFailedException()
            scs_comm_result = self.groupSyncRead.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                raise GroupSyncReadFailedException()
            agg = []
            init = False
            for i in range(len(self.motor_ids)):
                scs_data_result, scs_error = self.groupSyncRead.isAvailable(self.motor_ids[i], read_addr, read_length)
                if scs_data_result == True:
                    ret = func(self, self.motor_ids[i])
                    if not init:
                        agg = [[] for _ in range(len(ret))]
                        init = True
                    for j in range(len(ret)):
                        agg[j].append(ret[j])
                else:
                    raise GroupSyncReadNotAvailableException(self.motor_ids[i])
                    
            self.groupSyncRead.clearParam()
            return [np.array(a) for a in agg]
        return wrapper
    
    @_group_read(SMS_STS_PRESENT_POSITION_L, 4)
    def receive_raw_motor_states(self, motor_id):
        pos = self.groupSyncRead.getData(motor_id, SMS_STS_PRESENT_POSITION_L, 2),
        speed = self.groupSyncRead.getData(motor_id, SMS_STS_PRESENT_SPEED_L, 2)
        speed = self.packetHandler.scs_tohost(speed, 15)
        return pos, speed
    
    def receive_motor_states(self):
        raw_positions, raw_speeds = self.receive_raw_motor_states()
        positions = step2rad(raw_positions)
        speeds = step2rad(raw_speeds)
        return positions, speeds
    
    @_group_read(SMS_STS_PRESENT_CURRENT_L, 2)
    def receive_raw_motor_currents(self, motor_id):
        return self.groupSyncRead_Current.getData(motor_id, SMS_STS_PRESENT_CURRENT_L, 2)
    
    @_group_read(SMS_STS_PRESENT_LOAD_L, 2)
    def receive_raw_motor_loads(self, motor_id):
        return self.groupSyncRead_Load.getData(motor_id, SMS_STS_PRESENT_LOAD_L, 2)
    
    def set_mode(self, mode):
        for motor_id in self.motor_ids:
            scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(motor_id, SMS_STS_MODE, mode)
            if scs_comm_result != COMM_SUCCESS:
                raise WriteFailedException(scs_comm_result, scs_error)
    
    def set_duty(self, torques):
        torques = np.clip((torques).astype(np.int32), -1000, 1000)

        for i in range(len(self.motor_ids)):
            servo_id = self.motor_ids[i]
            torque = torques[servo_id - 1]

            scs_addparam_result = self.packetHandler.SyncWritePWMTorque(servo_id, torque)
            if scs_addparam_result != True:
                raise GroupAddParamFailedException()

        scs_comm_result = self.packetHandler.groupSyncWrite_PWMTorque.txPacket()
        if scs_comm_result != COMM_SUCCESS:
           raise SyncWriteFailedException()
        self.packetHandler.groupSyncWrite_PWMTorque.clearParam()
    
    def zero_motors(self):
        for i in range(len(self.motor_ids)):
            servo_id = self.motor_ids[i]

            scs_addparam_result = self.packetHandler.SyncTorqueOffCalPos(servo_id, np.uint8(128))
            if scs_addparam_result != True:
                raise GroupAddParamFailedException()

        scs_comm_result = self.packetHandler.groupSyncWrite_TorqueOffCalPos.txPacket()
        if scs_comm_result != COMM_SUCCESS:
           raise SyncWriteFailedException()

        self.packetHandler.groupSyncWrite_TorqueOffCalPos.clearParam()