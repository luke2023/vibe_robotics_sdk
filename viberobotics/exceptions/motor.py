class MotorException(Exception):
    """Base exception class for motor-related errors."""
    pass

class GroupAddParamFailedException(MotorException):
    """Exception raised when adding parameters to a group fails."""
    pass

class GroupSyncReadFailedException(MotorException):
    """Exception raised when a group sync read operation fails."""
    pass

class GroupSyncReadNotAvailableException(MotorException):
    """Exception raised when data is not available after a group sync read."""
    def __init__(self, motor_id):
        super().__init__(f"Data not available for motor ID: {motor_id}")
        self.motor_id = motor_id

class WriteFailedException(MotorException):
    """Exception raised when a write operation to a motor fails."""
    def __init__(self, result, error):
        super().__init__(f"Write failed with result: {result}, error: {error}")
        self.result = result
        self.error = error

class SyncWriteFailedException(MotorException):
    """Exception raised when a sync write operation fails."""
    pass
