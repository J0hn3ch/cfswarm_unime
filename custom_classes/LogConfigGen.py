import errno
import logging
import struct

from cflib.crazyflie.log import LogConfig, LogTocElement

# Custom LogConfig class
class LogConfigGen(LogConfig):
    def __init__(self, name, period_in_ms):
        """Initialize the entry"""
        super().__init__(name, period_in_ms)

    def add_callback(self, func):
        self.data_received_cb.add_callback(cb=func)

    def remove_callback(self, func):
        self.data_received_cb.remove_callback(cb=func)
    
    def unpack_log_data(self, log_data, timestamp):
        # Modify the behavior of the unpack_log_data() method for the LogConfig class
        """Unpack received logging data so it represent real values according
        to the configuration in the entry"""
        ret_data = {}
        data_index = 0
        for var in self.variables:
            size = LogTocElement.get_size_from_id(var.fetch_as)
            name = var.name
            unpackstring = LogTocElement.get_unpack_string_from_id(
                var.fetch_as)
            value = struct.unpack(
                unpackstring, log_data[data_index:data_index + size])[0]
            data_index += size
            ret_data[name] = value
        self.data_received_cb.call(timestamp, ret_data, self)