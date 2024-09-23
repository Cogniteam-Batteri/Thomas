import serial
import struct
from thomas_driver.kinco_servo.kinco_enum import *

class KincoServo:
    def __init__(self, port, baudrate=38400, node_id=1):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.node_id = node_id

    def _calculate_checksum(self, data):
        return (-sum(data)) & 0xFF

    def _send_receive(self, data):
        full_data = bytes([self.node_id]) + data
        checksum = self._calculate_checksum(full_data)
        self.ser.write(full_data + bytes([checksum]))
        # temp:bytearray = full_data + bytes([checksum])
        # print(temp.hex(":"))
        response = self.ser.read(10)
        if len(response) != 10:
            raise Exception(f"Invalid response length: {len(response)}")
        if self._calculate_checksum(response[:-1]) != response[-1]:
            raise Exception("Checksum mismatch")
        return response[1:-1]
        
    def read_parameter(self, index, subindex):
        command = struct.pack('<BHBL', 0x40, index, subindex, 0)
        try:
            response = self._send_receive(command)
            cmd = response[0]
            if cmd == 0x4F:
                return response[4]
            elif cmd == 0x4B:
                return struct.unpack('<H', response[4:6])[0]
            elif cmd == 0x43:
                return struct.unpack('<L', response[4:8])[0]
            elif cmd == 0x80:
                error_code = struct.unpack('<L', response[4:8])[0]
                # raise Exception(f"Read failed. Error code: 0x{error_code:08X}")
                return None
            else:
                # raise Exception(f"Unexpected response command: {cmd}")
                return None
        except Exception as e:
            # print(e)
            return None
    

    def write_parameter(self, index, subindex, value) -> bool:
        if isinstance(value, int):
            # Check if the `index` matches the ControlWord index
            if index == ControlWord.NAME.value:  # Controlword
                # Always use 2-byte format for Controlword, indicated by '0x2B' as the command specifier
                command = struct.pack('<BHBH2x', 0x2B, index, subindex, value)
            # Check if the value fits in 1 byte (0 to 0xFF)
            elif 0 <= value <= 0xFF:
                # Use 1-byte format for the value, indicated by '0x2F' as the command specifier
                command = struct.pack('<BHBB3x', 0x2F, index, subindex, value)
            # Check if the value fits in 2 bytes (-0x8000 to 0x7FFF)
            elif -0x8000 <= value <= 0x7FFF:
                # Use 2-byte format for the value, indicated by '0x2B' as the command specifier
                command = struct.pack('<BHBh2x', 0x2B, index, subindex, value)  # '<BHBh' for signed short
            # Check if the value fits in 4 bytes (-0x80000000 to 0x7FFFFFFF)
            elif -0x80000000 <= value <= 0x7FFFFFFF:
                # Use 4-byte signed integer format for the value, indicated by '0x23' as the command specifier
                command = struct.pack('<BHBi', 0x23, index, subindex, value)  # '<BHBi' for signed int
            else:
                # Print an error message if the value is out of range and return False
                # print("Value out of range")
                return False
        else:
            # print('not an integer !!')
            return
        try:
            response = self._send_receive(command)
            if response[0] == 0x80:
                error_code = struct.unpack('<L', response[4:8])[0]
                # print(f"Write failed. Error code: 0x{error_code:08X}")
                return False
            elif response[0] != 0x60:
                # print(f"Unexpected response: 0x{response[0]:02X}")
                return False

            return True
        except Exception as e:
            # print(e)
            return False

