import serial
import struct
from kinco_enum import *

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
        temp:bytearray = full_data + bytes([checksum])
        print(temp.hex(":"))
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
            print(e)
            return None
    

    def write_parameter(self, index, subindex, value) -> bool:
        if isinstance(value, int):
            if index == ControlWord.NAME.value:  # Controlword
            # Always use 2-byte format for Controlword
                command = struct.pack('<BHBH2x', 0x2B, index, subindex, value)
            elif 0 <= value <= 0xFF:
                command = struct.pack('<BHBB3x', 0x2F, index, subindex, value)
            elif -0x8000 <= value <= 0x7FFF:
                command = struct.pack('<BHBH2x', 0x2B, index, subindex, value)
            elif -0x80000000 <= value <= 0x7FFFFFFF:
                command = struct.pack('<BHBL', 0x23, index, subindex, value)
            else:
                print("Value out of range")
                return False
        else:
            print('not an integer !!')
            return
        try:

            #command = struct.pack('<BHBL', 0x23, index, subindex, value)
            response = self._send_receive(command)
            if response[0] == 0x80:
                error_code = struct.unpack('<L', response[4:8])[0]
                print(f"Write failed. Error code: 0x{error_code:08X}")
                return False
            elif response[0] != 0x60:
                print(f"Unexpected response: 0x{response[0]:02X}")
                return False
            
            return True
        except Exception as e:
            print(e)
            return False
