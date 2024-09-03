import serial
import struct
import time

class ServoDriver:
    def __init__(self, port, baudrate=38400, node_id=1):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.node_id = node_id

    def _calculate_checksum(self, data):
        return (-sum(data)) & 0xFF

    def _send_receive(self, data):
        full_data = bytes([self.node_id]) + data
        checksum = self._calculate_checksum(full_data)
        self.ser.write(full_data + bytes([checksum]))
        # print(f'{(full_data + bytes([checksum])).hex(":")}\n')
        response = self.ser.read(10)
        if len(response) != 10:
            raise Exception(f"Invalid response length: {len(response)}")
        if self._calculate_checksum(response[:-1]) != response[-1]:
            raise Exception("Checksum mismatch")
        return response[1:-1]
        
    def read_parameter(self, index, subindex):
        # Swap bytes for little-endian
        index_le = ((index & 0xFF) << 8) | ((index >> 8) & 0xFF)
        # index_le = index
        command = struct.pack('<BHBL', 0x40, index_le, subindex, 0)
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
            raise Exception(f"Read failed. Error code: 0x{error_code:08X}")
        else:
            raise Exception(f"Unexpected response command: {cmd}")

    def write_parameter(self, index, subindex, value):
        # Swap bytes for little-endian
        index_le = ((index & 0xFF) << 8) | ((index >> 8) & 0xFF)
        # index_le = index
        if isinstance(value, int):
            if index_le == 0x6040:  # Controlword
            # Always use 2-byte format for Controlword
                command = struct.pack('<BHBH2x', 0x2B, index_le, subindex, value)
            elif 0 <= value <= 0xFF:
                command = struct.pack('<BHBB3x', 0x2F, index_le, subindex, value)
            elif -0x8000 <= value <= 0x7FFF:
                command = struct.pack('<BHBH2x', 0x2B, index_le, subindex, value)
            elif -0x80000000 <= value <= 0x7FFFFFFF:
                command = struct.pack('<BHBL', 0x23, index_le, subindex, value)
            else:
                raise ValueError("Value out of range")
        else:
            raise TypeError("Value must be an integer")
        
        response = self._send_receive(command)
        if response[0] == 0x80:
            error_code = struct.unpack('<L', response[4:8])[0]
            raise Exception(f"Write failed. Error code: 0x{error_code:08X}")
        elif response[0] != 0x60:
            raise Exception(f"Unexpected response: 0x{response[0]:02X}")

    def read_status_word(self):
        return self.read_parameter(0x4160, 0)  # Note the byte swap here

    def read_control_word(self):
        return self.read_parameter(0x4060, 0)  # Note the byte swap here

    def read_operation_mode(self):
        return self.read_parameter(0x6060, 0)  # Note the byte swap here

    def check_status(self,to_print:bool = True):
        status = self.read_status_word()
        if to_print:
            print(f"Status word: 0x{status:04X}")
            if status & 0x0080: print("Warning: Fault detected")
            if status & 0x0040: print("Switch on disabled")
            if status & 0x0020: print("Quick stop active")
            if status & 0x0010: print("Voltage enabled")
            if status & 0x0008: print("Fault")
            if status & 0x0004: print("Operation enabled")
            if status & 0x0002: print("Switched on")
            if status & 0x0001: print("Ready to switch on")
        return status

    def enable_driver(self):
        print("Enabling driver...")
        self.write_parameter(0x4060, 0, 0x2F)  # Enable operation
        time.sleep(0.1)
        status = self.check_status(False)
        if status & 0x0004:
            print("Driver enabled successfully")
        else:
            print(f"Failed to enable driver\n {status.hex(':')}\n")

    def set_operation_mode(self, mode):
        print(f"Setting operation mode to {mode}...")
        self.write_parameter(0x6060, 0, mode)
        time.sleep(0.1)
        actual_mode = self.read_operation_mode()
        if actual_mode == mode:
            print(f"Operation mode set to {mode}")
        else:
            print(f"Failed to set operation mode. Current mode: {actual_mode}")

    def move_and_monitor(self, target_position, timeout=10):
        print(f"Moving to position {target_position}...")
        self.write_parameter(0x7A60, 0, target_position)
        self.write_parameter(0x8160, 0, 1000)
        self.write_parameter(0x8360, 0, 20)
        self.write_parameter(0x8460, 0, 20)
        self.write_parameter(0xFF60, 0, 200)
        
            # Trigger the movement
        control_word = self.read_control_word()
        control_word |= 0x10  # Set bit 4 to start the movement
        # self.write_parameter(0x4060, 0, 0x1f)
        self.write_parameter(0x4060, 0, control_word)
        print(f'{control_word}')
        
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            actual_position = self.read_parameter(0x6360, 0)
            status = self.read_status_word()
            print(f"Actual position: {actual_position}, Status: 0x{status:04X}")
            if status & 0x0400:  # Target reached
                print("Target reached!")
                return
            time.sleep(0.5)
        print("Timeout: target not reached")
        
def main():
    driver = ServoDriver("/dev/ttyUSB0")  # Adjust port as needed
    
    try:
        print("Initial status:")
        driver.check_status()
        
        print(f"\nInitial control word: 0x{driver.read_control_word():04X}")
        print(f"Initial operation mode: {driver.read_operation_mode()}")

        driver.enable_driver()
        
        driver.set_operation_mode(1)  # Set to Position mode


        driver.move_and_monitor(1000000)
        print("\nFinal status:")
        driver.check_status()
        
        print(f"\nFinal control word: 0x{driver.read_control_word():04X}")
        print(f"Final operation mode: {driver.read_operation_mode()}")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        print("Script completed")

if __name__ == "__main__":
    main()