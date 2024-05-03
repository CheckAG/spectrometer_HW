import serial
import struct
import time


class SerialSpeedTester:
    def __init__(self):
        self.serial_port = None
        self.is_port_opened = False
        self.serial_buffer = bytearray()
        self.dt1 = None

    def get_available_ports(self):
        import serial.tools.list_ports
        return [port.device for port in serial.tools.list_ports.comports()]

    def open_serial_port(self, port_name):
        self.serial_port = serial.Serial(
            port_name, baudrate=2000000, timeout=1)
        self.is_port_opened = True

    def send_command(self, command):
        if self.is_port_opened:
            self.serial_port.write(command.encode())

    def read_serial_data(self):
        while self.serial_port.in_waiting > 0:
            self.serial_buffer += self.serial_port.read(
                self.serial_port.in_waiting)
        return len(self.serial_buffer) > 0

    def print_converted_numbers(self):
        buffer_array_numbers = len(self.serial_buffer) // 3
        for i in range(buffer_array_numbers):
            converted_number = int.from_bytes(
                self.serial_buffer[i * 3: (i + 1) * 3], byteorder='little', signed=False)
            print("Converted number:", converted_number)
        self.serial_buffer.clear()

    def close_serial_port(self):
        if self.is_port_opened:
            self.serial_port.close()
            self.is_port_opened = False

    def run(self):
        available_ports = self.get_available_ports()
        if not available_ports:
            print("No serial ports available.")
            return

        port_name = input("Enter the serial port : ")
        self.open_serial_port(port_name)

        command_character = input("Enter the character to send ('b' or 'd'): ")
        self.send_command(command_character)

        self.dt1 = time.time()  # Record the start time

        while True:
            if self.read_serial_data():
                self.print_converted_numbers()

            # Check if 5 seconds have elapsed
            if time.time() - self.dt1 >= 5:
                break

        # Ensure all remaining data is read before closing
        while self.read_serial_data():
            self.print_converted_numbers()

        self.close_serial_port()


if __name__ == "__main__":
    tester = SerialSpeedTester()
    tester.run()
