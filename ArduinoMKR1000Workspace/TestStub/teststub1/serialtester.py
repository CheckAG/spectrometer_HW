import serial
import struct


class SerialSpeedTester:
    def __init__(self):
        self.serial_port = None
        self.is_port_opened = False
        self.incoming_data = []
        self.file_buffer = []
        self.serial_buffer = bytearray()
        self.total_buffer_length = 0
        self.converted_number = 0
        self.filename = ""
        self.dt1 = None
        self.dt2 = None
        self.elapsed_seconds = 0

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
        if self.is_port_opened:
            # returns the number of bytes in the input buffer
            bytes_to_read = self.serial_port.in_waiting
        if bytes_to_read:  # checks if there are any available bytes to read
            self.serial_buffer += self.serial_port.read(bytes_to_read)
            self.incoming_data.append(self.serial_buffer)
            self.total_buffer_length += bytes_to_read
            return True
        return False

    def print_to_file(self):
        if self.incoming_data:
            buffer_array = b''.join(self.incoming_data)
            buffer_array_numbers = len(buffer_array) // 3

            for i in range(buffer_array_numbers):
                converted_number = int.from_bytes(
                    buffer_array[i*3: (i+1)*3], byteorder='little', signed=False)
                self.file_buffer.append(converted_number)

            with open(self.filename, 'a') as file:
                for number in self.file_buffer:
                    file.write(str(number) + '\n')

            self.incoming_data.clear()
            self.file_buffer.clear()

    def close_serial_port(self):
        if self.is_port_opened:
            self.serial_port.close()
            self.is_port_opened = False

    def run(self):
        available_ports = self.get_available_ports()
        if not available_ports:
            print("No serial ports available.")
            return

        port_name = input(
            "Enter the serial port : ")
        self.open_serial_port(port_name)

        command_character = input("Enter the character to send ('b' or 'd'): ")
        self.send_command(command_character)

        self.filename = input("Enter filename: ")

        while True:
            if self.read_serial_data():
                self.print_to_file()

        self.close_serial_port()


if __name__ == "__main__":
    tester = SerialSpeedTester()
    tester.run()
