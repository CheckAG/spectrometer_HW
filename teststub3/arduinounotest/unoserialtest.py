import serial
import struct


class SerialSpeedTester:
    def __init__(self):
        self.serial_port = None
        self.is_port_opened = False
        self.incoming_data = bytearray()
        self.file_buffer = []
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
            bytes_to_read = self.serial_port.in_waiting
            if bytes_to_read:
                self.incoming_data += self.serial_port.read(bytes_to_read)
                return True
        return False

    def process_data(self):
        data = []
        while len(self.incoming_data) >= 2:  # Adjusted for 16-bit numbers
            # Unpack 16-bit numbers
            data.append(struct.unpack('<H', self.incoming_data[:2])[0])
            # Remove processed bytes from the buffer
            del self.incoming_data[:2]
        return data

    def print_to_file(self):
        data = self.process_data()
        if data:
            with open(self.filename, 'a') as file:
                for number in data:
                    file.write(str(number) + '\n')

    def close_serial_port(self):
        if self.is_port_opened:
            self.serial_port.close()
            self.is_port_opened = False

    def run(self):
        available_ports = self.get_available_ports()
        if not available_ports:
            print("No serial ports available.")
            return

        port_name = input("Enter the serial port: ")
        self.open_serial_port(port_name)

        command_character = input("Enter the character to send ('b' or 'd'): ")
        self.send_command(command_character)

        self.filename = input("Enter filename: ")

        try:
            while True:
                if self.read_serial_data():
                    self.print_to_file()
        except KeyboardInterrupt:
            pass
        finally:
            self.close_serial_port()


if __name__ == "__main__":
    tester = SerialSpeedTester()
    tester.run()
