import serial
import struct
import matplotlib.pyplot as plt


class SerialSpeedTester:
    def __init__(self):
        self.serial_port = None
        self.is_port_opened = False
        self.data_buffer = []  # Buffer to store received data
        self.filename = ""
        self.fig, self.ax = plt.subplots()  # Create figure and axis objects for plotting
        # Create a line object for the plot
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('Time')  # Set x-axis label
        self.ax.set_ylabel('Data')  # Set y-axis label
        self.ax.set_title('Real-time Data Plot')  # Set plot title

    def open_serial_port(self, port_name):
        self.serial_port = serial.Serial(
            port_name, baudrate=2000000, timeout=1)
        self.is_port_opened = True

    def read_serial_data(self):
        if self.is_port_opened:
            bytes_to_read = self.serial_port.in_waiting
            if bytes_to_read:
                self.data_buffer.extend(self.serial_port.read(bytes_to_read))
                return True
        return False

    def process_data(self):
        data = []
        while len(self.data_buffer) >= 3:
            # Convert 3 bytes to an integer
            data.append(struct.unpack('<I', bytes(
                self.data_buffer[:3]) + b'\x00')[0])
            del self.data_buffer[:3]
        return data

    def update_plot(self, data):
        # X-axis data (index of received data points)
        x_data = range(len(data))
        self.line.set_data(x_data, data)  # Update line data
        self.ax.relim()  # Recalculate limits
        self.ax.autoscale_view()  # Autoscale axes
        plt.pause(0.01)  # Pause to allow plot to update

    def run(self):
        port_name = input("Enter the serial port: ")
        self.open_serial_port(port_name)

        char_to_send = input("Enter the character to send ('b' or 'd'): ")
        self.serial_port.write(char_to_send.encode())

        plt.ion()  # Turn on interactive mode for real-time plotting
        plt.show()  # Show the plot window

        try:
            while True:
                if self.read_serial_data():
                    data = self.process_data()
                    if data:
                        self.update_plot(data)
        except KeyboardInterrupt:
            pass
        finally:
            if self.is_port_opened:
                self.serial_port.close()


if __name__ == "__main__":
    tester = SerialSpeedTester()
    tester.run()
