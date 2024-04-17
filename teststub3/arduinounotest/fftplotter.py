import serial
import struct
import matplotlib.pyplot as plt
import numpy as np


class SerialSpeedTester:
    def __init__(self):
        self.serial_port = None
        self.is_port_opened = False
        self.data_buffer = []  # Buffer to store received data
        self.filename = ""
        # Create figure and axis objects for plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        # Create a line object for the plot
        self.line1, = self.ax1.plot([], [], lw=2)
        self.ax1.set_xlabel('Time')  # Set x-axis label
        self.ax1.set_ylabel('Data')  # Set y-axis label
        self.ax1.set_title('Real-time Data Plot')  # Set plot title

        self.line2, = self.ax2.plot([], [], lw=2)
        self.ax2.set_xlabel('Frequency [Hz]')
        self.ax2.set_ylabel('Amplitude')
        self.ax2.set_title('FFT Plot')

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
        while len(self.data_buffer) >= 2:  # Change from 3 to 2 bytes
            # Convert 2 bytes to an integer (16-bit number)
            data.append(struct.unpack('<H', bytes(
                self.data_buffer[:2]))[0])  # Change from '<I' to '<H'
            del self.data_buffer[:2]  # Change from 3 to 2
        return data

    def update_plot(self, data):
        # X-axis data (index of received data points)
        x_data = range(len(data))
        self.line1.set_data(x_data, data)  # Update line data
        self.ax1.relim()  # Recalculate limits
        self.ax1.autoscale_view()  # Autoscale axes

        # Perform FFT
        N = len(data)
        freq = np.fft.fftfreq(N, d=1)  # Frequency bins
        fft_data = np.fft.fft(data)  # Perform FFT
        fft_data_abs = np.abs(fft_data) / N  # Normalize FFT data

        # Plot FFT
        # Plot only positive frequencies
        self.line2.set_data(freq[:N // 2], fft_data_abs[:N // 2])
        self.ax2.relim()  # Recalculate limits
        self.ax2.autoscale_view()  # Autoscale axes

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
