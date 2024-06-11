import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def read_data_from_serial(serial_port):
    # Read data from the serial port
    # Reading 6000 16-bit integers (12000 bytes)
    raw_data = serial_port.read(12000)

    if len(raw_data) == 12000:
        # Convert the byte data to integers
        data = []
        for i in range(0, len(raw_data), 2):
            value = int.from_bytes(
                raw_data[i:i+2], byteorder='little', signed=False)
            data.append(value)
        return data
    else:
        print("Incomplete data received.")
        return None


def update_plot(frame, serial_port, line):
    data = read_data_from_serial(serial_port)
    if data:
        line.set_ydata(data)
    return line,


def main():
    # Open the serial port
    ser = serial.Serial('COM48', 115200, timeout=1)

    fig, ax = plt.subplots()
    x = np.arange(6000)  # 6000 data points
    y = np.zeros(6000)  # Initial data
    line, = ax.plot(x, y)

    ax.set_ylim(0, 4096)  # Assuming 12-bit ADC resolution
    ax.set_xlim(0, 6000)

    def animate(frame):
        return update_plot(frame, ser, line)

    ani = animation.FuncAnimation(
        fig, animate, interval=12, blit=True)  # Adjusted interval

    plt.show()

    ser.close()


if __name__ == '__main__':
    main()
