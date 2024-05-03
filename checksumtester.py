import serial

# Serial port configuration
ser = serial.Serial('COM3', 9600, timeout=1)  # Adjust COM port as needed


def calculate_checksum(data):
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def receive_data():
    # Wait until there's data available
    while ser.in_waiting < 12:  # Adjust according to the size of data sent from Arduino + checksum bytes
        pass

    # Read data from serial port
    # Adjust according to the size of data sent from Arduino
    data = ser.read(10)

    # Read checksum bytes
    received_checksum_bytes = ser.read(2)
    received_checksum = int.from_bytes(received_checksum_bytes, 'little')

    # Calculate checksum for received data
    calculated_checksum = calculate_checksum(data)

    # Debugging: Print received and calculated checksums
    print("Received checksum:", received_checksum)
    print("Calculated checksum:", calculated_checksum)

    # Verify checksum
    if received_checksum == calculated_checksum:
        print("Data integrity verified!")
        for byte in data:
            binary_representation = bin(byte)[2:].zfill(8)
            print("Received data:", binary_representation)
    else:
        print("CRC verification failed!")


def main():
    while True:
        receive_data()


if __name__ == "__main__":
    main()
