import serial

# Open serial port
ser = serial.Serial('COM3', 9600)

while True:
    # Read bytes from serial port
    data = ser.read(12)  # Assuming 3 bytes are sent in each iteration

    # Display binary representation
    for byte in data:
        binary_representation = bin(byte)[2:].zfill(8)
        print(binary_representation)

# Close serial port
ser.close()
