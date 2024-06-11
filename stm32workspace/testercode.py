import serial
import crcmod.predefined

# Define the CRC32 function with the given polynomial (0x04C11DB7)
crc32_func = crcmod.predefined.mkPredefinedCrcFun('crc-32')


def read_crc_from_stm32(port, original_data, baudrate=115200, timeout=1):
    # Open the serial port
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        while True:
            # Read 4 bytes from the serial port
            crc_data = ser.read(5)

            if len(crc_data) == 5:
                # Convert bytes to a 32-bit unsigned integer
                crc_received = int.from_bytes(crc_data, byteorder='little')

                # Compute the CRC of the original data
                crc_computed = crc32_func(original_data)

                # Compare the received CRC with the computed CRC
                if crc_received == crc_computed:
                    print(
                        f"Received CRC: 0x{crc_received:08X} matches computed CRC: 0x{crc_computed:08X}")
                else:
                    print(
                        f"Received CRC: 0x{crc_received:08X} does not match computed CRC: 0x{crc_computed:08X}")
            else:
                print("Failed to read 4 bytes from the serial port")


if __name__ == "__main__":
    # Replace 'COM3' with your actual serial port name
    port_name = 'COM44'

    # Define the original data used in the STM32
    # The data should be a byte array representing the data used on the STM32
    original_data = bytearray([1, 2, 3, 4, 5])

    # Convert the example data to a byte array assuming 16-bit values (2 bytes per value)
    original_data_bytes = bytearray()
    for value in [1, 2, 3, 4, 5]:
        original_data_bytes.extend(value.to_bytes(4, byteorder='little'))

    read_crc_from_stm32(port_name, original_data_bytes)
