import serial
import csv
import struct
from ctypes import c_float
def char_float_to_float_char(char_string):
    # Pack the float into its raw binary representation and converts that representation into a utf-8 charecter string for transmission
    c_float_variable =c_float(float(char_string))
    packed_data = struct.pack('!f', c_float_variable.value)
    return packed_data


serial_port = 'COM6'  # Change this to your specific port (e.g., 'COM1' on Windows)
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)
if ser.is_open:
    print(f"Serial port {serial_port} opened successfully.")
else:
    print(f"Failed to open serial port {serial_port}. Exiting.")
    exit()
try:
    while True:
        ser.write(char_float_to_float_char("4.523") + "S".encode())
except KeyboardInterrupt:
    # Handle keyboard interrupt (Ctrl+C)
    print("Program terminated b0y user.")

finally:
    # Close the serial port
    ser.close()
    print(f"Serial port {serial_port} closed.")
