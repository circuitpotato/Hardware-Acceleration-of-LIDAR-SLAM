import serial
import csv
import struct
from ctypes import c_float


#Cast float into int
#Goals rewrite the program so it can take specific inputs regarding the row and collum of the CSV and output it directly into the serial  for the webpage

def char_float_to_float_char(char_string):
    # Pack the float into its raw binary representation and converts that representation into a utf-8 charecter string for transmission
    c_float_variable =c_float(float(char_string))
    packed_data = struct.pack('!f', c_float_variable.value)
    return packed_data

#csv program for reading a specific line
def return_specific_line(csv_file_path, line_number):
    with open(csv_file_path, 'r', newline='') as file:
        reader = csv.reader(file)
        # Skip lines until the desired line number
        for _ in range(line_number - 1):
            next(reader, None)
        # Read and print the desired line
        specific_line = next(reader, None)
        if specific_line is not None:
            return specific_line
        else:
            print(f"Line {line_number} not found in the CSV file.")
            return None

# Define the serial port and baud rate
serial_port = 'COM6'  # Change this to your specific port (e.g., 'COM1' on Windows)
baud_rate = 115200

# Create a serial object
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Check if the port is open
if ser.is_open:
    print(f"Serial port {serial_port} opened successfully.")
else:
    print(f"Failed to open serial port {serial_port}. Exiting.")
    exit()
try:
    while True:
        3#get input for which line to transmit
        line_input = input("what line number do you read?")
        line_sent = return_specific_line('dataset.csv',int(line_input))
        ser.write("!".encode())
        ser.write(str(line_sent).encode())
        ser.write("_".encode())
        ser.write("@".encode())

        # Read data from the serial port
        data = ser.readline().decode('utf-8').strip()
        # Print the received data
        #prints(f"Received data: {data}")
        print("Transmitted data")
except KeyboardInterrupt:
    # Handle keyboard interrupt (Ctrl+C)
    print("Program terminated b0y user.")


finally:
    # Close the serial port
    ser.close()
    print(f"Serial port {serial_port} closed.")
