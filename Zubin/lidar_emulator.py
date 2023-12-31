import serial
import csv
import struct
import pandas as pd
from ctypes import c_float


#Cast float into int
#Goals rewrite the program so it can take specific inputs regarding the row and collum of the CSV and output it directly into the serial  for
df = pd.read_csv('dataset.csv')
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
def return_specific_values(row_number,col_number,quant):
    row = df.loc[row_number]
    send_list = []
    for i in range(0,quant):
        send_list.append(row[i+col_number])
    return send_list


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
        #get input for which line to transmit
        data = ser.readline().decode('utf-8').strip()
        print(data)
        row_number = 0
        row_quant = 0
        if len(data) > 0 and "+" in data: 
            line_input = int(data.split("+",2)[1][:4]) #used to get rid of padding and various serial console artifacts
            row_number = int(data.split("*",2)[1][:4])
            row_quant = int(data.split("|",2)[1][:4])
            print( "The Line_input is {}, The Row Number is {}, and the quant is {}".format(line_input,row_number,row_quant))
            send_list_serial = str(return_specific_values(line_input,row_number,row_quant))
            ser.write("!".encode())
            ser.write(send_list_serial.encode())
            ser.write("_".encode())

        # Read data "from the serial port
        # Print the received data
        #prints(f"Received data: {data}")
except KeyboardInterrupt:
    # Handle keyboard interrupt (Ctrl+C)
    print("Program terminated b0y user.")


finally:
    # Close the serial port
    ser.close()
    print(f"Serial port {serial_port} closed.")
