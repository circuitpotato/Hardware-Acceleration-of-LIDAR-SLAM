from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import json
import pandas as pd
df = pd.read_csv("A.csv")

class MyRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Parse the query parameters
        query_params = parse_qs(self.path[2:])
        param_value = query_params.get('param', [None])[0]

        # Convert the parameter value to a float if provided
        # Read and send lines from the CSV file based on the parameter value
        if (int(param_value) != 9000):
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Transfer-Encoding', 'chunked')
            self.end_headers()

            response_data = self.read_csv_lines(param_value)
            response_json = json.dumps(response_data)
            #print(response_data)
            chunk_size = len(response_json) 


            # Send the JSON response
            self.wfile.write((hex(chunk_size)[2:] + '\r\n').encode('utf-8'))
            self.wfile.write(bytes(response_json, 'utf-8'))
            self.wfile.write(b'\r\n')
            self.wfile.write(b'0\r\n\r\n')
            print("Finished Response\n")
        else:
            with open('output_map.png', 'rb') as file:
                image_data = file.read()

            # Send response headers
                self.send_response(200)
                self.send_header('Content-type', 'image/png')
                self.end_headers()

            # Send the image data in the response body
                self.wfile.write(image_data)




    def read_csv_lines(self, param_value):
        # Replace 'your_file.csv' with the actual path to your CSV file
        row = df.iloc[int(param_value)].values
        float_row = row.tolist()

        return float_row

def run_server(port=8080):
    server_address = ('', port)
    httpd = HTTPServer(server_address, MyRequestHandler)
    print(f"Server is running on http://localhost:{port}")
    httpd.serve_forever()

if __name__ == '__main__':
    run_server()
