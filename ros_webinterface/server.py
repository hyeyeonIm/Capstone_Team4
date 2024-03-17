import http.server
import socketserver
import subprocess
from urllib.parse import urlparse, unquote

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path)
        path = parsed_path.path

        if path == "/start_hector":
            self.start_hector()
        elif path.startswith("/stop_and_save/"):
            map_name = path[len("/stop_and_save/"):]
            self.stop_and_save(map_name)
        else:
            super().do_GET()

    def start_hector(self):
        try:
            subprocess.Popen(['roslaunch', 'hector_slam_launch', 'tutorial.launch'])
            response = "Hector SLAM launch started successfully."
        except FileNotFoundError as e:
            response = f"Error starting Hector SLAM launch: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))

    def stop_and_save(self, map_name):
        map_name = unquote(map_name)  # Decode URL-encoded map name
        try:
            subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', f'/home/haley/catkin_ws/saveMap/{map_name}'])
            response = f"Map saved as {map_name}."
        except FileNotFoundError as e:
            response = f"Error saving map: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))



with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving at port: {PORT}")
    httpd.serve_forever()
