import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from pathlib import Path

class ControllerWebServer:
    
    
    
    class Handler(BaseHTTPRequestHandler):
        def __init__(self, request, client_address, server):
            # Store shared state references provided by ControllerWebServer
            self.lock = server.lock
            self.key_state = server.key_state
            print(Path(__file__).parent / 'controller_client.html')
            with open(Path(__file__).parent / 'controller_client.html', 'r', encoding='utf-8') as f:
                self.page = f.read().encode('utf-8')
            # Base class __init__ will start handling the request immediately.
            super().__init__(request, client_address, server)
            
        def _send(self, code, body=b"ok", ctype="text/plain; charset=utf-8"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if urlparse(self.path).path in ("/", "/index.html"):
                return self._send(200, self.page, "text/html; charset=utf-8")
            return self._send(404, b"not found")

        def do_POST(self):
            path = urlparse(self.path).path

            if path == "/state":
                n = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(n)
                try:
                    msg = json.loads(raw.decode("utf-8"))
                    code = msg["code"]
                    down = bool(msg["down"])
                    with self.lock:
                        self.key_state[code] = down
                    return self._send(200)
                except Exception:
                    return self._send(400, b"bad request")

            if path == "/release_all":
                with self.lock:
                    self.key_state.clear()
                return self._send(200)

            return self._send(404, b"not found")
        
    def __init__(self):
        self.lock = threading.Lock()
        self.key_state = {}
    
    def start_server(self, host="127.0.0.1", port=3000):
        httpd = ThreadingHTTPServer((host, port), ControllerWebServer.Handler)
        # Expose shared state to the handler via the server instance
        httpd.lock = self.lock
        httpd.key_state = self.key_state
        t = threading.Thread(target=httpd.serve_forever, daemon=True)
        t.start()
    
    def get_key_state_snapshot(self):
        with self.lock:
            return dict(self.key_state)
    
    
if __name__ == "__main__":
    server = ControllerWebServer()
    server.start_server()
    while True:
        time.sleep(5)
        print("Current key state:", server.get_key_state_snapshot())
