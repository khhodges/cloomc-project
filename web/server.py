#!/usr/bin/env python3
import http.server
import socketserver
import os
import json

PORT = 5000
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
DOCS_DIR = os.path.join(os.path.dirname(WEB_DIR), 'docs')

class CTMMHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def do_GET(self):
        if self.path == '/api/docs':
            self.send_docs_list()
        elif self.path.startswith('/api/docs/'):
            self.send_doc_file(self.path[10:])
        elif self.path.startswith('/docs/'):
            filename = self.path[6:].split('?')[0]
            filepath = os.path.join(DOCS_DIR, filename)
            if os.path.isfile(filepath):
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.end_headers()
                with open(filepath, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404)
        else:
            super().do_GET()

    def send_docs_list(self):
        files = []
        if os.path.isdir(DOCS_DIR):
            for f in sorted(os.listdir(DOCS_DIR)):
                if f.endswith('.md'):
                    path = os.path.join(DOCS_DIR, f)
                    files.append({
                        'name': f,
                        'size': os.path.getsize(path),
                        'title': f.replace('.md', '').replace('-', ' ').title()
                    })
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(files).encode())

    def send_doc_file(self, filename):
        if not filename.endswith('.md'):
            filename += '.md'
        filename = os.path.basename(filename)
        filepath = os.path.join(DOCS_DIR, filename)
        if os.path.isfile(filepath):
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.end_headers()
            with open(filepath, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

class ReuseAddrTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

with ReuseAddrTCPServer(("0.0.0.0", PORT), CTMMHandler) as httpd:
    print(f"CTMM Web Simulator running at http://0.0.0.0:{PORT}")
    httpd.serve_forever()
